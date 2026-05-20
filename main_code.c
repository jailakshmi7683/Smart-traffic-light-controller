/*
 * traffic_light.c
 * Smart Traffic Light Controller
 * QNX SDP 8 on Raspberry Pi 4 (aarch64le)
 *
 * Features:
 *   - Fixed RED->GREEN->YELLOW cycle with tick-based FSM
 *   - Emergency mode  (GREEN blinks, cycle position saved)
 *   - Distance mode   (HC-SR04 drives LEDs)
 *   - Peak-hour mode  (time-of-day adjusts GREEN/RED durations)
 *   - Built-in HTTP server on port 8080
 *       GET  /        -> web dashboard HTML
 *       GET  /state   -> JSON status
 *       POST /cmd     -> body: cmd=e|n|h|g|q|p
 *
 * GPIO Connections:
 *   GPIO 17 (Pin 11) --> Red    LED (330 ohm resistor)
 *   GPIO 27 (Pin 13) --> Yellow LED (330 ohm resistor)
 *   GPIO 22 (Pin 15) --> Green  LED (330 ohm resistor)
 *   GPIO 23 (Pin 16) --> HC-SR04 TRIG
 *   GPIO 24 (Pin 18) --> HC-SR04 ECHO
 *   Pin 17  (3.3V)   --> HC-SR04 VCC
 *   Pin 9   (GND)    --> All LED cathodes + HC-SR04 GND
 *
 * MUST run as root (mmap of physical GPIO registers):
 *   ssh root@10.0.0.1 "/tmp/traffic_light"
 *
 * Build:
 *   make PLATFORM=aarch64le BUILD_PROFILE=debug
 *
 * Deploy:
 *   scp build/aarch64le-debug/traffic_light root@10.0.0.1:/tmp/
 *
 * Open browser at: http://10.0.0.1:8080
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ═══════════════════════════════════════════════
   BCM2711 GPIO REGISTER MAP
   Physical base 0xfe200000 (Raspberry Pi 4 only)
   Reference: QNX book Section 5.6, gpio_map.c
   ═══════════════════════════════════════════════ */

#define GPIO_BASE      0xfe200000UL
#define GPIO_MAP_SIZE  0x1000

/* Word-index offsets into the mapped register block */
#define GPFSEL0   0    /* Function select: GPIO  0-9  */
#define GPSET0    7    /* Output set:      GPIO  0-31 */
#define GPCLR0    10   /* Output clear:    GPIO  0-31 */
#define GPLEV0    13   /* Pin level:       GPIO  0-31 */

#define GPIO_FUNC_OUT  1u

static volatile uint32_t *g_gpio = NULL;

/* ═══════════════════════════════════════════════
   GPIO PIN ASSIGNMENTS
   ═══════════════════════════════════════════════ */

#define GPIO_RED     17
#define GPIO_YELLOW  27
#define GPIO_GREEN   22
#define GPIO_TRIG    23
#define GPIO_ECHO    24

/* ═══════════════════════════════════════════════
   BCM2711 REGISTER ACCESS
   Identical pattern to book Section 5.6 gpio_map.c
   ═══════════════════════════════════════════════ */

static void gpio_set_output(int pin)
{
    int reg   = GPFSEL0 + pin / 10;
    int shift = (pin % 10) * 3;
    g_gpio[reg] &= ~(7u << shift);
    g_gpio[reg] |=  (GPIO_FUNC_OUT << shift);
}

static void gpio_set_input(int pin)
{
    int reg   = GPFSEL0 + pin / 10;
    int shift = (pin % 10) * 3;
    g_gpio[reg] &= ~(7u << shift);   /* 000 = input */
}

static void gpio_write(int pin, int val)
{
    if (val) g_gpio[GPSET0] = 1u << pin;
    else     g_gpio[GPCLR0] = 1u << pin;
}

static int gpio_read(int pin)
{
    return (g_gpio[GPLEV0] >> pin) & 1u;
}

static int gpio_init(void)
{
    g_gpio = mmap(0, GPIO_MAP_SIZE,
                  PROT_READ | PROT_WRITE | PROT_NOCACHE,
                  MAP_SHARED | MAP_PHYS,
                  -1, GPIO_BASE);
    if (g_gpio == MAP_FAILED) {
        perror("[GPIO] mmap");
        return -1;
    }
    printf("[GPIO] BCM2711 registers mapped at 0x%lx\n",
           (unsigned long)GPIO_BASE);
    return 0;
}

static void gpio_cleanup(void)
{
    if (g_gpio && g_gpio != MAP_FAILED)
        munmap((void *)g_gpio, GPIO_MAP_SIZE);
    g_gpio = NULL;
}

/* ═══════════════════════════════════════════════
   STATES AND MODES
   ═══════════════════════════════════════════════ */

typedef enum { PHASE_RED = 0, PHASE_GREEN = 1, PHASE_YELLOW = 2 } Phase;
typedef enum {
    MODE_CYCLE    = 0,   /* normal timed cycle            */
    MODE_EMERG    = 1,   /* emergency — GREEN blinks      */
    MODE_DISTANCE = 2,   /* sensor drives LEDs            */
    MODE_PEAK     = 3    /* peak-hour adjusted cycle      */
} AppMode;

static const char *phase_str(Phase p)
{
    switch (p) {
        case PHASE_RED:    return "RED";
        case PHASE_GREEN:  return "GREEN";
        case PHASE_YELLOW: return "YELLOW";
        default:           return "UNKNOWN";
    }
}

static const char *mode_str(AppMode m)
{
    switch (m) {
        case MODE_CYCLE:    return "FIXED_CYCLE";
        case MODE_EMERG:    return "EMERGENCY";
        case MODE_DISTANCE: return "DISTANCE";
        case MODE_PEAK:     return "PEAK_HOUR";
        default:            return "UNKNOWN";
    }
}

/* ═══════════════════════════════════════════════
   TIMING CONSTANTS (milliseconds)
   Peak-hour values are set dynamically.
   ═══════════════════════════════════════════════ */

#define TIME_RED_MS       5000u
#define TIME_GREEN_MS     5000u
#define TIME_YELLOW_MS    3000u

/* Peak-hour: shorter RED, longer GREEN */
#define PEAK_RED_MS       3000u
#define PEAK_GREEN_MS     8000u
#define PEAK_YELLOW_MS    3000u

/* Off-peak: longer RED, normal GREEN */
#define OFFPEAK_RED_MS    7000u
#define OFFPEAK_GREEN_MS  5000u
#define OFFPEAK_YELLOW_MS 3000u

/* Peak-hour window: 08:00-10:00 and 17:00-19:00 */
#define PEAK_AM_START  8
#define PEAK_AM_END   10
#define PEAK_PM_START 17
#define PEAK_PM_END   19

/* HC-SR04 vehicle threshold */
#define VEHICLE_CM   50.0

/* FSM tick */
#define TICK_MS  100u

/* HTTP port */
#define HTTP_PORT  8080

/* ═══════════════════════════════════════════════
   SHARED STATE  (all fields protected by g_mutex)
   ═══════════════════════════════════════════════ */

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

static AppMode      g_mode          = MODE_CYCLE;
static Phase        g_phase         = PHASE_RED;
static unsigned int g_elapsed_ms    = 0;   /* ms into current phase      */
static double       g_distance      = 999.0;
static int          g_peak_hour     = 0;   /* 1 = currently peak hour    */
static unsigned int g_red_ms        = TIME_RED_MS;
static unsigned int g_green_ms      = TIME_GREEN_MS;
static unsigned int g_yellow_ms     = TIME_YELLOW_MS;

/* ═══════════════════════════════════════════════
   SLEEP HELPERS
   ═══════════════════════════════════════════════ */

static void sleep_ms(unsigned int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static void sleep_us(unsigned int us)
{
    struct timespec ts = { 0, (long)us * 1000L };
    nanosleep(&ts, NULL);
}

/* ═══════════════════════════════════════════════
   LED HELPERS
   ═══════════════════════════════════════════════ */

static void all_leds_off(void)
{
    gpio_write(GPIO_RED,    0);
    gpio_write(GPIO_YELLOW, 0);
    gpio_write(GPIO_GREEN,  0);
}

static void led_for_phase(Phase p)
{
    all_leds_off();
    switch (p) {
        case PHASE_RED:    gpio_write(GPIO_RED,    1); break;
        case PHASE_GREEN:  gpio_write(GPIO_GREEN,  1); break;
        case PHASE_YELLOW: gpio_write(GPIO_YELLOW, 1); break;
    }
}

static Phase next_phase(Phase p)
{
    switch (p) {
        case PHASE_RED:    return PHASE_GREEN;
        case PHASE_GREEN:  return PHASE_YELLOW;
        case PHASE_YELLOW: return PHASE_RED;
        default:           return PHASE_RED;
    }
}

static unsigned int phase_duration_ms(Phase p)
{
    pthread_mutex_lock(&g_mutex);
    unsigned int d;
    switch (p) {
        case PHASE_RED:    d = g_red_ms;    break;
        case PHASE_GREEN:  d = g_green_ms;  break;
        case PHASE_YELLOW: d = g_yellow_ms; break;
        default:           d = g_red_ms;
    }
    pthread_mutex_unlock(&g_mutex);
    return d;
}

/* ═══════════════════════════════════════════════
   THREAD 1: TRAFFIC LIGHT FSM
   Priority 20

   Tick-based (TICK_MS = 100 ms).
   Handles MODE_CYCLE, MODE_EMERG, MODE_DISTANCE,
   MODE_PEAK.  All mode transitions are clean:
   position is saved on emergency entry and restored
   on exit; distance/peak modes restart at RED.
   ═══════════════════════════════════════════════ */

static void *fsm_thread(void *arg)
{
    (void)arg;
    printf("[FSM] started (tick=%u ms)\n", TICK_MS);

    gpio_set_output(GPIO_RED);
    gpio_set_output(GPIO_YELLOW);
    gpio_set_output(GPIO_GREEN);
    all_leds_off();

    Phase        cur_phase  = PHASE_RED;
    unsigned int elapsed    = 0;
    AppMode      prev_mode  = MODE_CYCLE;
    int          blink_on   = 0;
    unsigned int blink_acc  = 0;

    /* saved position for emergency restore */
    Phase        saved_phase   = PHASE_RED;
    unsigned int saved_elapsed = 0;

    led_for_phase(cur_phase);
    printf("[FSM] --> RED (%u s)\n", TIME_RED_MS / 1000u);

    while (1) {
        sleep_ms(TICK_MS);

        pthread_mutex_lock(&g_mutex);
        AppMode mode = g_mode;
        double  dist = g_distance;
        pthread_mutex_unlock(&g_mutex);

        /* ── EMERGENCY ────────────────────────── */
        if (mode == MODE_EMERG) {
            if (prev_mode != MODE_EMERG) {
                /* first tick in emergency: save position */
                saved_phase   = cur_phase;
                saved_elapsed = elapsed;
            }

            blink_acc += TICK_MS;
            if (blink_acc >= 500u) {
                blink_acc = 0;
                blink_on ^= 1;
                all_leds_off();
                if (blink_on) gpio_write(GPIO_GREEN, 1);
                printf("[FSM] EMERGENCY blink %s\n", blink_on ? "ON" : "OFF");
            }
            prev_mode = MODE_EMERG;
            continue;
        }

        /* ── Return from EMERGENCY ───────────── */
        if (prev_mode == MODE_EMERG && mode != MODE_EMERG) {
            cur_phase = saved_phase;
            elapsed   = saved_elapsed;
            blink_on  = 0;
            blink_acc = 0;
            led_for_phase(cur_phase);
            unsigned int dur = phase_duration_ms(cur_phase);
            printf("[FSM] Emergency cleared — resuming %s (%u s left)\n",
                   phase_str(cur_phase),
                   (dur > elapsed ? dur - elapsed : 0u) / 1000u);
        }

        /* ── DISTANCE MODE ───────────────────── */
        if (mode == MODE_DISTANCE) {
            Phase want = (dist < VEHICLE_CM)       ? PHASE_GREEN  :
                         (dist < VEHICLE_CM * 2.0) ? PHASE_YELLOW :
                                                     PHASE_RED;
            static Phase last_dist_phase = (Phase)255;
            if (want != last_dist_phase) {
                led_for_phase(want);
                printf("[FSM] DISTANCE: %s (%.1f cm)\n",
                       phase_str(want), dist);
                last_dist_phase = want;
            }
            prev_mode = MODE_DISTANCE;
            continue;
        }

        /* ── Return from DISTANCE or PEAK restart ── */
        if (prev_mode == MODE_DISTANCE ||
            (prev_mode == MODE_PEAK && mode == MODE_CYCLE)) {
            pthread_mutex_lock(&g_mutex);
            cur_phase = g_phase;
            elapsed   = g_elapsed_ms;
            pthread_mutex_unlock(&g_mutex);
            led_for_phase(cur_phase);
            printf("[FSM] Restarting at %s\n", phase_str(cur_phase));
        }

        /* ── CYCLE and PEAK (same FSM, different timings) ── */
        elapsed += TICK_MS;

        pthread_mutex_lock(&g_mutex);
        g_phase      = cur_phase;
        g_elapsed_ms = elapsed;
        pthread_mutex_unlock(&g_mutex);

        if (elapsed >= phase_duration_ms(cur_phase)) {
            cur_phase = next_phase(cur_phase);
            elapsed   = 0;
            led_for_phase(cur_phase);
            printf("[FSM] --> %s (%u s)\n",
                   phase_str(cur_phase),
                   phase_duration_ms(cur_phase) / 1000u);
        }

        prev_mode = mode;
    }
    return NULL;
}

/* ═══════════════════════════════════════════════
   THREAD 2: HC-SR04 ULTRASONIC SENSOR
   Priority 15
   ═══════════════════════════════════════════════ */

static void *sensor_thread(void *arg)
{
    (void)arg;
    printf("[SENSOR] started\n");

    gpio_set_output(GPIO_TRIG);
    gpio_set_input(GPIO_ECHO);
    gpio_write(GPIO_TRIG, 0);
    sleep_ms(1000);

    while (1) {
        gpio_write(GPIO_TRIG, 0); sleep_us(2);
        gpio_write(GPIO_TRIG, 1); sleep_us(10);
        gpio_write(GPIO_TRIG, 0);

        struct timespec t0, t1, now;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        /* Wait for ECHO HIGH (30 ms timeout) */
        while (gpio_read(GPIO_ECHO) == 0) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            long us = (now.tv_sec - t0.tv_sec) * 1000000L
                    + (now.tv_nsec - t0.tv_nsec) / 1000L;
            if (us > 30000) { goto next; }
        }

        clock_gettime(CLOCK_MONOTONIC, &t0);
        while (gpio_read(GPIO_ECHO) == 1) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            long us = (now.tv_sec - t0.tv_sec) * 1000000L
                    + (now.tv_nsec - t0.tv_nsec) / 1000L;
            if (us > 30000) { goto next; }
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);

        {
            long pulse = (t1.tv_sec - t0.tv_sec) * 1000000L
                       + (t1.tv_nsec - t0.tv_nsec) / 1000L;
            double cm = (double)pulse / 58.0;
            pthread_mutex_lock(&g_mutex);
            g_distance = cm;
            pthread_mutex_unlock(&g_mutex);
            printf("[SENSOR] %.1f cm\n", cm);
        }
next:
        sleep_ms(250);
    }
    return NULL;
}

/* ═══════════════════════════════════════════════
   THREAD 3: PEAK-HOUR SCHEDULER
   Priority 11

   Checks wall clock every 30 seconds.
   Sets timing variables so the FSM automatically
   uses shorter RED / longer GREEN during rush hours
   without needing a separate mode switch.
   Peak hours: 08:00-10:00 and 17:00-19:00.
   ═══════════════════════════════════════════════ */

static void *peak_thread(void *arg)
{
    (void)arg;
    printf("[PEAK] Peak-hour scheduler started\n");
    printf("[PEAK] Peak windows: %02d:00-%02d:00 and %02d:00-%02d:00\n",
           PEAK_AM_START, PEAK_AM_END, PEAK_PM_START, PEAK_PM_END);

    while (1) {
        time_t     now  = time(NULL);
        struct tm *tm   = localtime(&now);
        int        hour = tm->tm_hour;

        int is_peak = (hour >= PEAK_AM_START && hour < PEAK_AM_END) ||
                      (hour >= PEAK_PM_START && hour < PEAK_PM_END);

        pthread_mutex_lock(&g_mutex);
        int was_peak = g_peak_hour;

        if (is_peak) {
            g_red_ms    = PEAK_RED_MS;
            g_green_ms  = PEAK_GREEN_MS;
            g_yellow_ms = PEAK_YELLOW_MS;
            g_peak_hour = 1;
        } else {
            g_red_ms    = OFFPEAK_RED_MS;
            g_green_ms  = OFFPEAK_GREEN_MS;
            g_yellow_ms = OFFPEAK_YELLOW_MS;
            g_peak_hour = 0;
        }
        pthread_mutex_unlock(&g_mutex);

        if (is_peak != was_peak) {
            printf("[PEAK] %s — RED=%us GREEN=%us YELLOW=%us\n",
                   is_peak ? "PEAK HOUR" : "OFF-PEAK",
                   (is_peak ? PEAK_RED_MS    : OFFPEAK_RED_MS)    / 1000u,
                   (is_peak ? PEAK_GREEN_MS  : OFFPEAK_GREEN_MS)  / 1000u,
                   (is_peak ? PEAK_YELLOW_MS : OFFPEAK_YELLOW_MS) / 1000u);
        }

        sleep_ms(30000);   /* check every 30 seconds */
    }
    return NULL;
}

/* ═══════════════════════════════════════════════
   THREAD 4: COMMAND LISTENER (stdin)
   Priority 25

   Same commands as before plus 'p' to force
   peak-hour timings manually for testing.
   ═══════════════════════════════════════════════ */

static void apply_command(char cmd)
{
    pthread_mutex_lock(&g_mutex);
    switch (cmd) {
        case 'e': case 'E':
            g_mode = MODE_EMERG;
            printf("[CMD] EMERGENCY activated\n");
            break;
        case 'n': case 'N':
            if (g_mode == MODE_EMERG) g_mode = MODE_CYCLE;
            printf("[CMD] Emergency cleared\n");
            break;
        case 'h': case 'H':
            g_mode = MODE_DISTANCE;
            printf("[CMD] Distance mode ON\n");
            break;
        case 'g': case 'G':
            g_mode      = MODE_CYCLE;
            g_phase     = PHASE_RED;
            g_elapsed_ms = 0;
            printf("[CMD] Fixed cycle mode — restarting at RED\n");
            break;
        case 'p': case 'P':
            /* toggle peak timings manually for demo */
            if (g_peak_hour) {
                g_peak_hour = 0;
                g_red_ms    = OFFPEAK_RED_MS;
                g_green_ms  = OFFPEAK_GREEN_MS;
                g_yellow_ms = OFFPEAK_YELLOW_MS;
                printf("[CMD] Forced OFF-PEAK timings\n");
            } else {
                g_peak_hour = 1;
                g_red_ms    = PEAK_RED_MS;
                g_green_ms  = PEAK_GREEN_MS;
                g_yellow_ms = PEAK_YELLOW_MS;
                printf("[CMD] Forced PEAK timings\n");
            }
            break;
        case 'q': case 'Q':
            pthread_mutex_unlock(&g_mutex);
            printf("[CMD] Quitting\n");
            all_leds_off();
            gpio_cleanup();
            exit(0);
        default:
            break;
    }
    pthread_mutex_unlock(&g_mutex);
}

static void *cmd_thread(void *arg)
{
    (void)arg;
    printf("[CMD] started\n");
    printf("  e=emergency  n=clear  h=distance  g=cycle  p=toggle-peak  q=quit\n\n");

    char buf[8];
    while (fgets(buf, sizeof(buf), stdin))
        apply_command(buf[0]);
    return NULL;
}

/* ═══════════════════════════════════════════════
   THREAD 5: HTTP SERVER
   Priority 12

   Serves a self-contained HTML dashboard on port
   8080.  The page polls /state every second to
   refresh, and posts to /cmd to send commands.

   /         -> full dashboard HTML page
   /state    -> JSON snapshot of shared state
   /cmd      -> POST body: cmd=<letter>
   ═══════════════════════════════════════════════ */

/* Build JSON state string (caller must free) */
static char *build_json(void)
{
    pthread_mutex_lock(&g_mutex);
    const char *mstr = mode_str(g_mode);
    const char *pstr = phase_str(g_phase);
    unsigned int dur = 0;
    switch (g_phase) {
        case PHASE_RED:    dur = g_red_ms;    break;
        case PHASE_GREEN:  dur = g_green_ms;  break;
        case PHASE_YELLOW: dur = g_yellow_ms; break;
    }
    unsigned int remaining = (g_elapsed_ms < dur) ? dur - g_elapsed_ms : 0u;
    double dist    = g_distance;
    int    peak    = g_peak_hour;
    unsigned int rm = g_red_ms, gm = g_green_ms, ym = g_yellow_ms;
    pthread_mutex_unlock(&g_mutex);

    char *buf = malloc(512);
    if (!buf) return NULL;
    snprintf(buf, 512,
        "{"
        "\"mode\":\"%s\","
        "\"phase\":\"%s\","
        "\"remaining_ms\":%u,"
        "\"distance_cm\":%.1f,"
        "\"peak_hour\":%s,"
        "\"timings\":{\"red_ms\":%u,\"green_ms\":%u,\"yellow_ms\":%u}"
        "}",
        mstr, pstr, remaining, dist,
        peak ? "true" : "false",
        rm, gm, ym);
    return buf;
}

/* The dashboard HTML — embedded as a C string.
   Uses fetch() to poll /state every second.
   Sends POST /cmd for each button press. */
static const char HTML_PAGE[] =
		"<!DOCTYPE html>"
		"<html lang='en'>"
		"<head>"
		"<meta charset='UTF-8'>"
		"<meta name='viewport' content='width=device-width,initial-scale=1'>"
		"<title>Traffic Light Controller</title>"
		"<style>"
		"*{box-sizing:border-box;margin:0;padding:0}"
		"body{font-family:system-ui,sans-serif;background:#f5f5f5;padding:20px}"
		"h1{font-size:1.05rem;font-weight:500;color:#111;margin-bottom:14px}"
		".card{background:#fff;border:1px solid #e2e2e2;border-radius:14px;padding:18px}"
		".card-title{font-size:.7rem;font-weight:600;color:#999;text-transform:uppercase;"
		"letter-spacing:.08em;margin-bottom:14px}"
		/* top state card */
		".state-card{margin-bottom:14px}"
		".state-inner{display:grid;grid-template-columns:auto 1fr;gap:20px;align-items:center}"
		".light-col{display:flex;gap:14px;align-items:center;padding:4px 0}"
		".bulb{width:60px;height:60px;border-radius:50%;border:3px solid #e0e0e0;transition:all .35s}"
		".bulb.red-on   {background:#ff3333;border-color:#ff3333;box-shadow:0 0 18px #ff333377}"
		".bulb.yellow-on{background:#ffcc00;border-color:#ffcc00;box-shadow:0 0 18px #ffcc0077}"
		".bulb.green-on {background:#22cc55;border-color:#22cc55;box-shadow:0 0 18px #22cc5577}"
		".bulb.red-off  {background:#ffe0e0;border-color:#f5c0c0}"
		".bulb.yellow-off{background:#fff9e0;border-color:#f0e080}"
		".bulb.green-off{background:#dff5e8;border-color:#b0ddc0}"
		".info-grid{display:grid;grid-template-columns:auto 1fr auto 1fr;gap:6px 18px;"
		"font-size:.84rem;align-items:center}"
		".lbl{color:#aaa;font-size:.78rem}"
		".val{font-weight:500;color:#111}"
		".badge{display:inline-block;padding:2px 10px;border-radius:20px;font-size:.74rem;font-weight:500}"
		".badge.peak   {background:#fff8d6;color:#856200;border:1px solid #e8c840}"
		".badge.offpeak{background:#e8f5ee;color:#1a6e35;border:1px solid #80d4a0}"
		/* bottom row */
		".bottom-row{display:grid;grid-template-columns:1fr 1fr;gap:14px}"
		/* timing */
		".timing-row{display:flex;align-items:center;gap:10px;margin:8px 0}"
		".tlbl{width:46px;font-size:.78rem;font-weight:600}"
		".tlbl.red   {color:#d42020}"
		".tlbl.yellow{color:#b89000}"
		".tlbl.green {color:#1a8a3a}"
		".timing-bar{flex:1;height:8px;background:#f0f0f0;border-radius:4px;overflow:hidden}"
		".timing-fill{height:100%;border-radius:4px;transition:width .6s}"
		".fill-red   {background:#ff4444}"
		".fill-green {background:#22cc55}"
		".fill-yellow{background:#ffcc00}"
		".tval{color:#666;width:30px;text-align:right;font-size:.78rem}"
		/* controls */
		".btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
		"button{padding:10px 12px;border-radius:9px;border:none;cursor:pointer;"
		"font-size:.82rem;font-weight:600;transition:filter .15s;text-align:left;"
		"display:flex;align-items:center;gap:6px}"
		"button:hover{filter:brightness(.9)}"
		".btn-emerg{background:#ff4444;color:#fff}"
		".btn-clear{background:#22cc55;color:#fff}"
		".btn-dist {background:#3399ff;color:#fff}"
		".btn-cycle{background:#9966ff;color:#fff}"
		".btn-peak {background:#ffaa00;color:#fff}"
		".btn-quit {background:#444444;color:#fff}"
		"#status-bar{font-size:.7rem;color:#bbb;margin-top:10px}"
		"</style>"
		"</head>"
		"<body>"
		"<h1>Traffic Light Controller</h1>"

		/* ── Card 1: Current state (full width) ── */
		"<div class='card state-card'>"
		"<div class='card-title'>Current state</div>"
		"<div class='state-inner'>"
		"<div class='light-col'>"
		"<div class='bulb red-off'    id='b-red'></div>"
		"<div class='bulb yellow-off' id='b-yellow'></div>"
		"<div class='bulb green-off'  id='b-green'></div>"
		"</div>"
		"<div class='info-grid'>"
		"<span class='lbl'>Mode</span>     <span id='mode-val'  class='val'>—</span>"
		"<span class='lbl'>Phase</span>    <span id='phase-val' class='val'>—</span>"
		"<span class='lbl'>Remaining</span><span id='rem-val'   class='val'>—</span>"
		"<span class='lbl'>Distance</span> <span id='dist-val'  class='val'>—</span>"
		"<span class='lbl'>Peak hour</span><span id='peak-val'  class='val'>—</span>"
		"</div>"
		"</div>"
		"</div>"

		/* ── Bottom row: timing + controls ── */
		"<div class='bottom-row'>"

		/* Card 2: Cycle timing */
		"<div class='card'>"
		"<div class='card-title'>Cycle timing</div>"
		"<div class='timing-row'>"
		"<span class='tlbl red'>Red</span>"
		"<div class='timing-bar'><div class='timing-fill fill-red' id='bar-red' style='width:38%'></div></div>"
		"<span class='tval' id='t-red'>—</span>"
		"</div>"
		"<div class='timing-row'>"
		"<span class='tlbl green'>Green</span>"
		"<div class='timing-bar'><div class='timing-fill fill-green' id='bar-green' style='width:38%'></div></div>"
		"<span class='tval' id='t-green'>—</span>"
		"</div>"
		"<div class='timing-row'>"
		"<span class='tlbl yellow'>Yellow</span>"
		"<div class='timing-bar'><div class='timing-fill fill-yellow' id='bar-yellow' style='width:24%'></div></div>"
		"<span class='tval' id='t-yellow'>—</span>"
		"</div>"
		"<p style='font-size:.72rem;color:#bbb;margin-top:10px'>Width = proportion of total cycle</p>"
		"</div>"

		/* Card 3: Controls */
		"<div class='card'>"
		"<div class='card-title'>Controls</div>"
		"<div class='btn-grid'>"
		"<button class='btn-emerg' onclick='cmd(\"e\")'><span>&#9888;</span>Emergency</button>"
		"<button class='btn-clear' onclick='cmd(\"n\")'><span>&#10003;</span>Clear</button>"
		"<button class='btn-dist'  onclick='cmd(\"h\")'><span>&#9677;</span>Distance</button>"
		"<button class='btn-cycle' onclick='cmd(\"g\")'><span>&#8635;</span>Fixed cycle</button>"
		"<button class='btn-peak'  onclick='cmd(\"p\")'><span>&#9201;</span>Toggle peak</button>"
		"<button class='btn-quit'  onclick='cmd(\"q\")'><span>&#9632;</span>Quit</button>"
		"</div>"
		"</div>"

		"</div>" /* end .bottom-row */
		"<div id='status-bar'>Connecting\u2026</div>"

		"<script>"
		"function cmd(c){"
		"fetch('/cmd',{method:'POST',"
		"headers:{'Content-Type':'application/x-www-form-urlencoded'},"
		"body:'cmd='+c}).catch(()=>{});}"

		"function sc(id,cls){document.getElementById(id).className=cls}"
		"function st(id,txt){document.getElementById(id).textContent=txt}"

		"function poll(){"
		"fetch('/state').then(r=>r.json()).then(d=>{"
		"st('status-bar','Last update: '+new Date().toLocaleTimeString());"
		"const p=d.phase;"
		"sc('b-red',   'bulb '+(p==='RED'   ?'red-on'   :'red-off'));"
		"sc('b-yellow','bulb '+(p==='YELLOW'?'yellow-on':'yellow-off'));"
		"sc('b-green', 'bulb '+(p==='GREEN' ?'green-on' :'green-off'));"
		"st('mode-val', d.mode.replace('_',' '));"
		"st('phase-val',p);"
		"st('rem-val',  (d.remaining_ms/1000).toFixed(1)+' s');"
		"st('dist-val', d.distance_cm+' cm');"
		"const pkEl=document.getElementById('peak-val');"
		"pkEl.innerHTML='';"
		"const b=document.createElement('span');"
		"b.className='badge '+(d.peak_hour?'peak':'offpeak');"
		"b.textContent=d.peak_hour?'Peak hour':'Off-peak';"
		"pkEl.appendChild(b);"
		"const t=d.timings;"
		"const tot=(t.red_ms+t.green_ms+t.yellow_ms)||1;"
		"document.getElementById('bar-red').style.width=(t.red_ms/tot*100)+'%';"
		"document.getElementById('bar-green').style.width=(t.green_ms/tot*100)+'%';"
		"document.getElementById('bar-yellow').style.width=(t.yellow_ms/tot*100)+'%';"
		"st('t-red',   (t.red_ms/1000)+'s');"
		"st('t-green', (t.green_ms/1000)+'s');"
		"st('t-yellow',(t.yellow_ms/1000)+'s');"
		"}).catch(()=>st('status-bar','Connection error'));}"

		"poll();setInterval(poll,1000);"
		"</script>"
		"</body></html>";

static void http_send(int fd, const char *status,
                      const char *ctype, const char *body, int len)
{
    char hdr[256];
    int hlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n",
        status, ctype, len);
    send(fd, hdr, hlen, 0);
    if (body && len > 0)
        send(fd, body, len, 0);
}

static void handle_request(int fd)
{
    char req[2048] = {0};
    int n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) return;
    req[n] = '\0';

    /* Parse method and path from first line */
    char method[8] = {0}, path[64] = {0};
    sscanf(req, "%7s %63s", method, path);

    if (strcmp(path, "/state") == 0) {
        /* JSON status */
        char *json = build_json();
        if (json) {
            http_send(fd, "200 OK", "application/json", json, (int)strlen(json));
            free(json);
        }

    } else if (strcmp(path, "/cmd") == 0 && strcmp(method, "POST") == 0) {
        /* Find body after blank line */
        char *body = strstr(req, "\r\n\r\n");
        if (body) {
            body += 4;
            /* expect body: cmd=X */
            char c = 0;
            if (sscanf(body, "cmd=%c", &c) == 1)
                apply_command(c);
        }
        http_send(fd, "200 OK", "text/plain", "OK", 2);

    } else {
        /* Default: serve dashboard */
        http_send(fd, "200 OK", "text/html",
                  HTML_PAGE, (int)sizeof(HTML_PAGE) - 1);
    }
}

static void *http_thread(void *arg)
{
    (void)arg;
    printf("[HTTP] Starting server on port %d\n", HTTP_PORT);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("[HTTP] socket"); return NULL; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(HTTP_PORT);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[HTTP] bind"); close(srv); return NULL;
    }
    if (listen(srv, 8) < 0) {
        perror("[HTTP] listen"); close(srv); return NULL;
    }

    printf("[HTTP] Dashboard: http://10.0.0.1:%d\n", HTTP_PORT);

    while (1) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int fd = accept(srv, (struct sockaddr *)&cli, &cli_len);
        if (fd < 0) continue;
        handle_request(fd);
        close(fd);
    }
    return NULL;
}

/* ═══════════════════════════════════════════════
   THREAD 6: STATUS LOGGER
   Priority 10
   ═══════════════════════════════════════════════ */

static void *logger_thread(void *arg)
{
    (void)arg;
    printf("[LOGGER] started\n");
    while (1) {
        sleep_ms(5000);
        pthread_mutex_lock(&g_mutex);
        AppMode      m    = g_mode;
        Phase        p    = g_phase;
        unsigned int el   = g_elapsed_ms;
        double       dist = g_distance;
        int          peak = g_peak_hour;
        unsigned int rm   = g_red_ms;
        unsigned int gm   = g_green_ms;
        unsigned int ym   = g_yellow_ms;
        pthread_mutex_unlock(&g_mutex);

        unsigned int dur = (p == PHASE_RED) ? rm : (p == PHASE_GREEN) ? gm : ym;
        unsigned int rem = (el < dur) ? dur - el : 0u;

        printf("\n  +------------------------------------------+\n");
        printf("  |   TRAFFIC LIGHT STATUS                   |\n");
        printf("  +------------------------------------------+\n");
        printf("  |  Mode      : %-26s|\n", mode_str(m));
        printf("  |  Phase     : %-26s|\n", phase_str(p));
        printf("  |  Remaining : %-23u ms |\n", rem);
        printf("  |  Distance  : %-23.1f cm |\n", dist);
        printf("  |  Peak hour : %-26s|\n", peak ? "YES" : "No");
        printf("  |  Timings   : R=%-2us G=%-2us Y=%-2us          |\n",
               rm/1000u, gm/1000u, ym/1000u);
        printf("  +------------------------------------------+\n\n");
    }
    return NULL;
}

/* ═══════════════════════════════════════════════
   MAIN
   ═══════════════════════════════════════════════ */

static pthread_t make_thread(pthread_attr_t *attr, int prio,
                             void *(*fn)(void *))
{
    pthread_t t;
    struct sched_param sp;
    sp.sched_priority = prio;
    pthread_attr_setschedparam(attr, &sp);
    pthread_create(&t, attr, fn, NULL);
    return t;
}

int main(void)
{
    printf("==========================================\n");
    printf("  Smart Traffic Light Controller\n");
    printf("  QNX SDP 8 — Raspberry Pi 4\n");
    printf("  GPIO via mmap (no rpi_gpio driver)\n");
    printf("  Web UI: http://10.0.0.1:%d\n", HTTP_PORT);
    printf("==========================================\n\n");

    if (gpio_init() != 0) {
        fprintf(stderr,
            "Run as root: ssh root@10.0.0.1 /tmp/traffic_light\n");
        return EXIT_FAILURE;
    }

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    pthread_t t_cmd    = make_thread(&attr, 25, cmd_thread);
    pthread_t t_fsm    = make_thread(&attr, 20, fsm_thread);
    pthread_t t_sensor = make_thread(&attr, 15, sensor_thread);
    pthread_t t_http   = make_thread(&attr, 12, http_thread);
    pthread_t t_peak   = make_thread(&attr, 11, peak_thread);
    pthread_t t_logger = make_thread(&attr, 10, logger_thread);

    (void)t_fsm; (void)t_sensor;
    (void)t_http; (void)t_peak; (void)t_logger;

    pthread_attr_destroy(&attr);
    pthread_join(t_cmd, NULL);   /* exits only when 'q' pressed */

    gpio_cleanup();
    return EXIT_SUCCESS;
}
