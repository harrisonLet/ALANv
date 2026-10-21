/*
 * gs_term.c - lightweight terminal ground station
 *
 * Reads the SAMD21 LoRa bridge over USB serial, parses LoRaTelemetry_t
 * packets from the boat and redraws an ASCII status screen. Keypresses are
 * forwarded to the bridge as single-byte commands, same keys as the boat
 * expects.
 *
 * Wire format from the boat (raw LoRa payload, forwarded byte for byte):
 *   [0] dest 0xAA  [1] src 0xBB  [2] seq  [3] len (= 72)  [4..75] LoRaTelemetry_t
 *
 * Build:  make
 * Run:    ./gs_term [/dev/ttyACM0]
 * Quit:   Ctrl-C
 */

#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "telemetry.h"

_Static_assert(sizeof(LoRaTelemetry_t) == 72, "LoRaTelemetry_t must be 72 bytes");

#define ADDR_GROUND    0xAA
#define ADDR_BOAT      0xBB
#define HDR_LEN        4
#define PKT_LEN        (HDR_LEN + sizeof(LoRaTelemetry_t))
#define NO_SIGNAL_MS   3000
#define SERVO_RANGE    135
#define CMD_LOG_LEN    5

/* ─── State ─────────────────────────────────────────────────────── */

static struct termios s_stdin_orig;
static int            s_stdin_raw;
static volatile sig_atomic_t s_quit;

static LoRaTelemetry_t s_telem;
static int      s_have_telem;
static uint8_t  s_last_seq;
static uint32_t s_pkts, s_lost, s_junk;
static double   s_last_rx;          /* seconds, monotonic */

static uint16_t s_cmds_sent;
static char     s_cmd_log[CMD_LOG_LEN][48];
static int      s_cmd_log_n;

/* ─── Helpers ───────────────────────────────────────────────────── */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void on_signal(int sig) { (void)sig; s_quit = 1; }

static void restore_terminal(void)
{
    if (s_stdin_raw) tcsetattr(STDIN_FILENO, TCSANOW, &s_stdin_orig);
    printf("\033[?25h\n");   /* show cursor again */
    fflush(stdout);
}

static int open_serial(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios t;
    if (tcgetattr(fd, &t) < 0) { close(fd); return -1; }
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }
    return fd;
}

static void stdin_raw(void)
{
    if (!isatty(STDIN_FILENO)) return;
    tcgetattr(STDIN_FILENO, &s_stdin_orig);
    struct termios t = s_stdin_orig;
    t.c_lflag &= ~(ICANON | ECHO);   /* keep ISIG so Ctrl-C still works */
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    s_stdin_raw = 1;
}

/* Validity_e codes from Common/Inc/sharedSensor.h on sensor_polling */
static const char *validity_name(uint8_t v)
{
    switch (v) {
    case 0x00: return "UNKNOWN";
    case 0x01: return "VALID";
    case 0x02: return "CALIBRATING";
    case 0x03: return "WARMING";
    case 0x04: return "WINDOW_FILLING";
    case 0x05: return "DISABLED";
    case 0x06: return "PARTIAL_DATA";
    case 0x10: return "UNTRUSTED";
    case 0x11: return "OUT_OF_RANGE";
    case 0x12: return "DEGRADED";
    case 0x13: return "IMPLAUSIBLE";
    case 0x14: return "STATIC";
    case 0x15: return "OVER_SATURATION";
    case 0x16: return "UNDER_SATURATION";
    case 0x20: return "ERROR";
    case 0x21: return "UNREACHABLE";
    case 0x22: return "PARSE_FAIL";
    case 0x23: return "ERROR_UNKNOWN";
    case 0x80: return "INVALID_BATTERY";
    case 0x90: return "INVALID_ENCODER";
    case 0xA0: return "INVALID_GPS";
    case 0xA1: return "GPS_STALE";
    case 0xA2: return "GPS_NO_FIX";
    case 0xA3: return "GPS_CHECKSUM";
    case 0xB0: return "INVALID_IMU";
    case 0xC0: return "INVALID_WIND";
    default:   return "?";
    }
}

/* 0x01-0x0F usable, 0x10-0x1F untrusted, everything else failed */
static const char *validity_tag(uint8_t v)
{
    if (v >= 0x01 && v < 0x10) return "[ OK ]";
    if (v >= 0x10 && v < 0x20) return "[WARN]";
    return "[FAIL]";
}

/* ASCII bar for a servo angle: [.........|....#....] */
static void servo_bar(char *out, int angle)
{
    const int width = 41;   /* odd so 0 has a centre cell */
    if (angle >  SERVO_RANGE) angle =  SERVO_RANGE;
    if (angle < -SERVO_RANGE) angle = -SERVO_RANGE;
    int pos = (angle + SERVO_RANGE) * (width - 1) / (2 * SERVO_RANGE);

    out[0] = '[';
    for (int i = 0; i < width; i++)
        out[1 + i] = (i == pos) ? '#' : (i == width / 2) ? '|' : '.';
    out[1 + width] = ']';
    out[2 + width] = '\0';
}

/* ─── Commands ──────────────────────────────────────────────────── */

static const char *cmd_desc(char c)
{
    switch (c) {
    case 'q': return "rudder +20";
    case 'e': return "rudder -20";
    case 'a': return "rudder +10";
    case 'd': return "rudder -10";
    case 'z': return "rudder +5";
    case 'c': return "rudder -5";
    case 'w': return "sail   +20";
    case 'r': return "sail   -20";
    case 's': return "sail   +10";
    case 'f': return "sail   -10";
    case 'x': return "sail   +5";
    case 'v': return "sail   -5";
    default:  return NULL;
    }
}

static void send_cmd(int fd, char c)
{
    const char *desc = cmd_desc(c);
    if (!desc) return;
    if (write(fd, &c, 1) != 1) return;
    s_cmds_sent++;

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);

    /* newest first */
    memmove(s_cmd_log[1], s_cmd_log[0], sizeof(s_cmd_log[0]) * (CMD_LOG_LEN - 1));
    snprintf(s_cmd_log[0], sizeof(s_cmd_log[0]), "%02d:%02d:%02d  '%c'  %s",
             tm.tm_hour, tm.tm_min, tm.tm_sec, c, desc);
    if (s_cmd_log_n < CMD_LOG_LEN) s_cmd_log_n++;
}

/* ─── Parser ────────────────────────────────────────────────────── */

static uint8_t s_buf[1024];
static size_t  s_buf_len;

static void handle_packet(const uint8_t *p)
{
    uint8_t seq = p[2];
    if (s_pkts > 0) s_lost += (uint8_t)(seq - s_last_seq - 1);
    s_last_seq = seq;
    memcpy(&s_telem, p + HDR_LEN, sizeof(s_telem));
    s_have_telem = 1;
    s_pkts++;
    s_last_rx = now_s();
}

/* Scan the buffer for [AA BB seq 72] + payload, skipping anything else */
static void parse(void)
{
    size_t i = 0;
    while (s_buf_len - i >= PKT_LEN) {
        const uint8_t *p = s_buf + i;
        if (p[0] == ADDR_GROUND && p[1] == ADDR_BOAT && p[3] == sizeof(LoRaTelemetry_t)) {
            handle_packet(p);
            i += PKT_LEN;
        } else {
            s_junk++;
            i++;
        }
    }
    memmove(s_buf, s_buf + i, s_buf_len - i);
    s_buf_len -= i;
}

/* ─── Wind / sail dial ──────────────────────────────────────────── */

/* Boat-relative circle: bow is always up, starboard is right.
   Terminal cells are about twice as tall as wide, so RX = 2 * RY. */
#define DIAL_W   23
#define DIAL_H   11
#define DIAL_RX  10.0
#define DIAL_RY  5.0

enum { C_NONE, C_RING, C_BOAT, C_SAIL, C_WIND };
static const char *DIAL_COLOR[] = { "", "\033[90m", "\033[1;33m", "\033[1;32m", "\033[1;36m" };

static char    s_dial_ch[DIAL_H][DIAL_W];
static uint8_t s_dial_col[DIAL_H][DIAL_W];

/* r is 0..1 of the radius (negative = opposite side), deg clockwise from bow */
static void dial_put(double r, double deg, char ch, uint8_t col)
{
    double a = deg * M_PI / 180.0;
    int x = (int)lround(DIAL_W / 2 + r * sin(a) * DIAL_RX);
    int y = (int)lround(DIAL_H / 2 - r * cos(a) * DIAL_RY);
    if (x < 0 || x >= DIAL_W || y < 0 || y >= DIAL_H) return;
    s_dial_ch[y][x]  = ch;
    s_dial_col[y][x] = col;
}

/* Character that best matches a line drawn at deg */
static char line_char(double deg)
{
    double a = fmod(fmod(deg, 180) + 180, 180);
    if (a < 22.5 || a >= 157.5) return '|';
    if (a >= 67.5 && a < 112.5) return '-';
    return a < 90 ? '/' : '\\';
}

/* Arrowhead for something travelling towards deg */
static char head_char(double deg)
{
    int q = (int)lround(fmod(fmod(deg, 360) + 360, 360) / 90.0) % 4;
    return "^>v<"[q];
}

static void dial_build(int wind, int sail)
{
    memset(s_dial_ch, ' ', sizeof(s_dial_ch));
    memset(s_dial_col, C_NONE, sizeof(s_dial_col));

    for (int d = 0; d < 360; d += 3)
        dial_put(1.0, d, '.', C_RING);

    /* boat: arrow from the mast to the bow */
    for (double r = 0.15; r < 0.9; r += 0.05)
        dial_put(r, 0, '|', C_BOAT);
    dial_put(1.0, 0, '^', C_BOAT);

    /* sail: leading edge points at the sail angle, most of the wing trails aft */
    for (double r = -0.75; r <= 0.35; r += 0.05)
        dial_put(r, sail, line_char(sail), C_SAIL);
    dial_put(0, 0, 'O', C_SAIL);

    /* wind: comes from the rim at the wind angle and blows towards the mast */
    for (double r = 1.0; r > 0.25; r -= 0.05)
        dial_put(r, wind, line_char(wind), C_WIND);
    dial_put(0.2, wind, head_char(wind + 180), C_WIND);
    dial_put(1.0, wind, 'W', C_WIND);
}

static void dial_print_row(int y)
{
    uint8_t cur = C_NONE;
    for (int x = 0; x < DIAL_W; x++) {
        if (s_dial_col[y][x] != cur) {
            cur = s_dial_col[y][x];
            fputs("\033[0m", stdout);
            fputs(DIAL_COLOR[cur], stdout);
        }
        putchar(s_dial_ch[y][x]);
    }
    fputs("\033[0m", stdout);
}

/* ─── Display ───────────────────────────────────────────────────── */

static const char *HR =
    "--------------------------------------------------------------------------------";

static void draw(const char *port)
{
    const LoRaTelemetry_t *t = &s_telem;
    double age = s_have_telem ? now_s() - s_last_rx : -1;
    char bar[64], junk[16];
    snprintf(junk, sizeof(junk), "%uB", s_junk);

    printf("\033[H\033[2J\033[?25l");   /* home, clear, hide cursor */

    printf("ALANv GROUND STATION   %s\n", port);
    printf("pkts %-6u lost %-5u junk %-6s ", s_pkts, s_lost, junk);
    if (!s_have_telem)                  printf("WAITING FOR BOAT...\n");
    else if (age * 1000 > NO_SIGNAL_MS) printf("NO SIGNAL  last %.1fs ago\n", age);
    else                                printf("LINK OK  last %.1fs ago  seq %u\n", age, s_last_seq);
    printf("%s\n", HR);

    if (!s_have_telem) {
        printf("\n  no telemetry yet\n\n");
    } else {
        printf("%-12s %-6s %-16s %s\n", "SUBSYSTEM", "", "VALIDITY", "DATA");

        printf("%-12s %s %-16s %.2f V   cells %.2f %.2f %.2f %.2f\n", "battery",
               validity_tag(t->battery_valid), validity_name(t->battery_valid),
               t->battery_voltage / 1000.0,
               t->cell_voltage[0] / 1000.0, t->cell_voltage[1] / 1000.0,
               t->cell_voltage[2] / 1000.0, t->cell_voltage[3] / 1000.0);

        printf("%-12s %s %-16s sail %+4d deg   mag %-5u %s %s %s\n", "encoder",
               validity_tag(t->encoder_valid), validity_name(t->encoder_valid),
               t->sail_ang_forward, t->sail_encoder_magnitude,
               (t->sail_encoder_status & 0x20) ? "MD" : "md",
               (t->sail_encoder_status & 0x10) ? "ML" : "ml",
               (t->sail_encoder_status & 0x08) ? "MH" : "mh");

        printf("%-12s %s %-16s apparent %+4d deg (%s)\n", "wind",
               validity_tag(t->wind_valid), validity_name(t->wind_valid),
               t->ang_wind, t->ang_wind > 0 ? "starboard" : t->ang_wind < 0 ? "port" : "ahead");

        printf("%-12s %s %-16s %.6f %.6f  alt %.1fm  sats %u fix %u  %02u:%02u:%02uZ\n", "gps",
               validity_tag(t->gps_valid), validity_name(t->gps_valid),
               t->latitude, t->longitude, t->altitude, t->satellites, t->fix_valid,
               t->utc_hours, t->utc_minutes, t->utc_seconds);

        printf("%-12s %s %-16s cal S%u G%u A%u M%u   temp %dC\n", "imu",
               validity_tag(t->imu_valid), validity_name(t->imu_valid),
               (t->imu_cal >> 6) & 3, (t->imu_cal >> 4) & 3,
               (t->imu_cal >> 2) & 3, t->imu_cal & 3, t->imu_temp);

        printf("%-12s %s %-16s hdg %3u  roll %+4d  pitch %+4d\n", "orientation",
               validity_tag(t->orientation_valid), validity_name(t->orientation_valid),
               t->heading, t->roll, t->pitch);

        printf("%s\n", HR);
        servo_bar(bar, t->rudder_servo_angle);
        printf("rudder servo %+5d  %s\n", t->rudder_servo_angle, bar);
        servo_bar(bar, t->sail_servo_angle);
        printf("sail servo   %+5d  %s\n", t->sail_servo_angle, bar);
        printf("%-19s-135%18s0%18s+135\n", "", "", "");

        dial_build(t->ang_wind, t->sail_ang_forward);
        for (int y = 0; y < DIAL_H; y++) {
            printf("      ");
            dial_print_row(y);
            switch (y) {
            case 1: printf("     \033[1;36mW\033[0m  wind  %+4d deg  from %-9s %s", t->ang_wind,
                           t->ang_wind > 0 ? "starboard" : t->ang_wind < 0 ? "port" : "ahead",
                           validity_tag(t->wind_valid)); break;
            case 3: printf("     \033[1;32m%c\033[0m  sail  %+4d deg  to   %-9s %s", line_char(t->sail_ang_forward),
                           t->sail_ang_forward,
                           t->sail_ang_forward > 0 ? "starboard" : t->sail_ang_forward < 0 ? "port" : "centre",
                           validity_tag(t->encoder_valid)); break;
            case 5: printf("     \033[1;33m^\033[0m  bow   hdg %3u", t->heading); break;
            case 7: printf("     \033[90mbow up, starboard right\033[0m"); break;
            }
            putchar('\n');
        }

        printf("%s\n", HR);
        uint32_t up = t->uptime_ms / 1000;
        printf("mode %u   desired hdg %3u   segment %u   drift %+d m   uptime %02u:%02u:%02u\n",
               t->mode, t->desired_heading, t->route_segment, t->drift_distance,
               up / 3600, (up / 60) % 60, up % 60);
    }

    printf("%s\n", HR);
    printf("COMMANDS  sent %u", s_cmds_sent);
    if (s_have_telem) printf("   boat received %u", t->last_cmd_seq);
    printf("\n");
    for (int i = 0; i < CMD_LOG_LEN; i++)
        printf("  %s %s\n", i == 0 && s_cmd_log_n ? ">" : " ", i < s_cmd_log_n ? s_cmd_log[i] : "");
    printf("%s\n", HR);
    printf("rudder q/e a/d z/c   sail w/r s/f x/v   (+20/-20 +10/-10 +5/-5)   Ctrl-C quit\n");
    fflush(stdout);
}

/* ─── Main ──────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    const char *port = argc > 1 ? argv[1] : "/dev/ttyACM0";

    int fd = open_serial(port);
    if (fd < 0) {
        fprintf(stderr, "cannot open %s: %s\n", port, strerror(errno));
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    stdin_raw();
    atexit(restore_terminal);

    draw(port);
    while (!s_quit) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        if (s_stdin_raw) FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = { 0, 250000 };   /* redraw at least 4x/s so link age ticks */

        int n = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }

        if (n > 0 && FD_ISSET(fd, &rfds)) {
            ssize_t r = read(fd, s_buf + s_buf_len, sizeof(s_buf) - s_buf_len);
            if (r > 0) {
                s_buf_len += r;
                parse();
            } else if (r == 0 || (errno != EAGAIN && errno != EINTR)) {
                fprintf(stderr, "\nserial port closed\n");
                break;
            }
        }

        if (n > 0 && s_stdin_raw && FD_ISSET(STDIN_FILENO, &rfds)) {
            char c;
            while (read(STDIN_FILENO, &c, 1) == 1) send_cmd(fd, c);
        }

        draw(port);
    }

    close(fd);
    return 0;
}
