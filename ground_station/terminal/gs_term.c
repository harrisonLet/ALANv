/*
 * gs_term.c - lightweight terminal ground station
 *
 * Reads the SAMD21 LoRa bridge over USB serial, parses LoRaTelemetry_t
 * packets from the boat and redraws an ASCII status screen. Commands are
 * typed on the line at the bottom (type "help") and sent as command packets.
 *
 * Wire format from the boat (raw LoRa payload, forwarded byte for byte):
 *   [0] dest 0xAA  [1] src 0xBB  [2] seq  [3] len (= 74)  [4] status flags  [5..77] LoRaTelemetry_t
 *
 * Wire format to the boat (the bridge sends each write as one LoRa packet):
 *   [0] dest 0xBB  [1] src 0xAA  [2] seq  [3] len (= 17)  [4] command type  [5..20] payload
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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "telemetry.h"
#include "commands.h"

_Static_assert(sizeof(LoRaTelemetry_t) == 73, "LoRaTelemetry_t must be 73 bytes");

#define ADDR_GROUND    0xAA
#define ADDR_BOAT      0xBB
#define HDR_LEN        5   /* dest, src, seq, len, flags */
#define PKT_LEN        (HDR_LEN + sizeof(LoRaTelemetry_t))
#define CMD_PKT_LEN    (HDR_LEN + CMD_PAYLOAD_LEN)   /* 21, the length the CM4 accepts */
#define NO_SIGNAL_MS   3000
#define SERVO_RANGE    135
#define CMD_LOG_LEN    6
#define HISTORY_LEN    20
#define LINE_MAX_LEN   72
#define HB_PERIOD_S    1.0    /* boat drops to SAFE after 5 s without a command in manual/semi */
#define MIN_GAP_S      0.05   /* bridge flushes after 10 ms idle; keep packets from merging */

/* ─── State ─────────────────────────────────────────────────────── */

static struct termios s_stdin_orig;
static int            s_stdin_raw;
static volatile sig_atomic_t s_quit;

static LoRaTelemetry_t s_telem;
static int      s_have_telem;
static uint8_t  s_last_seq;
static uint32_t s_pkts, s_lost, s_junk;
static double   s_last_rx;          /* seconds, monotonic */

/* Command log: newest first. result is a cmd_result_t once the boat reports it */
enum { RES_PENDING = -1, RES_MISSED = -2 };
typedef struct {
    char    when[9];
    char    text[40];
    uint8_t seq;
    int     result;
} cmd_entry_t;

static int      s_serial_fd = -1;
static uint8_t  s_tx_seq;            /* boat expects the first command to be seq 0 */
static uint32_t s_cmds_sent, s_hbs_sent;
static double   s_last_send;         /* seconds, monotonic; 0 = never */
static int      s_hb_on = 1;
static cmd_entry_t s_cmd_log[CMD_LOG_LEN];
static int      s_cmd_log_n;

/* Command line */
static char     s_line[LINE_MAX_LEN + 1];
static size_t   s_line_len;
static char     s_status[160];       /* reply to the last line typed */
static int      s_status_err;
static char     s_history[HISTORY_LEN][LINE_MAX_LEN + 1];
static int      s_history_n;
static int      s_history_pos = -1;  /* -1 = editing a new line */
static int      s_esc;               /* escape sequence state: 0 none, 1 got ESC, 2 got ESC [ */
static int      s_show_help;

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

/* LoRaTelemetryValidity_e from Common/Inc/loraTelemetry.h on master:
   bit 0 = sensor metadata healthy, bit 1 = fresh telemetry reading */
static const char *validity_name(uint8_t v)
{
    switch (v) {
    case 0x0: return "(00) FAILED";
    case 0x1: return "(01) NO DATA";
    case 0x2: return "(10) SENSOR FAULT";
    case 0x3: return "(11) ALL GOOD";
    default:  return "?";
    }
}

/* 11 good, 01/10 partial, 00 failed */
static const char *validity_tag(uint8_t v)
{
    if (v == 0x3) return "[ OK ]";
    if (v == 0x1 || v == 0x2) return "[WARN]";
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

/* How one actuator is driven. Each system mode is one rudder/sail pair. */
enum { ACT_AUTO, ACT_SEMI, ACT_MAN, ACT_SAFE };
static const char *ACT_NAME[] = { "AUTO", "SEMI", "MAN", "SAFE" };

static const struct {
    const char *name;
    int rudder, sail;
} MODES[SYSTEM_MODE_COUNT] = {
    [SYSTEM_MODE_AUTO]                  = { "AUTO",                  ACT_AUTO, ACT_AUTO },
    [SYSTEM_MODE_FULL_MAN]              = { "FULL_MAN",              ACT_MAN,  ACT_MAN  },
    [SYSTEM_MODE_SAFE]                  = { "SAFE",                  ACT_SAFE, ACT_SAFE },
    [SYSTEM_MODE_AUTO_SAIL_MAN_RUDDER]  = { "AUTO_SAIL_MAN_RUDDER",  ACT_MAN,  ACT_AUTO },
    [SYSTEM_MODE_AUTO_RUDDER_MAN_SAIL]  = { "AUTO_RUDDER_MAN_SAIL",  ACT_AUTO, ACT_MAN  },
    [SYSTEM_MODE_STANDBY]               = { "STANDBY",               ACT_SAFE, ACT_SAFE },
    [SYSTEM_MODE_FULL_SEMI]             = { "FULL_SEMI",             ACT_SEMI, ACT_SEMI },
    [SYSTEM_MODE_SEMI_RUDDER_MAN_SAIL]  = { "SEMI_RUDDER_MAN_SAIL",  ACT_SEMI, ACT_MAN  },
    [SYSTEM_MODE_SEMI_RUDDER_AUTO_SAIL] = { "SEMI_RUDDER_AUTO_SAIL", ACT_SEMI, ACT_AUTO },
    [SYSTEM_MODE_SEMI_SAIL_MAN_RUDDER]  = { "SEMI_SAIL_MAN_RUDDER",  ACT_MAN,  ACT_SEMI },
    [SYSTEM_MODE_SEMI_SAIL_AUTO_RUDDER] = { "SEMI_SAIL_AUTO_RUDDER", ACT_AUTO, ACT_SEMI },
};

static const char *RESULT_NAME[CMD_RESULT_COUNT] = {
    "NONE", "OK", "UNSUPPORTED", "BAD_LEN", "BAD_VALUE",
    "TOO_MANY", "INCOMPLETE", "BAD_CRC", "DROPPED", "WRONG_MODE",
};

static const char *result_name(int r)
{
    if (r == RES_PENDING) return "...";
    if (r == RES_MISSED)  return "not reported";   /* boat reported a later command first; may still have run */
    if (r >= 0 && r < CMD_RESULT_COUNT) return RESULT_NAME[r];
    return "?";
}

static void set_status(int err, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void set_status(int err, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_status, sizeof(s_status), fmt, ap);
    va_end(ap);
    s_status_err = err;
}

/* Build and write one 21-byte command packet. Returns the seq used, or -1 on a write error. */
static int send_packet(uint8_t type, int16_t value)
{
    /* Two writes closer than the bridge's 10 ms flush gap would go out as one packet,
       which the CM4 rejects for its length */
    double gap = now_s() - s_last_send;
    if (s_last_send > 0 && gap < MIN_GAP_S) usleep((useconds_t)((MIN_GAP_S - gap) * 1e6));

    uint8_t pkt[CMD_PKT_LEN] = {0};
    pkt[0] = ADDR_BOAT;
    pkt[1] = ADDR_GROUND;
    pkt[2] = s_tx_seq;
    pkt[3] = 1 + CMD_PAYLOAD_LEN;
    pkt[4] = type;
    pkt[5] = (uint8_t)((uint16_t)value & 0xFF);   /* little-endian int16 */
    pkt[6] = (uint8_t)((uint16_t)value >> 8);

    if (write(s_serial_fd, pkt, sizeof(pkt)) != (ssize_t)sizeof(pkt)) return -1;
    s_last_send = now_s();
    return s_tx_seq++;   /* wraps 255 -> 0 like the boat expects */
}

/* Send a command and add it to the log. Heartbeats are not logged: the boat reports no result for them. */
static void send_logged(uint8_t type, int16_t value, const char *text)
{
    int seq = send_packet(type, value);
    if (seq < 0) {
        set_status(1, "serial write failed: %s", strerror(errno));
        return;
    }
    s_cmds_sent++;

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);

    memmove(&s_cmd_log[1], &s_cmd_log[0], sizeof(s_cmd_log[0]) * (CMD_LOG_LEN - 1));
    cmd_entry_t *e = &s_cmd_log[0];
    snprintf(e->when, sizeof(e->when), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    snprintf(e->text, sizeof(e->text), "%s", text);
    e->seq = (uint8_t)seq;
    e->result = RES_PENDING;
    if (s_cmd_log_n < CMD_LOG_LEN) s_cmd_log_n++;

    set_status(0, "sent %s (seq %d)", text, seq);
}

static void send_heartbeat(void)
{
    if (send_packet(COMMAND_NONE, 0) >= 0) s_hbs_sent++;
}

/* auto/semi/man, plus a few spellings */
static int parse_act(const char *s)
{
    if (!strcasecmp(s, "auto") || !strcasecmp(s, "a")) return ACT_AUTO;
    if (!strcasecmp(s, "semi") || !strcasecmp(s, "s")) return ACT_SEMI;
    if (!strcasecmp(s, "man")  || !strcasecmp(s, "manual") || !strcasecmp(s, "m")) return ACT_MAN;
    return -1;
}

/* Whole-string integer, so "20x" or "" is an error rather than 20 or 0 */
static int parse_int(const char *s, long *out)
{
    if (!s || !*s) return 0;
    char *end;
    errno = 0;
    long v = strtol(s, &end, 0);
    if (errno || *end) return 0;
    *out = v;
    return 1;
}

/* Current rudder/sail modes from telemetry, or -1 if unknown */
static void current_acts(int *rudder, int *sail)
{
    *rudder = *sail = -1;
    if (s_have_telem && s_telem.mode < SYSTEM_MODE_COUNT && MODES[s_telem.mode].name) {
        *rudder = MODES[s_telem.mode].rudder;
        *sail   = MODES[s_telem.mode].sail;
    }
}

/* mode <m> | mode -r <m> [-s <m>] | mode -s <m> [-r <m>] */
static void cmd_mode(int argc, char **argv)
{
    int rudder = -1, sail = -1;

    if (argc == 2 && argv[1][0] != '-') {
        rudder = sail = parse_act(argv[1]);
        if (rudder < 0) { set_status(1, "unknown mode '%s' (auto, semi, man)", argv[1]); return; }
    } else {
        for (int i = 1; i < argc; i++) {
            int *dst = !strcmp(argv[i], "-r") ? &rudder : !strcmp(argv[i], "-s") ? &sail : NULL;
            if (!dst) { set_status(1, "usage: mode <m> | mode [-r <m>] [-s <m>]   m = auto, semi, man"); return; }
            if (i + 1 >= argc) { set_status(1, "%s needs a mode (auto, semi, man)", argv[i]); return; }
            *dst = parse_act(argv[++i]);
            if (*dst < 0) { set_status(1, "unknown mode '%s' (auto, semi, man)", argv[i]); return; }
        }
        if (rudder < 0 && sail < 0) { set_status(1, "usage: mode <m> | mode [-r <m>] [-s <m>]   m = auto, semi, man"); return; }

        /* The flag left out keeps what the boat last reported */
        int cur_r, cur_s;
        current_acts(&cur_r, &cur_s);
        if (rudder < 0) rudder = cur_r;
        if (sail < 0)   sail = cur_s;
        if (rudder < 0 || rudder == ACT_SAFE || sail < 0 || sail == ACT_SAFE) {
            set_status(1, "boat's current mode is %s, give both -r and -s",
                       s_have_telem ? "SAFE/STANDBY" : "unknown (no telemetry)");
            return;
        }
    }

    for (int m = 0; m < SYSTEM_MODE_COUNT; m++) {
        if (MODES[m].name && MODES[m].rudder == rudder && MODES[m].sail == sail) {
            char text[40];
            snprintf(text, sizeof(text), "mode %s", MODES[m].name);
            send_logged((uint8_t)(0x10 | m), 0, text);
            return;
        }
    }
    set_status(1, "no system mode for rudder %s / sail %s", ACT_NAME[rudder], ACT_NAME[sail]);
}

/* rsang / ssang / head / sang: one angle, range checked like the boat does */
static void cmd_angle(int argc, char **argv, uint8_t type, long min, long max, const char *what)
{
    long v;
    if (argc != 2 || !parse_int(argv[1], &v)) { set_status(1, "usage: %s <deg>   %s, %ld..%ld", argv[0], what, min, max); return; }
    if (v < min || v > max) { set_status(1, "%s: %ld out of range %ld..%ld", argv[0], v, min, max); return; }
    char text[40];
    snprintf(text, sizeof(text), "%s %ld", argv[0], v);
    send_logged(type, (int16_t)v, text);
}

static void run_line(char *line)
{
    char *argv[8];
    int argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < 8; tok = strtok(NULL, " \t"))
        argv[argc++] = tok;
    if (argc == 0) return;

    const char *c = argv[0];
    s_show_help = 0;

    if (!strcmp(c, "help") || !strcmp(c, "?")) {
        s_show_help = 1;
        set_status(0, "help shown below the log, any command hides it");
    }
    else if (!strcmp(c, "mode"))  cmd_mode(argc, argv);
    else if (!strcmp(c, "safe") || !strcmp(c, "kill") || !strcmp(c, "stop")) {
        send_logged(COMMAND_MODE_SAFE, 0, "mode SAFE");
    }
    else if (!strcmp(c, "standby")) send_logged(COMMAND_MODE_STANDBY, 0, "mode STANDBY");
    else if (!strcmp(c, "rsang")) cmd_angle(argc, argv, COMMAND_RUDDER_SERVO, -LIMIT_SERVO_DEG, LIMIT_SERVO_DEG, "rudder servo, rudder in man");
    else if (!strcmp(c, "ssang")) cmd_angle(argc, argv, COMMAND_SAIL_SERVO, -LIMIT_SERVO_DEG, LIMIT_SERVO_DEG, "tail servo, sail in man");
    else if (!strcmp(c, "head"))  cmd_angle(argc, argv, COMMAND_RUDDER_PID, 0, LIMIT_HEADING_MAX, "heading to hold, rudder in semi");
    else if (!strcmp(c, "sang"))  cmd_angle(argc, argv, COMMAND_SAIL_PID, -LIMIT_SAIL_PID_DEG, LIMIT_SAIL_PID_DEG, "main sail angle to hold, sail in semi");
    else if (!strcmp(c, "hb")) {
        if (argc == 2 && !strcmp(argv[1], "on"))       { s_hb_on = 1; set_status(0, "heartbeat on, every %.1f s", HB_PERIOD_S); }
        else if (argc == 2 && !strcmp(argv[1], "off")) { s_hb_on = 0; set_status(0, "heartbeat off: the boat drops to SAFE 5 s after the last command in man/semi"); }
        else set_status(1, "usage: hb on|off");
    }
    else if (!strcmp(c, "raw")) {
        long type, v = 0;
        if (argc < 2 || argc > 3 || !parse_int(argv[1], &type) || type < 0 || type > 0xFF ||
            (argc == 3 && (!parse_int(argv[2], &v) || v < INT16_MIN || v > INT16_MAX))) {
            set_status(1, "usage: raw <type 0x00-0xFF> [int16 value]");
            return;
        }
        char text[40];
        snprintf(text, sizeof(text), "raw 0x%02lX %ld", type, v);
        send_logged((uint8_t)type, (int16_t)v, text);
    }
    else set_status(1, "unknown command '%s', type help", c);
}

/* Fill in results as the boat reports them. The boat only reports the last command it ran,
   so a command it reported past without ever reporting was lost on the way. */
static void match_results(void)
{
    uint8_t reported = (uint8_t)s_telem.last_cmd_seq;
    for (int i = 0; i < s_cmd_log_n; i++) {
        cmd_entry_t *e = &s_cmd_log[i];
        if (e->result != RES_PENDING) continue;
        uint8_t ahead = (uint8_t)(reported - e->seq);
        if (ahead == 0)        e->result = s_telem.last_cmd_result;
        else if (ahead < 128)  e->result = RES_MISSED;
    }
}

/* ─── Line editing ──────────────────────────────────────────────── */

static void history_load(int pos)
{
    s_history_pos = pos;
    if (pos < 0) { s_line_len = 0; s_line[0] = '\0'; return; }
    snprintf(s_line, sizeof(s_line), "%s", s_history[pos]);
    s_line_len = strlen(s_line);
}

static void on_key(char ch)
{
    unsigned char c = (unsigned char)ch;

    /* Arrow keys arrive as ESC [ A/B/C/D */
    if (s_esc == 1) { s_esc = (c == '[') ? 2 : 0; return; }
    if (s_esc == 2) {
        s_esc = 0;
        if (c == 'A' && s_history_pos + 1 < s_history_n) history_load(s_history_pos + 1);   /* up: older */
        else if (c == 'B' && s_history_pos >= 0)          history_load(s_history_pos - 1);   /* down: newer */
        return;
    }

    if (c == 0x1B) { s_esc = 1; return; }
    if (c == '\r' || c == '\n') {
        if (s_line_len == 0) return;
        memmove(s_history[1], s_history[0], sizeof(s_history[0]) * (HISTORY_LEN - 1));
        snprintf(s_history[0], sizeof(s_history[0]), "%s", s_line);
        if (s_history_n < HISTORY_LEN) s_history_n++;
        run_line(s_line);   /* tokenizes in place, so it goes last */
        history_load(-1);
        return;
    }
    if (c == 0x7F || c == 0x08) { if (s_line_len) s_line[--s_line_len] = '\0'; return; }
    if (c == 0x15) { history_load(-1); return; }   /* Ctrl-U clears the line */
    if (c >= 0x20 && c < 0x7F && s_line_len < LINE_MAX_LEN) {
        s_line[s_line_len++] = (char)c;
        s_line[s_line_len] = '\0';
    }
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
    match_results();
}

/* Scan the buffer for [AA BB seq 73 flags] + payload, skipping anything else */
static void parse(void)
{
    size_t i = 0;
    while (s_buf_len - i >= PKT_LEN) {
        const uint8_t *p = s_buf + i;
        if (p[0] == ADDR_GROUND && p[1] == ADDR_BOAT && p[3] == 1 + sizeof(LoRaTelemetry_t)) {
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
        printf("%-12s %-6s %-18s %s\n", "SUBSYSTEM", "", "VALIDITY", "DATA");

        printf("%-12s %s %-18s %.2f V   cells %.2f %.2f %.2f %.2f\n", "battery",
               validity_tag(t->battery_valid), validity_name(t->battery_valid),
               t->battery_voltage / 1000.0,
               t->cell_voltage[0] / 1000.0, t->cell_voltage[1] / 1000.0,
               t->cell_voltage[2] / 1000.0, t->cell_voltage[3] / 1000.0);

        printf("%-12s %s %-18s sail %+4d deg   mag %-5u %s %s %s\n", "encoder",
               validity_tag(t->encoder_valid), validity_name(t->encoder_valid),
               t->sail_ang_forward, t->sail_encoder_magnitude,
               (t->sail_encoder_status & 0x20) ? "MD" : "md",
               (t->sail_encoder_status & 0x10) ? "ML" : "ml",
               (t->sail_encoder_status & 0x08) ? "MH" : "mh");

        printf("%-12s %s %-18s apparent %+4d deg (%s)\n", "wind",
               validity_tag(t->wind_valid), validity_name(t->wind_valid),
               t->ang_wind, t->ang_wind > 0 ? "starboard" : t->ang_wind < 0 ? "port" : "ahead");

        printf("%-12s %s %-18s %.6f %.6f  alt %.1fm  sats %u fix %u  %02u:%02u:%02uZ\n", "gps",
               validity_tag(t->gps_valid), validity_name(t->gps_valid),
               t->latitude, t->longitude, t->altitude, t->satellites, t->fix_valid,
               t->utc_hours, t->utc_minutes, t->utc_seconds);

        printf("%-12s %s %-18s cal S%u G%u A%u M%u   temp %dC\n", "imu",
               validity_tag(t->imu_valid), validity_name(t->imu_valid),
               (t->imu_cal >> 6) & 3, (t->imu_cal >> 4) & 3,
               (t->imu_cal >> 2) & 3, t->imu_cal & 3, t->imu_temp);

        printf("%-12s %s %-18s hdg %3u  roll %+4d  pitch %+4d\n", "orientation",
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
        char hdg[8];
        if (t->desired_heading == 0xFFFF) snprintf(hdg, sizeof(hdg), "none");
        else snprintf(hdg, sizeof(hdg), "%3u", t->desired_heading);
        printf("desired hdg %s   segment %u   drift %+d m   uptime %02u:%02u:%02u\n",
               hdg, t->route_segment, t->drift_distance, up / 3600, (up / 60) % 60, up % 60);

        if (t->mode < SYSTEM_MODE_COUNT && MODES[t->mode].name)
            printf("\033[1mMODE  rudder %-4s  sail %-4s\033[0m  (%s, %u)\n",
                   ACT_NAME[MODES[t->mode].rudder], ACT_NAME[MODES[t->mode].sail], MODES[t->mode].name, t->mode);
        else
            printf("\033[1mMODE  unknown (%u)\033[0m\n", t->mode);
    }

    printf("%s\n", HR);
    printf("COMMANDS  sent %u   heartbeat %s (%u sent)   next seq %u",
           s_cmds_sent, s_hb_on ? "on" : "\033[1;31mOFF\033[0m", s_hbs_sent, s_tx_seq);
    if (s_have_telem) printf("   boat last ran seq %u -> %s", (uint8_t)t->last_cmd_seq, result_name(t->last_cmd_result));
    printf("\n");
    for (int i = 0; i < CMD_LOG_LEN; i++) {
        if (i >= s_cmd_log_n) { printf("\n"); continue; }
        const cmd_entry_t *e = &s_cmd_log[i];
        const char *col = e->result == CMD_RESULT_OK ? "\033[32m" : e->result == RES_PENDING ? "\033[90m" : "\033[31m";
        printf("  %s  #%-3u %-28s %s%s\033[0m\n", e->when, e->seq, e->text, col, result_name(e->result));
    }

    if (s_show_help) {
        printf("%s\n", HR);
        printf("  mode <m>                 both actuators          m = auto | semi | man\n");
        printf("  mode -r <m> -s <m>       rudder and/or sail; one left out keeps its current mode\n");
        printf("  safe | standby           both actuators out of play (safe = emergency stop)\n");
        printf("  rsang <deg>  ssang <deg> rudder / tail servo, -%d..%d       needs that side in man\n", LIMIT_SERVO_DEG, LIMIT_SERVO_DEG);
        printf("  head <deg>               heading for the rudder PID, 0..%d  needs rudder in semi\n", LIMIT_HEADING_MAX);
        printf("  sang <deg>               sail angle for the sail PID, -%d..%d  needs sail in semi\n", LIMIT_SAIL_PID_DEG, LIMIT_SAIL_PID_DEG);
        printf("  hb on|off                heartbeat every %.0f s (keeps man/semi from timing out)\n", HB_PERIOD_S);
        printf("  raw <type> [value]       any command type, e.g. raw 0x4F\n");
        printf("  up/down history   Ctrl-U clear line   Ctrl-C quit\n");
    }

    printf("%s\n", HR);
    printf("%s%s\033[0m\n", s_status_err ? "\033[31m" : "\033[90m", s_status[0] ? s_status : "type help for commands");
    printf("> %s\033[?25h", s_line);   /* leave the cursor at the end of the typed line */
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
    s_serial_fd = fd;

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
        struct timeval tv = { 0, 250000 };   /* redraw at least 4x/s so link age ticks and heartbeats go out */

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
            while (read(STDIN_FILENO, &c, 1) == 1) on_key(c);
        }

        /* Any command resets the boat's link timer, so a heartbeat only goes out after a quiet second */
        if (s_hb_on && (s_last_send == 0 || now_s() - s_last_send >= HB_PERIOD_S)) send_heartbeat();

        draw(port);
    }

    close(fd);
    return 0;
}
