/* Serial console command parser (HELP, SHOW, WATCH, RUN, STOP, CLEAR, REBOOT, CFG/RULES/BRIDGE as hex Protobuf);
 * hands configuration to the same handlers the MQTT path uses. */
#include "repl.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static int reply(char *out, size_t cap, int rc, const char *msg)
{
    snprintf(out, cap, "%s", msg);
    return rc;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* decode hex into buf; returns bytes or -1 */
static int decode_hex(const char *s, uint8_t *buf, size_t cap)
{
    size_t n = 0;
    while (*s && !isspace((unsigned char)*s)) {
        const int hi = hexval(s[0]);
        const int lo = hi < 0 ? -1 : hexval(s[1]);
        if (hi < 0 || lo < 0 || n >= cap) return -1;
        buf[n++] = (uint8_t)(hi << 4 | lo);
        s += 2;
    }
    while (*s) {
        if (!isspace((unsigned char)*s)) return -1;
        s++;
    }
    return n ? (int)n : -1;
}

static bool word_is(const char *w, size_t n, const char *cmd)
{
    if (strlen(cmd) != n) return false;
    for (size_t i = 0; i < n; i++) {
        if (toupper((unsigned char)w[i]) != cmd[i]) return false;
    }
    return true;
}

int repl_exec(const repl_ops_t *ops, const char *line, char *out, size_t cap)
{
    static uint8_t payload[REPL_MAX_PAYLOAD];
    while (isspace((unsigned char)*line)) line++;
    if (*line == '\0') return reply(out, cap, 0, "");
    const char *w = line;
    while (*line && !isspace((unsigned char)*line)) line++;
    const size_t n = (size_t)(line - w);
    while (isspace((unsigned char)*line)) line++;
    const char *arg = line;

    if (word_is(w, n, "HELP")) {
        return reply(out, cap, 0, "RUN | STOP | CLEAR | SHOW | REBOOT | WATCH <id|ALL|OFF> | CFG <hex> | RULES <hex> | BRIDGE <hex>");
    }
    if (word_is(w, n, "RUN") || word_is(w, n, "STOP") || word_is(w, n, "CLEAR")) {
        if (*arg) return reply(out, cap, -1, "ERR no arguments expected");
        const bool ok = word_is(w, n, "RUN") ? ops->run(ops->ctx) : word_is(w, n, "STOP") ? ops->stop(ops->ctx) : ops->clear(ops->ctx);
        return ok ? reply(out, cap, 0, "OK") : reply(out, cap, -2, "ERR command failed");
    }
    if (word_is(w, n, "SHOW")) {
        const size_t k = ops->show(ops->ctx, out, cap);
        (void)k;
        return 0;
    }
    if (word_is(w, n, "REBOOT")) {
        if (!ops->reboot) return reply(out, cap, -4, "ERR reboot not supported on this platform");
        snprintf(out, cap, "OK rebooting");
        ops->reboot(ops->ctx); /* does not return on a real node */
        return 0;
    }
    if (word_is(w, n, "WATCH")) {
        if (!ops->watch) return reply(out, cap, -4, "ERR watch not supported");
        int which = -1;
        if (!strcasecmp(arg, "ALL")) which = 256;
        else if (!strcasecmp(arg, "OFF")) which = 0;
        else if (*arg && strspn(arg, "0123456789") == strlen(arg) && atoi(arg) >= 1 && atoi(arg) <= 255) which = atoi(arg);
        if (which < 0) return reply(out, cap, -1, "ERR usage: WATCH <sensor id 1..255 | ALL | OFF>");
        ops->watch(ops->ctx, which);
        return reply(out, cap, 0, which == 0 ? "OK watch off" : "OK watching (one line per sensor per second)");
    }
    void (*handler)(void *, const uint8_t *, size_t) = NULL;
    if (word_is(w, n, "CFG")) handler = ops->apply_config;
    else if (word_is(w, n, "RULES")) handler = ops->apply_rules;
    else if (word_is(w, n, "BRIDGE")) handler = ops->apply_bridge;
    if (handler) {
        if (!*arg) return reply(out, cap, -1, "ERR missing hex payload");
        const int len = decode_hex(arg, payload, sizeof(payload));
        if (len < 0) return reply(out, cap, -1, "ERR bad or oversized hex payload (max 4096 bytes; use MQTT for larger)");
        handler(ops->ctx, payload, (size_t)len); /* result arrives as the same ACK the MQTT path produces */
        return reply(out, cap, 0, "OK submitted");
    }
    return reply(out, cap, -3, "ERR unknown command (try HELP)");
}
