/* Host test: every message of the demo copy-paste pack passes the firmware's own validators.
 *
 * Usage: test_demo_pack <dir>   (the directory holds <name>.config.bin / .rules.bin / .bridge.bin files written by run.py)
 * Each file is decoded with nanopb exactly as the node decodes it, then checked with cm_validate_apply (sensor profiles),
 * re_validate (rule sets, with the firmware's pin policy) or bridge_validate (bridge sources, as seen by node1).
 */
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aai16.pb.h"
#include "bridge.h"
#include "config_manager.h"
#include "pb_decode.h"
#include "rule_engine.h"

static uint8_t buf[32768];

static size_t slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("  FAIL cannot open %s\n", path);
        exit(2);
    }
    const size_t n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    return n;
}

static bool pin_ok(void *ctx, uint32_t pin)
{
    (void)ctx;
    return cm_pin_allowed(pin);
}

static bool ends_with(const char *s, const char *suffix)
{
    const size_t a = strlen(s), b = strlen(suffix);
    return a >= b && strcmp(s + a - b, suffix) == 0;
}

static int check_file(const char *dir, const char *name)
{
    char path[1024];
    snprintf(path, sizeof path, "%s/%s", dir, name);
    const size_t n = slurp(path);
    pb_istream_t is = pb_istream_from_buffer(buf, n);

    if (ends_with(name, ".config.bin")) {
        static aai16_ConfigApply c;
        memset(&c, 0, sizeof c);
        if (!pb_decode(&is, aai16_ConfigApply_fields, &c)) {
            printf("  FAIL %s: does not decode as ConfigApply\n", name);
            return 1;
        }
        cm_error_t e;
        const bool ok = cm_validate_apply(&c, &e);
        printf("  %s %s: %u profile(s)", ok ? "ok  " : "FAIL", name, (unsigned)c.profiles_count);
        if (!ok) printf(" (reason %d, sensor %u, profile %u, detail %u)", (int)e.reason, (unsigned)e.sensor_id,
                        (unsigned)e.profile_index, (unsigned)e.detail);
        printf("\n");
        return ok ? 0 : 1;
    }
    if (ends_with(name, ".rules.bin")) {
        static aai16_RuleSetApply s;
        memset(&s, 0, sizeof s);
        if (!pb_decode(&is, aai16_RuleSetApply_fields, &s)) {
            printf("  FAIL %s: does not decode as RuleSetApply\n", name);
            return 1;
        }
        re_error_t e;
        const bool ok = re_validate(&s, pin_ok, NULL, &e);
        printf("  %s %s: %u rule(s)\n", ok ? "ok  " : "FAIL", name, (unsigned)s.rules_count);
        return ok ? 0 : 1;
    }
    if (ends_with(name, ".bridge.bin")) {
        static aai16_BridgeApply s;
        memset(&s, 0, sizeof s);
        if (!pb_decode(&is, aai16_BridgeApply_fields, &s)) {
            printf("  FAIL %s: does not decode as BridgeApply\n", name);
            return 1;
        }
        static bridge_t b;
        bridge_init(&b, "node1");
        aai16_Reason reason;
        uint32_t index;
        const bool ok = bridge_validate(&b, &s, &reason, &index);
        printf("  %s %s: %u source(s)\n", ok ? "ok  " : "FAIL", name, (unsigned)s.sources_count);
        return ok ? 0 : 1;
    }
    return 0; /* other files are checked in Python */
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: test_demo_pack <dir>\n");
        return 2;
    }
    DIR *d = opendir(argv[1]);
    if (!d) {
        printf("FAIL cannot open directory %s\n", argv[1]);
        return 2;
    }
    int failures = 0, checked = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (ends_with(e->d_name, ".config.bin") || ends_with(e->d_name, ".rules.bin") || ends_with(e->d_name, ".bridge.bin")) {
            failures += check_file(argv[1], e->d_name);
            checked++;
        }
    }
    closedir(d);
    printf("%d firmware-validator checks, %d failed\n", checked, failures);
    return failures ? 1 : 0;
}
