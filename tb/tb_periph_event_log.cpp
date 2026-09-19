#include "periph_event_log.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__    \
                      << ": " #cond << "\n";                                  \
            failures++;                                                        \
        }                                                                      \
    } while (0)

static bool writable_dir(const char* path) {
    struct stat st;
    return path && stat(path, &st) == 0 && S_ISDIR(st.st_mode) &&
           access(path, W_OK) == 0;
}

static std::string temp_log_path() {
    const char* root = writable_dir("/dev/shm") ? "/dev/shm" : "/tmp";
    std::ostringstream os;
    os << root << "/m68k-ooo-periph-event-log-test-" << getpid() << ".log";
    return os.str();
}

static std::string read_file(const std::string& path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static int count_detail_lines(const std::string& text) {
    int count = 0;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("cycle=", 0) == 0)
            count++;
    }
    return count;
}

static bool contains(const std::string& haystack, const char* needle) {
    return haystack.find(needle) != std::string::npos;
}

static void test_default_off() {
    PeriphEventLog log;
    CHECK(!log.enabled());

    PeriphEventContext ctx;
    log.record(ctx, "VIA1", "read", 0x50000000u, 0x12u);
    CHECK(!log.enabled());
}

static void test_limit_filter_and_summary() {
    const std::string path = temp_log_path();
    std::remove(path.c_str());

    PeriphEventLog log;
    log.set_log_path(path);
    log.set_detail_limit(2);
    log.set_category_filter({"VIA1", "SCSI"});
    log.watch_category("VIA1");
    log.watch_category("SCSI");
    log.watch_category("ADB");

    CHECK(log.enabled());
    CHECK(log.open());

    PeriphEventContext ctx;
    ctx.cycle = 10;
    ctx.committed = 1;
    ctx.pc = 0x4000008cu;
    log.record(ctx, "VIA1", "read", 0x50000000u, 0x80u, "reg=ORB");

    ctx.cycle = 11;
    log.record(ctx, "ADB", "sr_write", 0x50001400u, 0x01u, "filtered");

    ctx.cycle = 12;
    ctx.committed = 2;
    log.record(ctx, "SCSI", "read_data", 0x5000f000u, 0x00u, "off=0x0");

    ctx.cycle = 13;
    ctx.committed = 3;
    log.record(ctx, "VIA1", "write", 0x50000000u, 0x00u, "reg=ORB");

    ctx.cycle = 14;
    ctx.committed = 4;
    log.record(ctx, "VIA1", "ier_set", 0x50001c00u, 0x82u, "ifrbits=CA1");

    log.close();

    const std::string text = read_file(path);
    CHECK(count_detail_lines(text) == 2);
    CHECK(contains(text, "# detail limit reached at 2 events"));
    CHECK(contains(text, "category_filter=SCSI,VIA1"));
    CHECK(contains(text, "category=VIA1 count=3"));
    CHECK(contains(text, "category=SCSI count=1"));
    CHECK(!contains(text, "category=ADB"));
    CHECK(contains(text, "VIA1.read x1"));
    CHECK(contains(text, "VIA1.write x1"));
    CHECK(contains(text, "VIA1.ier_set x1"));
    CHECK(contains(text, "SCSI.read_data x1"));
    CHECK(contains(text, "detail_log="));
    CHECK(contains(text, "emitted=2 limit=2 suppressed=yes"));

    std::remove(path.c_str());
}

int main() {
    test_default_off();
    test_limit_filter_and_summary();

    if (failures != 0) {
        std::cerr << "tb_periph_event_log: FAIL failures=" << failures << "\n";
        return 1;
    }
    std::cout << "tb_periph_event_log: PASS\n";
    return 0;
}
