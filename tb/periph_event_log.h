#ifndef TB_PERIPH_EVENT_LOG_H
#define TB_PERIPH_EVENT_LOG_H

#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

struct PeriphEventContext {
    uint64_t cycle = 0;
    uint64_t committed = 0;
    uint32_t pc = 0;
};

class PeriphEventLog {
public:
    void set_log_path(const std::string& path);
    void set_detail_limit(uint64_t limit);
    void set_summary_enabled(bool enabled);
    void set_category_filter(const std::vector<std::string>& categories);
    void watch_category(const char* category);

    bool enabled() const;
    bool open();
    void close();

    void record(const PeriphEventContext& ctx,
                const char* category,
                const char* event,
                uint32_t addr,
                uint32_t value,
                const std::string& detail = std::string());

    void dump_summary(FILE* out) const;

private:
    struct Snapshot {
        bool valid = false;
        uint64_t cycle = 0;
        uint64_t committed = 0;
        uint32_t pc = 0;
        uint32_t addr = 0;
        uint32_t value = 0;
        std::string event;
        std::string detail;
    };

    struct Stats {
        uint64_t count = 0;
        Snapshot first;
        Snapshot last;
    };

    void update_stats(std::map<std::string, Stats>& stats,
                      const std::string& key,
                      const PeriphEventContext& ctx,
                      const char* event,
                      uint32_t addr,
                      uint32_t value,
                      const std::string& detail);
    bool category_allowed(const char* category) const;

    std::string log_path_;
    FILE* fp_ = nullptr;
    uint64_t detail_limit_ = 256;
    uint64_t detail_emitted_ = 0;
    bool detail_suppressed_ = false;
    bool summary_enabled_ = false;
    std::set<std::string> category_filter_;

    std::map<std::string, Stats> category_stats_;
    std::map<std::string, Stats> event_stats_;
};

#endif
