#include "periph_event_log.h"

void PeriphEventLog::set_log_path(const std::string& path) {
    log_path_ = path;
}

void PeriphEventLog::set_detail_limit(uint64_t limit) {
    detail_limit_ = limit;
}

void PeriphEventLog::set_summary_enabled(bool enabled) {
    summary_enabled_ = enabled;
}

void PeriphEventLog::set_category_filter(const std::vector<std::string>& categories) {
    category_filter_.clear();
    for (const auto& category : categories) {
        if (!category.empty())
            category_filter_.insert(category);
    }
}

void PeriphEventLog::watch_category(const char* category) {
    if (!category) return;
    if (!category_allowed(category)) return;
    category_stats_[category];
}

bool PeriphEventLog::enabled() const {
    return summary_enabled_ || !log_path_.empty();
}

bool PeriphEventLog::open() {
    if (log_path_.empty()) return true;
    fp_ = std::fopen(log_path_.c_str(), "w");
    if (!fp_) return false;
    std::fprintf(fp_,
        "# peripheral model event log\n"
        "# fields: cycle committed pc category event addr value detail\n");
    if (!category_filter_.empty()) {
        std::fprintf(fp_, "# category_filter=");
        bool first = true;
        for (const auto& category : category_filter_) {
            std::fprintf(fp_, "%s%s", first ? "" : ",", category.c_str());
            first = false;
        }
        std::fprintf(fp_, "\n");
    }
    return true;
}

void PeriphEventLog::close() {
    if (fp_) {
        // Make the on-disk log self-contained: even if detail logging was capped
        // (or disabled via limit=0), always append the end-of-run summary so a
        // single artifact captures "what happened" without relying on stderr.
        dump_summary(fp_);
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

void PeriphEventLog::update_stats(std::map<std::string, Stats>& stats,
                                  const std::string& key,
                                  const PeriphEventContext& ctx,
                                  const char* event,
                                  uint32_t addr,
                                  uint32_t value,
                                  const std::string& detail) {
    Stats& s = stats[key];
    s.count++;
    Snapshot snap;
    snap.valid = true;
    snap.cycle = ctx.cycle;
    snap.committed = ctx.committed;
    snap.pc = ctx.pc;
    snap.addr = addr;
    snap.value = value;
    snap.event = event ? event : "";
    snap.detail = detail;
    if (!s.first.valid) s.first = snap;
    s.last = snap;
}

void PeriphEventLog::record(const PeriphEventContext& ctx,
                            const char* category,
                            const char* event,
                            uint32_t addr,
                            uint32_t value,
                            const std::string& detail) {
    if (!enabled() || !category || !event) return;
    if (!category_allowed(category)) return;

    update_stats(category_stats_, category, ctx, event, addr, value, detail);
    update_stats(event_stats_, std::string(category) + "." + event,
                 ctx, event, addr, value, detail);

    if (!fp_ || detail_limit_ == 0) return;
    if (detail_emitted_ < detail_limit_) {
        std::fprintf(fp_,
            "cycle=%llu committed=%llu pc=0x%08x category=%s event=%s "
            "addr=0x%08x value=0x%08x",
            (unsigned long long)ctx.cycle,
            (unsigned long long)ctx.committed,
            ctx.pc,
            category,
            event,
            addr,
            value);
        if (!detail.empty())
            std::fprintf(fp_, " detail=%s", detail.c_str());
        std::fprintf(fp_, "\n");
        detail_emitted_++;
    } else if (!detail_suppressed_) {
        std::fprintf(fp_,
            "# detail limit reached at %llu events; continuing summary only\n",
            (unsigned long long)detail_limit_);
        detail_suppressed_ = true;
    }
}

void PeriphEventLog::dump_summary(FILE* out) const {
    if (!enabled() || !out) return;

    auto dump_snapshot = [](FILE* f, const char* label,
                            const Snapshot& s) {
        if (!s.valid) {
            std::fprintf(f, " %s=none", label);
            return;
        }
        std::fprintf(f,
            " %s=%s@cycle:%llu,committed:%llu,pc:0x%08x,addr:0x%08x,value:0x%08x",
            label,
            s.event.c_str(),
            (unsigned long long)s.cycle,
            (unsigned long long)s.committed,
            s.pc,
            s.addr,
            s.value);
        if (!s.detail.empty())
            std::fprintf(f, ",detail:%s", s.detail.c_str());
    };

    std::fprintf(out, "\n-------- peripheral model event summary --------\n");
    if (!category_filter_.empty()) {
        std::fprintf(out, "  category_filter=");
        bool first = true;
        for (const auto& category : category_filter_) {
            std::fprintf(out, "%s%s", first ? "" : ",", category.c_str());
            first = false;
        }
        std::fprintf(out, "\n");
    }
    for (const auto& kv : category_stats_) {
        const Stats& s = kv.second;
        std::fprintf(out, "  category=%s count=%llu",
                     kv.first.c_str(),
                     (unsigned long long)s.count);
        dump_snapshot(out, "first", s.first);
        dump_snapshot(out, "last", s.last);
        std::fprintf(out, "\n");
    }

    std::fprintf(out, "  event breakdown:\n");
    bool any_event = false;
    for (const auto& kv : event_stats_) {
        if (kv.second.count == 0) continue;
        any_event = true;
        std::fprintf(out, "    %s x%llu\n",
                     kv.first.c_str(),
                     (unsigned long long)kv.second.count);
    }
    if (!any_event)
        std::fprintf(out, "    none\n");
    if (!log_path_.empty()) {
        std::fprintf(out,
            "  detail_log=%s emitted=%llu limit=%llu%s\n",
            log_path_.c_str(),
            (unsigned long long)detail_emitted_,
            (unsigned long long)detail_limit_,
            detail_suppressed_ ? " suppressed=yes" : "");
    }
    std::fprintf(out, "-----------------------------------------------\n");
}

bool PeriphEventLog::category_allowed(const char* category) const {
    if (category_filter_.empty()) return true;
    return category && category_filter_.find(category) != category_filter_.end();
}
