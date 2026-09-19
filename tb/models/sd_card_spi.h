// tb/models/sd_card_spi.h — host-side functional SD card model (SPI mode),
// backed by a real disk-image file.
//
// WHY THIS FILE EXISTS
//   Three testbenches already carried their own inline `SdCard` struct:
//     * tb/tb_sd_boot.cpp        — full init sequence (CMD0/8/55+41/58/59/
//                                  9/6) + CMD17/CMD18/CMD12, synthetic data
//     * tb/tb_scsi_sd_e2e.cpp    — CMD17/18/12 + the write path (CMD24 /
//                                  CMD25 / 0xFC / 0xFD / 0xE5 / busy),
//                                  no init sequence
//     * tb/tb_sd_ctrl.cpp, tb/tb_sd_provision.cpp, tb/tb_pram_sd.cpp
//   None of them were reusable from another harness and none of them could
//   be backed by an actual disk image, so tb_fpga_top_rom.cpp — the ONE
//   simulation that runs the real ROM through the real SoC — simply tied
//   sd_miso high and had zero coverage of the path that boots the machine.
//
//   This header is the union of the tb_sd_boot init model and the
//   tb_scsi_sd_e2e transport model, with file-backed storage bolted on.
//   Behaviour of the individual commands is copied from those two models
//   rather than re-derived, so it stays bit-compatible with what the
//   existing unit tbs already prove sd_ctrl.v / boot_fsm.v accept.
//
// PROTOCOL CONTRACT (see rtl/board/sd_ctrl.v's header for the RTL side)
//   * SPI mode 0, MSB first, 1-bit.  MISO is launched on the SCK falling
//     edge, MOSI is sampled on the SCK rising edge (rtl/board/sd_spi.v
//     samples its 2-flop-synchronised MISO at the end of the HI phase,
//     i.e. effectively at the rising edge).
//   * Commands are 6-byte frames whose first byte matches 01xxxxxx.
//   * R1 is a single byte; R3 (CMD58) and R7 (CMD8) are R1 + 4 trailing
//     bytes.  Read data blocks are 0xFE + 512 + CRC16.  CMD18 streams
//     blocks back-to-back until CMD12.  Writes take 0xFE (CMD24) or 0xFC
//     (CMD25) + 512 + CRC16 and are answered with 0xE5 then busy (0x00)
//     bytes; 0xFD ends a CMD25 stream.
//   * Every response is preceded by at least one 0xFF stuff byte, which is
//     what sd_ctrl.v's R1 poll and token hunt expect.
//
// STORAGE LAYOUT
//   The SoC reserves SD LBA 0..8191 (4 MiB) for the boot-ROM image and the
//   PRAM sector (8191); the SCSI virtual HDD starts at SD LBA 8192
//   (rtl/soc/sd_scsi_lba_mapper.v RESERVED_LBAS, rtl/soc/boot_fsm.v
//   SD_RESERVED_LBAS).  Two ways to attach an image:
//     attach_raw(path)      — path is a whole-card image; file offset 0 is
//                             SD LBA 0.
//     attach_hdd(path)      — path is just the Mac HDD image; file offset 0
//                             is SD LBA 8192.
//   Unbacked LBAs read back as zeros.  Writes go to an in-memory overlay
//   and never touch the image on disk unless writeback is enabled, so a
//   simulation can never corrupt the user's disk image.
//
// USAGE (pin level, one call per core_clk after eval)
//   SdCardSpi sd;
//   sd.attach_hdd("disk.img");
//   ...
//   dut->sd_miso = sd.tick(dut->sd_clk, dut->sd_mosi, dut->sd_cs_n);

#ifndef TB_MODELS_SD_CARD_SPI_H
#define TB_MODELS_SD_CARD_SPI_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

class SdCardSpi {
public:
    // ── Configuration ────────────────────────────────────────────────
    // SD LBA at which the attached image's byte 0 lives.
    static const uint32_t RESERVED_LBAS = 8192u;

    bool     trace          = false;   // per-command stderr trace
    bool     writeback      = false;   // persist writes into the image file
    uint64_t cmd_count      = 0;
    uint64_t read_blocks    = 0;
    uint64_t write_blocks   = 0;
    uint64_t bytes_clocked  = 0;

    ~SdCardSpi() { if (fp_) std::fclose(fp_); }

    // Attach a whole-card image (file offset 0 == SD LBA 0).
    bool attach_raw(const std::string& path) { return attach(path, 0u); }
    // Attach a bare HDD image (file offset 0 == SD LBA RESERVED_LBAS).
    bool attach_hdd(const std::string& path) { return attach(path, RESERVED_LBAS); }

    bool attach(const std::string& path, uint32_t base_lba) {
        if (fp_) { std::fclose(fp_); fp_ = nullptr; }
        fp_ = std::fopen(path.c_str(), writeback ? "r+b" : "rb");
        if (!fp_) {
            std::fprintf(stderr, "[sd] cannot open image '%s'\n", path.c_str());
            return false;
        }
        std::fseek(fp_, 0, SEEK_END);
        long sz = std::ftell(fp_);
        std::fseek(fp_, 0, SEEK_SET);
        img_bytes_ = (sz > 0) ? (uint64_t)sz : 0;
        base_lba_  = base_lba;
        path_      = path;
        std::fprintf(stderr,
            "[sd] attached '%s' (%llu bytes, %llu sectors) at SD LBA %u\n",
            path.c_str(), (unsigned long long)img_bytes_,
            (unsigned long long)(img_bytes_ / 512u), (unsigned)base_lba);
        return true;
    }

    bool attached() const { return fp_ != nullptr; }

    // ── Pin-level tick ───────────────────────────────────────────────
    // Call once per core_clk, AFTER the DUT has been evaluated for that
    // posedge.  Returns the MISO level to drive into the DUT for the next
    // posedge.
    uint8_t tick(uint8_t sck, uint8_t mosi, uint8_t cs_n) {
        const bool clk_now  = (sck  != 0);
        const bool cs_n_now = (cs_n != 0);

        // CS falling edge: restart the per-byte bit counters.  Launch one
        // idle-high byte so bit 7 of the first reply is already stable
        // before the first rising SCK edge (SPI mode 0).
        if (last_cs_n_ && !cs_n_now) {
            rise_count_     = 0;
            mosi_byte_      = 0;
            miso_byte_      = 0xFF;
            miso_bits_left_ = 8;
        }
        last_cs_n_ = cs_n_now;

        if (cs_n_now) {
            last_clk_ = clk_now;
            return 1;                    // card releases DO; pull-up wins
        }

        const bool rising  = (!last_clk_ &&  clk_now);
        const bool falling = ( last_clk_ && !clk_now);

        if (rising) {
            mosi_byte_ = (uint8_t)((mosi_byte_ << 1) | (mosi & 1));
            if (++rise_count_ == 8) {
                bytes_clocked++;
                observe_mosi(mosi_byte_);
                rise_count_ = 0;
                mosi_byte_  = 0;
            }
        }

        if (falling) {
            if (miso_bits_left_ <= 1) {
                miso_byte_      = pop_miso();
                miso_bits_left_ = 8;
            } else {
                miso_byte_ = (uint8_t)((miso_byte_ << 1) | 1);
                miso_bits_left_--;
            }
        }

        last_clk_ = clk_now;
        return (uint8_t)((miso_byte_ >> 7) & 1);
    }

private:
    // ── Backing store ────────────────────────────────────────────────
    std::FILE*  fp_        = nullptr;
    std::string path_;
    uint64_t    img_bytes_ = 0;
    uint32_t    base_lba_  = 0;
    std::map<uint32_t, std::vector<uint8_t> > overlay_;   // written sectors

    void read_sector(uint32_t lba, uint8_t* out) {
        std::map<uint32_t, std::vector<uint8_t> >::iterator it =
            overlay_.find(lba);
        if (it != overlay_.end()) {
            std::memcpy(out, &it->second[0], 512);
            return;
        }
        std::memset(out, 0, 512);
        if (!fp_ || lba < base_lba_) return;
        const uint64_t off = (uint64_t)(lba - base_lba_) * 512ull;
        if (off >= img_bytes_) return;
        if (std::fseek(fp_, (long)off, SEEK_SET) != 0) return;
        std::fread(out, 1, 512, fp_);
    }

    void write_sector(uint32_t lba, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> d = data;
        d.resize(512, 0);
        overlay_[lba] = d;
        if (writeback && fp_ && lba >= base_lba_) {
            const uint64_t off = (uint64_t)(lba - base_lba_) * 512ull;
            if (off < img_bytes_ && std::fseek(fp_, (long)off, SEEK_SET) == 0) {
                std::fwrite(&d[0], 1, 512, fp_);
                std::fflush(fp_);
            }
        }
    }

    static uint16_t crc16_ccitt(const uint8_t* data, size_t n) {
        uint16_t crc = 0x0000;
        for (size_t i = 0; i < n; i++) {
            crc ^= (uint16_t)((uint16_t)data[i] << 8);
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                     : (uint16_t)(crc << 1);
        }
        return crc;
    }

    // ── MISO queue ───────────────────────────────────────────────────
    std::deque<uint8_t> miso_;
    void push(uint8_t b) { miso_.push_back(b); }
    void push(const uint8_t* v, size_t n) {
        for (size_t i = 0; i < n; i++) miso_.push_back(v[i]);
    }

    void queue_read_block(uint32_t lba) {
        uint8_t buf[512];
        read_sector(lba, buf);
        push(0xFF);                       // token-hunt stuff byte
        push(0xFE);                       // start-of-block token
        push(buf, 512);
        const uint16_t crc = crc16_ccitt(buf, 512);
        push((uint8_t)(crc >> 8));
        push((uint8_t)(crc & 0xFF));
        read_blocks++;
    }

    // ── Command / data-phase state ───────────────────────────────────
    std::vector<uint8_t> frame_;
    bool     in_frame_    = false;

    bool     read_active_ = false;        // CMD18 stream in flight
    uint32_t read_lba_    = 0;
    int      acmd41_seen_ = 0;

    enum WState { W_IDLE, W_AWAIT_TOKEN, W_RECV_DATA, W_RECV_CRC, W_BUSY };
    WState   wstate_      = W_IDLE;
    int      w_data_cnt_  = 0;
    int      w_crc_cnt_   = 0;
    int      w_busy_      = 0;
    uint32_t w_lba_       = 0;
    uint8_t  w_cmd_       = 0;
    bool     w_multi_     = false;
    std::vector<uint8_t> w_block_;

    uint8_t pop_miso() {
        // CMD18: top the queue up with the next block so the host never
        // sees a gap longer than the spec's 0xFF token wait.
        if (read_active_ && miso_.empty()) {
            queue_read_block(read_lba_);
            read_lba_++;
        }
        if (wstate_ == W_BUSY && miso_.empty()) {
            if (w_busy_ > 0) { w_busy_--; return 0x00; }
            wstate_ = w_multi_ ? W_AWAIT_TOKEN : W_IDLE;
        }
        if (miso_.empty()) return 0xFF;
        uint8_t b = miso_.front();
        miso_.pop_front();
        return b;
    }

    void handle_frame() {
        const uint8_t cmd = (uint8_t)(frame_[0] & 0x3F);
        const uint32_t arg = ((uint32_t)frame_[1] << 24) |
                             ((uint32_t)frame_[2] << 16) |
                             ((uint32_t)frame_[3] <<  8) |
                             ((uint32_t)frame_[4]);
        cmd_count++;
        if (trace)
            std::fprintf(stderr, "[sd] CMD%u arg=0x%08x\n",
                         (unsigned)cmd, (unsigned)arg);

        switch (cmd) {
            case 0:   // GO_IDLE_STATE — R1 = 0x01 (idle)
                push(0xFF); push(0x01);
                break;
            case 8: { // SEND_IF_COND — R7: R1 + 4-byte echo
                static const uint8_t r[] = {0xFF, 0x01, 0x00, 0x00, 0x01, 0xAA};
                push(r, sizeof r);
                break;
            }
            case 55:  // APP_CMD
                push(0xFF); push(0x01);
                break;
            case 41:  // ACMD41 — idle once, then ready
                push(0xFF);
                push((uint8_t)(acmd41_seen_ >= 1 ? 0x00 : 0x01));
                acmd41_seen_++;
                break;
            case 58: { // READ_OCR — R3: R1 + OCR, CCS=1 (SDHC)
                static const uint8_t r[] = {0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00};
                push(r, sizeof r);
                break;
            }
            case 59:  // CRC_ON_OFF — accepted
                push(0xFF); push(0x00);
                break;
            case 16:  // SET_BLOCKLEN — accepted (SDHC ignores it)
                push(0xFF); push(0x00);
                break;
            case 9: { // SEND_CSD — CSD v2.0, C_SIZE sized from the image
                uint8_t csd[16];
                std::memset(csd, 0, sizeof csd);
                csd[0]  = 0x40;                 // CSD_STRUCTURE = 1 (v2.0)
                csd[1]  = 0x0E; csd[2] = 0x00; csd[3] = 0x32;
                csd[4]  = 0x5B; csd[5] = 0x59;
                // C_SIZE (bits 69:48) = capacity/512KiB - 1; report a card
                // comfortably larger than the image.
                uint64_t sectors = (img_bytes_ / 512ull) + base_lba_ + 1024ull;
                uint32_t csize = (uint32_t)(sectors / 1024ull);
                if (csize == 0) csize = 1;
                csize -= 1;
                if (csize > 0x3FFFFF) csize = 0x3FFFFF;
                csd[7]  = (uint8_t)((csize >> 16) & 0x3F);
                csd[8]  = (uint8_t)((csize >>  8) & 0xFF);
                csd[9]  = (uint8_t)( csize        & 0xFF);
                csd[10] = 0x7F; csd[11] = 0x80; csd[12] = 0x0A; csd[13] = 0x40;
                push(0xFF); push(0x00); push(0xFF); push(0xFE);
                push(csd, 16);
                const uint16_t crc = crc16_ccitt(csd, 16);
                push((uint8_t)(crc >> 8)); push((uint8_t)(crc & 0xFF));
                break;
            }
            case 6:   // SWITCH_FUNC — refuse, matching the cards the board
                      // ships with (boot_fsm falls back to 25 MHz).
                push(0xFF); push(0x04);
                break;
            case 17:  // READ_SINGLE_BLOCK
                push(0xFF); push(0x00);
                queue_read_block(arg);
                break;
            case 18:  // READ_MULTIPLE_BLOCK
                push(0xFF); push(0x00);
                queue_read_block(arg);
                read_active_ = true;
                read_lba_    = arg + 1;
                break;
            case 12:  // STOP_TRANSMISSION
                read_active_ = false;
                miso_.clear();
                push(0xFF);               // stuff byte
                push(0x00);               // R1 = OK
                push(0xFF);
                break;
            case 24:  // WRITE_BLOCK
            case 25:  // WRITE_MULTIPLE_BLOCK
                push(0xFF); push(0x00);
                wstate_     = W_AWAIT_TOKEN;
                w_data_cnt_ = 0;
                w_crc_cnt_  = 0;
                w_lba_      = arg;
                w_cmd_      = cmd;
                w_multi_    = (cmd == 25);
                w_block_.clear();
                break;
            default:
                push(0xFF); push(0x04);   // illegal command
                break;
        }
    }

    void observe_mosi(uint8_t b) {
        switch (wstate_) {
            case W_AWAIT_TOKEN:
                if (w_cmd_ == 24 && b == 0xFE) {
                    wstate_ = W_RECV_DATA; w_data_cnt_ = 0; w_block_.clear();
                    return;
                }
                if (w_cmd_ == 25 && b == 0xFC) {
                    wstate_ = W_RECV_DATA; w_data_cnt_ = 0; w_block_.clear();
                    return;
                }
                if (w_cmd_ == 25 && b == 0xFD) {   // stop-tran
                    w_busy_  = 4;
                    wstate_  = W_BUSY;
                    w_cmd_   = 0;
                    w_multi_ = false;
                    return;
                }
                return;                            // gap bytes
            case W_RECV_DATA:
                w_block_.push_back(b);
                if (++w_data_cnt_ == 512) { wstate_ = W_RECV_CRC; w_crc_cnt_ = 0; }
                return;
            case W_RECV_CRC:
                if (++w_crc_cnt_ == 2) {
                    push(0xE5);                    // data accepted
                    write_sector(w_lba_, w_block_);
                    write_blocks++;
                    if (trace)
                        std::fprintf(stderr, "[sd] wrote LBA %u\n",
                                     (unsigned)w_lba_);
                    if (w_multi_) w_lba_++;
                    w_busy_ = 4;
                    wstate_ = W_BUSY;
                }
                return;
            case W_BUSY:
                return;                            // busy poll bytes
            case W_IDLE:
            default:
                break;
        }

        if (!in_frame_) {
            if ((b & 0xC0) == 0x40) {              // 01xxxxxx = command
                frame_.clear();
                frame_.push_back(b);
                in_frame_ = true;
            }
        } else {
            frame_.push_back(b);
            if (frame_.size() == 6) {
                handle_frame();
                in_frame_ = false;
            }
        }
    }

    // ── SPI bit state ────────────────────────────────────────────────
    uint8_t mosi_byte_      = 0;
    int     rise_count_     = 0;
    uint8_t miso_byte_      = 0xFF;
    int     miso_bits_left_ = 0;
    bool    last_clk_       = false;
    bool    last_cs_n_      = true;
};

#endif  // TB_MODELS_SD_CARD_SPI_H
