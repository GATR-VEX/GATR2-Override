// control_board_source.h
// The control board: each tick, the newest frame it streamed, each record to
// the decoder for its sensor type. Records with no decoder are counted in
// SensorMap::board. A Smart Port that does not open is tried again every
// kRetryReads reads.

#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "locagatr/link.h"
#include "locagatr/sources/source.h"
#include "locagatr/sources/decoders/encoder_count_decoder.h"
#include "locagatr/sources/decoders/record_decoder.h"

namespace locagatr
{

class ControlBoardSource : public Source {
  public:
    static constexpr uint32_t kRetryReads = 50; // about 0.5 s at 100 Hz

    ControlBoardSource() : decoders_{&encoder_counts_} {}
    explicit ControlBoardSource(std::vector<const RecordDecoder*> decoders)
        : decoders_(std::move(decoders)) {}
    ControlBoardSource(const ControlBoardSource&)            = delete;
    ControlBoardSource& operator=(const ControlBoardSource&) = delete;

    const char* name() const override { return "control board"; }
    bool        begin(const Config& config) override;
    void        read(SensorMap& sensors) override;

    // Part of read(), for tests.
    void decode(const Poll& poll, const LinkStats& stats, SensorMap& sensors);

    // Part of begin(), for tests.
    bool buildTable();

  private:
    bool open();

    EncoderCountDecoder                   encoder_counts_;
    std::vector<const RecordDecoder*>     decoders_;
    std::array<const RecordDecoder*, 256> by_type_{};
    std::optional<Link>                   link_;
    uint8_t                               port_      = 0;
    int32_t                               baud_      = 0;
    Timing                                timing_;
    uint32_t                              retry_in_  = 0; // reads until the next open
    uint32_t                              unhandled_ = 0;
    uint8_t                               last_type_ = 0;
    uint8_t                               last_port_ = 0;
};

} // namespace locagatr
