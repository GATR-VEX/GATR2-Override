// control_board_source.cpp

#include "locagatr/sources/control_board_source.h"

#include "locagatr/smart_port.h"

namespace locagatr
{

bool ControlBoardSource::buildTable() {
    by_type_.fill(nullptr);
    for (const RecordDecoder* decoder : decoders_) {
        const RecordDecoder*& slot = by_type_[decoder->sensorType()];
        if (slot != nullptr) {
            return false; // two decoders for one sensor type
        }
        slot = decoder;
    }
    return true;
}

bool ControlBoardSource::begin(const Config& config) {
    if (!buildTable() || config.robot.board_port < 1 || config.robot.board_port > 21) {
        return false;
    }
    port_            = config.robot.board_port;
    baud_            = config.robot.board_baud;
    timing_          = Timing{};
    timing_.stale_us = config.robot.board_stale_us;
    link_.reset();
    retry_in_ = 0;
    open();
    return true; // a port that did not open is tried again from read()
}

bool ControlBoardSource::open() {
    const std::optional<Io> io = openSmartPort(port_, baud_);
    if (!io) {
        return false;
    }
    link_.emplace(*io, timing_);
    return true;
}

void ControlBoardSource::read(SensorMap& sensors) {
    if (!link_) {
        if (retry_in_ > 0) {
            --retry_in_;
        } else if (!open()) {
            retry_in_ = kRetryReads;
        }
    }
    if (!link_) {
        decode(Poll{}, LinkStats{}, sensors);
        sensors.board.port_open = false;
        return;
    }
    const Poll poll = link_->read();
    decode(poll, link_->stats(), sensors);
    sensors.board.port_open = true;
}

void ControlBoardSource::decode(const Poll& poll, const LinkStats& stats, SensorMap& sensors) {
    BoardStatus& board = sensors.board;
    board.fresh        = poll.fresh;
    board.restarted    = poll.restarted;
    board.frames       = stats.frames;
    board.dropped      = stats.dropped;

    if (poll.updated) {
        for (std::size_t i = 0; i < poll.readings.count; ++i) {
            const brainlink::Record& record  = poll.readings.records[i];
            const RecordDecoder*     decoder = by_type_[record.sensor_type];
            if (decoder != nullptr) {
                decoder->decode(record, sensors);
            } else {
                ++unhandled_;
                last_type_ = record.sensor_type;
                last_port_ = record.port;
            }
        }
    }
    board.unhandled_records   = unhandled_;
    board.last_unhandled_type = last_type_;
    board.last_unhandled_port = last_port_;
}

} // namespace locagatr
