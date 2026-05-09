#pragma once

#include "constants.hpp"

#include <array>
#include <span>
#include <vector>

// --- NCI Message Wrapper ---
class NciMessage {
public:
    NciMessage() = default;

    NciMessage(const uint8_t mt, const uint8_t gid, const uint8_t oid,
               const std::span<const uint8_t> payload = {}) {
        build(mt, gid, oid, payload);
    }

    void build(const uint8_t mt, const uint8_t gid, const uint8_t oid,
               const std::span<const uint8_t> payload = {}) {
        len_ = nci::NCI_HEADER_SIZE;
        buffer_[0] = (mt << 5) | (gid & 0x0F);
        buffer_[1] = oid & 0x3F;
        size_t plen = (payload.size() > nci::NCI_MAX_PAYLOAD_SIZE)
                          ? nci::NCI_MAX_PAYLOAD_SIZE
                          : payload.size();
        buffer_[2] = static_cast<uint8_t>(plen);
        for (size_t i = 0; i < plen; ++i)
            buffer_[nci::NCI_HEADER_SIZE + i] = payload[i];
        len_ += plen;
    }

    void build_data(std::span<const uint8_t> payload, uint8_t conn_id = 0,
                    bool last_segment = true) {
        len_ = nci::NCI_HEADER_SIZE;
        uint8_t pbf = last_segment ? 0 : 1;
        buffer_[0] = (nci::PKT_MT_DATA << 5) | (pbf << 4) | (conn_id & 0x0F);
        buffer_[1] = 0;
        size_t plen = (payload.size() > nci::NCI_MAX_PAYLOAD_SIZE)
                          ? nci::NCI_MAX_PAYLOAD_SIZE
                          : payload.size();
        buffer_[2] = static_cast<uint8_t>(plen);
        for (size_t i = 0; i < plen; ++i)
            buffer_[nci::NCI_HEADER_SIZE + i] = payload[i];
        len_ += plen;
    }

    [[nodiscard]] uint8_t get_mt()  const { return (len_ > 0) ? (buffer_[0] >> 5) : 0; }
    [[nodiscard]] uint8_t get_pbf() const { return (len_ > 0) ? ((buffer_[0] >> 4) & 0x01) : 0; }
    [[nodiscard]] uint8_t get_gid() const { return (len_ > 0) ? (buffer_[0] & 0x0F) : 0; }
    [[nodiscard]] uint8_t get_oid() const { return (len_ > 1) ? (buffer_[1] & 0x3F) : 0; }
    [[nodiscard]] uint8_t get_len() const { return (len_ > 2) ? buffer_[2] : 0; }

    [[nodiscard]] uint8_t get_status() const {
        return (len_ > 3 && get_len() > 0) ? buffer_[3] : nci::STATUS_FAILED;
    }

    [[nodiscard]] std::span<const uint8_t> get_payload() const {
        if (len_ > nci::NCI_HEADER_SIZE)
            return {buffer_.data() + nci::NCI_HEADER_SIZE,
                                              len_ - nci::NCI_HEADER_SIZE};
        return {};
    }

    [[nodiscard]] const uint8_t* get_payload_ptr() const {
        return buffer_.data() + nci::NCI_HEADER_SIZE;
    }

    [[nodiscard]] std::vector<uint8_t> get_payload_copy() const {
        auto p = get_payload();
        return {p.begin(), p.end()};
    }

    [[nodiscard]] bool is_control_notification(const uint8_t expected_gid,
                                 const uint8_t expected_oid) const {
        if (get_mt() != nci::PKT_MT_CTRL_NOTIFICATION) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    [[nodiscard]] bool is_control_response(const uint8_t expected_gid,
                             const uint8_t expected_oid) const {
        if (get_mt() != nci::PKT_MT_CTRL_RESPONSE) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    void    clear()                   { len_ = 0; }
    [[nodiscard]] size_t  size()              const { return len_; }
    uint8_t* data()                   { return &buffer_[0]; }
    [[nodiscard]] const uint8_t* data()       const { return buffer_.data(); }
    [[nodiscard]] bool    empty()             const { return len_ == 0; }

    uint8_t operator[](size_t idx) const {
        return (idx < len_) ? buffer_[idx] : 0;
    }

    void push_back(uint8_t byte) {
        if (len_ < nci::NCI_MAX_PACKET_SIZE) buffer_[len_++] = byte;
    }

    void resize(size_t n) {
        len_ = (n > nci::NCI_MAX_PACKET_SIZE) ? nci::NCI_MAX_PACKET_SIZE : n;
    }

    void assign(const uint8_t* begin, const uint8_t* end) {
        auto n = static_cast<size_t>(end - begin);
        if (n > nci::NCI_MAX_PACKET_SIZE) n = nci::NCI_MAX_PACKET_SIZE;
        for (size_t i = 0; i < n; ++i) buffer_[i] = begin[i];
        len_ = n;
    }

private:
    std::array<uint8_t, nci::NCI_MAX_PACKET_SIZE> buffer_{};
    uint16_t len_ = 0;
};
