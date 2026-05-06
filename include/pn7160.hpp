#pragma once

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "transport.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>
#include <atomic>

// --- NCI Constants (NCI Core Spec v2.0 & PN7160 UM11495) ---
namespace nci {

// Message Types (MT)
inline constexpr uint8_t PKT_MT_DATA = 0x00;
inline constexpr uint8_t PKT_MT_CTRL_COMMAND = 0x01;
inline constexpr uint8_t PKT_MT_CTRL_NOTIFICATION = 0x03; // NCI 2.0 uses 0x03
inline constexpr uint8_t PKT_MT_CTRL_RESPONSE = 0x02;     // NCI 2.0 uses 0x02

// Group IDs (GID)
inline constexpr uint8_t CORE_GID = 0x00;
inline constexpr uint8_t RF_GID = 0x01;
inline constexpr uint8_t NFCEE_MGMT_GID = 0x02;       // Added for completeness
inline constexpr uint8_t PROPRIETARY_GID = 0x0F;        // NXP Proprietary

// Opcode IDs (OID) - Core GID (CORE_GID = 0x00)
inline constexpr uint8_t CORE_RESET_OID = 0x00;
inline constexpr uint8_t CORE_INIT_OID = 0x01;
inline constexpr uint8_t CORE_SET_CONFIG_OID = 0x02;
inline constexpr uint8_t CORE_GET_CONFIG_OID = 0x03;
inline constexpr uint8_t CORE_CONN_CREATE_OID = 0x04;       // Added for completeness
inline constexpr uint8_t CORE_CONN_CLOSE_OID = 0x05;          // Added for completeness
inline constexpr uint8_t CORE_CONN_CREDITS_OID = 0x06;
inline constexpr uint8_t CORE_GENERIC_ERROR_OID = 0x07;
inline constexpr uint8_t CORE_INTERFACE_ERROR_OID = 0x08; // Added for completeness
inline constexpr uint8_t CORE_SET_POWER_SUB_STATE_OID = 0x09; // NCI 2.0

// Opcode IDs (OID) - RF Management GID (RF_GID = 0x01)
inline constexpr uint8_t RF_DISCOVER_MAP_OID = 0x00;
inline constexpr uint8_t RF_SET_LISTEN_MODE_ROUTING_OID = 0x01;
inline constexpr uint8_t RF_GET_LISTEN_MODE_ROUTING_OID = 0x02;
inline constexpr uint8_t RF_DISCOVER_OID = 0x03;
inline constexpr uint8_t RF_DISCOVER_SELECT_OID = 0x04;
inline constexpr uint8_t RF_INTF_ACTIVATED_OID = 0x05;
inline constexpr uint8_t RF_DEACTIVATE_OID = 0x06;
inline constexpr uint8_t RF_FIELD_INFO_OID = 0x07;
inline constexpr uint8_t RF_T3T_POLLING_OID = 0x08;
inline constexpr uint8_t RF_NFCEE_ACTION_OID = 0x09;
inline constexpr uint8_t RF_NFCEE_DISCOVERY_REQ_OID = 0x0A;
inline constexpr uint8_t RF_PARAMETER_UPDATE_OID = 0x0B;
inline constexpr uint8_t RF_INTF_EXT_START_OID = 0x0C; // NCI 2.0
inline constexpr uint8_t RF_INTF_EXT_STOP_OID = 0x0D;  // NCI 2.0
inline constexpr uint8_t RF_EXT_AGG_ABORT_OID = 0x0E;  // NCI 2.0
inline constexpr uint8_t RF_NDEF_ABORT_OID = 0x0F;     // NCI 2.0
inline constexpr uint8_t RF_ISO_DEP_NAK_PRESENCE_OID = 0x10; // NCI 2.0

inline constexpr uint8_t RF_DISCOVER_MAP_MODE_POLL = 0x1;
inline constexpr uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

// Opcode IDs (OID) - NFCEE Management GID (NFCEE_MGMT_GID = 0x02)
inline constexpr uint8_t NFCEE_DISCOVER_OID = 0x00;            // Added for completeness
inline constexpr uint8_t NFCEE_MODE_SET_OID = 0x01;            // Added for completeness
inline constexpr uint8_t NFCEE_STATUS_OID = 0x02;              // Added for completeness
inline constexpr uint8_t NFCEE_POWER_AND_LINK_CNTRL_OID = 0x03; // Added for completeness

// NCI Status Codes (NCI Core Spec v2.0 Table 129)
inline constexpr uint8_t STATUS_OK = 0x00;
inline constexpr uint8_t STATUS_REJECTED = 0x01;
inline constexpr uint8_t STATUS_RF_FRAME_CORRUPTED = 0x02;
inline constexpr uint8_t STATUS_FAILED = 0x03;
inline constexpr uint8_t STATUS_NOT_INITIALIZED = 0x04;
inline constexpr uint8_t STATUS_SYNTAX_ERROR = 0x05;
inline constexpr uint8_t STATUS_SEMANTIC_ERROR = 0x06;
inline constexpr uint8_t STATUS_INVALID_PARAM = 0x09;
inline constexpr uint8_t STATUS_MESSAGE_SIZE_EXCEEDED = 0x0A;
inline constexpr uint8_t DISCOVERY_ALREADY_STARTED = 0xA0;
inline constexpr uint8_t DISCOVERY_TARGET_ACTIVATION_FAILED = 0xA1;
inline constexpr uint8_t DISCOVERY_TEAR_DOWN = 0xA2;
inline constexpr uint8_t RF_TRANSMISSION_ERROR = 0xB0;
inline constexpr uint8_t RF_PROTOCOL_ERROR = 0xB1;
inline constexpr uint8_t RF_TIMEOUT_ERROR = 0xB2;
inline constexpr uint8_t NFCEE_INTERFACE_ACTIVATION_FAILED = 0xC0;
inline constexpr uint8_t NFCEE_TRANSMISSION_ERROR = 0xC1;
inline constexpr uint8_t NFCEE_PROTOCOL_ERROR = 0xC2;
inline constexpr uint8_t NFCEE_TIMEOUT_ERROR = 0xC3;
// Proprietary Status Codes (UM11495 Section 7.4.8 / Table 23)
inline constexpr uint8_t STATUS_LPCD_FAKE_DETECTION = 0xA3;
inline constexpr uint8_t STATUS_BOOT_TRIM_CORRUPTED = 0xE1;
inline constexpr uint8_t STATUS_EMVCO_PCD_COLLISION = 0xE4;

// Deactivation Types (NCI Core Spec v2.0 Section 7.3.2)
inline constexpr uint8_t DEACTIVATION_TYPE_IDLE = 0x00;       // Idle Mode
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP = 0x01;      // Sleep Mode
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP_AF = 0x02;   // Sleep_AF Mode
inline constexpr uint8_t DEACTIVATION_TYPE_DISCOVERY = 0x03; // Back to discovery

// RF Technologies (NCI Core Spec v2.0 Table 130)
inline constexpr uint8_t TECH_NFCA = 0x00;
inline constexpr uint8_t TECH_NFCB = 0x01;
inline constexpr uint8_t TECH_NFCF = 0x02;
inline constexpr uint8_t TECH_NFCV = 0x03; // ISO15693

// RF Technology and Mode (NCI Core Spec v2.0 Table 131)
inline constexpr uint8_t MODE_PASSIVE_POLL_A = 0x00;
inline constexpr uint8_t MODE_PASSIVE_POLL_B = 0x01;
inline constexpr uint8_t MODE_PASSIVE_POLL_F = 0x02;
inline constexpr uint8_t MODE_ACTIVE_POLL = 0x03;
inline constexpr uint8_t MODE_PASSIVE_POLL_V = 0x06; // ISO15693
inline constexpr uint8_t MODE_PASSIVE_LISTEN_A = 0x80;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_B = 0x81;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_F = 0x82;
inline constexpr uint8_t MODE_ACTIVE_LISTEN = 0x83;

// RF Protocols (NCI Core Spec v2.0 Table 133)
inline constexpr uint8_t PROT_UNDETERMINED = 0x00;
inline constexpr uint8_t PROT_T1T = 0x01;
inline constexpr uint8_t PROT_T2T = 0x02;
inline constexpr uint8_t PROT_T3T = 0x03;
inline constexpr uint8_t PROT_ISODEP = 0x04;  // ISO-DEP over NFC-A or NFC-B
inline constexpr uint8_t PROT_NFCDEP = 0x05;    // NFC-DEP over NFC-A or NFC-F
inline constexpr uint8_t PROT_T5T = 0x06;       // ISO15693
inline constexpr uint8_t PROT_NDEF = 0x07;      // NDEF on T1T/T2T/T3T/T4T/T5T
inline constexpr uint8_t PROT_MIFARE = 0x80;  // Proprietary (PN7160 UM Table 15)

// RF Interfaces (NCI Core Spec v2.0 Table 134)
inline constexpr uint8_t INTF_NFCEE_DIRECT = 0x00;
inline constexpr uint8_t INTF_FRAME = 0x01;
inline constexpr uint8_t INTF_ISODEP = 0x02;
inline constexpr uint8_t INTF_NFCDEP = 0x03;
inline constexpr uint8_t INTF_NDEF = 0x06;
inline constexpr uint8_t INTF_TAGCMD = 0x80; // Proprietary (PN7160 UM Table 18)

// --- PN7160 Specific ---
inline constexpr uint16_t PN7160_DEFAULT_TIMEOUT_MS = 10;
inline constexpr uint16_t PN7160_INIT_TIMEOUT_MS = 500;
inline constexpr uint16_t PN7160_IRQ_TIMEOUT_MS = 250;
inline constexpr size_t NCI_HEADER_SIZE = 3;
inline constexpr size_t NCI_MAX_PAYLOAD_SIZE = 255;
inline constexpr size_t NCI_MAX_PACKET_SIZE = NCI_HEADER_SIZE + NCI_MAX_PAYLOAD_SIZE;

} // namespace nci

// --- NCI Message Wrapper ---
class NciMessage {
public:
    NciMessage() = default;

    NciMessage(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        build(mt, gid, oid, payload);
    }

    void build(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        len_ = nci::NCI_HEADER_SIZE;
        buffer_[0] = (mt << 5) | (gid & 0x0F); // PBF is always 0 for CMD/RSP/NTF
        buffer_[1] = oid & 0x3F;
        size_t plen = (payload.size() > nci::NCI_MAX_PAYLOAD_SIZE)
                          ? nci::NCI_MAX_PAYLOAD_SIZE
                          : payload.size();
        buffer_[2] = static_cast<uint8_t>(plen);
        for (size_t i = 0; i < plen; ++i) {
            buffer_[nci::NCI_HEADER_SIZE + i] = payload[i];
        }
        len_ += plen;
    }

    // Method to build a data packet (ConnID 0 for Static RF Connection)
    void build_data(const std::vector<uint8_t>& payload, uint8_t conn_id = 0, bool last_segment = true) {
        len_ = nci::NCI_HEADER_SIZE;
        uint8_t pbf = last_segment ? 0 : 1; // Packet Boundary Flag
        buffer_[0] = (nci::PKT_MT_DATA << 5) | (pbf << 4) | (conn_id & 0x0F);
        buffer_[1] = 0; // RFU / OID placeholder for data
        size_t plen = (payload.size() > nci::NCI_MAX_PAYLOAD_SIZE)
                          ? nci::NCI_MAX_PAYLOAD_SIZE
                          : payload.size();
        buffer_[2] = static_cast<uint8_t>(plen);
        for (size_t i = 0; i < plen; ++i) {
            buffer_[nci::NCI_HEADER_SIZE + i] = payload[i];
        }
        len_ += plen;
    }

    uint8_t get_mt() const { return (len_ > 0) ? (buffer_[0] >> 5) : 0; }
    uint8_t get_pbf() const { return (len_ > 0) ? ((buffer_[0] >> 4) & 0x01) : 0; }
    uint8_t get_gid() const { return (len_ > 0) ? (buffer_[0] & 0x0F) : 0; }
    uint8_t get_oid() const { return (len_ > 1) ? (buffer_[1] & 0x3F) : 0; }
    uint8_t get_len() const { return (len_ > 2) ? buffer_[2] : 0; }

    // Status is typically the first byte of the payload for RSP/NTF
    uint8_t get_status() const {
        return (len_ > 3 && get_len() > 0) ? buffer_[3] : nci::STATUS_FAILED;
    }

    const uint8_t* get_payload_ptr() const {
        return buffer_.data() + nci::NCI_HEADER_SIZE;
    }

    std::vector<uint8_t> get_payload_copy() const {
        if (len_ > nci::NCI_HEADER_SIZE) {
            return std::vector<uint8_t>(buffer_.begin() + nci::NCI_HEADER_SIZE,
                                        buffer_.begin() + len_);
        }
        return {};
    }

    bool is_control_notification(uint8_t expected_gid = 0xFF,
                                 uint8_t expected_oid = 0xFF) const {
        if (get_mt() != nci::PKT_MT_CTRL_NOTIFICATION) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    bool is_control_response(uint8_t expected_gid = 0xFF,
                             uint8_t expected_oid = 0xFF) const {
        if (get_mt() != nci::PKT_MT_CTRL_RESPONSE) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    void clear() { len_ = 0; }
    size_t size() const { return len_; }
    uint8_t* data() { return &buffer_[0]; }
    const uint8_t* data() const { return buffer_.data(); }

    // Safe indexed access (read-only, bounds-checked)
    uint8_t operator[](size_t idx) const {
        return (idx < len_) ? buffer_[idx] : 0;
    }

    bool empty() const { return len_ == 0; }

    void push_back(uint8_t byte) {
        if (len_ < nci::NCI_MAX_PACKET_SIZE) {
            buffer_[len_++] = byte;
        }
    }

    void resize(size_t n) {
        len_ = (n > nci::NCI_MAX_PACKET_SIZE) ? nci::NCI_MAX_PACKET_SIZE : n;
    }

    void assign(const uint8_t* begin, const uint8_t* end) {
        size_t n = static_cast<size_t>(end - begin);
        if (n > nci::NCI_MAX_PACKET_SIZE) n = nci::NCI_MAX_PACKET_SIZE;
        for (size_t i = 0; i < n; ++i) buffer_[i] = begin[i];
        len_ = n;
    }

private:
    std::array<uint8_t, nci::NCI_MAX_PACKET_SIZE> buffer_{};
    uint16_t len_ = 0;
};

// --- Event Queue Types ---

enum class NciEventType {
    RF_INTF_ACTIVATED,
    RF_DISCOVER,
    RF_DEACTIVATE,
    CORE_NOTIFICATION,
    DATA_PACKET
};

struct NciEvent {
    NciEventType type;
    NciMessage msg;
};

// --- FreeRTOS RAII Helpers ---

/**
 * @brief RAII wrapper for a FreeRTOS mutex.
 *
 * Acquires the mutex on construction and releases it on destruction.
 * Safe to use even if acquisition times out (check acquired()).
 */
class ScopedMutex {
public:
    ScopedMutex(SemaphoreHandle_t mutex, TickType_t timeout) : mutex_(mutex) {
        acquired_ = (mutex_ != nullptr && xSemaphoreTake(mutex_, timeout) == pdTRUE);
    }
    ~ScopedMutex() {
        if (acquired_ && mutex_) {
            xSemaphoreGive(mutex_);
        }
    }
    ScopedMutex(const ScopedMutex&) = delete;
    ScopedMutex& operator=(const ScopedMutex&) = delete;

    bool acquired() const { return acquired_; }

private:
    SemaphoreHandle_t mutex_;
    bool acquired_ = false;
};

// --- Main Driver Class ---
class PN7160_NCI {
public:
    explicit PN7160_NCI(IPN7160Transport& transport);
    ~PN7160_NCI();

    // Initialization sequence
    // Returns ESP_OK on success, ESP_FAIL or other ESP error codes on failure.
    [[nodiscard]] esp_err_t initialize();

    // NCI Commands (High Level)
    // These helpers reuse caller-provided NciMessage buffers to avoid
    // stacking multiple 260-byte objects inside nested init calls.
    [[nodiscard]] esp_err_t core_reset(bool reset_config, NciMessage& cmd, NciMessage& rsp, NciMessage& ntf);
    [[nodiscard]] esp_err_t core_init(NciMessage& cmd, NciMessage& rsp);
    [[nodiscard]] esp_err_t core_set_config(const std::vector<uint8_t>& config_params);
    [[nodiscard]] esp_err_t rf_discover_map(const std::vector<uint8_t>& mappings);
    [[nodiscard]] esp_err_t rf_set_listen_mode_routing(
        const std::vector<uint8_t>& routing_config);
    [[nodiscard]] esp_err_t rf_start_discovery(const std::vector<uint8_t>& discovery_config);
    [[nodiscard]] esp_err_t rf_stop_discovery(); // Sends deactivate idle
    [[nodiscard]] esp_err_t rf_select_target(uint8_t discovery_id, uint8_t protocol,
                               uint8_t interface);
    [[nodiscard]] esp_err_t rf_deactivate(uint8_t type);
    [[nodiscard]] esp_err_t rf_iso_dep_presence_check();

    // Send raw NCI command and get response
    // Returns NCI Status Code from the response (e.g., STATUS_OK)
    // or an ESP error code if the send/receive itself failed.
    [[nodiscard]] esp_err_t send_command_wait_response(
        NciMessage& cmd, NciMessage& rsp,
        uint32_t timeout_ms = nci::PN7160_DEFAULT_TIMEOUT_MS);
    [[nodiscard]] esp_err_t send_apdu_sync(const std::vector<uint8_t>& c_apdu,
        std::vector<uint8_t>& r_apdu,
        uint32_t timeout_ms);
    // Send NCI data packet
    // Returns ESP_OK on successful SPI write, ESP error code otherwise.
    [[nodiscard]] esp_err_t send_data_packet(const NciMessage& data_pkt);

    // Get the next event from the driver's internal queue.
    // Blocks up to timeout_ms.  Copies the event into @p event.
    [[nodiscard]] esp_err_t get_event(NciEvent& event,
                                      uint32_t timeout_ms = portMAX_DELAY);

    // Access the underlying event queue directly (advanced use)
    QueueHandle_t event_queue() const { return event_queue_; }

    // Task runner function to process incoming messages (run this in a FreeRTOS task)
    void task_runner();

    // Signal the task runner to stop gracefully
    void shutdown();

    // Query initialization state
    bool is_initialized() const { return initialized_.load(); }
    bool tag_in_field() const { return selected_tag_still_in_field.load(); }

private:
    // NCI Packet Read/Write
    [[nodiscard]] esp_err_t read_nci_packet(NciMessage& msg, uint32_t timeout_ms);
    [[nodiscard]] esp_err_t write_nci_packet(const NciMessage& msg);

    // Init helper: send command, read response, validate type
    [[nodiscard]] esp_err_t send_init_command(const NciMessage& cmd,
                                               NciMessage& rsp,
                                               uint8_t expected_gid,
                                               uint8_t expected_oid,
                                               const char* name);

    // Sync helpers
    [[nodiscard]] esp_err_t validate_response(const NciMessage& cmd, const NciMessage& rsp);
    bool try_dispatch_sync(const NciMessage& msg);

    // Internal state/config
    IPN7160Transport& transport;
    TaskHandle_t task_handle_ = nullptr; // Handle of the task running task_runner
    std::atomic<bool> initialized_{false};
    std::atomic<bool> stop_flag_{false};

    // --- Unified Synchronous Exchange ---
    enum class SyncWaitType : uint8_t { NONE, CMD, APDU };

    SemaphoreHandle_t sync_sem_ = nullptr;
    SemaphoreHandle_t sync_mutex_ = nullptr;
    std::atomic<bool> sync_pending_{false};
    SyncWaitType sync_type_{SyncWaitType::NONE};
    NciMessage sync_cmd_response_;
    std::array<uint8_t, nci::NCI_MAX_PAYLOAD_SIZE> sync_apdu_response_{};
    size_t sync_apdu_response_len_ = 0;

    std::atomic<bool> selected_tag_still_in_field{false};

    // Event queue
    QueueHandle_t event_queue_ = nullptr;
    static constexpr size_t EVENT_QUEUE_SIZE = 10;
    void post_event(NciEventType type, const NciMessage& msg);

    // Task runner dispatch helpers
    void dispatch_data_packet(const NciMessage& msg);
    void dispatch_control_response(const NciMessage& msg);
    void dispatch_control_notification(const NciMessage& msg);
    void handle_rf_notification_oid(uint8_t oid, const NciMessage& msg);
    void handle_core_oid(uint8_t oid, const NciMessage& msg, bool is_response);
    void signal_sync_failure();

    static constexpr const char* TAG = "PN7160_NCI"; // Logging tag
};
