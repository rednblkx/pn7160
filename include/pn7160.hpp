#pragma once

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "transport.hpp"

#include <cstdint>
#include <functional>
#include <vector>
#include <atomic>

// --- NCI Constants (NCI Core Spec v2.0 & PN7160 UM11495) ---
namespace nci {

// Distinct type alias for NCI status codes to clarify intent
// where they are mixed with esp_err_t return values.
using status_t = uint8_t;

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
inline constexpr uint8_t PN7160_SPI_READ_TDD = 0xFF;  // UM11495 Table 7
inline constexpr uint8_t PN7160_SPI_WRITE_TDD = 0x0A; // UM11495 Table 7
inline constexpr size_t NCI_HEADER_SIZE = 3;
inline constexpr size_t NCI_MAX_PAYLOAD_SIZE = 255;
inline constexpr size_t NCI_MAX_PACKET_SIZE = NCI_HEADER_SIZE + NCI_MAX_PAYLOAD_SIZE;

} // namespace nci

// --- NCI Message Wrapper ---
class NciMessage {
public:
    NciMessage(size_t initial_size = nci::NCI_HEADER_SIZE) {
        buffer_.reserve(initial_size);
    }

    NciMessage(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        build(mt, gid, oid, payload);
    }

    void build(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        buffer_.clear();
        // MT (3 bits), PBF (1 bit), GID (4 bits)
        buffer_.push_back((mt << 5) | (gid & 0x0F)); // PBF is always 0 for CMD/RSP/NTF
        buffer_.push_back(oid & 0x3F);                 // OID (6 bits)
        buffer_.push_back(static_cast<uint8_t>(payload.size())); // Length
        buffer_.insert(buffer_.end(), payload.begin(), payload.end());
    }

    // Method to build a data packet (ConnID 0 for Static RF Connection)
    void build_data(const std::vector<uint8_t>& payload, uint8_t conn_id = 0, bool last_segment = true) {
        buffer_.clear();
        uint8_t pbf = last_segment ? 0 : 1; // Packet Boundary Flag
        // MT (3 bits=0), PBF (1 bit), ConnID (4 bits)
        buffer_.push_back((nci::PKT_MT_DATA << 5) | (pbf << 4) | (conn_id & 0x0F));
        buffer_.push_back(0); // RFU / OID placeholder for data
        buffer_.push_back(static_cast<uint8_t>(payload.size())); // Length
        buffer_.insert(buffer_.end(), payload.begin(), payload.end());
    }

    uint8_t get_mt() const { return (buffer_.size() > 0) ? (buffer_[0] >> 5) : 0; }
    uint8_t get_pbf() const { return (buffer_.size() > 0) ? ((buffer_[0] >> 4) & 0x01) : 0; }
    uint8_t get_gid() const { return (buffer_.size() > 0) ? (buffer_[0] & 0x0F) : 0; }
    uint8_t get_oid() const { return (buffer_.size() > 1) ? (buffer_[1] & 0x3F) : 0; }
    uint8_t get_len() const { return (buffer_.size() > 2) ? buffer_[2] : 0; }

    // Status is typically the first byte of the payload for RSP/NTF
    uint8_t get_status() const {
        return (buffer_.size() > 3 && get_len() > 0) ? buffer_[3] : nci::STATUS_FAILED;
    }

    const uint8_t* get_payload_ptr() const {
        return (buffer_.size() > nci::NCI_HEADER_SIZE) ? buffer_.data() + nci::NCI_HEADER_SIZE
                                                        : nullptr;
    }

    std::vector<uint8_t> get_payload_copy() const {
        if (buffer_.size() > nci::NCI_HEADER_SIZE) {
            return std::vector<uint8_t>(buffer_.begin() + nci::NCI_HEADER_SIZE,
                                        buffer_.end());
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

    void clear() { buffer_.clear(); }
    size_t size() const { return buffer_.size(); }
    uint8_t* data() { return buffer_.data(); }
    const uint8_t* data() const { return buffer_.data(); }

    // Safe indexed access (read-only for const, bounds-checked for non-const)
    uint8_t operator[](size_t idx) const {
        return (idx < buffer_.size()) ? buffer_[idx] : 0;
    }

    uint8_t at(size_t idx) const {
        return (idx < buffer_.size()) ? buffer_[idx] : 0;
    }

    bool empty() const { return buffer_.empty(); }

    void push_back(uint8_t byte) { buffer_.push_back(byte); }

    void resize(size_t n) { buffer_.resize(n); }

    void assign(const uint8_t* begin, const uint8_t* end) {
        buffer_.assign(begin, end);
    }

    // Unsafe direct data access for performance-critical internal use only
    uint8_t* raw_data() { return buffer_.data(); }
    const uint8_t* raw_data() const { return buffer_.data(); }

private:
    std::vector<uint8_t> buffer_;
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
    [[nodiscard]] esp_err_t core_reset(bool reset_config);
    [[nodiscard]] esp_err_t core_init();
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

private:
    // NCI Packet Read/Write
    [[nodiscard]] esp_err_t read_nci_packet(NciMessage& msg, uint32_t timeout_ms);
    [[nodiscard]] esp_err_t write_nci_packet(const NciMessage& msg);

    // Hardware Control
    [[nodiscard]] esp_err_t hardware_reset(); // Toggles VEN pin

    // Internal state/config
    IPN7160Transport& transport;
    TaskHandle_t task_handle_ = nullptr; // Handle of the task running task_runner
    std::atomic<bool> initialized_{false};
    std::atomic<bool> stop_flag_{false};

    // --- Members for Synchronous Exchange ---
    SemaphoreHandle_t apdu_sync_sem_ = nullptr;
    SemaphoreHandle_t cmd_sync_sem_ = nullptr;
    SemaphoreHandle_t sync_mutex_ = nullptr; // Mutex for flag and buffer
    std::vector<uint8_t> sync_apdu_response_;
    NciMessage sync_cmd_response_;
    std::atomic<bool> sync_apdu_in_progress_{false};
    std::atomic<bool> sync_cmd_in_progress_{false};

    std::atomic<bool> selected_tag_still_in_field{false};

    // Event queue
    QueueHandle_t event_queue_ = nullptr;
    static constexpr size_t EVENT_QUEUE_SIZE = 10;
    void post_event(NciEventType type, const NciMessage& msg);

    static constexpr const char* TAG = "PN7160_NCI"; // Logging tag
};
