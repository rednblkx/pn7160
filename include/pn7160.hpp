#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "transport.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

// --- NCI Constants (NCI Core Spec v2.0 & PN7160 UM11495) ---
namespace nci {

// Message Types (MT)
inline constexpr uint8_t PKT_MT_DATA                = 0x00;
inline constexpr uint8_t PKT_MT_CTRL_COMMAND        = 0x01;
inline constexpr uint8_t PKT_MT_CTRL_RESPONSE       = 0x02; // NCI 2.0
inline constexpr uint8_t PKT_MT_CTRL_NOTIFICATION   = 0x03; // NCI 2.0

// Group IDs (GID)
inline constexpr uint8_t CORE_GID        = 0x00;
inline constexpr uint8_t RF_GID          = 0x01;
inline constexpr uint8_t NFCEE_MGMT_GID  = 0x02;
inline constexpr uint8_t PROPRIETARY_GID = 0x0F;

// Opcode IDs (OID) - Core GID
inline constexpr uint8_t CORE_RESET_OID             = 0x00;
inline constexpr uint8_t CORE_INIT_OID              = 0x01;
inline constexpr uint8_t CORE_SET_CONFIG_OID        = 0x02;
inline constexpr uint8_t CORE_GET_CONFIG_OID        = 0x03;
inline constexpr uint8_t CORE_CONN_CREATE_OID       = 0x04;
inline constexpr uint8_t CORE_CONN_CLOSE_OID        = 0x05;
inline constexpr uint8_t CORE_CONN_CREDITS_OID      = 0x06;
inline constexpr uint8_t CORE_GENERIC_ERROR_OID     = 0x07;
inline constexpr uint8_t CORE_INTERFACE_ERROR_OID   = 0x08;
inline constexpr uint8_t CORE_SET_POWER_SUB_STATE_OID = 0x09;

// Opcode IDs (OID) - RF Management GID
inline constexpr uint8_t RF_DISCOVER_MAP_OID            = 0x00;
inline constexpr uint8_t RF_SET_LISTEN_MODE_ROUTING_OID = 0x01;
inline constexpr uint8_t RF_GET_LISTEN_MODE_ROUTING_OID = 0x02;
inline constexpr uint8_t RF_DISCOVER_OID                = 0x03;
inline constexpr uint8_t RF_DISCOVER_SELECT_OID         = 0x04;
inline constexpr uint8_t RF_INTF_ACTIVATED_OID          = 0x05;
inline constexpr uint8_t RF_DEACTIVATE_OID              = 0x06;
inline constexpr uint8_t RF_FIELD_INFO_OID              = 0x07;
inline constexpr uint8_t RF_T3T_POLLING_OID             = 0x08;
inline constexpr uint8_t RF_NFCEE_ACTION_OID            = 0x09;
inline constexpr uint8_t RF_NFCEE_DISCOVERY_REQ_OID     = 0x0A;
inline constexpr uint8_t RF_PARAMETER_UPDATE_OID        = 0x0B;
inline constexpr uint8_t RF_INTF_EXT_START_OID          = 0x0C;
inline constexpr uint8_t RF_INTF_EXT_STOP_OID           = 0x0D;
inline constexpr uint8_t RF_EXT_AGG_ABORT_OID           = 0x0E;
inline constexpr uint8_t RF_NDEF_ABORT_OID              = 0x0F;
inline constexpr uint8_t RF_ISO_DEP_NAK_PRESENCE_OID    = 0x10;

inline constexpr uint8_t RF_DISCOVER_MAP_MODE_POLL   = 0x1;
inline constexpr uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

// Opcode IDs (OID) - NFCEE Management GID
inline constexpr uint8_t NFCEE_DISCOVER_OID            = 0x00;
inline constexpr uint8_t NFCEE_MODE_SET_OID            = 0x01;
inline constexpr uint8_t NFCEE_STATUS_OID              = 0x02;
inline constexpr uint8_t NFCEE_POWER_AND_LINK_CNTRL_OID = 0x03;

// NCI Status Codes (NCI Core Spec v2.0 Table 129)
inline constexpr uint8_t STATUS_OK                  = 0x00;
inline constexpr uint8_t STATUS_REJECTED            = 0x01;
inline constexpr uint8_t STATUS_RF_FRAME_CORRUPTED  = 0x02;
inline constexpr uint8_t STATUS_FAILED              = 0x03;
inline constexpr uint8_t STATUS_NOT_INITIALIZED     = 0x04;
inline constexpr uint8_t STATUS_SYNTAX_ERROR        = 0x05;
inline constexpr uint8_t STATUS_SEMANTIC_ERROR      = 0x06;
inline constexpr uint8_t STATUS_INVALID_PARAM       = 0x09;
inline constexpr uint8_t STATUS_MESSAGE_SIZE_EXCEEDED = 0x0A;
inline constexpr uint8_t DISCOVERY_ALREADY_STARTED          = 0xA0;
inline constexpr uint8_t DISCOVERY_TARGET_ACTIVATION_FAILED = 0xA1;
inline constexpr uint8_t DISCOVERY_TEAR_DOWN                = 0xA2;
inline constexpr uint8_t RF_TRANSMISSION_ERROR  = 0xB0;
inline constexpr uint8_t RF_PROTOCOL_ERROR      = 0xB1;
inline constexpr uint8_t RF_TIMEOUT_ERROR       = 0xB2;
inline constexpr uint8_t NFCEE_INTERFACE_ACTIVATION_FAILED = 0xC0;
inline constexpr uint8_t NFCEE_TRANSMISSION_ERROR          = 0xC1;
inline constexpr uint8_t NFCEE_PROTOCOL_ERROR              = 0xC2;
inline constexpr uint8_t NFCEE_TIMEOUT_ERROR               = 0xC3;
// Proprietary Status Codes (UM11495 Section 7.4.8 / Table 23)
inline constexpr uint8_t STATUS_LPCD_FAKE_DETECTION  = 0xA3;
inline constexpr uint8_t STATUS_BOOT_TRIM_CORRUPTED  = 0xE1;
inline constexpr uint8_t STATUS_EMVCO_PCD_COLLISION  = 0xE4;

// Deactivation Types (NCI Core Spec v2.0 Section 7.3.2)
inline constexpr uint8_t DEACTIVATION_TYPE_IDLE      = 0x00;
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP     = 0x01;
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP_AF  = 0x02;
inline constexpr uint8_t DEACTIVATION_TYPE_DISCOVERY = 0x03;

// RF Technologies (NCI Core Spec v2.0 Table 130)
inline constexpr uint8_t TECH_NFCA = 0x00;
inline constexpr uint8_t TECH_NFCB = 0x01;
inline constexpr uint8_t TECH_NFCF = 0x02;
inline constexpr uint8_t TECH_NFCV = 0x03;

// RF Technology and Mode (NCI Core Spec v2.0 Table 131)
inline constexpr uint8_t MODE_PASSIVE_POLL_A   = 0x00;
inline constexpr uint8_t MODE_PASSIVE_POLL_B   = 0x01;
inline constexpr uint8_t MODE_PASSIVE_POLL_F   = 0x02;
inline constexpr uint8_t MODE_ACTIVE_POLL      = 0x03;
inline constexpr uint8_t MODE_PASSIVE_POLL_V   = 0x06;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_A = 0x80;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_B = 0x81;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_F = 0x82;
inline constexpr uint8_t MODE_ACTIVE_LISTEN    = 0x83;

// RF Protocols (NCI Core Spec v2.0 Table 133)
inline constexpr uint8_t PROT_UNDETERMINED = 0x00;
inline constexpr uint8_t PROT_T1T         = 0x01;
inline constexpr uint8_t PROT_T2T         = 0x02;
inline constexpr uint8_t PROT_T3T         = 0x03;
inline constexpr uint8_t PROT_ISODEP      = 0x04;
inline constexpr uint8_t PROT_NFCDEP      = 0x05;
inline constexpr uint8_t PROT_T5T         = 0x06;
inline constexpr uint8_t PROT_NDEF        = 0x07;
inline constexpr uint8_t PROT_MIFARE      = 0x80;

// RF Interfaces (NCI Core Spec v2.0 Table 134)
inline constexpr uint8_t INTF_NFCEE_DIRECT = 0x00;
inline constexpr uint8_t INTF_FRAME        = 0x01;
inline constexpr uint8_t INTF_ISODEP       = 0x02;
inline constexpr uint8_t INTF_NFCDEP       = 0x03;
inline constexpr uint8_t INTF_NDEF         = 0x06;
inline constexpr uint8_t INTF_TAGCMD       = 0x80;

// --- PN7160 Specific ---
inline constexpr uint16_t PN7160_DEFAULT_TIMEOUT_MS = 500;
inline constexpr uint16_t PN7160_INIT_TIMEOUT_MS    = 500;
inline constexpr uint16_t PN7160_IRQ_TIMEOUT_MS     = 250;
inline constexpr size_t   NCI_HEADER_SIZE            = 3;
inline constexpr size_t   NCI_MAX_PAYLOAD_SIZE       = 255;
inline constexpr size_t   NCI_MAX_PACKET_SIZE        = NCI_HEADER_SIZE + NCI_MAX_PAYLOAD_SIZE;

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

    void build_data(const std::vector<uint8_t>& payload, uint8_t conn_id = 0,
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

    uint8_t get_mt()  const { return (len_ > 0) ? (buffer_[0] >> 5) : 0; }
    uint8_t get_pbf() const { return (len_ > 0) ? ((buffer_[0] >> 4) & 0x01) : 0; }
    uint8_t get_gid() const { return (len_ > 0) ? (buffer_[0] & 0x0F) : 0; }
    uint8_t get_oid() const { return (len_ > 1) ? (buffer_[1] & 0x3F) : 0; }
    uint8_t get_len() const { return (len_ > 2) ? buffer_[2] : 0; }

    uint8_t get_status() const {
        return (len_ > 3 && get_len() > 0) ? buffer_[3] : nci::STATUS_FAILED;
    }

    const uint8_t* get_payload_ptr() const {
        return buffer_.data() + nci::NCI_HEADER_SIZE;
    }

    std::vector<uint8_t> get_payload_copy() const {
        if (len_ > nci::NCI_HEADER_SIZE)
            return std::vector<uint8_t>(buffer_.begin() + nci::NCI_HEADER_SIZE,
                                        buffer_.begin() + len_);
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

    void    clear()                   { len_ = 0; }
    size_t  size()              const { return len_; }
    uint8_t* data()                   { return &buffer_[0]; }
    const uint8_t* data()       const { return buffer_.data(); }
    bool    empty()             const { return len_ == 0; }

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
        size_t n = static_cast<size_t>(end - begin);
        if (n > nci::NCI_MAX_PACKET_SIZE) n = nci::NCI_MAX_PACKET_SIZE;
        for (size_t i = 0; i < n; ++i) buffer_[i] = begin[i];
        len_ = n;
    }

private:
    std::array<uint8_t, nci::NCI_MAX_PACKET_SIZE> buffer_{};
    uint16_t len_ = 0;
};

// ---------------------------------------------------------------------------
// Event Queue Types
// ---------------------------------------------------------------------------

enum class NciEventType : uint8_t {
    RF_INTF_ACTIVATED,
    RF_DISCOVER,
    RF_DEACTIVATE,
    CORE_NOTIFICATION,
    DATA_PACKET,
};

// Priority assigned to each event type for drop-under-pressure policy.
// Higher value = higher priority (never dropped).
inline constexpr int event_priority(NciEventType t) {
    switch (t) {
        case NciEventType::RF_DEACTIVATE:      return 3; // must never be dropped
        case NciEventType::RF_INTF_ACTIVATED:  return 2;
        case NciEventType::DATA_PACKET:        return 2;
        case NciEventType::RF_DISCOVER:        return 1;
        case NciEventType::CORE_NOTIFICATION:  return 0; // lowest – drop first
        default:                               return 0;
    }
}

struct NciEvent {
    NciEventType type;
    NciMessage   msg;
};

// ---------------------------------------------------------------------------
// FreeRTOS RAII Helpers
// ---------------------------------------------------------------------------

/**
 * @brief RAII wrapper for a FreeRTOS mutex.
 */
class ScopedMutex {
public:
    ScopedMutex(SemaphoreHandle_t mutex, TickType_t timeout) : mutex_(mutex) {
        acquired_ = (mutex_ != nullptr &&
                     xSemaphoreTake(mutex_, timeout) == pdTRUE);
    }
    ~ScopedMutex() {
        if (acquired_ && mutex_) xSemaphoreGive(mutex_);
    }
    ScopedMutex(const ScopedMutex&)            = delete;
    ScopedMutex& operator=(const ScopedMutex&) = delete;
    bool acquired() const { return acquired_; }

private:
    SemaphoreHandle_t mutex_;
    bool acquired_ = false;
};

// ---------------------------------------------------------------------------
// Priority Ring Buffer
//
// Used as the app-facing event queue.  Under memory pressure, lower-priority
// events are overwritten before higher-priority ones.  Critical events
// (RF_DEACTIVATE) are never dropped.
// ---------------------------------------------------------------------------
template <size_t N>
class NciEventRingBuffer {
public:
    explicit NciEventRingBuffer() {
        mutex_ = xSemaphoreCreateMutex();
        data_avail_ = xSemaphoreCreateCounting(N, 0);
    }
    ~NciEventRingBuffer() {
        if (mutex_)      vSemaphoreDelete(mutex_);
        if (data_avail_) vSemaphoreDelete(data_avail_);
    }

    NciEventRingBuffer(const NciEventRingBuffer&)            = delete;
    NciEventRingBuffer& operator=(const NciEventRingBuffer&) = delete;

    bool valid() const { return mutex_ && data_avail_; }

    /**
     * Push an event.  If the buffer is full, the oldest event whose priority
     * is strictly lower than @p evt is overwritten.  If all existing events
     * have equal-or-higher priority, the new event is dropped and false is
     * returned (prevents starving critical events).
     */
    bool push(NciEvent&& evt) {
        ScopedMutex lock(mutex_, portMAX_DELAY);
        if (!lock.acquired()) return false;

        if (count_ < N) {
            buf_[tail_] = std::move(evt);
            tail_ = (tail_ + 1) % N;
            ++count_;
            xSemaphoreGive(data_avail_);
            return true;
        }

        // Buffer full – try to evict the oldest low-priority event.
        int new_prio = event_priority(evt.type);
        // Walk from head_ to find the first event with lower priority.
        for (size_t i = 0; i < count_; ++i) {
            size_t idx = (head_ + i) % N;
            if (event_priority(buf_[idx].type) < new_prio) {
                // Compact: shift all subsequent entries one step toward head.
                for (size_t j = i; j + 1 < count_; ++j) {
                    buf_[(head_ + j) % N] = std::move(buf_[(head_ + j + 1) % N]);
                }
                tail_ = (tail_ == 0) ? N - 1 : tail_ - 1;
                buf_[tail_] = std::move(evt);
                tail_ = (tail_ + 1) % N;
                // count_ stays the same; semaphore count unchanged.
                return true;
            }
        }

        // All existing events have >= priority – drop the new one.
        return false;
    }

    /**
     * Pop the oldest event.  Blocks up to @p timeout_ms.
     * Returns ESP_OK on success, ESP_ERR_TIMEOUT if nothing arrived.
     */
    esp_err_t pop(NciEvent& out, uint32_t timeout_ms) {
        if (xSemaphoreTake(data_avail_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
            return ESP_ERR_TIMEOUT;
        ScopedMutex lock(mutex_, portMAX_DELAY);
        if (!lock.acquired() || count_ == 0) return ESP_FAIL;
        out = std::move(buf_[head_]);
        head_ = (head_ + 1) % N;
        --count_;
        return ESP_OK;
    }

    size_t size() const {
        ScopedMutex lock(mutex_, portMAX_DELAY);
        return count_;
    }

private:
    std::array<NciEvent, N> buf_{};
    size_t head_{0}, tail_{0}, count_{0};
    SemaphoreHandle_t mutex_      = nullptr;
    SemaphoreHandle_t data_avail_ = nullptr;
};

// ---------------------------------------------------------------------------
// Unified Synchronous Exchange Slot
//
// Replaces the scattered sync_pending_ / sync_type_ / sync_sem_ machinery.
// One slot is ever active at a time.  RAII ensures sync_pending_ is always
// cleared even on early returns.
// ---------------------------------------------------------------------------
enum class SyncWaitType : uint8_t { NONE, CMD, APDU };

struct SyncExchange {
    SyncWaitType type{SyncWaitType::NONE};
    // For CMD matching – only accept RSP with matching GID+OID.
    uint8_t expected_gid{0xFF};
    uint8_t expected_oid{0xFF};
    // Result storage.
    NciMessage  cmd_result;
    std::array<uint8_t, nci::NCI_MAX_PAYLOAD_SIZE> apdu_result{};
    size_t      apdu_result_len{0};
    // Terminal error set by signal_sync_failure().
    esp_err_t   terminal_error{ESP_OK};
};

/**
 * @brief RAII guard for a SyncExchange slot.
 *
 * Acquires the slot on construction (fails if one is already active).
 * Clears sync_pending_ on destruction regardless of how the scope exits.
 */
class SyncSlot {
public:
    SyncSlot(SemaphoreHandle_t   mutex,
             SemaphoreHandle_t   sem,
             std::atomic<bool>&  pending,
             SyncExchange&       exchange,
             SyncWaitType        type,
             uint8_t             expected_gid = 0xFF,
             uint8_t             expected_oid = 0xFF)
        : sem_(sem), pending_(pending), valid_(false) {

        ScopedMutex lock(mutex, pdMS_TO_TICKS(50));
        if (!lock.acquired() || pending_.load()) return;

        exchange.type           = type;
        exchange.expected_gid   = expected_gid;
        exchange.expected_oid   = expected_oid;
        exchange.cmd_result.clear();
        exchange.apdu_result_len = 0;
        exchange.terminal_error  = ESP_OK;

        // Drain any stale signal before arming.
        xSemaphoreTake(sem_, 0);
        pending_.store(true);
        valid_ = true;
    }

    ~SyncSlot() {
        if (valid_) pending_.store(false);
    }

    SyncSlot(const SyncSlot&)            = delete;
    SyncSlot& operator=(const SyncSlot&) = delete;

    bool valid() const { return valid_; }

    /**
     * Block until the exchange completes or times out.
     * Returns pdTRUE if the semaphore was signalled, pdFALSE on timeout.
     */
    BaseType_t wait(uint32_t timeout_ms) {
        return xSemaphoreTake(sem_, pdMS_TO_TICKS(timeout_ms));
    }

private:
    SemaphoreHandle_t  sem_;
    std::atomic<bool>& pending_;
    bool               valid_;
};

// ---------------------------------------------------------------------------
// Main Driver Class
// ---------------------------------------------------------------------------
class PN7160_NCI {
public:
    explicit PN7160_NCI(IPN7160Transport& transport);
    ~PN7160_NCI();

    // Initialization sequence.
    // Returns ESP_OK on success, ESP_FAIL or other ESP error codes on failure.
    [[nodiscard]] esp_err_t initialize();

    // High-level NCI Commands.
    // Helpers reuse caller-provided NciMessage buffers to avoid stacking
    // multiple 260-byte objects inside nested init calls.
    [[nodiscard]] esp_err_t core_reset(bool reset_config,
                                       NciMessage& cmd,
                                       NciMessage& rsp,
                                       NciMessage& ntf);
    [[nodiscard]] esp_err_t core_init(NciMessage& cmd, NciMessage& rsp);
    [[nodiscard]] esp_err_t core_set_config(const std::vector<uint8_t>& config_params);
    [[nodiscard]] esp_err_t core_get_config(const std::vector<uint8_t>& config_params, NciMessage& rsp);
    [[nodiscard]] esp_err_t rf_discover_map(const std::vector<uint8_t>& mappings);
    [[nodiscard]] esp_err_t rf_set_listen_mode_routing(const std::vector<uint8_t>& routing_config);
    [[nodiscard]] esp_err_t rf_start_discovery(const std::vector<uint8_t>& discovery_config);
    [[nodiscard]] esp_err_t rf_stop_discovery();
    [[nodiscard]] esp_err_t rf_select_target(uint8_t discovery_id,
                                             uint8_t protocol,
                                             uint8_t interface);
    [[nodiscard]] esp_err_t rf_deactivate(uint8_t type);
    [[nodiscard]] esp_err_t rf_iso_dep_presence_check();

    // Send raw NCI command and wait for matching response.
    // Returns NCI Status Code or an ESP error code if transport failed.
    [[nodiscard]] esp_err_t send_command_wait_response(
        NciMessage& cmd,
        NciMessage& rsp,
        uint32_t    timeout_ms = nci::PN7160_DEFAULT_TIMEOUT_MS);

    // Send a C-APDU as an NCI data packet and synchronously await the R-APDU.
    [[nodiscard]] esp_err_t send_apdu_sync(const std::vector<uint8_t>& c_apdu,
                                           std::vector<uint8_t>&       r_apdu,
                                           uint32_t                    timeout_ms);

    // Send a raw NCI data packet (fire-and-forget write).
    [[nodiscard]] esp_err_t send_data_packet(const NciMessage& data_pkt);

    // Pop the next event from the app-facing priority ring buffer.
    // Blocks up to timeout_ms.
    [[nodiscard]] esp_err_t get_event(NciEvent& event,
                                      uint32_t  timeout_ms = portMAX_DELAY);

    // Task runner – run in a dedicated FreeRTOS task.
    // Once started this task exclusively owns the transport read path.
    void task_runner();

    // Signal the task runner to stop gracefully.
    void shutdown();

    bool is_initialized() const { return initialized_.load(); }
    bool tag_in_field()   const { return tag_in_field_.load(); }

private:
    // --- Transport access (exclusively owned by task_runner once started) ---
    [[nodiscard]] esp_err_t read_nci_packet(NciMessage& msg, uint32_t timeout_ms);
    [[nodiscard]] esp_err_t write_nci_packet(const NciMessage& msg);

    // --- Sync exchange helpers (called from task_runner) ---
    //
    // try_complete_exchange() inspects an incoming message and, if it matches
    // the active SyncExchange slot, stores the result and signals the waiter.
    // Returns true if the message was consumed.
    bool try_complete_exchange(const NciMessage& msg);

    // Called by task_runner on transport error to unblock any waiting caller
    // with a terminal error so it can return promptly instead of timing out.
    void signal_sync_failure(esp_err_t err = ESP_FAIL);

    // --- Task-runner dispatch (pure classification, no sync side effects) ---
    //
    // update_driver_state() updates atomic flags (tag_in_field_ etc.) from a
    // notification.  This is the only place driver state is mutated outside
    // of initialize() / shutdown(), keeping state changes serialised through
    // the task_runner loop.
    void update_driver_state(const NciMessage& msg);

    // Classify and push to the app-facing ring buffer.
    void enqueue_notification(const NciMessage& msg);
    void enqueue_data_packet(const NciMessage& msg);

    // Per-OID handlers (called from enqueue_notification).
    void handle_rf_notification(uint8_t oid, const NciMessage& msg);
    void handle_core_notification(uint8_t oid, const NciMessage& msg);

    // --- Members ---
    IPN7160Transport& transport_;

    TaskHandle_t       task_handle_ = nullptr;
    std::atomic<bool>  initialized_{false};
    std::atomic<bool>  stop_flag_{false};
    std::atomic<bool>  tag_in_field_{false};

    // --- Synchronous exchange state ---
    // sync_mutex_ guards all fields of exchange_ and sync_pending_.
    // sync_sem_ is given by task_runner when the exchange is complete.
    SemaphoreHandle_t  sync_mutex_   = nullptr;
    SemaphoreHandle_t  sync_sem_     = nullptr;
    std::atomic<bool>  sync_pending_{false};
    SyncExchange       exchange_{};

    SemaphoreHandle_t  transport_mutex_    = nullptr;

    // --- App-facing priority event ring buffer ---
    static constexpr size_t EVENT_QUEUE_SIZE = 10;
    NciEventRingBuffer<EVENT_QUEUE_SIZE> event_ring_;

    static constexpr const char* TAG = "PN7160_NCI";
};
