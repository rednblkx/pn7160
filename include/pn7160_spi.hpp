/* --- pn7160_spi.hpp --- */
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <cstdint>
#include <functional>
#include <vector>
#include <atomic>

// --- NCI Constants (NCI Core Spec v2.0 & PN7160 UM11495) ---
// Message Types (MT)
static const uint8_t NCI_PKT_MT_DATA = 0x00;
static const uint8_t NCI_PKT_MT_CTRL_COMMAND = 0x01;
static const uint8_t NCI_PKT_MT_CTRL_NOTIFICATION = 0x03; // NCI 2.0 uses 0x03
static const uint8_t NCI_PKT_MT_CTRL_RESPONSE = 0x02; // NCI 2.0 uses 0x02

// Group IDs (GID)
static const uint8_t NCI_CORE_GID = 0x00;
static const uint8_t RF_GID = 0x01;
static const uint8_t NFCEE_MGMT_GID = 0x02; // Added for completeness
static const uint8_t NCI_PROPRIETARY_GID = 0x0F; // NXP Proprietary

// Opcode IDs (OID) - Core GID (NCI_CORE_GID = 0x00)
static const uint8_t NCI_CORE_RESET_OID = 0x00;
static const uint8_t NCI_CORE_INIT_OID = 0x01;
static const uint8_t NCI_CORE_SET_CONFIG_OID = 0x02;
static const uint8_t NCI_CORE_GET_CONFIG_OID = 0x03;
static const uint8_t NCI_CORE_CONN_CREATE_OID = 0x04; // Added for completeness
static const uint8_t NCI_CORE_CONN_CLOSE_OID = 0x05; // Added for completeness
static const uint8_t NCI_CORE_CONN_CREDITS_OID = 0x06;
static const uint8_t NCI_CORE_GENERIC_ERROR_OID = 0x07;
static const uint8_t NCI_CORE_INTERFACE_ERROR_OID = 0x08; // Added for completeness
static const uint8_t NCI_CORE_SET_POWER_SUB_STATE_OID = 0x09; // NCI 2.0

// Opcode IDs (OID) - RF Management GID (RF_GID = 0x01)
static const uint8_t RF_DISCOVER_MAP_OID = 0x00;
static const uint8_t RF_SET_LISTEN_MODE_ROUTING_OID = 0x01;
static const uint8_t RF_GET_LISTEN_MODE_ROUTING_OID = 0x02;
static const uint8_t RF_DISCOVER_OID = 0x03;
static const uint8_t RF_DISCOVER_SELECT_OID = 0x04;
static const uint8_t RF_INTF_ACTIVATED_OID = 0x05;
static const uint8_t RF_DEACTIVATE_OID = 0x06;
static const uint8_t RF_FIELD_INFO_OID = 0x07;
static const uint8_t RF_T3T_POLLING_OID = 0x08;
static const uint8_t RF_NFCEE_ACTION_OID = 0x09;
static const uint8_t RF_NFCEE_DISCOVERY_REQ_OID = 0x0A;
static const uint8_t RF_PARAMETER_UPDATE_OID = 0x0B;
static const uint8_t RF_INTF_EXT_START_OID = 0x0C; // NCI 2.0
static const uint8_t RF_INTF_EXT_STOP_OID = 0x0D; // NCI 2.0
static const uint8_t RF_EXT_AGG_ABORT_OID = 0x0E; // NCI 2.0
static const uint8_t RF_NDEF_ABORT_OID = 0x0F; // NCI 2.0
static const uint8_t RF_ISO_DEP_NAK_PRESENCE_OID = 0x10; // NCI 2.0

static const uint8_t RF_DISCOVER_MAP_MODE_POLL = 0x1;
static const uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

// Opcode IDs (OID) - NFCEE Management GID (NFCEE_MGMT_GID = 0x02)
static const uint8_t NFCEE_DISCOVER_OID = 0x00; // Added for completeness
static const uint8_t NFCEE_MODE_SET_OID = 0x01; // Added for completeness
static const uint8_t NFCEE_STATUS_OID = 0x02; // Added for completeness
static const uint8_t NFCEE_POWER_AND_LINK_CNTRL_OID = 0x03; // Added for completeness

// NCI Status Codes (NCI Core Spec v2.0 Table 129)
static const uint8_t STATUS_OK = 0x00;
static const uint8_t STATUS_REJECTED = 0x01;
static const uint8_t STATUS_RF_FRAME_CORRUPTED = 0x02;
static const uint8_t STATUS_FAILED = 0x03;
static const uint8_t STATUS_NOT_INITIALIZED = 0x04;
static const uint8_t STATUS_SYNTAX_ERROR = 0x05;
static const uint8_t STATUS_SEMANTIC_ERROR = 0x06;
static const uint8_t STATUS_INVALID_PARAM = 0x09;
static const uint8_t STATUS_MESSAGE_SIZE_EXCEEDED = 0x0A;
static const uint8_t DISCOVERY_ALREADY_STARTED = 0xA0;
static const uint8_t DISCOVERY_TARGET_ACTIVATION_FAILED = 0xA1;
static const uint8_t DISCOVERY_TEAR_DOWN = 0xA2;
static const uint8_t RF_TRANSMISSION_ERROR = 0xB0;
static const uint8_t RF_PROTOCOL_ERROR = 0xB1;
static const uint8_t RF_TIMEOUT_ERROR = 0xB2;
static const uint8_t NFCEE_INTERFACE_ACTIVATION_FAILED = 0xC0;
static const uint8_t NFCEE_TRANSMISSION_ERROR = 0xC1;
static const uint8_t NFCEE_PROTOCOL_ERROR = 0xC2;
static const uint8_t NFCEE_TIMEOUT_ERROR = 0xC3;
// Proprietary Status Codes (UM11495 Section 7.4.8 / Table 23)
static const uint8_t STATUS_LPCD_FAKE_DETECTION = 0xA3;
static const uint8_t STATUS_BOOT_TRIM_CORRUPTED = 0xE1;
static const uint8_t STATUS_EMVCO_PCD_COLLISION = 0xE4;

// Deactivation Types (NCI Core Spec v2.0 Section 7.3.2)
static const uint8_t DEACTIVATION_TYPE_IDLE = 0x00; // Idle Mode
static const uint8_t DEACTIVATION_TYPE_SLEEP = 0x01; // Sleep Mode
static const uint8_t DEACTIVATION_TYPE_SLEEP_AF = 0x02; // Sleep_AF Mode
static const uint8_t DEACTIVATION_TYPE_DISCOVERY = 0x03; // Back to discovery

// RF Technologies (NCI Core Spec v2.0 Table 130)
static const uint8_t TECH_NFCA = 0x00;
static const uint8_t TECH_NFCB = 0x01;
static const uint8_t TECH_NFCF = 0x02;
static const uint8_t TECH_NFCV = 0x03; // ISO15693

// RF Technology and Mode (NCI Core Spec v2.0 Table 131)
static const uint8_t MODE_PASSIVE_POLL_A = 0x00;
static const uint8_t MODE_PASSIVE_POLL_B = 0x01;
static const uint8_t MODE_PASSIVE_POLL_F = 0x02;
static const uint8_t MODE_ACTIVE_POLL = 0x03;
static const uint8_t MODE_PASSIVE_POLL_V = 0x06; // ISO15693
static const uint8_t MODE_PASSIVE_LISTEN_A = 0x80;
static const uint8_t MODE_PASSIVE_LISTEN_B = 0x81;
static const uint8_t MODE_PASSIVE_LISTEN_F = 0x82;
static const uint8_t MODE_ACTIVE_LISTEN = 0x83;

// RF Protocols (NCI Core Spec v2.0 Table 133)
static const uint8_t PROT_UNDETERMINED = 0x00;
static const uint8_t PROT_T1T = 0x01;
static const uint8_t PROT_T2T = 0x02;
static const uint8_t PROT_T3T = 0x03;
static const uint8_t PROT_ISODEP = 0x04; // ISO-DEP over NFC-A or NFC-B
static const uint8_t PROT_NFCDEP = 0x05; // NFC-DEP over NFC-A or NFC-F
static const uint8_t PROT_T5T = 0x06; // ISO15693
static const uint8_t PROT_NDEF = 0x07; // NDEF on T1T/T2T/T3T/T4T/T5T
static const uint8_t PROT_MIFARE = 0x80; // Proprietary (PN7160 UM Table 15)

// RF Interfaces (NCI Core Spec v2.0 Table 134)
static const uint8_t INTF_NFCEE_DIRECT = 0x00;
static const uint8_t INTF_FRAME = 0x01;
static const uint8_t INTF_ISODEP = 0x02;
static const uint8_t INTF_NFCDEP = 0x03;
static const uint8_t INTF_NDEF = 0x06;
static const uint8_t INTF_TAGCMD = 0x80; // Proprietary (PN7160 UM Table 18)

// --- PN7160 Specific ---
static const uint16_t PN7160_DEFAULT_TIMEOUT_MS = 10;
static const uint16_t PN7160_INIT_TIMEOUT_MS = 500;
static const uint16_t PN7160_IRQ_TIMEOUT_MS = 250;
static const uint8_t PN7160_SPI_READ_TDD = 0xFF; // UM11495 Table 7
static const uint8_t PN7160_SPI_WRITE_TDD = 0x0A; // UM11495 Table 7
static const size_t NCI_HEADER_SIZE = 3;
static const size_t NCI_MAX_PAYLOAD_SIZE = 255;
static const size_t NCI_MAX_PACKET_SIZE = NCI_HEADER_SIZE + NCI_MAX_PAYLOAD_SIZE;

// --- Configuration Structures ---
struct PN7160_SPI_PinConfig {
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t irq;
    gpio_num_t ven; // VEN/ENABLE pin
    // Optional pins (set to GPIO_NUM_NC if not used)
    gpio_num_t dwl_req = GPIO_NUM_NC;
    gpio_num_t wkup_req = GPIO_NUM_NC;
};

// --- NCI Message Wrapper ---
class NciMessage {
public:
    std::vector<uint8_t> buffer;

    NciMessage(size_t initial_size = NCI_HEADER_SIZE) {
        buffer.reserve(initial_size);
    }

    NciMessage(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        build(mt, gid, oid, payload);
    }

    void build(uint8_t mt, uint8_t gid, uint8_t oid,
               const std::vector<uint8_t>& payload = {}) {
        buffer.clear();
        // MT (3 bits), PBF (1 bit), GID (4 bits)
        buffer.push_back((mt << 5) | (gid & 0x0F)); // PBF is always 0 for CMD/RSP/NTF
        buffer.push_back(oid & 0x3F); // OID (6 bits)
        buffer.push_back(static_cast<uint8_t>(payload.size())); // Length
        buffer.insert(buffer.end(), payload.begin(), payload.end());
    }

    // Method to build a data packet (ConnID 0 for Static RF Connection)
    void build_data(const std::vector<uint8_t>& payload, uint8_t conn_id = 0, bool last_segment = true) {
        buffer.clear();
        uint8_t pbf = last_segment ? 0 : 1; // Packet Boundary Flag
        // MT (3 bits=0), PBF (1 bit), ConnID (4 bits)
        buffer.push_back((NCI_PKT_MT_DATA << 5) | (pbf << 4) | (conn_id & 0x0F));
        buffer.push_back(0); // RFU / OID placeholder for data
        buffer.push_back(static_cast<uint8_t>(payload.size())); // Length
        buffer.insert(buffer.end(), payload.begin(), payload.end());
    }

    uint8_t get_mt() const { return (buffer.size() > 0) ? (buffer[0] >> 5) : 0; }
    uint8_t get_pbf() const { return (buffer.size() > 0) ? ((buffer[0] >> 4) & 0x01) : 0; }
    uint8_t get_gid() const { return (buffer.size() > 0) ? (buffer[0] & 0x0F) : 0; }
    uint8_t get_oid() const { return (buffer.size() > 1) ? (buffer[1] & 0x3F) : 0; }
    uint8_t get_len() const { return (buffer.size() > 2) ? buffer[2] : 0; }
    // Status is typically the first byte of the payload for RSP/NTF
    uint8_t get_status() const {
        return (buffer.size() > 3 && get_len() > 0) ? buffer[3] : STATUS_FAILED;
    }

    const uint8_t* get_payload_ptr() const {
        return (buffer.size() > NCI_HEADER_SIZE) ? buffer.data() + NCI_HEADER_SIZE
                                                  : nullptr;
    }

    std::vector<uint8_t> get_payload_copy() const {
        if (buffer.size() > NCI_HEADER_SIZE) {
            return std::vector<uint8_t>(buffer.begin() + NCI_HEADER_SIZE,
                                        buffer.end());
        }
        return {};
    }

    bool is_control_notification(uint8_t expected_gid = 0xFF,
                                 uint8_t expected_oid = 0xFF) const {
        if (get_mt() != NCI_PKT_MT_CTRL_NOTIFICATION) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    bool is_control_response(uint8_t expected_gid = 0xFF,
                             uint8_t expected_oid = 0xFF) const {
        if (get_mt() != NCI_PKT_MT_CTRL_RESPONSE) return false;
        if (expected_gid != 0xFF && get_gid() != expected_gid) return false;
        if (expected_oid != 0xFF && get_oid() != expected_oid) return false;
        return true;
    }

    void clear() { buffer.clear(); }
    size_t size() const { return buffer.size(); }
    uint8_t* data() { return buffer.data(); }
    const uint8_t* data() const { return buffer.data(); }
};

// --- Main Driver Class ---
class PN7160_SPI {
public:
    // --- Callbacks ---
    using RfInterfaceActivatedCallback = std::function<void(const NciMessage&)>;
    using RfDiscoverCallback = std::function<void(const NciMessage&)>;
    using RfDeactivateCallback = std::function<void(const NciMessage&)>;
    using CoreNotificationCallback = std::function<void(const NciMessage&)>;
    using DataPacketCallback = std::function<void(const NciMessage&)>;

    PN7160_SPI(spi_host_device_t host, const PN7160_SPI_PinConfig& pins,
               int spi_clock_mhz = 4);
    ~PN7160_SPI();

    // Initialization sequence
    // Returns ESP_OK on success, ESP_FAIL or other ESP error codes on failure.
    esp_err_t initialize();

    // NCI Commands (High Level)
    // These functions return NCI Status Codes (e.g., STATUS_OK, STATUS_FAILED)
    // cast to esp_err_t for convenience with ESP_RETURN_ON_ERROR.
    // Check return value against STATUS_OK (0x00) for success.
    esp_err_t core_reset(bool reset_config, bool power_cycle);
    esp_err_t core_init();
    esp_err_t core_set_config(const std::vector<uint8_t>& config_params);
    esp_err_t rf_discover_map(const std::vector<uint8_t>& mappings);
    esp_err_t rf_set_listen_mode_routing(
        const std::vector<uint8_t>& routing_config);
    esp_err_t rf_start_discovery(const std::vector<uint8_t>& discovery_config);
    esp_err_t rf_stop_discovery(); // Sends deactivate idle
    esp_err_t rf_select_target(uint8_t discovery_id, uint8_t protocol,
                               uint8_t interface);
    esp_err_t rf_deactivate(uint8_t type);
    esp_err_t rf_iso_dep_presence_check();
    void presence_check_runner();

    // Send raw NCI command and get response
    // Returns NCI Status Code from the response (e.g., STATUS_OK)
    // or an ESP error code if the send/receive itself failed.
    esp_err_t send_command_wait_response(
        NciMessage& cmd, NciMessage& rsp,
        uint32_t timeout_ms = PN7160_DEFAULT_TIMEOUT_MS);
    esp_err_t send_apdu_sync(const std::vector<uint8_t>& c_apdu,
        std::vector<uint8_t>& r_apdu,
        uint32_t timeout_ms);
    // Send NCI data packet
    // Returns ESP_OK on successful SPI write, ESP error code otherwise.
    esp_err_t send_data_packet(const NciMessage& data_pkt);

    // Register callbacks
    void set_on_rf_interface_activated(RfInterfaceActivatedCallback cb) {
        on_rf_intf_activated_ = std::move(cb);
    }
    void set_on_rf_discover(RfDiscoverCallback cb) {
        on_rf_discover_ = std::move(cb);
    }
    void set_on_rf_deactivate(RfDeactivateCallback cb) {
        on_rf_deactivate_ = std::move(cb);
    }
    void set_on_core_notification(CoreNotificationCallback cb) {
        on_core_notification_ = std::move(cb);
    }
    void set_on_data_packet(DataPacketCallback cb) {
        on_data_packet_ = std::move(cb);
    }

    // Task runner function to process incoming messages (run this in a FreeRTOS task)
    void task_runner();

private:
    // Low-level SPI communication
    esp_err_t spi_read(uint8_t* buffer, size_t length);
    esp_err_t spi_write(const uint8_t* buffer, size_t length);
    esp_err_t spi_transfer(spi_transaction_t* trans);

    // NCI Packet Read/Write
    esp_err_t read_nci_packet(NciMessage& msg, uint32_t timeout_ms);
    esp_err_t write_nci_packet(const NciMessage& msg);

    // IRQ Handling
    esp_err_t setup_irq();
    esp_err_t wait_for_irq(bool expected_state, TickType_t timeout_ticks);
    static void IRAM_ATTR gpio_isr_handler(void* arg);

    // Hardware Control
    esp_err_t hardware_reset(); // Toggles VEN pin
    void set_ven(bool enable);
    void chip_select(bool select);

    // Internal state/config
    spi_host_device_t spi_host_;
    PN7160_SPI_PinConfig pins_;
    int spi_clock_hz_;
    spi_device_handle_t spi_device_ = nullptr;
    SemaphoreHandle_t irq_sem_ = nullptr;
    TaskHandle_t task_handle_ = nullptr; // Handle of the task running task_runner
    bool initialized_ = false;
    bool irq_isr_installed_ = false;

    // --- Members for Synchronous APDU Exchange ---
    SemaphoreHandle_t apdu_sync_sem_ = nullptr;
    SemaphoreHandle_t sync_mutex_ = nullptr; // Mutex for flag and buffer
    std::vector<uint8_t> sync_apdu_response_;
    volatile bool sync_apdu_in_progress_ = false; // volatile as it's accessed by ISR context potentially via callback

    std::atomic<bool> selected_tag_still_in_field{false};

    // Callbacks
    RfInterfaceActivatedCallback on_rf_intf_activated_;
    RfDiscoverCallback on_rf_discover_;
    RfDeactivateCallback on_rf_deactivate_;
    CoreNotificationCallback on_core_notification_;
    DataPacketCallback on_data_packet_;

    const char* TAG = "PN7160_SPI"; // Logging tag
};