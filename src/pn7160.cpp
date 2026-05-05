#include "pn7160.hpp"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "transport.hpp"
#include <cstdint>
#include <cstring>
#include <vector>

// UM11495 Section 13.1 - PMU_CFG (Tag 0xA00E)
static const uint8_t PMU_CFG[] = {
    0x01,        // Number of parameters
    0xA0, 0x0E,  // ext. tag
    11,          // length
    0x11,        // IRQ Enable: PVDD + temp sensor IRQs
    0x01,        // RFU
    0x01,        // Power and Clock Configuration, device on (CFG1)
    0x01,        // Power and Clock Configuration, device off (CFG1)
    0x00,        // RFU
    0x00,        // DC-DC 0
    0x00,        // DC-DC 1
    0xFF,  // TXLDO (5.0V / 5.0V)
    0x00,  // RFU
    0xD0,  // TXLDO check
    0x0C,  // RFU
};

// NCI Core Spec v2.0 Section 6.1 - TOTAL_DURATION (Tag 0x00)
static const uint8_t CORE_CONFIG_TOTAL_DURATION_SOLO[] = {
    0x01, // Number of parameter fields
    0x00, // config param identifier (TOTAL_DURATION)
    0x02, // length of value
    0x01, // TOTAL_DURATION (low)... 0x03E8 = 1000ms
    0x00 // TOTAL_DURATION (high): 1s
};

// --- Constructor / Destructor ---

PN7160_NCI::PN7160_NCI(IPN7160Transport& transport) :
    transport(transport) {
    sync_sem_ = xSemaphoreCreateBinary();
    sync_mutex_ = xSemaphoreCreateMutex();
    event_queue_ = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(NciEvent));
    if (!sync_sem_ || !sync_mutex_ || !event_queue_) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives or event queue");
        if (sync_sem_) vSemaphoreDelete(sync_sem_);
        if (sync_mutex_) vSemaphoreDelete(sync_mutex_);
        if (event_queue_) vQueueDelete(event_queue_);
    }
}

PN7160_NCI::~PN7160_NCI() {
    shutdown();
    if (sync_sem_) {
        vSemaphoreDelete(sync_sem_);
        sync_sem_ = nullptr;
    }
    if (sync_mutex_) {
        vSemaphoreDelete(sync_mutex_);
        sync_mutex_ = nullptr;
    }
    if (event_queue_) {
        vQueueDelete(event_queue_);
        event_queue_ = nullptr;
    }
}

// --- Event Queue Helpers ---

void PN7160_NCI::post_event(NciEventType type, const NciMessage& msg) {
    if (!event_queue_) return;
    NciEvent evt{type, msg};
    if (xQueueSend(event_queue_, &evt, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Event queue full, dropping event");
    }
}

esp_err_t PN7160_NCI::get_event(NciEvent& event, uint32_t timeout_ms) {
    if (!event_queue_) return ESP_ERR_INVALID_STATE;
    if (xQueueReceive(event_queue_, &event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

// --- Initialization ---

esp_err_t PN7160_NCI::send_init_command(const NciMessage& cmd,
                                          NciMessage& rsp,
                                          uint8_t expected_gid,
                                          uint8_t expected_oid,
                                          const char* name) {
    esp_err_t write_err = write_nci_packet(cmd);
    if (write_err != ESP_OK) {
        ESP_LOGE(TAG, "Error sending %s (err=0x%X)", name, write_err);
        return write_err;
    }

    esp_err_t read_err = read_nci_packet(rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (read_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %s (err=0x%X)", name, read_err);
        return read_err;
    }

    if (!rsp.is_control_response(expected_gid, expected_oid)) {
        ESP_LOGE(TAG, "Received unexpected packet after %s (expected RSP)", name);
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return ESP_FAIL;
    }

    uint8_t nci_status = rsp.get_status();
    if (nci_status != nci::STATUS_OK) {
        ESP_LOGW(TAG, "%s returned NCI status 0x%X", name, nci_status);
    }
    return static_cast<esp_err_t>(nci_status);
}

esp_err_t PN7160_NCI::initialize() {
    ESP_LOGI(TAG, "Initializing PN7160...");

    esp_err_t ret = transport.init();
    if (ret != ESP_OK) return ret;

    // NCI Core Reset (buffers declared here so core_reset doesn't add its own)
    {
        NciMessage cmd, rsp, ntf;
        ret = core_reset(true, cmd, rsp, ntf);
        if (ret != nci::STATUS_OK) {
            ESP_LOGE(TAG, "NCI Core Reset command failed (NCI Status=0x%X)", ret);
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "NCI Core Reset successful");

    // NCI Core Init
    {
        NciMessage cmd, rsp;
        ret = core_init(cmd, rsp);
    }
    if (ret != nci::STATUS_OK) {
        ESP_LOGE(TAG, "NCI Core Init command failed (NCI Status=0x%X)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Init successful");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Activate Proprietary Extensions
    ESP_LOGD(TAG, "Activating NXP Proprietary Extensions...");
    NciMessage cmd, rsp;
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::PROPRIETARY_GID, nci::CORE_SET_CONFIG_OID);
    ret = send_init_command(cmd, rsp,
                            nci::PROPRIETARY_GID, nci::CORE_SET_CONFIG_OID,
                            "PROPRIETARY_ACT_CMD");
    if (ret == ESP_FAIL) return ESP_FAIL;
    if (ret != nci::STATUS_OK) {
        ESP_LOGW(TAG, "Failed to activate proprietary extensions (NCI Status=0x%X). Configs might fail.", ret);
    } else {
        ESP_LOGI(TAG, "Proprietary extensions activated.");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.get_payload_ptr(), rsp.get_len(), ESP_LOG_DEBUG);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "Sending initial configuration...");

    // Send PMU Config
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_SET_CONFIG_OID,
              std::vector<uint8_t>(std::begin(PMU_CFG), std::end(PMU_CFG)));
    rsp.clear();
    ret = send_init_command(cmd, rsp, nci::CORE_GID, nci::CORE_SET_CONFIG_OID, "PMU_CFG");
    if (ret == ESP_FAIL) return ESP_FAIL;
    if (ret != nci::STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set PMU config (NCI Status=0x%X)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PMU Config set successfully");
    vTaskDelay(pdMS_TO_TICKS(5));

    // Send TOTAL_DURATION Config
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_SET_CONFIG_OID,
              std::vector<uint8_t>(std::begin(CORE_CONFIG_TOTAL_DURATION_SOLO),
                                   std::end(CORE_CONFIG_TOTAL_DURATION_SOLO)));
    rsp.clear();
    ret = send_init_command(cmd, rsp, nci::CORE_GID, nci::CORE_SET_CONFIG_OID, "TOTAL_DURATION");
    if (ret == ESP_FAIL) return ESP_FAIL;
    if (ret != nci::STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set TOTAL_DURATION config (NCI Status=0x%X)", ret);
    } else {
        ESP_LOGI(TAG, "TOTAL_DURATION Config set successfully");
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    stop_flag_.store(false);
    initialized_.store(true);
    ESP_LOGI(TAG, "PN7160 Initialized Successfully");
    return ESP_OK;
}

// --- NCI Packet Read/Write ---

esp_err_t PN7160_NCI::read_nci_packet(NciMessage& msg, uint32_t timeout_ms) {
    msg.clear();
    uint8_t header[nci::NCI_HEADER_SIZE];

    // Wait for IRQ high (indicates data available)
    esp_err_t ret = transport.wait_for_irq(true, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (transport.read_irq_level()) {
             ESP_LOGW(TAG, "IRQ became HIGH right after timeout check in read_nci_packet");
        } else {
            ESP_LOGD(TAG, "Timeout waiting for IRQ high before read");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Read NCI Header (3 bytes)
    ESP_RETURN_ON_ERROR(transport.read(header, nci::NCI_HEADER_SIZE), TAG,
                        "Failed to read NCI header");

    // Parse header and get payload length
    msg.assign(header, header + nci::NCI_HEADER_SIZE);
    uint8_t payload_len = msg.get_len();

    ESP_LOGD(TAG, "Read Header: MT=0x%X GID=0x%X OID=0x%X Len=%d", msg.get_mt(),
             msg.get_gid(), msg.get_oid(), payload_len);

    if (payload_len > nci::NCI_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "NCI payload length too large: %d", payload_len);
        msg.clear();
        return ESP_FAIL;
    }

    // Read Payload (if any)
    if (payload_len > 0) {
        size_t current_size = msg.size();
        msg.resize(current_size + payload_len);
        ESP_RETURN_ON_ERROR(
            transport.read(msg.data() + current_size, payload_len), TAG,
            "Failed to read NCI payload");
    }

    ESP_LOGD(TAG, "Read NCI Packet (Size: %zu):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);

    return ESP_OK;
}

esp_err_t PN7160_NCI::write_nci_packet(const NciMessage& msg) {
    if (msg.size() == 0 || msg.size() > nci::NCI_MAX_PACKET_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Write NCI Packet (Size: %zu):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);
    return transport.write(msg.data(), msg.size());
}

// --- Synchronous Exchange Helpers ---

esp_err_t PN7160_NCI::validate_response(const NciMessage& cmd, const NciMessage& rsp) {
    if (!rsp.is_control_response(cmd.get_gid(), cmd.get_oid())) {
        ESP_LOGE(TAG,
                 "Unexpected NCI packet received. Expected RSP for GID=0x%02X, "
                 "OID=0x%02X. Got MT=%d, GID=0x%02X, OID=0x%02X",
                 cmd.get_gid(), cmd.get_oid(), rsp.get_mt(), rsp.get_gid(),
                 rsp.get_oid());
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return nci::STATUS_FAILED;
    }
    ESP_LOGD(TAG,
             "Received RSP: MT=0x%X, GID=0x%02X, OID=0x%02X, Len=%d, "
             "Status=0x%X",
             rsp.get_mt(), rsp.get_gid(), rsp.get_oid(), rsp.get_len(),
             rsp.get_status());
    return rsp.get_status();
}

bool PN7160_NCI::try_dispatch_sync(const NciMessage& msg) {
    ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(10));
    if (!lock.acquired() || !sync_pending_.load()) return false;

    if (sync_type_ == SyncWaitType::APDU && msg.get_mt() == nci::PKT_MT_DATA) {
        size_t plen = msg.get_len();
        if (plen > nci::NCI_MAX_PAYLOAD_SIZE) plen = nci::NCI_MAX_PAYLOAD_SIZE;
        sync_apdu_response_len_ = plen;
        for (size_t i = 0; i < plen; ++i) {
            sync_apdu_response_[i] = msg[nci::NCI_HEADER_SIZE + i];
        }
        sync_pending_.store(false);
        sync_type_ = SyncWaitType::NONE;
        xSemaphoreGive(sync_sem_);
        return true;
    }

    if (sync_type_ == SyncWaitType::CMD && msg.get_mt() == nci::PKT_MT_CTRL_RESPONSE) {
        sync_cmd_response_ = msg;
        sync_pending_.store(false);
        sync_type_ = SyncWaitType::NONE;
        xSemaphoreGive(sync_sem_);
        return true;
    }

    return false;
}

void PN7160_NCI::signal_sync_failure() {
    ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(10));
    if (lock.acquired() && sync_pending_.load()) {
        ESP_LOGE(TAG, "Signaling sync failure due to transport error");
        sync_pending_.store(false);
        sync_type_ = SyncWaitType::NONE;
        sync_apdu_response_len_ = 0;
        sync_cmd_response_.clear();
        xSemaphoreGive(sync_sem_);
    }
}

// Returns NCI Status Code or ESP error code
esp_err_t PN7160_NCI::send_command_wait_response(NciMessage& cmd,
                                                   NciMessage& rsp,
                                                   uint32_t timeout_ms) {
    if (!initialized_.load() && cmd.get_gid() == nci::CORE_GID &&
        cmd.get_oid() != nci::CORE_RESET_OID &&
        cmd.get_oid() != nci::CORE_INIT_OID) {
        ESP_LOGE(TAG,
                 "Driver not initialized, cannot send command GID=0x%02X, "
                 "OID=0x%02X",
                 cmd.get_gid(), cmd.get_oid());
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Sending CMD: GID=0x%02X, OID=0x%02X, Len=%d", cmd.get_gid(),
             cmd.get_oid(), cmd.get_len());

    bool use_sync = (task_handle_ != nullptr) &&
                    (xTaskGetCurrentTaskHandle() != task_handle_);

    if (!use_sync) {
        esp_err_t write_err = write_nci_packet(cmd);
        if (write_err != ESP_OK) return write_err;

        esp_err_t read_err = read_nci_packet(rsp, timeout_ms);
        if (read_err != ESP_OK) return read_err;

        return validate_response(cmd, rsp);
    }

    // --- Sync path: task_runner is running, we're in a different task ---
    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGE(TAG, "send_command_wait_response: Could not acquire sync mutex.");
        return ESP_FAIL;
    }
    if (sync_pending_.load()) {
        ESP_LOGE(TAG, "send_command_wait_response: Another sync operation is already in progress.");
        xSemaphoreGive(sync_mutex_);
        return ESP_ERR_INVALID_STATE;
    }
    sync_pending_.store(true);
    sync_type_ = SyncWaitType::CMD;
    sync_cmd_response_.clear();
    xSemaphoreTake(sync_sem_, 0); // Drain stale
    xSemaphoreGive(sync_mutex_);

    esp_err_t write_err = write_nci_packet(cmd);
    if (write_err != ESP_OK) {
        sync_pending_.store(false);
        return write_err;
    }

    if (xSemaphoreTake(sync_sem_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(50));
        if (!lock.acquired()) {
            sync_pending_.store(false);
            return ESP_FAIL;
        }
        if (sync_cmd_response_.empty() ||
            !sync_cmd_response_.is_control_response(cmd.get_gid(), cmd.get_oid())) {
            ESP_LOGE(TAG,
                     "Unexpected NCI packet received. Expected RSP for GID=0x%02X, "
                     "OID=0x%02X. Got MT=%d, GID=0x%02X, OID=0x%02X",
                     cmd.get_gid(), cmd.get_oid(),
                     sync_cmd_response_.get_mt(),
                     sync_cmd_response_.get_gid(),
                     sync_cmd_response_.get_oid());
            return nci::STATUS_FAILED;
        }
        rsp = sync_cmd_response_;
        ESP_LOGD(TAG,
                 "Received RSP: MT=0x%X, GID=0x%02X, OID=0x%02X, Len=%d, "
                 "Status=0x%X",
                 rsp.get_mt(), rsp.get_gid(), rsp.get_oid(), rsp.get_len(),
                 rsp.get_status());
        return rsp.get_status();
    }

    ESP_LOGE(TAG, "send_command_wait_response: Timeout waiting for RSP.");
    sync_pending_.store(false);
    return ESP_ERR_TIMEOUT;
}

esp_err_t PN7160_NCI::send_data_packet(const NciMessage& data_pkt) {
    if (data_pkt.get_mt() != nci::PKT_MT_DATA) {
        ESP_LOGE(TAG, "Message is not an NCI Data packet (MT=%d)",
                 data_pkt.get_mt());
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Sending DATA: Len=%d", data_pkt.get_len());
    return write_nci_packet(data_pkt);
}

esp_err_t PN7160_NCI::send_apdu_sync(const std::vector<uint8_t> &c_apdu,
                                     std::vector<uint8_t> &r_apdu,
                                     uint32_t timeout_ms) {
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "send_apdu_sync: Driver not initialized.");
        return ESP_ERR_INVALID_STATE;
    }
    if (c_apdu.empty()) {
        ESP_LOGE(TAG, "send_apdu_sync: Command APDU cannot be empty.");
        return ESP_ERR_INVALID_ARG;
    }

    r_apdu.clear();

    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGE(TAG, "send_apdu_sync: Could not acquire sync mutex.");
        return ESP_FAIL;
    }
    if (sync_pending_.load()) {
        ESP_LOGE(TAG, "send_apdu_sync: Another synchronous operation is already in progress.");
        xSemaphoreGive(sync_mutex_);
        return ESP_ERR_INVALID_STATE;
    }
    sync_pending_.store(true);
    sync_type_ = SyncWaitType::APDU;
    sync_apdu_response_len_ = 0;
    xSemaphoreTake(sync_sem_, 0);
    xSemaphoreGive(sync_mutex_);

    NciMessage command_msg;
    command_msg.build_data(c_apdu);

    ESP_LOGD(TAG, "send_apdu_sync: Sending C-APDU (%zu bytes)", c_apdu.size());
    esp_err_t send_ret = send_data_packet(command_msg);
    if (send_ret != ESP_OK) {
        ESP_LOGE(TAG, "send_apdu_sync: Failed to send NCI Data Packet (err=0x%X)", send_ret);
        sync_pending_.store(false);
        return send_ret;
    }

    if (xSemaphoreTake(sync_sem_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(50));
        if (!lock.acquired()) {
            sync_pending_.store(false);
            return ESP_FAIL;
        }
        if (sync_apdu_response_len_ == 0) {
            ESP_LOGE(TAG, "send_apdu_sync: Semaphore received but response buffer is empty!");
            sync_pending_.store(false);
            return ESP_FAIL;
        }
        r_apdu.assign(sync_apdu_response_.data(),
                      sync_apdu_response_.data() + sync_apdu_response_len_);
        ESP_LOGD(TAG, "send_apdu_sync: R-APDU received (%zu bytes)", r_apdu.size());
        sync_pending_.store(false);
        return nci::STATUS_OK;
    }

    ESP_LOGE(TAG, "send_apdu_sync: Timeout waiting for R-APDU response.");
    sync_pending_.store(false);
    return ESP_ERR_TIMEOUT;
}

// --- High Level NCI Commands ---

esp_err_t PN7160_NCI::core_reset(bool reset_config, NciMessage& cmd, NciMessage& rsp, NciMessage& ntf) {
    if (transport.has_ven()) {
        transport.set_ven(false);
        vTaskDelay(pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
        transport.set_ven(true);
        vTaskDelay(pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
        ESP_LOGD(TAG, "Power cycle reset complete.");
    }

    if (transport.read_irq_level()) {
        ESP_LOGW(TAG, "IRQ high after power cycle, clearing pending message");
        ntf.clear();
        (void)read_nci_packet(ntf, 50);
    }

    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_RESET_OID,
              {(uint8_t)(reset_config ? 0x01 : 0x00)});
    rsp.clear();
    ntf.clear();
    ESP_LOGD(TAG, "Sending CORE_RESET_CMD (Reset Config: %d)", reset_config);

    esp_err_t write_err = write_nci_packet(cmd);
    if (write_err != ESP_OK) {
        ESP_LOGE(TAG, "Error sending CORE_RESET_CMD (err=0x%X)", write_err);
        return nci::STATUS_FAILED;
    }

    esp_err_t read_err = read_nci_packet(rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (read_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_RSP (err=0x%X)", read_err);
        return nci::STATUS_FAILED;
    }
    if (!rsp.is_control_response(nci::CORE_GID, nci::CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after CORE_RESET_CMD (expected RSP)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return nci::STATUS_FAILED;
    }

    uint8_t rsp_status = rsp.get_status();
    ESP_LOGD(TAG, "Received CORE_RESET_RSP (status: 0x%X)", rsp_status);

    ESP_LOGD(TAG, "Expecting CORE_RESET_NTF...");
    read_err = read_nci_packet(ntf, nci::PN7160_INIT_TIMEOUT_MS);
    if (read_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_NTF (err=0x%X)", read_err);
        return nci::STATUS_FAILED;
    }
    if (!ntf.is_control_notification(nci::CORE_GID, nci::CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after CORE_RESET_RSP (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, ntf.data(), ntf.size(), ESP_LOG_ERROR);
        return nci::STATUS_FAILED;
    }

    ESP_LOGD(TAG, "Received CORE_RESET_NTF");
    if (ntf.get_len() >= 2) {
        uint8_t ntf_status = ntf[3];
        uint8_t nci_version = ntf[4];
        ESP_LOGD(TAG, "CORE_RESET_NTF Info: Status=0x%02X", ntf_status);
        ESP_LOGI(TAG, "NCI Version: %u.%u", nci_version >> 4, nci_version & 0x0F);

        if (ntf.get_len() >= 3) {
            uint8_t manuf_len = ntf[5];
            if (manuf_len > 0 && ntf.get_len() >= 3 + manuf_len) {
                ESP_LOGI(TAG, "Manufacturer Specific Info (%d bytes):", manuf_len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, ntf.data() + 6, manuf_len, ESP_LOG_INFO);
                if (manuf_len >= 4) {
                    ESP_LOGI(TAG, "  Ver code: %02x, FW Ver: %02x.%02x.%02x",
                             ntf[6], ntf[7], ntf[8], ntf[9]);
                }
            } else if (manuf_len == 0) {
                ESP_LOGI(TAG, "No Manufacturer Specific Info present in NTF.");
            } else {
                ESP_LOGW(TAG, "CORE_RESET_NTF manuf_spec_info length mismatch (len=%d, payload=%d)",
                         manuf_len, ntf.get_len());
            }
        }
    } else {
        ESP_LOGW(TAG, "CORE_RESET_NTF payload too short (%d bytes)", ntf.get_len());
    }

    if (rsp_status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "CORE_RESET_RSP returned status 0x%X", rsp_status);
    }

    return rsp_status;
}

esp_err_t PN7160_NCI::core_init(NciMessage& cmd, NciMessage& rsp) {
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_INIT_OID);
    rsp.clear();
    ESP_LOGD(TAG, "Sending CORE_INIT");
    esp_err_t nci_status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (nci_status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "CORE_INIT failed (NCI Status=0x%X)", nci_status);
        return nci_status;
    }

    if (rsp.size() > 20) {
        ESP_LOGD(TAG, "Hardware version: %u", rsp[17]);
        ESP_LOGD(TAG, "ROM code version: %u", rsp[18]);
        ESP_LOGD(TAG, "FLASH major version: %u", rsp[19]);
        ESP_LOGD(TAG, "FLASH minor version: %u", rsp[20]);
    } else {
        ESP_LOGW(TAG, "CORE_INIT_RSP too short (%zu bytes) to parse version info", rsp.size());
    }

    if (rsp.size() >= 8) {
        ESP_LOGD(TAG, "Features: %02X %02X %02X %02X",
                 rsp[4], rsp[5], rsp[6], rsp[7]);
    }
    return nci_status;
}

esp_err_t PN7160_NCI::core_set_config(const std::vector<uint8_t>& config_params) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID,
                   nci::CORE_SET_CONFIG_OID, config_params);
    NciMessage rsp;
    esp_err_t nci_status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (nci_status == nci::STATUS_INVALID_PARAM) {
        ESP_LOGW(TAG, "CORE_SET_CONFIG_RSP indicates invalid parameters:");
        if (rsp.get_len() > 1) {
            ESP_LOGW(TAG, "  Num Invalid: %d", rsp[4]);
            if (rsp.get_len() > 2) {
                 ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data() + 5, rsp.get_len() - 2, ESP_LOG_WARN);
            }
        } else {
             ESP_LOGW(TAG, "  (Payload too short to list invalid IDs)");
        }
    } else if (nci_status != nci::STATUS_OK) {
         ESP_LOGE(TAG, "CORE_SET_CONFIG failed (NCI Status=0x%X)", nci_status);
    }
    return nci_status;
}

esp_err_t PN7160_NCI::rf_discover_map(const std::vector<uint8_t>& mappings) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_MAP_OID, mappings);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_set_listen_mode_routing(
    const std::vector<uint8_t>& routing_config) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_SET_LISTEN_MODE_ROUTING_OID, routing_config);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_start_discovery(
    const std::vector<uint8_t>& discovery_config) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_OID, discovery_config);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_stop_discovery() {
    return rf_deactivate(nci::DEACTIVATION_TYPE_IDLE);
}

esp_err_t PN7160_NCI::rf_select_target(uint8_t discovery_id, uint8_t protocol,
                                       uint8_t interface) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_SELECT_OID,
                   {discovery_id, protocol, interface});
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_deactivate(uint8_t type) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DEACTIVATE_OID, {type});
    NciMessage rsp;

    esp_err_t status = send_command_wait_response(cmd, rsp, 1000);
    if (status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "RF_DEACTIVATE_CMD failed (NCI Status=0x%X)", status);
        return status;
    }

    ESP_LOGD(TAG, "RF_DEACTIVATE_RSP OK");
    return nci::STATUS_OK;
}

esp_err_t PN7160_NCI::rf_iso_dep_presence_check() {
    if (!initialized_.load()) return ESP_ERR_INVALID_STATE;

    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_ISO_DEP_NAK_PRESENCE_OID);
    NciMessage rsp;
    ESP_LOGD(TAG, "Sending RF_ISO_DEP_NAK_PRESENCE_CMD");

    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "RF_ISO_DEP_NAK_PRESENCE_CMD failed (NCI Status=0x%X)", nci_status);
        return nci_status;
    }

    return nci_status;
}

// --- Lifecycle ---

void PN7160_NCI::shutdown() {
    stop_flag_.store(true);
    initialized_.store(false);
}

// --- Task Runner Dispatch Helpers ---

void PN7160_NCI::dispatch_data_packet(const NciMessage& msg) {
    ESP_LOGD(TAG, "Received DATA packet, Len=%d", msg.get_len());
    post_event(NciEventType::DATA_PACKET, msg);
}

void PN7160_NCI::handle_rf_notification_oid(uint8_t oid, const NciMessage& msg) {
    switch (oid) {
        case nci::RF_INTF_ACTIVATED_OID:
            ESP_LOGD(TAG, "RF_INTF_ACTIVATED_NTF: DiscID=0x%02X Intf=0x%02X Proto=0x%02X Mode=0x%02X",
                    msg[3], msg[4], msg[5], msg[6]);
            selected_tag_still_in_field.store(true);
            post_event(NciEventType::RF_INTF_ACTIVATED, msg);
            break;
        case nci::RF_DISCOVER_OID:
            ESP_LOGD(TAG, "RF_DISCOVER_NTF received");
            post_event(NciEventType::RF_DISCOVER, msg);
            break;
        case nci::RF_DEACTIVATE_OID: {
            if (msg.get_len() >= 2) {
                uint8_t deact_type = msg[3];
                uint8_t deact_reason = msg[4];
                ESP_LOGD(TAG, "RF_DEACTIVATE_NTF received. Type: 0x%02X, Reason: 0x%02X", deact_type, deact_reason);
                if (deact_reason == 0x02) {
                    ESP_LOGW(TAG, "Tag removed (RF Link Loss detected by NFCC)!");
                }
                post_event(NciEventType::RF_DEACTIVATE, msg);
            } else {
                ESP_LOGW(TAG, "RF_DEACTIVATE_NTF payload too short (%d bytes)", msg.get_len());
            }
            break;
        }
        case nci::RF_ISO_DEP_NAK_PRESENCE_OID: {
            esp_log_buffer_hexdump_internal(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);
            uint8_t presence = msg[3];
            if (presence != 0x00) {
                ESP_LOGW(TAG, "Tag no longer in field");
                selected_tag_still_in_field.store(false);
                (void)rf_deactivate(nci::DEACTIVATION_TYPE_DISCOVERY);
            } else {
                ESP_LOGD(TAG, "Tag still in field");
                (void)rf_iso_dep_presence_check();
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandled RF Notification OID: 0x%02X", oid);
            break;
    }
}

void PN7160_NCI::handle_core_oid(uint8_t oid, const NciMessage& msg, bool is_response) {
    switch (oid) {
        case nci::CORE_GENERIC_ERROR_OID:
            ESP_LOGW(TAG, "CORE_GENERIC_ERROR_%s Status: 0x%X",
                     is_response ? "RSP" : "NTF", msg.get_status());
            if (!is_response) post_event(NciEventType::CORE_NOTIFICATION, msg);
            break;
        case nci::CORE_INTERFACE_ERROR_OID:
            ESP_LOGW(TAG, "CORE_INTERFACE_ERROR_%s Status: 0x%X ConnID: %d",
                     is_response ? "RSP" : "NTF", msg[3], msg[4]);
            selected_tag_still_in_field.store(true);
            if (!is_response) post_event(NciEventType::CORE_NOTIFICATION, msg);
            break;
        case nci::CORE_CONN_CREDITS_OID:
            ESP_LOGD(TAG, "CORE_CONN_CREDITS_%s received", is_response ? "RSP" : "NTF");
            if (!is_response) post_event(NciEventType::CORE_NOTIFICATION, msg);
            break;
        default:
            ESP_LOGW(TAG, "Unhandled Core %s OID: 0x%02X",
                     is_response ? "RSP" : "NTF", oid);
            if (!is_response) post_event(NciEventType::CORE_NOTIFICATION, msg);
            break;
    }
}

void PN7160_NCI::dispatch_control_notification(const NciMessage& msg) {
    ESP_LOGD(TAG, "Received NTF: GID=0x%02X, OID=0x%02X",
             msg.get_gid(), msg.get_oid());

    if (msg.get_gid() == nci::RF_GID) {
        handle_rf_notification_oid(msg.get_oid(), msg);
    } else if (msg.get_gid() == nci::CORE_GID) {
        handle_core_oid(msg.get_oid(), msg, false);
    } else {
        ESP_LOGW(TAG, "Unhandled Notification GID: 0x%02X", msg.get_gid());
    }
}

void PN7160_NCI::dispatch_control_response(const NciMessage& msg) {
    if (msg.get_gid() == nci::RF_GID) {
        switch (msg.get_oid()) {
            case nci::RF_INTF_ACTIVATED_OID:
                selected_tag_still_in_field.store(true);
                post_event(NciEventType::RF_INTF_ACTIVATED, msg);
                break;
            case nci::RF_ISO_DEP_NAK_PRESENCE_OID:
                if (msg.get_len() >= 1) {
                    uint8_t presence_status = msg[3];
                    ESP_LOGD(TAG, "RF_ISO_DEP_NAK_PRESENCE_RSP received. Status: 0x%02X", presence_status);
                    if (presence_status != nci::STATUS_OK) {
                        (void)rf_deactivate(nci::DEACTIVATION_TYPE_DISCOVERY);
                    }
                } else {
                    ESP_LOGW(TAG, "RF_ISO_DEP_NAK_PRESENCE_RSP payload too short.");
                }
                break;
            default:
                ESP_LOGW(TAG, "Unexpected RSP received in task loop: GID=0x%02X, OID=0x%02X",
                        msg.get_gid(), msg.get_oid());
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_WARN);
                break;
        }
    } else if (msg.get_gid() == nci::CORE_GID) {
        handle_core_oid(msg.get_oid(), msg, true);
    } else {
        ESP_LOGW(TAG, "Unexpected RSP received in task loop: GID=0x%02X, OID=0x%02X",
                msg.get_gid(), msg.get_oid());
        ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_WARN);
    }
}

// --- Task Runner ---

void PN7160_NCI::task_runner() {
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "Task runner started but driver not initialized!");
        vTaskDelete(NULL);
        return;
    }
    task_handle_ = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "PN7160 task runner started.");

    NciMessage incoming_msg;
    esp_err_t ret = 0;
    while (!stop_flag_.load()) {
        ret = transport.wait_for_irq(true, pdMS_TO_TICKS(100));
        if (ret == ESP_OK) {
            ret = read_nci_packet(incoming_msg, nci::PN7160_DEFAULT_TIMEOUT_MS);

            if (ret == ESP_OK) {
                if (!try_dispatch_sync(incoming_msg)) {
                    switch (incoming_msg.get_mt()) {
                        case nci::PKT_MT_CTRL_NOTIFICATION:
                            dispatch_control_notification(incoming_msg);
                            break;
                        case nci::PKT_MT_CTRL_RESPONSE:
                            dispatch_control_response(incoming_msg);
                            break;
                        case nci::PKT_MT_DATA:
                            dispatch_data_packet(incoming_msg);
                            break;
                        default:
                            ESP_LOGW(TAG, "Unknown NCI Message Type received: 0x%X", incoming_msg.get_mt());
                            break;
                    }
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Timeout reading NCI packet after IRQ trigger");
            } else {
                ESP_LOGE(TAG, "Error reading NCI packet: 0x%X", ret);
                signal_sync_failure();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Error waiting for IRQ semaphore (returned %#X)", ret);
            signal_sync_failure();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "PN7160 task runner stopping.");
    task_handle_ = nullptr;
    vTaskDelete(NULL);
}
