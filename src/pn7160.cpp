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

// =============================================================================
// Constructor / Destructor
// =============================================================================

PN7160_NCI::PN7160_NCI(IPN7160Transport& transport)
    : transport_(transport) {
    sync_sem_   = xSemaphoreCreateBinary();
    sync_mutex_ = xSemaphoreCreateMutex();
    transport_mutex_  = xSemaphoreCreateMutex();

    if (!sync_sem_ || !sync_mutex_ || !transport_mutex_ || !event_ring_.valid()) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        if (sync_sem_)   vSemaphoreDelete(sync_sem_);
        if (sync_mutex_) vSemaphoreDelete(sync_mutex_);
        if (transport_mutex_)  vSemaphoreDelete(transport_mutex_);
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
    if (transport_mutex_) {
        vSemaphoreDelete(transport_mutex_);
        transport_mutex_ = nullptr;
    }
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t PN7160_NCI::initialize() {
    ESP_LOGI(TAG, "Initializing PN7160...");

    esp_err_t ret = transport_.init();
    if (ret != ESP_OK) return ret;

    // NCI Core Reset
    {
        NciMessage cmd, rsp, ntf;
        ret = core_reset(true, cmd, rsp, ntf);
        if (ret != nci::STATUS_OK) {
            ESP_LOGE(TAG, "NCI Core Reset failed (NCI Status=0x%02X)", ret);
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
        ESP_LOGE(TAG, "NCI Core Init failed (NCI Status=0x%02X)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Init successful");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Activate Proprietary Extensions
    {
        NciMessage cmd, rsp;
        cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::PROPRIETARY_GID,
                  nci::CORE_SET_CONFIG_OID);
        ret = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
        if (ret == ESP_FAIL) return ESP_FAIL;
        if (ret != nci::STATUS_OK) {
            ESP_LOGW(TAG, "Proprietary extensions not activated (NCI Status=0x%02X). "
                         "Some configs may fail.", ret);
        } else {
            ESP_LOGI(TAG, "Proprietary extensions activated.");
            ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.get_payload_ptr(), rsp.get_len(), ESP_LOG_DEBUG);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    stop_flag_.store(false);
    initialized_.store(true);
    ESP_LOGI(TAG, "PN7160 Initialized Successfully");
    return ESP_OK;
}

// =============================================================================
// NCI Packet Read / Write
// (After task_runner starts, only task_runner calls read_nci_packet.)
// =============================================================================

esp_err_t PN7160_NCI::read_nci_packet(NciMessage& msg, uint32_t timeout_ms) {
    msg.clear();
    uint8_t header[nci::NCI_HEADER_SIZE];

    esp_err_t ret = transport_.wait_for_irq(true, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (transport_.read_irq_level()) {
            ESP_LOGW(TAG, "IRQ became HIGH right after timeout check in read_nci_packet");
        } else {
            ESP_LOGD(TAG, "Timeout waiting for IRQ high before read");
            return ESP_ERR_TIMEOUT;
        }
    }

    ESP_RETURN_ON_ERROR(transport_.read(header, nci::NCI_HEADER_SIZE), TAG,
                        "Failed to read NCI header");

    msg.assign(header, header + nci::NCI_HEADER_SIZE);
    uint8_t payload_len = msg.get_len();

    ESP_LOGD(TAG, "Read Header: MT=0x%X GID=0x%X OID=0x%X Len=%d",
             msg.get_mt(), msg.get_gid(), msg.get_oid(), payload_len);

    if (payload_len > nci::NCI_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "NCI payload length too large: %d", payload_len);
        msg.clear();
        return ESP_FAIL;
    }

    if (payload_len > 0) {
        size_t current_size = msg.size();
        msg.resize(current_size + payload_len);
        ESP_RETURN_ON_ERROR(
            transport_.read(msg.data() + current_size, payload_len), TAG,
            "Failed to read NCI payload");
    }

    ESP_LOGD(TAG, "Read NCI Packet (%zu bytes):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);
    return ESP_OK;
}

esp_err_t PN7160_NCI::write_nci_packet(const NciMessage& msg) {
    if (msg.size() == 0 || msg.size() > nci::NCI_MAX_PACKET_SIZE)
        return ESP_ERR_INVALID_ARG;

    ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
    if (!lock.acquired()) {
        ESP_LOGE(TAG, "write_nci_packet: failed to acquire SPI mutex");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Write NCI Packet (%zu bytes):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);
    return transport_.write(msg.data(), msg.size());
}

// =============================================================================
// Synchronous Exchange – task_runner side
// =============================================================================

bool PN7160_NCI::try_complete_exchange(const NciMessage& msg) {
    // Fast-path: nothing pending.
    if (!sync_pending_.load()) return false;

    ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(10));
    if (!lock.acquired() || !sync_pending_.load()) return false;

    const SyncWaitType type = exchange_.type;

    // --- APDU: expect a data packet on the static RF connection ---
    if (type == SyncWaitType::APDU && msg.get_mt() == nci::PKT_MT_DATA) {
        size_t plen = msg.get_len();
        if (plen > nci::NCI_MAX_PAYLOAD_SIZE) plen = nci::NCI_MAX_PAYLOAD_SIZE;
        exchange_.apdu_result_len = plen;
        for (size_t i = 0; i < plen; ++i)
            exchange_.apdu_result[i] = msg[nci::NCI_HEADER_SIZE + i];
        exchange_.terminal_error = ESP_OK;
        sync_pending_.store(false);
        exchange_.type = SyncWaitType::NONE;
        xSemaphoreGive(sync_sem_);
        return true;
    }

    // --- CMD: expect a control response with matching GID + OID ---
    if (type == SyncWaitType::CMD &&
        msg.get_mt()  == nci::PKT_MT_CTRL_RESPONSE &&
        msg.get_gid() == exchange_.expected_gid &&
        msg.get_oid() == exchange_.expected_oid) {
        exchange_.cmd_result     = msg;
        exchange_.terminal_error = ESP_OK;
        sync_pending_.store(false);
        exchange_.type = SyncWaitType::NONE;
        xSemaphoreGive(sync_sem_);
        return true;
    }

    return false;
}

void PN7160_NCI::signal_sync_failure(esp_err_t err) {
    ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(10));
    if (!lock.acquired() || !sync_pending_.load()) return;

    ESP_LOGE(TAG, "Signaling sync failure (err=0x%X)", err);
    exchange_.terminal_error  = err;
    exchange_.apdu_result_len = 0;
    exchange_.cmd_result.clear();
    sync_pending_.store(false);
    exchange_.type = SyncWaitType::NONE;
    xSemaphoreGive(sync_sem_);
}

// =============================================================================
// Synchronous Exchange – caller side
// =============================================================================

esp_err_t PN7160_NCI::send_command_wait_response(NciMessage& cmd,
                                                  NciMessage& rsp,
                                                  uint32_t    timeout_ms) {
    ESP_LOGD(TAG, "Sending CMD: GID=0x%02X OID=0x%02X Len=%d",
             cmd.get_gid(), cmd.get_oid(), cmd.get_len());

    // --- Direct path: task_runner not yet running (e.g. during init) ---
    if (task_handle_ == nullptr ||
        xTaskGetCurrentTaskHandle() == task_handle_) {
        esp_err_t err = write_nci_packet(cmd);
        if (err != ESP_OK) return err;

        {
            ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(timeout_ms));
            if (!lock.acquired()) return ESP_ERR_TIMEOUT;
            err = read_nci_packet(rsp, timeout_ms);
        }
        if (err != ESP_OK) return err;

        // Validate GID + OID match before accepting.
        if (!rsp.is_control_response(cmd.get_gid(), cmd.get_oid())) {
            ESP_LOGE(TAG,
                     "Unexpected RSP: expected GID=0x%02X OID=0x%02X, "
                     "got MT=%d GID=0x%02X OID=0x%02X",
                     cmd.get_gid(), cmd.get_oid(),
                     rsp.get_mt(), rsp.get_gid(), rsp.get_oid());
            ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
            return static_cast<esp_err_t>(nci::STATUS_FAILED);
        }

        ESP_LOGD(TAG, "RSP: MT=0x%X GID=0x%02X OID=0x%02X Len=%d Status=0x%02X",
                 rsp.get_mt(), rsp.get_gid(), rsp.get_oid(),
                 rsp.get_len(), rsp.get_status());
        return static_cast<esp_err_t>(rsp.get_status());
    }

    // --- Sync path: task_runner owns the transport; register a slot ---
    SyncSlot slot(sync_mutex_, sync_sem_, sync_pending_, exchange_,
                  SyncWaitType::CMD, cmd.get_gid(), cmd.get_oid());
    if (!slot.valid()) {
        ESP_LOGE(TAG, "send_command_wait_response: could not acquire sync slot "
                      "(another operation in progress or mutex timeout)");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = write_nci_packet(cmd);
    if (err != ESP_OK) {
        // SyncSlot destructor clears sync_pending_ for us.
        return err;
    }

    if (slot.wait(timeout_ms) != pdTRUE) {
        ESP_LOGE(TAG, "send_command_wait_response: timeout waiting for RSP "
                      "GID=0x%02X OID=0x%02X", cmd.get_gid(), cmd.get_oid());
        return ESP_ERR_TIMEOUT;
    }

    // Check for terminal error (transport failure signalled by task_runner).
    {
        ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(50));
        if (!lock.acquired()) return ESP_FAIL;
        if (exchange_.terminal_error != ESP_OK)
            return exchange_.terminal_error;
        if (exchange_.cmd_result.empty()) {
            ESP_LOGE(TAG, "send_command_wait_response: semaphore fired but "
                         "result is empty");
            return ESP_FAIL;
        }
        rsp = exchange_.cmd_result;
    }

    ESP_LOGD(TAG, "RSP: MT=0x%X GID=0x%02X OID=0x%02X Len=%d Status=0x%02X",
             rsp.get_mt(), rsp.get_gid(), rsp.get_oid(),
             rsp.get_len(), rsp.get_status());
    return static_cast<esp_err_t>(rsp.get_status());
}

esp_err_t PN7160_NCI::send_data_packet(const NciMessage& data_pkt) {
    if (data_pkt.get_mt() != nci::PKT_MT_DATA) {
        ESP_LOGE(TAG, "send_data_packet: message is not a data packet (MT=%d)",
                 data_pkt.get_mt());
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Sending DATA: Len=%d", data_pkt.get_len());
    return write_nci_packet(data_pkt);
}

esp_err_t PN7160_NCI::send_apdu_sync(const std::vector<uint8_t>& c_apdu,
                                     std::vector<uint8_t>&       r_apdu,
                                     uint32_t                    timeout_ms) {
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "send_apdu_sync: driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (c_apdu.empty()) {
        ESP_LOGE(TAG, "send_apdu_sync: C-APDU cannot be empty");
        return ESP_ERR_INVALID_ARG;
    }

    r_apdu.clear();

    SyncSlot slot(sync_mutex_, sync_sem_, sync_pending_, exchange_,
                  SyncWaitType::APDU);
    if (!slot.valid()) {
        ESP_LOGE(TAG, "send_apdu_sync: could not acquire sync slot");
        return ESP_ERR_INVALID_STATE;
    }

    NciMessage cmd;
    cmd.build_data(c_apdu);

    ESP_LOGD(TAG, "send_apdu_sync: sending C-APDU (%zu bytes)", c_apdu.size());
    esp_err_t err = send_data_packet(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "send_apdu_sync: failed to send NCI data packet (err=0x%X)", err);
        // SyncSlot destructor clears sync_pending_.
        return err;
    }

    if (slot.wait(timeout_ms) != pdTRUE) {
        ESP_LOGE(TAG, "send_apdu_sync: timeout waiting for R-APDU");
        return ESP_ERR_TIMEOUT;
    }

    // Inspect result under the mutex.
    {
        ScopedMutex lock(sync_mutex_, pdMS_TO_TICKS(50));
        if (!lock.acquired()) return ESP_FAIL;

        if (exchange_.terminal_error != ESP_OK)
            return exchange_.terminal_error;

        if (exchange_.apdu_result_len == 0) {
            ESP_LOGE(TAG, "send_apdu_sync: semaphore fired but response buffer is empty");
            return ESP_FAIL;
        }

        r_apdu.assign(exchange_.apdu_result.data(),
                      exchange_.apdu_result.data() + exchange_.apdu_result_len);
    }

    ESP_LOGD(TAG, "send_apdu_sync: R-APDU received (%zu bytes)", r_apdu.size());
    return static_cast<esp_err_t>(nci::STATUS_OK);
}

// =============================================================================
// High-Level NCI Commands
// =============================================================================

esp_err_t PN7160_NCI::core_reset(bool reset_config,
                                  NciMessage& cmd,
                                  NciMessage& rsp,
                                  NciMessage& ntf) {
    if (transport_.has_ven()) {
        transport_.set_ven(false);
        vTaskDelay(pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
        transport_.set_ven(true);
        vTaskDelay(pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
        ESP_LOGD(TAG, "Power cycle reset complete.");
    }

    if (transport_.read_irq_level()) {
        ESP_LOGW(TAG, "IRQ high after power cycle – clearing pending message");
        ntf.clear();
        {
            ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
            if (!lock.acquired()) return ESP_ERR_TIMEOUT;
            (void)read_nci_packet(ntf, 50);
        }
    }

    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_RESET_OID,
              {(uint8_t)(reset_config ? 0x01 : 0x00)});
    rsp.clear();
    ntf.clear();
    ESP_LOGD(TAG, "Sending CORE_RESET_CMD (Reset Config: %d)", reset_config);

    esp_err_t err = write_nci_packet(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error sending CORE_RESET_CMD (err=0x%X)", err);
        return nci::STATUS_FAILED;
    }

    {
        ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(nci::PN7160_INIT_TIMEOUT_MS));
        if (!lock.acquired()) return ESP_ERR_TIMEOUT;
        err = read_nci_packet(rsp, nci::PN7160_INIT_TIMEOUT_MS);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_RSP (err=0x%X)", err);
        return nci::STATUS_FAILED;
    }
    if (!rsp.is_control_response(nci::CORE_GID, nci::CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Unexpected packet after CORE_RESET_CMD (expected RSP)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return nci::STATUS_FAILED;
    }

    uint8_t rsp_status = rsp.get_status();
    ESP_LOGD(TAG, "CORE_RESET_RSP (status: 0x%02X)", rsp_status);

    {
        ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(nci::PN7160_INIT_TIMEOUT_MS));
        if (!lock.acquired()) return ESP_ERR_TIMEOUT;
        err = read_nci_packet(ntf, nci::PN7160_INIT_TIMEOUT_MS);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_NTF (err=0x%X)", err);
        return nci::STATUS_FAILED;
    }
    if (!ntf.is_control_notification(nci::CORE_GID, nci::CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Unexpected packet after CORE_RESET_RSP (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, ntf.data(), ntf.size(), ESP_LOG_ERROR);
        return nci::STATUS_FAILED;
    }

    ESP_LOGD(TAG, "CORE_RESET_NTF received");
    if (ntf.get_len() >= 2) {
        uint8_t ntf_status  = ntf[3];
        uint8_t nci_version = ntf[4];
        ESP_LOGD(TAG, "CORE_RESET_NTF: Status=0x%02X", ntf_status);
        ESP_LOGI(TAG, "NCI Version: %u.%u", nci_version >> 4, nci_version & 0x0F);

        if (ntf.get_len() >= 3) {
            uint8_t manuf_len = ntf[5];
            if (manuf_len > 0 && ntf.get_len() >= 3 + manuf_len) {
                ESP_LOGI(TAG, "Manufacturer Specific Info (%d bytes):", manuf_len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, ntf.data() + 6, manuf_len, ESP_LOG_INFO);
                if (manuf_len >= 4)
                    ESP_LOGI(TAG, "  Ver code: %02x, FW Ver: %02x.%02x.%02x",
                             ntf[6], ntf[7], ntf[8], ntf[9]);
            } else if (manuf_len == 0) {
                ESP_LOGI(TAG, "No Manufacturer Specific Info in NTF.");
            } else {
                ESP_LOGW(TAG, "CORE_RESET_NTF manuf_spec_info length mismatch "
                             "(len=%d, payload=%d)", manuf_len, ntf.get_len());
            }
        }
    } else {
        ESP_LOGW(TAG, "CORE_RESET_NTF payload too short (%d bytes)", ntf.get_len());
    }

    if (rsp_status != nci::STATUS_OK)
        ESP_LOGE(TAG, "CORE_RESET_RSP returned status 0x%02X", rsp_status);

    return rsp_status;
}

esp_err_t PN7160_NCI::core_init(NciMessage& cmd, NciMessage& rsp) {
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_INIT_OID);
    rsp.clear();
    ESP_LOGD(TAG, "Sending CORE_INIT_CMD");
    esp_err_t status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "CORE_INIT failed (NCI Status=0x%02X)", status);
        return status;
    }

    if (rsp.size() > 20) {
        ESP_LOGD(TAG, "HW version: %u, ROM: %u, FLASH: %u.%u",
                 rsp[17], rsp[18], rsp[19], rsp[20]);
    } else {
        ESP_LOGW(TAG, "CORE_INIT_RSP too short (%zu bytes) to parse version info",
                 rsp.size());
    }

    if (rsp.size() >= 8)
        ESP_LOGD(TAG, "Features: %02X %02X %02X %02X",
                 rsp[4], rsp[5], rsp[6], rsp[7]);

    return status;
}

esp_err_t PN7160_NCI::core_set_config(const std::vector<uint8_t>& config_params) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID,
                   nci::CORE_SET_CONFIG_OID, config_params);
    NciMessage rsp;
    esp_err_t status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (status == nci::STATUS_INVALID_PARAM) {
        ESP_LOGW(TAG, "CORE_SET_CONFIG_RSP: invalid parameters");
        if (rsp.get_len() > 1) {
            ESP_LOGW(TAG, "  Num Invalid: %d", rsp[4]);
            if (rsp.get_len() > 2)
                ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data() + 5, rsp.get_len() - 2, ESP_LOG_WARN);
        }
    } else if (status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "CORE_SET_CONFIG failed (NCI Status=0x%02X)", status);
    }
    return status;
}

esp_err_t PN7160_NCI::core_get_config(const std::vector<uint8_t>& config_params, NciMessage& rsp) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID,
                   nci::CORE_GET_CONFIG_OID, config_params);
    esp_err_t status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (status == nci::STATUS_INVALID_PARAM) {
        ESP_LOGW(TAG, "CORE_GET_CONFIG_RSP: invalid parameters");
        if (rsp.get_len() > 1) {
            ESP_LOGW(TAG, "  Num Invalid: %d", rsp[4]);
            if (rsp.get_len() > 2)
                ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data() + 5, rsp.get_len() - 2, ESP_LOG_WARN);
        }
    } else if (status != nci::STATUS_OK) {
        ESP_LOGE(TAG, "CORE_GET_CONFIG failed (NCI Status=0x%02X)", status);
    }
    return status;
}

esp_err_t PN7160_NCI::rf_discover_map(const std::vector<uint8_t>& mappings) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_DISCOVER_MAP_OID, mappings);
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

esp_err_t PN7160_NCI::rf_start_discovery(const std::vector<uint8_t>& discovery_config) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_DISCOVER_OID, discovery_config);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_stop_discovery() {
    return rf_deactivate(nci::DEACTIVATION_TYPE_IDLE);
}

esp_err_t PN7160_NCI::rf_select_target(uint8_t discovery_id,
                                        uint8_t protocol,
                                        uint8_t interface) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_DISCOVER_SELECT_OID,
                   {discovery_id, protocol, interface});
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_deactivate(uint8_t type) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_DEACTIVATE_OID, {type});
    NciMessage rsp;
    esp_err_t status = send_command_wait_response(cmd, rsp, 1000);
    if (status != nci::STATUS_OK)
        ESP_LOGE(TAG, "RF_DEACTIVATE failed (NCI Status=0x%02X)", status);
    else
        ESP_LOGD(TAG, "RF_DEACTIVATE_RSP OK");
    return status;
}

esp_err_t PN7160_NCI::rf_iso_dep_presence_check() {
    if (!initialized_.load()) return ESP_ERR_INVALID_STATE;
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID,
                   nci::RF_ISO_DEP_NAK_PRESENCE_OID);
    ESP_LOGD(TAG, "Sending RF_ISO_DEP_NAK_PRESENCE_CMD");
    return write_nci_packet(cmd);
}

// =============================================================================
// App-facing event queue
// =============================================================================

esp_err_t PN7160_NCI::get_event(NciEvent& event, uint32_t timeout_ms) {
    return event_ring_.pop(event, timeout_ms);
}

// =============================================================================
// Lifecycle
// =============================================================================

void PN7160_NCI::shutdown() {
    stop_flag_.store(true);
    initialized_.store(false);
}

// =============================================================================
// Task-runner dispatch helpers
//
// All mutable driver state (tag_in_field_) is updated exclusively here,
// serialised through the task_runner loop.  The app-facing ring buffer is
// populated separately after state has been updated so consumers always see
// a consistent view.
// =============================================================================

void PN7160_NCI::update_driver_state(const NciMessage& msg) {
    if (msg.get_gid() != nci::RF_GID) return;

    switch (msg.get_oid()) {
        case nci::RF_INTF_ACTIVATED_OID:
            tag_in_field_.store(true);
            ESP_LOGD(TAG,
                     "RF_INTF_ACTIVATED_NTF: DiscID=0x%02X Intf=0x%02X "
                     "Proto=0x%02X Mode=0x%02X",
                     msg[3], msg[4], msg[5], msg[6]);
            break;

        case nci::RF_DEACTIVATE_OID:
            if (msg.get_len() >= 2) {
                uint8_t deact_reason = msg[4];
                if (deact_reason == 0x02)
                    ESP_LOGW(TAG, "Tag removed (RF Link Loss)!");
                ESP_LOGD(TAG, "RF_DEACTIVATE_NTF: Type=0x%02X Reason=0x%02X",
                         msg[3], deact_reason);
            }
            tag_in_field_.store(false);
            break;

        case nci::RF_ISO_DEP_NAK_PRESENCE_OID:
            if (msg.get_len() >= 1) {
                bool present = (msg[3] == 0x00);
                tag_in_field_.store(present);
                ESP_LOGD(TAG,
                         "RF_ISO_DEP_NAK_PRESENCE: status=0x%02X in_field=%d",
                         msg[3], present);
            }
            break;

        default:
            break;
    }
}

void PN7160_NCI::handle_rf_notification(uint8_t oid, const NciMessage& msg) {
    NciEventType evt_type;
    switch (oid) {
        case nci::RF_INTF_ACTIVATED_OID: evt_type = NciEventType::RF_INTF_ACTIVATED; break;
        case nci::RF_DISCOVER_OID:       evt_type = NciEventType::RF_DISCOVER;       break;
        case nci::RF_DEACTIVATE_OID:     evt_type = NciEventType::RF_DEACTIVATE;     break;
        case nci::RF_ISO_DEP_NAK_PRESENCE_OID:
            // Internal presence check result – update state only, do not
            // forward to the application event queue.
            return;
        default:
            ESP_LOGW(TAG, "Unhandled RF NTF OID: 0x%02X", oid);
            return;
    }

    if (!event_ring_.push({evt_type, msg}))
        ESP_LOGW(TAG, "Event ring full – dropped %s NTF (OID=0x%02X)",
                 (evt_type == NciEventType::RF_DEACTIVATE) ? "RF_DEACTIVATE" : "RF",
                 oid);
}

void PN7160_NCI::handle_core_notification(uint8_t oid, const NciMessage& msg) {
    switch (oid) {
        case nci::CORE_GENERIC_ERROR_OID:
            ESP_LOGW(TAG, "CORE_GENERIC_ERROR_NTF Status: 0x%02X", msg.get_status());
            break;
        case nci::CORE_INTERFACE_ERROR_OID:
            ESP_LOGW(TAG, "CORE_INTERFACE_ERROR_NTF Status: 0x%02X ConnID: %d",
                     msg[3], msg[4]);
            break;
        case nci::CORE_CONN_CREDITS_OID:
            ESP_LOGD(TAG, "CORE_CONN_CREDITS_NTF received");
            break;
        default:
            ESP_LOGW(TAG, "Unhandled Core NTF OID: 0x%02X", oid);
            break;
    }
    // All core notifications are forwarded at low priority.
    if (!event_ring_.push({NciEventType::CORE_NOTIFICATION, msg}))
        ESP_LOGW(TAG, "Event ring full – dropped CORE NTF (OID=0x%02X)", oid);
}

void PN7160_NCI::enqueue_notification(const NciMessage& msg) {
    ESP_LOGD(TAG, "NTF: GID=0x%02X OID=0x%02X", msg.get_gid(), msg.get_oid());

    update_driver_state(msg);

    if (msg.get_gid() == nci::RF_GID)
        handle_rf_notification(msg.get_oid(), msg);
    else if (msg.get_gid() == nci::CORE_GID)
        handle_core_notification(msg.get_oid(), msg);
    else
        ESP_LOGW(TAG, "Unhandled NTF GID: 0x%02X", msg.get_gid());
}

void PN7160_NCI::enqueue_data_packet(const NciMessage& msg) {
    ESP_LOGD(TAG, "DATA packet: Len=%d", msg.get_len());
    if (!event_ring_.push({NciEventType::DATA_PACKET, msg}))
        ESP_LOGW(TAG, "Event ring full – dropped DATA packet");
}

// Unsolicited RSPs that arrive in the task loop (not matched by
// try_complete_exchange) are handled here.  This is rare in normal operation.
// Called from task_runner for PKT_MT_CTRL_RESPONSE packets that were NOT
// consumed by try_complete_exchange (e.g. RF_ISO_DEP_NAK_PRESENCE_RSP
// which arrives asynchronously after rf_iso_dep_presence_check()).
static void handle_unsolicited_response(const NciMessage& msg,
                                        std::atomic<bool>& tag_in_field,
                                        const char* TAG) {
    if (msg.get_gid() == nci::RF_GID) {
        switch (msg.get_oid()) {
            case nci::RF_INTF_ACTIVATED_OID:
                tag_in_field.store(true);
                break;
            case nci::RF_ISO_DEP_NAK_PRESENCE_OID:
                if (msg.get_len() >= 1)
                    tag_in_field.store(msg[3] == nci::STATUS_OK);
                break;
            default:
                ESP_LOGW(TAG, "Unsolicited RSP: GID=0x%02X OID=0x%02X",
                         msg.get_gid(), msg.get_oid());
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_WARN);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Unsolicited RSP: GID=0x%02X OID=0x%02X",
                 msg.get_gid(), msg.get_oid());
        ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_WARN);
    }
}

// =============================================================================
// Task Runner
//
// Exclusively owns the transport read path after startup.
// All incoming packets flow through here:
//   1. try_complete_exchange() – wake a blocked caller if this is their RSP/APDU
//   2. Otherwise classify and dispatch to the app-facing ring buffer.
// =============================================================================

void PN7160_NCI::task_runner() {
    task_running_.store(true);
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "task_runner started but driver not initialized!");
        vTaskDelete(nullptr);
        return;
    }
    task_handle_ = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "PN7160 task runner started (handle=%p).", (void*)task_handle_);

    NciMessage incoming;
    while (!stop_flag_.load()) {
        esp_err_t ret = transport_.wait_for_irq(true, pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));

        if (ret == ESP_ERR_TIMEOUT) {
            // Normal idle – nothing to read.
            continue;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "IRQ wait error (0x%X)", ret);
            signal_sync_failure(ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        {
            ScopedMutex lock(transport_mutex_, pdMS_TO_TICKS(nci::PN7160_DEFAULT_TIMEOUT_MS));
            if (!lock.acquired()) {
                ESP_LOGE(TAG, "task_runner: failed to acquire SPI mutex");
                continue;
            }

            if (!transport_.read_irq_level()) {
                continue;
            }

            ret = read_nci_packet(incoming, nci::PN7160_DEFAULT_TIMEOUT_MS);
        }
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Timeout reading NCI packet after IRQ trigger");
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error reading NCI packet: 0x%X", ret);
            signal_sync_failure(ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (try_complete_exchange(incoming)) continue;

        switch (incoming.get_mt()) {
            case nci::PKT_MT_CTRL_NOTIFICATION:
                enqueue_notification(incoming);
                break;
            case nci::PKT_MT_CTRL_RESPONSE:
                handle_unsolicited_response(incoming, tag_in_field_, TAG);
                break;
            case nci::PKT_MT_DATA:
                enqueue_data_packet(incoming);
                break;
            default:
                ESP_LOGW(TAG, "Unknown NCI MT=0x%X", incoming.get_mt());
                break;
        }
    }

    ESP_LOGI(TAG, "PN7160 task runner stopping.");
    task_handle_ = nullptr;
    task_running_.store(false);
    vTaskDelete(NULL);
}
