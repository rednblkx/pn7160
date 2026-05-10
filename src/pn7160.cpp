#include "pn7160.hpp"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

// =============================================================================
// Constructor / Destructor
// =============================================================================

PN7160_NCI::PN7160_NCI(IPN7160Transport& transport)
    : transport_(transport) {
    if (!event_ring_.valid()) {
        ESP_LOGE(TAG, "Failed to create event ring buffer");
    }
}

PN7160_NCI::~PN7160_NCI() {
    stop();
}

// =============================================================================
// Initialization & Lifecycle
// =============================================================================

esp_err_t PN7160_NCI::initialize() {
    ESP_LOGI(TAG, "Initializing PN7160...");

    esp_err_t ret = transport_.init();
    if (ret != ESP_OK) return ret;

    ret = core_reset(true);
    if (ret != nci::STATUS_OK) {
        ESP_LOGE(TAG, "NCI Core Reset failed (NCI Status=0x%02X)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Reset successful");

    ret = core_init();
    if (ret != nci::STATUS_OK) {
        ESP_LOGE(TAG, "NCI Core Init failed (NCI Status=0x%02X)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Init successful");
    vTaskDelay(pdMS_TO_TICKS(10));

    {
        NciMessage cmd, rsp;
        cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::PROPRIETARY_GID, nci::CORE_SET_CONFIG_OID);
        ret = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
        if (ret == ESP_FAIL) return ESP_FAIL;
        if (ret != nci::STATUS_OK) {
            ESP_LOGW(TAG, "Proprietary extensions not activated (NCI Status=0x%02X).", ret);
        } else {
            ESP_LOGI(TAG, "Proprietary extensions activated.");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    stop_flag_.store(false);
    cancelled_.store(false);
    initialized_.store(true);
    ESP_LOGI(TAG, "PN7160 Initialized Successfully");
    return ESP_OK;
}

esp_err_t PN7160_NCI::start() {
    if (task_handle_) return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<PN7160_NCI*>(arg)->task_runner(); vTaskDelete(NULL); },
        "pn7160_runner", 4096, this, 4, &task_handle_, tskNO_AFFINITY);
    if (ok != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

void PN7160_NCI::stop() {
    shutdown();
    if (task_handle_) {
        int wait = 0;
        while (task_running_.load() && wait < 500) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ++wait;
        }
        if (task_running_.load()) {
            vTaskDelete(task_handle_);
            task_running_.store(false);
        }
        ESP_LOGI(TAG, "Task stopped");
        task_handle_ = nullptr;
    }
}

void PN7160_NCI::shutdown() {
    stop_flag_.store(true);
    cancelled_.store(true);
    initialized_.store(false);
    promise_.cancel();
    (void)event_ring_.push({NciEventType::SHUTDOWN, {}});
}

// =============================================================================
// NCI Packet Read / Write
// =============================================================================

esp_err_t PN7160_NCI::read_nci_packet(NciMessage& msg, uint32_t timeout_ms) {
    msg.clear();
    uint8_t header[nci::NCI_HEADER_SIZE];

    esp_err_t ret = transport_.wait_for_irq(true, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (transport_.read_irq_level()) {
            ESP_LOGW(TAG, "IRQ high after timeout – packet may be pending");
        } else {
            return ESP_ERR_TIMEOUT;
        }
    }

    ESP_RETURN_ON_ERROR(transport_.read(header, nci::NCI_HEADER_SIZE), TAG, "Failed to read NCI header");
    msg.assign(header, header + nci::NCI_HEADER_SIZE);

    uint8_t payload_len = msg.get_len();
    if (payload_len > nci::NCI_MAX_PAYLOAD_SIZE) return ESP_FAIL;

    if (payload_len > 0) {
        size_t current_size = msg.size();
        msg.resize(current_size + payload_len);
        ESP_RETURN_ON_ERROR(
            transport_.read(msg.data() + current_size, payload_len), TAG, "Failed to read NCI payload");
    }
    return ESP_OK;
}

esp_err_t PN7160_NCI::write_nci_packet(const NciMessage& msg) {
    if (msg.empty() || msg.size() > nci::NCI_MAX_PACKET_SIZE) return ESP_ERR_INVALID_ARG;
    return transport_.write(msg.data(), msg.size());
}

// =============================================================================
// Synchronous Exchange
// =============================================================================

bool PN7160_NCI::try_complete_exchange(const NciMessage& msg) {
    if (promise_.waiting_task.load(std::memory_order_acquire) == nullptr) return false;

    if (msg.get_mt() == nci::PKT_MT_DATA) {
        return promise_.complete(msg, ESP_OK);
    }

    if (msg.get_mt() == nci::PKT_MT_CTRL_RESPONSE &&
        msg.get_gid() == promise_.expected_gid &&
        msg.get_oid() == promise_.expected_oid) {
        return promise_.complete(msg, ESP_OK);
    }

    return false;
}

void PN7160_NCI::signal_sync_failure(esp_err_t err) {
    if (promise_.fail(err)) ESP_LOGE(TAG, "Signaled sync failure (err=0x%X)", err);
}

esp_err_t PN7160_NCI::send_command_wait_response(const NciMessage& cmd, NciMessage& rsp, uint32_t timeout_ms) {
    if (task_handle_ == nullptr || xTaskGetCurrentTaskHandle() == task_handle_) {
        esp_err_t err = write_nci_packet(cmd);
        if (err != ESP_OK) return err;

        err = read_nci_packet(rsp, timeout_ms);
        if (err != ESP_OK) return err;

        if (!rsp.is_control_response(cmd.get_gid(), cmd.get_oid())) return nci::STATUS_FAILED;
        return rsp.get_status();
    }

    if (!promise_.arm(cmd.get_gid(), cmd.get_oid())) return ESP_ERR_INVALID_STATE;

    if (auto err = write_nci_packet(cmd); err != ESP_OK) {
        promise_.cancel();
        return err;
    }

    esp_err_t err = promise_.wait(timeout_ms);
    if (err != ESP_OK) return err;

    if (promise_.result.empty()) return ESP_FAIL;

    rsp = promise_.result;
    return rsp.get_status();
}

std::expected<std::vector<uint8_t>, esp_err_t> PN7160_NCI::send_apdu_sync(
    std::span<const uint8_t> c_apdu, uint32_t timeout_ms)
{
    if (!initialized_.load()) return std::unexpected(ESP_ERR_INVALID_STATE);
    if (c_apdu.empty()) return std::unexpected(ESP_ERR_INVALID_ARG);

    if (!promise_.arm()) return std::unexpected(ESP_ERR_INVALID_STATE);

    NciMessage cmd;
    cmd.build_data(c_apdu);

    esp_err_t err = send_data_packet(cmd);
    if (err != ESP_OK) {
        promise_.cancel();
        return std::unexpected(err);
    }

    err = promise_.wait(timeout_ms);
    if (err != ESP_OK) return std::unexpected(err);

    if (promise_.result.empty()) return std::unexpected(ESP_FAIL);

    // Returns the vector copy securely to the Application Thread
    return promise_.result.get_payload_copy();
}

esp_err_t PN7160_NCI::send_data_packet(const NciMessage& data_pkt) {
    if (data_pkt.get_mt() != nci::PKT_MT_DATA) return ESP_ERR_INVALID_ARG;
    return write_nci_packet(data_pkt);
}

// =============================================================================
// High-Level NCI Commands
// =============================================================================

esp_err_t PN7160_NCI::core_reset(bool reset_config) {
    if (transport_.has_ven()) {
        transport_.set_ven(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        transport_.set_ven(true);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    NciMessage ntf;
    if (transport_.read_irq_level()) {
        (void)read_nci_packet(ntf, nci::PN7160_INIT_TIMEOUT_MS);
    }

    NciMessage cmd, rsp;
    uint8_t reset_byte = reset_config ? 0x01 : 0x00;
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_RESET_OID, std::span<const uint8_t>(&reset_byte, 1));
    rsp.clear();
    ntf.clear();

    esp_err_t err = write_nci_packet(cmd);
    if (err != ESP_OK) return nci::STATUS_FAILED;

    err = read_nci_packet(rsp, nci::PN7160_INIT_TIMEOUT_MS);
    if (err != ESP_OK || !rsp.is_control_response(nci::CORE_GID, nci::CORE_RESET_OID)) return nci::STATUS_FAILED;
    uint8_t rsp_status = rsp.get_status();

    err = read_nci_packet(ntf, nci::PN7160_INIT_TIMEOUT_MS);
    if (err != ESP_OK || !ntf.is_control_notification(nci::CORE_GID, nci::CORE_RESET_OID)) return nci::STATUS_FAILED;
    auto data = ntf.get_payload();
    if (data.size() == 9) {
        ESP_LOGI(TAG, "NCI Version: %x.%x", data[2] >> 4, data[2] & 0x0f);
        ESP_LOGI(TAG, "Manufacturer ID: %x", data[3]);
        if (data[4] == 0x04) {
            ESP_LOGI(TAG, "Hardware version number: %x", data[5]);
            fw_version_[0] = data[6];
            ESP_LOGI(TAG, "ROM Code version number: %x", data[6]);
            fw_version_[1] = data[7];
            ESP_LOGI(TAG, "FLASH Major version: %x", data[7]);
            fw_version_[2] = data[8];
            ESP_LOGI(TAG, "FLASH Minor version: %x", data[8]);
        }
    }
    return rsp_status;
}

esp_err_t PN7160_NCI::core_init() {
    NciMessage cmd, rsp;
    cmd.build(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_INIT_OID);
    esp_err_t status = send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
    return status;
}

esp_err_t PN7160_NCI::core_set_config(std::span<const uint8_t> config_params) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_SET_CONFIG_OID, config_params);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::core_get_config(std::span<const uint8_t> config_params, NciMessage& rsp) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::CORE_GID, nci::CORE_GET_CONFIG_OID, config_params);
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_discover_map(std::span<const uint8_t> mappings) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_MAP_OID, mappings);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_set_listen_mode_routing(std::span<const uint8_t> routing_config) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_SET_LISTEN_MODE_ROUTING_OID, routing_config);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_start_discovery(std::span<const uint8_t> discovery_config) {
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_OID, discovery_config);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_stop_discovery() {
    return rf_deactivate(nci::DEACTIVATION_TYPE_IDLE);
}

esp_err_t PN7160_NCI::rf_select_target(uint8_t discovery_id, uint8_t protocol, uint8_t interface) {
    uint8_t payload[] = {discovery_id, protocol, interface};
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DISCOVER_SELECT_OID, payload);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, nci::PN7160_INIT_TIMEOUT_MS);
}

esp_err_t PN7160_NCI::rf_deactivate(uint8_t type) {
    uint8_t payload[] = {type};
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_DEACTIVATE_OID, payload);
    NciMessage rsp;
    return send_command_wait_response(cmd, rsp, 1000);
}

esp_err_t PN7160_NCI::rf_iso_dep_presence_check() {
    if (!initialized_.load()) return ESP_ERR_INVALID_STATE;
    NciMessage cmd(nci::PKT_MT_CTRL_COMMAND, nci::RF_GID, nci::RF_ISO_DEP_NAK_PRESENCE_OID);
    return write_nci_packet(cmd);
}

// =============================================================================
// App-facing event queue
// =============================================================================

esp_err_t PN7160_NCI::get_event(NciEvent& event, uint32_t timeout_ms) {
    constexpr uint32_t chunk_ms = 50;
    uint32_t elapsed = 0;
    while (!stop_flag_.load() && !cancelled_.load()) {
        uint32_t wait = (timeout_ms == portMAX_DELAY) ? chunk_ms : std::min(chunk_ms, timeout_ms - elapsed);
        esp_err_t ret = event_ring_.pop(event, wait);
        if (ret == ESP_OK) return ESP_OK;
        if (ret != ESP_ERR_TIMEOUT) return ret;

        elapsed += wait;
        if (timeout_ms != portMAX_DELAY && elapsed >= timeout_ms) return ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_INVALID_STATE;
}

uint16_t PN7160_NCI::get_firmware_version() const {
    return fw_version_[1] << 8 | fw_version_[2];
}

// =============================================================================
// Task-runner dispatch helpers
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
            if (msg.get_len() >= 1) tag_in_field_.store(msg[3] == 0x00);
            break;
        default: ;
    }
}

void PN7160_NCI::handle_rf_notification(uint8_t oid, const NciMessage& msg) {
    NciEventType evt_type = {};
    switch (oid) {
        case nci::RF_INTF_ACTIVATED_OID: evt_type = NciEventType::RF_INTF_ACTIVATED; break;
        case nci::RF_DISCOVER_OID:       evt_type = NciEventType::RF_DISCOVER;       break;
        case nci::RF_DEACTIVATE_OID:     evt_type = NciEventType::RF_DEACTIVATE;     break;
        case nci::RF_ISO_DEP_NAK_PRESENCE_OID: return; // Internal state only
        default:
            ESP_LOGW(TAG, "Unhandled RF NTF OID=0x%02X", oid);
            return;
    }
    if (!event_ring_.push({evt_type, msg}))
        ESP_LOGW(TAG, "Event ring full – dropped RF NTF (OID=0x%02X)", oid);
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
            ESP_LOGW(TAG, "Unhandled Core NTF OID=0x%02X", oid);
            break;
    }
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
        ESP_LOGW(TAG, "Unhandled NTF GID=0x%02X", msg.get_gid());
}

void PN7160_NCI::enqueue_data_packet(const NciMessage& msg) {
    ESP_LOGD(TAG, "DATA packet: Len=%d", msg.get_len());
    if (!event_ring_.push({NciEventType::DATA_PACKET, msg}))
        ESP_LOGW(TAG, "Event ring full – dropped DATA packet");
}

void PN7160_NCI::handle_unsolicited_response(const NciMessage& msg) {
    if (msg.get_gid() == nci::RF_GID) {
        switch (msg.get_oid()) {
            case nci::RF_INTF_ACTIVATED_OID:
                tag_in_field_.store(true);
                break;
            case nci::RF_ISO_DEP_NAK_PRESENCE_OID:
                if (msg.get_len() >= 1) tag_in_field_.store(msg[3] == nci::STATUS_OK);
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
// =============================================================================

void PN7160_NCI::task_runner() {
    task_running_.store(true);
    if (!initialized_.load()) {
        task_handle_ = nullptr;
        task_running_.store(false);
        return;
    }
    task_handle_ = xTaskGetCurrentTaskHandle();

    NciMessage incoming;
    while (!stop_flag_.load()) {
        esp_err_t ret = transport_.wait_for_irq(true, pdMS_TO_TICKS(10));

        if (ret == ESP_ERR_TIMEOUT) continue;
        if (ret != ESP_OK) {
            signal_sync_failure(ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!transport_.read_irq_level()) continue;

        ret = read_nci_packet(incoming, nci::PN7160_DEFAULT_TIMEOUT_MS);
        if (ret == ESP_ERR_TIMEOUT) continue;
        if (ret != ESP_OK) {
            signal_sync_failure(ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (try_complete_exchange(incoming)) continue;

        switch (incoming.get_mt()) {
            case nci::PKT_MT_CTRL_NOTIFICATION: enqueue_notification(incoming); break;
            case nci::PKT_MT_CTRL_RESPONSE:     handle_unsolicited_response(incoming); break;
            case nci::PKT_MT_DATA:              enqueue_data_packet(incoming); break;
        }
        taskYIELD();
    }

    ESP_LOGI(TAG, "PN7160 task runner stopping.");
    task_running_.store(false);
}