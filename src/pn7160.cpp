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
    // 0x14,        // TXLDO (3.3V / 4.75V)
    // 0xBB,        // TXLDO (4.7V / 4.7V)
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
    irq_sem_ = xSemaphoreCreateBinary();
    apdu_sync_sem_ = xSemaphoreCreateBinary();
    sync_mutex_ = xSemaphoreCreateMutex();
    if (!irq_sem_ || !apdu_sync_sem_ || !sync_mutex_) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        if (irq_sem_) vSemaphoreDelete(irq_sem_);
        if (apdu_sync_sem_) vSemaphoreDelete(apdu_sync_sem_);
        if (sync_mutex_) vSemaphoreDelete(sync_mutex_);
    }
}

PN7160_NCI::~PN7160_NCI() {
    if (apdu_sync_sem_) {
        vSemaphoreDelete(apdu_sync_sem_);
    }
    if (sync_mutex_) {
        vSemaphoreDelete(sync_mutex_);
    }
}

// --- Initialization ---

esp_err_t PN7160_NCI::initialize() {
    esp_err_t nci_status = STATUS_FAILED;

    ESP_LOGI(TAG, "Initializing PN7160...");

    transport.init();

    // NCI Core Reset
    // Reset config=true, power_cycle=true (if VEN available)
    nci_status = core_reset(true);
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "NCI Core Reset command failed (NCI Status=0x%X)", nci_status);
        spi_bus_remove_device(spi_device_);
        spi_device_ = nullptr;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Reset successful");

    // NCI Core Init
    nci_status = core_init();
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "NCI Core Init command failed (NCI Status=0x%X)", nci_status);
        spi_bus_remove_device(spi_device_);
        spi_device_ = nullptr;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NCI Core Init successful");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Activate Proprietary Extensions
    ESP_LOGD(TAG, "Activating NXP Proprietary Extensions...");
    // OID 0x02 for NCI_PROPRIETARY_ACT_CMD (UM11495 Table 36)
    NciMessage prop_act_cmd(NCI_PKT_MT_CTRL_COMMAND, NCI_PROPRIETARY_GID, NCI_CORE_SET_CONFIG_OID);
    NciMessage prop_act_rsp;
    nci_status = write_nci_packet(prop_act_cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_PROPRIETARY_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    nci_status = read_nci_packet(prop_act_rsp, PN7160_INIT_TIMEOUT_MS);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_PROPRIETARY_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED; // Original behavior
    }
    if (!prop_act_rsp.is_control_response(NCI_PROPRIETARY_GID, NCI_CORE_SET_CONFIG_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_PROPRIETARY_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, prop_act_rsp.data(), prop_act_rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
     if (nci_status != STATUS_OK) {
        // This might not be strictly fatal, but configs might fail later
        ESP_LOGW(TAG, "Failed to activate proprietary extensions (NCI Status=0x%X). Configs might fail.", nci_status);
    } else {
        ESP_LOGI(TAG, "Proprietary extensions activated.");
        // Optional: Log response payload if needed (UM11495 Table 37)
        ESP_LOG_BUFFER_HEXDUMP(TAG, prop_act_rsp.get_payload_ptr(), prop_act_rsp.get_len(), ESP_LOG_DEBUG);
    }
     vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "Sending initial configuration...");

    // Send PMU Config
    NciMessage pmu_cmd(
        NCI_PKT_MT_CTRL_COMMAND, NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID,
        std::vector<uint8_t>(std::begin(PMU_CFG), std::end(PMU_CFG)));
    NciMessage pmu_rsp;
    nci_status = write_nci_packet(pmu_cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    nci_status = read_nci_packet(pmu_rsp, PN7160_INIT_TIMEOUT_MS);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }
    if (!pmu_rsp.is_control_response(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, pmu_rsp.data(), pmu_rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set PMU config (NCI Status=0x%X)", nci_status);
        spi_bus_remove_device(spi_device_); spi_device_ = nullptr; return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "PMU Config set successfully");
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Send TOTAL_DURATION Config
    NciMessage duration_cmd(
        NCI_PKT_MT_CTRL_COMMAND, NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID,
        std::vector<uint8_t>(std::begin(CORE_CONFIG_TOTAL_DURATION_SOLO),
                             std::end(CORE_CONFIG_TOTAL_DURATION_SOLO)));
    NciMessage duration_rsp;
    nci_status = write_nci_packet(duration_cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    nci_status = read_nci_packet(duration_rsp, PN7160_INIT_TIMEOUT_MS);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED; // Original behavior
    }
    if (!duration_rsp.is_control_response(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, duration_rsp.data(), duration_rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set TOTAL_DURATION config (NCI Status=0x%X)", nci_status);
    } else {
        ESP_LOGI(TAG, "TOTAL_DURATION Config set successfully");
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    initialized_ = true;
    ESP_LOGI(TAG, "PN7160 Initialized Successfully");
    return ESP_OK;
}

// --- Hardware Control ---

esp_err_t PN7160_NCI::hardware_reset() {
    if (transport.has_ven()) {
        ESP_LOGE(TAG, "Cannot perform hardware reset, VEN pin not configured");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGD(TAG, "Performing hardware reset...");
    transport.set_ven(true);
    vTaskDelay(pdMS_TO_TICKS(10)); // Reset pulse width (min 10ms UM11495?)
    transport.set_ven(false);
    vTaskDelay(pdMS_TO_TICKS(10)); // Reset pulse width (min 10ms UM11495?)
    transport.set_ven(true);
    vTaskDelay(pdMS_TO_TICKS(
        PN7160_INIT_TIMEOUT_MS)); // Wait for chip to boot (UM11495 Tboot=5ms)
    ESP_LOGD(TAG, "Hardware reset complete.");
    return ESP_OK;
}

// --- NCI Packet Read/Write ---

esp_err_t PN7160_NCI::read_nci_packet(NciMessage& msg, uint32_t timeout_ms) {
    msg.clear();
    uint8_t header[NCI_HEADER_SIZE];
    uint8_t payload_len = 0;

    // Wait for IRQ high (indicates data available)
    esp_err_t ret = transport.wait_for_irq(true, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        // Check level again in case it went high just after timeout
        if (transport.read_irq_level()) {
             ESP_LOGW(TAG, "IRQ became HIGH right after timeout check in read_nci_packet");
             // Proceed to read attempt
        } else {
            ESP_LOGD(TAG, "Timeout waiting for IRQ high before read");
            return ESP_ERR_TIMEOUT;
        }
    }


    // Read NCI Header (3 bytes)
    ESP_RETURN_ON_ERROR(transport.read(header, NCI_HEADER_SIZE), TAG,
                        "Failed to read NCI header");

    // Parse header and get payload length
    msg.buffer.assign(header, header + NCI_HEADER_SIZE);
    payload_len = msg.get_len();

    ESP_LOGD(TAG, "Read Header: MT=0x%X GID=0x%X OID=0x%X Len=%d", msg.get_mt(),
             msg.get_gid(), msg.get_oid(), payload_len);

    if (payload_len > NCI_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "NCI payload length too large: %d", payload_len);
        msg.clear();
        return ESP_FAIL; // Or a more specific error
    }

    // Read Payload (if any)
    if (payload_len > 0) {
        // Resize buffer to accommodate payload
        size_t current_size = msg.buffer.size();
        msg.buffer.resize(current_size + payload_len);
        ESP_RETURN_ON_ERROR(
            transport.read(msg.data() + current_size, payload_len), TAG,
            "Failed to read NCI payload");
    }

    ESP_LOGD(TAG, "Read NCI Packet (Size: %zu):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);

    return ESP_OK;
}

esp_err_t PN7160_NCI::write_nci_packet(const NciMessage& msg) {
    if (msg.size() == 0 || msg.size() > NCI_MAX_PACKET_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Write NCI Packet (Size: %zu):", msg.size());
    ESP_LOG_BUFFER_HEXDUMP(TAG, msg.data(), msg.size(), ESP_LOG_DEBUG);
    return transport.write(msg.data(), msg.size());
}

// --- NCI Command / Response ---

// Returns NCI Status Code or ESP error code
esp_err_t PN7160_NCI::send_command_wait_response(NciMessage& cmd,
                                                 NciMessage& rsp,
                                                 uint32_t timeout_ms) {
    // Allow Reset/Init even if not marked initialized yet
    if (!initialized_ && cmd.get_gid() == NCI_CORE_GID &&
        cmd.get_oid() != NCI_CORE_RESET_OID &&
        cmd.get_oid() != NCI_CORE_INIT_OID) {
        ESP_LOGE(TAG,
                 "Driver not initialized, cannot send command GID=0x%02X, "
                 "OID=0x%02X",
                 cmd.get_gid(), cmd.get_oid());
        return ESP_ERR_INVALID_STATE;
    }

    // --- GENERIC HANDLING (for all other commands) ---
    ESP_LOGD(TAG, "Sending CMD: GID=0x%02X, OID=0x%02X, Len=%d", cmd.get_gid(),
             cmd.get_oid(), cmd.get_len());
    esp_err_t write_status = write_nci_packet(cmd);
    if (write_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write NCI command (err=0x%X)", write_status);
        return write_status; // Return ESP-IDF error
    }

    // Wait for and read the response
    esp_err_t read_status = read_nci_packet(rsp, timeout_ms);
    if (read_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI response (err=0x%X)", read_status);
        return read_status; // Return ESP-IDF error
    }

    ESP_LOGD(TAG,
             "Received RSP: MT=0x%X, GID=0x%02X, OID=0x%02X, Len=%d, "
             "Status=0x%X",
             rsp.get_mt(), rsp.get_gid(), rsp.get_oid(), rsp.get_len(),
             rsp.get_status());

    // Basic validation: Check if it's a response, matches GID/OID
    if (!rsp.is_control_response(cmd.get_gid(), cmd.get_oid())) {
        ESP_LOGE(TAG,
                 "Unexpected NCI packet received. Expected RSP for GID=0x%02X, "
                 "OID=0x%02X. Got MT=%d, GID=0x%02X, OID=0x%02X",
                 cmd.get_gid(), cmd.get_oid(), rsp.get_mt(), rsp.get_gid(),
                 rsp.get_oid());
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED; // Return NCI failure status
    }

    // Return the NCI status code from the response payload
    return rsp.get_status(); // NCI status code, 0x00 is STATUS_OK
}

esp_err_t PN7160_NCI::send_data_packet(const NciMessage& data_pkt) {
    if (data_pkt.get_mt() != NCI_PKT_MT_DATA) {
        ESP_LOGE(TAG, "Message is not an NCI Data packet (MT=%d)",
                 data_pkt.get_mt());
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Sending DATA: Len=%d", data_pkt.get_len());
    // Data packets don't usually have an immediate NCI response,
    // but might get credits notification later.
    return write_nci_packet(data_pkt);
}

// --- High Level NCI Commands ---

/**
 * @brief Sends a Command APDU synchronously and waits for the Response APDU.
 * @note This function MUST be called only when an ISO-DEP interface is active.
 *       It assumes communication happens over the Static RF Connection (Conn ID 0).
 *       Only one synchronous APDU exchange can be active at a time.
 *
 * @param c_apdu The Command APDU bytes to send.
 * @param r_apdu Output vector to store the received Response APDU (including SW1, SW2).
 * @param timeout_ms Timeout in milliseconds to wait for the response.
 * @return esp_err_t STATUS_OK on success (APDU exchange completed, check r_apdu status word),
 *                   ESP_ERR_TIMEOUT if response wasn't received in time,
 *                   ESP_ERR_INVALID_STATE if driver not initialized or another sync operation is running,
 *                   ESP_FAIL or other ESP errors on communication failure.
 */
esp_err_t PN7160_NCI::send_apdu_sync(const std::vector<uint8_t> &c_apdu,
                                     std::vector<uint8_t> &r_apdu,
                                     uint32_t timeout_ms) {
  if (!initialized_) {
    ESP_LOGE(TAG, "send_apdu_sync: Driver not initialized.");
    return ESP_ERR_INVALID_STATE;
  }
  if (c_apdu.empty()) {
    ESP_LOGE(TAG, "send_apdu_sync: Command APDU cannot be empty.");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_FAIL;
  r_apdu.clear();

  // --- Lock and set state for synchronous operation ---
  if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGE(TAG, "send_apdu_sync: Could not acquire sync mutex.");
    return ESP_FAIL;
  }

  if (sync_apdu_in_progress_) {
    ESP_LOGE(TAG, "send_apdu_sync: Another synchronous APDU operation is "
                  "already in progress.");
    xSemaphoreGive(sync_mutex_);
    return ESP_ERR_INVALID_STATE; // Or maybe ESP_ERR_TIMEOUT?
  }

  // Mark as busy and clear previous response
  sync_apdu_in_progress_ = true;
  sync_apdu_response_.clear();
  // Clear the response semaphore before waiting
  xSemaphoreTake(apdu_sync_sem_, 0);

  xSemaphoreGive(sync_mutex_);
  // --- End Lock ---

  // --- Send the APDU ---
  NciMessage command_msg;
  command_msg.build_data(c_apdu); // Builds data packet for ConnID 0

  ESP_LOGD(TAG, "send_apdu_sync: Sending C-APDU (%zu bytes)", c_apdu.size());
  esp_err_t send_ret = send_data_packet(command_msg);

  if (send_ret != ESP_OK) {
    ESP_LOGE(TAG, "send_apdu_sync: Failed to send NCI Data Packet (err=0x%X)",
             send_ret);
    result = send_ret; // Propagate the error
    // Clean up sync state immediately on send failure
    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
      sync_apdu_in_progress_ = false;
      xSemaphoreGive(sync_mutex_);
    }
    return result;
  }

  // --- Wait for the response semaphore (given by task_runner) ---
  ESP_LOGD(TAG, "send_apdu_sync: Waiting for R-APDU...");
  if (xSemaphoreTake(apdu_sync_sem_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
    // Semaphore received, get the response from shared buffer
    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (!sync_apdu_response_.empty()) {
        r_apdu = sync_apdu_response_; // Copy response
        ESP_LOGD(TAG, "send_apdu_sync: R-APDU received (%zu bytes)",
                 r_apdu.size());
        result = STATUS_OK; // NCI exchange successful
      } else {
        ESP_LOGE(
            TAG,
            "send_apdu_sync: Semaphore received but response buffer is empty!");
        result = ESP_FAIL; // Internal error state
      }
      sync_apdu_in_progress_ = false; // Mark operation as finished
      xSemaphoreGive(sync_mutex_);
    } else {
      ESP_LOGE(TAG, "send_apdu_sync: Could not acquire sync mutex after "
                    "receiving semaphore!");
      result = ESP_FAIL; // Mutex error
      // Need to ensure flag is cleared eventually
      sync_apdu_in_progress_ = false; // Attempt to clear anyway
    }
  } else {
    // Timeout waiting for semaphore
    ESP_LOGE(TAG, "send_apdu_sync: Timeout waiting for R-APDU response.");
    result = ESP_ERR_TIMEOUT;
    // Clean up sync state on timeout
    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
      sync_apdu_in_progress_ = false;
      xSemaphoreGive(sync_mutex_);
    }
  }

  return result;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::core_reset(bool reset_config) {
    if(transport.has_ven()){
      // *** Perform power cycle first if requested ***
      transport.set_ven(false);
      vTaskDelay(pdMS_TO_TICKS(PN7160_DEFAULT_TIMEOUT_MS)); // Reset pulse width
      transport.set_ven(true);
      vTaskDelay(pdMS_TO_TICKS(PN7160_DEFAULT_TIMEOUT_MS)); // Wait for boot
      ESP_LOGD(TAG, "Power cycle reset complete.");
    }
    // Optional: Clear potential initial IRQ high after power cycle
    if (transport.read_irq_level()) {
        ESP_LOGW(TAG, "IRQ high after power cycle, clearing pending message");
        NciMessage dummy_msg;
        read_nci_packet(dummy_msg, 50);
    }

    // NCI 2.0 Sequence: CMD -> RSP -> NTF
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, NCI_CORE_GID, NCI_CORE_RESET_OID,
                   {(uint8_t)(reset_config ? 0x01 : 0x00)});
    NciMessage ntf;
    NciMessage rsp;
    ESP_LOGD(TAG, "Sending CORE_RESET_CMD (Reset Config: %d)", reset_config);
    esp_err_t write_status = write_nci_packet(cmd);
    if (write_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending CORE_RESET_CMD (err=0x%X)", write_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_NTF (err=0x%X)", read_status_ntf);
        return STATUS_FAILED;
    }
    if (!rsp.is_control_response(NCI_CORE_GID, NCI_CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after CORE_RESET_NTF (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    ESP_LOGD(TAG, "Expecting CORE_RESET_NTF...");
    read_status_ntf =
        read_nci_packet(ntf, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CORE_RESET_NTF (err=0x%X)", read_status_ntf);
        return STATUS_FAILED;
    }
    ESP_LOGD(TAG, "Expecting CORE_RESET_RSP...");
    if (!ntf.is_control_notification(NCI_CORE_GID, NCI_CORE_RESET_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after CORE_RESET_RSP (expected RSP)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    } else {
        ESP_LOGD(TAG, "Received CORE_RESET_RSP");
        if (ntf.get_len() >= 2) {
            std::vector<uint8_t> ntf_payload = ntf.get_payload_copy();
            uint8_t ntf_status = ntf.buffer[1];
            uint8_t nci_version_byte = ntf_payload[2];
            uint8_t manuf_spec_info_len = 0;
            if (ntf.get_len() > 2) {
                 manuf_spec_info_len = ntf_payload[4];
            }

            ESP_LOGD(TAG, "CORE_RESET_RSP Info: Status=0x%02X", ntf_status);
            ESP_LOGI(TAG, "NCI Version: %u.%u", nci_version_byte >> 4,
                     nci_version_byte & 0x0F);
            if (manuf_spec_info_len > 0 && ntf.size() >= 8 + manuf_spec_info_len) {
                ESP_LOGI(TAG, "Manufacturer Specific Info (%d bytes):", manuf_spec_info_len);
                ESP_LOG_BUFFER_HEXDUMP(TAG, ntf_payload.data() + 5, manuf_spec_info_len,
                                       ESP_LOG_INFO);
                // PN7160 Specific (UM11495 Table 35): First 4 bytes are FW Major/FW Minor
                 if (manuf_spec_info_len >= 4) {
                     ESP_LOGI(TAG, "  Ver code: %02x, FW Ver: %02x.%02x.%02x",
                        ntf_payload[5], ntf_payload[6], ntf_payload[7], ntf_payload[8]);
                 }
            } else if (ntf.get_len() > 2 && manuf_spec_info_len == 0) {
                 ESP_LOGI(TAG, "No Manufacturer Specific Info present in NTF.");
            } else if (ntf.get_len() > 2) {
                 ESP_LOGW(TAG, "CORE_RESET_RSP manuf_spec_info length mismatch (len=%d, pkt_size=%zu)", manuf_spec_info_len, ntf.size());
            }
        } else {
            ESP_LOGW(TAG, "CORE_RESET_RSP payload too small (%d bytes)",
                     ntf.get_len());
        }
    }

    uint8_t rsp_status = rsp.get_status();
    ESP_LOGD(TAG, "Received CORE_RESET_RSP (status: 0x%X)", rsp_status);

    // Final check: CORE_RESET_RSP status must be OK
    if (rsp_status != STATUS_OK) {
        ESP_LOGE(TAG, "CORE_RESET_RSP returned status 0x%X", rsp_status);
    }

    return rsp_status; // Return the NCI status from RSP
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::core_init() {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, NCI_CORE_GID, NCI_CORE_INIT_OID);
    NciMessage rsp;
    // Core Init response contains chip info, might be longer
    ESP_LOGD(TAG, "Sending NCI_CORE_INIT_OID");
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_INIT_OID (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_INIT_OID (err=0x%X)", read_status_ntf);
        return STATUS_FAILED; // Original behavior
    }
    if (!rsp.is_control_response(NCI_CORE_GID, NCI_CORE_INIT_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_INIT_OID (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }

    uint8_t hw_version = rsp.buffer[17 + rsp.buffer[8]];
    uint8_t rom_code_version = rsp.buffer[18 + rsp.buffer[8]];
    uint8_t flash_major_version = rsp.buffer[19 + rsp.buffer[8]];
    uint8_t flash_minor_version = rsp.buffer[20 + rsp.buffer[8]];
    std::vector<uint8_t> features(rsp.buffer.begin() + 4, rsp.buffer.begin() + 8);
  
    ESP_LOGD(TAG, "Hardware version: %u", hw_version);
    ESP_LOGD(TAG, "ROM code version: %u", rom_code_version);
    ESP_LOGD(TAG, "FLASH major version: %u", flash_major_version);
    ESP_LOGD(TAG, "FLASH minor version: %u", flash_minor_version);
    ESP_LOGD(TAG, "Features: %02X %02X %02X %02X", features[0], features[1], features[2], features[3]);
    return nci_status;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::core_set_config(
    const std::vector<uint8_t>& config_params) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, NCI_CORE_GID,
                   NCI_CORE_SET_CONFIG_OID, config_params);
    NciMessage rsp;
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", read_status_ntf);
        return STATUS_FAILED; // Original behavior
    }
    if (!rsp.is_control_response(NCI_CORE_GID, NCI_CORE_SET_CONFIG_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    if (nci_status == STATUS_INVALID_PARAM) {
        ESP_LOGW(TAG, "CORE_SET_CONFIG_RSP indicates invalid parameters:");
        if (rsp.get_len() > 1) { // Payload = Status (1) + NumInvalid(1) + IDs...
            uint8_t num_invalid = rsp.buffer[4];
            ESP_LOGW(TAG, "  Num Invalid: %d", num_invalid);
            if (rsp.get_len() > 2) {
                 ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data() + 5, rsp.get_len() - 2, ESP_LOG_WARN);
            }
        } else {
             ESP_LOGW(TAG, "  (Payload too short to list invalid IDs)");
        }
        // Treat STATUS_INVALID_PARAM as a warning, but return the status
    } else if (nci_status != STATUS_OK) {
         ESP_LOGE(TAG, "CORE_SET_CONFIG_CMD failed (NCI Status=0x%X)", nci_status);
    }
    return nci_status;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::rf_discover_map(
    const std::vector<uint8_t>& mappings) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID, RF_DISCOVER_MAP_OID,
                   mappings);
    NciMessage rsp;
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", read_status_ntf);
        return STATUS_FAILED;
    }
    if (!rsp.is_control_response(RF_GID, RF_DISCOVER_MAP_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    return nci_status;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::rf_set_listen_mode_routing(
    const std::vector<uint8_t>& routing_config) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID,
                   RF_SET_LISTEN_MODE_ROUTING_OID, routing_config);
    NciMessage rsp;
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", read_status_ntf);
        return STATUS_FAILED; // Original behavior
    }
    if (!rsp.is_control_response(RF_GID, RF_SET_LISTEN_MODE_ROUTING_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    return nci_status;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::rf_start_discovery(
    const std::vector<uint8_t>& discovery_config) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID, RF_DISCOVER_OID,
                   discovery_config);
    NciMessage rsp;
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    esp_err_t read_status_ntf = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (read_status_ntf != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (err=0x%X)", read_status_ntf);
        return STATUS_FAILED; // Original behavior
    }
    if (!rsp.is_control_response(RF_GID, RF_DISCOVER_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after NCI_CORE_GID (NCI_CORE_SET_CONFIG_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    return nci_status;
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::rf_stop_discovery() {
    // Deactivate to IDLE state to stop discovery
    return rf_deactivate(DEACTIVATION_TYPE_IDLE);
}

// Returns NCI Status Code
esp_err_t PN7160_NCI::rf_select_target(uint8_t discovery_id, uint8_t protocol,
                                       uint8_t interface) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID, RF_DISCOVER_SELECT_OID,
                   {discovery_id, protocol, interface});
    NciMessage rsp;
    // Selecting a target might take longer if tag is slow
    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Error sending RF_GID (RF_DISCOVER_SELECT_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED;
    }

    nci_status = read_nci_packet(rsp, PN7160_INIT_TIMEOUT_MS);
    if (nci_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RF_GID (RF_DISCOVER_SELECT_OID) (err=0x%X)", nci_status);
        return STATUS_FAILED; // Original behavior
    }
    if (!rsp.is_control_response(RF_GID, RF_DISCOVER_SELECT_OID)) {
        ESP_LOGE(TAG, "Received unexpected packet after RF_GID (RF_DISCOVER_SELECT_OID) (expected NTF)");
        ESP_LOG_BUFFER_HEXDUMP(TAG, rsp.data(), rsp.size(), ESP_LOG_ERROR);
        return STATUS_FAILED;
    }
    return nci_status;
}

// Returns NCI Status Code (STATUS_OK if RSP is OK, error otherwise)
// Note: This function waits for the NTF as well for completion.
esp_err_t PN7160_NCI::rf_deactivate(uint8_t type) {
    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID, RF_DEACTIVATE_OID, {type});
    NciMessage rsp;
    NciMessage ntf;

    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "RF_DEACTIVATE_CMD failed (NCI Status=0x%X)", nci_status);
        return nci_status; // Return the error status from RSP
    }

    esp_err_t read_status =
    read_nci_packet(ntf, PN7160_DEFAULT_TIMEOUT_MS * 2); // Longer timeout?
    if (read_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RF_DEACTIVATE_CMD (err=0x%X)",
                read_status);
        return STATUS_FAILED; // Indicate failure to get RSP
    }

    if (!ntf.is_control_response(RF_GID, RF_DEACTIVATE_OID)) {
        ESP_LOGE(TAG, "Unexpected packet received after RF_DEACTIVATE_RSP");
        ESP_LOG_BUFFER_HEXDUMP(TAG, ntf.data(), ntf.size(), ESP_LOG_WARN);
        return STATUS_FAILED; // Indicate unexpected packet
    }

    // If RSP is OK, expect RF_DEACTIVATE_NTF
    ESP_LOGD(TAG, "Expecting RF_DEACTIVATE_NTF...");
    read_status =
        read_nci_packet(ntf, PN7160_DEFAULT_TIMEOUT_MS * 2); // Longer timeout?
    if (read_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RF_DEACTIVATE_NTF (err=0x%X)",
                 read_status);
        return STATUS_FAILED; // Indicate failure to get NTF
    }

    // Log info from NTF (NCI 2.0 Table 69)
    if (ntf.get_len() >= 2) {
        ESP_LOGD(TAG, "RF_DEACTIVATE_NTF received. Type: 0x%02X, Reason: 0x%02X",
                 ntf.buffer[3], ntf.buffer[4]);
    } else {
         ESP_LOGW(TAG, "RF_DEACTIVATE_NTF payload too short (%d bytes)", ntf.get_len());
    }

    return STATUS_OK; // Return OK as both RSP and NTF were handled
}

esp_err_t PN7160_NCI::rf_iso_dep_presence_check() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    NciMessage cmd(NCI_PKT_MT_CTRL_COMMAND, RF_GID, RF_ISO_DEP_NAK_PRESENCE_OID);
    NciMessage rsp;
    ESP_LOGD(TAG, "Sending RF_ISO_DEP_NAK_PRESENCE_CMD");

    esp_err_t nci_status = write_nci_packet(cmd);
    if (nci_status != STATUS_OK) {
        ESP_LOGE(TAG, "RF_ISO_DEP_NAK_PRESENCE_CMD failed (NCI Status=0x%X)", nci_status);
        return nci_status; // Return the error status from RSP
    }
    
    return nci_status;
}

// --- Task Runner ---

void PN7160_NCI::task_runner() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Task runner started but driver not initialized!");
        vTaskDelete(NULL);
        return;
    }
    task_handle_ = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "PN7160 task runner started.");

    NciMessage incoming_msg;
    esp_err_t ret = 0; 
    while (true) {
        if (transport.wait_for_irq(true, portMAX_DELAY) == ESP_OK) {
            ret = read_nci_packet(incoming_msg, PN7160_DEFAULT_TIMEOUT_MS);

            if (ret == ESP_OK) {
                bool processed_synchronously = false; // Flag to skip async callback if handled sync

                // --- Check if this is data for a synchronous wait ---
                if (incoming_msg.get_mt() == NCI_PKT_MT_DATA) {
                    if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) { // Short wait for mutex
                        if (sync_apdu_in_progress_) {
                            ESP_LOGD(TAG, "Data packet received during sync APDU wait");
                            sync_apdu_response_ = incoming_msg.get_payload_copy(); // Store payload
                            xSemaphoreGive(apdu_sync_sem_); // Signal the waiting function
                            processed_synchronously = true;
                        }
                        xSemaphoreGive(sync_mutex_);
                    } else {
                        ESP_LOGE(TAG, "Failed to take sync mutex in task_runner!");
                        // Proceed with async handling if mutex fails? Or log error?
                    }
                }
                if(!processed_synchronously){
                    switch (incoming_msg.get_mt()) {
                        case NCI_PKT_MT_CTRL_NOTIFICATION:
                            ESP_LOGD(TAG, "Received NTF: GID=0x%02X, OID=0x%02X",
                                    incoming_msg.get_gid(), incoming_msg.get_oid());
                            // --- Standard Notification Handling ---
                            if (incoming_msg.get_gid() == RF_GID) {
                                switch (incoming_msg.get_oid()) {
                                    case RF_INTF_ACTIVATED_OID:
                                        ESP_LOGD(TAG, "RF_INTF_ACTIVATED_NTF: DiscID=0x%02X Intf=0x%02X Proto=0x%02X Mode=0x%02X",
                                                incoming_msg.buffer[3], incoming_msg.buffer[4],
                                                incoming_msg.buffer[5], incoming_msg.buffer[6]);
                                        selected_tag_still_in_field.store(true);
                                        if (on_rf_intf_activated_) on_rf_intf_activated_(incoming_msg);
                                        break;
                                    case RF_DISCOVER_OID:
                                        ESP_LOGD(TAG, "RF_DISCOVER_NTF received");
                                        if (on_rf_discover_) on_rf_discover_(incoming_msg);
                                        break;
                                    case RF_DEACTIVATE_OID: // RF_DEACTIVATE_NTF
                                    if (incoming_msg.get_len() >= 2) { // Need Type and Reason
                                        uint8_t deact_type = incoming_msg.buffer[3];
                                        uint8_t deact_reason = incoming_msg.buffer[4];
                                        ESP_LOGD(TAG, "RF_DEACTIVATE_NTF received. Type: 0x%02X, Reason: 0x%02X", deact_type, deact_reason);
                    
                                        if (deact_reason == 0x02) { // 0x02 = RF_Link_Loss
                                            ESP_LOGW(TAG, "Tag removed (RF Link Loss detected by NFCC)!");
                                        } else {
                                            ESP_LOGD(TAG, "Deactivation reason was not Link Loss.");
                                        }
                    
                                        if (on_rf_deactivate_)
                                            on_rf_deactivate_(incoming_msg);
                    
                                    } else {
                                        ESP_LOGW(TAG, "RF_DEACTIVATE_NTF payload too short (%d bytes)", incoming_msg.get_len());
                                    }
                                    break;
                                    case RF_ISO_DEP_NAK_PRESENCE_OID: {  // CTRL_NOTIFICATION
                                        esp_log_buffer_hexdump_internal(TAG, incoming_msg.buffer.data(), incoming_msg.buffer.size(), ESP_LOG_DEBUG);
                                        uint8_t presence = incoming_msg.buffer[3];
                                        if (presence != 0x00) {
                                            ESP_LOGW(TAG, "Tag no longer in field");
                                            selected_tag_still_in_field.store(false);
                                            rf_deactivate(DEACTIVATION_TYPE_DISCOVERY);
                                        } else {
                                            ESP_LOGD(TAG, "Tag still in field");
                                            rf_iso_dep_presence_check();
                                        }
                                        break;
                                    }
                                    default:
                                        ESP_LOGW(TAG, "Unhandled RF Notification OID: 0x%02X", incoming_msg.get_oid());
                                        break;
                                }
                            } else if (incoming_msg.get_gid() == NCI_CORE_GID) {
                                switch (incoming_msg.get_oid()) {
                                    case NCI_CORE_GENERIC_ERROR_OID:
                                        ESP_LOGW(TAG, "CORE_GENERIC_ERROR_NTF Status: 0x%X", incoming_msg.get_status());
                                        if (on_core_notification_) on_core_notification_(incoming_msg);
                                        break;
                                    case NCI_CORE_INTERFACE_ERROR_OID:
                                        ESP_LOGW(TAG, "CORE_INTERFACE_ERROR_NTF Status: 0x%X ConnID: %d",
                                                incoming_msg.buffer[3], incoming_msg.buffer[4]);
                                        selected_tag_still_in_field.store(true);
                                        selected_tag_still_in_field.notify_all();
                                        if (on_core_notification_) on_core_notification_(incoming_msg);
                                        break;
                                    case NCI_CORE_CONN_CREDITS_OID:
                                        ESP_LOGD(TAG, "CORE_CONN_CREDITS_NTF received");
                                        if (on_core_notification_) on_core_notification_(incoming_msg);
                                        break;
                                    default:
                                        ESP_LOGW(TAG, "Unhandled Core Notification OID: 0x%02X", incoming_msg.get_oid());
                                        if (on_core_notification_) on_core_notification_(incoming_msg);
                                        break;
                                }
                            } else {
                                ESP_LOGW(TAG, "Unhandled Notification GID: 0x%02X", incoming_msg.get_gid());
                            }
                            break; // End case NOTIFICATION

                        case NCI_PKT_MT_DATA:
                            ESP_LOGD(TAG, "Received DATA packet, Len=%d", incoming_msg.get_len());
                            if (on_data_packet_) on_data_packet_(incoming_msg);
                            break; // End case DATA

                        case NCI_PKT_MT_CTRL_RESPONSE:
                            if (incoming_msg.get_gid() == RF_GID) {
                                switch (incoming_msg.get_oid()) {
                                    case RF_INTF_ACTIVATED_OID:{
                                        selected_tag_still_in_field.store(true);
                                        selected_tag_still_in_field.notify_all();
                                        if (on_rf_intf_activated_) {
                                            on_rf_intf_activated_(incoming_msg);
                                        }
                                    }
                                    break;
                                    case RF_ISO_DEP_NAK_PRESENCE_OID:{
                                        if (incoming_msg.get_len() >= 1) {
                                            uint8_t presence_status = incoming_msg.buffer[3];
                                            ESP_LOGD(TAG, "RF_ISO_DEP_NAK_PRESENCE_RSP received. Status: 0x%02X", presence_status);
                                            if (presence_status != STATUS_OK) {
                                                rf_deactivate(DEACTIVATION_TYPE_DISCOVERY);
                                            }
                                        } else {
                                            ESP_LOGW(TAG, "RF_ISO_DEP_NAK_PRESENCE_RSP payload too short.");
                                        }
                                    }
                                    break;
                                }
                            } else if (incoming_msg.get_gid() == NCI_CORE_GID) {
                                switch (incoming_msg.get_oid()) {
                                    case NCI_CORE_GENERIC_ERROR_OID:
                                        ESP_LOGW(TAG, "CORE_GENERIC_ERROR_RSP Status: 0x%X", incoming_msg.get_status());
                                        break;
                                    case NCI_CORE_INTERFACE_ERROR_OID:
                                        ESP_LOGW(TAG, "CORE_INTERFACE_ERROR_RSP Status: 0x%X ConnID: %d",
                                                incoming_msg.buffer[3], incoming_msg.buffer[4]);
                                        break;
                                    case NCI_CORE_CONN_CREDITS_OID:
                                        ESP_LOGD(TAG, "CORE_CONN_CREDITS_RSP received");
                                        break;
                                    default:
                                        ESP_LOGW(TAG, "Unhandled Core Notification OID: 0x%02X", incoming_msg.get_oid());
                                        break;
                                }
                            } else {
                                // Otherwise, it's a truly unexpected response
                                ESP_LOGW(TAG, "Unexpected RSP received in task loop: GID=0x%02X, OID=0x%02X",
                                        incoming_msg.get_gid(), incoming_msg.get_oid());
                                ESP_LOG_BUFFER_HEXDUMP(TAG, incoming_msg.data(), incoming_msg.size(), ESP_LOG_WARN);
                            }
                            break; // End case RESPONSE

                        default:
                            ESP_LOGW(TAG, "Unknown NCI Message Type received: 0x%X", incoming_msg.get_mt());
                            break;
                    }
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Timeout reading NCI packet after IRQ trigger");
            } else {
                ESP_LOGE(TAG, "Error reading NCI packet: 0x%X", ret);
                // If a sync operation was in progress, we need to unblock it with an error
                if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (sync_apdu_in_progress_) {
                        ESP_LOGE(TAG, "Signaling APDU sync failure due to read error");
                        sync_apdu_response_.clear(); // Ensure no stale data
                        xSemaphoreGive(apdu_sync_sem_); // Signal failure
                    }
                    xSemaphoreGive(sync_mutex_);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else { // wait_for_irq failed
            ESP_LOGE(TAG, "Error waiting for IRQ semaphore (returned 0x%X)", ret);
             // If a sync operation was in progress, we need to unblock it with an error
            if (xSemaphoreTake(sync_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (sync_apdu_in_progress_) {
                    ESP_LOGE(TAG, "Signaling APDU sync failure due to IRQ wait error");
                    sync_apdu_response_.clear(); // Ensure no stale data
                    xSemaphoreGive(apdu_sync_sem_); // Signal failure
                }
                xSemaphoreGive(sync_mutex_);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } // end while(true)

    ESP_LOGI(TAG, "PN7160 task runner stopping.");
    task_handle_ = nullptr;
    vTaskDelete(NULL);
}
