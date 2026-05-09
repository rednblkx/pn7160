#pragma once

#include "transport.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <span>
#include <expected>
#include <vector>

#include "nci/event.hpp"
#include "nci/message.hpp"

// ---------------------------------------------------------------------------
// Zero-Allocation Direct-to-Task Promise
// ---------------------------------------------------------------------------
struct NciPromise {
    std::atomic<TaskHandle_t> waiting_task{nullptr};
    NciMessage        result;
    esp_err_t         terminal_error{ESP_OK};
    uint8_t           expected_gid{0xFF};
    uint8_t           expected_oid{0xFF};

    bool arm(uint8_t gid = 0xFF, uint8_t oid = 0xFF) {
        TaskHandle_t expected = nullptr;
        // Lock-free claim: only succeeds if slot is free
        if (!waiting_task.compare_exchange_strong(expected, xTaskGetCurrentTaskHandle())) {
            return false;
        }
        ulTaskNotifyTake(pdTRUE, 0); // Drain stale signals
        expected_gid   = gid;
        expected_oid   = oid;
        terminal_error = ESP_OK;
        result.clear();
        return true;
    }

    bool complete(const NciMessage& msg, esp_err_t err = ESP_OK) {
        TaskHandle_t task = waiting_task.exchange(nullptr, std::memory_order_acq_rel);
        if (!task) return false;

        result         = msg;
        terminal_error = err;
        xTaskNotify(task, 1, eSetBits); // Instant Wakeup
        return true;
    }

    void cancel() {
        TaskHandle_t task = waiting_task.exchange(nullptr, std::memory_order_acq_rel);
        if (task) {
            terminal_error = ESP_ERR_INVALID_STATE;
            xTaskNotify(task, 1, eSetBits);
        }
    }

    bool fail(esp_err_t err) {
        TaskHandle_t task = waiting_task.exchange(nullptr, std::memory_order_acq_rel);
        if (!task) return false;

        terminal_error = err;
        xTaskNotify(task, 1, eSetBits);
        return true;
    }

    esp_err_t wait(uint32_t timeout_ms) {
        uint32_t notif_val = 0;
        if (xTaskNotifyWait(0, portMAX_DELAY, &notif_val, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            // Only clear the slot if we still own it.  If complete() already
            // fired between the timeout and this line, the result is ready.
            TaskHandle_t me = xTaskGetCurrentTaskHandle();
            waiting_task.compare_exchange_strong(me, nullptr);
            if (waiting_task.load(std::memory_order_acquire) != me) {
                // The slot was already cleared by complete/fail/cancel,
                // so a result is waiting.  Consume the notification.
                xTaskNotifyWait(0, portMAX_DELAY, &notif_val, 0);
                return terminal_error;
            }
            return ESP_ERR_TIMEOUT;
        }
        return terminal_error;
    }
};

// ---------------------------------------------------------------------------
// Main Driver Class
// ---------------------------------------------------------------------------
class PN7160_NCI {
public:
    explicit PN7160_NCI(IPN7160Transport& transport);
    ~PN7160_NCI();

    [[nodiscard]] esp_err_t initialize();
    [[nodiscard]] esp_err_t start();
    void stop();
    void shutdown();

    [[nodiscard]] esp_err_t core_reset(bool reset_config);
    [[nodiscard]] esp_err_t core_init();
    [[nodiscard]] esp_err_t core_set_config(std::span<const uint8_t> config_params);
    [[nodiscard]] esp_err_t core_get_config(std::span<const uint8_t> config_params, NciMessage& rsp);
    [[nodiscard]] esp_err_t rf_discover_map(std::span<const uint8_t> mappings);
    [[nodiscard]] esp_err_t rf_set_listen_mode_routing(std::span<const uint8_t> routing_config);
    [[nodiscard]] esp_err_t rf_start_discovery(std::span<const uint8_t> discovery_config);
    [[nodiscard]] esp_err_t rf_stop_discovery();
    [[nodiscard]] esp_err_t rf_select_target(uint8_t discovery_id, uint8_t protocol, uint8_t interface);
    [[nodiscard]] esp_err_t rf_deactivate(uint8_t type);
    [[nodiscard]] esp_err_t rf_iso_dep_presence_check();

    // Returns the safely copied payload to the App thread
    [[nodiscard]] std::expected<std::vector<uint8_t>, esp_err_t> send_apdu_sync(
        std::span<const uint8_t> c_apdu, uint32_t timeout_ms = nci::PN7160_DEFAULT_TIMEOUT_MS);

    [[nodiscard]] esp_err_t send_command_wait_response(
        const NciMessage& cmd, NciMessage& rsp, uint32_t timeout_ms = nci::PN7160_DEFAULT_TIMEOUT_MS);

    [[nodiscard]] esp_err_t send_data_packet(const NciMessage& data_pkt);
    [[nodiscard]] esp_err_t get_event(NciEvent& event, uint32_t timeout_ms = portMAX_DELAY);

    void task_runner();

    [[nodiscard]] bool is_initialized() const { return initialized_.load(); }
    [[nodiscard]] bool tag_in_field()   const { return tag_in_field_.load(); }
    [[nodiscard]] bool is_task_running() const { return task_running_.load(); }

private:
    [[nodiscard]] esp_err_t read_nci_packet(NciMessage& msg, uint32_t timeout_ms);
    [[nodiscard]] esp_err_t write_nci_packet(const NciMessage& msg);

    bool try_complete_exchange(const NciMessage& msg);
    void signal_sync_failure(esp_err_t err = ESP_FAIL);
    void update_driver_state(const NciMessage& msg);
    void enqueue_notification(const NciMessage& msg);
    void enqueue_data_packet(const NciMessage& msg);
    void handle_rf_notification(uint8_t oid, const NciMessage& msg);
    void handle_core_notification(uint8_t oid, const NciMessage& msg);
    void handle_unsolicited_response(const NciMessage& msg);

    IPN7160Transport& transport_;

    TaskHandle_t       task_handle_ = nullptr;
    std::atomic<bool>  task_running_{false};
    std::atomic<bool>  initialized_{false};
    std::atomic<bool>  stop_flag_{false};
    std::atomic<bool>  cancelled_{false};
    std::atomic<bool>  tag_in_field_{false};

    NciPromise         promise_{};

    static constexpr size_t EVENT_QUEUE_SIZE = 10;
    NciEventRingBuffer<EVENT_QUEUE_SIZE> event_ring_;

    static constexpr const char* TAG = "PN7160_NCI";
};