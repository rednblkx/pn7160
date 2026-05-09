#pragma once

#include "message.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <array>
#include <cstdint>

// ---------------------------------------------------------------------------
// Event Queue Types
// ---------------------------------------------------------------------------

enum class NciEventType : uint8_t {
    RF_INTF_ACTIVATED,
    RF_DISCOVER,
    RF_DEACTIVATE,
    CORE_NOTIFICATION,
    DATA_PACKET,
    SHUTDOWN,
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
        case NciEventType::SHUTDOWN:           return 4; // highest – never dropped
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
// (RF_DEACTIVATE, SHUTDOWN) are never dropped.
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

    [[nodiscard]] size_t size() const {
        ScopedMutex lock(mutex_, portMAX_DELAY);
        return count_;
    }

private:
    std::array<NciEvent, N> buf_{};
    size_t head_{0}, tail_{0}, count_{0};
    SemaphoreHandle_t mutex_      = nullptr;
    SemaphoreHandle_t data_avail_ = nullptr;
};
