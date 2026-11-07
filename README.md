# PN7160 NCI driver

The NCI engine communicates through `IPN7160Transport`. The existing
`PN7160_SPI` implementation uses native ESP-IDF SPI. `PN7160_ArduinoI2C`
provides an additional transport for Arduino ESP32's shared `TwoWire` bus.

## Arduino ESP32 I2C

```cpp
#include <Wire.h>
#include "arduino_i2c.hpp"
#include "pn7160.hpp"

// SDA, SCL, IRQ, VEN, optional DWL (-1 if not connected), address, clock.
PN7160_ArduinoI2C transport(Wire, 21, 22, 14, 13, 19, 0x28, 400000);
PN7160_NCI nci(transport);

// During setup, check the result of nci.initialize() before configuring RF
// discovery or exchanging NCI commands. Keep both objects alive while in use.
// Stop the NCI worker before deinitializing/destroying its transport.
```

GPIOs, address and supply wiring must match the attached board. The transport
does not set PMU/VANT values, generate ECP frames, authenticate credentials or
decide access permissions; those belong to the NCI configuration/application.
The source was extracted from an Arduino PN7161 integration; simulated transport
tests do not establish hardware coverage for every PN7160/PN7161 board.

The caller supplies `TwoWire`. The transport initializes it with the requested
pins/clock, uses a 258-byte buffer and a 20 ms I2C timeout, and never calls
`Wire.end()`. Other bus users must agree on bus configuration, including timeout
and buffer settings. Its native device handle belongs only to this instance.
Stop concurrent NCI operations before shutdown; synchronize lifecycle changes
with any other bus users. Copying/moving the transport is disabled.

IDF 5.4+ Arduino cores use the native synchronous I2C device API on Arduino's
existing bus. This permits bounded NACK retries before reporting final errors,
without Arduino's premature per-attempt error log. ACK checks remain enabled.
Writes wait for IRQ to become inactive and retry NACKs up to three times;
timeouts and other errors are returned without replay. Native reads retry only
address NACKs while IRQ remains active and clear failed receive buffers.
An older Arduino I2C fallback remains available in the source.

`library.json` enables Arduino ESP32/PlatformIO discovery without renaming the
upstream project to the integration-specific name PN7161. The NCI engine still
requires C++23; configure the consuming environment accordingly. The native
ESP-IDF `CMakeLists.txt` and SPI implementation are unchanged; they do not
compile or depend on the Arduino-only transport.

## Transport regression tests

```sh
python3 tests/arduino_i2c/run.py
```

This compiles the actual transport and upstream interface with scripted
Arduino/IDF APIs under AddressSanitizer and UndefinedBehaviorSanitizer. It checks
NCI header/payload transfers, short reads, IRQ gating, NACK retries, non-replay
on timeouts, tick wrap, partial initialization failures and shared-bus teardown.
IDF 5.3/5.4/5.5/6.0 error-mapping branches are simulated; these are not builds
against all four real SDKs. Hardware validation and full SDK builds remain
separate checks.

## Arduino reader facade

`PN7160_ArduinoReader` in `reader.hpp` combines transport ownership and the NCI
engine into a lifecycle/discovery/UID/APDU interface. Set `Config::wire` to a
shared `TwoWire` instance for I2C, or leave it null for SPI2 and supply SPI pins
in CS, SCLK, MISO, MOSI order. IRQ and VEN are explicit in both modes.
Call from one application task or synchronize access externally. The NCI engine
owns its background runner; `stop()` stops it before releasing the transport.
Copying and moving the facade are disabled.

`init()` configures the reader and starts its runner. `beginDiscovery()` enables
passive NFC-A discovery. `pollForTag()` returns UID/ATQA/SAK, including repeated
activation events; the caller decides whether to suppress repeated presentations.
`isTagStillPresent()` uses ISO-DEP presence probes and the engine's asynchronous
presence state. A true result is not a fresh UID detection or authentication.
`exchangeApdu()` operates only on a selected ISO-DEP tag.

ECP is disabled by default. `setEcpFrame()` copies an optional 18-byte frame;
`setEcpFrame(nullptr, 0)` stages disabling it. Set the frame before initialization
or call `updateECP()` to apply a live change and restart discovery. The setter
checks length, not CRC: the caller supplies a valid frame. The NCI configuration
uses the first 16 bytes because the controller appends CRC-A. This API does not
generate application identifiers or perform credential authentication.

`Config::vantMv` selects the carried PN7161 PMU configuration for 3300/5000 mV;
zero preserves the original automatic PMU query/configuration behavior. ECP and
PMU commands are model-specific: this facade is not a claim of verified support
on every PN7160/PN7161 variant or board. Match configuration to actual hardware.

Real polling errors and failed presence writes invalidate connection state.
`healthCheck()` cannot turn a latched connection fault into a healthy result;
the caller must reinitialize. An idle polling timeout leaves the connection up.
Failed APDU calls clear the output buffer, and failed initialization releases
the engine/transport. The facade does not implement an application retry policy.

Run `python3 tests/reader/run.py` for sanitizer tests compiling the real reader
and I2C transport with the upstream packet representation, a scripted NCI engine
and simulated SDK/SPI/I2C APIs. These exercise lifecycle, UID sizes, APDU gating,
copied/disabled ECP data and connection faults; they do not test actual RF,
NCI worker concurrency, or native SDK linking. Original donor attribution is
retained in `LICENSE.HomeKey-ESP32`.
