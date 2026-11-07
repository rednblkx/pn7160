# Native ESP-IDF I2C transport

`include/i2c.hpp` and `src/i2c.cpp` implement `IPN7160Transport` using only
ESP-IDF GPIO, FreeRTOS, and the synchronous `i2c_master` API. Minimum SDK: 5.4.
SPI and the NCI engine are unchanged. CMake includes I2C on supported SDKs
and retains C++23. The transport accepts an existing native master bus handle;
application-specific reader facades and bus bridges are outside this component.

## Ownership and scheduling

The caller supplies an already-created master bus and must keep it alive until
after transport destruction. The transport registers/removes only its device;
it never creates, deletes, reinitializes or remaps a bus. The IRQ/VEN/optional
DWL pins are exclusive to this transport and must differ from SDA/SCL and other
peripherals. IRQ is polled by level, at one RTOS tick intervals, without an ISR.
VEN changes include a 10 ms settling delay, matching the SPI transport.
VEN is optional; without it, recovery requires an external reset/power cycle.

Stop all NCI users before transport deinit/destruction. Init/deinit and other
bus/device lifecycle changes require external serialization. Synchronous
transfers on different device handles are serialized by IDF's bus semaphore;
this does not make arbitrary bus reconfiguration safe. Do not attach asynchronous
callbacks, access the NFC address through a second driver, or probe/scan it.

Reads retain NCI's separate 3-byte header and up-to-255-byte payload transfers.
Each transfer permits 1..258 bytes. Failed reads clear the output. A transaction
has a finite configurable timeout (default 20 ms). NACKs on supported SDKs get
at most three attempts with 5 ms gaps, rounded up to a tick. Writes also wait
up to 20 ms per attempt for IRQ to fall; allow this extra time in caller budgets.
Timeouts and unknown errors are returned immediately without replaying a frame.
On IDF 5.4/5.5 ACK errors are reported as ESP_ERR_INVALID_STATE; IDF 6 uses
ESP_ERR_INVALID_RESPONSE. The caller must not remove/reconfigure the bus/device
behind the transport, which could otherwise invalidate that distinction.

## Standalone example

With an ESP-IDF 5.4+ installation and a C++23-capable toolchain:

```sh
cd examples/native_i2c
idf.py set-target esp32
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Review `main/main.cpp` wiring before flashing. The example owns its own bus,
initializes NCI (CORE_RESET + CORE_INIT), logs the result, destroys the transport,
and only then deletes its bus. It does not enroll credentials or enable ECP.
Use suitable external pull-ups and 3.3 V signaling.

## Host tests

From this library's root (no Arduino headers or stubs are used):

```sh
CXX=clang++ python3 test/native_i2c/run.py
```

These tests cover lifecycle/partial initialization, borrowed-bus preservation,
header/payload framing, maximum frames, IRQ levels/delays/wraparound, bounded
NACK retries, failed-read clearing, and no timeout replay. They simulate driver
results; they do not prove electrical framing, a second physical peripheral,
concurrent hardware operation, or compatibility with every PN716x firmware.

## Validation status

Host tests compile the actual native transport against simulated IDF API results.
The runner exercises the 5.4, 5.5 and 6.0 error-code branches with sanitizers;
this is not equivalent to building against all three SDKs.

An ESP32-S3 application using Arduino-ESP32 3.3.8 / ESP-IDF 5.5.4 compiled the
native transport with the unchanged SPI and NCI sources. On a connected PN7161,
startup logs showed successful initialization/discovery and tag exchanges, and
the tester confirmed successful iPhone HomeKey authentication after correcting
an unrelated application permission-download issue. The application bridge is
not included here; the transport itself includes no Arduino code.

The standalone `idf.py` example has not been built locally because a standalone
ESP-IDF installation is unavailable. CI is configured to build that example.
Standalone-example hardware execution, physical PN7160 I2C testing, SPI hardware
regression, and concurrent traffic with another physical I2C peripheral remain
unverified. The existing SPI and NCI implementation files are unchanged.
