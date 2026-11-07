# PN7160 ESP-IDF driver

Native ESP-IDF NCI driver with SPI and I2C transports implementing
`IPN7160Transport`. The NCI engine requires C++23.

For I2C configuration, borrowed-bus ownership, lifecycle requirements, tests and
validation status, see [the native I2C guide](docs/native-i2c.md).

The [standalone example](examples/native_i2c/main/main.cpp) creates a master bus,
runs NCI initialization, and releases the transport before deleting its bus.
Review the example pins for your hardware before building or flashing it.
