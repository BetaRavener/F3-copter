                                             F3-copter
                                STM32-F3 Discovery based tricopter
                                        c) Ivan Sevcik, 2013
                           zlib/libpng license  (see LICENSE for details)

## Description
F3-copter is a project aimed at creating tricopter controlled by STM32-F3 Discovery board.
This board has proven to be great choice as it comes with preequiped 3-axis gyrosensor,
accelerometer and magnetometer along with fully featured debugging interface for a very
reasonable price. Also the mounted Cortex-M4 MCU with dedicated FPU gives enough computing
power for even complex algorithms.
The software features following modules:
- RC receiver for controlling tricopter by radio
- UART communicator with custom protocol for commanding the device in real-time
- Sensor readers and filtering
- Engine and servo controll
- PID controller
- Simple math library for 3D vectors
Coded for reusability and ease of use, it is also fast and allows for often sensor updates.

