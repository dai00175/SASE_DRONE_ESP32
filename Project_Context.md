# Project Context: Quadcopter Flight Controller Migration

## 1. Project Overview
We are migrating a custom quadcopter flight controller from a Teensy 4.1 (Arduino framework) to an **Adafruit Metro ESP32-S3** using the native **ESP-IDF framework**. The primary goal is to leverage the ESP32-S3's dual-core architecture and FreeRTOS to separate time-critical flight stabilization from background tasks and telemetry.

## 2. Hardware Specifications
* **MCU:** Adafruit Metro ESP32-S3 (Xtensa Dual-Core 32-bit LX7, running at 240MHz).
* **IMU:** BNO085 (communicating via SPI, utilizing the INT pin for hardware interrupts).
* **Motors/ESCs:** 4x SimonK ESCs requiring standard PWM pulses (1000µs to 2000µs) operating at a 400Hz refresh rate (2500µs period).
* **Barometer:** SparkFun LPS28DFW (Communicating via I2C. No community available esp component so the arduino library is provided as example)
* **Bluetooth:** HC-05 (Communicating via UART with MCU at max baud rate 38400 if possible. JSONs are sent and received for commands from pc)
* **Ultrasound Distance Sensor** HC-SR04 (Trig and Echo pins)

## 3. Current Architecture (Arduino)
The current Arduino codebase relies on a strict, non-blocking 300Hz loop.
* **Loop Timing:** Uses `micros()` to ensure the main flight loop (IMU read -> PID calculation -> Motor mixing) fires exactly every 3333µs.
* **Motor Control:** Uses `analogWrite` with 12-bit resolution and custom frequency settings to drive the ESCs.
* **IMU Handling:** Uses the Adafruit_BNO08x library. Reads Euler angles (from `SH2_GAME_ROTATION_VECTOR`) and angular velocity (from `SH2_GYROSCOPE_CALIBRATED`). Uses a `while` loop to drain the sensor buffer entirely every cycle.
* **Motor Mixing:** Standard X-configuration mixing (Roll, Pitch, Yaw applied to a base throttle).

## 4. Migration Requirements & Instructions for ESP-IDF

Please refactor the codebase to native ESP-IDF C/C++ adhering to the following architectural guidelines:

### A. Dual-Core FreeRTOS Task Management
Utilize `xTaskCreatePinnedToCore` to separate the workload:
* **Core 1 (APP_CPU) - The Flight Loop:** This core must handle the time-critical 300Hz stabilization loop. It should wait for the 3333µs interval (preferably using an ESP-IDF hardware timer or highly accurate FreeRTOS tick delay), read the IMU, calculate the PID, and update motor outputs.
* **Core 0 (PRO_CPU) - Background Tasks:** This core will handle non-time-critical tasks. This includes UART telemetry output, logging, and reading data from any of the slower "Other Sensors" listed above.
* **Inter-Task Communication:** Use FreeRTOS thread-safe mechanisms (Queues or Mutex-protected structs) to pass data (like telemetry stats or auxiliary sensor data) between Core 1 and Core 0.

### B. Motor Control (PWM) Migration
* Do not use Arduino's `analogWrite`.
* Implement the ESC control using the **ESP-IDF MCPWM (Motor Control PWM)** peripheral (preferred for precise motor control) or the **LEDC** peripheral.
* The frequency must be exactly 400Hz.
* The duty cycle must accurately map 1000µs to 2000µs pulses to the ESCs.
* Example class for ESC exists in motors.cpp but it might not be required.

### C. SPI & IMU Migration
* Port the SPI communication for the BNO085 to use the native ESP-IDF `spi_master` driver.
* Configure the ESP32-S3 GPIO hardware interrupt (`gpio_install_isr_service`) to attach to the BNO085's INT pin, signaling to the flight loop that fresh data is ready. Existing components for the BNO085 is already pulled. Check the component to determine if custom component is required or it is possible to use existing one.

### D. General Code Quality
* Keep the PID math, anti-windup constraints, and X-configuration motor mixing logic intact from the provided source files.
* Provide robust error handling for peripheral initialization (SPI, MCPWM, etc.).
* Ensure all code complies with standard ESP-IDF conventions (using `ESP_LOGI`, `ESP_LOGE`, `vTaskDelay`, etc.).