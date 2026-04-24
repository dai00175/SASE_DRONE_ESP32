#pragma once

#include <array>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/uart.h"

enum class DroneState : uint8_t
{
    IDLE = 0,
    TAKEOFF,
    USERCNTRL,
    PRGMCNTRL,
    LANDING
};

struct BoardConfig
{
    std::array<gpio_num_t, 4> motor_gpios;
    std::array<gpio_num_t, 3> bno_spi_gpios;
    gpio_num_t bno_cs_gpio;
    gpio_num_t bno_int_gpio;
    gpio_num_t bno_rst_gpio;
    gpio_num_t ultrasonic_trig_gpio;
    gpio_num_t ultrasonic_echo_gpio;
    uart_port_t bluetooth_uart_port;
    gpio_num_t bluetooth_tx_gpio;
    gpio_num_t bluetooth_rx_gpio;
    // ENS160/AHT21 support is disabled.
    // gpio_num_t env_i2c_sda_gpio;
    // gpio_num_t env_i2c_scl_gpio;
    gpio_num_t barometer_i2c_sda_gpio;
    gpio_num_t barometer_i2c_scl_gpio;
    spi_host_device_t bno_spi_host;
    int bluetooth_baud_rate;
    int i2c_clock_hz;
    // int ens160_i2c_address;
    int pwm_frequency_hz;
    int pwm_period_us;
    int pwm_min_us;
    int pwm_max_us;
    int flight_loop_period_us;
    int telemetry_period_ms;
    int serial_log_period_ms;
    int bluetooth_timeout_ms;
    int ultrasonic_timeout_us;
};

inline constexpr BoardConfig kBoardConfig = {
    .motor_gpios = {GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8},
    .bno_spi_gpios = {GPIO_NUM_11, GPIO_NUM_13, GPIO_NUM_12},
    .bno_cs_gpio = GPIO_NUM_10,
    .bno_int_gpio = GPIO_NUM_9,
    .bno_rst_gpio = GPIO_NUM_16,
    .ultrasonic_trig_gpio = GPIO_NUM_14,
    .ultrasonic_echo_gpio = GPIO_NUM_15,
    .bluetooth_uart_port = UART_NUM_1,
    .bluetooth_tx_gpio = GPIO_NUM_41,
    .bluetooth_rx_gpio = GPIO_NUM_40,
    // ENS160/AHT21 support is disabled.
    // .env_i2c_sda_gpio = GPIO_NUM_47,
    // .env_i2c_scl_gpio = GPIO_NUM_48,
    .barometer_i2c_sda_gpio = GPIO_NUM_17,
    .barometer_i2c_scl_gpio = GPIO_NUM_18,
    .bno_spi_host = SPI2_HOST,
    .bluetooth_baud_rate = 9600,
    .i2c_clock_hz = 400000,
    // .ens160_i2c_address = 0x53,
    .pwm_frequency_hz = 300,
    .pwm_period_us = 3333,
    .pwm_min_us = 1000,
    .pwm_max_us = 2000,
    .flight_loop_period_us = 3333,
    .telemetry_period_ms = 500,
    .serial_log_period_ms = 3000,
    .bluetooth_timeout_ms = 20000,
    .ultrasonic_timeout_us = 30000,
};
