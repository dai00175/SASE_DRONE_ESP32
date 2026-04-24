set(LPS28_API_DIR "../temp/SparkFun LPS28DFW Arduino Library/src/lps28dfw_api")

idf_component_register(
    SRCS
        "bluetooth_link.cpp"
        "flight_control.cpp"
        "imu_component.cpp"
        "main.cpp"
        "motors.cpp"
        "pid_controller.cpp"
        "sensors.cpp"
        "shared_state.cpp"
        "${LPS28_API_DIR}/lps28dfw_reg.c"
    INCLUDE_DIRS
        "."
        "${LPS28_API_DIR}"
    REQUIRES
        cjson
        driver
        esp32_BNO08x
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_i2c
        esp_driver_mcpwm
        esp_driver_spi
        esp_driver_uart
        esp_timer
        freertos
        # k0i05__esp_ahtxx
        # k0i05__esp_ens160
)
