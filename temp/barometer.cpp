#include "baromter.h"
#include <SparkFun_LPS28DFW_Arduino_Library.h>
#ifdef __IMXRT1062__
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
#include "i2c_driver.h"
#endif

// Moving average buffers (static inside this file)
static float pressureBuffer[BARO_WINDOW_SIZE];
static int bufferIndex = 0;
static float runningSum = 0.0f;
static bool bufferFull = false;

// Latest values
static float rawPressure = 0.0f;
static float filteredPressure = 0.0f;
static float altitude = 0.0f;

#ifdef __IMXRT1062__
enum BarometerI2CState {
    BARO_STATE_IDLE,
    BARO_STATE_WRITE_REGISTER,
    BARO_STATE_READ_PRESSURE,
    BARO_STATE_ERROR,
};

BarometerI2CState baroState = BARO_STATE_IDLE;

static uint8_t baroWriteBuffer[1] = { LPS28DFW_PRESS_OUT_XL };
static uint8_t baroReadBuffer[3] = { 0 };
#endif

// Helper: write a register using Wire for initialization
static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission((uint8_t)LPS28DFW_I2C_ADDRESS_DEFAULT);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

// Helper: read a single byte using Wire for initialization
static bool readReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission((uint8_t)LPS28DFW_I2C_ADDRESS_DEFAULT);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)LPS28DFW_I2C_ADDRESS_DEFAULT, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

bool initBarometer() {
    Wire.setClock(400000);
    Wire.begin(); // Ensure I2C is started on the Teensy4 I2C driver

    // Check WHO_AM_I
    uint8_t whoami = 0;
    if (!readReg(LPS28DFW_WHO_AM_I, whoami)) return false;
    if (whoami != LPS28DFW_ID) return false;

    // Configure for 200 Hz ODR and 4-sample averaging.
    const uint8_t ctrlReg1 = (uint8_t)LPS28DFW_200Hz | (uint8_t)LPS28DFW_4_AVG;
    if (!writeReg(LPS28DFW_CTRL_REG1, ctrlReg1)) return false;

    // Enable block data update (BDU) in CTRL_REG2. Keep default 1260 hPa range.
    const uint8_t ctrlReg2 = (1 << 3); // bdu = 1
    if (!writeReg(LPS28DFW_CTRL_REG2, ctrlReg2)) return false;

    // Optional: CTRL_REG3 (interrupt config) – leave default

    for (int i = 0; i < BARO_WINDOW_SIZE; i++) {
        while (!updateBarometer()) {
            delay(1);
        }
        delay(5);
    }

    return true;
}

bool updateBarometer() {
    bool success = false;
#ifdef __IMXRT1062__
    switch(baroState) {
        case BARO_STATE_IDLE:{
            Master.write_async(LPS28DFW_I2C_ADDRESS_DEFAULT, baroWriteBuffer, sizeof(baroWriteBuffer), false);
            baroState = BARO_STATE_WRITE_REGISTER;
            success = false;
            break;
        }
        case BARO_STATE_WRITE_REGISTER: {
            if (!Master.finished()) {
                success = false;
                break;
            }
            if (Master.error() != I2CError::ok) {
                baroState = BARO_STATE_ERROR;
                success = false;
                break;
            }
            Master.read_async(LPS28DFW_I2C_ADDRESS_DEFAULT, baroReadBuffer, sizeof(baroReadBuffer), true);
            baroState = BARO_STATE_READ_PRESSURE;
            success = false;
            break;
        }
        case BARO_STATE_READ_PRESSURE:{
            if (!Master.finished()) {
                success = false;
                break;
            }

            if (Master.error() != I2CError::ok) {
                baroState = BARO_STATE_ERROR;
                success = false;
                break;
            }

            size_t received = Master.get_bytes_transferred();
            if (received != sizeof(baroReadBuffer)) {
                baroState = BARO_STATE_ERROR;
                success = false;
                break;
            }

            uint32_t raw24 = 0;
            raw24 |= (uint32_t)baroReadBuffer[0];
            raw24 |= (uint32_t)baroReadBuffer[1] << 8;
            raw24 |= (uint32_t)baroReadBuffer[2] << 16;

            int32_t pressure_raw = (int32_t)raw24;
            if (pressure_raw & 0x800000) {
                pressure_raw |= 0xFF000000;
            }

            pressure_raw <<= 8;
            rawPressure = lps28dfw_from_fs1260_to_hPa(pressure_raw);

            runningSum -= pressureBuffer[bufferIndex];
            pressureBuffer[bufferIndex] = rawPressure;
            runningSum += rawPressure;
            bufferIndex = (bufferIndex + 1) % BARO_WINDOW_SIZE;
            if (bufferIndex == 0) {
                bufferFull = true;
            }

            int samples = bufferFull ? BARO_WINDOW_SIZE : bufferIndex;
            filteredPressure = runningSum / (float)samples;

            const float SEA_LEVEL_HPA = 1013.25f;
            float ratio = filteredPressure / SEA_LEVEL_HPA;
            altitude = 44330.0f * (1.0f - powf(ratio, 0.1903f));

            baroState = BARO_STATE_IDLE;
            success = true;
            break;
        }
        case BARO_STATE_ERROR:{
            baroState = BARO_STATE_IDLE;
            success = false;
            break;
        }
        default:{
            success = false;
            break;
        }
    }

    return success;

#else
    Wire.beginTransmission(LPS28DFW_I2C_ADDRESS_DEFAULT);
    Wire.write(LPS28DFW_PRESS_OUT_XL);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(LPS28DFW_I2C_ADDRESS_DEFAULT, (uint8_t)3) != 3) {
        return false;
    }

    uint32_t raw24 = 0;
    raw24 |= Wire.read();
    raw24 |= (Wire.read() << 8);
    raw24 |= (Wire.read() << 16);

    int32_t pressure_raw = (int32_t)raw24;
    if (pressure_raw & 0x800000) {
        pressure_raw |= 0xFF000000;
    }

    pressure_raw <<= 8;
    rawPressure = lps28dfw_from_fs1260_to_hPa(pressure_raw);

    runningSum -= pressureBuffer[bufferIndex];
    pressureBuffer[bufferIndex] = rawPressure;
    runningSum += rawPressure;
    bufferIndex = (bufferIndex + 1) % BARO_WINDOW_SIZE;
    if (bufferIndex == 0) {
        bufferFull = true;
    }

    int samples = bufferFull ? BARO_WINDOW_SIZE : bufferIndex;
    filteredPressure = runningSum / (float)samples;

    const float SEA_LEVEL_HPA = 1013.25f;
    float ratio = filteredPressure / SEA_LEVEL_HPA;
    altitude = 44330.0f * (1.0f - powf(ratio, 0.1903f));

    return true;
#endif
}

float getAltitude() {
    return altitude;
}

float getRawPressure() {
    return rawPressure;
}

float getFilteredPressure() {
    return filteredPressure;
}