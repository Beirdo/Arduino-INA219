/**************************************************************************/
/*!
    @file     INA219.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	Driver for the INA219 current sensor

	This is a library for the Adafruit INA219 breakout
	----> https://www.adafruit.com/products/???
		
	Adafruit invests time and resources providing this open source code,
	please support Adafruit and open-source hardware by purchasing
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include "INA219.h"

/**************************************************************************/
/*!
        @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void INA219::wireWriteRegister (uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(m_i2caddr);
    Wire.setClock(400000);
    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits
    Wire.endTransmission();
}

/**************************************************************************/
/*!
        @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void INA219::wireReadRegister(uint8_t reg, uint16_t *value)
{
    Wire.beginTransmission(m_i2caddr);
    Wire.write(reg);                       // Register
    Wire.endTransmission();

    delay(1); // Max 12-bit conversion time is 586us per sample

    Wire.requestFrom(m_i2caddr, (uint8_t)2);

    // Shift values to create properly formed integer
    *value = ((Wire.read() << 8) | Wire.read());
}

void INA219::setCalibration(uint8_t maxVoltage, uint16_t maxVShunt,
                            uint16_t rShunt, float maxIExpected)
{
    // We support 16V FSR and 32V FSR on Bus Voltage
    if (maxVoltage > 16) {
        maxVoltage = 32;
    } else {
        maxVoltage = 16;
    }

    // maxVShunt in mV, rShunt in mOhms
    // We support 40mV, 80mV, 160mV, 320mV FSR on vShunt
    if (maxVShunt > 160) {
        maxVShunt = 320;
    } else if (maxVShunt > 80) {
        maxVShunt = 160;
    } else if (maxVShunt > 40) {
        maxVShunt = 80;
    } esle {
        maxVShunt = 40;
    }

    // float maxIPossible = maxVShunt * 1.0 / rShunt;
    // //float minLSB = maxIExpected / 32767.0;
    // //float maxLSB = maxIExpected / 4096.0;
    float currentLSB = maxIExpected / 16384.0;
    uint16_t calibrate = (uint16_t)(40.96 / (currentLSB * rShunt));
    float powerLSB = currentLSB * 20.0;
    // float maxICalculated = currentLSB * 32767.0;
    // float maxIOverflow = min(maxIPossible, maxICalculated);
    // float maxVShuntCalc = maxICalculated * rShunt / 1000.0;
    // float maxVShutOverflow = min(maxVShunt / 1000.0, maxVShuntCalc);
    // float maxPower = maxIOverflow * maxVoltage;

    // Set Calibration register to 'Cal' calculated above	
    m_calValue = calibrate;
    wireWriteRegister(INA219_REG_CALIBRATION, m_calValue);

    // Set Config register to take into account the settings above
    m_configValue = (maxVoltage == 32 ? INA219_CONFIG_BVOLTAGERANGE_32V :
                     INA219_CONFIG_BVOLTAGERANGE_16V) |
                    (maxVShunt == 40 ? INA_CONFIG_GAIN_1_40MV :
                     (maxVShunt == 80 ? INA_CONFIG_GAIN_2_80MV :
                      (maxVShunt == 160 ? INA_CONFIG_GAIN_4_160MV :
                       INA219_CONFIG_GAIN_8_320MV))) |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    wireWriteRegister(INA219_REG_CONFIG, m_configValue);

    m_current_lsb = currentLSB;
    m_power_lsb = powerLSB;
}

/**************************************************************************/
/*!
        @brief  Instantiates a new INA219 class
*/
/**************************************************************************/
INA219::INA219(uint8_t addr) {
    m_i2caddr = addr;
    m_current_lsb = 0.0;
    m_power_lsb = 0.0;
}

/**************************************************************************/
/*!
        @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void INA219::begin(uint8_t addr) {
    m_i2caddr = addr;
    begin();
}

void INA219::begin(void) {
    Wire.begin();
}

/**************************************************************************/
/*!
        @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t INA219::getBusVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_BUSVOLTAGE, &value);

    // Shift to the right 3 to drop CNVR and OVF and multiply by 4mV LSB
    return (int16_t)((value >> 1) & 0xFFFC);
}

/**************************************************************************/
/*!
        @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t INA219::getShuntVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
    return (int16_t)value;
}

/**************************************************************************/
/*!
        @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t INA219::getCurrent_raw() {
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    wireWriteRegister(INA219_REG_CALIBRATION, m_calValue);

    // Now we can safely read the CURRENT register!
    wireReadRegister(INA219_REG_CURRENT, &value);

    return (int16_t)value;
}

/**************************************************************************/
/*!
        @brief  Gets the raw power value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t INA219::getPower_raw() {
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    wireWriteRegister(INA219_REG_CALIBRATION, m_calValue);

    // Now we can safely read the POWER register!
    wireReadRegister(INA219_REG_POWER, &value);

    return (int16_t)value;
}

/**************************************************************************/
/*!
        @brief  Gets the shunt voltage in mV (so +-327mV)
*/
/**************************************************************************/
float INA219::getShuntVoltage_mV() {
    int16_t value;
    value = getShuntVoltage_raw();
    return value * 0.01;
}

/**************************************************************************/
/*!
        @brief  Gets the shunt voltage in mV
*/
/**************************************************************************/
uint32_t INA219::getBusVoltage_mV() {
    int16_t value = getBusVoltage_raw();
    return (uint32_t)(value < 0 : -value : value);
}

/**************************************************************************/
/*!
        @brief  Gets the current value in mA, taking into account the
                        config settings and current LSB
*/
/**************************************************************************/
uint32_t INA219::getCurrent_mA() {
    int16_t value = getCurrent_raw();
    int32_t valueOut = (int32_t)(value * m_current_lsb * 1000.0);
    return (uint32_t)(valueOut < 0 : -valueOut : valueOut);
}

/**************************************************************************/
/*!
        @brief  Gets the power value in mW, taking into account the
                        config settings and power LSB
*/
/**************************************************************************/
uint32_t INA219::getPower_mW() {
    int16_t value = getPower_raw();
    int32_t valueOut = (int32_t)(value * m_power_lsb * 1000.0);
    return (uint32_t)(valueOut < 0 : -valueOut : valueOut);
}


// vim:ts=4:sw=4:ai:et:si:sts=4
