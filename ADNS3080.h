//
// Created by Lars Schwarz on 07.10.2023.
//

#ifndef STM32_VFS_ADNS3080_H
#define STM32_VFS_ADNS3080_H

#include "main.h"
#include "types/parameter.h"
#include "types/sensor.h"
#include <vector>


#define ADNS3080_TIMEOUT 200

// Signal delay time:
#define ADNS3080_T_IN_RST 500
#define ADNS3080_T_PW_RESET 10
#define ADNS3080_T_SRAD_MOT 75
#define ADNS3080_T_SWW 50
#define ADNS3080_T_SRAD 50
#define ADNS3080_T_LOAD 10
#define ADNS3080_T_BEXIT 4

// ADNS3080 hardware configure
#define ADNS3080_PIXELS 30
#define ADNS3080_CLOCK_SPEED 24000000

// Register Map
#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60

// Configuration Bits
#define ADNS3080_LED_MODE_ALWAYS_ON 0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED 0x01

#define ADNS3080_RESOLUTION_400 400
#define ADNS3080_RESOLUTION_1600 1600

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF 0x02

#define ADNS3080_FRAME_RATE_MAX 6469
#define ADNS3080_FRAME_RATE_MIN 2000

class ADNS3080 {
public:
  /// @brief Visual flow sensor object
  /// @param[in,out] hspi SPI interface to communicate with the connected sensor.
  /// @param[in] parameter Parameters which contains the information about the hardware implementation of the Sensor an configurable settings.
  ADNS3080(SPI_HandleTypeDef &hspi, const Parameter::Vfs &parameter);

  /// @brief Sends a hardware reset to the sensor.
  void reset();

  /// @brief Checks if the sensor is accessible.
  /// @return true if the sensor is accessible.
  bool is_accessible();

  /// @brief Initializer to setup the sensor
  /// @return SUCCESS if the sensor is ready for use
  /// @todo This function can be eliminated if the Constructor is called inside the main function and after the initialisation of the hspi
  uint8_t init();

  /// @brief Clears the current motion data in the sensor buffer
  void motionClear();

  /// @brief Reads the current motion data (dy, dy, surface quality) form the sensor buffer.
  /// @param vfs Structure to store the motion data.
  void motionBurst(Sensor::Vfs* vfs);

  /// @brief Reads the current motion data (dy, dy) form the sensor buffer.
  /// @param vfs Structure to store the motion data.
  void displacement(Sensor::Vfs* vfs);

  /// @brief Reads the current camera frame from the sensor buffer and prints it to the console to view it with a python visualizer.
  /// @attention This call will erase the current firmware from the sensor. A power cycle is needed to reload the firmware.
  void frameCapture();

private:
  /// @brief Wrapper for the built in HAL_SPI_Transmit function for better ease of use.
  /// @param reg Register to write to.
  /// @param data Data which needs to be writen to the corresponding register.
  void writeRegister(const uint8_t reg, const uint8_t data);

  /// @brief Wrapper for the built in HAL_SPI_Receive function for better ease of use.
  /// @param reg Register to read from.
  /// @return Data which is stored in the corresponding register.
  uint8_t readRegister(const uint8_t reg);

  SPI_HandleTypeDef &_hspi;
  const Parameter::Vfs &_parameter;
};


#endif //STM32_VFS_ADNS3080_H