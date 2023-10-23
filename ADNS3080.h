//
// Created by Lars Schwarz on 07.10.2023.
//

#ifndef STM32_VFS_ADNS3080_H
#define STM32_VFS_ADNS3080_H

#include "main.h"
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
#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30
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


extern SPI_HandleTypeDef hspi1;

class ADNS3080 {
public:
  static void reset();
  bool is_accessible();
  void configure(uint8_t led_shutter, uint8_t high_resolution);
  void motionClear();
  void motionBurst(uint8_t &motion, int8_t &dx, int8_t &dy, uint8_t &squal, uint16_t &shutter,
                   uint8_t &max_pix);
  void displacement(int8_t &dx, int8_t &dy);
  void frameCapture(uint8_t *frame);

private:
  static void writeRegister(uint8_t reg, uint8_t data);
  static uint8_t readRegister(uint8_t reg);
};


#endif //STM32_VFS_ADNS3080_H