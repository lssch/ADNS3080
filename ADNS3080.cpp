//
// Created by Lars Schwarz on 07.10.2023.
//

#include <cmath>
#include <iostream>
#include <valarray>
#include "ADNS3080.h"
#include "main.h"

ADNS3080::ADNS3080(SPI_HandleTypeDef &hspi, const Parameter::Vfs &parameter)
  : _hspi(hspi),
    _parameter(parameter){
}

void ADNS3080::reset() {
  HAL_GPIO_WritePin(VFS_RESET_GPIO_Port, VFS_RESET_Pin, GPIO_PIN_SET);
  delay_us(ADNS3080_T_PW_RESET);
  HAL_GPIO_WritePin(VFS_RESET_GPIO_Port, VFS_RESET_Pin, GPIO_PIN_RESET);
  delay_us(ADNS3080_T_IN_RST);
}

bool ADNS3080::is_accessible() {
  if (readRegister(ADNS3080_PRODUCT_ID) == 0x17) return true;
  return false;
}

uint8_t ADNS3080::init() {
  reset();
  if (not is_accessible()) return ERROR;
  // Configure the sensor
  writeRegister(ADNS3080_CONFIGURATION_BITS, 0b00000000 | _parameter.led_shutter << 6 | _parameter.led_shutter << 4);
  motionClear();
  return SUCCESS;
}

void ADNS3080::motionClear() {
  writeRegister(ADNS3080_MOTION_CLEAR, 0x00);
}

void ADNS3080::motionBurst(Sensor::Vfs* vfs) {
  uint8_t tx_data[1] = {0}, rx_data[7] = {0};

  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_MOTION_BURST;
  HAL_SPI_Transmit(&_hspi, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD_MOT);

  // Get motion byte to see if motion occurred since the last call
  HAL_SPI_Receive(&_hspi, rx_data, 7, ADNS3080_TIMEOUT);

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);

  // Check if motion occurred
  // Calculate values if motion occurred
  // old tan value 0.22595393133353245f
  Cartesian2<int8_t> motion{0,0};
  if ((rx_data[0] & 0b10000000) >> 7){
    if (_parameter.high_resolution)
      vfs->motion = {static_cast<float>(-static_cast<int8_t>(rx_data[2]))*2*_parameter.height/(ADNS3080_PIXELS*1600)
                     *std::tan(static_cast<float>(_parameter.measured_target_length)/static_cast<float>(_parameter.height)),
                     static_cast<float>(static_cast<int8_t>(rx_data[1]))*2*_parameter.height/(ADNS3080_PIXELS*1600)
                     *std::tan(static_cast<float>(_parameter.measured_target_length)/static_cast<float>(_parameter.height))};
    else
      vfs->motion = {static_cast<float>(-static_cast<int8_t>(rx_data[2]))*2*_parameter.height/(ADNS3080_PIXELS*400)
                     *std::tan(static_cast<float>(_parameter.measured_target_length)/static_cast<float>(_parameter.height)),
                     static_cast<float>(static_cast<int8_t>(rx_data[1]))*2*_parameter.height/(ADNS3080_PIXELS*400)
                     *std::tan(static_cast<float>(_parameter.measured_target_length)/static_cast<float>(_parameter.height))};
  } else
    vfs->motion = {0,0};

  vfs->surface_quality = rx_data[3];
}

void ADNS3080::displacement(Sensor::Vfs* vfs) {
  uint8_t tx_data[1] = {0}, rx_data[3] = {0};
  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_MOTION_BURST;
  HAL_SPI_Transmit(&_hspi, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD_MOT);

  // Get motion byte to see if motion occurred since the last call
  tx_data[0] = 0x00;
  HAL_SPI_Receive(&_hspi, rx_data, 3, ADNS3080_TIMEOUT);

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);

  // Check if motion occurred
  // Calculate values if motion occurred
  Cartesian2<int8_t> motion{0,0};
  if ((rx_data[0] & 0b10000000) >> 7){
    motion = {-static_cast<int8_t>(rx_data[2]), static_cast<int8_t>(rx_data[1])};
  }

  vfs->motion = {static_cast<float>(motion.x)*2*_parameter.height/(ADNS3080_PIXELS*1600)*0.22595393133353245f,
                 static_cast<float>(motion.y)*2*_parameter.height/(ADNS3080_PIXELS*1600)*0.22595393133353245f};
}

void ADNS3080::frameCapture() {
  std::array<std::array<uint8_t, ADNS3080_PIXELS>, ADNS3080_PIXELS> image{};
  uint8_t tx_data[1] = {0}, rx_data[1] = {0};

  // Store pixel values
  writeRegister(ADNS3080_FRAME_CAPTURE, 0x83);

  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_PIXEL_BURST;
  HAL_SPI_Transmit(&_hspi, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD);

  // Receive _pixels until the first pixel (0,0) is found
  rx_data[0] = 0;
  while ((rx_data[0] & 0B01000000) == 0) {
    HAL_SPI_Receive(&_hspi, rx_data, 1, ADNS3080_TIMEOUT);
    delay_us(ADNS3080_T_LOAD);
  }

  uint8_t first_pixel = rx_data[0] - 64;

  delay_us(2*ADNS3080_T_LOAD);

  // Receive a complete frame in row mayor
  for (int y = 0; y < ADNS3080_PIXELS; ++y) {
    for (int x = 0; x < ADNS3080_PIXELS; ++x) {
      image.at(x).at(y) = rx_data[0];
      HAL_SPI_Receive(&_hspi, rx_data, 1, ADNS3080_TIMEOUT);
      delay_us(ADNS3080_T_LOAD);
    }
  }

  image.at(0).at(0) = first_pixel;

  std::cout << "A" << std::endl;
  for (int y = 0; y < ADNS3080_PIXELS; ++y) {
    for (int x = 0; x < ADNS3080_PIXELS; ++x) {
      std::cout << +image.at(x).at(y) << std::endl;
    }
  }

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  delay_us(ADNS3080_T_LOAD + ADNS3080_T_BEXIT);
}

void ADNS3080::writeRegister(const uint8_t reg, const uint8_t data) {
  uint8_t tx_data[2] = {static_cast<uint8_t>(0b10000000 | reg), data};
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&_hspi, tx_data, 2, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to access a register to VFS." << std::endl;
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  delay_us(ADNS3080_T_SWW);
}

uint8_t ADNS3080::readRegister(const uint8_t reg) {
  uint8_t tx_data[1] = {static_cast<uint8_t>(0b00000000 | reg)}, rx_data[1] = {0};
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&_hspi, tx_data, 1, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to write a register to VFS." << std::endl;
  delay_us(ADNS3080_T_SRAD_MOT);
  if (HAL_SPI_Receive(&_hspi, rx_data, 1, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to read a register to VFS." << std::endl;
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  return *rx_data;
}