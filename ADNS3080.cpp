//
// Created by Lars Schwarz on 07.10.2023.
//

#include <iostream>
#include "ADNS3080.h"
#include "main.h"

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

void ADNS3080::configure(uint8_t led_shutter, uint8_t high_resolution) {
  // Configure the sensor
  writeRegister(ADNS3080_CONFIGURATION_BITS, 0b00000000 | led_shutter << 6 | high_resolution << 4);
}

void ADNS3080::motionClear() {
  writeRegister(ADNS3080_MOTION_CLEAR, 0x00);
}

void ADNS3080::motionBurst(uint8_t *motion, int8_t *dx, int8_t *dy, uint8_t *squal, uint16_t *shutter,
                           uint8_t *max_pix) {
  uint8_t tx_data[1] = {0}, rx_data[7] = {0};

  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_MOTION_BURST;
  HAL_SPI_Transmit(&hspi1, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD_MOT);

  // Get motion byte to see if motion occurred since the last call
  HAL_SPI_Receive(&hspi1, rx_data, 7, ADNS3080_TIMEOUT);

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);

  // Calculate values
  *motion = (rx_data[0] & 0b10000000) >> 7;
  if (*motion != 0) {
    *dx = rx_data[1];
    *dy = rx_data[2];
  } else {
    *dx = 0;
    *dy = 0;
  }
  *squal = rx_data[3];
  *shutter = rx_data[4] << 8 | rx_data[5];
  *max_pix = rx_data[6];

}

void ADNS3080::displacement(int8_t *dx, int8_t *dy) {
  uint8_t tx_data[1] = {0}, rx_data[3] = {0};
  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_MOTION_BURST;
  HAL_SPI_Transmit(&hspi1, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD_MOT);

  // Get motion byte to see if motion occurred since the last call
  tx_data[0] = 0x00;
  HAL_SPI_Receive(&hspi1, rx_data, 3, ADNS3080_TIMEOUT);

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);

  // Check if motion occurred
  if ((rx_data[0] & 0b10000000) >> 7 == 1) {
    *dx = rx_data[1];
    *dy = rx_data[2];
  } else {
    *dx = 0;
    *dy = 0;
  }
}

void ADNS3080::frameCapture(uint8_t *frame) {
  uint8_t tx_data[1] = {0}, rx_data[1] = {0};

  // Store pixel values
  writeRegister(ADNS3080_FRAME_CAPTURE, 0x83);

  // Enable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);

  tx_data[0] = ADNS3080_PIXEL_BURST;
  HAL_SPI_Transmit(&hspi1, tx_data, 1, ADNS3080_TIMEOUT);
  delay_us(ADNS3080_T_SRAD);

  // Receive pixels until the first pixel (0,0) is found
  rx_data[0] = 0;
  while ((rx_data[0] & 0b01000000) == 0) {
    HAL_SPI_Receive(&hspi1, rx_data, 1, ADNS3080_TIMEOUT);
    delay_us(ADNS3080_T_LOAD);
  }

  // Receive a complete frame in row mayor
  for( int i = 0; i < 30*30; i++) {
    HAL_SPI_Receive(&hspi1, rx_data, 1, ADNS3080_TIMEOUT);
    frame[i] = rx_data[0] << 2;
    delay_us(ADNS3080_T_LOAD);
  }

  // Disable SPI communication
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  delay_us(ADNS3080_T_LOAD + ADNS3080_T_BEXIT);
}

void ADNS3080::writeRegister(uint8_t reg, uint8_t data) {
  uint8_t tx_data[2] = {static_cast<uint8_t>(0b10000000 | reg), data};
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, tx_data, 2, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to access a register to VFS." << std::endl;
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  delay_us(ADNS3080_T_SWW);
}

uint8_t ADNS3080::readRegister(uint8_t reg) {
  uint8_t tx_data[1] = {static_cast<uint8_t>(0b00000000 | reg)}, rx_data[1] = {0};
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, tx_data, 1, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to write a register to VFS." << std::endl;
  delay_us(ADNS3080_T_SRAD_MOT);
  if (HAL_SPI_Receive(&hspi1, rx_data, 1, ADNS3080_TIMEOUT) != HAL_OK)
    std::cout << "Error while trying to read a register to VFS." << std::endl;
  HAL_GPIO_WritePin(VFS_SPI_CS_GPIO_Port, VFS_SPI_CS_Pin, GPIO_PIN_SET);
  return *rx_data;
}