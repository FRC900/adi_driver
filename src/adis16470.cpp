// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "adi_driver/adis16470.h"
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include "ros/ros.h"

/**
 * @brief Constructor
 */
Adis16470::Adis16470()
{
}

/**
 * @brief Destructor
 */
Adis16470::~Adis16470()
{
  serial_port.closePort();
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int Adis16470::open_port(const std::string &device)
{
  if (serial_port.openPort(device))
  {
    std::fprintf(stderr, "[Adis16470] Failed to open serial port.\r\n");
    return -1;
  }

  if (!init_usb_iss())
  {
    std::fprintf(stderr, "Failed to initialize a USB-ISS to SPI mode.\r\n");
	serial_port.closePort();
    return -1;
  }

  std::printf("[Adis16470] Opened\r\n");

  return 0;
}

/**
 * @param pid Product ID (0x4056)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16470::get_product_id(uint16_t& pid)
{
  if (!read_register(0x72, pid))
  {
    return -1;
  }
  return 0;
}

/**
 *  @brief Convert big endian bytestream to a short
 *  @param buf Input buffer
 *  @retval converted value
 */
int16_t Adis16470::big_endian_to_short(const uint8_t *buf)
{
	return (static_cast<uint16_t>(buf[0]) << 8) | (static_cast<uint16_t>(buf[1]) & 0x00FF);
}

/**
 * @brief Update all information by burst read
 * @retval 0 Success
 * @retval -1 Failed
 *
 * - See burst read function at pp.14
 * - Data resolution is 16 bit
 */
int Adis16470::update_burst(void)
{
  static std::vector<uint8_t> burstWriteBuf(23);
  // 0x6800: Burst read function
  burstWriteBuf[0] = 0x61;
  burstWriteBuf[1] = 0x68;
  burstWriteBuf[2] = 0x00;
  if (!serial_port.write_bytes(burstWriteBuf))
  {
	  perror("burst write burstWriteBuf");
	  return -1;
  }
  serial_port.flushPort(); // TODO - necessary?
  static std::vector<uint8_t> burstReadBuf(23);
  if (!serial_port.read_bytes(burstReadBuf))
  {
	  perror("burst read burstreadbuf");
	  return -1;
  }
  const uint16_t diag_stat = big_endian_to_short(&burstReadBuf[3]);
  if (diag_stat != 0)
  {
    fprintf(stderr, "burst diag_stat error: expected 0000, read %04x\n", diag_stat);
    return -1;
  }

  uint16_t calced_checksum = 0;
  for (size_t i = 3; i < (3+18); i++)
	  calced_checksum += static_cast<uint16_t>(burstReadBuf[i]) & 0x00FF;
  const uint16_t read_checksum = big_endian_to_short(&burstReadBuf[21]);
  if (calced_checksum != read_checksum)
  {
	  std::stringstream s;
	  for (const auto b : burstReadBuf)
	  {
		  s << " ";
		  if (b < 16)
			  s << " ";
		  s << std::hex << static_cast<unsigned>(b);
	  }
	  ROS_INFO_STREAM(s.str().c_str());
	  fprintf(stderr, "burst checksum error: calculated %04x, read %04x\n",
			  calced_checksum, read_checksum);
	  return -1;
  }

  //
  // X_GYRO_OUT
  gyro[0] = big_endian_to_short(&burstReadBuf[5]) * M_PI / 180 / 10.0;
  // Y_GYRO_OUT
  gyro[1] = big_endian_to_short(&burstReadBuf[7]) * M_PI / 180 / 10.0;
  // Z_GYRO_OUT
  gyro[2] = big_endian_to_short(&burstReadBuf[9]) * M_PI / 180 / 10.0;
  // X_ACCL_OUT
  accl[0] = big_endian_to_short(&burstReadBuf[11]) * (1.25 / 1000.) * 9.80665;
  // Y_ACCL_OUT
  accl[1] = big_endian_to_short(&burstReadBuf[13]) * (1.25 / 1000.) * 9.80665;
  // Z_ACCL_OUT
  accl[2] = big_endian_to_short(&burstReadBuf[15]) * (1.25 / 1000.) * 9.80665;
  // TEMP_OUT
  temp = big_endian_to_short(&burstReadBuf[17]) * 0.1;

  return 0;
#if 0
  unsigned char buff[64] = {0};
  // 0x6800: Burst read function
  buff[0] = 0x61;
  buff[1] = 0x68;
  buff[2] = 0x00;
  int size = write(fd_, buff, 24);
  if (size != 24)
  {
    perror("update_burst");
    return -1;
  }
  if (tcdrain(fd_) < 0)
  {
    perror("update_burst");
    return -1;
  }
  size = read(fd_, buff, 30);
  if (size != 30)
  {
    perror("update_burst");
    return -1;
  }
  int16_t diag_stat = big_endian_to_short(&buff[3]);
  if (diag_stat != 0)
  {
    fprintf(stderr, "diag_stat error: %04x\n", (uint16_t)diag_stat);
    return -1;
  }
  // X_GYRO_OUT
  gyro[0] = big_endian_to_short(&buff[5]) * M_PI / 180 / 10.0;
  // Y_GYRO_OUT
  gyro[1] = big_endian_to_short(&buff[7]) * M_PI / 180 / 10.0;
  // Z_GYRO_OUT
  gyro[2] = big_endian_to_short(&buff[9]) * M_PI / 180 / 10.0;
  // X_ACCL_OUT
  accl[0] = big_endian_to_short(&buff[11]) * M_PI / 180 / 10.0;
  // Y_ACCL_OUT
  accl[1] = big_endian_to_short(&buff[13]) * M_PI / 180 / 10.0;
  // Z_ACCL_OUT
  accl[2] = big_endian_to_short(&buff[15]) * M_PI / 180 / 10.0;
  // TEMP_OUT
  temp = big_endian_to_short(&buff[17]) * 0.1;
#endif
  return 0;
}

/**
 * @brief update gyro and accel in high-precision read
 */
int Adis16470::update(void)
{
  uint16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3], temp_out;
  //printf("update start read_register\r\n");

  if(!read_register_half_transaction(0x04, gyro_low[0])){ // First read doesn't return anything
    return -1;
  }
  if(!read_register_half_transaction(0x06, gyro_low[0])){ // next read gets the previous req plus
    return -1;                               // requests the next address
  }
  if(!read_register_half_transaction(0x08, gyro_out[0])){
    return -1;
  }
  if(!read_register_half_transaction(0x0a, gyro_low[1])){
    return -1;
  }
  if(!read_register_half_transaction(0x0c, gyro_out[1])){
    return -1;
  }
  if(!read_register_half_transaction(0x0e, gyro_low[2])){
    return -1;
  }
  if(!read_register_half_transaction(0x10, gyro_out[2])){
    return -1;
  }
  if(!read_register_half_transaction(0x12, accl_low[0])){
    return -1;
  }
  if(!read_register_half_transaction(0x14, accl_out[0])){
    return -1;
  }
  if(!read_register_half_transaction(0x16, accl_low[1])){
    return -1;
  }
  if(!read_register_half_transaction(0x18, accl_out[1])){
    return -1;
  }
  if(!read_register_half_transaction(0x1a, accl_low[2])){
    return -1;
  }
  if(!read_register_half_transaction(0x1C, accl_out[2])){
    return -1;
  }
  if(!read_register_half_transaction(0x1C, temp_out)){
    return -1;
  }
  //printf("update start read_register done\r\n");

  // temperature convert
  temp = (double)temp_out * 0.1;

  //printf("update start rconvert\r\n");
  // 32bit convert
  for (int i = 0; i < 3; i++)
  {
    gyro[i] = (int32_t)((uint32_t(gyro_out[i]) << 16) | (uint32_t(gyro_low[i]) & 0x0000FFFF)) * M_PI / 180.0 / 655360.0;
    accl[i] = (int32_t)((uint32_t(accl_out[i]) << 16) | (uint32_t(accl_low[i]) & 0x0000FFFF)) * 9.80665 * 1.25 / (double)(1<<16);
  }
  //printf("update start convert done\r\n");
  return 0;
}

/**
 * @brief set bias estimating time (GLOB_CMD)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16470::set_bias_estimation_time(uint16_t tbc)
{
  if(!write_register(0x66, tbc)){
    return -1;
  }
  tbc = 0;
  uint16_t dummy = 0;
  if(!read_register(0x66, dummy)){
    return -1;
  }
  std::printf("TBC: %04x\r\n", dummy);
  return 0;
}

/**
 * @brief Bias correction update (GLOB_CMD)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16470::bias_correction_update(void)
{
  return write_register(0x68, 0x01) ? 0 : -1;
}

int Adis16470::set_filt_ctrl(const uint16_t filt)
{
  if(filt > 8){
    std::fprintf(stderr, "[Adis16470] Failed to set FILT_CTRL value\r\n");
    return -1;
  }
  if(!write_register(0x5C, filt)){
    return -1;
  }
  uint16_t dummy = 0;
  if(!read_register(0x5C, dummy)){
    return -1;
  }
  std::printf("FILT_CTRL: %04x\r\n", dummy);
  return 0;
}

int Adis16470::set_dec_rate(const uint16_t rate)
{
  if(rate > 1999){
    std::fprintf(stderr, "[Adis16470] Failed to set DEC_RATE value\r\n");
    return -1;
  }
  if(!write_register(0x64, rate)){
    return -1;
  }
  uint16_t dummy = 0;
  if(!read_register(0x64, dummy)){
    return -1;
  }
  std::printf("DEC_RATE: %04x\r\n", dummy);
  return 0;
}


bool Adis16470::write_register(const uint8_t address, const uint16_t data)
{
  static std::vector<uint8_t> tx_packet(3);
  static std::vector<uint8_t> rx_packet(3);
  tx_packet[0] = 0x61;
  tx_packet[1] = address | 0x80;
  tx_packet[2] = data & 0xFF;
  if (!serial_port.write_bytes(tx_packet))
  {
    std::fprintf(stderr, "[Adis16470] SPI write while status not IDLE\r\n");
    return false;
  }
  if (!serial_port.read_bytes(rx_packet))
  {
    return false;
  }
  if (rx_packet[0] != 0xFF)
  {
    std::fprintf(stderr, "[Adis16470] Recieve NACK\r\n");
    return false;
  }

  ++tx_packet[1];
  tx_packet[2] = (data >> 8) & 0xFF;
  if (!serial_port.write_bytes(tx_packet))
  {
    return false;
  }
  if (!serial_port.read_bytes(rx_packet))
  {
    return false;
  }
  if (rx_packet[0] != 0xFF)
  {
    std::fprintf(stderr, "[Adis16470] Recieve NACK\r\n");
    return false;
  }

  return true;
}

/*
 * @brief - request a register read. Data is available on the next read
 * @param address 7-bit address to read
 * @param data holds return data
 * @retval true Sucess
 * @retval false Failure
 */
bool Adis16470::read_register_half_transaction(const uint8_t address, uint16_t& data)
{
  static std::vector<uint8_t> tx_packet(3);
  static std::vector<uint8_t> rx_packet(3);
  tx_packet[0] = 0x61;
  tx_packet[1] = address & ~0x80;
  tx_packet[2] = 0x00;
  if (!serial_port.write_bytes(tx_packet))
  {
    return false;
  }
  if (!serial_port.read_bytes(rx_packet))
  {
    return false;
  }
  if (rx_packet[0] != 0xFF)
  {
    std::fprintf(stderr, "[Adis16470] Recieve NACK\r\n");
    return false;
  }
  data = big_endian_to_short(&rx_packet[1]);
  return true;
}

bool Adis16470::read_register(const uint8_t address, uint16_t& data)
{
	return (read_register_half_transaction(address, data) &&
	        read_register_half_transaction(address, data));
}

bool Adis16470::init_usb_iss()
{
  serial_port.flushPort();
  const std::vector<uint8_t> init_packet = { 0x5A, 0x02, 0x93, 0x05 };
  if (!serial_port.write_bytes(init_packet))
  {
    return false;
  }

  std::vector<uint8_t> ack(2);
  if (!serial_port.read_bytes(ack))
  {
    return false;
  }
  //std::printf("Ack : %02X %02X\r\n", ack[0], ack[1]);
  if (ack[0] != 0xFF || ack[1] != 0x00)
  {
    return false;
  }
  return true;
}

bool Adis16470::initAdis16470()
{
	return true;
}
