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
Adis16470::Adis16470() : port_io(), port(port_io), wdg_timeout(0.1), status(PORT_STATUS::IDLE)
{
}

Adis16470::~Adis16470()
{
  closePort();
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int Adis16470::openPort(const std::string device)
{
  if (isOpened())
  {
    std::fprintf(stderr, "[Adis16470] Failed to open. Already opened.\r\n");
    return -1;
  }

  boost::system::error_code ec;
  ec.clear();
  port.open(device, ec);
  if (ec.value() != 0)
  {
    std::fprintf(stderr, "[Adis16470] Failed to open. Error code : %d", ec.value());
    return -1;
  }
  ec.clear();
  port.set_option(ba::serial_port_base::character_size(8), ec);
  if (ec.value() != 0)
  {
    std::fprintf(stderr, "[Adis16470] Failed to set options. Error code : %d", ec.value());
    return -1;
  }

  if (!init_usb_iss())
  {
    std::fprintf(stderr, "Failed to initialize a USB-ISS to SPI mode.\r\n");
    return -1;
  }

  std::printf("[Adis16470] Opened\r\n");

  return 0;
}

/**
 * @brief Close device
 */
void Adis16470::closePort()
{
  if (isOpened())
  {
    port.cancel();
    port.close();
    //port_handel_thread.join();
  }
}

bool Adis16470::isOpened()
{
  return port.is_open();
}

/**
 * @param data Product ID (0x4056)
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
 * @brief Update all information by bust read
 * @retval 0 Success
 * @retval -1 Failed
 *
 * - See burst read function at pp.14
 * - Data resolution is 16 bit
 */
int Adis16470::update_burst(void)
{
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
  temp = big_endian_to_short(&buff[16]) * 0.1;
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

  if(!read_register(0x04, gyro_low[0])){
    return -1;
  }
  if(!read_register(0x06, gyro_out[0])){
    return -1;
  }
  if(!read_register(0x08, gyro_low[1])){
    return -1;
  }
  if(!read_register(0x0a, gyro_out[1])){
    return -1;
  }
  if(!read_register(0x0c, gyro_low[2])){
    return -1;
  }
  if(!read_register(0x0e, gyro_out[2])){
    return -1;
  }
  if(!read_register(0x10, accl_low[0])){
    return -1;
  }
  if(!read_register(0x12, accl_out[0])){
    return -1;
  }
  if(!read_register(0x14, accl_low[1])){
    return -1;
  }
  if(!read_register(0x16, accl_out[1])){
    return -1;
  }
  if(!read_register(0x18, accl_low[2])){
    return -1;
  }
  if(!read_register(0x1A, accl_out[2])){
    return -1;
  }
  if(!read_register(0x1C, temp_out)){
    return -1;
  }
  //printf("update start read_register done\r\n");

  // temperature convert
  temp = (double)temp_out * 0.1;

  //printf("update start rconvert\r\n");
  // 32bit convert
  for (int i = 0; i < 3; i++)
  {
    gyro[i] = (int32_t)((uint32_t(gyro_out[i]) << 16) | uint32_t(gyro_low[i])) * M_PI / 180.0 / 655360.0;
    accl[i] = (int32_t)((uint32_t(accl_out[i]) << 16) | uint32_t(accl_low[i])) * 9.8 / 52428800.0;
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

bool Adis16470::flush_port()
{
  return tcflush(port.lowest_layer().native_handle(), TCIOFLUSH) == 0 ? true : false;
}

void Adis16470::wdg_handler(const boost::system::error_code& ec)
{
  if (!ec)
  {
    switch (status)
    {
      case PORT_STATUS::READ:
        std::fprintf(stderr, "[Adis16470] SPI read timeouted\r\n");
        break;

      case PORT_STATUS::WRITE:
        std::fprintf(stderr, "[Adis16470] SPI write timeouted\r\n");
        break;
    }
	status = PORT_STATUS::TIME_OUT;
  }
  else
  {
    if (ec.value() != ECANCELED)
    {
      std::fprintf(stderr, "[Adis16470] ??? wdg handle error. Code : %d\r\n", ec.value());
    }
	status = PORT_STATUS::SERIAL_ERROR;
  }
  port.cancel();
}

void Adis16470::serial_handler(const boost::system::error_code& ec)
{
  if(ec){
    std::fprintf(stderr, "[ADIS16470] seiral error : %s\r\n", ec.message().c_str());
	status = PORT_STATUS::SERIAL_ERROR;
  }
}

bool Adis16470::write_bytes(const std::vector<uint8_t>& tx_bytes, const double timeout = 1.0)
{
  if (status != PORT_STATUS::IDLE)
  {
    std::fprintf(stderr, "[Adis16470] SPI write while status not IDLE\r\n");
    return false;
  }

  port_io.reset();
  ba::deadline_timer wdg(port_io, boost::posix_time::millisec(timeout * 1000));
  status = PORT_STATUS::WRITE;
  ba::async_write(port, ba::buffer(tx_bytes),
		  boost::bind(&Adis16470::serial_handler, this, ba::placeholders::error));

  port_io.run();
  wdg.cancel();

  if (status == PORT_STATUS::TIME_OUT)
  {
    std::fprintf(stderr, "[Adis16470] SPI write timeout\r\n");
	status = PORT_STATUS::IDLE;
    return false;
  }
  if (status == PORT_STATUS::SERIAL_ERROR)
  {
    std::fprintf(stderr, "[Adis16470] SPI write error\r\n");
	status = PORT_STATUS::IDLE;
    return false;
  }
  status = PORT_STATUS::IDLE;
  return true;
}

bool Adis16470::read_bytes(std::vector<uint8_t>& rx_bytes, const double timeout = 1.0)
{
  if (status != PORT_STATUS::IDLE)
  {
    std::fprintf(stderr, "[Adis16470] SPI read while status not IDLE\r\n");
    return false;
  }

  port_io.reset();
  ba::deadline_timer wdg(port_io, boost::posix_time::millisec(timeout * 1000));
  status = PORT_STATUS::READ;
  wdg.async_wait(boost::bind(&Adis16470::wdg_handler, this, ba::placeholders::error));
  ba::async_read(port, ba::buffer(rx_bytes),
		  boost::bind(&Adis16470::serial_handler, this, ba::placeholders::error));

  port_io.run();
  wdg.cancel();

  if (status == PORT_STATUS::TIME_OUT)
  {
    std::fprintf(stderr, "[Adis16470] SPI read timeout\r\n");
	status = PORT_STATUS::IDLE;
    return false;
  }
  if (status == PORT_STATUS::SERIAL_ERROR)
  {
    std::fprintf(stderr, "[Adis16470] SPI read error\r\n");
	status = PORT_STATUS::IDLE;
    return false;
  }
  status = PORT_STATUS::IDLE;
  return true;
}

bool Adis16470::write_register(const uint8_t address, const uint16_t data)
{
  static std::vector<uint8_t> tx_packet(3);
  static std::vector<uint8_t> rx_packet(3);
  tx_packet[0] = 0x61;
  tx_packet[1] = address | 0x80;
  tx_packet[2] = data & 0xFF;
  flush_port();
  if (!write_bytes(tx_packet))
  {
    std::fprintf(stderr, "[Adis16470] SPI write while status not IDLE\r\n");
    return false;
  }
  if (!read_bytes(rx_packet))
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
  flush_port();
  if (!write_bytes(tx_packet))
  {
    return false;
  }
  if (!read_bytes(rx_packet))
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

bool Adis16470::read_register(const uint8_t address, uint16_t& data)
{
  static std::vector<uint8_t> tx_packet(3);
  static std::vector<uint8_t> rx_packet(3);
  tx_packet[0] = 0x61;
  tx_packet[1] = address & ~0x80;
  tx_packet[2] = 0x00;
  //flush_port();
  if (!write_bytes(tx_packet))
  {
    return false;
  }
  if (!read_bytes(rx_packet))
  {
    return false;
  }
  if (rx_packet[0] != 0xFF)
  {
    std::fprintf(stderr, "[Adis16470] Recieve NACK\r\n");
    return false;
  }

  //flush_port();
  if (!write_bytes(tx_packet))
  {
    return false;
  }
  if (!read_bytes(rx_packet))
  {
    return false;
  }
  if (rx_packet[0] != 0xFF)
  {
    std::fprintf(stderr, "[Adis16470] Recieve NACK\r\n");
    return false;
  }

  //std::printf("TX packet : %02X %02X %02X\r\n", tx_packet[0], tx_packet[1], tx_packet[2]);
  //std::printf("RX packet : %02X %02X %02X\r\n", rx_packet[0], rx_packet[1], rx_packet[2]);

  data = (static_cast<uint16_t>(rx_packet[1]) << 8) | static_cast<uint16_t>(rx_packet[2]);
  return true;
}

bool Adis16470::init_usb_iss()
{
  flush_port();
  std::vector<uint8_t> init_packet = { 0x5A, 0x02, 0x93, 5 };
  if (!write_bytes(init_packet))
  {
    return false;
  }

  std::vector<uint8_t> ack(2);
  if (!read_bytes(ack))
  {
    return false;
  }
  std::printf("Ack : %02X %02X\r\n", ack[0], ack[1]);
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
