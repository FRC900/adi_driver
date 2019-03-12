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

#ifndef ADI_DRIVER_ADIS16470_H
#define ADI_DRIVER_ADIS16470_H

#include <termios.h>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

namespace ba = boost::asio;

class Adis16470
{
public:
  // Gyro sensor(x, y, z)
  double gyro[3];
  // Acceleration sensor(x, y, z)
  double accl[3];
  // Temperature sensor
  double temp;

  Adis16470();
  ~Adis16470();
  int openPort(const std::string device);
  void closePort();
  bool isOpened();
  int get_product_id(uint16_t &);
  int update(void);
  int update_burst(void);
  int bias_correction_update(void);
  int set_bias_estimation_time(uint16_t tbc);

private:
  typedef enum
  {
    IDLE,
    WRITE,
    READ,
    TRANSFER,
    TIME_OUT
  } PORT_STATUS;
  ba::io_service port_io;
  ba::io_service wdg_io;
  ba::serial_port port;
  std::thread port_handel_thread;
  ba::streambuf serial_buf;
  const double wdg_timeout;
  PORT_STATUS status;

  bool flush_port();
  bool write_bytes(const std::vector<uint8_t> &, const double);
  bool read_bytes(std::vector<uint8_t> &, const double);
  bool write_register(const uint8_t address, const uint16_t);
  bool read_register(const uint8_t address, uint16_t &);
  void wdg_handler(const boost::system::error_code &);
  void serial_handler(const boost::system::error_code &, std::size_t);
  bool init_usb_iss();
  bool initAdis16470();
};

#endif // ADI_DRIVER_ADIS16470_H
