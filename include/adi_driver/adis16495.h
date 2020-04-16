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

#ifndef ADI_DRIVER_ADIS16495_H
#define ADI_DRIVER_ADIS16495_H

#include <termios.h>
#include <array>
#include <string>

#include "adi_driver/serial_port.h"

class Adis16495
{
public:
  // Gyro sensor(x, y, z)
  std::array<double, 3> gyro;
  // Acceleration sensor(x, y, z)
  std::array<double, 3> accl;

  // Reported device temperature
  double temp;

  Adis16495();
  ~Adis16495();
  int open_port(const std::string device);
  int get_product_id(int16_t& data);
  int update(void);
  int update_burst(void);
  int read_register(unsigned char address, int16_t& data);
  int write_register(unsigned char address, int16_t data);
  int set_bias_estimation_time(int16_t tbc);
  int bias_correction_update(void);
private:
  int16_t set_new_page(int16_t new_page_id);
  SerialPort serial_port;
  double k_g;
  double accl_scale_factor; // 495 vs 497 have different accl scale factors
};

#endif  // ADI_DRIVER_ADIS16470_H
