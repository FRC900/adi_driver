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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include "adi_driver/adis16495.h"

static SerialPort serial_port;
constexpr double gravity = 9.80665;

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */

int16_t big_endian_to_short(uint8_t *data)
{
  const uint8_t buff[2] = {data[1], data[0]};
  return *reinterpret_cast<const int16_t*>(buff);
}

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */

void short_to_big_endian(uint8_t *buff, int16_t data)
{
  buff[0] = data >> 8;
  buff[1] = data & 0x00ff;
}

/**
 * @brief Constructor
 */

Adis16495::Adis16495()
{
}

/**
 * @brief Destructor
 */

Adis16495::~Adis16495()
{
	serial_port.closePort();
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */

int Adis16495::open_port(const std::string& device)
{
  if (serial_port.openPort(device))
  {
    std::fprintf(stderr, "[Adis1649x] Failed to open serial port.\r\n");
    return -1;
  }
  serial_port.flushPort();

  // Set SPI mode
  const std::vector<uint8_t> init_packet = { 0x5A, 0x02, 0x93, 0x05 };
  if (!serial_port.write_bytes(init_packet))
  {
	serial_port.closePort();
    return -1;
  }
  std::vector<uint8_t> ack(2);
  if (!serial_port.read_bytes(ack))
  {
	serial_port.closePort();
    return -1;
  }
  //std::printf("Ack : %02X %02X\r\n", ack[0], ack[1]);
  if (ack[0] != 0xFF || ack[1] != 0x00)
  {
	serial_port.closePort();
    return -1;
  }
  // Wait 20ms for SPI ready
  usleep(20000);

  std::printf("[Adis1649x] Opened\r\n");

  if (write_register(0x00, 0) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to write IMU to page 0.\r\n");
	serial_port.closePort();
    return -1;
  }

  std::string prod_ver;
  int16_t prod_id;
  if (get_product_id(prod_id))
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to read product ID.\r\n");
	serial_port.closePort();
    return -1;
  }

  switch (prod_id)
  {
	case 0x406f: // 16495
	  accl_scale_factor = 0.25;
	  prod_ver += "ADIS16495";
	  break;
	case 0x4071: // 16497
	  accl_scale_factor = 1.25;
	  prod_ver += "ADIS16497";
	  break;
	default:
	  std::fprintf(stderr, "[Adis1649x] Init failed to decode product ID %4.4x.\r\n", prod_id);
	  serial_port.closePort();
	  return -1;
  }

  if (write_register(0x00, 3) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to write IMU to page 3.\r\n");
	serial_port.closePort();
    return -1;
  }

  int16_t rang_mdl;
  if (read_register(0x12, rang_mdl) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to request RANG_MDL.\r\n");
	serial_port.closePort();
    return -1;
  }
  if (read_register(0x78, rang_mdl) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to read RANG_MDL.\r\n");
	serial_port.closePort();
    return -1;
  }
  switch(rang_mdl)
  {
	case 0x03: // -1
	  k_g = .00625;
	  prod_ver += "-1";
	  break;
	case 0x07: // -2
	  k_g = 0.025;
	  prod_ver += "-2";
	  break;
	case 0x0f: // -3
	  k_g = 0.1;
	  prod_ver += "-3";
	  break;
	default:
	  std::fprintf(stderr, "[Adis1649x] Init failed to decode RANG_MDL %2.2x.\r\n", rang_mdl);
	  serial_port.closePort();
	  return -1;
  }

  int16_t fw_rev;
  if (read_register(0x7e, fw_rev) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to read FIRM_REV.\r\n");
	serial_port.closePort();
  }
  prod_ver += " FW " +
	  std::to_string((static_cast<uint16_t>(fw_rev) >> 12) & 0x0F) +
	  std::to_string((static_cast<uint16_t>(fw_rev) >>  8) & 0x0F) +
	  "." +
	  std::to_string((static_cast<uint16_t>(fw_rev) >>  4) & 0x0F) +
	  std::to_string((static_cast<uint16_t>(fw_rev)      ) & 0x0F);

  int16_t boot_rev;
  if (read_register(0x00, boot_rev) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to read BOOT_REV.\r\n");
	serial_port.closePort();
  }
  prod_ver += " BOOT " +
	  std::to_string((static_cast<uint16_t>(boot_rev) >> 8) & 0xFF) +
	  "." +
	  std::to_string((static_cast<uint16_t>(boot_rev)     ) & 0xFF);

  fprintf(stderr, "Product is : %s\r\n", prod_ver.c_str());

  if (write_register(0x00, 0) < 0)
  {
    std::fprintf(stderr, "[Adis1649x] Init failed to write IMU to page 0.\r\n");
	serial_port.closePort();
    return -1;
  }

  return 0;
}

/**
 * @param data Product ID (0x4056)
 * @retval 0 Success
 * @retval -1 Failed
 */

int Adis16495::get_product_id(int16_t& pid)
{
	if (read_register(0x7e, pid) < 0)
	{
		fprintf(stderr, "[Adis1649x] Error first read register for product ID\r\n");
		return -1;
	}
	pid = 0;
	if (read_register(0x00, pid) < 0)
	{
		fprintf(stderr, "[Adis1649x] Error second read register for product ID\r\n");
		return -1;
	}
	return 0;
}

/**
 * @brief Read data from the register
 * @param address Register address
 * @retval 0 Success
 * @retval -1 Failed
 *
 * - Adress is the first byte of actual address
 * - Actual data at the address will be returned by next call.
 */

int Adis16495::read_register(unsigned char address, int16_t& data)
{
  static std::vector<uint8_t> tx_packet(3);
  tx_packet[0] = 0x61;
  tx_packet[1] = address;
  tx_packet[2] = 0x00;
  if (!serial_port.write_bytes(tx_packet))
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : read_register write_bytes\r\n");
    return -1;
  }
  static std::vector<uint8_t> rx_packet(3);
  std::fill(rx_packet.begin(), rx_packet.end(), 0x00);
  if (!serial_port.read_bytes(rx_packet))
  {
	data = 0;
    std::fprintf(stderr, "[Adis1649x] Serial error : read_register read_bytes\r\n");
    return -1;
  }
  data = big_endian_to_short(&rx_packet[1]);
  //fprintf(stderr, "read_register : address=%2.2x data=%4.4x\r\n", address, data);
  return 0;
}

/**
 * @brief Write data to the register
 * @param address Register address
 * @retval 0 Success
 * @retval -1 Failed
 *
 * - Adress is the first byte of actual address.
 * - Specify data at the adress.
 */

int Adis16495::write_register(unsigned char address, int16_t data)
{
  //fprintf(stderr, "write_register : address=%2.2x data=%4.4x\r\n", address, data);
  static std::vector<uint8_t> tx_packet(5);
  tx_packet[0] = 0x61;
  // Set R~/W bit 1
  tx_packet[1] = address | 0x80;
  tx_packet[3] = (address + 1) | 0x80;
  // Set data
  tx_packet[2] = data & 0xff;
  tx_packet[4] = data >> 8;

  if (!serial_port.write_bytes(tx_packet))
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : write_register write_bytes\r\n");
    return -1;
  }
  static std::vector<uint8_t> rx_packet(5);
  std::fill(rx_packet.begin(), rx_packet.end(), 0x00);
  if (!serial_port.read_bytes(rx_packet))
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : write_register read_bytes\r\n");
    return -1;
  }
  if (rx_packet[0] != 0xff)
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : write_register ACK error : %2.2x\r\n", rx_packet[0]);
    return -1;
  }
  return 0;
}

/**
 * @brief Update all information by bust read
 * @retval 0 Success
 * @retval -1 Failed
 *
 * - See burst read function at pp.15
 * - Data resolution is 16 bit
 */

int Adis16495::update_burst(void)
{
  constexpr size_t burst_size = 43;
  static std::vector<uint8_t> tx_packet(burst_size);
  tx_packet[0] = 0x61;
  // 0x7c00: Burst read function
  tx_packet[1] = 0x7c;
  std::fill(tx_packet.begin() + 2, tx_packet.end(), 0x00);
  fprintf(stderr, "burst write: ");
  for (size_t i = 0; i < tx_packet.size(); i++)
	  fprintf(stderr, " %2.2x", static_cast<unsigned>(tx_packet[i]));
  fprintf(stderr, "\r\n");
  if (!serial_port.write_bytes(tx_packet))
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : update_burst write_bytes\r\n");
    return -1;
  }
  static std::vector<uint8_t> rx_packet(burst_size);
  std::fill(rx_packet.begin(), rx_packet.end(), 0x00);
  if (!serial_port.read_bytes(rx_packet))
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : update_burst read_bytes\r\n");
    return -1;
  }
  fprintf(stderr, "burst read : ");
  for (size_t i = 0; i < rx_packet.size(); i++)
	  fprintf(stderr, " %2.2x", static_cast<unsigned>(rx_packet[i]));
  fprintf(stderr, "\r\n");

  const int16_t diag_stat = big_endian_to_short(&rx_packet[3]);
  if (diag_stat != 0)
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : update_burst diag_stat error: %04x\r\n", static_cast<uint16_t>(diag_stat));
    return -1;
  }

  const uint16_t burst_id = static_cast<uint16_t>(big_endian_to_short(&rx_packet[5]));
  if (burst_id != 0xA5A5)
  {
    std::fprintf(stderr, "[Adis1649x] Serial error : update_burst burst_id error: %04x\r\n", static_cast<uint16_t>(burst_id));
    return -1;
  }

  //Burst read register for ADIS16495-x
  // TEMP_OUT
  temp = 25. + static_cast<double>(big_endian_to_short(&rx_packet[7])) / 80.;
  // X_GYRO_OUT
  gyro[0] = big_endian_to_short(&rx_packet[9]) * M_PI / 180. * k_g;
  // Y_GYRO_OUT
  gyro[1] = big_endian_to_short(&rx_packet[11]) * M_PI / 180. * k_g;
  // Z_GYRO_OUT
  gyro[2] = big_endian_to_short(&rx_packet[13]) * M_PI / 180. * k_g;
  // X_ACCL_OUT
  accl[0] = big_endian_to_short(&rx_packet[15]) * accl_scale_factor * gravity / 1000.;
  // Y_ACCL_OUT
  accl[1] = big_endian_to_short(&rx_packet[17]) * accl_scale_factor * gravity / 1000.;
  // Z_ACCL_OUT
  accl[2] = big_endian_to_short(&rx_packet[19]) * accl_scale_factor * gravity / 1000.;

  return 0;
}


static int32_t convert_out_low_short(uint16_t out, uint16_t low)
{
	return (uint32_t(out) << 16) | uint32_t(low);
}

/**
 * @brief update gyro and accel in high-precision read
 */
int Adis16495::update(void)
{
  int16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3], temp_out;

  //Register for ADIS16495
  read_register(0x10, gyro_low[0]); // Each request has a response 1 read later
  read_register(0x12, gyro_low[0]);
  read_register(0x14, gyro_out[0]);
  read_register(0x16, gyro_low[1]);
  read_register(0x18, gyro_out[1]);
  read_register(0x1a, gyro_low[2]);
  read_register(0x1c, gyro_out[2]);
  read_register(0x1e, accl_low[0]);
  read_register(0x20, accl_out[0]);
  read_register(0x22, accl_low[1]);
  read_register(0x24, accl_out[1]);
  read_register(0x26, accl_low[2]);
  read_register(0x0e, accl_out[2]);
  read_register(0x00, temp_out);

  // temperature convert
  temp = 25. + static_cast<double>(temp_out) / 80.;

  // 32bit convert
  for (size_t i = 0; i < 3; i++)
  {
    gyro[i] = convert_out_low_short(gyro_out[i], gyro_low[i]) * (M_PI / 180.0) * (k_g / static_cast<double>(1<<16));
    accl[i] = convert_out_low_short(accl_out[i], accl_low[i]) * (accl_scale_factor / static_cast<double>(1<<16)) * (gravity / 1000.);
  }
  return 0;
}

/**
 * @brief Save away current page and set a new one
 * @param new_page_id page id to set
 * @retval previous page id
 * @retval -1 error
 */
int16_t Adis16495::set_new_page(int16_t new_page_id)
{
  int16_t old_page_id;
  if (read_register(0x00, old_page_id) < 0) // Request page ID read
  {
	  fprintf(stderr, "[Adis1649x] Set page ID request\r\n");
	  return -1;
  }
  if (read_register(0x00, old_page_id) < 0)
  {
	  fprintf(stderr, "[Adis1649x] Set page ID response\r\n");
	  return -1;
  }

  if (old_page_id != new_page_id)
  {
	  if (write_register(0x00, new_page_id) < 0) // set page 3
	  {
		  fprintf(stderr, "[Adis1649x] Set write page %4.4x\r\n", new_page_id);
		  write_register(0x00, old_page_id);
		  return -1;
	  }
  }
  return 0;
}

/**
 * @brief set bias estimating time (NULL_CNFG)
 * @retval 0 Success
 * @retval -1 Failed
 */
int Adis16495::set_bias_estimation_time(int16_t tbc)
{
  int16_t page_id = set_new_page(3);
  if (page_id < 0)
	  return -1;

  int rc = 0;
  if (write_register(0x0e, tbc) < 0)
  {
	  fprintf(stderr, "[Adis1649x] Set bias write TBC = %4.4x\r\n", tbc);
	  rc = -1;
  }
  if (!rc && read_register(0x0e, tbc) < 0) // request TBC readback
  {
	  fprintf(stderr, "[Adis1649x] Set bias TBC readback request\r\n");
	  rc = -1;
  }
  tbc = 0;
  if (!rc && read_register(0x00, tbc) < 0) // Get TBC request data response
  {
	  fprintf(stderr, "[Adis1649x] Set bias TBC readback response\r\n");
	  rc = -1;
  }
  fprintf(stderr, "TBC: %04x\r\n", tbc);
  if ((page_id != 3) && (write_register(0x00, page_id) < 0)) // restore page ID
  {
	  fprintf(stderr, "[Adis1649x] Set bias restore page ID %4.4x\r\n", page_id);
	  rc = -1;
  }
  return rc;
}

/**
 * @brief Bias correction update (GLOB_CMD)
 * @retval 0 Success
 * @retval -1 Failed
 */

int Adis16495::bias_correction_update(void)
{
  int16_t page_id = set_new_page(3);
  if (page_id < 0)
	  return -1;

  // Bit0: Bias correction update
  int16_t data = 1;
  int rc = 0;
  if (write_register(0x02, data) < 0)
  {
	  fprintf(stderr, "[Adis1649x] Set bias correction command\r\n");
	  rc = -1;
  }
  if ((page_id != 3) && (write_register(0x00, page_id) < 0)) // restore page ID
  {
	  fprintf(stderr, "[Adis1649x] Set bias correction restore page ID %4.4x\r\n", page_id);
	  rc = -1;
  }
  return rc;
}
