// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Library to drive the I2CXL-MaxSonar®- EZTM Series High Performance Sonar 
 * Rangefinder MB1202, MB1212, MB1222, MB1232, MB1242
 * Reduced functionality, just enough to take a range measurement
 * ***********************************************************************/

#include <maxsonar/maxsonar.h>

// Constructor ////////////////////////////////////////////
MaxSonar::MaxSonar()
  : address(AddressDefault)
{ }

// Public Methods ////////////////////////////////////////
//Open the i2c port for reading and writing
//and initialise the sensor
bool MaxSonar::init() {
  // Open it i2c port
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
      return false;
  }

  // Setup flow control
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    return false;
  }
  
  return true;
}  
 
 
/*------------------------------------------------------------------------------
  Read Distance
  Initiate a distance measurement by writing 0x51 to register 0x70.
  Read and return result of distance measurement.
------------------------------------------------------------------------------*/
uint16_t MaxSonar::readDistance()
{
    using namespace std::this_thread;     // sleep_for, sleep_until
    using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
  
    uint8_t commandByte = RANGE_CMD;
    writeReg(RANGE_ADDR, commandByte);
  
    sleep_for(100ms);        

    uint16_t   distance;
    distance = readReg16Bit(RANGE_REG);

    return distance;
}
             
             
// Private Methods ////////////////////////////////////////

//Write an 8-bit register
void MaxSonar::writeReg(uint8_t reg, uint8_t value)
{
  uint8_t writeBuffer[2];

  writeBuffer[0] = reg;
  writeBuffer[1] = value;

  auto result = write(fd, writeBuffer, sizeof(writeBuffer));
  last_status = (result > 0) ? 0 : 1;
}

// Read an 8-bit register
uint8_t MaxSonar::readReg(uint8_t reg)
{
  uint8_t  writeBuffer[1];

  writeBuffer[0] = reg;

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < (long int)sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "DEBUG: ERROR on write to i2c" << std::endl;
    return 0x00;
  }

  uint8_t readBuffer[1];
  read(fd, readBuffer, sizeof(readBuffer));

  return readBuffer[0];
}

//Read 16 bits from a register
uint16_t MaxSonar::readReg16Bit(uint8_t reg) {

  uint8_t  writeBuffer[1];
  writeBuffer[0] = reg;

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < (long int)sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "ERROR on write to i2c" << std::endl;
    return 0x0000;
  }

  uint8_t readBuffer[2];
  if( read(fd, readBuffer, sizeof(readBuffer)) < (long int)sizeof(readBuffer)) {
    std::cout << "Error on read from i2c" << std::endl;
    return 0x0000;
  }

  uint16_t value = (uint16_t)readBuffer[0] << 8;  //value high byte
  value |= readBuffer[1];                         // value low byte

  return value;
}
