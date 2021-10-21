#ifndef MAXSONAR_H
#define MAXSONAR_H

#include <stdio.h>
#include <iostream>  // for output to std::cout
#include <linux/i2c-dev.h> // for the ioctl() function
#include <unistd.h> // for the read() and write() function
#include <fcntl.h> // for the open() function include <stdio.h>
#include <string.h> // for the strlen() function
#include <stdlib.h> // for exit
#include <sys/ioctl.h>
#include <errno.h>
#include <chrono>
#include <thread>

// LIDAR-Lite default I2C device address
#define AddressDefault 224      //  0xE0 

// LIDAR-Lite internal register addresses
#define RANGE_ADDR      244     // 0x70
#define RANGE_CMD       81      // 0x51
#define RANGE_REG       225

class MaxSonar {
  public:
    MaxSonar();
  
    bool init();
    uint16_t readDistance();
  
  private:
    
    int fd;  //File Descriptor
    uint8_t address;  //I2C address 
    uint8_t last_status;
  
    // Generic i2c Functions
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);    
};

#endif  // MAXSONAR_H
