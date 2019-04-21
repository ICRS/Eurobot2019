#ifndef __I2C_HPP__
#define __I2C_HPP__

#include <map>
#include <string>
#include <string.h>

#ifndef __arm__
#include <iostream> // for debugging
#else
#include <pigpio.h>
#endif // __arm__

#define I2C_BUS 1

#define DRIVE_ADDR 0x08
#define GRABBER_ADDR 0x00
#define DROPPER_ADDR 0x00

enum I2CMessageType {
    DRIVE,
    GRABBER,
    DROPPER
};

class I2C {
public:
    I2C();
    ~I2C();
    // Write a *message* to the *target*
    void write(I2CMessageType target, std::string message);
    // Read a message of length *len* to the *target*
    void read(I2CMessageType target, unsigned short len, char* buf);
private:
    std::map<I2CMessageType, int> handles_;
    char error_check_drive_[8] = {(char) 128, '\0', (char) 128, '\0', (char)  128, '\0', (char) 128, '\0'};
};

#endif // __I2C_HPP__
