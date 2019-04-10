#ifndef __I2C_HPP__
#define __I2C_HPP__

#include <map>
#include <string>

#define __PC_TEST__

#ifdef __PC_TEST__
#include <iostream> // for debugging
#else
#include <pigpio>
#endif // __PC_TEST__

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
    std::string read(I2CMessageType target, unsigned short len);
private:
    std::map<I2CMessageType, int> handles_;
};

#endif // __I2C_HPP__
