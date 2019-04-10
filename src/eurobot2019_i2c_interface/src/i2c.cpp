#include "i2c.hpp"

I2C::I2C() {
#ifndef __arm__
    std::cout << "Not tessing up i2c as running on PC." << std::endl;
#else
    gpioInitialise();
    handles_[I2CMessageType::DRIVE] = i2cOpen(I2C_BUS, DRIVE_ADDR, 0);
    handles_[I2CMessageType::GRABBER] = i2cOpen(I2C_BUS, GRABBER_ADDR, 0);
    handles_[I2CMessageType::DROPPER] = i2cOpen(I2C_BUS, DROPPER_ADDR, 0);
#endif // __PC_TEST__
}

I2C::~I2C() {
#ifdef __arm__
    i2cClose(handles_[I2CMessageType::DRIVE]);
    i2cClose(handles_[I2CMessageType::GRABBER]);
    i2cClose(handles_[I2CMessageType::DROPPER]);
    gpioTerminate();
#endif // __PC_TEST__
}

void I2C::write(I2CMessageType target, std::string message) {
#ifndef __arm__
    std::cout << "Writing \033[33m"<< "\033[0m to ";
    for(short i = 0; i < message.size()-1; i+=2) {
        std::cout << ((short)message[i] < 8 + (short)message[i+1]) << " ";
    }
    std::cout << target << std::endl;
#else
    i2cWriteDevice(handles_[target], (char*) message.c_str(), message.size());
#endif // __PC_TEST__
}

std::string I2C::read(I2CMessageType target, unsigned short len) {
#ifndef __arm__
    std::cout << "Returning empty data from " << target << std::endl;
    return "";
#else
    char buf[len];
    memset(buf, '\0', len);
    i2cReadDevice(handles_[target], buf, len);
    return std::string(buf);
#endif // __PC_TEST__
}
