#include "i2c.hpp"

I2C::I2C() {
#ifndef __arm__
    std::cout << "Not tessing up i2c as running on PC." << std::endl;
#else
    gpioInitialise();
    handles_[I2CMessageType::DRIVE] = i2cOpen(I2C_BUS, DRIVE_ADDR, 0);
    handles_[I2CMessageType::GRABBER] = i2cOpen(I2C_BUS, GRABBER_ADDR, 0);
    handles_[I2CMessageType::DROPPER] = i2cOpen(I2C_BUS, DROPPER_ADDR, 0);
#endif // __arm__
}

I2C::~I2C() {
#ifdef __arm__
    i2cClose(handles_[I2CMessageType::DRIVE]);
    i2cClose(handles_[I2CMessageType::GRABBER]);
    i2cClose(handles_[I2CMessageType::DROPPER]);
    gpioTerminate();
#endif // __arm__
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
#endif // __arm__
}

void I2C::read(I2CMessageType target, unsigned short len, char* buf) {
#ifndef __arm__
    std::cout << "Returning empty data from " << target << std::endl;
#else
    if(len == 8){
        memcpy(buf, error_check_drive_, 8);
    }
    switch(i2cReadDevice(handles_[target], buf, len)) {
    case PI_BAD_HANDLE:
        std::cout << "\033[31mBad Handle\033[0m" << std::endl;
        break;
    case PI_BAD_PARAM:
        std::cout << "\033[31mBad Param\033[0m" << std::endl;
        break;
    case PI_I2C_READ_FAILED:
        std::cout << "\033[31mRead Failed\033[0m" << std::endl;
        break;
    default:
        std::cout << "\033[32mNo Problem\033[0m" << std::endl;
        break;
    }
#endif // __PC_TEST__
}
