// -*- coding:utf-8-unix -*-
/*!
  @file   AsyncStepper.hpp
  
  @brief  Drive a stepper motor using A4988 with asynchronous.
  
  @author   T.Kawamura
  @version  1.0
  @date     2018-07-12  T.Kawamura  Written for C++/mbed.
  @see
  Copyright (C) 2018 Takuma Kawamura.
  Released under the MIT license.
  http://opensource.org/licenses/mit-license.php
*/
#ifndef ASYNCSTEPPER_H
#define ASYNCSTEPPER_H
 
#include "mbed.h"
 
/**
     Disable all interuupts and save the status of interrupts.
     This macro is usable at ONLY Cortex-M Series. 
*/
#define DISABLE_INTERRUPTS uint32_t primask = __get_PRIMASK();  __disable_irq()
/**
     Enable all interuupts when the status of interrupts is ENABLED.
     This macro is usable at ONLY Cortex-M Series. 
*/
#define RESTORE_INTERRUPTS if( !(primask & 1) ) __enable_irq()
 
using namespace std;
 
enum stepMode_e {
  FULL, HALF, QUARTER, EIGHTH, SIXTEENTH
};
enum direction_e {
  NEGATIVE, POSITIVE
};
enum stopMode_e {
  FREE, LOCKED
};
 
/*! 
  @class AsyncStepper
  @brief Drive a stepper motor using A4988 with asynchronous
 */
class AsyncStepper{
private:
  Ticker *ticker;
  DigitalOut *enableOut;
  DigitalOut *dirOut;
  DigitalOut *stepOut;
  BusOut *stepModeBus;
 
  stepMode_e stepMode;
  PinName ms1Pin;
  PinName ms2Pin;
  PinName ms3Pin;
  uint32_t oneRotationFullSteps;
  stopMode_e stopMode;
  uint32_t oneRotationSteps;
  uint64_t pulseWidth_us;
  volatile uint32_t halfPulseCount;
  volatile uint32_t currentMaxStepCount;
  volatile bool stopPulseOut;
  
  void ISR_PULSEOUT( void );
  
  
public:
  /*!
    @brief Create a new AsyncStepper port.
    
    @param enablePin  A4988 Enable pin
    @param stepPin    A4988 Step pin
    @param dirPin     A4988 Direction pin
    @param rpm        RPM
    @param stepMode   Division ratio
    @param ms1Pin     A4988 MS1
    @param ms2Pin     A4988 MS2
    @param ms3Pin     A4988 MS1
    @param oneRotationFullSteps Steps per rotation of your stepper motor
    @param stopMode   Enable/Disable holding torque
  */
  AsyncStepper( PinName enablePin, PinName stepPin, PinName dirPin, uint32_t rpm, stepMode_e stepMode=FULL, PinName ms1Pin=NC, PinName ms2Pin=NC, PinName ms3Pin=NC, uint32_t oneRotationFullSteps=200, stopMode_e stopMode=FREE );
 
  /*! 
    @brief Destrutor of AsyncStepper
  */
  virtual ~AsyncStepper();
 
  /*! 
    @brief Set RPM of the stepper motor
    @param rpm 
  */
  virtual void SetRPM( uint32_t rpm );
 
  /*! 
    @brief Set division ratio of the stepper motor
    @param stepMode division ratio
  */
  virtual void SetStepMode( stepMode_e stepMode );
 
  /*! 
    @brief Apply the current to the stepper motor
  */
  virtual void Enable( void );
 
  /*! 
    @brief Stop the current to the stepper motor
  */
  virtual void Disable( void );
 
  /*! 
    @brief Check the stepper motor stopping
    
    @retval true Yes (Stopping)
    @retval false No
  */
  virtual bool IsStopping( void );
 
  /*! 
    @brief Rotate the stepper motor
    
    @param direction POSITIVE or NEGATIVE
    @param steps Steps of rotation
  */
  virtual void Rotate( direction_e direction, uint32_t steps );
};
 
 
#endif
