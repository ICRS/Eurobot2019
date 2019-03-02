// -*- coding:utf-8-unix -*-
/*!
  @file   AsyncStepper.cpp
  
  @brief  Drive a stepper motor using A4988 with asynchronous.
  
  @author   T.Kawamura
  @version  1.0
  @date     2018-07-12  T.Kawamura  Written for C++/mbed.
  @see
  Copyright (C) 2018 Takuma Kawamura.
  Released under the MIT license.
  http://opensource.org/licenses/mit-license.php
*/
#include "AsyncStepper.hpp"
 
AsyncStepper::AsyncStepper( PinName enablePin, PinName stepPin, PinName dirPin, uint32_t rpm, stepMode_e stepMode, PinName ms1Pin, PinName ms2Pin, PinName ms3Pin, uint32_t oneRotationFullSteps, stopMode_e stopMode ) : stepMode( stepMode ), ms1Pin( ms1Pin ), ms2Pin( ms2Pin ), ms3Pin( ms3Pin ), oneRotationFullSteps( oneRotationFullSteps ), stopMode( stopMode ), halfPulseCount( 0 ), currentMaxStepCount( 0 ), stopPulseOut( true )
{
  ticker = new Ticker();
  enableOut = new DigitalOut( enablePin );
  stepOut = new DigitalOut( stepPin );
  dirOut = new DigitalOut( dirPin );
  if ( ms1Pin != NC && ms2Pin != NC && ms3Pin != NC ) {
    stepModeBus = new BusOut( ms1Pin, ms2Pin, ms3Pin );
    stepModeBus->write( stepMode );
  }
  enableOut->write( 1 );
  stepOut->write( 0 );
  dirOut->write( 0 );
  
  oneRotationSteps = oneRotationFullSteps * ( stepMode + 1 );
  pulseWidth_us = (uint64_t)( 30000000 / ( rpm * oneRotationSteps ) );
  ticker->attach_us( callback(this, &AsyncStepper::ISR_PULSEOUT), pulseWidth_us );
}
 
AsyncStepper::~AsyncStepper()
{
  delete ticker;
  delete enableOut;
  delete stepOut;
  delete dirOut;
  if ( ms1Pin != NC && ms2Pin != NC && ms3Pin != NC ) {
    delete stepModeBus;
  }
}
 
void AsyncStepper::ISR_PULSEOUT( void )
{
  DISABLE_INTERRUPTS;
  
  if ( !stopPulseOut ) {
    stepOut->write( !stepOut->read() );
 
    if ( ++halfPulseCount / 2 > currentMaxStepCount ) {
      if ( stopMode == FREE ) {
        AsyncStepper::Disable();
      }
      halfPulseCount = 0;
      stopPulseOut = true;
    }
  }
  
  RESTORE_INTERRUPTS;
  return;
}
 
void AsyncStepper::SetRPM( uint32_t rpm )
{
  DISABLE_INTERRUPTS;
  
  ticker->detach();
  oneRotationSteps = oneRotationFullSteps * ( stepMode + 1 );
  pulseWidth_us = (uint64_t)( 30000000 / ( rpm * oneRotationSteps ) );
  ticker->attach_us( callback(this, &AsyncStepper::ISR_PULSEOUT), pulseWidth_us );
  
  RESTORE_INTERRUPTS;
  return;
}
 
void AsyncStepper::SetStepMode( stepMode_e stepMode )
{
  if ( ms1Pin != NC && ms2Pin != NC && ms3Pin != NC ) {
    stepModeBus->write( stepMode );
  }
  
  return;
}
 
void AsyncStepper::Enable( void )
{
  enableOut->write( 0 );
  
  return;
}
 
void AsyncStepper::Disable( void )
{
  enableOut->write( 1 );
  
  return;
}
 
bool AsyncStepper::IsStopping( void )
{
  return stopPulseOut;
}
 
void AsyncStepper::Rotate( direction_e direction, uint32_t steps )
{
  DISABLE_INTERRUPTS;
  
  dirOut->write( direction );
  
  AsyncStepper::Enable();
  currentMaxStepCount = steps;
  stopPulseOut = false;
  
  RESTORE_INTERRUPTS;
  return;
}
