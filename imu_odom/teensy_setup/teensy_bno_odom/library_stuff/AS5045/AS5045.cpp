/*
  AS5045.cpp - Library for encoder
  Created by Aaron Chong, December 2017
  minebot, Team B, MRSD 2018
*/

#include "Arduino.h"
#include "AS5045.h"

encoder::encoder(int clock_pin, int CSn_pin, int input_pin, float wheel_diam)
{
  _clock_pin  = clock_pin;
  _CSn_pin    = CSn_pin;
  _input_pin  = input_pin;
  _wheel_diam = wheel_diam;
}

void encoder::setup_rotary_encoder()
{
  pinMode(_clock_pin, OUTPUT); // SCK
  pinMode(_CSn_pin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
  pinMode(_input_pin, INPUT); // SDA
  debug = 0;
}

void encoder::calibrate_rotary_encoder()
{
  _resting_angle = 0;
  _resting_angle = rotary_data();
  Serial.print("Angle calibrated, new 0 at ");
  Serial.println(_resting_angle);
}

float encoder::rotary_data()
{
  digitalWrite(_CSn_pin, HIGH); // CSn high
  digitalWrite(_clock_pin, HIGH); // CLK high
  delayMicroseconds(5); // wait for 1 second for no particular reason
  digitalWrite(_CSn_pin, LOW); // CSn low: start of transfer
  delayMicroseconds(5); // delay for chip -- 1000x as long as it needs to be
  digitalWrite(_clock_pin, LOW); // CLK goes low: start clocking
  delayMicroseconds(5); // hold low for 10 ms
  for (int x=0; x < 18; x++) // clock signal, 18 transitions, output to clock pin
  { 
    digitalWrite(_clock_pin, HIGH); // clock goes high
    delayMicroseconds(5); // wait 5 microsec
    input_stream = digitalRead(_input_pin); // read one bit of data from pin
    //Serial.print(inputstream, DEC); // useful if you want to see the actual bits
    packed_data = ((packed_data << 1) + input_stream); // left-shift summing variable, add pin value
    digitalWrite(_clock_pin, LOW);
    delayMicroseconds(5); // end of one clock cycle
  } // end of entire clock cycle

  angle = packed_data & angle_mask; // mask rightmost 6 digits of packeddata to zero, into angle.
  angle = (angle >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  angle_float = angle * resolution; // angle * (360/4096) == actual degrees

  //arduino's C++ can't handle remainder of floats, here's a workaround
  angle_temp = round((angle_float - _resting_angle + 360.0) * 10000);
  angle_temp = angle_temp % (360*10000);
  true_angle = angle_temp / 10000.0;

  //Convert angle to rad.
  true_rad = pi*true_angle / 180;

  //Count rotations 
  if (true_rad - prev_rad > 4) {
    cnt = cnt-1;
  }
  if (true_rad - prev_rad < -4) {
    cnt = cnt+1;
  }

  //Serial.println(cnt);

  true_dist = - _wheel_diam*(cnt*pi + true_rad/(2));  //negative in front for the wheel rolling direction relative to scorpion
   
  prev_rad = true_rad;
  
  return true_dist;
}