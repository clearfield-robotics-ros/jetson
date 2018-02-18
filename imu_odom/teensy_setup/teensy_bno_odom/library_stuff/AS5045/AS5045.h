/*
  AS5045.h - Library for encoder
  Created by Aaron Chong, December 2017
  minebot, Team B, MRSD 2018

*/
#ifndef AS5045_h
#define AS5045_h

#include "Arduino.h"

class encoder
{
  public:
    encoder(int clock_pin, int CSn_pin, int input_pin, float wheel_diam);
    void setup_rotary_encoder();
    void calibrate_rotary_encoder();
    float rotary_data();
    int input_stream = 0; // one bit read from pin
    long packed_data = 0; // two bytes concatenated from inputstream
    long angle = 0; // holds processed angle value
    float angle_float = 0.0;
    float true_angle = 0.0;
    float calib_angle = 0.0;
    float offset_angle = 0.0; //18 22
    float rotary_angle = 0.0;
    float starting_angle = 0.0;
    long printing_angle = 0;
    long angle_mask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
    float pi = 3.1415;
    float resolution = 0.08789;
    int debug;
    
    float true_rad;
    int cnt = 0;
    float prev_rad;
    float true_dist;

    long angle_temp = 0;
    
  private:
    int _clock_pin;
    int _CSn_pin;
    int _input_pin;
    float _resting_angle;
    float _wheel_diam;
};

#endif