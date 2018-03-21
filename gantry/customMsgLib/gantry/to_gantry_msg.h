#ifndef _ROS_gantry_to_gantry_msg_h
#define _ROS_gantry_to_gantry_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gantry
{

  class to_gantry_msg : public ros::Msg
  {
    public:
      int16_t state_desired;
      double sweep_speed_desired;
      double x_desired;
      double y_desired;
      double yaw_desired;
      double probe_angle_desired;

    to_gantry_msg():
      state_desired(0),
      sweep_speed_desired(0),
      x_desired(0),
      y_desired(0),
      yaw_desired(0),
      probe_angle_desired(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_state_desired;
      u_state_desired.real = this->state_desired;
      *(outbuffer + offset + 0) = (u_state_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_desired.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state_desired);
      union {
        double real;
        uint64_t base;
      } u_sweep_speed_desired;
      u_sweep_speed_desired.real = this->sweep_speed_desired;
      *(outbuffer + offset + 0) = (u_sweep_speed_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sweep_speed_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sweep_speed_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sweep_speed_desired.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sweep_speed_desired.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sweep_speed_desired.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sweep_speed_desired.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sweep_speed_desired.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sweep_speed_desired);
      union {
        double real;
        uint64_t base;
      } u_x_desired;
      u_x_desired.real = this->x_desired;
      *(outbuffer + offset + 0) = (u_x_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_desired.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x_desired.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x_desired.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x_desired.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x_desired.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x_desired);
      union {
        double real;
        uint64_t base;
      } u_y_desired;
      u_y_desired.real = this->y_desired;
      *(outbuffer + offset + 0) = (u_y_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_desired.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y_desired.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y_desired.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y_desired.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y_desired.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y_desired);
      union {
        double real;
        uint64_t base;
      } u_yaw_desired;
      u_yaw_desired.real = this->yaw_desired;
      *(outbuffer + offset + 0) = (u_yaw_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_desired.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yaw_desired.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yaw_desired.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yaw_desired.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yaw_desired.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yaw_desired);
      union {
        double real;
        uint64_t base;
      } u_probe_angle_desired;
      u_probe_angle_desired.real = this->probe_angle_desired;
      *(outbuffer + offset + 0) = (u_probe_angle_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probe_angle_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probe_angle_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probe_angle_desired.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_probe_angle_desired.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_probe_angle_desired.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_probe_angle_desired.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_probe_angle_desired.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->probe_angle_desired);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_state_desired;
      u_state_desired.base = 0;
      u_state_desired.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_desired.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state_desired = u_state_desired.real;
      offset += sizeof(this->state_desired);
      union {
        double real;
        uint64_t base;
      } u_sweep_speed_desired;
      u_sweep_speed_desired.base = 0;
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sweep_speed_desired.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sweep_speed_desired = u_sweep_speed_desired.real;
      offset += sizeof(this->sweep_speed_desired);
      union {
        double real;
        uint64_t base;
      } u_x_desired;
      u_x_desired.base = 0;
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x_desired.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x_desired = u_x_desired.real;
      offset += sizeof(this->x_desired);
      union {
        double real;
        uint64_t base;
      } u_y_desired;
      u_y_desired.base = 0;
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y_desired.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y_desired = u_y_desired.real;
      offset += sizeof(this->y_desired);
      union {
        double real;
        uint64_t base;
      } u_yaw_desired;
      u_yaw_desired.base = 0;
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yaw_desired.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yaw_desired = u_yaw_desired.real;
      offset += sizeof(this->yaw_desired);
      union {
        double real;
        uint64_t base;
      } u_probe_angle_desired;
      u_probe_angle_desired.base = 0;
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_probe_angle_desired.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->probe_angle_desired = u_probe_angle_desired.real;
      offset += sizeof(this->probe_angle_desired);
     return offset;
    }

    const char * getType(){ return "gantry/to_gantry_msg"; };
    const char * getMD5(){ return "cec2849a2070b496fe4ca5ee428be1c7"; };

  };

}
#endif