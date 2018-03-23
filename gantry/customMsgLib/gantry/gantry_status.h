#ifndef _ROS_gantry_gantry_status_h
#define _ROS_gantry_gantry_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gantry
{

  class gantry_status : public ros::Msg
  {
    public:
      int16_t state;
      double sweep_speed;
      double x;
      double y;
      double yaw;
      double probe_angle;
      bool position_reached;

    gantry_status():
      state(0),
      sweep_speed(0),
      x(0),
      y(0),
      yaw(0),
      probe_angle(0),
      position_reached(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      union {
        double real;
        uint64_t base;
      } u_sweep_speed;
      u_sweep_speed.real = this->sweep_speed;
      *(outbuffer + offset + 0) = (u_sweep_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sweep_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sweep_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sweep_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sweep_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sweep_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sweep_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sweep_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sweep_speed);
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yaw.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yaw.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yaw.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yaw.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        double real;
        uint64_t base;
      } u_probe_angle;
      u_probe_angle.real = this->probe_angle;
      *(outbuffer + offset + 0) = (u_probe_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probe_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probe_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probe_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_probe_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_probe_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_probe_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_probe_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->probe_angle);
      union {
        bool real;
        uint8_t base;
      } u_position_reached;
      u_position_reached.real = this->position_reached;
      *(outbuffer + offset + 0) = (u_position_reached.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_reached);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        double real;
        uint64_t base;
      } u_sweep_speed;
      u_sweep_speed.base = 0;
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sweep_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sweep_speed = u_sweep_speed.real;
      offset += sizeof(this->sweep_speed);
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        double real;
        uint64_t base;
      } u_probe_angle;
      u_probe_angle.base = 0;
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_probe_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->probe_angle = u_probe_angle.real;
      offset += sizeof(this->probe_angle);
      union {
        bool real;
        uint8_t base;
      } u_position_reached;
      u_position_reached.base = 0;
      u_position_reached.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->position_reached = u_position_reached.real;
      offset += sizeof(this->position_reached);
     return offset;
    }

    const char * getType(){ return "gantry/gantry_status"; };
    const char * getMD5(){ return "d6368bb8ddbfcc8136f48c56c0a7b113"; };

  };

}
#endif