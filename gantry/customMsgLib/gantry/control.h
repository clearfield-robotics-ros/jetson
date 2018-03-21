#ifndef _ROS_gantry_control_h
#define _ROS_gantry_control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gantry
{

  class control : public ros::Msg
  {
    public:
      double x;
      double y;
      double yaw;
      double probe_angle;

    control():
      x(0),
      y(0),
      yaw(0),
      probe_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
     return offset;
    }

    const char * getType(){ return "gantry/control"; };
    const char * getMD5(){ return "85814b7a94f54eb7d71ba2d505258289"; };

  };

}
#endif