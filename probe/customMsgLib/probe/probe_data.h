#ifndef _ROS_probe_probe_data_h
#define _ROS_probe_probe_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace probe
{

  class probe_data : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef bool _init_type;
      _init_type init;
      typedef bool _probe_complete_type;
      _probe_complete_type probe_complete;
      typedef float _linear_position_type;
      _linear_position_type linear_position;
      typedef bool _contact_made_type;
      _contact_made_type contact_made;

    probe_data():
      state(0),
      init(0),
      probe_complete(0),
      linear_position(0),
      contact_made(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_init;
      u_init.real = this->init;
      *(outbuffer + offset + 0) = (u_init.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->init);
      union {
        bool real;
        uint8_t base;
      } u_probe_complete;
      u_probe_complete.real = this->probe_complete;
      *(outbuffer + offset + 0) = (u_probe_complete.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->probe_complete);
      union {
        float real;
        uint32_t base;
      } u_linear_position;
      u_linear_position.real = this->linear_position;
      *(outbuffer + offset + 0) = (u_linear_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_position);
      union {
        bool real;
        uint8_t base;
      } u_contact_made;
      u_contact_made.real = this->contact_made;
      *(outbuffer + offset + 0) = (u_contact_made.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contact_made);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_init;
      u_init.base = 0;
      u_init.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->init = u_init.real;
      offset += sizeof(this->init);
      union {
        bool real;
        uint8_t base;
      } u_probe_complete;
      u_probe_complete.base = 0;
      u_probe_complete.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->probe_complete = u_probe_complete.real;
      offset += sizeof(this->probe_complete);
      union {
        float real;
        uint32_t base;
      } u_linear_position;
      u_linear_position.base = 0;
      u_linear_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_position = u_linear_position.real;
      offset += sizeof(this->linear_position);
      union {
        bool real;
        uint8_t base;
      } u_contact_made;
      u_contact_made.base = 0;
      u_contact_made.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->contact_made = u_contact_made.real;
      offset += sizeof(this->contact_made);
     return offset;
    }

    const char * getType(){ return "probe/probe_data"; };
    const char * getMD5(){ return "60ad033b20a6122bc29169d93bd826d6"; };

  };

}
#endif