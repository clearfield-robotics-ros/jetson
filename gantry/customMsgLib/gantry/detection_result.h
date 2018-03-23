#ifndef _ROS_gantry_detection_result_h
#define _ROS_gantry_detection_result_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gantry
{

  class detection_result : public ros::Msg
  {
    public:
      uint8_t id;
      bool truth;
      float radius_truth;
      float x_truth;
      float y_truth;
      bool estimate;
      float radius_estimate;
      float x_estimate;
      float y_estimate;
      float estimate_euclidean_error;
      float warning_delay;
      float probe_time;

    detection_result():
      id(0),
      truth(0),
      radius_truth(0),
      x_truth(0),
      y_truth(0),
      estimate(0),
      radius_estimate(0),
      x_estimate(0),
      y_estimate(0),
      estimate_euclidean_error(0),
      warning_delay(0),
      probe_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_truth;
      u_truth.real = this->truth;
      *(outbuffer + offset + 0) = (u_truth.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->truth);
      union {
        float real;
        uint32_t base;
      } u_radius_truth;
      u_radius_truth.real = this->radius_truth;
      *(outbuffer + offset + 0) = (u_radius_truth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius_truth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius_truth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius_truth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->radius_truth);
      union {
        float real;
        uint32_t base;
      } u_x_truth;
      u_x_truth.real = this->x_truth;
      *(outbuffer + offset + 0) = (u_x_truth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_truth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_truth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_truth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_truth);
      union {
        float real;
        uint32_t base;
      } u_y_truth;
      u_y_truth.real = this->y_truth;
      *(outbuffer + offset + 0) = (u_y_truth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_truth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_truth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_truth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_truth);
      union {
        bool real;
        uint8_t base;
      } u_estimate;
      u_estimate.real = this->estimate;
      *(outbuffer + offset + 0) = (u_estimate.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->estimate);
      union {
        float real;
        uint32_t base;
      } u_radius_estimate;
      u_radius_estimate.real = this->radius_estimate;
      *(outbuffer + offset + 0) = (u_radius_estimate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius_estimate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius_estimate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius_estimate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->radius_estimate);
      union {
        float real;
        uint32_t base;
      } u_x_estimate;
      u_x_estimate.real = this->x_estimate;
      *(outbuffer + offset + 0) = (u_x_estimate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_estimate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_estimate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_estimate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_estimate);
      union {
        float real;
        uint32_t base;
      } u_y_estimate;
      u_y_estimate.real = this->y_estimate;
      *(outbuffer + offset + 0) = (u_y_estimate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_estimate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_estimate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_estimate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_estimate);
      union {
        float real;
        uint32_t base;
      } u_estimate_euclidean_error;
      u_estimate_euclidean_error.real = this->estimate_euclidean_error;
      *(outbuffer + offset + 0) = (u_estimate_euclidean_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_estimate_euclidean_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_estimate_euclidean_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_estimate_euclidean_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->estimate_euclidean_error);
      union {
        float real;
        uint32_t base;
      } u_warning_delay;
      u_warning_delay.real = this->warning_delay;
      *(outbuffer + offset + 0) = (u_warning_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_warning_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_warning_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_warning_delay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->warning_delay);
      union {
        float real;
        uint32_t base;
      } u_probe_time;
      u_probe_time.real = this->probe_time;
      *(outbuffer + offset + 0) = (u_probe_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_probe_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_probe_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_probe_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->probe_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_truth;
      u_truth.base = 0;
      u_truth.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->truth = u_truth.real;
      offset += sizeof(this->truth);
      union {
        float real;
        uint32_t base;
      } u_radius_truth;
      u_radius_truth.base = 0;
      u_radius_truth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius_truth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius_truth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius_truth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->radius_truth = u_radius_truth.real;
      offset += sizeof(this->radius_truth);
      union {
        float real;
        uint32_t base;
      } u_x_truth;
      u_x_truth.base = 0;
      u_x_truth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_truth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_truth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_truth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_truth = u_x_truth.real;
      offset += sizeof(this->x_truth);
      union {
        float real;
        uint32_t base;
      } u_y_truth;
      u_y_truth.base = 0;
      u_y_truth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_truth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_truth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_truth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_truth = u_y_truth.real;
      offset += sizeof(this->y_truth);
      union {
        bool real;
        uint8_t base;
      } u_estimate;
      u_estimate.base = 0;
      u_estimate.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->estimate = u_estimate.real;
      offset += sizeof(this->estimate);
      union {
        float real;
        uint32_t base;
      } u_radius_estimate;
      u_radius_estimate.base = 0;
      u_radius_estimate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius_estimate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius_estimate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius_estimate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->radius_estimate = u_radius_estimate.real;
      offset += sizeof(this->radius_estimate);
      union {
        float real;
        uint32_t base;
      } u_x_estimate;
      u_x_estimate.base = 0;
      u_x_estimate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_estimate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_estimate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_estimate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_estimate = u_x_estimate.real;
      offset += sizeof(this->x_estimate);
      union {
        float real;
        uint32_t base;
      } u_y_estimate;
      u_y_estimate.base = 0;
      u_y_estimate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_estimate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_estimate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_estimate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_estimate = u_y_estimate.real;
      offset += sizeof(this->y_estimate);
      union {
        float real;
        uint32_t base;
      } u_estimate_euclidean_error;
      u_estimate_euclidean_error.base = 0;
      u_estimate_euclidean_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_estimate_euclidean_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_estimate_euclidean_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_estimate_euclidean_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->estimate_euclidean_error = u_estimate_euclidean_error.real;
      offset += sizeof(this->estimate_euclidean_error);
      union {
        float real;
        uint32_t base;
      } u_warning_delay;
      u_warning_delay.base = 0;
      u_warning_delay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_warning_delay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_warning_delay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_warning_delay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->warning_delay = u_warning_delay.real;
      offset += sizeof(this->warning_delay);
      union {
        float real;
        uint32_t base;
      } u_probe_time;
      u_probe_time.base = 0;
      u_probe_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_probe_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_probe_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_probe_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->probe_time = u_probe_time.real;
      offset += sizeof(this->probe_time);
     return offset;
    }

    const char * getType(){ return "gantry/detection_result"; };
    const char * getMD5(){ return "0c7cd2fcc1e719ad16dc16d2df5a777a"; };

  };

}
#endif