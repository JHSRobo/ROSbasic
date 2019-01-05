#ifndef _ROS_neopixels_flash_h
#define _ROS_neopixels_flash_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neopixels
{

  class flash : public ros::Msg
  {
    public:
      typedef int8_t _red_type;
      _red_type red;
      typedef int8_t _green_type;
      _green_type green;
      typedef int8_t _blue_type;
      _blue_type blue;
      typedef int32_t _count_type;
      _count_type count;
      typedef int32_t _wait_type;
      _wait_type wait;

    flash():
      red(0),
      green(0),
      blue(0),
      count(0),
      wait(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_red;
      u_red.real = this->red;
      *(outbuffer + offset + 0) = (u_red.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->red);
      union {
        int8_t real;
        uint8_t base;
      } u_green;
      u_green.real = this->green;
      *(outbuffer + offset + 0) = (u_green.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->green);
      union {
        int8_t real;
        uint8_t base;
      } u_blue;
      u_blue.real = this->blue;
      *(outbuffer + offset + 0) = (u_blue.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blue);
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      union {
        int32_t real;
        uint32_t base;
      } u_wait;
      u_wait.real = this->wait;
      *(outbuffer + offset + 0) = (u_wait.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wait.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wait.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wait.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wait);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_red;
      u_red.base = 0;
      u_red.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->red = u_red.real;
      offset += sizeof(this->red);
      union {
        int8_t real;
        uint8_t base;
      } u_green;
      u_green.base = 0;
      u_green.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->green = u_green.real;
      offset += sizeof(this->green);
      union {
        int8_t real;
        uint8_t base;
      } u_blue;
      u_blue.base = 0;
      u_blue.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blue = u_blue.real;
      offset += sizeof(this->blue);
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->count = u_count.real;
      offset += sizeof(this->count);
      union {
        int32_t real;
        uint32_t base;
      } u_wait;
      u_wait.base = 0;
      u_wait.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wait.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wait.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wait.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wait = u_wait.real;
      offset += sizeof(this->wait);
     return offset;
    }

    const char * getType(){ return "neopixels/flash"; };
    const char * getMD5(){ return "f2e04b9fd10d59ba43f9061004df66b3"; };

  };

}
#endif