#ifndef _ROS_ms5837_ms5837_data_h
#define _ROS_ms5837_ms5837_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ms5837
{

  class ms5837_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _tempC_type;
      _tempC_type tempC;
      typedef double _tempF_type;
      _tempF_type tempF;
      typedef double _depth_type;
      _depth_type depth;
      typedef double _altitudeM_type;
      _altitudeM_type altitudeM;

    ms5837_data():
      header(),
      tempC(0),
      tempF(0),
      depth(0),
      altitudeM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_tempC;
      u_tempC.real = this->tempC;
      *(outbuffer + offset + 0) = (u_tempC.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tempC.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tempC.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tempC.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tempC.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tempC.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tempC.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tempC.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tempC);
      union {
        double real;
        uint64_t base;
      } u_tempF;
      u_tempF.real = this->tempF;
      *(outbuffer + offset + 0) = (u_tempF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tempF.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tempF.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tempF.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tempF.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tempF.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tempF.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tempF.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tempF);
      union {
        double real;
        uint64_t base;
      } u_depth;
      u_depth.real = this->depth;
      *(outbuffer + offset + 0) = (u_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_depth.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_depth.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_depth.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_depth.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->depth);
      union {
        double real;
        uint64_t base;
      } u_altitudeM;
      u_altitudeM.real = this->altitudeM;
      *(outbuffer + offset + 0) = (u_altitudeM.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitudeM.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitudeM.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitudeM.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_altitudeM.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_altitudeM.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_altitudeM.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_altitudeM.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->altitudeM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_tempC;
      u_tempC.base = 0;
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tempC.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tempC = u_tempC.real;
      offset += sizeof(this->tempC);
      union {
        double real;
        uint64_t base;
      } u_tempF;
      u_tempF.base = 0;
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tempF.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tempF = u_tempF.real;
      offset += sizeof(this->tempF);
      union {
        double real;
        uint64_t base;
      } u_depth;
      u_depth.base = 0;
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_depth.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->depth = u_depth.real;
      offset += sizeof(this->depth);
      union {
        double real;
        uint64_t base;
      } u_altitudeM;
      u_altitudeM.base = 0;
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_altitudeM.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->altitudeM = u_altitudeM.real;
      offset += sizeof(this->altitudeM);
     return offset;
    }

    const char * getType(){ return "ms5837/ms5837_data"; };
    const char * getMD5(){ return "eca2bdcabad4ac8096363838d8496716"; };

  };

}
#endif