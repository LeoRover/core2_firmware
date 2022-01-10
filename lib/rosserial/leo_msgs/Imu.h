#ifndef _ROS_leo_msgs_Imu_h
#define _ROS_leo_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace leo_msgs
{

  class Imu : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _temperature_type;
      _temperature_type temperature;
      typedef float _gyro_x_type;
      _gyro_x_type gyro_x;
      typedef float _gyro_y_type;
      _gyro_y_type gyro_y;
      typedef float _gyro_z_type;
      _gyro_z_type gyro_z;
      typedef float _accel_x_type;
      _accel_x_type accel_x;
      typedef float _accel_y_type;
      _accel_y_type accel_y;
      typedef float _accel_z_type;
      _accel_z_type accel_z;

    Imu():
      stamp(),
      temperature(0),
      gyro_x(0),
      gyro_y(0),
      gyro_z(0),
      accel_x(0),
      accel_y(0),
      accel_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.real = this->gyro_x;
      *(outbuffer + offset + 0) = (u_gyro_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.real = this->gyro_y;
      *(outbuffer + offset + 0) = (u_gyro_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.real = this->gyro_z;
      *(outbuffer + offset + 0) = (u_gyro_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_z);
      union {
        float real;
        uint32_t base;
      } u_accel_x;
      u_accel_x.real = this->accel_x;
      *(outbuffer + offset + 0) = (u_accel_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_x);
      union {
        float real;
        uint32_t base;
      } u_accel_y;
      u_accel_y.real = this->accel_y;
      *(outbuffer + offset + 0) = (u_accel_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_y);
      union {
        float real;
        uint32_t base;
      } u_accel_z;
      u_accel_z.real = this->accel_z;
      *(outbuffer + offset + 0) = (u_accel_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.base = 0;
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_x = u_gyro_x.real;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.base = 0;
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_y = u_gyro_y.real;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.base = 0;
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_z = u_gyro_z.real;
      offset += sizeof(this->gyro_z);
      union {
        float real;
        uint32_t base;
      } u_accel_x;
      u_accel_x.base = 0;
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_x = u_accel_x.real;
      offset += sizeof(this->accel_x);
      union {
        float real;
        uint32_t base;
      } u_accel_y;
      u_accel_y.base = 0;
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_y = u_accel_y.real;
      offset += sizeof(this->accel_y);
      union {
        float real;
        uint32_t base;
      } u_accel_z;
      u_accel_z.base = 0;
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_z = u_accel_z.real;
      offset += sizeof(this->accel_z);
     return offset;
    }

    virtual const char * getType() override { return "leo_msgs/Imu"; };
    virtual const char * getMD5() override { return "d8596af47cb860f11b5f9246dd06f4fc"; };

  };

}
#endif
