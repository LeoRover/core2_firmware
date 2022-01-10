#ifndef _ROS_leo_msgs_WheelStates_h
#define _ROS_leo_msgs_WheelStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace leo_msgs
{

  class WheelStates : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      float position[4];
      float velocity[4];
      float torque[4];
      float pwm_duty_cycle[4];

    WheelStates():
      stamp(),
      position(),
      velocity(),
      torque(),
      pwm_duty_cycle()
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_velocityi;
      u_velocityi.real = this->velocity[i];
      *(outbuffer + offset + 0) = (u_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocityi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_torquei;
      u_torquei.real = this->torque[i];
      *(outbuffer + offset + 0) = (u_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torquei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_pwm_duty_cyclei;
      u_pwm_duty_cyclei.real = this->pwm_duty_cycle[i];
      *(outbuffer + offset + 0) = (u_pwm_duty_cyclei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pwm_duty_cyclei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pwm_duty_cyclei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pwm_duty_cyclei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pwm_duty_cycle[i]);
      }
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.base = 0;
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position[i] = u_positioni.real;
      offset += sizeof(this->position[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_velocityi;
      u_velocityi.base = 0;
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity[i] = u_velocityi.real;
      offset += sizeof(this->velocity[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_torquei;
      u_torquei.base = 0;
      u_torquei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torquei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torquei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torquei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torque[i] = u_torquei.real;
      offset += sizeof(this->torque[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_pwm_duty_cyclei;
      u_pwm_duty_cyclei.base = 0;
      u_pwm_duty_cyclei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pwm_duty_cyclei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pwm_duty_cyclei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pwm_duty_cyclei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pwm_duty_cycle[i] = u_pwm_duty_cyclei.real;
      offset += sizeof(this->pwm_duty_cycle[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "leo_msgs/WheelStates"; };
    virtual const char * getMD5() override { return "3fd78a5ebfca19b565fd49f45052c8cd"; };

  };

}
#endif
