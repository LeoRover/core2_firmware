#ifndef _ROS_leo_msgs_WheelOdom_h
#define _ROS_leo_msgs_WheelOdom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace leo_msgs
{

  class WheelOdom : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _velocity_lin_type;
      _velocity_lin_type velocity_lin;
      typedef float _velocity_ang_type;
      _velocity_ang_type velocity_ang;
      typedef float _pose_x_type;
      _pose_x_type pose_x;
      typedef float _pose_y_type;
      _pose_y_type pose_y;
      typedef float _pose_yaw_type;
      _pose_yaw_type pose_yaw;

    WheelOdom():
      stamp(),
      velocity_lin(0),
      velocity_ang(0),
      pose_x(0),
      pose_y(0),
      pose_yaw(0)
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
      } u_velocity_lin;
      u_velocity_lin.real = this->velocity_lin;
      *(outbuffer + offset + 0) = (u_velocity_lin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_lin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_lin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_lin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_lin);
      union {
        float real;
        uint32_t base;
      } u_velocity_ang;
      u_velocity_ang.real = this->velocity_ang;
      *(outbuffer + offset + 0) = (u_velocity_ang.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_ang.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_ang.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_ang.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_ang);
      union {
        float real;
        uint32_t base;
      } u_pose_x;
      u_pose_x.real = this->pose_x;
      *(outbuffer + offset + 0) = (u_pose_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_x);
      union {
        float real;
        uint32_t base;
      } u_pose_y;
      u_pose_y.real = this->pose_y;
      *(outbuffer + offset + 0) = (u_pose_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_y);
      union {
        float real;
        uint32_t base;
      } u_pose_yaw;
      u_pose_yaw.real = this->pose_yaw;
      *(outbuffer + offset + 0) = (u_pose_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_yaw);
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
      } u_velocity_lin;
      u_velocity_lin.base = 0;
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_lin = u_velocity_lin.real;
      offset += sizeof(this->velocity_lin);
      union {
        float real;
        uint32_t base;
      } u_velocity_ang;
      u_velocity_ang.base = 0;
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_ang = u_velocity_ang.real;
      offset += sizeof(this->velocity_ang);
      union {
        float real;
        uint32_t base;
      } u_pose_x;
      u_pose_x.base = 0;
      u_pose_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pose_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pose_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pose_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pose_x = u_pose_x.real;
      offset += sizeof(this->pose_x);
      union {
        float real;
        uint32_t base;
      } u_pose_y;
      u_pose_y.base = 0;
      u_pose_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pose_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pose_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pose_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pose_y = u_pose_y.real;
      offset += sizeof(this->pose_y);
      union {
        float real;
        uint32_t base;
      } u_pose_yaw;
      u_pose_yaw.base = 0;
      u_pose_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pose_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pose_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pose_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pose_yaw = u_pose_yaw.real;
      offset += sizeof(this->pose_yaw);
     return offset;
    }

    virtual const char * getType() override { return "leo_msgs/WheelOdom"; };
    virtual const char * getMD5() override { return "5bb892afaf24a6d3bedf13c8fe986f2a"; };

  };

}
#endif
