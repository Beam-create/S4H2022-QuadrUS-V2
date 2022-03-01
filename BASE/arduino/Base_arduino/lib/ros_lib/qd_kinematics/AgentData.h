#ifndef _ROS_qd_kinematics_AgentData_h
#define _ROS_qd_kinematics_AgentData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qd_kinematics
{

  class AgentData : public ros::Msg
  {
    public:
      typedef float _action0_type;
      _action0_type action0;
      typedef float _action1_type;
      _action1_type action1;
      typedef float _action2_type;
      _action2_type action2;
      typedef float _action3_type;
      _action3_type action3;
      typedef float _action4_type;
      _action4_type action4;
      typedef float _action5_type;
      _action5_type action5;
      typedef float _action6_type;
      _action6_type action6;
      typedef float _action7_type;
      _action7_type action7;
      typedef float _action8_type;
      _action8_type action8;
      typedef float _action9_type;
      _action9_type action9;
      typedef float _action10_type;
      _action10_type action10;
      typedef float _action11_type;
      _action11_type action11;
      typedef float _action12_type;
      _action12_type action12;
      typedef float _action13_type;
      _action13_type action13;

    AgentData():
      action0(0),
      action1(0),
      action2(0),
      action3(0),
      action4(0),
      action5(0),
      action6(0),
      action7(0),
      action8(0),
      action9(0),
      action10(0),
      action11(0),
      action12(0),
      action13(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_action0;
      u_action0.real = this->action0;
      *(outbuffer + offset + 0) = (u_action0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action0);
      union {
        float real;
        uint32_t base;
      } u_action1;
      u_action1.real = this->action1;
      *(outbuffer + offset + 0) = (u_action1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action1);
      union {
        float real;
        uint32_t base;
      } u_action2;
      u_action2.real = this->action2;
      *(outbuffer + offset + 0) = (u_action2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action2);
      union {
        float real;
        uint32_t base;
      } u_action3;
      u_action3.real = this->action3;
      *(outbuffer + offset + 0) = (u_action3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action3);
      union {
        float real;
        uint32_t base;
      } u_action4;
      u_action4.real = this->action4;
      *(outbuffer + offset + 0) = (u_action4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action4);
      union {
        float real;
        uint32_t base;
      } u_action5;
      u_action5.real = this->action5;
      *(outbuffer + offset + 0) = (u_action5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action5);
      union {
        float real;
        uint32_t base;
      } u_action6;
      u_action6.real = this->action6;
      *(outbuffer + offset + 0) = (u_action6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action6);
      union {
        float real;
        uint32_t base;
      } u_action7;
      u_action7.real = this->action7;
      *(outbuffer + offset + 0) = (u_action7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action7.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action7.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action7.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action7);
      union {
        float real;
        uint32_t base;
      } u_action8;
      u_action8.real = this->action8;
      *(outbuffer + offset + 0) = (u_action8.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action8.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action8.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action8.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action8);
      union {
        float real;
        uint32_t base;
      } u_action9;
      u_action9.real = this->action9;
      *(outbuffer + offset + 0) = (u_action9.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action9.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action9.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action9.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action9);
      union {
        float real;
        uint32_t base;
      } u_action10;
      u_action10.real = this->action10;
      *(outbuffer + offset + 0) = (u_action10.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action10.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action10.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action10.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action10);
      union {
        float real;
        uint32_t base;
      } u_action11;
      u_action11.real = this->action11;
      *(outbuffer + offset + 0) = (u_action11.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action11.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action11.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action11.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action11);
      union {
        float real;
        uint32_t base;
      } u_action12;
      u_action12.real = this->action12;
      *(outbuffer + offset + 0) = (u_action12.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action12.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action12.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action12.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action12);
      union {
        float real;
        uint32_t base;
      } u_action13;
      u_action13.real = this->action13;
      *(outbuffer + offset + 0) = (u_action13.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action13.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action13.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action13.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action13);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_action0;
      u_action0.base = 0;
      u_action0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action0 = u_action0.real;
      offset += sizeof(this->action0);
      union {
        float real;
        uint32_t base;
      } u_action1;
      u_action1.base = 0;
      u_action1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action1 = u_action1.real;
      offset += sizeof(this->action1);
      union {
        float real;
        uint32_t base;
      } u_action2;
      u_action2.base = 0;
      u_action2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action2 = u_action2.real;
      offset += sizeof(this->action2);
      union {
        float real;
        uint32_t base;
      } u_action3;
      u_action3.base = 0;
      u_action3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action3 = u_action3.real;
      offset += sizeof(this->action3);
      union {
        float real;
        uint32_t base;
      } u_action4;
      u_action4.base = 0;
      u_action4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action4 = u_action4.real;
      offset += sizeof(this->action4);
      union {
        float real;
        uint32_t base;
      } u_action5;
      u_action5.base = 0;
      u_action5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action5 = u_action5.real;
      offset += sizeof(this->action5);
      union {
        float real;
        uint32_t base;
      } u_action6;
      u_action6.base = 0;
      u_action6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action6 = u_action6.real;
      offset += sizeof(this->action6);
      union {
        float real;
        uint32_t base;
      } u_action7;
      u_action7.base = 0;
      u_action7.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action7.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action7.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action7.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action7 = u_action7.real;
      offset += sizeof(this->action7);
      union {
        float real;
        uint32_t base;
      } u_action8;
      u_action8.base = 0;
      u_action8.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action8.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action8.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action8.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action8 = u_action8.real;
      offset += sizeof(this->action8);
      union {
        float real;
        uint32_t base;
      } u_action9;
      u_action9.base = 0;
      u_action9.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action9.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action9.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action9.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action9 = u_action9.real;
      offset += sizeof(this->action9);
      union {
        float real;
        uint32_t base;
      } u_action10;
      u_action10.base = 0;
      u_action10.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action10.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action10.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action10.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action10 = u_action10.real;
      offset += sizeof(this->action10);
      union {
        float real;
        uint32_t base;
      } u_action11;
      u_action11.base = 0;
      u_action11.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action11.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action11.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action11.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action11 = u_action11.real;
      offset += sizeof(this->action11);
      union {
        float real;
        uint32_t base;
      } u_action12;
      u_action12.base = 0;
      u_action12.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action12.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action12.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action12.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action12 = u_action12.real;
      offset += sizeof(this->action12);
      union {
        float real;
        uint32_t base;
      } u_action13;
      u_action13.base = 0;
      u_action13.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action13.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action13.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action13.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action13 = u_action13.real;
      offset += sizeof(this->action13);
     return offset;
    }

    virtual const char * getType() override { return "qd_kinematics/AgentData"; };
    virtual const char * getMD5() override { return "eb20a647338517e50d32ae4353795371"; };

  };

}
#endif
