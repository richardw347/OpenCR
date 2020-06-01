/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include <Dynamixel2Arduino.h>


enum MortorLocation{
  M1 = 0,
  M2,
  M3,
  MOTOR_NUM_MAX
};

enum VelocityType{
  LINEAR_X = 0,
  LINEAR_Y,
  ANGULAR,
  TYPE_NUM_MAX
};


class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();

  bool set_torque(bool onoff);
  bool get_torque();
  
  bool read_present_position(int32_t &m1_value, int32_t &m2_value, int32_t &m3_value);
  bool read_present_velocity(int32_t &m1_value, int32_t &m2_value, int32_t &m3_value);
  bool read_present_current(int16_t &m1_value, int16_t &m2_value, int16_t &m3_value);
  bool read_profile_acceleration(uint32_t &m1_value, uint32_t &m2_value, uint32_t &m3_value);
  
  bool write_velocity(int32_t m1_value, int32_t m2_value, int32_t m3_value);
  bool write_profile_acceleration(uint32_t m1_value, uint32_t m2_value, uint32_t m3_value);

  bool control_motors(float goal_linear_x_velocity, float goal_linear_y_velocity, float goal_angular_velocity);

  void debug_print();

 private:
  uint8_t m1_wheel_id_;
  uint8_t m2_wheel_id_;
  uint8_t m3_wheel_id_;
  bool torque_;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
