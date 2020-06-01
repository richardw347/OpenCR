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

#include "../../include/turtlebot3/turtlebot3_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882;


const float WHEEL_DIAMETER = 0.058;
const float WHEEL_RADIUS = WHEEL_DIAMETER/2.0;      // meter
const float DISTANCE_CENTER_TO_WHEEL = 0.106;     // meter
const float RPM_CONSTANT_VALUE = 0.229;
const uint8_t OMNIWHEEL_NUM = 3;


/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_M1 = 1;
const uint8_t DXL_MOTOR_ID_M2 = 2;
const uint8_t DXL_MOTOR_ID_M3 = 3;
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);



Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: m1_wheel_id_(DXL_MOTOR_ID_M1),
  m2_wheel_id_(DXL_MOTOR_ID_M2),
  m3_wheel_id_(DXL_MOTOR_ID_M3),
  torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_write_param.id_count = 3;
  sync_write_param.xel[M1].id = m1_wheel_id_;
  sync_write_param.xel[M2].id = m2_wheel_id_;
  sync_write_param.xel[M3].id = m3_wheel_id_;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 3;
  sync_read_param.xel[M1].id = m1_wheel_id_;
  sync_read_param.xel[M2].id = m2_wheel_id_;
  sync_read_param.xel[M3].id = m3_wheel_id_;

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

bool Turtlebot3MotorDriver::is_connected()
{
  return (dxl.ping(m1_wheel_id_) == true && dxl.ping(m2_wheel_id_) == true && dxl.ping(m3_wheel_id_) == true);
}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[M1].data[0] = onoff;
  sync_write_param.xel[M2].data[0] = onoff;
  sync_write_param.xel[M3].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool Turtlebot3MotorDriver::get_torque()
{
  if(dxl.readControlTableItem(TORQUE_ENABLE, m1_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, m2_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, m3_wheel_id_) == true){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool Turtlebot3MotorDriver::read_present_position(int32_t &m1_value, int32_t &m2_value, int32_t &m3_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&m1_value, read_result.xel[M1].data, read_result.xel[M1].length);
    memcpy(&m2_value, read_result.xel[M2].data, read_result.xel[M2].length);
    memcpy(&m3_value, read_result.xel[M3].data, read_result.xel[M3].length);
    ret = true;
  }

    m2_value = -m2_value;
    m3_value = -m3_value;

  return ret;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t &m1_value, int32_t &m2_value, int32_t &m3_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&m1_value, read_result.xel[M1].data, read_result.xel[M1].length);
    memcpy(&m2_value, read_result.xel[M2].data, read_result.xel[M2].length);
    memcpy(&m3_value, read_result.xel[M3].data, read_result.xel[M3].length);
    ret = true;
  }

    m2_value = -m2_value;
    m3_value = -m3_value;

  return ret;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t &m1_value, int16_t &m2_value, int16_t &m3_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&m1_value, read_result.xel[M1].data, read_result.xel[M1].length);
    memcpy(&m2_value, read_result.xel[M2].data, read_result.xel[M2].length);
    memcpy(&m3_value, read_result.xel[M3].data, read_result.xel[M3].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t &m1_value, uint32_t &m2_value, uint32_t &m3_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&m1_value, read_result.xel[M1].data, read_result.xel[M1].length);
    memcpy(&m2_value, read_result.xel[M2].data, read_result.xel[M2].length);
    memcpy(&m3_value, read_result.xel[M3].data, read_result.xel[M3].length);
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::write_velocity(int32_t m1_value, int32_t m2_value, int32_t m3_value)
{
  bool ret = false;

  m2_value = -m2_value;
  m3_value = -m3_value;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[M1].data, &m1_value, sync_write_param.length);
  memcpy(sync_write_param.xel[M2].data, &m2_value, sync_write_param.length);
  memcpy(sync_write_param.xel[M3].data, &m3_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t m1_value, uint32_t m2_value, uint32_t m3_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[M1].data, &m1_value, sync_write_param.length);
  memcpy(sync_write_param.xel[M2].data, &m2_value, sync_write_param.length);
  memcpy(sync_write_param.xel[M3].data, &m3_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::control_motors(float goal_linear_x_velocity, float goal_linear_y_velocity, float goal_angular_velocity)
{
  bool dxl_comm_result = false;


  int32_t wheel_value[OMNIWHEEL_NUM] = {0, 0, 0};
  float  wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};

  wheel_angular_velocity[M1] = (goal_linear_x_velocity * (sqrt(3) / (2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[M2] = (goal_linear_x_velocity * (sqrt(3) / (-2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[M3] = (goal_linear_x_velocity * 0) + (goal_linear_y_velocity * (1 / WHEEL_RADIUS)) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));


  wheel_value[M1] = wheel_angular_velocity[M1] * 9.5493 / RPM_CONSTANT_VALUE;
  wheel_value[M2] = wheel_angular_velocity[M2] * 9.5493 / RPM_CONSTANT_VALUE;
  wheel_value[M3] = wheel_angular_velocity[M3] * 9.5493 / RPM_CONSTANT_VALUE;


//  for (int id = 0; id < OMNIWHEEL_NUM; id++)
//  {
//    wheel_value[id] = wheel_angular_velocity[id] * 9.54 /  RPM_CONSTANT_VALUE;
//
//    if (wheel_value[id] > LIMIT_X_MAX_VELOCITY)       wheel_value[id] =  LIMIT_X_MAX_VELOCITY;
//    else if (wheel_value[id] < -LIMIT_X_MAX_VELOCITY) wheel_value[id] = -LIMIT_X_MAX_VELOCITY;
//  }
//
  SerialBT2.print("Vx:\t");  SerialBT2.print(goal_linear_x_velocity);
  SerialBT2.print(" Vy:\t"); SerialBT2.print(goal_linear_y_velocity);
  SerialBT2.print(" W:\t");  SerialBT2.println(goal_angular_velocity);
  SerialBT2.println();

  SerialBT2.print("M1:\t");  SerialBT2.print(wheel_angular_velocity[M1]);
  SerialBT2.print(" M2:\t"); SerialBT2.print(wheel_angular_velocity[M2]);
  SerialBT2.print(" M3:\t");  SerialBT2.println(wheel_angular_velocity[M3]);
  SerialBT2.println();

  SerialBT2.print("M1:\t");  SerialBT2.print(wheel_value[M1]);
  SerialBT2.print(" M2:\t"); SerialBT2.print(wheel_value[M2]);
  SerialBT2.print(" M3:\t");  SerialBT2.println(wheel_value[M3]);

  dxl_comm_result = write_velocity((int32_t)wheel_value[M1], (int32_t)wheel_value[M2], (int32_t)wheel_value[M3]);

  if (!dxl_comm_result)
    return false;

  return true;
}
