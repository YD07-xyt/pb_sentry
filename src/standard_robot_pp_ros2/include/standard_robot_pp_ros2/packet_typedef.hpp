// Copyright 2025 SMBU-PolarBear-Robotics-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2 {
const uint8_t SOF_RECEIVE = {'M'};
const uint8_t SOF_SEND = {'M'};

// Receive
// debug
const uint8_t ID_DEBUG = 0x01;
// IMU  ID 不需要
const uint8_t ID_IMU = 0x02;
// 机器人信息数据包 id 不需要
const uint8_t ID_ROBOT_STATE_INFO = 0x03;

// !! 事件数据包id 需要
const uint8_t ID_EVENT_DATA = 0x04;
//  PID调参数据包id
const uint8_t ID_PID_DEBUG = 0x05;
// !! 全场机器人hp信息数据包id 需要
const uint8_t ID_ALL_ROBOT_HP = 0x06;
// !! 比赛信息数据包id 需要
const uint8_t ID_GAME_STATUS = 0x07;
// 机器人运动数据包id 不需要
const uint8_t ID_ROBOT_MOTION = 0x08;
// !! 地面机器人位置数据包id  需要
const uint8_t ID_GROUND_ROBOT_POSITION = 0x09;
// !!!! RFID 状态数据包id 需要
const uint8_t ID_RFID_STATUS = 0x0A;
// !!!! 机器人状态数据包id 需要
const uint8_t ID_ROBOT_STATUS = 0x0B;
//  云台状态数据包id 不需要
const uint8_t ID_JOINT_STATE = 0x0C;
// !!!! 机器人增益和底盘能量数据包id 需要
const uint8_t ID_BUFF = 0x0D;
// Send
const uint8_t ID_ROBOT_CMD = 0x01;

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame {
  uint8_t sof; // 数据帧起始字节，固定值为 M
  uint8_t len; // 数据段长度
  uint8_t id;  // 数据段id
  uint8_t crc; // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct ReceiveDebugData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t checksum;
} __attribute__((packed));

// IMU 数据包 不需要
struct ReceiveImuData {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    float yaw;   // rad
    float pitch; // rad
    float roll;  // rad

    float yaw_vel;   // rad/s
    float pitch_vel; // rad/s
    float roll_vel;  // rad/s

    // float x_accel;  // m/s^2
    // float y_accel;  // m/s^2
    // float z_accel;  // m/s^2
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人信息数据包  不需要
struct ReceiveRobotInfoData {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    /// @brief 机器人部位类型 2 bytes
    struct {
      uint16_t chassis : 2;
      uint16_t gimbal : 3;
      uint16_t shoot : 3;
      uint16_t arm : 3;
      uint16_t custom_controller : 3;
      uint16_t reserve : 1;
    } __attribute__((packed)) type;

    /// @brief 机器人部位状态 1 byte
    /// @note 0: 错误，1: 正常
    struct {
      uint8_t chassis : 1;
      uint8_t gimbal : 1;
      uint8_t shoot : 1;
      uint8_t arm : 1;
      uint8_t custom_controller : 1;
      uint8_t reserve : 3;
    } __attribute__((packed)) state;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// !! 事件数据包 需要
struct ReceiveEventData {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    // 己方与资源区区不重叠的补给区占领状态
    uint8_t non_overlapping_supply_zone : 1;
    // 己方与资源区重叠的补给区占领状态
    uint8_t overlapping_supply_zone : 1;
    // 补给区的占领状态
    uint8_t supply_zone : 1;
    // 小能量机关的激活状态
    uint8_t small_energy : 1;
    // 己方大能量机关的激活状态
    uint8_t big_energy : 1;
    // 己方中央高地的占领状态
    uint8_t central_highland : 2;

    uint8_t reserved1 : 1;
    // 己方梯形高地的占领状态
    uint8_t trapezoidal_highland : 2;
    // 中心增益点的占领状态
    uint8_t center_gain_zone : 2;
    // 保留
    uint8_t reserved2 : 4;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// PID调参数据包 不需要
struct ReceivePidDebugData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct {
    float fdb;
    float ref;
    float pid_out;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// !!! 全场机器人hp信息数据包 需要
struct ReceiveAllRobotHpData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct {
    // 红 1  红 2 红 3 红 4 红 7(哨兵)
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_7_robot_hp;
    // 前哨站血量
    uint16_t red_outpost_hp;
    // 基地血量
    uint16_t red_base_hp;
    // 蓝 1  蓝 2 蓝 3 蓝 4  蓝 7(哨兵)
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_7_robot_hp;
    // 前哨站血量
    uint16_t blue_outpost_hp;
    // 基地血量
    uint16_t blue_base_hp;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// !!! 比赛信息数据包 需要
struct ReceiveGameStatusData {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    // 比赛进度 当前比赛阶段 0-5
    uint8_t game_progress;
    // 当前比赛阶段剩余时间
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人运动数据包 不需要
struct ReceiveRobotMotionData {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    struct {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// !! 地面机器人位置数据包 需要
struct ReceiveGroundRobotPosition {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct {
    // 英雄
    float hero_x;
    float hero_y;
    // 工程
    float engineer_x;
    float engineer_y;
    // 3号
    float standard_3_x;
    float standard_3_y;
    // 4号
    float standard_4_x;
    float standard_4_y;
    // 保留
    float reserved1;
    float reserved2;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// !! RFID 状态数据包 需要
struct ReceiveRfidStatus {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    // 己方基地增益点
    uint32_t base_gain_point : 1;
    // 己方中央高地增益点
    uint32_t central_highland_gain_point : 1;
    // 对方中央高地增益点
    uint32_t enemy_central_highland_gain_point : 1;
    // 己方梯形高地增益点
    uint32_t friendly_trapezoidal_highland_gain_point : 1;
    // 对方梯形高地增益点
    uint32_t enemy_trapezoidal_highland_gain_point : 1;
    // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    uint32_t friendly_fly_ramp_front_gain_point : 1;
    // 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    uint32_t friendly_fly_ramp_back_gain_point : 1;
    // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    uint32_t enemy_fly_ramp_front_gain_point : 1;
    // 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    uint32_t enemy_fly_ramp_back_gain_point : 1;
    // 己方地形跨越增益点（中央高地下方）
    uint32_t friendly_central_highland_lower_gain_point : 1;
    // 己方地形跨越增益点（中央高地上方）
    uint32_t friendly_central_highland_upper_gain_point : 1;
    // 对方地形跨越增益点（中央高地下方）
    uint32_t enemy_central_highland_lower_gain_point : 1;
    // 对方地形跨越增益点（中央高地上方）
    uint32_t enemy_central_highland_upper_gain_point : 1;
    // 己方地形跨越增益点（公路下方）
    uint32_t friendly_highway_lower_gain_point : 1;
    // 己方地形跨越增益点（公路上方）
    uint32_t friendly_highway_upper_gain_point : 1;
    // 对方地形跨越增益点（公路下方）
    uint32_t enemy_highway_lower_gain_point : 1;
    // 对方地形跨越增益点（公路上方）
    uint32_t enemy_highway_upper_gain_point : 1;
    // 己方堡垒增益点
    uint32_t friendly_fortress_gain_point : 1;
    // 己方前哨站增益点
    uint32_t friendly_outpost_gain_point : 1;
    // 己方与资源区不重叠的补给区/RMUL 补给区
    uint32_t friendly_supply_zone_non_exchange : 1;
    // 己方与资源区重叠的补给区
    uint32_t friendly_supply_zone_exchange : 1;
    // 己方装配增益点
    uint32_t friendly_big_resource_island : 1;
    // 对方装配增益点
    uint32_t enemy_big_resource_island : 1;
    // 中心增益点（仅  RMUL  适用）
    uint32_t center_gain_point : 1;
    //  剩下的8个 保留
    uint32_t reserved : 8;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// !!  机器人状态数据包 需要
struct ReceiveRobotStatus {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct {
    // 本机器人 ID
    uint8_t robot_id;
    // 机器人等级
    uint8_t robot_level;
    // 机器人当前血量
    uint16_t current_up;
    // 机器人血量上限
    uint16_t maximum_hp;
    // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_cooling_value;
    // 机器人射击热量上限
    uint16_t shooter_barrel_heat_limit;
    // 机器人底盘功率上限
    uint16_t shooter_17mm_1_barrel_heat;
    // 本机器人位置 x 坐标
    float robot_pos_x;
    // 本机器人位置 y 坐标
    float robot_pos_y;
    // 本机器人测速模块的朝向
    float robot_pos_angle;
    /* 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
    数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0*/
    uint8_t armor_id : 4;
    // 血量变化类型
    uint8_t hp_deduction_reason : 4;
    // 机器人自身拥有的 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_17mm;
    // 剩余金币数量
    uint16_t remaining_gold_coin;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 云台状态数据包 不需要
struct ReceiveJointState {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    float pitch;
    float yaw;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// !! 机器人增益和底盘能量数据包 需要
struct ReceiveBuff {
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    //机器人回血增益 百分比
    uint8_t recovery_buff;
    // 机器人射击热量冷却增益 直接值
    uint8_t cooling_buff;
    // 机器人防御增益 百分比
    uint8_t defence_buff;
    // 机器人负防御增益 百分比
    uint8_t vulnerability_buff;
    // 机器人攻击增益 百分比
    uint16_t attack_buff;
    /* 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，
    仅在机器人剩余能量小于 50% 时反馈，其余默认反馈 0x32。*/
    uint8_t remaining_energy;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData {
  HeaderFrame frame_header;
  uint8_t is_rotate;
  uint32_t time_stamp;

  struct {
    // 速度
    struct {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T> inline T fromVector(const std::vector<uint8_t> &data) {
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T> inline std::vector<uint8_t> toVector(const T &data) {
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
            packet.begin());
  return packet;
}

} // namespace standard_robot_pp_ros2

#endif // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
