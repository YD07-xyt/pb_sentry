const uint8_t SOF_RECEIVE = 0xA5;
const uint8_t SOF_SEND = {'M'};

// !! 比赛信息数据包id 需要
const uint16_t ID_GAME_STATUS = 0x0001;
// !!!! 机器人状态数据包id 需要
const uint16_t ID_ROBOT_STATUS = 0x0201;


struct HeaderFrame {
  uint8_t sof; // 数据帧起始字节，固定值为 M
  uint16_t len; // 数据段长度
  uint8_t id;  // 数据段seq 包序号
  uint8_t crc; // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// !!! 比赛信息数据包 需要
struct ReceiveGameStatusData {
  HeaderFrame frame_header;
  uint16_t cmd_id;
  struct {
    
    uint8_t game_type : 4;
    // 比赛进度 当前比赛阶段 0-5
    uint8_t game_progress : 4;
    // 当前比赛阶段剩余时间
    uint64_t SyncTimeStamp;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));


struct ReceiveRobotStatus {
  HeaderFrame frame_header;
  uint16_t cmd_id;
    // !!  机器人状态数据包 需要
  struct
  {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
  }__attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));


/* Send data                                            */
/********************************************************/

struct SendRobotCmdData {
  HeaderFrame frame_header;
  uint32_t time_stamp;
  uint8_t is_scan;
  uint8_t sentry_pose;
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