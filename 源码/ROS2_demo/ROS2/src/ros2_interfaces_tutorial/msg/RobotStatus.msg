# RobotStatus.msg
# 包含机器人当前状态的信息

# 时间戳
builtin_interfaces/Time stamp

# 机器人ID
int32 robot_id

# 机器人名称
string robot_name

# 机器人状态 (0: 空闲, 1: 忙碌, 2: 错误)
uint8 status

# 电池电量 (百分比)
float32 battery_percentage

# 位置信息
float64 position_x
float64 position_y
float64 position_z

# 方向信息（四元数）
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w

# 是否有错误
bool has_error

# 错误消息（如果有）
string error_message 