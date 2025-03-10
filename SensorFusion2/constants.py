
# 定义所有字段的偏移量（正值，根据协议文档）
OFFSETS = {
    # 主目标相关
    "PFC_Main_Obstacle_Pos_Y": 31.93,
    "PFC_Main_Obstacle_Rel_Vel_X": 127.93,
    "PFC_Main_Obstacle_Rel_Vel_Y": 31.93,
    # 左车道线
    "Distance_to_Left_Lane": 16.0,
    "Left_Lane_Heading": 0.9,
    "Left_Lane_Curvature": 0.02,
    "Rate_of_curvature_Left": 0.0002048,
    # 右车道线
    "Distance_to_Right_Lane": 16.0,
    "Right_Lane_Heading": 0.9,
    "Right_Lane_Curvature": 0.02,
    "Rate_of_curvature_Right": 0.0002048

}

# 各字段的分辨率（来自协议文档）
RESOLUTIONS = {
    "PFC_Main_Obstacle_Type": 1,
    "PFC_Main_Obstacle_Pos_X": 0.0625,
    "PFC_Main_Obstacle_Pos_Y": 0.0625,
    "PFC_Main_Obstacle_Rel_Vel_X": 0.0625,
    "PFC_Main_Obstacle_Rel_Vel_Y": 0.0625,
    "Fault": 1,
    "Distance_to_Left_Lane": 0.01,
    "Left_Lane_Heading": 0.0009,
    "Left_Lane_Curvature": 0.00001,
    "Rate_of_curvature_Left": 0.0000001,
    "Lane1_Confidence": 1,
    "Distance_to_Right_Lane": 0.01,
    "Right_Lane_Heading": 0.0009,
    "Right_Lane_Curvature": 0.00001,
    "Rate_of_curvature_Right": 0.0000001,
    "Lane2_Confidence": 1
}


# 配置UDP参数
UDP_IP = "127.0.0.1"
UDP_PORT = 9999
FREQUENCY = 50  # 50Hz
