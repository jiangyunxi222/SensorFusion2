import struct

def transfer_physical_values(physical_values,OFFSETS,RESOLUTIONS):
    """生成含偏移量的原始整数值（修正公式）"""
    # 目标物理值（实际需要传输的物理值）
        # 转换为原始整数（考虑偏移量）
    raw_values = {}
    for field, value in physical_values.items():
        if field in OFFSETS:
            offset = OFFSETS[field]
            resolution = RESOLUTIONS[field]
            # 修正公式：原始值 = (物理值 + 偏移量) / 分辨率
            raw = round((value + offset) / resolution)  # 四舍五入避免截断误差
        else:
            raw = round(value / RESOLUTIONS[field])
        raw_values[field] = raw
    
    # 强制检查范围（例如横向距离范围0~1023）
    print(raw_values["PFC_Main_Obstacle_Pos_Y"])
    if not (0 <= raw_values["PFC_Main_Obstacle_Pos_Y"] <= 1023):
        raise ValueError("横向相对距离超出协议范围")
    return raw_values


def pack_data(values):
    """打包为协议报文（大端16个字段）"""
    fmt = '>' + 'H' * 16
    data = struct.pack(fmt,
        values["PFC_Main_Obstacle_Type"],
        values["PFC_Main_Obstacle_Pos_X"],
        values["PFC_Main_Obstacle_Pos_Y"],
        values["PFC_Main_Obstacle_Rel_Vel_X"],
        values["PFC_Main_Obstacle_Rel_Vel_Y"],
        values["Fault"],
        values["Distance_to_Left_Lane"],
        values["Left_Lane_Heading"],
        values["Left_Lane_Curvature"],
        values["Rate_of_curvature_Left"],
        values["Lane1_Confidence"],
        values["Distance_to_Right_Lane"],
        values["Right_Lane_Heading"],
        values["Right_Lane_Curvature"],
        values["Rate_of_curvature_Right"],
        values["Lane2_Confidence"]
    )
    return data
