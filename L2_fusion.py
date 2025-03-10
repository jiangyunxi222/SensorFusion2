from sensors import LaneSensor
from sensors import ObstacleSensor


class L2Fusion():
    def __init__(self, world, parent_actor):
        self.lane_sensor = LaneSensor(world,parent_actor)
        self.obstacle_sensor = ObstacleSensor(world,parent_actor)
        self.PFC_Main_Obstacle_Type =  0
        self.PFC_Main_Obstacle_Pos_X = 100.0   # 无偏移
        self.PFC_Main_Obstacle_Pos_Y = 10.0   # 实际值 = 原始值*0.0625 - 31.93
        self.PFC_Main_Obstacle_Rel_Vel_X = 20.0   # 实际值 = 原始值*0.0625 - 127.93
        self.PFC_Main_Obstacle_Rel_Vel_Y = 5.0
        self.Fault = 0
        self.Distance_to_Left_Lane = 1.6      # 实际值 = 原始值*0.01 - 16.0
        self.Left_Lane_Heading = 0.9             # 实际值 = 原始值*0.0009 - 0.9
        self.Left_Lane_Curvature = 0.02            # 实际值 = 原始值*0.00001 - 0.02
        self.Rate_of_curvature_Left = 0.0002048  # 实际值 = 原始值*0.0000001 - 0.0002048
        self.Lane1_Confidence = 0
        self.Distance_to_Right_Lane = 1.6
        self.Right_Lane_Heading = 0.9
        self.Right_Lane_Curvature = 0.02
        self.Rate_of_curvature_Right = 0.0002048
        self.Lane2_Confidence = 0
    
    def set_physical_values(self):
        self.PFC_Main_Obstacle_Type = self.obstacle_sensor.Main_Obstacle_Type
        self.PFC_Main_Obstacle_Pos_X = self.obstacle_sensor.Main_Obstacle_Pos_X
        self.PFC_Main_Obstacle_Pos_Y = self.obstacle_sensor.Main_Obstacle_Pos_Y
        self.PFC_Main_Obstacle_Rel_Vel_X = self.obstacle_sensor.Main_Obstacle_Rel_Vel_X
        self.PFC_Main_Obstacle_Rel_Vel_Y = self.obstacle_sensor.Main_Obstacle_Rel_Vel_Y
        self.Fault = 0  
        self.Distance_to_Left_Lane = self.lane_sensor.left_lane_distance
        self.Left_Lane_Heading = self.lane_sensor.left_lane_slope
        self.Left_Lane_Curvature = self.lane_sensor.left_lane_curvature
        self.Rate_of_curvature_Left = self.lane_sensor.left_lane_curvature
        self.Lane1_Confidence = 0
        self.Distance_to_Right_Lane = self.lane_sensor.right_lane_distance
        self.Right_Lane_Heading = self.lane_sensor.right_lane_slope
        self.Right_Lane_Curvature = self.lane_sensor.left_lane_curvature_rate
        self.Rate_of_curvature_Right = self.lane_sensor.right_lane_curvature_rate
        self.Lane2_Confidence = 0  
    
    def get_physical_values(self):
        return {
            "PFC_Main_Obstacle_Type": self.PFC_Main_Obstacle_Type,
            "PFC_Main_Obstacle_Pos_X": self.PFC_Main_Obstacle_Pos_X,
            "PFC_Main_Obstacle_Pos_Y": self.PFC_Main_Obstacle_Pos_Y,
            "PFC_Main_Obstacle_Rel_Vel_X": self.PFC_Main_Obstacle_Rel_Vel_X,
            "PFC_Main_Obstacle_Rel_Vel_Y": self.PFC_Main_Obstacle_Rel_Vel_Y,
            "Fault": self.Fault,
            "Distance_to_Left_Lane": self.Distance_to_Left_Lane,
            "Left_Lane_Heading": self.Left_Lane_Heading,
            "Left_Lane_Curvature": self.Left_Lane_Curvature,
            "Rate_of_curvature_Left": self.Rate_of_curvature_Left,
            "Lane1_Confidence": self.Lane1_Confidence,
            "Distance_to_Right_Lane": self.Distance_to_Right_Lane,
            "Right_Lane_Heading": self.Right_Lane_Heading,
            "Right_Lane_Curvature": self.Right_Lane_Curvature,
            "Rate_of_curvature_Right": self.Rate_of_curvature_Right,
            "Lane2_Confidence": self.Lane2_Confidence
        }



