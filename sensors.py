import carla
import random
import pygame
import numpy as np
import math 
import weakref
from math import tan, radians

class ObstacleSensor(object):
    def __init__(self, world,parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.obstacle_actor = None
        self.Main_Obstacle_Type = 0
        self.Main_Obstacle_Pos_X = 0
        self.Main_Obstacle_Pos_Y = 0
        self.Main_Obstacle_Rel_Vel_X = 0
        self.Main_Obstacle_Rel_Vel_Y = 0
        obstacle_detector_bp = world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_detector_bp.set_attribute('distance', '100')
        obstacle_detector_bp.set_attribute('hit_radius', '0.5')
        obstacle_detector_bp.set_attribute('only_dynamics', 'True')
        obstacle_detector_bp.set_attribute('sensor_tick', '0.0')
        self.sensor = world.spawn_actor(obstacle_detector_bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: ObstacleSensor._obstacle_detector_callback(weak_self, sensor_data))

    @staticmethod
    def _obstacle_detector_callback(weak_self,sensor_data):
        '''obstacle detector的监听方法
        '''
        self = weak_self()
        ego_tf = sensor_data.actor.get_transform()
        self.obstacle_actor = sensor_data.other_actor
        if self.obstacle_actor is not None:
            obs_tf = self.obstacle_actor.get_transform()
            if self.obstacle_actor.type_id.startswith('vehicle'):
                self.Main_Obstacle_Type = 2
                 # 只有目标物在前时才有距离，且距离为正
                self.Main_Obstacle_Pos_X = self.get_relative_distance(ego_tf, obs_tf)[0]
                self.Main_Obstacle_Pos_Y = self.get_relative_distance(ego_tf, obs_tf)[1]
                self.Main_Obstacle_Rel_Vel_X = self.obstacle_actor.get_velocity().x - sensor_data.actor.get_velocity().x
                self.Main_Obstacle_Rel_Vel_Y = self.obstacle_actor.get_velocity().y - sensor_data.actor.get_velocity().y
            elif self.obstacle_actor.type_id.startswith('walker'):
                self.Main_Obstacle_Type = 1
                self.Main_Obstacle_Pos_X = self.get_relative_distance(ego_tf, obs_tf)[0]
                self.Main_Obstacle_Pos_Y = self.get_relative_distance(ego_tf, obs_tf)[1]
                self.Main_Obstacle_Rel_Vel_X = self.obstacle_actor.get_velocity().x - sensor_data.actor.get_velocity().x
                self.Main_Obstacle_Rel_Vel_Y = self.obstacle_actor.get_velocity().y - sensor_data.actor.get_velocity().y
            else:
                self.Main_Obstacle_Type = 0
        
    def get_relative_distance(self,ego_transform, target_transform):
        # 提取本车的坐标和偏航角（yaw）
        ego_location = ego_transform.location
        ego_yaw = math.radians(ego_transform.rotation.yaw)  # 转为弧度
        # 计算相对位置向量（目标车辆 - 本车）
        delta_x = target_transform.location.x - ego_location.x
        delta_y = target_transform.location.y - ego_location.y
        # 构建旋转矩阵（绕 Z 轴）
        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)
        # 坐标变换到本车坐标系
        longitudinal = delta_x * cos_yaw + delta_y * sin_yaw
        lateral = -delta_x * sin_yaw + delta_y * cos_yaw

        return longitudinal, lateral

class GnssSensor(object):
    def __init__(self,world,parent_actor):
        self._parent = parent_actor
        self.lat = 0 #gnss传感器的经度信息
        self.lon = 0 #gnss传感器的纬度信息
        self.height = 0 #gnss传感器的高程信息
        gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', '0.0')
        self.sensor = world.spawn_actor(gnss_bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: GnssSensor._gnss_callback(weak_self,sensor_data))
    #gnss传感器的回调函数
    @staticmethod
    def _gnss_callback(weak_self, sensor_data):
        self = weak_self()
        self.lat= sensor_data.latitude
        self.lon = sensor_data.longitude
        self.height = sensor_data.altitude
    
    def get_gnss_data(self):
        return [self.lat,self.lon,self.height]
    
class ImuSensor(object):
    def __init__(self,world,parent_actor):
        self._parent = parent_actor
        self.liner_acceleration = 0 #线加速度矢量
        self.acceleration_x = 0 #x轴加速度信息
        self.acceleration_y = 0 #y轴加速度信息
        self.acceleration_z = 0 #z轴加速度信息
        self.angular_velocity = 0 #角速度矢量
        self.angular_velocity_x = 0 #x轴角速度信息
        self.angular_velocity_y = 0 #y轴角速度信息
        self.angular_velocity_z = 0 #z轴角速度信息
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', '0.0')  #设置传感器的刷新频率
        self.sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=self._parent)  #将传感器绑定到车辆上
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: ImuSensor._imu_callback(weak_self, sensor_data))  #设置传感器的监听函数
    #imu传感器的回调函数
    @staticmethod
    def _imu_callback(weak_self, sensor_data):
        self = weak_self()
        self.acceleration_x = sensor_data.accelerometer.x
        self.acceleration_y = sensor_data.accelerometer.y
        self.acceleration_z = sensor_data.accelerometer.z
        self.angular_velocity_x = sensor_data.gyroscope.x
        self.angular_velocity_y = sensor_data.gyroscope.y
        self.angular_velocity_z = sensor_data.gyroscope.z
        #将xyz数据转换为矢量
        self.liner_acceleration = math.sqrt(self.acceleration_x**2+self.acceleration_y**2+self.acceleration_z**2)
        self.angular_velocity = math.sqrt(self.angular_velocity_x**2+self.angular_velocity_y**2+self.angular_velocity_z**2)
    
    def get_imu_data(self):
        return [[self.liner_acceleration,self.acceleration_x,self.acceleration_y,self.acceleration_z],
                [self.angular_velocity,self.angular_velocity_x,self.angular_velocity_y,self.angular_velocity_z]]
    
#语义点云雷达传感器
class SemanticLidarSensor(object):
    def __init__(self, world, parent_actor):
        self._parent = parent_actor
        self.sensor = None
        self.obstacle_x = []
        self.obstacle_y = []
        self.obstacle_z = []
        self.obstacle_type = []
        self.obstacle_id = []

        # 创建语义 LiDAR 传感器 blueprint
        SemLidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        SemLidar_bp.set_attribute('range', '50')  # 最大探测距离 50m
        SemLidar_bp.set_attribute('rotation_frequency', '10')  # 旋转频率 10Hz
        SemLidar_bp.set_attribute('channels', '32')  # LiDAR 通道数 32
        SemLidar_bp.set_attribute('upper_fov', '10')  # 上视角 10°
        SemLidar_bp.set_attribute('lower_fov', '-30')  # 下视角 -30°
        SemLidar_bp.set_attribute('points_per_second', '100000')  # 每秒点数 100000
        SemLidar_bp.set_attribute('sensor_tick', '0.0')  # 实时采样

        # 传感器在车辆上的位置
        lidar_transform = carla.Transform(carla.Location(x=1.0, z=2.8))
        self.sensor = world.spawn_actor(SemLidar_bp, lidar_transform, attach_to=self._parent)

        # 绑定回调函数
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: SemanticLidarSensor._semantic_lidar_callback(weak_self, sensor_data))

    @staticmethod
    def _semantic_lidar_callback(weak_self, sensor_data):
        """语义 LiDAR 数据回调函数，解析点云数据"""
        self = weak_self()
        
        # 0	    未知（Unknown）
        # 1	    建筑（Buildings）
        # 2	    道路（Fences）
        # 3	    其他（Other）
        # 4	    道路（Poles）
        # 5	    道路标志（RoadLines）
        # 6	    人行道（Roads）
        # 7	    人行道（Sidewalks）
        # 8	    路灯（Vegetation）
        # 9	    树木（Vegetation）
        # 10	墙壁（Walls）
        # 11	交通标志（TrafficSigns）
        # 12	路缘石（Curb）
        # 13	车辆（Vehicles）
        # 14	行人（Pedestrians）
        # 15	自行车（Bicycles）
        # 16	交通信号灯（TrafficLights）
        # 17	地面（Ground）
        ALLOWED_CLASS_IDS = {13}  # 自行根据需求替换实际 ID
        point_dtype = np.dtype([
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('cos_incidence', 'f4'),
        ('object_id', 'i4'),
        ('class_id', 'u1'),
        ('pad1', 'u1'),
        ('pad2', 'u1'),
        ('pad3', 'u1')
])

        # 解析点云数据
        point_cloud = np.frombuffer(sensor_data.raw_data, dtype=point_dtype)

        # 清空之前存储的数据
        self.obstacle_x.clear()
        self.obstacle_y.clear()
        self.obstacle_z.clear()
        self.obstacle_type.clear()
        self.obstacle_id.clear()

        # 仅保留 ALLOWED_CLASS_IDS 对应的点
        for point in point_cloud:
            if point['class_id'] in ALLOWED_CLASS_IDS:
                self.obstacle_x.append(float(point['x']))
                self.obstacle_y.append(float(point['y']))
                self.obstacle_z.append(float(point['z']))
                self.obstacle_type.append(int(point['class_id']))
                self.obstacle_id.append(int(point['object_id']))

        print(f"Detected {len(self.obstacle_x)} points (filtered) from semantic LiDAR.")



    def get_semantic_lidar_data(self):
        #列表为空时返回None
        if not self.obstacle_x:
            return None
        else:
            return [self.obstacle_x, self.obstacle_y, self.obstacle_z,
                    self.obstacle_type, self.obstacle_id]


        





            
class LaneSensor():
    def __init__(self,world, parent_actor):
        self.map = world.get_map()
        self._parent = parent_actor
        self.obstacle_actor = None
        self.left_lane_distance = 0
        self.right_lane_distance = 0
        self.left_lane_slope = 0
        self.right_lane_slope = 0
        self.left_lane_curvature = 0
        self.right_lane_curvature = 0
        self.left_lane_curvature_rate = 0
        self.right_lane_curvature_rate = 0
        self.last_left_lane_curvature = 0
        self.last_right_lane_curvature = 0
        
    def set_lane_infor(self):
        # ====== 精确车道边界计算 ======
        current_waypoint = self.map.get_waypoint(self._parent.get_location(), project_to_road=True)
        lane_width = current_waypoint.lane_width
        
        # 获取车辆位置和方向向量
        vehicle_loc = self._parent.get_location()
        waypoint_loc = current_waypoint.transform.location
        right_vector = current_waypoint.transform.get_right_vector()
        # print(right_vector)
        # 计算真实车道边界位置（相对当前waypoint中心）
        left_boundary_pos = waypoint_loc + carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        right_boundary_pos = waypoint_loc - carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        # 可视化真实边界
        self.world.debug.draw_string(left_boundary_pos, '|', color=carla.Color(255,0,0), life_time=0.1)
        self.world.debug.draw_string(right_boundary_pos, '|', color=carla.Color(0,255,0), life_time=0.1)
        # 计算横向距离（向量投影法）
        def lateral_distance(vehicle, boundary_point):
            v = boundary_point - vehicle
            return abs(np.dot([v.x, v.y], [right_vector.x, right_vector.y]))

        self.left_lane_distance  = lateral_distance(vehicle_loc, left_boundary_pos)
        self.right_lane_distance = lateral_distance(vehicle_loc, right_boundary_pos)

        # 获取相邻waypoints计算车道线斜率和曲率
        next_waypoints = [current_waypoint.next(i) for i in range(1,8)]
        next_waypoints = next_waypoints[:2]  # 取2个后续waypoints
        if not next_waypoints or len(next_waypoints) < 2:
            print("Warning: Not enough waypoints ahead.")

        # print(next_waypoints)

        next_wp1 = next_waypoints[0][0]
        next_wp2 = next_waypoints[1][0]
        
        left_boundary_next1 = next_wp1.transform.location + carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        left_boundary_next2 = next_wp2.transform.location + carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        
        right_boundary_next1 = next_wp1.transform.location - carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        right_boundary_next2 = next_wp2.transform.location - carla.Location(x=right_vector.x * (lane_width / 2), y=right_vector.y * (lane_width / 2))
        
        # 计算斜率
        def compute_slope(p1, p2):
            return (p2.y - p1.y) / (p1.x - p2.x) if abs(p2.x - p1.x) > 1e-6 else float('inf')
        self.left_lane_slope = compute_slope(left_boundary_pos, left_boundary_next1)
        self.right_lane_slope = compute_slope(right_boundary_pos, right_boundary_next1)
        
        # 计算曲率（基于三点拟合圆心半径）
        left_boundary_next2 = left_boundary_pos + carla.Location(x=2 * right_vector.x, y=2 * right_vector.y)
        right_boundary_next2 = right_boundary_pos + carla.Location(x=2 * right_vector.x, y=2 * right_vector.y)
        # 计算曲率
        def compute_curvature(p1, p2, p3):
            a = np.linalg.norm([p2.x - p1.x, p2.y - p1.y])
            b = np.linalg.norm([p3.x - p2.x, p3.y - p2.y])
            c = np.linalg.norm([p3.x - p1.x, p3.y - p1.y])
            s = (a + b + c) / 2  # 半周长
            area = np.sqrt(max(s * (s - a) * (s - b) * (s - c), 0))
            return (4 * area) / (a * b * c) if a * b * c != 0 else 0
        
        # 计算曲率的变化率
        

        self.left_lane_curvature = compute_curvature(left_boundary_pos, left_boundary_next1, left_boundary_next2)
        self.right_lane_curvature = compute_curvature(right_boundary_pos, right_boundary_next1, right_boundary_next2)
        
        self.left_lane_curvature_rate = self.left_lane_curvature - self.last_left_lane_curvature
        self.right_lane_curvature_rate = self.right_lane_curvature - self.last_right_lane_curvature

        self.last_left_lane_curvature = self.left_lane_curvature
        self.last_right_lane_curvature = self.right_lane_curvature

        # print(f"左车道线距离: {left_distance:.2f}, 右车道线距离: {right_distance:.2f}")
        # print(f"左车道线斜率: {left_lane_slope:.2f}, 右车道线斜率: {right_lane_slope:.2f}")
        # print(f"左车道线曲率: {left_lane_curvature:.4f}, 右车道线曲率: {right_lane_curvature:.4f}")
