from sensors import GnssSensor, ImuSensor, SemanticLidarSensor
import math
class L4Fusion():
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        #初始化传感器类
        self.gnss_sensor = GnssSensor(world,vehicle)
        self.imu_sensor = ImuSensor(world,vehicle)
        self.semlidar_sensor = SemanticLidarSensor(world,vehicle)

        #公共字段
        self.timestamp = 0 #时间戳

        ##################################################################################
    
        #定位数据0x01
        self.lon = 0 #经度
        self.lat = 0 #纬度
        self.heigt = 0 #高度

        self.pitch = 0 #俯仰角
        self.roll = 0 #横滚角
        self.heading = 0 #航向角

        self.liner_velocity = 0 #线速度矢量
        self.velocity_x = 0 #x轴速度信息
        self.velocity_y = 0 #y轴速度信息
        self.velocity_z = 0 #z轴速度信息

        self.liner_acceleration = 0 #线加速度矢量
        self.acceleration_x = 0 #x轴加速度信息
        self.acceleration_y = 0 #y轴加速度信息
        self.acceleration_z = 0 #z轴加速度信息
        self.angular_velocity = 0 #角速度矢量
        self.angular_velocity_x = 0 #x轴角速度信息
        self.angular_velocity_y = 0 #y轴角速度信息
        self.angular_velocity_z = 0 #z轴角速度信息

        self.origin_lon = 0 #参考点经度
        self.origin_lat = 0 #参考点纬度

        self.Utm_position_x = 0 #相对参考点x位置
        self.Utm_position_y = 0 #相对参考点y位置
        self.Utm_position_z = 0 #相对参考点y位置

        ##################################################################################

        #障碍物数据0x02
        self.id = 0 #障碍物ID
        self.Center_pos_vehicle_x = 0 #障碍物相对车辆x位置
        self.Center_pos_vehicle_y = 0 #障碍物相对车辆y位置
        self.Center_pos_vehicle_z = 0 #障碍物相对车辆z位置
        self.Center_pos_abs_x  = 0 #障碍物绝对x位置
        self.Center_pos_abs_y = 0 #障碍物绝对y位置
        self.Center_pos_abs_z = 0 #障碍物绝对z位置
        self.length = 0 #障碍物长度
        self.width = 0 #障碍物宽度
        self.height = 0 #障碍物高度
        self.type = 0 #障碍物类型
        self.confidence = 0 #障碍物置信度
        self.Lane_position = 0 #障碍物在车道的位置
        self.Fusion_type = 0 #融合类型
        self.Cross_id = 0 #交叉口ID
        self.Src_type = 0 #路段id
        self.Lane_id = 0 #车道id
        self.Lane_index =0 #车道序列号

        ##################################################################################


    def get_position_data(self, ref_point = [[0,0],[0,0,0]]):
        #获取传感器数据
        gnss_data = self.gnss_sensor.get_gnss_data()
        imu_data = self.imu_sensor.get_imu_data()
        transform_data = self.vehicle.get_transform()
        velocity_data = self.vehicle.get_velocity()

        self.timestamp = self.world.get_snapshot().timestamp.elapsed_seconds
        self.lon = gnss_data[0]
        self.lat = gnss_data[1]
        self.heigt = gnss_data[2]

        self.pitch = math.radians(transform_data.rotation.pitch)
        self.roll = math.radians(transform_data.rotation.roll)
        self.heading = math.radians(transform_data.rotation.yaw)

        self.liner_velocity = math.sqrt(velocity_data.x**2 + velocity_data.y**2 + velocity_data.z**2) #线速度矢量
        self.velocity_x = velocity_data.x #x轴速度信息
        self.velocity_y = velocity_data.y #y轴速度信息
        self.velocity_z = velocity_data.z #z轴速度信息

        self.liner_acceleration = imu_data[0][0]
        self.acceleration_x = imu_data[0][1]
        self.acceleration_y = imu_data[0][2]
        self.acceleration_z = imu_data[0][3]
        self.angular_velocity = imu_data[1][0]
        self.angular_velocity_x = imu_data[1][1]
        self.angular_velocity_y = imu_data[1][2]
        self.angular_velocity_z = imu_data[1][3]

        self.origin_lon = ref_point[0][0]
        self.origin_lat = ref_point[0][1]

        self.Utm_position_x = transform_data.location.x - ref_point[1][0]
        self.Utm_position_y = transform_data.location.y - ref_point[1][1]   
        self.Utm_position_z = transform_data.location.z - ref_point[1][2]

        return {
            "timestamp" : self.timestamp,
            "lon" : self.lon,
            "lat" : self.lat,
            "heigt" : self.heigt,
            "pitch" : self.pitch,
            "roll" : self.roll,
            "heading" : self.heading,
            "liner_velocity" : self.liner_velocity,
            "velocity_x" : self.velocity_x,
            "velocity_y" : self.velocity_y,
            "velocity_z" : self.velocity_z,
            "liner_acceleration" : self.liner_acceleration,
            "acceleration_x" : self.acceleration_x,
            "acceleration_y" : self.acceleration_y,
            "acceleration_z" : self.acceleration_z,
            "angular_velocity" : self.angular_velocity,
            "angular_velocity_x" : self.angular_velocity_x,
            "angular_velocity_y" : self.angular_velocity_y,
            "angular_velocity_z" : self.angular_velocity_z,
            "origin_lon" : self.origin_lon,
            "origin_lat" : self.origin_lat,
            "Utm_position_x" : self.Utm_position_x,
            "Utm_position_y" : self.Utm_position_y,
            "Utm_position_z" : self.Utm_position_z
            }

    def get_obstacle_data(self):
        #获取传感器数据
        if not self.semlidar_sensor.get_semantic_lidar_data():
            return None
        else:
            obstacle_data = self.semlidar_sensor.get_semantic_lidar_data()
            transform_data = self.vehicle.get_transform()

            obstacle_num = len(obstacle_data[0]) #障碍物个数
            obstacle_list = [] #障碍物数据列表初始化

            self.timestamp = self.world.get_snapshot().timestamp.elapsed_seconds


            for i in range(obstacle_num):

                self.id = obstacle_data[4][i]
                obstacle_actor = self.world.get_actor(self.id)
                obstacle_waypoint = self.world.get_map().get_waypoint(obstacle_actor.get_location(), project_to_road=True)

                self.Center_pos_vehicle_x = obstacle_data[0][i] - transform_data.location.x
                self.Center_pos_vehicle_y = obstacle_data[1][i] - transform_data.location.y
                self.Center_pos_vehicle_z = obstacle_data[2][i] - transform_data.location.z
                self.Center_pos_abs_x = obstacle_data[0][i]
                self.Center_pos_abs_y = obstacle_data[1][i]
                self.Center_pos_abs_z = obstacle_data[2][i]
                self.length = obstacle_actor.bounding_box.extent.x * 2
                self.width = obstacle_actor.bounding_box.extent.y * 2
                self.height = obstacle_actor.bounding_box.extent.z * 2
                self.type = obstacle_data[3][i]
                self.confidence = 1 #障碍物置信度没有提供具体公式，默认采用1
                self.Fusion_type = 1 #融合类型没有提供具体方案，默认采用1
                
                self.Lane_id = obstacle_waypoint.lane_id
                self.Lane_position = self.Lane_id - self.world.get_map().get_waypoint(self.vehicle.get_location(), project_to_road=True).lane_id
                #Lane_index 具体含义有待确认，此处采用section_id
                self.lane_index = obstacle_waypoint.section_id    

                if obstacle_waypoint.is_junction:
                    self.Cross_id = obstacle_waypoint.get_junction().id
                else:
                    self.Cross_id = -1
                self.Src_type = obstacle_waypoint.road_id
                
                
                
                obstacle_list.append({
                "timestamp" : self.timestamp,
                "id" : self.id,
                "Center_pos_vehicle_x" : self.Center_pos_vehicle_x,
                "Center_pos_vehicle_y" : self.Center_pos_vehicle_y,
                "Center_pos_vehicle_z" : self.Center_pos_vehicle_z,
                "Center_pos_abs_x" : self.Center_pos_abs_x,
                "Center_pos_abs_y" : self.Center_pos_abs_y,
                "Center_pos_abs_z" : self.Center_pos_abs_z,
                "length" : self.length,
                "width" : self.width,
                "height" : self.height,
                "type" : self.type,
                "confidence" : self.confidence,
                "Lane_position" : self.Lane_position,
                "Fusion_type" : self.Fusion_type,
                "Cross_id" : self.Cross_id,
                "Src_type" : self.Src_type,
                "Lane_id" : self.Lane_id,
                "lane_index" : self.lane_index
                })

            return obstacle_list




    


    def set_physical_values(self):
        pass

    def get_physical_values(self):
        pass