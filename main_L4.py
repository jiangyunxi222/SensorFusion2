import carla
import random
import pygame
import time
import socket
from math import tan, radians
from L4_fusion import *
from utils import *
from constants import *



# 配置UDP参数
UDP_IP = "127.0.0.1"
UDP_PORT = 9999
FREQUENCY = 50  # 50Hz
PERIOD = 1.0 / FREQUENCY  # Calculate the period

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()


    # 创建ego车辆
    ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    ego_bp.set_attribute('role_name', 'ego')
    spawn_point = random.choice(map.get_spawn_points())
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    ego_vehicle.set_autopilot(True)

    # 仿真设置
    running = True
    settings = world.get_settings()
    settings.synchronous_mode = True  # 开启同步模式
    settings.fixed_delta_seconds = 0.02 
    world.apply_settings(settings)

    L4_fusion = L4Fusion(world,ego_vehicle)
    try:
        while running:
            # 更新观察者视角
            ego_tf = ego_vehicle.get_transform()
            spectator = world.get_spectator()
            spectator_tf = carla.Transform(ego_tf.location + ego_tf.get_forward_vector() * (-10) + ego_tf.get_up_vector() * 3,ego_tf.rotation)
            spectator.set_transform(spectator_tf)
            # Record the start time
            start_time = time.time() 
            world.tick()
            position_data = L4_fusion.get_position_data()
            obstacle_data_list = L4_fusion.get_obstacle_data()
            print(position_data)
            print(obstacle_data_list)
            
            packet_data = pack_position_data(position_data)
            sock.sendto(packet_data, (UDP_IP, UDP_PORT))
            print(packet_data)
            elapsed_time = time.time() - start_time  # Calculate elapsed time
            time.sleep(max(0, PERIOD - elapsed_time))  # Sleep to maintain 50Hz frequency
            
    finally:
        ego_vehicle.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()
