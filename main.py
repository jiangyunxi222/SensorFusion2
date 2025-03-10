import carla
import random
import pygame
import numpy as np
import time
import socket
import struct
from math import tan, radians
from L2_fusion import *
from L4_fusion import *
from constants import OFFSETS,RESOLUTIONS
from utils import *

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
    settings.fixed_delta_seconds = 0.02  # 每帧 50ms
    world.apply_settings(settings)

    L2_fusion = L2Fusion(world,ego_vehicle)
    L2_fusion.set_physical_values()
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
            # 通过L2融合传感器得到当前车辆状态
            L2_fusion.set_physical_values()
            physical_values = L2_fusion.get_physical_values()
            print(physical_values)
            raw_values = transfer_physical_values(physical_values, OFFSETS, RESOLUTIONS)
            packet_data = pack_data(raw_values)
            sock.sendto(packet_data, (UDP_IP, UDP_PORT))
            print(packet_data)
            elapsed_time = time.time() - start_time  # Calculate elapsed time
            time.sleep(max(0, PERIOD - elapsed_time))  # Sleep to maintain 50Hz frequency
            
    finally:
        ego_vehicle.destroy()
        pygame.quit()

if __name__ == "__main__":
    main()
