# -*- coding: utf-8 -*-

# General
import numpy as np
import pybullet as p

# App
from utils import Camera
from robot import UR5Robotiq85
from env import TsuzukiSim

def main():
    robot = UR5Robotiq85(
            (-0.5, 0., 0.555), # ロボット取付け位置[m]
            (0., 0., -0.7853), # ロボット取付け姿勢[rad] default: (0., 0., 0.)
            )

    camera = Camera(
            (0, 0, 1.6), # カメラ目の位置[m]
            (0., 0., 0.), # カメラ視点位置[m]
            (-1, 0, 0), # カメラのアップベクター (norm: 1)
            0.05, # 近距離平面 (perhaps [m]) 0.27
            3., # 遠距離平面 (perhaps [m]) 3.
            (640, 512), # 画素サイズ[px] (width, height)
            45.94, # FOV [deg] 45.92
            )

    env = TsuzukiSim(
            robot=robot,
            models=None,
            camera=camera,
            vis=True,
            )
    
    # ---- main state ----
    env.reset()
    while True:
        env.step(env.read_debug_parameter(), 'end')

if __name__ == '__main__':
    main()
