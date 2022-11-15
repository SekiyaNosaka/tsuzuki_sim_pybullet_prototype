# -*- coding: utf-8 -*-

# General
import time
import math
import random
import numpy as np
import pybullet as p
import pybullet_data as pd
from tqdm import tqdm

# App
from utils import Models, Camera

class TsuzukiSim:
    def __init__(self, robot, models: Models, camera = None, vis = False) -> None: # python3系からの記述方法(悪しからず)
        self.robot = robot
        self.camera = camera
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False) # ncols: プログレスバーの長さ

        # 環境定義
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())

        p.setGravity(0, 0, -9.8) # 重力設定
        self.planeID = p.loadURDF('plane.urdf')

        self.robot.load()
        self.robot.step_simulation = self.step_simulation

        self.set_container()
        self.set_objects()

        # DebugParameter定義 (パラメータ名, 範囲[ , ], 初期値)
        self.xin                            = p.addUserDebugParameter('x', -0.224, 0.224, 0)
        self.yin                            = p.addUserDebugParameter('y', -0.224, 0.224, 0)
        self.zin                            = p.addUserDebugParameter('z', 0, 1., 0.8)
        self.rollId                         = p.addUserDebugParameter('roll', -np.pi, np.pi, 0)
        self.pitchId                        = p.addUserDebugParameter('pitch', -np.pi, np.pi, np.pi/2)
        self.yawId                          = p.addUserDebugParameter('yaw', -np.pi/2, np.pi/2, np.pi/2)
        self.gripper_opening_length_control = p.addUserDebugParameter('gripper_opening_length', 0, 0.085, 0.04)

    def read_debug_parameter(self):
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        z = p.readUserDebugParameter(self.zin)
        roll  = p.readUserDebugParameter(self.rollId)
        pitch = p.readUserDebugParameter(self.pitchId)
        yaw   = p.readUserDebugParameter(self.yawId)
        gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)
        return x, y, z, roll, pitch, yaw, gripper_opening_length

    def set_container(self):
        self.container_visualID = p.createVisualShape(
                shapeType = p.GEOM_MESH,
                fileName = '../meshes/container/container.stl',
                rgbaColor = [0.95, 0.95, 0.95, 1],
                specularColor = [1., 1., 1.],
                meshScale = [.001, .001, .001],
                )

        self.container_collisionID = p.createCollisionShape(
                shapeType = p.GEOM_MESH,
                fileName = '../meshes/container/container.stl',
                flags = p.GEOM_FORCE_CONCAVE_TRIMESH,
                meshScale = [.001, .001, .001],
                )

        self.container_init_position = [0., 0., 0.001]
        self.container_init_orientation = p.getQuaternionFromEuler([0., 0., 0.])

        self.container_ID = p.createMultiBody(
                baseMass = 15,
                baseVisualShapeIndex = self.container_visualID,
                baseCollisionShapeIndex = self.container_collisionID,
                basePosition = self.container_init_position,
                baseOrientation = self.container_init_orientation,
                )

    def set_objects(self):
        objID = []
        for i in range(5):
            pos_x = random.uniform(-0.3, 0.3)
            pos_y = random.uniform(-0.2, 0.2)
            pos_z = random.uniform(0.7, 0.9)
            pos = [pos_x, pos_y, pos_z]
    
            rx = random.uniform(-0.05, 0.05)
            ry = random.uniform(-0.05, 0.05)
            rz = random.uniform(-0.05, 0.05)
            ori = p.getQuaternionFromEuler([rx, ry, rz])
    
            #objID.append(p.loadURDF('../urdf/knuckle.urdf', pos, ori, flags = p.URDF_USE_SELF_COLLISION))
            objID.append(p.loadURDF('../urdf/knuckle_approximate.urdf', pos, ori, flags = p.URDF_USE_SELF_COLLISION))
            time.sleep(1./100.)

    def reset(self):
        # ロボットのリセット
        self.robot.reset()

        # コンテナのリセット
        p.resetBasePositionAndOrientation(
                self.container_ID,
                self.container_init_position,
                self.container_init_orientation,
                )

        # オブジェクトのリセット

    def step_simulation(self):
        p.stepSimulation()
        """# 09/12 add by nosaka
        if self.vis:
            time.sleep(1/250.)
            self.p_bar.update(1)
        """

    def step(self, action, control_method = 'joint'):
        assert control_method in ('joint', 'end')

        self.robot.move_ee(action[:-1], control_method)
        self.robot.move_gripper(action[-1])

        # TODO: 接触検知
        #attach_detect = p.getContactPoints()

        #self.camera.shot()

        self.step_simulation()
        """
        for _ in range(100):
            self.step_simulation()
        """
