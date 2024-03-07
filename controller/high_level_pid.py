import numpy as np
from controller.simple_pid import PID
from scipy.spatial.transform import Rotation as R
from ruamel.yaml import YAML

import matplotlib.pyplot as plt

class HighLevelPID:
    def __init__(self, ctr_dt):
        self.ctr_dt = ctr_dt
        self.act_dim = 4

        self.pid_thr = PID(1.0, 0.5, 1.0, output_limits=(-1,1))
        self.pid_y = PID(0.15, 0.1, 0.1, output_limits=(-0.5,0.5))
        self.pid_z = PID(0.4, 0.15, 0.1, output_limits=(-0.5,0.5))

        self.pid_roll = PID(4.0, 0.0, 0.1, output_limits=(-1.0,1.0))
        self.pid_pitch = PID(3.0, 0.0, 0.05, output_limits=(-1.0,1.0))
        self.pid_yaw = PID(4.0, 0.0, 0.15, output_limits=(-1.0,1.0))

        self.offset_pitch = -0.15

    def predict(self, state, deterministic=True):
        goal = [0,0,1]
        return self.update(state, goal), []

    def update(self, state, goal):
        state = state.flatten()
        self.pos = state[1:4]
        self.quat = state[4:8]
        self.vel = state[8:11]
        self.omega = state[11:14]
        self.acc = state[14:17]
        self.aac = state[17:20]

        yaw_pitch_roll = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_euler('ZYX')
        self.roll_pitch_yaw = np.flip(yaw_pitch_roll)

        # actions
        actions = np.zeros(self.act_dim)

        time = state[0]
        if self.time_init is None:
            self.time_init = time

        # goal in body frame
        goal_b = self.world_to_body(goal - self.pos)
        
        # position control
        self.pid_y.setpoint = 0
        self.pid_z.setpoint = 0

        roll_pitch_yaw_des = np.zeros(3)
        roll_pitch_yaw_des[0] = -self.pid_y(-goal_b[1], vel=self.vel[1], dt=self.ctr_dt)
        roll_pitch_yaw_des[1] = -self.pid_z(-goal_b[2], vel=self.vel[2], dt=self.ctr_dt) + self.offset_pitch
        roll_pitch_yaw_des[2] = -roll_pitch_yaw_des[0]

        # thrust control
        self.pid_thr.setpoint = 0
        a_thr = self.pid_thr(-goal_b[0]-goal_b[2]/4, vel=self.vel[0], dt=self.ctr_dt) #+ self.offset_thrust

        # rate control
        self.pid_roll.setpoint = roll_pitch_yaw_des[0]
        self.pid_pitch.setpoint = roll_pitch_yaw_des[1]
        self.pid_yaw.setpoint = roll_pitch_yaw_des[2]

        omega_des = np.zeros(3)
        omega_des[0] = self.pid_roll(self.roll_pitch_yaw[0], vel=self.omega[0], dt=self.ctr_dt)
        omega_des[1] = self.pid_pitch(self.roll_pitch_yaw[1], vel=self.omega[1], dt=self.ctr_dt)
        omega_des[2] = self.pid_yaw(self.roll_pitch_yaw[2], vel=self.omega[2], dt=self.ctr_dt)
        
        actions = np.hstack([a_thr, omega_des])

        return actions
    
    def store_reservoir(self):
        self.pid_y.store_reservoir()
        self.pid_z.store_reservoir()
        self.pid_thr.store_reservoir()
        self.pid_roll.store_reservoir()
        self.pid_pitch.store_reservoir()
        self.pid_yaw.store_reservoir()

    def restore_reservoir(self):
        self.pid_y.restore_reservoir()
        self.pid_z.restore_reservoir()
        self.pid_thr.restore_reservoir()
        self.pid_roll.restore_reservoir()
        self.pid_pitch.restore_reservoir()
        self.pid_yaw.restore_reservoir()

    def world_to_body(self, vec):
        R_matrix = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_matrix()
        return np.matmul(R_matrix.transpose(), vec)
