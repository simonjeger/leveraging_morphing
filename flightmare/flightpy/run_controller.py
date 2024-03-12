#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper
from scipy.spatial.transform import Rotation as R
import pandas as pd
import numpy as np
import torch
import time

from flightgym import RobotEnv_v1
from flightrl.rpg_baselines.torch.envs import vec_env_wrapper as wrapper
from mavros_msgs.msg import ActuatorControl
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from flightros_msgs.msg import State, StampedFloatArray
from controller.high_level_pid import HighLevelPID

import rospy
from std_msgs.msg import Bool
from control_msgs.msg import PidState

# for warnings
import logging
logger = logging.getLogger('py.warnings')
logging.captureWarnings(True)
sh = logging.StreamHandler()
sh.setLevel(logging.DEBUG)
sh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)-1s - %(message)s'))
logger.addHandler(sh)
logger.setLevel(logging.DEBUG)

class Pilot(object):
    #
    def __init__(
        self,
        cfg,
        policy,
        env=None
    ):

        # Environment
        self.env = env
        self.cfg = cfg
        self.policy = policy

        self.action = np.zeros(8)

        self.stop = False
        self.on = False
        self.on_time = None
        self.start_time = None
        self.policy_started = False

        # lowpass filter
        self.LP_cutoff_omega = 1.75
        self.LP_cutoff_aac = 1.75
        self.LP_prev_omega = np.zeros(3)
        self.LP_prev_aac = np.zeros(3)

        #
        self.control_rate = 1/self.cfg["simulation"]["ctr_dt"]
        self.control_rateLL = 50 #Hz
        self.control_rateExpl = 10 #Hz
        
        #
        self.init()

        # Publishers
        self.cmd_pub_ = rospy.Publisher(
            "/mavros/actuator_control", ActuatorControl, queue_size=1, tcp_nodelay=True
        )
        self.state_pub_ = rospy.Publisher(
            "/state", State, queue_size=1, tcp_nodelay=True
        )
        self.action_pub_ = rospy.Publisher(
            "/action", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.obs_pub_ = rospy.Publisher(
            "/observation", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.actuator_pub_ = rospy.Publisher(
            "/actuator", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )

        # Lowlevel controller
        self.stateLL_pub_ = rospy.Publisher(
            "/stateLL", State, queue_size=1, tcp_nodelay=True
        )
        self.actionLL_pub_ = rospy.Publisher(
            "/actionLL", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.infoLL_pub_ = rospy.Publisher(
            "/infoLL", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.infoEnergy_pub_ = rospy.Publisher(
            "/infoEnergy", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.infoConf_pub_ = rospy.Publisher(
            "/infoConf", StampedFloatArray, queue_size=1, tcp_nodelay=True
        )
        self.stop_sub = rospy.Subscriber(
            "/stop/state", PidState, self.interrupt_callback, queue_size=1
        )

        # Subscribers to get sensor measurements
        self.start_sub = rospy.Subscriber(
            "/start_policy", Bool, self.start_callback, queue_size=1
        )
        self.stop_sub = rospy.Subscriber(
            "/stop_policy", Bool, self.stop_callback, queue_size=1
        )
        self.pixhawk_vel_sub = rospy.Subscriber(
            "/mavros/local_position/velocity_body", TwistStamped, self.pixhawk_vel_callback, queue_size=1
        )
        self.pixhawk_imu_sub = rospy.Subscriber(
            "/mavros/imu/data", Imu, self.pixhawk_imu_callback, queue_size=1
        )
        self.pixhawk_pos_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.pixhawk_pos_callback, queue_size=1
        )
        self.pixhawk_battery_sub = rospy.Subscriber(
            '/mavros/battery', BatteryState, self.pixhawk_battery_callback, queue_size=1
        )
        self.bo_next_sub = rospy.Subscriber(
            '/bo_next', StampedFloatArray, self.bo_next_callback, queue_size=1
        )

        self.timer_control_loop = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.cmd_rates
        )

        self.timer_control_loopLL = rospy.Timer(
            rospy.Duration(1.0 / self.control_rateLL), self.cmd_LL
        )

        if self.cfg["temp"]["energy"]:
            self.timer_control_loopLL = rospy.Timer(
                rospy.Duration(1.0 / self.control_rateExpl), self.cmd_expl
            )

    def init(self):  
        self.init_pixhawk_pose = False
        self.init_pixhawk_vel = False
        self.init_pixhawk_imu = False
        self.init_pixhawk_battery = False
        self.init_step = False

        #
        self.action_ll = np.zeros(8)

        self.pos = np.array([0,0,1])
        self.quat = np.array([1,0,0,0])
        self.vel = np.array([0,0,0])
        self.omega = np.array([0,0,0])
        self.omega_old = self.omega
        self.acc = np.array([0,0,0])
        self.aac = np.array([0,0,0])

        self.thrust_old = 0


        #
        self.dt = self.cfg["simulation"]["ctr_dt"]
        #
        self.hist_act_buffer = []

        self.cmd_msg = ActuatorControl()
        self.cmd_msg.group_mix = ActuatorControl.PX4_MIX_FLIGHT_CONTROL

        self.state_msg = State()
        self.action_msg = StampedFloatArray()
        self.obs_msg = StampedFloatArray()
        self.actuator_msg = StampedFloatArray()

        self.stateLL_msg = State()
        self.actionLL_msg = StampedFloatArray()
        self.infoLL_msg = StampedFloatArray()
        self.infoEnergy_msg = StampedFloatArray()
        self.infoConf_msg = StampedFloatArray()

        self.alive = 10

        # Exploration for data generation
        self.expl_first_burnin = 60
        self.expl_timer = 0
        self.expl_timer_settle = None
        self.expl_state = 0
        self.expl_burnin = 10
        self.expl_running = False

        self.expl_params = {'x': 0, 'y': 0, 'z': 0}
        self.expl_params_current = {'x': 0, 'y': 0, 'z': 0}
        self.expl_params_best = {'x': 0, 'y': 0, 'z': 0}
        self.expl_params_sugg = None
        self.expl_params_sugg_ts = None
        self.expl_act_next = np.zeros(8)
        self.expl_act_current = np.zeros(8)
        self.expl_meas_list = []
        self.expl_n_max = 50
        self.expl_print = True
        self.expl_meas = None
        self.expl_null_meas = None
        self.expl_best = -np.inf
        self.expl_iter = 0
        self.expl_prior = []
        self.expl_prior_N = 3 # length of prior
        self.expl_iter_max = 16 # length of pure exploration, after prior
        self.expl_done = False
        
        # prior baeysian optimizer
        self.pbounds = {'x': (-1, 1), 'y': (-1.0, 1.0), 'z': (-1.0, 1.0)}
        df = pd.read_csv("hover.csv")
        self.fillPrior(df)

        logger.info('Codebase initialized')

    def start_callback(self, start):
        # Prepare environment
        pass

    def stop_callback(self, stop):
        pass

    def cmd_rates(self, timer):
        done = False
        ts = rospy.Time.now()
        if self.init_pixhawk_vel & self.init_pixhawk_imu & self.init_pixhawk_pose:
            # filling state
            state = np.concatenate([self.pos, self.quat, self.vel, self.omega, self.acc, self.aac], axis=0).astype(np.float64)
            
            obs, _, done_pos, _ = self.env.setRobotState(state)

            if done_pos: #setRobotState can't detect time out because it doesn't propagate time
                done = True
            actu = self.env.getRobotActuator() #only care about the first (and only) environment
            obs = np.array([obs])

            # predict action
            pitch = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_euler('ZYX')[1]
            if self.on_time is None:
                self.on_time = time.time()
            
            if ((pitch > -0.3) & (time.time() - self.on_time > 5.0)):
                self.on = True

            if (self.on & (not self.stop)):
                if self.start_time is None:
                    self.start_time = time.time()
                trans_time = 0
                trans_duration = 0
                if trans_duration != 0:
                    trans_smooth = np.clip((time.time() - self.start_time - trans_time) / trans_duration,0,1)
                else:
                    if time.time() - self.start_time < trans_time:
                        trans_smooth = 1
                    else:
                        trans_smooth = 0
                
                robot_state = np.concatenate(([rospy.Time.now().to_sec()],state)) #in simulation the first entry of the robot state is the time
                done = False
                
                if ((self.policy_started == False) & (trans_smooth == 1)):
                    logger.info("Started RateController")
                    self.policy_started = True
                action_rates = self.policy.predict(robot_state, deterministic=True)[0].flatten()

                if (self.expl_timer < 0) | (self.alive < self.expl_first_burnin): #if out of bounds during exploration or during take off sequence, make controller less agressive
                    action_rates[1:4] /= 1.2
                    action_rates[1:4] = np.clip(action_rates[1:4],-0.4,0.4)

            else:
                done = False
                action_rates = np.zeros(4)
                action_rates[0] = -1
                self.action = np.array([-1.0,0.0,0.0,-1.0,-1.0,-1.0,0.0,0.0])

            if self.alive < 10:
                self.action[0] = -1
                self.action[1] = 0
                self.action[2] = 0
                self.action[3] = -1
                self.action[4] = -1
                self.action[5] = -1
                self.action[6] = 0
                self.action[7] = 0

            self.logger_state(ts, state)
            self.logger_action(ts, action_rates)
            self.logger_observation(ts, obs)
            self.logger_actuator(ts, actu)

            if done:
                logger.info("Done state reached")
                self.alive = 0
            else:
                self.alive += 1/self.control_rate
            
            self.env.setLLDynamics(state, action_rates)

            if not self.init_step:
                logger.info('First step taken')
                self.init_step = True

    def cmd_LL(self, timer):
        ts = rospy.Time.now()

        if self.init_step:
            # filling state
            state_ll = np.concatenate([self.pos, self.quat, self.vel, self.omega, self.acc, self.aac], axis=0).astype(np.float64)
            if self.cfg["temp"]["energy"]:
                self.env.setLLOffset(self.expl_act_current) #should be the previous expl_act_currrent, but they are close together
            self.action_ll, gt_ll, ref_ll, proj_matrix_ll = self.env.callLLRun(state_ll)

            # limit thrust (otherwise the power cuts out because the ESC draws too much current)
            thrust_old = self.thrust_old
            self.action_ll[0] = np.clip(self.action_ll[0],-1,np.min((thrust_old+0.01,0.8)))
            self.thrust_old = self.action_ll[0]

            if (not self.on) or (self.alive < 10) or self.stop:
                self.action_ll[0] = -1
                self.action_ll[1] = 0
                self.action_ll[2] = 0
                self.action_ll[3] = -1
                self.action_ll[4] = -1
                self.action_ll[5] = -1
                self.action_ll[6] = 0
                self.action_ll[7] = 0

            self.loggerLL_state(ts, state_ll)
            self.loggerLL_action(ts, self.action_ll + self.expl_act_current)
            self.logger_infoLL(ts, gt_ll, ref_ll, proj_matrix_ll)

            self.cmd_msg.controls = self.convert_action(self.action_ll + self.expl_act_current)
            self.cmd_pub_.publish(self.cmd_msg)

    def cmd_expl(self, timer):
        ts = rospy.Time.now()
        if self.alive > self.expl_first_burnin:
            yaw_pitch_roll = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_euler('ZYX')
            roll_pitch_yaw = np.flip(yaw_pitch_roll)
            self.getEnergyEff(self.pos, roll_pitch_yaw, self.vel, self.omega, ts)

    def getEnergyEff(self, pos, roll_pitch_yaw, vel, omega, ts):
        pos_ok = True
        if np.linalg.norm(pos - np.array([0,0,1])) > 0.13:
            pos_ok = False
        ori_ok = np.linalg.norm(roll_pitch_yaw - np.array([0, -0.2, 0])) < 0.4
        vel_ok = np.linalg.norm(vel) < 0.5
        omega_ok = np.linalg.norm(omega) < 0.5

        # increase timer
        if pos_ok & ori_ok & vel_ok & omega_ok:
            self.expl_timer += 1/self.control_rateExpl
        else:
            self.expl_act_current /= 1.2 # dampen the reset instead of just setting = 0
        displ_message = ""
        if (pos_ok & ori_ok & vel_ok & omega_ok & (self.expl_timer >= self.expl_burnin)):
            if not self.expl_running:
                # Loading saved I reservoirs
                self.policy.store_reservoir()
                self.env.storeLLReservoir()
                print("Reservoir stored")

            # Next configuration
            if self.expl_state == 0:
                if self.expl_meas is None:
                    self.expl_params_sugg_ts = ts
                if (abs(ts - self.expl_params_sugg_ts) < rospy.Duration().from_sec(1.0) or (not self.expl_running) or self.expl_done): #make sure we take the current one
                    if self.expl_meas is not None:
                        if self.expl_iter < self.expl_prior_N:
                            self.expl_params = {'x': self.expl_prior[self.expl_iter][0], 'y': self.expl_prior[self.expl_iter][1], 'z': self.expl_prior[self.expl_iter][2]}
                        elif not self.expl_done:
                            self.expl_params = self.expl_params_sugg
                        else:
                            self.expl_params = self.expl_params_best
                        
                        self.expl_act_next[3] = self.expl_params["x"]
                        self.expl_act_next[4] = self.expl_params["y"]
                        self.expl_act_next[5] = self.expl_params["y"]
                        self.expl_act_next[6] = self.expl_params["z"]
                        self.expl_act_next[7] = self.expl_params["z"]

                        displ_message = "new configuration " + str(np.round(self.expl_params["x"],3)) + ", " + str(np.round(self.expl_params["y"],3)) + ", " + str(np.round(self.expl_params["z"],3))

                    self.expl_state = 1
                    self.expl_print = True
                else:
                    pass # waiting for BO

            # Reaching configuration
            elif self.expl_state == 1:
                self.expl_params_current["x"] = self.expl_act_current[3]
                self.expl_params_current["y"] = self.expl_act_current[4]
                self.expl_params_current["z"] = self.expl_act_current[6]

                diff = self.expl_act_next - self.expl_act_current
                diff[3] = np.clip(diff[3],-0.2/self.control_rateExpl, 0.2/self.control_rateExpl)
                diff[4:6] = np.clip(diff[4:6],-0.02/self.control_rateExpl, 0.02/self.control_rateExpl)
                diff[6:8] = np.clip(diff[6:8],-0.02/self.control_rateExpl, 0.02/self.control_rateExpl)
                if np.linalg.norm(pos - [0,0,1]) > 0.05:
                    diff = np.ones(8)*1e-10
                self.expl_act_current += diff

                displ_message = "configuration aborted at " + str(self.expl_act_current) + " on the way to " + str(self.expl_act_next)

                if np.sum(abs(diff)) == 0:
                    displ_message = "configuration reached"
                    self.expl_print = True
                    self.expl_state = 2

            # Wait for the system to settle
            elif self.expl_state == 2:
                if self.expl_timer_settle is None:
                    self.expl_timer_settle = self.expl_timer
                elif self.expl_timer - self.expl_timer_settle > 5.0:
                    self.expl_meas_list = []
                    self.expl_meas = None

                    displ_message = "system settled"
                    self.expl_print = True

                    self.expl_timer_settle = None
                    self.expl_state = 3

            # Collecting data in that configuration
            elif self.expl_state == 3:
                # Continue to the next state
                if (not np.isnan(self.expl_meas_list).any()):
                    if (len(self.expl_meas_list) >= self.expl_n_max):
                        self.expl_print = True
                        self.expl_state = 0
                        if self.expl_meas is None: #the first round is exploring in the default position
                            self.expl_iter += 1
                        self.expl_meas = np.mean(self.expl_meas_list)
                        self.expl_std = np.std(self.expl_meas_list)

                        if self.expl_null_meas is None:
                            self.expl_null_meas = self.expl_meas

                        if self.expl_meas > self.expl_best:
                            self.expl_best = self.expl_meas
                            self.expl_params_best = self.expl_params
                        if self.expl_iter == 0: #before there is no .max["target"]
                            displ_message = "measured " + str(np.round(self.expl_meas,3)) + ", iteration " + str(self.expl_iter) + "/" + str(self.expl_iter_max + self.expl_prior_N) + "\n"
                        else:
                            displ_message = "measured " + str(np.round(self.expl_meas,3)) + "  with best measured " + str(np.round(self.expl_best,3)) + ", iteration " + str(self.expl_iter) + "/" + str(self.expl_iter_max + self.expl_prior_N + 1) + "\n"
                        self.logger_infoEnergy(ts, self.expl_params_current, self.expl_meas, self.expl_std, self.expl_iter)
                else:
                    displ_message = "NaN measured, trying again"
                    self.expl_meas_list = []
                    self.expl_meas = None
                
                if self.expl_done:
                    self.expl_state = -2
            
            # Done collecting data
            elif self.expl_state == -1:
                self.expl_params = self.expl_params_best
                displ_message = "exploration done"
                self.expl_state = 0
                self.expl_print = True
                self.expl_done = True

            elif self.expl_state == -2:
                displ_message = "best state reached"
                self.expl_state = -3
                self.expl_print = True

            if (self.expl_iter >= self.expl_prior_N + self.expl_iter_max + 1) & (not self.expl_done):
                self.expl_state = -1

            if self.expl_print & (len(displ_message) > 0):
                print("EnergyEff: " + displ_message)
                self.expl_print = False

            self.expl_running = True
        else:

            # Out of bounds
            if self.expl_running:
                print("Out of bounds: pos:", pos_ok, "ori:", ori_ok, "vel:", vel_ok, "omega:", omega_ok)
                self.expl_state = 0.0
                self.expl_meas = self.expl_null_meas*1.1 #So this doesn't get revisited
                self.expl_iter += 1
                self.logger_infoEnergy(ts, self.expl_params_current, self.expl_meas, self.expl_std, self.expl_iter)

                # Settings for next round
                self.expl_running = False
                self.expl_timer = -10 #to try out

                # Loading load I reservoir
                self.policy.restore_reservoir()
                self.env.restoreLLReservoir()
                print("Reservoir loaded")
        self.logger_infoConf(ts, self.expl_act_current)
    
    def fillPrior(self, df):
        df.sort_values(by=['thr'], inplace=True)
        self.expl_prior = []
        i = 0
        while len(self.expl_prior) < self.expl_prior_N:
            contester = df[["conf_0", "conf_1", "conf_2"]].iloc[i].to_numpy()
            take = True
            
            if (contester[0] < self.pbounds["x"][0]) or (contester[0] > self.pbounds["x"][1]):
                take = False
            if (contester[1] < self.pbounds["y"][0]) or (contester[1] > self.pbounds["y"][1]):
                take = False
            if (contester[2] < self.pbounds["z"][0]) or (contester[2] > self.pbounds["z"][1]):
                take = False
            else:
                for j in range(len(self.expl_prior)):
                    if np.linalg.norm(contester - self.expl_prior[j]) < 0.5: #to not explore things that are too close to each other
                        take = False
                        break
                    if np.linalg.norm(contester) < 0.5:
                        take = False
                        break
            if take:
                self.expl_prior.append(contester)
            i += 1

    def convert_action(self, action):
        thrust = action[0]
        tail_pitch = action[1]
        tail_rudder = action[2]
        tail_sweep = action[3]
        left_wing_sweep = action[4]
        right_wing_sweep = action[5]
        left_wing_twist = action[6]
        right_wing_twist = action[7]

        thrust = self.nonLinThrust(thrust)
        thrust = thrust/2+0.5 #because thrust takes [0,1]
        
        tail_pitch_min = -0.94
        tail_pitch_max = 0.16
        tail_rudder_min = 0.32
        tail_rudder_max = -0.84
        tail_sweep_min = -0.16
        tail_sweep_max = 1.0
        left_sweep_min = 0.64
        left_sweep_max = -0.54
        right_sweep_min = -0.72
        right_sweep_max = 0.61
        left_twist_min = -0.82
        left_twist_max = 0.56
        right_twist_min = 1.0
        right_twist_max = -0.6

        #to compensate for offsets
        twist_o = -0.07
        left_twist_min += twist_o
        left_twist_max += twist_o
        right_twist_min += twist_o #+ because it's reversed
        right_twist_max += twist_o

        thrust = thrust
        tail_pitch = tail_pitch*(tail_pitch_max - tail_pitch_min)/2 + (tail_pitch_max + tail_pitch_min)/2
        tail_rudder = tail_rudder*(tail_rudder_max - tail_rudder_min)/2 + (tail_rudder_max + tail_rudder_min)/2
        tail_sweep = tail_sweep*(tail_sweep_max - tail_sweep_min)/2 + (tail_sweep_max + tail_sweep_min)/2
        left_wing_sweep = left_wing_sweep*(left_sweep_max - left_sweep_min)/2 + (left_sweep_max + left_sweep_min)/2
        right_wing_sweep = right_wing_sweep*(right_sweep_max - right_sweep_min)/2 + (right_sweep_max + right_sweep_min)/2
        left_wing_twist = left_wing_twist*(left_twist_max - left_twist_min)/2 + (left_twist_max + left_twist_min)/2
        right_wing_twist = right_wing_twist*(right_twist_max - right_twist_min)/2 + (right_twist_max + right_twist_min)/2

        return [left_wing_twist, tail_pitch, tail_rudder, thrust, right_wing_twist, left_wing_sweep, right_wing_sweep, tail_sweep]

    def logger_state(self,ts,state):
        # logging state
        self.state_msg.header.stamp = ts
        self.state_msg.position.x = state[0]
        self.state_msg.position.y = state[1]
        self.state_msg.position.z = state[2]
        self.state_msg.orientation.w = state[3]
        self.state_msg.orientation.x = state[4]
        self.state_msg.orientation.y = state[5]
        self.state_msg.orientation.z = state[6]
        self.state_msg.linear_velocity.x = state[7]
        self.state_msg.linear_velocity.y = state[8]
        self.state_msg.linear_velocity.z = state[9]
        self.state_msg.angular_velocity.x = state[10]
        self.state_msg.angular_velocity.y = state[11]
        self.state_msg.angular_velocity.z = state[12]
        self.state_msg.linear_acceleration.x = state[13]
        self.state_msg.linear_acceleration.y = state[14]
        self.state_msg.linear_acceleration.z = state[15]
        self.state_msg.angular_acceleration.x = state[16]
        self.state_msg.angular_acceleration.y = state[17]
        self.state_msg.angular_acceleration.z = state[18]
        self.state_pub_.publish(self.state_msg)

    def logger_action(self,ts,action):
        # logging action
        self.action_msg.header.stamp = ts
        self.action_msg.data = action.tolist()
        self.action_pub_.publish(self.action_msg)

    def logger_observation(self,ts,obs):
        # logging observation
        self.obs_msg.header.stamp = ts
        self.obs_msg.data = obs.flatten().tolist()
        self.obs_pub_.publish(self.obs_msg)

    def logger_actuator(self,ts,actu):
        # logging actuator
        self.actuator_msg.header.stamp = ts
        self.actuator_msg.data = actu.tolist()
        self.actuator_pub_.publish(self.actuator_msg)

    def loggerLL_state(self,ts,state):
        # logging state
        self.stateLL_msg.header.stamp = ts
        self.stateLL_msg.position.x = state[0]
        self.stateLL_msg.position.y = state[1]
        self.stateLL_msg.position.z = state[2]
        self.stateLL_msg.orientation.w = state[3]
        self.stateLL_msg.orientation.x = state[4]
        self.stateLL_msg.orientation.y = state[5]
        self.stateLL_msg.orientation.z = state[6]
        self.stateLL_msg.linear_velocity.x = state[7]
        self.stateLL_msg.linear_velocity.y = state[8]
        self.stateLL_msg.linear_velocity.z = state[9]
        self.stateLL_msg.angular_velocity.x = state[10]
        self.stateLL_msg.angular_velocity.y = state[11]
        self.stateLL_msg.angular_velocity.z = state[12]
        self.stateLL_msg.linear_acceleration.x = state[13]
        self.stateLL_msg.linear_acceleration.y = state[14]
        self.stateLL_msg.linear_acceleration.z = state[15]
        self.stateLL_msg.angular_acceleration.x = state[16]
        self.stateLL_msg.angular_acceleration.y = state[17]
        self.stateLL_msg.angular_acceleration.z = state[18]
        self.stateLL_pub_.publish(self.stateLL_msg)

    def loggerLL_action(self,ts,action):
        self.actionLL_msg.header.stamp = ts
        self.actionLL_msg.data = action.tolist()
        self.actionLL_pub_.publish(self.actionLL_msg)

    def logger_infoLL(self,ts,gt,ref,proj_matrix):
        self.infoLL_msg.header.stamp = ts
        self.infoLL_msg.data = np.concatenate([gt.flatten(), ref.flatten(), proj_matrix.flatten()])
        self.infoLL_pub_.publish(self.infoLL_msg)

    def logger_infoConf(self, ts, conf):
        self.infoConf_msg.header.stamp = ts
        self.infoConf_msg.data = conf
        self.infoConf_pub_.publish(self.infoConf_msg)

    def logger_infoEnergy(self, ts, params, meas, std, iter):
        self.infoEnergy_msg.header.stamp = ts
        self.infoEnergy_msg.data = np.array([params["x"], params["y"], params["z"], meas, std, iter])
        self.infoEnergy_pub_.publish(self.infoEnergy_msg)

    def pixhawk_imu_callback(self, data):
        # setting quat
        self.quat = np.array([data.orientation.w,data.orientation.x,data.orientation.y,data.orientation.z]) # w,x,y,z

        # setting acceleration
        acc = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])

        # subtracting g
        g_w = np.array([0,0,9.81])
        g = self.world_to_body(g_w)
        self.acc = acc - g

        # angular velocity
        omega = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        
        # angular acceleration
        aac = (omega - self.omega_old)*self.control_rateLL
        self.omega_old = omega
        
        # LP filter
        self.omega = self.lowPass(omega, self.LP_prev_omega, self.LP_cutoff_omega, 1/self.control_rateLL)
        self.LP_prev_omega = self.omega
        
        # self.aac_unfiltered = aac #DEBUG
        self.aac = self.lowPass(aac, self.LP_prev_aac, self.LP_cutoff_aac, 1/self.control_rateLL)
        self.LP_prev_aac = self.aac

        if not self.init_pixhawk_imu:
            logger.info('Pixhawk_imu initialized')
            self.init_pixhawk_imu = True

    def pixhawk_vel_callback(self, data):
        self.vel = np.array([data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])

        if not self.init_pixhawk_vel:
            logger.info('Pixhawk_vel initialized')
            self.init_pixhawk_vel = True

    def pixhawk_pos_callback(self, data):
        offset = [0.0,-0.03,0.05] # Compensate offset by MoCap coordinate system
        self.pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.pos += offset

        if not self.init_pixhawk_pose:
            logger.info('Pixhawk_pose initialized')
            self.init_pixhawk_pose = True

    def pixhawk_battery_callback(self, data):
        if (not np.isnan(data.voltage)) & (not np.isnan(data.current)):
            self.expl_meas_list.append(data.voltage*data.current) #W, current is negative when discharging

            if not self.init_pixhawk_battery:
                logger.info('Pixhawk_battery initialized')
                self.init_pixhawk_battery = True

    def bo_next_callback(self, data):
        self.expl_params_sugg  = {'x': data.data[0], 'y': data.data[1], 'z': data.data[2]}
        self.expl_params_sugg_ts = rospy.Time.now()


    def nonLinThrust(self, x):
        x = x/2+0.5 #because fitting over symmetric domain doesn't make sense
        y = [-0.04426512620393867, 0.7742715538958314, 0.2770086111476493]
        fit = y[0] + x*y[1] + x**2*y[2]
        fit_norm = (fit-0.5)*2
        return np.clip(fit_norm,-1,1)

    def world_to_body(self, vec):
        R_matrix = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_matrix()
        return np.matmul(R_matrix.transpose(), vec)

    def lowPass(self, input, prevInput, LP_cutoff, dt):
        w = 2*np.pi * LP_cutoff
        ang_rate_d = w*(-prevInput + input)
        output = prevInput + dt*ang_rate_d
        return output

    def interrupt_callback(self, data):
        if data.output >= 1:
            self.stop = True
        else:
            self.stop = False

def main():    
    cfg = YAML().load(open("/config.yaml", "r"))
    
    rospy.init_node("flightpilot", anonymous=True)
    torch.set_num_threads(1)

    # environment configurations
    cfg["simulation"]["num_envs"] = 1

    # create policy
    env = RobotEnv_v1(dump(cfg, Dumper=RoundTripDumper), False)
    env = wrapper.FlightEnvVec(env)

    policy = HighLevelPID(1/50) #50 Hz

    # racing ROS pilot
    pilot = Pilot(
        cfg,
        policy,
        env=env
    )

    # -- ros spin
    rospy.spin()


if __name__ == "__main__":
    main()