#!/usr/bin/env python3

from flightros_msgs.msg import StampedFloatArray

import rospy

import numpy as np
from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction

# for warnings
import logging
logger = logging.getLogger('py.warnings')
logging.captureWarnings(True)
sh = logging.StreamHandler()
sh.setLevel(logging.DEBUG)
sh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)-1s - %(message)s'))
logger.addHandler(sh)
logger.setLevel(logging.DEBUG)

class BO(object):
    #
    def __init__(
        self
    ):  
        #
        self.init()

        # Publishers
        self.bo_pub_ = rospy.Publisher(
            "/bo_next", StampedFloatArray, queue_size=1, tcp_nodelay=False
        )

        # Subscribers
        self.infoEnergy_sub_ = rospy.Subscriber(
            "/infoEnergy", StampedFloatArray, self.getInfo, queue_size=1
        )

    def init(self):
        # Bayesian optimizer
        self.pbounds = {'x': (-1, 1), 'y': (-1.0, 1.0), 'z': (-1.0, 1.0)}
        self.optimizer = BayesianOptimization(f=None, pbounds=self.pbounds)
        self.utility = UtilityFunction(kind="ei", xi=0.05)

        # message
        self.infoEnergy_msg = StampedFloatArray()
        self.params_msg = StampedFloatArray()
        self.null_meas = None
        self.null_std = None

    def getInfo(self, msg):
        past_params = msg.data[0:3]
        past_meas = msg.data[3]
        past_std = msg.data[4]

        if self.null_meas is None:
            self.null_meas = past_meas
            self.null_std = past_std

            self.optimizer.set_gp_params(alpha=self.null_std**2, n_restarts_optimizer=1000)

        self.set_energyeff(past_params, past_meas - self.null_meas) #zero mean

    def set_energyeff(self, past_params, past_meas):
        try:
            self.optimizer.register(past_params, target=past_meas)
            print("BO: saved conf " + str(np.round(past_params[0],3)) + ", " + str(np.round(past_params[1],3)) + ", " + str(np.round(past_params[2],3)) + " with measurement " + str(np.round(past_meas,3)))
            next_params = self.optimizer.suggest(self.utility)
            ts = rospy.Time.now()
            self.logger_nextparams(ts, next_params)
        except:
            print("BO: failed registration " + str(np.round(past_params[0],3)) + ", " + str(np.round(past_params[1],3)) + ", " + str(np.round(past_params[2],3)))
        

    def logger_nextparams(self,ts,next_params):
        # logging action
        print("BO: next proposal " + str(np.round(next_params["x"],3)) + ", " + str(np.round(next_params["y"],3)) + ", " + str(np.round(next_params["z"],3)))
        self.params_msg.header.stamp = ts
        self.params_msg.data = [next_params["x"], next_params["y"], next_params["z"]]
        self.bo_pub_.publish(self.params_msg)

def main():
    # -- ros spin
    rospy.init_node("bo", anonymous=True)
    bo = BO()
    rospy.spin()

if __name__ == "__main__":
    main()