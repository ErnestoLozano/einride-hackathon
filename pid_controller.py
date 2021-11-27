import websocket
import _thread
import time
import cv2
import matplotlib.pyplot as plt

import random
import numpy as np

from simple_pid import PID


def steering_angle_f(lane_center_position, tau_p = 1, tau_i = 0.1, tau_d = 0.05, pid_sample_time = 0.01):
        
    #pid = PID(tau_p, tau_i, tau_d, setpoint = center_position, sample_time = pid_sample_time)
    pid = PID(tau_p, tau_i, tau_d, setpoint = 0, sample_time = pid_sample_time)
    
    error_distance = 0.5 - lane_center_position/160 
    
    steering = pid(error_distance)
    
    if steering > max_steering_angle:
        steering = max_steering_angle
    if steering < -max_steering_angle:
        steering = -max_steering_angle
                
    return steering


def throttle_f(angle, const = 0.25, coeff = 0.1, max_throttle = 0.5):
    
    throttle = const - coeff * abs(angle)
    
    if throttle > max_throttle:
        throttle = max_throttle
    if throttle < -max_throttle:
        throttle = -max_throttle
    
    
    return throttle


def move(center_position, max_steering_angle, max_throttle, throttle_const, throttle_coeff, tau_p, tau_d, tau_i, pid_sample_time):
    """
    steering = front wheel steering angle, limited by max_steering_angle
    throttle = based on steering angle
    """
    
    steer = steering_angle_f(lane_center_position = center_position, tau_p = 1, tau_d = 0.1, tau_i = 0.05, pid_sample_time = pid_sample_time)
    throttle = throttle_f(angle = steer, const = throttle_const, coeff = throttle_coeff, max_throttle = max_throttle)
    
    return steer, throttle




