#!/usr/bin/env python

########################################################################
# noise utils
########################################################################

import math
import numpy as np
import random

from collections import deque

class ErrorPerceptionLocalizationStruct:

    ########################################################################
    # PERCEPTION VARIABLES

    _frequency_update = 10

    _perception_latency_detection = 1
    _detection_alpha = 2
    _detection_beta = 5
    
    _perception_latency = 0.15

    _tracking_error_prob = 0.05
    _max_frames_not_detected = 4

    _prob_pcp_buildup_start = 0.05    
    _size_pcp_jump = 0.75
    _seconds_pcp_acum = 2


    # Noise based on relative distance
    _noise_agents_pos_lat_start = 0.1
    _noise_agents_pos_lat_rel = 0.05
    _noise_agents_pos_long_start = 0.1
    _noise_agents_pos_long_rel = 0.075
    _noise_agents_head_start = 10 # degrees
    _noise_agents_head_rel = 0.25 # degrees 
    
    # Noise based on relative value 
    _noise_agents_vel_start = 0.05
    _noise_agents_vel_rel = 0.1
    _noise_agents_acc_start = 0.05
    _noise_agents_acc_rel = 0.1
    _thld_agents_stopped = 0.1 # m/s
    _prob_agents_not_stopped = 0.05

    
    ########################################################################
    # LOCALIZATION VARIABLES
        

    _latency_localization = 0.15
    
    _prob_loc_buildup_start = 0.05    
    _prob_loc_small_jump = 0.8
    _size_loc_small_jump = 0.2
    _size_loc_big_jump = 0.75
    _seconds_loc_acum = 2
    
    _noise_loc_pos_lat = 0.1
    _noise_loc_pos_long = 0.3
    _noise_loc_head = 5 # degrees

    _noise_loc_speed = 0.2


########################################################################

"""
Errors on localization:
1 - Small latency on the position and velocity
2 - Buildup of noise when using odometry + big/small jump when relocated 
3 - Noise on lateral and longitudinal position
4 - Noise on heading
5 - Minimal noise on velocity based on odometry
"""

class ErrorLocalization:
	
    """
    Error for localization
    """
    
    def __init__(self, error_struct):
        
        self._error_struct = error_struct
        
        self._localization_state = 0 # 0-no error, 1-buildup odometry
        len_queue = max(math.ceil(self._error_struct._latency_localization * self._error_struct._frequency_update * 1.5), 10)
        self._localization_deque = deque([], maxlen=len_queue) # latency
        self._localization_acum_val = 0 # buildup error
        self._localization_direction = 0 # 1 positive, -1 negative
        self._localization_max_jump = 0

    
    def LocalizationUpdate(self, s_in, d_in, vel_in, yaw_in):

        # TODO: Verify units (radians, m, m/s)

        # Add noise on the yaw first
        yaw_out = random.gauss(yaw_in, math.radians(self._error_struct._noise_loc_head))

        # Add the noise in velocity
        vel_out = max(random.gauss(vel_in, self._error_struct._noise_loc_speed), 0)
        
        # Figure out the possible odometry error        
        if self._localization_state == 0:
            # Figure out if we are starting a buildup error on localization
            val_start_jump = random.uniform(0.0, 1.0)
            if val_start_jump <= self._error_struct._prob_loc_buildup_start:
                
                self._localization_state = 1
                
                val_size_jump = random.uniform(0.0, 1.0)                
                if val_size_jump <= self._error_struct._prob_loc_small_jump:
                    val_jump = self._error_struct._size_loc_small_jump
                else:
                    val_jump = self._error_struct._size_loc_big_jump

                self._localization_max_jump = random.uniform(0.5 * val_jump, val_jump)

                val_direction = random.randint(0, 1)
                if val_direction == 0:
                    self._localization_direction = -1
                else:
                    self._localization_direction = 1                
        else: 
            #Continue the buildup
            self._localization_acum_val += self._localization_direction * self._localization_max_jump / (self._error_struct._frequency_update * self._error_struct._seconds_loc_acum)

            # Reset error to
            if self._localization_acum_val * self._localization_direction > self._localization_max_jump:
                self._localization_state = 0
                self._localization_acum_val = 0
                self._localization_direction = 0
                self._localization_max_jump = 0

        # Add the noise to the position
        
        s_out = random.gauss(s_in + self._localization_acum_val, self._error_struct._noise_loc_pos_long)

        d_out = random.gauss(d_in + self._localization_acum_val, self._error_struct._noise_loc_pos_lat)

        self._localization_deque.append((s_out, d_out, vel_out, yaw_out))

        index_pick = int(math.ceil(self._error_struct._latency_localization * self._error_struct._frequency_update))

        if index_pick > len(self._localization_deque):
            index_pick = len(self._localization_deque)            

        return self._localization_deque[-index_pick]

########################################################################

"""
    Error on perception:
    1 - Latency first time they are detected    
    2 - Cars position is off for a little bit and jumps usually to the correct position (tracking error, new track)
    3 - Cars not detected for a small amount of frames
    4 - Noise on lateral and longitudinal position
    5 - Noise on heading position
    6 - Noise on velocity estimation
    7 - Car may be stopped but still sending some small velocity
"""


class ErrorPerception:

    """
    Error for perception
    """
    
    def __init__(self, error_struct, cur_time):
        
        self._error_struct = error_struct
        
        # Detection latency vars
        self._detection_state = False # 0-not detected yet, 1-detected
        self._counter_detection = 0
        self._max_counter_detection = int(round(random.betavariate(self._error_struct._detection_alpha, self._error_struct._detection_beta) * self._error_struct._perception_latency_detection * self._error_struct._frequency_update))

        self._perception_state = 0 # 0-no error, 1-buildup odometry
        len_queue = max(math.ceil(self._error_struct._perception_latency * self._error_struct._frequency_update * 1.5), 10)
        self._perception_deque = deque([], maxlen=len_queue) # latency
        self._perception_acum_val = 0 # buildup error
        self._perception_direction = 0 # 1 positive, -1 negative
        self._perception_max_jump = 0

        self._ghost_state = 0 # 0-car seen, 1-car not seen
        self._ghost_counter = 0
        self._ghost_counter_max = 0

        self._prev_time = cur_time
    
    def PerceptionUpdate(self, s_in, d_in, vel_in, a_in, yaw_in, cur_time, dist_with_ad):

        # TODO: Verify units (radians, m, m/s)


        self._prev_time = cur_time
        
        # Detection latency
        if self._counter_detection >= self._max_counter_detection:
            self._detection_state = True
        else:
            self._counter_detection += 1
            return (0, 0, 0, 0, 0, False)


        # Lost car for some frames
        if self._ghost_state == 0:
            prob_ghost = random.uniform(0.0, 1.0)
            if prob_ghost < self._error_struct._tracking_error_prob:
                self._ghost_state = 1
                self._ghost_counter_max = random.randint(2,5)
        else:
            self._ghost_counter += 1
            if self._ghost_counter >= self._ghost_counter_max:
                self._ghost_state = 0
                self._ghost_counter = 0
                self._ghost_counter_max = 0
                self._perception_deque.clear()

        if self._ghost_state == 1:            
            return (0, 0, 0, 0, 0, False)

        # Add noise on the yaw first
        rel_error_yaw = self._error_struct._noise_agents_head_start + dist_with_ad * self._error_struct._noise_agents_head_rel
        yaw_out = random.gauss(yaw_in, math.radians(rel_error_yaw))

        # Add the noise in velocity
        rel_error_vel = self._error_struct._noise_agents_vel_start + vel_in * self._error_struct._noise_agents_vel_rel
        vel_out = max(random.gauss(vel_in, rel_error_vel), 0)

        # For most vehicles a really small velocity would be considered as a stopped vehicle
        if vel_out < self._error_struct._thld_agents_stopped:
            prob_stopped = random.uniform(0.0, 1.0)
            if prob_stopped > self._error_struct._prob_agents_not_stopped:
                vel_out = 0

        # Add noise to acceleration
        rel_error_acc = self._error_struct._noise_agents_acc_start + abs(a_in) * self._error_struct._noise_agents_acc_rel
        a_out = random.gauss(a_in, rel_error_acc)
        
        # Figure out the possible odometry error        
        if self._perception_state == 0:
            # Figure out if we are starting a buildup error on perception
            val_start_jump = random.uniform(0.0, 1.0)
            if val_start_jump <= self._error_struct._prob_pcp_buildup_start:
                
                self._perception_state = 1
                
                val_jump = self._error_struct._size_pcp_jump
                self._perception_max_jump = random.uniform(0.5 * val_jump, val_jump)

                val_direction = random.randint(0, 1)
                if val_direction == 0:
                    self._perception_direction = -1
                else:
                    self._perception_direction = 1                
        else: 
            #Continue the buildup
            self._perception_acum_val += self._perception_direction * self._perception_max_jump / (self._error_struct._frequency_update * self._error_struct._seconds_pcp_acum)

            # Reset error to
            if self._perception_acum_val * self._perception_direction > self._perception_max_jump:
                self._perception_state = 0
                self._perception_acum_val = 0
                self._perception_direction = 0
                self._perception_max_jump = 0

        # Add the noise to the position
        rel_error_s = self._error_struct._noise_agents_pos_long_start + dist_with_ad * self._error_struct._noise_agents_pos_long_rel
        s_out = random.gauss(s_in + self._perception_acum_val, rel_error_s)

        rel_error_d = self._error_struct._noise_agents_pos_lat_start + dist_with_ad * self._error_struct._noise_agents_pos_lat_rel
        d_out = random.gauss(d_in + self._perception_acum_val, rel_error_d)

        self._perception_deque.append((s_out, d_out, vel_out, a_out, yaw_out, True))

        index_pick = int(math.ceil(self._error_struct._perception_latency * self._error_struct._frequency_update))

        if index_pick > len(self._perception_deque):
            index_pick = len(self._perception_deque)

        return self._perception_deque[-index_pick]
