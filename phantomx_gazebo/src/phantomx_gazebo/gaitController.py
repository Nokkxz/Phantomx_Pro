#!/usr/bin/env python

import math

class LegJointMotionOverTime: # prototype of joint motion
    """Walk Joint Function"""
    def __init__(self):
        self.scale = 1 # scale depends on the group of joint

    def getAngleAtTick(self, loop_time_tick):
        """loop_time_tick should be a float between 0 and 1"""
        f = math.sin(math.pi * loop_time_tick) #to make leg move in loop
        return self.scale * f

class GaitPlanner:
    """Walk Function"""
    def __init__(self):

        # default param
        self.parameters = {}
        self.parameters['swing_scale'] = 0.5
        self.parameters['vx_scale'] = 0.5
        self.parameters['vy_scale'] = 0.5
        self.parameters['vt_scale'] = 0.4

        # setup motion for each joint
        # f1=THIGH1=ANKLE1=L=R in phase
        self.pfn = {}  # phase joint functions dictionary
        self.afn = {}  # anti phase joint functions dictionary

        thigh_joint_motion = LegJointMotionOverTime() #joints moves periodically
        thigh_joint_motion.scale = -1

        tibia_joint_motion = LegJointMotionOverTime()
        tibia_joint_motion.scale = -0.5

        zero_output_function = LegJointMotionOverTime()
        zero_output_function.scale = 0

        self.set_motion_function('j_thigh', thigh_joint_motion, zero_output_function)
        self.set_motion_function('j_tibia', tibia_joint_motion, zero_output_function)
        self.set_motion_function('j_c1', zero_output_function, zero_output_function)

        self.phase = True
        self.tick = 0
        self.loopTIck = 50

        self.target_velocity = [0,0]

        self.joint_angle_dict = self.get_joint_angle_dict_at_tick(True,0,self.target_velocity)

        

        #Setup Original Joint Configuration

    def set_motion_function(self, joint, fp, fa):
        for leg in ['lf', 'rm', 'lr']:
            j = joint + '_' + leg
            self.pfn[j] = fp
            self.afn[j] = fa

        for leg in ['rf', 'lm', 'rr']:
            j = joint + '_' + leg
            self.pfn[j] = fa
            self.afn[j] = fp

    def get_joint_angle_dict_at_tick(self, phase, loop_time_tick, velocity):
        """loop_time_tick should be a float between 0 and 1"""
        angles_set_dict = {}
        for j in self.pfn.keys():
            if phase:
                angles_set_dict[j] = self.pfn[j].getAngleAtTick(loop_time_tick)
            else:
                angles_set_dict[j] = self.afn[j].getAngleAtTick(loop_time_tick)
        # VXd
        Vx = velocity[0] * self.parameters['vx_scale']
        d = (loop_time_tick * 2 - 1) * Vx
        if phase:
            angles_set_dict['j_c1_lf'] -= d
            angles_set_dict['j_c1_rm'] += d
            angles_set_dict['j_c1_lr'] -= d
            angles_set_dict['j_c1_rf'] -= d
            angles_set_dict['j_c1_lm'] += d
            angles_set_dict['j_c1_rr'] -= d
        else:
            angles_set_dict['j_c1_lf'] += d
            angles_set_dict['j_c1_rm'] -= d
            angles_set_dict['j_c1_lr'] += d
            angles_set_dict['j_c1_rf'] += d
            angles_set_dict['j_c1_lm'] -= d
            angles_set_dict['j_c1_rr'] += d
        # Vw
        Vw = velocity[1] * self.parameters['vt_scale']
        d = (loop_time_tick * 2 - 1) * Vw
        if phase:
            angles_set_dict['j_c1_lf'] += d
            angles_set_dict['j_c1_rm'] += d
            angles_set_dict['j_c1_lr'] += d
            angles_set_dict['j_c1_rf'] -= d
            angles_set_dict['j_c1_lm'] -= d
            angles_set_dict['j_c1_rr'] -= d
        else:
            angles_set_dict['j_c1_lf'] -= d
            angles_set_dict['j_c1_rm'] -= d
            angles_set_dict['j_c1_lr'] -= d
            angles_set_dict['j_c1_rf'] += d
            angles_set_dict['j_c1_lm'] += d
            angles_set_dict['j_c1_rr'] += d
        return angles_set_dict

    def update_plan(self):
        x = float(self.tick)/self.loopTIck
        self.joint_angle_dict = self.get_joint_angle_dict_at_tick(self.phase,x,self.target_velocity)
        # print(self.joint_angle_dict)
        self.tick += 1
        if self.tick >= self.loopTIck:
            self.tick = 0
            self.phase = not self.phase
        pass
    
    def get_plan_result(self):
        return self.joint_angle_dict
    
    def set_target_velocity(self,target_velocity):
        self.target_velocity = target_velocity