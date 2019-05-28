#!/usr/bin/env python3

import numpy as np
import argparse
import yaml
import motor_dynamics as dyn
import math
import state_plotter
import copy

class BLDC_Simulator(object):

    dynamics = dyn.dynamics
    
    def __init__ (self):
        # Initial Conditions
        self.state = {'phase_curr_a': 0.0,
                      'phase_curr_b': 0.0,
                      'phase_curr_c': 0.0,
                      'angular_position': 0.0,
                      'angular_velocity': 0.0}
        self.inputs = {'phase_volt_a': 0.0,
                      'phase_volt_b': 0.0,
                      'phase_volt_c': 0.0}    
        self.history = []
        self.history.append({'state': self.state, 'inputs': self.inputs})
        self.filename_ = "motor_params.yaml" # motor parameters
        self.loadMotorParams()

    def loadMotorParams(self):
        """
            Load motor parameters from file
        """
        with open(self.filename_, 'r') as stream:
            try:
                self.params = yaml.safe_load(stream)
                # Display the motor parameters
                print('\n')
                print ('{:_^30}'.format('Motor Parameters'))
                print('\n')
                print(yaml.dump(self.params))
            except yaml.YAMLError as exc:
                print(exc)

    def commutation_function(self):
        """
        Commutation method that determines the shape of the input phase voltages
        """
        theta_e = (self.state['angular_position'] * self.params['electrical']['num_pole_pairs'] * 360 / (2*math.pi)) % 360
        #phi = 2 * math.pi * self.params['electrical']['commutation_phase_diff'] / 360
        self.inputs = {'phase_volt_a': np.piecewise(theta_e, [0 <= theta_e < 30,
                                                              30 <= theta_e < 90,
                                                              90 <= theta_e < 150,
                                                              150 <= theta_e < 210,
                                                              210 <= theta_e < 270,
                                                              270 <= theta_e < 330,
                                                              330 <= theta_e < 360],
                                                              [0,
                                                               self.params['electrical']['input_voltage'],
                                                               self.params['electrical']['input_voltage'],
                                                               0,
                                                               -self.params['electrical']['input_voltage'],
                                                               -self.params['electrical']['input_voltage'],
                                                               0]),
                       'phase_volt_b': np.piecewise(theta_e, [0 <= theta_e < 30,
                                                              30 <= theta_e < 90,
                                                              90 <= theta_e < 150,
                                                              150 <= theta_e < 210,
                                                              210 <= theta_e < 270,
                                                              270 <= theta_e < 330,
                                                              330 <= theta_e < 360],
                                                              [-self.params['electrical']['input_voltage'],
                                                               -self.params['electrical']['input_voltage'],
                                                               0,
                                                               self.params['electrical']['input_voltage'],
                                                               self.params['electrical']['input_voltage'],
                                                               0,
                                                               -self.params['electrical']['input_voltage']]),
                       'phase_volt_c': np.piecewise(theta_e, [0 <= theta_e < 30,
                                                              30 <= theta_e < 90,
                                                              90 <= theta_e < 150,
                                                              150 <= theta_e < 210,
                                                              210 <= theta_e < 270,
                                                              270 <= theta_e < 330,
                                                              330 <= theta_e < 360],
                                                              [self.params['electrical']['input_voltage'],
                                                               0,
                                                              -self.params['electrical']['input_voltage'],
                                                              -self.params['electrical']['input_voltage'],
                                                               0,
                                                               self.params['electrical']['input_voltage'],
                                                               self.params['electrical']['input_voltage']])}
        print(self.inputs)


    def step_simulation(self):
        """
        Step through dynamic simulation
        """
        self.commutation_function()
        self.dynamics()
        self.history.append(copy.deepcopy({'state': self.state, 'inputs': self.inputs}))


if __name__ == '__main__':

    # Create a BLDC motor object
    bldc_ = BLDC_Simulator()

    # Simulate the BLDC motor
    for t in range(0,1000):
        bldc_.step_simulation()

    #print([elem['state']['angular_position'] for elem in bldc_.history])
    # Plot the history of state and inputs
    state_plotter.graph(bldc_)
