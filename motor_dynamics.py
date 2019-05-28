#!/usr/bin/env python
import math

def dynamics(self):
    # Motor Parameters
    Lq = self.params['electrical']['coil_inductance']
    R = self.params['electrical']['coil_resistance']
    Vb = self.params['electrical']['input_voltage']
    Kw = self.params['electrical']['bemf_constant']
    phi = 2 * math.pi * self.params['electrical']['commutation_phase_diff'] / 360
    Bm = self.params['mechanical']['damping_ratio']
    J = self.params['mechanical']['rotational_inertia']
    dt = self.params['dynamics_timestep']
    Tl = self.params['mechanical']['load_torque']

    # Electrical angle
    theta_e = self.params['electrical']['num_pole_pairs'] * self.state['angular_position']

    # Back EMF Calculation (assuming sinusoidal commutation)
    bemf_a = Kw * self.state['angular_velocity'] * math.sin( theta_e )
    bemf_b = Kw * self.state['angular_velocity'] * math.sin( theta_e - phi )
    bemf_c = Kw * self.state['angular_velocity'] * math.sin( theta_e + phi )

    # Phase currents derivatives
    dot_ia = (1/Lq) * (self.inputs['phase_volt_a'] - (R * self.state['phase_curr_a']) - bemf_a)
    dot_ib = (1/Lq) * (self.inputs['phase_volt_b'] - (R * self.state['phase_curr_b']) - bemf_b)
    dot_ic = (1/Lq) * (self.inputs['phase_volt_c'] - (R * self.state['phase_curr_c']) - bemf_c)

    # Step dynamics
    ia = self.state['phase_curr_a'] + (dot_ia * dt) 
    ib = self.state['phase_curr_b'] + (dot_ib * dt)
    ic = self.state['phase_curr_c'] + (dot_ic * dt)

    # Torque per phase (accounting for null angular velocity)
    Ta = Kw * math.sin( theta_e ) * self.state['phase_curr_a']
    Tb = Kw * math.sin( theta_e - phi ) * self.state['phase_curr_b']
    Tc = Kw * math.sin( theta_e + phi ) * self.state['phase_curr_c']

    # Electrical Torque
    Te = Ta + Tb + Tc

    # Mechanical Torque
    Tm = self.params['electrical']['num_pole_pairs'] * Te - Bm * self.state['angular_velocity'] - Tl

    # Friction calculation
    if abs(self.state['angular_velocity']) < self.params['mechanical']['min_velocity']:
        if (abs(Tm) < self.params['mechanical']['static_friction']):
            Tm = 0
        else:
            Tm = Tm - self.params['mechanical']['static_friction']
    else:
        Tm = Tm - math.copysign(1, self.state['angular_velocity']) * ( self.params['mechanical']['static_friction'] * math.exp( -5 * abs(self.state['angular_velocity'])) + self.params['mechanical']['stribeck_friction'] )

    # Angular acceleration
    dot_omega = Tm / J

    # Update angular velocity
    omega  = self.state['angular_velocity'] + dot_omega * dt

    # Update angular position
    theta = self.state['angular_position'] + self.state['angular_velocity'] * dt

    self.state = {'phase_curr_a': ia,
                  'phase_curr_b': ib,
                  'phase_curr_c': ic,
                  'angular_position': omega,
                  'angular_velocity': theta}
    return [dot_ia,
            dot_ib,
            dot_ic,
            dot_omega,
            omega]
