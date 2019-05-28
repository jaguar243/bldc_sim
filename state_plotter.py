#!/usr/bin/env python

from bldc import BLDC_Simulator
import matplotlib.pyplot as plt
import math 

def graph(bldc_):
    # create a figure with three subplots
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1,figsize=(8,8))
    left_ = 0.125  # the left side of the subplots of the figure
    right_ = 0.9   # the right side of the subplots of the figure
    bottom_ = 0.1  # the bottom of the subplots of the figure
    top_ = 0.9     # the top of the subplots of the figure
    wspace_ = 0.2  # the amount of width reserved for space between subplots,
                  # expressed as a fraction of the average axis width
    hspace_ = 0.5  # the amount of height reserved for space between subplots,
                  # expressed as a fraction of the average axis height
    plt.subplots_adjust(left=left_, bottom=bottom_, right=right_, top=top_, wspace=wspace_, hspace=hspace_)

    # initialize 3 line plots for each phase in each of the subplots
    # Phase Voltage
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Phase voltage (V)')
    v_a, = ax1.plot([], [], lw=1, color='red')
    v_b, = ax1.plot([], [], lw=1, color='green')
    v_c, = ax1.plot([], [], lw=1, color='blue')

    # Phase Current
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Phase current (A)')
    i_a, = ax2.plot([], [], lw=1, color='red')
    i_b, = ax2.plot([], [], lw=1, color='green')
    i_c, = ax2.plot([], [], lw=1, color='blue')

    # Angular Position
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Position (deg)')
    theta_m, = ax3.plot([], [], lw=1, color='red')

    # Time vector
    timed = [bldc_.params['dynamics_timestep'] * x for x in range(0, len(bldc_.history))]

    # Plot the input phase voltage
    v_a.set_data(timed,
                 [elem['inputs']['phase_volt_a'] for elem in bldc_.history])
    v_b.set_data(timed,
                 [elem['inputs']['phase_volt_b'] for elem in bldc_.history])
    v_c.set_data(timed,
                 [elem['inputs']['phase_volt_c'] for elem in bldc_.history])

    # Plot the phase currents
    i_a.set_data(timed,
                 [elem['state']['phase_curr_a'] for elem in bldc_.history])
    i_b.set_data(timed,
                 [elem['state']['phase_curr_b'] for elem in bldc_.history])
    i_c.set_data(timed,
                 [elem['state']['phase_curr_c'] for elem in bldc_.history])

    # Plot the angular position
    theta_m.set_data(timed,
                 [(360 * elem['state']['angular_position'] / (2*math.pi) % 360) for elem in bldc_.history])

    ax1.relim()
    ax2.relim()
    ax3.relim()
    ax1.autoscale_view()
    ax2.autoscale_view()
    ax3.autoscale_view()
    ax1.grid(True)
    ax2.grid(True)
    ax3.grid(True)
    plt.show()

