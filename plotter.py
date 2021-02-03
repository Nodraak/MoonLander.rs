#!/usr/bin/env python3

import sys
from math import pi
from matplotlib import pyplot as plt
import numpy as np


MATPLOTLIB_FIGSIZE = (2*6.4, 2*4.8)
PLOT_ROWS = 3
PLOT_COLS = 2


class SpacecraftData:
    def __init__(self):
        self.tgo = []

        self.eng_throttle = []
        self.mass = []

        self.eng_gimbal = []
        self.ang_acc = []

        self.ang_vel = []
        self.ang_pos = []

        self.acc_x = []
        self.acc_y = []

        self.vel_x = []
        self.vel_y = []

        self.pos_x = []
        self.pos_y = []


def rad2deg(rads):
    return [r*180/pi for r in rads]


def subplot_plot_quadruple_curves(
    plot_id, ylabel1, ylabel2,
    data1a, data1b, data2a, data2b,
    legend1a, legend1b, legend2a, legend2b,
    last_t, horiz=False,
):
    assert len(data1a) == len(data1b)
    assert len(data1a) == len(data2a)
    assert len(data1a) == len(data2b)

    xs = np.linspace(0, last_t, len(data1a))

    # set ax1 and ax2

    ax1 = plt.subplot(PLOT_ROWS, PLOT_COLS, plot_id)

    ax1.set_ylabel(ylabel1)
    ax1.yaxis.label.set_color('blue')

    ax1.tick_params(axis='y', labelcolor='blue')
    (handle_1a, ) = ax1.plot(xs, data1a, color='blue')
    (handle_1b, ) = ax1.plot(xs, data1b, color='darkblue')

    ax2 = ax1.twinx()

    ax2.set_ylabel(ylabel2)
    ax2.yaxis.label.set_color('red')

    ax2.tick_params(axis='y', labelcolor='red')
    (handle_2a, ) = ax2.plot(xs, data2a, color='red')
    (handle_2b, ) = ax2.plot(xs, data2b, color='darkred')

    # set xlim

    plt.xlim((0, last_t))

    # set horiz line

    if horiz:
        ax1.hlines(0, 0, last_t, color='lightsteelblue')
        ax2.hlines(0, 0, last_t, color='lightcoral')


def plot_all(sc_data, sim_data, save_to_file=None):
    plt.figure(figsize=MATPLOTLIB_FIGSIZE)

    subplot_plot_quadruple_curves(
        1, 'eng_throttle (0-1)', 'mass (kg)',
        sc_data.eng_throttle, sim_data.eng_throttle, sc_data.mass, sim_data.mass,
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
    )

    subplot_plot_quadruple_curves(
        2, 'gimbal pos (deg)', 'gimbal vel (deg/sec)',
        rad2deg(sc_data.eng_gimbal), rad2deg(sim_data.eng_gimbal), rad2deg(sc_data.eng_gimbal_vel), rad2deg(sim_data.eng_gimbal_vel),
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
        horiz=True,
    )

    subplot_plot_quadruple_curves(
        3, 'ang vel (deg/sec)', 'ang pos (deg)',
        rad2deg(sc_data.ang_vel), rad2deg(sim_data.ang_vel), rad2deg(sc_data.ang_pos), rad2deg(sim_data.ang_pos),
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
        horiz=True,
    )

    subplot_plot_quadruple_curves(
        4, 'acc x (m/sec**2)', 'acc y (m/sec**2)',
        sc_data.acc_x, sim_data.acc_x, sc_data.acc_y, sim_data.acc_y,
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
        horiz=True,
    )

    subplot_plot_quadruple_curves(
        5, 'vel x (m/sec)', 'vel y (m/sec)',
        sc_data.vel_x, sim_data.vel_x, sc_data.vel_y, sim_data.vel_y,
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
        horiz=True,
    )

    subplot_plot_quadruple_curves(
        6, 'pos x (m)', 'pos y (m)',
        sc_data.pos_x, sim_data.pos_x, sc_data.pos_y, sim_data.pos_y,
        'sc', 'sim', 'sc', 'sim',
        sc_data.tgo[0],
        horiz=True,
    )

    plt.tight_layout()

    # save and show

    if save_to_file:
        print('Saving plot to %s' % save_to_file)
        plt.savefig(save_to_file)


def main():
    sc_data = SpacecraftData()
    sim_data = SpacecraftData()

    print('Reading CSV data from stdin');

    for line in sys.stdin:
        if line.startswith('CSV SC;'):
            vals = [float(f) for f in line.strip().split(';')[1:] if f]
            sc_data.tgo.append(vals[0])
            sc_data.eng_throttle.append(vals[1])
            sc_data.mass.append(vals[2])
            sc_data.eng_gimbal.append(vals[3])
            sc_data.ang_acc.append(vals[4])
            sc_data.ang_vel.append(vals[5])
            sc_data.ang_pos.append(vals[6])
            sc_data.acc_x.append(vals[7])
            sc_data.acc_y.append(vals[8])
            sc_data.vel_x.append(vals[9])
            sc_data.vel_y.append(vals[10])
            sc_data.pos_x.append(vals[11])
            sc_data.pos_y.append(vals[12])
        elif line.startswith('CSV SIM;'):
            vals = [float(f) for f in line.strip().split(';')[1:] if f]
            sim_data.tgo.append(vals[0])
            sim_data.eng_throttle.append(vals[1])
            sim_data.mass.append(vals[2])
            sim_data.eng_gimbal.append(vals[3])
            sim_data.ang_acc.append(vals[4])
            sim_data.ang_vel.append(vals[5])
            sim_data.ang_pos.append(vals[6])
            sim_data.acc_x.append(vals[7])
            sim_data.acc_y.append(vals[8])
            sim_data.vel_x.append(vals[9])
            sim_data.vel_y.append(vals[10])
            sim_data.pos_x.append(vals[11])
            sim_data.pos_y.append(vals[12])
        else:
            continue

    sc_data.eng_gimbal_vel = [0] + list(np.diff(sc_data.eng_gimbal))
    sim_data.eng_gimbal_vel = [0] + list(np.diff(sim_data.eng_gimbal))

    print('Read and parsed all data, plotting - len=%d' % len(sc_data.tgo))

    plot_all(sc_data, sim_data, save_to_file='./output.png')

    print('Plotted all data, showing')

    plt.show()


if __name__ == '__main__':
    main()
