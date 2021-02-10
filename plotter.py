#!/usr/bin/env python3

import sys
from math import pi
from matplotlib import pyplot as plt
import numpy as np


MATPLOTLIB_FIGSIZE = (2*6.4, 2*4.8)


class SpacecraftData:
    def __init__(self):
        self.tgo = []

        # ctr

        self.eng_throttle = []
        self.mass = []

        self.eng_gimbal = []

        # acc

        self.acc_thrust = []
        self.acc_atm = []
        self.acc_gravity = []
        self.acc_centrifugal = []

        # nav - trans

        self.acc_x = []
        self.acc_y = []

        self.vel_x = []
        self.vel_y = []

        self.pos_x = []
        self.pos_y = []

        # nav - ang

        self.ang_acc = []
        self.ang_vel = []
        self.ang_pos = []


def rad2deg(rads):
    return [r*180/pi for r in rads]


def plt_align_yaxis(axes):
    # From https://stackoverflow.com/questions/10481990/matplotlib-axis-with-two-scales-shared-origin

    y_lims = np.array([ax.get_ylim() for ax in axes])

    # force 0 to appear on all axes, comment if don't need
    y_lims[:, 0] = y_lims[:, 0].clip(None, 0)
    y_lims[:, 1] = y_lims[:, 1].clip(0, None)

    # normalize all axes
    y_mags = (y_lims[:,1] - y_lims[:,0]).reshape(len(y_lims),1)
    y_lims_normalized = y_lims / y_mags

    # find combined range
    y_new_lims_normalized = np.array([np.min(y_lims_normalized), np.max(y_lims_normalized)])

    # denormalize combined range to get new axes
    new_lims = y_new_lims_normalized * y_mags
    for i, ax in enumerate(axes):
        ax.set_ylim(new_lims[i])


def subplot_plot_single_axis(plot_rows, plot_cols, plot_id, xs, plot_spec):
    """
        plot_spec example:
            plot_spec = [
                (label1, (data1a, data1b)),
                (label2, (data2a, data2b)),
            ]
    """

    for _label, (curve_a, curve_b) in plot_spec:
        assert len(xs) == len(curve_a)
        assert len(xs) == len(curve_b)

    # set ax
    ax = plt.subplot(plot_rows, plot_cols, plot_id)

    # plot
    handles = []
    for i, (_plot_label, (curve_a, curve_b)) in enumerate(plot_spec):
        (handle, ) = ax.plot(xs, curve_a)
        ax.plot(xs, curve_b, color=handle.get_color())  # TODO darken
        handles.append(handle)

    # legend
    plt.legend(handles, [tup[0] for tup in plot_spec])

    # set xlim
    plt.xlim((xs[0], xs[-1]))


def subplot_plot_twin_axis(plot_rows, plot_cols, plot_id, xs, plot_spec, align_yaxis=False):
    """
        plot_spec example:
            plot_spec = [
                (ylabel1, (data1a, data1b), horiz),
                (ylabel2, (data2a, data2b), horiz),
            ]
    """

    assert len(plot_spec) in [1, 2]

    for _label, curves, _horiz in plot_spec:
        for curve in curves:
            assert len(xs) == len(curve)

    COLORS_YLABEL = ['blue', 'red']
    COLORS_DATA = [['blue', 'darkblue'], ['red', 'darkred']]
    COLORS_HLINES = ['lightsteelblue', 'lightcoral']

    ax = None
    axis = []

    for i, (plot_ylabel, curves, plot_horiz) in enumerate(plot_spec):
        # set ax
        if ax is None:
            ax = plt.subplot(plot_rows, plot_cols, plot_id)
        else:
            ax = ax.twinx()

        # set label
        ax.set_ylabel(plot_ylabel)
        ax.yaxis.label.set_color(COLORS_YLABEL[i])

        # plot
        ax.tick_params(axis='y', labelcolor=COLORS_YLABEL[i])
        for j, curve in enumerate(curves):
            ax.plot(xs, curve, color=COLORS_DATA[i][j])

        # set horiz line
        if plot_horiz is not None:
            ax.hlines(plot_horiz, xs[0], xs[-1], color=COLORS_HLINES[i])

        axis.append(ax)

    if align_yaxis:
        plt_align_yaxis(axis)

    # set xlim
    plt.xlim((xs[0], xs[-1]))


def plot_all(sc_data, sim_data, save_to_file=None):
    xs = [sc_data.tgo[0]-t for t in sc_data.tgo]

    #
    # Figure 1 - ctr and ang
    #

    plt.figure(figsize=MATPLOTLIB_FIGSIZE)

    subplot_plot_twin_axis(
        3, 1, 1, xs, [
            ('eng_throttle (0-1)', (sc_data.eng_throttle, sim_data.eng_throttle), None),
            ('mass (kg)', (sc_data.mass, sim_data.mass), None),
        ]
    )

    subplot_plot_twin_axis(
        3, 1, 2, xs, [
            ('gimbal pos (deg)', (rad2deg(sc_data.eng_gimbal), rad2deg(sim_data.eng_gimbal)), 0),
            ('gimbal vel (deg/sec)', (rad2deg(sc_data.eng_gimbal_vel), rad2deg(sim_data.eng_gimbal_vel)), 0),
        ],
        align_yaxis=True,
    )

    subplot_plot_twin_axis(
        3, 1, 3, xs, [
            ('ang vel (deg/sec)', (rad2deg(sc_data.ang_vel), rad2deg(sim_data.ang_vel)), 0),
            ('ang pos (deg)', (rad2deg(sc_data.ang_pos), rad2deg(sim_data.ang_pos)), 90),
        ],
    )

    # save
    plt.tight_layout()
    print('Saving plot to output_ctr_ang.png')
    plt.savefig('output_ctr_ang.png')

    #
    # Figure 2 - nav
    #

    plt.figure(figsize=MATPLOTLIB_FIGSIZE)

    subplot_plot_single_axis(
        4, 1, 1, xs, [
            ('acc_thrust (m/s**2)', (sc_data.acc_thrust, sim_data.acc_thrust)),
            ('acc_atm (m/s**2)', (sc_data.acc_atm, sim_data.acc_atm)),
            ('acc_gravity (m/s**2)', (sc_data.acc_gravity, sim_data.acc_gravity)),
            ('acc_centrifugal (m/s**2)', (sc_data.acc_centrifugal, sim_data.acc_centrifugal)),
        ],
    )

    subplot_plot_twin_axis(
        4, 1, 2, xs, [
            ('acc x (m/sec**2)', (sc_data.acc_x, sim_data.acc_x), 0),
            ('acc y (m/sec**2)', (sc_data.acc_y, sim_data.acc_y), 0),
        ],
        align_yaxis=True,
    )

    subplot_plot_twin_axis(
        4, 1, 3, xs, [
            ('vel x (m/sec)', (sc_data.vel_x, sim_data.vel_x), 0),
            ('vel y (m/sec)', (sc_data.vel_y, sim_data.vel_y), 0),
        ],
    )

    subplot_plot_twin_axis(
        4, 1, 4, xs, [
            ('pos x (m)', (sc_data.pos_x, sim_data.pos_x), 0),
            ('pos y (m)', (sc_data.pos_y, sim_data.pos_y), 0),
        ],
    )

    # save
    plt.tight_layout()
    print('Saving plot to output_nav.png')
    plt.savefig('output_nav.png')


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
            sc_data.acc_thrust.append(vals[4])
            sc_data.acc_atm.append(vals[5])
            sc_data.acc_gravity.append(vals[6])
            sc_data.acc_centrifugal.append(vals[7])
            sc_data.acc_x.append(vals[8])
            sc_data.acc_y.append(vals[9])
            sc_data.vel_x.append(vals[10])
            sc_data.vel_y.append(vals[11])
            sc_data.pos_x.append(vals[12])
            sc_data.pos_y.append(vals[13])
            sc_data.ang_acc.append(vals[14])
            sc_data.ang_vel.append(vals[15])
            sc_data.ang_pos.append(vals[16])
        elif line.startswith('CSV SIM;'):
            vals = [float(f) for f in line.strip().split(';')[1:] if f]
            sim_data.tgo.append(vals[0])
            sim_data.eng_throttle.append(vals[1])
            sim_data.mass.append(vals[2])
            sim_data.eng_gimbal.append(vals[3])
            sim_data.acc_thrust.append(vals[4])
            sim_data.acc_atm.append(vals[5])
            sim_data.acc_gravity.append(vals[6])
            sim_data.acc_centrifugal.append(vals[7])
            sim_data.acc_x.append(vals[8])
            sim_data.acc_y.append(vals[9])
            sim_data.vel_x.append(vals[10])
            sim_data.vel_y.append(vals[11])
            sim_data.pos_x.append(vals[12])
            sim_data.pos_y.append(vals[13])
            sim_data.ang_acc.append(vals[14])
            sim_data.ang_vel.append(vals[15])
            sim_data.ang_pos.append(vals[16])
        else:
            continue

    sc_data.eng_gimbal_vel = [0] + list(np.diff(sc_data.eng_gimbal))
    sim_data.eng_gimbal_vel = [0] + list(np.diff(sim_data.eng_gimbal))

    print('Read and parsed all data, plotting - len=%d' % len(sc_data.tgo))

    plot_all(sc_data, sim_data)

    print('Plotted all data, showing')
    plt.show()


if __name__ == '__main__':
    main()
