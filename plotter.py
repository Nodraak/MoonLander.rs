#!/usr/bin/env python3

import json
import re
import sys
from math import pi
from matplotlib import pyplot as plt
import numpy as np


MATPLOTLIB_FIGSIZE = (2*6.4, 2*4.8)


class SpacecraftData:
    def __init__(self):
        self.conf = None
        self.cur = None
        self.processed = None

    def add_conf(self, conf):
        assert self.conf is None
        self.conf = conf

    def add_cur(self, cur):
        if self.cur is None:
            self.cur = {}

        for k1, v1 in cur.items():
            if isinstance(v1, dict):
                for k2, v2 in v1.items():
                    k = '%s_%s' % (k1, k2)
                    v = v2

                    if k not in self.cur:
                        self.cur[k] = []
                    self.cur[k].append(v)
            else:
                if k1 not in self.cur:
                    self.cur[k1] = []
                self.cur[k1].append(v1)

    def process(self):
        self.processed = {}

        self.processed['mass'] = [
            self.conf['sc_dry_mass']+fuel for fuel in self.cur['fuel_mass']
        ]
        self.processed['eng_gimbal'] = [
            self.conf['ctr_eng_gimbal_pos_max']*eng_gimbal for eng_gimbal in self.cur['eng_gimbal']
        ]

        self.processed['eng_gimbal_vel'] = [0] + list(np.diff(self.processed['eng_gimbal']))


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
            assert len(xs) == len(curve), \
                "For curve %s , expected len(xs) == len(curve) but %d != %d" % (_label, len(xs), len(curve))

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
    xs = sc_data.cur['t']

    #
    # Figure 1 - ctr and ang
    #

    plt.figure(figsize=MATPLOTLIB_FIGSIZE)

    subplot_plot_twin_axis(
        3, 1, 1, xs, [
            ('eng_throttle (0-1)', (sc_data.cur['eng_throttle'], sim_data.cur['eng_throttle']), None),
            ('mass (kg)', (sc_data.processed['mass'], sim_data.processed['mass']), None),
        ]
    )

    subplot_plot_twin_axis(
        3, 1, 2, xs, [
            ('gimbal pos (deg)', (rad2deg(sc_data.cur['eng_gimbal']), rad2deg(sim_data.cur['eng_gimbal'])), 0),
            ('gimbal vel (deg/sec)', (rad2deg(sc_data.processed['eng_gimbal_vel']), rad2deg(sim_data.processed['eng_gimbal_vel'])), 0),
        ],
        align_yaxis=True,
    )

    subplot_plot_twin_axis(
        3, 1, 3, xs, [
            ('ang vel (deg/sec)', (rad2deg(sc_data.cur['ang_vel']), rad2deg(sim_data.cur['ang_vel'])), 0),
            ('ang pos (deg)', (rad2deg(sc_data.cur['ang_pos']), rad2deg(sim_data.cur['ang_pos'])), 90),
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
            ('acc_thrust (m/s**2)', (sc_data.cur['acc_thrust'], sim_data.cur['acc_thrust'])),
            ('acc_atm (m/s**2)', (sc_data.cur['acc_atm'], sim_data.cur['acc_atm'])),
            ('acc_gravity (m/s**2)', (sc_data.cur['acc_gravity'], sim_data.cur['acc_gravity'])),
            ('acc_centrifugal (m/s**2)', (sc_data.cur['acc_centrifugal'], sim_data.cur['acc_centrifugal'])),
        ],
    )

    subplot_plot_twin_axis(
        4, 1, 2, xs, [
            ('acc x (m/sec**2)', (sc_data.cur['acc_x'], sim_data.cur['acc_x']), 0),
            ('acc y (m/sec**2)', (sc_data.cur['acc_y'], sim_data.cur['acc_y']), 0),
        ],
        align_yaxis=True,
    )

    subplot_plot_twin_axis(
        4, 1, 3, xs, [
            ('vel x (m/sec)', (sc_data.cur['vel_x'], sim_data.cur['vel_x']), 0),
            ('vel y (m/sec)', (sc_data.cur['vel_y'], sim_data.cur['vel_y']), 0),
        ],
    )

    subplot_plot_twin_axis(
        4, 1, 4, xs, [
            ('pos x (m)', (sc_data.cur['pos_x'], sim_data.cur['pos_x']), 0),
            ('pos y (m)', (sc_data.cur['pos_y'], sim_data.cur['pos_y']), 0),
        ],
    )

    # save
    plt.tight_layout()
    print('Saving plot to output_nav.png')
    plt.savefig('output_nav.png')


def main():
    sc_data = SpacecraftData()
    sim_data = SpacecraftData()

    print('Reading CSV data from stdin')

    for line in sys.stdin:
        line = line.strip()

        ret = re.findall(r'^\[LOGD:(.+)::(.+)\] CSV=(\{.+\})$', line)
        if ret:
            obj, func, data = ret[0]

            if obj == 'Spacecraft':
                obj = sc_data
            elif obj == 'Sim':
                obj = sim_data
            else:
                raise Exception

            if func == 'export_to_csv_cur':
                func = 'add_cur'
            elif func == 'export_to_csv_conf':
                func = 'add_conf'
            else:
                raise Exception

            getattr(obj, func)(json.loads(data))

    sc_data.process()
    sim_data.process()

    print('Read and parsed all data, plotting - len=%d' % len(sc_data.cur['t']))

    plot_all(sc_data, sim_data)

    print('Plotted all data, showing')
    plt.show()


if __name__ == '__main__':
    main()
