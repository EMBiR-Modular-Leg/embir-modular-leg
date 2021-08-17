#!/usr/bin/env python3

import argparse
import csv
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from scipy import stats
from scipy.signal import butter, lfilter, freqz

from utils import *

from kinematics import Kinematics

def main() :
    parser = argparse.ArgumentParser(description='Plots data from csv')
    parser.add_argument("filename",\
        help="filename of csv", type=str)
    parser.add_argument("-i", "--interactive",\
        help="interactive mode: decide which data to plot of what is present in csv",
        action='store_true')
    parser.add_argument("--minicheetah",\
        help="for looking at mini cheetah replay data",
        action='store_true')
    parser.add_argument("--kinematics",\
        help="do forward kinematics",
        action='store_true')
    parser.add_argument("--crouch",\
        help="do forward kinematics",
        type=float)
    parser.add_argument("--extract-torques",\
        help="write time, a1 torque, and a2 torque data to csv specified by this argument",
        type=str)
    parser.add_argument("--delay",\
        help="do forward kinematics",
        type=float)
    parser.add_argument("-o", "--outlier",\
        help="outlier rejection: ignore rows in the csv for which brake torque data are in the <arg> extremes of the data: -o 0.05 will drop the top and bottom 5%.",
        type=float)
    parser.add_argument("-g", "--gear",\
        help="specify gear ratio of actuator test sample",
        type=float)
    parser.add_argument("-a", "--averaging",\
        help="specify number of samples to use for running average filtering",
        type=int)

    args = parser.parse_args()
    
    data = pd.read_csv(args.filename, comment='#', header=0, float_precision='high')
    kin = Kinematics(l3_pll_in=18, l3_perp_in=135)

    num_rows = len(data.index)

    time = data["time [s]"]
    # import ipdb; ipdb.set_trace()
    dt = np.abs(np.array(time[1:-1]) - np.array(time[0:-2]))
    # for e in np.array(time)[0:20]:
    #     print('{:.5f}'.format(e))
    Ts = np.abs(np.median(dt))

    fs = 1/Ts
    cutoff = 0.02*fs
    order = 6
    b, a = butter_lowpass(cutoff, fs, order)

    if args.extract_torques is not None :
        a1_trq = data["a1 torque [Nm]"].astype(float).fillna(method="pad").fillna(method="bfill")
        a2_trq = data["a2 torque [Nm]"].astype(float).fillna(method="pad").fillna(method="bfill")
        with open(args.extract_torques, "w+") as my_csv:
            csvwriter = csv.writer(my_csv, delimiter=',')
            csvwriter.writerow(["# " +args.filename])
            csvwriter.writerow(["time [s]", "a1 torque [Nm]", "a2 torque [Nm]"])
            output_data = np.transpose(np.array([time, a1_trq, a2_trq]))
            for row in output_data :
                csvwriter.writerow(['{:-.6f}'.format(val) for val in row])

    if args.kinematics :
        cmd_fk = kin.fk_vec(data["a1 position cmd [rad]"].astype(float), data["a2 position cmd [rad]"].astype(float))
        res_fk = kin.fk_vec(data["a1 position [rad]"].astype(float), data["a2 position [rad]"].astype(float))
        
        data["cmd y [m]"] = cmd_fk[-1,-2,:]/1000
        data["cmd z [m]"] = cmd_fk[-1,-1,:]/1000

        data["res y [m]"] = res_fk[-1,-2,:]/1000
        data["res z [m]"] = res_fk[-1,-1,:]/1000

        fig, ax = plt.subplots()

        start = 0.0
        stop = 1.0
        cm_subsection = linspace(start, stop, num_rows) 
        colors = [ cm.viridis(x) for x in cm_subsection ]

        cc = 0
        for jj in range(20,num_rows, num_rows // 12) :
            y_pts = res_fk[:,0,jj] + cc*200
            z_pts = res_fk[:,1,jj] - res_fk[-1,1,jj]
            ax.plot(y_pts, z_pts, 'o-',color=colors[jj])
            cc += 1
        
        ax.plot(np.linspace(0, (cc*200), num_rows), -1000*data["res z [m]"])
        ax.set_aspect(1)
        plt.show()

    if args.crouch is not None :
        femur_trq = data["a1 torque [Nm]"][time > args.delay].astype(float)
        tibia_trq = data["a2 torque [Nm]"][time > args.delay].astype(float)

        femur_trq_filt = butter_lowpass_filter(femur_trq, cutoff, fs, order)
        tibia_trq_filt = butter_lowpass_filter(tibia_trq, cutoff, fs, order)

        time_periodic = time[time > args.delay] % (1/args.crouch)

        fig = plt.figure()
        ax = fig.gca()
        ax.plot(time_periodic, femur_trq, label="femur_trq")
        ax.plot(time_periodic, tibia_trq, label="tibia_trq")
        ax.plot(time_periodic, femur_trq_filt, label="femur_trq_filt")
        ax.plot(time_periodic, tibia_trq_filt, label="tibia_trq_filt")
        plt.legend()
        plt.xlabel("time [s]")
        plt.ylabel("torque [Nm]")
        plt.show()

    # if args.outlier is not None:
        # import ipdb; ipdb.set_trace()
        # data = data[data['brake torque [Nm]']\
        #     .between(\
        #         data['brake torque [Nm]'].quantile(args.outlier),\
        #         data['brake torque [Nm]'].quantile(1-args.outlier)\
        #         )\
        #     ]

    gear_ratio = 6
    if args.gear is not None:
        gear_ratio = args.gear

    averaging_num_samples = 1
    if args.averaging is not None:
        averaging_num_samples = args.averaging
        # data['brake torque [Nm]'] = np.convolve(data['brake torque [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        # data['motor torque measured [Nm]'] = np.convolve(data['motor torque measured [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        # data['motor torque setpoint [Nm]'] = np.convolve(data['motor torque setpoint [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
    
    # if args.filename[0:5] == "speed":
        # data['efficiency from measurement []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque measured [Nm]'])
        # data['efficiency from setpoint []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque setpoint [Nm]'])
    num_cols = data.shape[1]
    headers = data.columns.values
    
    if args.interactive:
        print("Ts mean= {}, Ts median = {}, fs = {}, sigma = {}, outlier fraction = {}".format(\
            round(np.mean(dt), 5), round(Ts, 5), round(1/Ts, 2), round(np.std(dt), 5), sum(dt > 1.3*Ts)/len(dt)))
        print('the following data series are available:')
        for i in range(num_cols) :
            print('\t{}:\t'.format(i) + headers[i])
        
        # fig = plt.figure()
        # ax = fig.gca()
        # # ax.hist(dt, bins = 1000)
        # plt.plot(dt)
        # plt.show()

        # import ipdb; ipdb.set_trace()
        # print(Ts)
        # return
        print('enter which data you would like to plot by their indices separated by pipes \'|\'')
        print('the first index will be taken as the x-axis and the rest will be plotted as separate series')
        if args.minicheetah :
            fig = plt.figure()
            ax = fig.gca()
            # ax.plot(data["velocity [rad/s]"])
            time = np.array(data["time [s]"])
            v1 = np.array(data["a1 velocity [rad/s]"])
            v2 = np.array(data["a2 velocity [rad/s]"])
            ax.plot(time, v1)
            ax.plot(time, -v2)
            p1 = np.array(data["a1 position [rad]"])
            p2 = np.array(data["a2 position [rad]"])
            fig = plt.figure()
            ax = fig.gca()
            p2 = -p2
            ax.plot(time, p1-(p2 - (p2[0]-p1[0])))
            # ax.plot(time, -p2)
            plt.show()
        while True:
            options = input('\t> ')
            if options[0] == 'q':
                return
            plot_strings = options.split(';')
            for plot_str in plot_strings:
                fig = plt.figure()
                ax = fig.gca()
                indices = plot_str.split('|')
                
                xidx=None
                filt_x = False

                xstr = indices[0]
                if "filter" in xstr:
                    filt_x = True
                    xstr = xstr.replace("filter",'')
                if "f" in xstr:
                    filt_x = True
                    xstr = xstr.replace("f",'')
                if not xstr.isdigit():
                    print("invalid input")
                    return
                xidx = int(xstr)
                xlabel = headers[xidx]
                xseries = data[xlabel] if not filt_x else butter_lowpass_filter(data[xlabel], cutoff, fs, order)

                for index in indices[1:]:
                    sidx = None
                    filt_s = False
                    sstr = index
                    if "filter" in sstr:
                        filt_s = True
                        sstr = sstr.replace("filter",'')
                    if "f" in sstr:
                        filt_s = True
                        sstr = sstr.replace("f",'')

                    scatter = False
                    if "scatter" in sstr:
                        scatter = True
                        sstr = sstr.replace("scatter",'')
                    if "s" in sstr:
                        scatter = True
                        sstr = sstr.replace("s",'')

                    if not sstr.isdigit():
                        print("invalid input")
                        return
                    sidx = int(sstr)
                    label = headers[sidx]
                    series = data[label].astype(float) if not filt_s else\
                        butter_lowpass_filter(\
                            data[label].astype(float), cutoff, fs, order)
                    
                    ratio = 1
                    # if label.find("torque") > -1\
                    #     and (label.find("c1") > -1\
                    #     or label.find("c2") > -1):

                    #     ratio = gear_ratio
                    if filt_s: label += ", filtered"
                    if scatter: ax.scatter(xseries, ratio*series, label=label, s=1)
                    else: 
                        # import ipdb; ipdb.set_trace()
                        ax.plot(np.array(xseries), ratio*np.array(series), label=label)
                if filt_x: xlabel += ", filtered"
                plt.xlabel(xlabel)
                plt.legend()
                plt.title(args.filename)
                # plt.ylim(-10, 10)
            plt.show()
        return

    # plt.subplot(111)
    # plt.plot(data['time [s]'], data['motor torque measured [Nm]'], label='Motor Torque [Nm]')
    # plt.plot(data['time [s]'], data['brake torque [Nm]'], label='Brake Torque [Nm]')
    # plt.xlabel('time [s]')
    # plt.legend()
    # plt.title(args.filename)


    # brake = data['brake torque [Nm]']
    # motor = data['motor torque measured [Nm]']
    # print('average brake torque = {}'.format(np.mean(brake[abs(brake) < 1.0])))
    # print('average motor torque = {}'.format(np.mean(motor)))

    # plt.show()

if __name__ == '__main__' :
    main()
