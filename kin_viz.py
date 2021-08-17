#!/usr/bin/env python3

import argparse
import pandas as pd
import numpy as np
from numpy import linspace
import matplotlib as mpl
from matplotlib import cm
from matplotlib import pyplot as plt
from scipy import stats
from scipy.signal import butter, lfilter, freqz



def main():

    data = pd.read_csv("kin_test.csv")
    num_rows = data.shape[0]

    # import ipdb; ipdb.set_trace();

    print(data)

    # print(kin.fk_vec(-0.15*np.pi/3, -1.05*np.pi/3))

    # print(np.array([thetfrom numpy import linspacea1, theta2]))
    x0 = 0
    y0 = 0

    fig, ax = plt.subplots()

    start = 0.0
    stop = 1.0
    cm_subsection = linspace(start, stop, num_rows) 
    colors = [ cm.viridis(x) for x in cm_subsection ]

    headers = ["p0", "p1","p21", "p22", "p31", "p32"]
    for ii in range(num_rows):
        y_pts = []
        z_pts = []
        for jj in range(len(headers)) :
            y_pts.append(data[headers[jj]+"_y"][ii])
            z_pts.append(data[headers[jj]+"_z"][ii])

        theta1 = data["femur"][ii]
        theta2 = data["tibia"][ii]

        ax.plot(y_pts, z_pts, 'o-',color=colors[ii])
        # ax.plot(link_pts[:,0], link_pts[:,1], 'ro-')
        # ax.plot(link_pts2[:,0], link_pts2[:,1], 'go-')
        plt.title('theta={}, foot={}'.format(\
            [round(e,2) for e in [theta1,theta2]],\
            [round(e,2) for e in [y_pts[-1], z_pts[-1]]]))

    ax.set_aspect(1)

    plt.show()

if __name__ == "__main__" :
    main()