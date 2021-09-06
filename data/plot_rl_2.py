from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.ticker import MaxNLocator
import numpy as np
import math

def time_slice(data, t_start, t_end):
    assert t_start < t_end
    i_start = 0
    i_end = -1
    for i in range(data.shape[0]):
        i_start = i
        if data[i, 0] > t_start:
            break
    for i in range(i_start,data.shape[0]):
        i_end = i
        if data[i, 0] > t_end:
            break
    return data[i_start:i_end, :]

if __name__ == '__main__':

    # background_color = 'blue'
    # foreground_color = 'red'

    background_color = 'white'
    foreground_color = 'black'
    plot_color = 'tab:red'
    #plot_color = foreground_color

    t_clip = 5

    data = genfromtxt('2021-09-01-18-10-51.labfloor.01.bag.full.csv', delimiter=',')
    data = time_slice(data,t_clip,math.inf)

    data_foam_mat = genfromtxt('2021-09-02-02-23-49.foam.02.bag.full.csv', delimiter=',')
    data_foam_mat = time_slice(data_foam_mat,t_clip,math.inf)

    # change size and resolution
    plt.rcParams["figure.figsize"] = (12,5)
    plt.rcParams['figure.dpi'] = 300
    plt.rcParams['axes.linewidth'] = 1.5
    plt.rcParams['xtick.color'] = foreground_color
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['xtick.top'] = True
    plt.rcParams['ytick.color'] = foreground_color
    plt.rcParams['ytick.direction'] = 'in'
    plt.rcParams['lines.linewidth'] = 2
    plt.rcParams['lines.color'] = foreground_color
    plt.rcParams['figure.facecolor'] = background_color
    plt.rcParams['axes.facecolor'] = background_color
    plt.rcParams['axes.edgecolor'] = foreground_color
    plt.rcParams['axes.labelcolor'] = foreground_color
    plt.rcParams['axes.labelsize'] = 20
    plt.rcParams['axes.labelpad'] = 20


    # setup subplots
    fig, axs = plt.subplots(2, 1)
    fig.tight_layout()

    fig.tight_layout(pad=1)

    rad2deg = 180./np.pi

    t_end = data[-1,0]

    # contact
    axs[0].plot(data[:, 8]+2.8, data[:, 7]-0.05, color=plot_color)
    axs[0].xaxis.set_ticklabels([])
    axs[0].set_yticks([-0.2,0,0.2])
    #axs[0].set_xlim((-2.8, 0.7))
    axs[0].set_xlim((0, 3.5))
    axs[0].set_ylim((-0.3, 0.3))
    axs[0].set_aspect('equal',adjustable='box')
    #axs[0].xaxis.tick_top()

    axs[1].plot(data_foam_mat[:, 8]+2.2, data_foam_mat[:, 7]-0.25, color=plot_color)
    # axs[1].set_xticks([-2,-1,0,1,2])
    # axs[1].set_yticks([])
    axs[1].set_xlim((0, 3.5))
    axs[1].set_ylim((-0.3, 0.3))
    axs[1].set_yticks([-0.2,0,0.2])
    axs[1].set_aspect('equal',adjustable='box')
    # axs[1].xaxis.tick_top()

    plt.savefig("rl_fig_2.png")
