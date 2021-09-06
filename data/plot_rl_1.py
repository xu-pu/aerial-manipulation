from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.ticker import MaxNLocator
import numpy as np

if __name__ == '__main__':

    # background_color = 'blue'
    # foreground_color = 'red'

    background_color = 'white'
    foreground_color = 'black'
    plot_color_1 = 'tab:blue'
    plot_color_2 = 'tab:orange'

    data = genfromtxt('2021-09-01-18-10-51.labfloor.01.bag.full.csv', delimiter=',')

    # change size and resolution
    plt.rcParams["figure.figsize"] = (12,6)
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

    fig.tight_layout(pad=1)

    rad2deg = 180./np.pi

    t_end = data[-1,0]

    action_scale = 0.1

    t1 = 30
    duration = 60
    t2 = t1+duration

    # action_x
    axs[0].plot(data[:, 0], data[:, 11], color=plot_color_2)
    axs[0].plot(data[:, 0], action_scale * data[:, 9], color=plot_color_1)
    axs[0].set_xlim((t1, t2))
    axs[0].set_ylim((-0.25,0.25))
    axs[0].set_yticks([-0.2,0,0.2])
    axs[0].xaxis.set_ticklabels([])
    # action_y
    axs[1].plot(data[:, 0], data[:, 12], color=plot_color_2)
    axs[1].plot(data[:, 0], action_scale * data[:, 10], color=plot_color_1)
    axs[1].set_xlim((t1, t2))
    axs[1].set_ylim((-0.25, 0.25))
    axs[1].set_yticks([-0.2,0,0.2])
    #axs[3].xaxis.set_ticklabels([])

    # for ax in axs:
    #     #ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    #     ax.xaxis.set_ticklabels([])

    plt.savefig("test.png")

    #plt.show()
