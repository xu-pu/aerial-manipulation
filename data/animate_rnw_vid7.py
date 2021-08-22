from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.ticker import MaxNLocator
import numpy as np


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

    data_25 = genfromtxt('vid7.n25t40.bag.cont.csv', delimiter=',')
    data_30 = -genfromtxt('vid7.n30t40.bag.cont.csv', delimiter=',')
    data_35 = -genfromtxt('vid7.n35t40.bag.cont.csv', delimiter=',')

    data_30[:,0] *= -1
    data_35[:,0] *= -1

    #data = genfromtxt('2021-07-01-04-24-56.vid2.ground.55.120.bag.full.csv', delimiter=',')

    # change size and resolution
    plt.rcParams["figure.figsize"] = (7,10)
    plt.rcParams['figure.dpi'] = 100
    plt.rcParams['axes.linewidth'] = 0.5
    plt.rcParams['xtick.color'] = foreground_color
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['xtick.top'] = True
    plt.rcParams['ytick.color'] = foreground_color
    plt.rcParams['ytick.left'] = False
    plt.rcParams['ytick.right'] = True
    plt.rcParams['ytick.labelleft'] = True
    plt.rcParams['ytick.labelright'] = False
    plt.rcParams['ytick.direction'] = 'in'
    plt.rcParams['lines.linewidth'] = 1.5
    plt.rcParams['lines.color'] = foreground_color
    plt.rcParams['figure.facecolor'] = background_color
    plt.rcParams['axes.facecolor'] = background_color
    plt.rcParams['axes.edgecolor'] = foreground_color
    plt.rcParams['axes.labelcolor'] = foreground_color
    plt.rcParams['axes.labelsize'] = 20
    plt.rcParams['axes.labelpad'] = 20

    # setup subplots
    fig, axes = plt.subplots(1, 3)
    fig.tight_layout()

    fps = 60
    speed = 8
    interval = (1000/fps)  # ms between frames
    duration_sec = max(data_25[-1, 0],data_30[-1, 0],data_35[-1, 0])
    frames = int(duration_sec * 1000 / interval / speed)
    t_start = 0
    t_window = 10
    t_end = t_start-t_window

    for ax in axes:
        ax.set_aspect('equal')
        ax.xaxis.set_ticklabels([])

    y_range = 5

    axes[0].set_ylim((-2, -2 + y_range))
    axes[1].set_ylim((-1, -1 + y_range))
    axes[2].set_ylim((-2.3, -2.3 + y_range))

    x_range = 0.5

    axes[0].set_xlim(( 0-x_range,0+x_range ))
    axes[1].set_xlim(( -0.2-x_range, -0.2+x_range ))
    axes[2].set_xlim(( 0-x_range, 0+x_range ))

    axes[1].yaxis.set_ticklabels([])
    axes[2].yaxis.set_ticklabels([])

    just_static = True

    if not just_static:
        line25, = axes[0].plot([], [], color=plot_color)
        line30, = axes[1].plot([], [], color=plot_color)
        line35, = axes[2].plot([], [], color=plot_color)

    def init():
        return line25, line30, line35

    def animate(i):
        offset = i*speed*interval/1000.
        start = max(0,offset-20)
        if offset > 0.1:
            for data, line in [(data_25,line25),(data_30,line30),(data_35,line35)]:
                seg = time_slice(data, 0, offset)
                line.set_data(seg[:, 4], seg[:, 5])
        return line25, line30, line35

    def render():
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='Me'), bitrate=1800)
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, blit=False)
        anim.save('vid7.mp4', writer=writer)

    def preview():
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=interval, blit=False)
        plt.show()

    if just_static:
        # ax.plot(data[:, 3], data[:, 6], color=plot_color)
        axes[0].plot(data_25[:, 4], data_25[:, 5],color=plot_color)
        axes[1].plot(data_30[:, 4], data_30[:, 5],color=plot_color)
        axes[2].plot(data_35[:, 4], data_35[:, 5],color=plot_color)
        plt.show()
    else:
        render()
        #preview()
