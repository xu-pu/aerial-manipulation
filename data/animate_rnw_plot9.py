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
    plot_color = 'tab:blue'
    #plot_color = foreground_color

    data = genfromtxt('vid7.n25t40.bag.cont.csv', delimiter=',')

    # change size and resolution
    plt.rcParams["figure.figsize"] = (12,12)
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
    plt.rcParams['lines.linewidth'] = 2
    plt.rcParams['lines.color'] = foreground_color
    plt.rcParams['figure.facecolor'] = background_color
    plt.rcParams['axes.facecolor'] = background_color
    plt.rcParams['axes.edgecolor'] = foreground_color
    plt.rcParams['axes.labelcolor'] = foreground_color
    plt.rcParams['axes.labelsize'] = 20
    plt.rcParams['axes.labelpad'] = 20


    # setup subplots
    fig, axs = plt.subplots(4, 1)

    fig.tight_layout(pad=1)

    rad2deg = 180./np.pi

    t_end = data[-1,0]

    # contact
    axs[0].plot(data[:, 5], data[:, 4], color=plot_color)
    #axs[0].set_xlim((-2.3, 5.7))
    axs[0].set_xticks([-2,-1,0,1,2])
    axs[0].set_yticks([])
    axs[0].set_xlim((-2, 2.5))
    axs[0].set_ylim((-0.4, 0.4))
    axs[0].set_aspect('equal',adjustable='box')
    axs[0].xaxis.tick_top()

    # ke
    axs[1].plot(data[:, 0], rad2deg * data[:, 1], color=plot_color)
    axs[1].set_xlim((0, t_end))
    axs[1].set_ylim((-80,80))
    axs[1].set_yticks([-60,0,60])
    axs[1].xaxis.set_ticklabels([])
    # phi
    axs[2].plot(data[:, 0], rad2deg * data[:, 2], color=plot_color)
    axs[2].set_xlim((0, t_end))
    axs[2].set_ylim((0, 50))
    axs[2].set_yticks([10, 25, 40])
    axs[2].xaxis.set_ticklabels([])
    # theta
    axs[3].plot(data[:, 0], rad2deg * data[:, 3], color=plot_color)
    axs[3].set_xlim((0, t_end))
    axs[3].set_ylim((-80, 80))
    axs[3].set_yticks([-60,0,60])
    #axs[3].xaxis.set_ticklabels([])

    # for ax in axs:
    #     #ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    #     ax.xaxis.set_ticklabels([])

    plt.show()

    def setup_format():
        # axs[0].set_ylabel('$\psi$').set_rotation(0)
        # axs[1].set_ylabel('$\\theta$').set_rotation(0)
        # axs[2].set_ylabel('$\phi$').set_rotation(0)
        # axs[3].set_ylabel('$\mathcal{E}$ $(\mathrm{J})$').set_rotation(0)
        #axs[3].set_xlabel('time (s)')
        pass

    setup_format()

    fps = 60
    speed = 10
    interval = (1000/fps)  # ms between frames
    duration_sec = data[-1, 0]
    frames = int(duration_sec * 1000 / interval / speed)
    t_start = 0
    t_window = 10
    t_end = t_start-t_window

    def init():
        for ax in axs:
            ax.set_xlim((t_end, t_start))
        setup_format()
        return axs

    def animate(i):
        offset = i*speed*interval/1000.
        for ax in axs:
            ax.set_xlim((t_end+offset, t_start+offset))
        setup_format()
        return axs

    def render():
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='Me'), bitrate=1800)
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, blit=False)
        anim.save('im.mp4', writer=writer)

    def preview():
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=interval, blit=False)
        plt.show()

    preview()

