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
    #plot_color = 'tab:blue'
    plot_color = foreground_color

    vid2 = genfromtxt('vid8-65.csv', delimiter=',')
    #vid3 = genfromtxt('vid3.csv', delimiter=',')
    vid3 = genfromtxt('vid8-45-2.csv', delimiter=',')

    # change size and resolution
    plt.rcParams["figure.figsize"] = (16, 8)
    plt.rcParams['figure.dpi'] = 100
    plt.rcParams['axes.linewidth'] = 0.5
    plt.rcParams['xtick.color'] = foreground_color
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['xtick.top'] = True
    plt.rcParams['ytick.color'] = foreground_color
    # plt.rcParams['ytick.left'] = True
    # plt.rcParams['ytick.right'] = True
    # plt.rcParams['ytick.labelleft'] = False
    # plt.rcParams['ytick.labelright'] = True
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

    rad2deg = 180./np.pi

    endt = min(vid2[-1,0],vid3[-1,0])

    # cable angles
    axs[0].plot(vid2[:, 0], rad2deg * vid2[:, 2], color='tab:blue')
    axs[0].plot(vid3[:, 0], rad2deg * vid3[:, 2], color='tab:orange')
    axs[0].set_ylim((25, 85))
    axs[0].set_yticks([45,65])
    axs[0].set_xlim((15,endt))

    # thrust
    axs[1].plot(vid2[:, 0], vid2[:, 11], color='tab:blue')
    axs[1].plot(vid3[:, 0], vid3[:, 11], color='tab:orange')
    axs[1].set_ylim((12, 38))
    axs[1].set_yticks([20,25,30])
    axs[1].set_xlim((15,endt))

    for ax in axs:
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))

    axs[0].xaxis.set_ticklabels([])

    def setup_format():
        # axs[0].set_ylabel('$\psi$').set_rotation(0)
        # axs[1].set_ylabel('$\\theta$').set_rotation(0)
        # axs[2].set_ylabel('$\phi$').set_rotation(0)
        # axs[3].set_ylabel('$\mathcal{E}$ $(\mathrm{J})$').set_rotation(0)
        #axs[3].set_xlabel('time (s)')
        pass

    setup_format()

    fps = 60
    speed = 5
    interval = (1000/fps)  # ms between frames
    duration_sec = max(vid2[-1, 0],vid2[-1, 0])
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
        anim.save('vid8.mp4', writer=writer)

    def preview():
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=interval, blit=False)
        plt.show()

    #preview()
    #render()
    plt.show()
