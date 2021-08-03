from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.ticker import MaxNLocator

if __name__ == '__main__':

    # background_color = 'blue'
    # foreground_color = 'red'

    background_color = 'black'
    foreground_color = 'white'

    data = genfromtxt('/home/sheep/2021-07-01-01-20-53.table6.perfect.ground.55.120.bag.cont.csv', delimiter=',')

    # change size and resolution
    plt.rcParams["figure.figsize"] = (5,10)
    plt.rcParams['figure.dpi'] = 100
    plt.rcParams['axes.linewidth'] = 0.5
    plt.rcParams['xtick.color'] = foreground_color
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.color'] = foreground_color
    plt.rcParams['lines.linewidth'] = 2
    plt.rcParams['lines.color'] = foreground_color
    plt.rcParams['figure.facecolor'] = background_color
    plt.rcParams['axes.facecolor'] = background_color
    plt.rcParams['axes.edgecolor'] = foreground_color

    # setup subplots
    fig, axs = plt.subplots(4, 1)

    axs[0].plot(data[:, 0], data[:, 1], label=r'$\phi$', color=foreground_color)
    axs[0].set_ylabel('phi_dot')

    axs[1].plot(data[:, 0], data[:, 2], label=r'$\phi$', color=foreground_color)
    axs[1].set_ylabel('phi_dot')

    axs[2].plot(data[:, 0], data[:, 3], label=r'$\phi$', color=foreground_color)
    axs[2].set_ylabel('phi_dot')

    axs[3].plot(data[:, 0], data[:, 4], label=r'$\phi$', color=foreground_color)
    axs[3].set_xlabel('t(time)')
    axs[3].set_ylabel('phi_dot')

    for ax in axs:
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.xaxis.set_ticklabels([])

    def setup_format():
        axs[0].set_ylabel('phi_dot')
        axs[1].set_ylabel('phi_dot')
        axs[2].set_ylabel('phi_dot')
        axs[3].set_xlabel('t(time)')
        axs[3].set_ylabel('phi_dot')

    setup_format()

    fps = 60
    speed = 1
    interval = (1000/fps)  # ms between frames
    duration_sec = 10
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

