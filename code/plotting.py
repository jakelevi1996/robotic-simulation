import matplotlib.pyplot as plt
import numpy as np

def add_frame_to_plot(x, y, L, frame=-1, alpha=1):
    # Plot links:
    foot1x = (x[0] + L).reshape(1, -1)
    foot2x = (x[4] - L).reshape(1, -1)
    x = np.concatenate([foot1x, x, foot2x], axis=0)
    foot1y = (y[0]).reshape(1, -1)
    foot2y = (y[4]).reshape(1, -1)
    y = np.concatenate([foot1y, y, foot2y], axis=0)
    plt.plot(x[:, frame], y[:, frame], 'k-', alpha=alpha)
    # Plot motors:
    plt.plot(x[2:5, frame], y[2:5, frame], 'ro', alpha=alpha)

def plot_ground():
    x_ground = [-1.1, -.5, -.5, .2]
    y_ground_top = [.05, .05, 0, 0]
    y_ground_bottom = [-.1, -.1, -.1, -.1]
    plt.fill_between(x_ground, y_ground_top, y_ground_bottom)

def plot_robot_trajectory(
    x, y, L, dt, T=9, filename="images/robot trajectory"
):
    plot_every = int(T / dt)
    plt.figure()
    for f in range(0, x.shape[1], plot_every):
        add_frame_to_plot(x, y, L, frame=f, alpha=0.3)
    add_frame_to_plot(x, y, L, frame=-1, alpha=1)
    plot_ground()
    plt.axis('equal')
    plt.xlim(-1.1, 0.2)
    plt.savefig(filename)
    plt.close()
    
def plot_frame(x, y, L, filename="robot pos", frame=-1):
    plt.figure()
    add_frame_to_plot(x, y, L, frame)
    plot_ground()
    plt.axis('equal')
    plt.xlim(-1.1, 0.2)
    plt.savefig(filename)
    plt.close()

def plot_traces(
    dt, traces, filename="traces", title=None,
    legend_prefix="", label_num_offset=0, legend_entries=None, xlims=None
):
    plt.figure()
    t = np.linspace(0, dt*traces[0].size, traces[0].size)
    labels = []
    for num, trace in enumerate(traces):
        plt.plot(t, trace)
        labels.append(legend_prefix + str(num + label_num_offset))
    if xlims is not None: plt.xlim(xlims)
    if title is not None: plt.title(title)
    if legend_entries is not None: plt.legend(legend_entries)
    else: plt.legend(labels)
    plt.grid(True)
    plt.savefig(filename)
    plt.close()


def plot_trace(
    dt, trace, filename="trace", title=None, xlims=None
):
    plt.figure()
    t = np.linspace(0, dt*trace.size, trace.size)
    plt.plot(t, trace)
    if xlims is not None: plt.xlim(xlims)
    if title is not None: plt.title(title)
    plt.grid(True)
    plt.savefig(filename)
    plt.close()