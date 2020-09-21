import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import a_star
import obstacles


def plot_curve(curr_node, plotter=plt, color="blue", linewidth=1):
    """
    Plot the curve from the parent of the curr_node to the curr_node.

    :param      curr_node:  The curr node
    :type       curr_node:  node
    :param      plotter:    The plotter
    :type       plotter:    matplotlib.pyplot
    :param      color:      The color
    :type       color:      string
    :param      linewidth:  The linewidth
    :type       linewidth:  number
    """
    try:
        Xi, Yi = curr_node.getParentXYCoords()
        Thetai = curr_node.parent_orientation
        UL, UR = curr_node.action
    except Exception:
        print("Cannot plot the curve for this node.")
        return

    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = math.radians(Thetai)

    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        plotter.plot([Xs, Xn], [Ys, Yn], color=color, linewidth=linewidth)


def markNodeXY(marker_node_xy, plotter=plt, color='#EE82EE', marker='o'):
    """
    Mark an XY coordinate on the map

    :param      marker_node_xy:  The xy-coordinate to be marked
    :type       marker_node_xy:  tuple
    :param      plotter:         The plotter
    :type       plotter:         matplotlib.pyplot
    :param      color:           The color
    :type       color:           string
    :param      marker:          The marker
    :type       marker:          string
    """
    x, y = marker_node_xy
    plotter.plot(x, y, color=color, marker=marker, markersize=7)


def markNode(marker_node, plotter=plt, color='#EE82EE', marker='o'):
    """
    Mark a node on the map

    :param      marker_node:  The node to be marked
    :type       marker_node:  Node
    :param      plotter:      The plotter
    :type       plotter:      matplotlib.pyplot
    :param      color:        The color
    :type       color:        string
    :param      marker:       The marker
    :type       marker:       string
    """
    markNodeXY(marker_node.getXYCoords(), plotter=plotter, color=color, marker=marker)


def plotPath(path, rev=False, pause_time=0.001, plotter=plt, color="black", linewidth=2, write_path_prefix=-1, show=False, skip_frames=8):
    """
    Plot the list of nodes as path on the map.

    :param      path:               The path
    :type       path:               List of Nodes
    :param      rev:                Whether to print the path in reverse
    :type       rev:                boolean
    :param      pause_time:         The pause time
    :type       pause_time:         number
    :param      plotter:            The plotter
    :type       plotter:            matplotlib.pyplot
    :param      color:              The color
    :type       color:              string
    :param      linewidth:          The linewidth
    :type       linewidth:          number
    :param      write_path_prefix:  The write path prefix filename
    :type       write_path_prefix:  number
    :param      show:               Whether to show the plot on the go
    :type       show:               boolean
    :param      skip_frames:        The number of frames to skip while live visualization
    :type       skip_frames:        number

    :returns:   Updated write_path_prefix
    :rtype:     number
    """
    if rev:
        path_plt = path[::-1]
    else:
        path_plt = path

    for i, node_itr in enumerate(path_plt):
        plot_curve(node_itr, color=color, plotter=plotter, linewidth=linewidth)
        if i % skip_frames == 0:
            if write_path_prefix > -1:
                write_path = os.path.join(a_star.OUTPUT_DIR, str(write_path_prefix) + ".png")
                plt.savefig(write_path)
                write_path_prefix += 1

            if show:
                plt.show()
                plt.pause(pause_time)

    return write_path_prefix


def initPlot(start_xy, goal_xy, title=""):
    """
    Initializes the plot.

    :param      start_xy:  The start xy node
    :type       start_xy:  tuple
    :param      goal_xy:   The goal xy node
    :type       goal_xy:   tuple
    :param      title:     frame title
    :type       title:     string

    :returns:   Plotter
    :rtype:     matplotlib.pyplot
    """
    fig, ax = plt.subplots()
    fig.suptitle(title, fontsize=16)

    ax.set(xlim=(-5, 5), ylim = (-5, 5))
    ax.set_aspect('equal')

    obstacles.generateMap(plotter=ax)

    markNodeXY(start_xy, plotter=ax, color='#00FF00', marker='o')
    markNodeXY(goal_xy, plotter=ax, color='#FF0000', marker='^')

    return ax
