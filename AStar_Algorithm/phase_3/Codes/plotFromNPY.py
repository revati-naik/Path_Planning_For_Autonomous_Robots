import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import visualization as viz


def function():
	npy_exploration_path = sys.argv[1]
	npy_path_path = sys.argv[2]
	output_dir = sys.argv[3]

	exp_nodes = np.load(npy_exploration_path, allow_pickle=True)
	path_nodes = np.load(npy_path_path, allow_pickle=True)

	print("Number of visited nodes:", len(exp_nodes))
	print("Number of nodes in path:", len(path_nodes))

	plotter = viz.initPlot(path_nodes[0].getXYCoords(), path_nodes[-1].getXYCoords(), title="Final Plotting")
	plt.savefig(os.path.join(output_dir, "1.png"))
	plt.ion()
	i = 2
	i = viz.plotPath(path=exp_nodes, rev=False, pause_time=0.001, plotter=plotter, color="blue", linewidth=1, write_path_prefix=i, show=False, skip_frames=25)
	i = viz.plotPath(path=path_nodes, rev=True, pause_time=0.001, plotter=plotter, color="lime", linewidth=3, write_path_prefix=i, show=False, skip_frames=1)
	plt.ioff()
	print("Done with plots.")
	plt.show()


def main():
	function()


if __name__ == '__main__':
	main()
