import matplotlib.pyplot as plt


def plainPlot(node_vector):
	plt.figure("plot")

	for node in node_vector:
		x,y = node.getXYCoords()
		plt.plot(x, y, 'ro')

	plt.show()
	plt.close()


def plainQuiver(node_vector):
	plt.figure("QuiverPlot")

	x,y = node_vector[0].getXYCoords()
	for node in node_vector[1:]:
		u,v = node.getXYCoords()
		u -= x
		v -= y
		q = plt.quiver(x, y, u, v, units='xy', scale=1, color='black')

		x,y = node.getXYCoords()
		# plt.plot(x, y, 'ro')

	plt.gca().set_aspect('equal')
	plt.xlim(-5, 5)
	plt.ylim(-5, 5)

	plt.minorticks_on()
	plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
	plt.grid(which='minor', color='black')
	plt.title('Vector plot', fontsize=25)

	plt.show()
	plt.close()


def explorationQuiver(node_vector):
	# plt.figure("exploration")

	# colors = ['red', 'blue', 'green', 'yellow']
	# color_i = 0

	# for node in node_vector:
	# 	xy = node.getParentXYCoords()
	# 	uv = node.getXYCoords()
	# 	if xy is None or uv is None:
	# 		continue
	# 	x,y = xy
	# 	u,v = uv
	# 	u -= x
	# 	v -= y
	# 	q = plt.quiver(x, y, u, v, units='xy', scale=1, color=colors[color_i%len(colors)])

	# 	# plt.plot(x, y, 'ro')	


	# 		color_i += 1


	# plt.gca().set_aspect('equal')
	# plt.xlim(-5, 5)
	# plt.ylim(-5, 5)

	# plt.minorticks_on()
	# plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
	# plt.grid(which='minor', color='black')
	# plt.title('Vector plot', fontsize=25)

	# plt.show()
	# plt.close()
	# 
	expl_fig = plt.figure("exploration")

	colors = ['red', 'blue', 'green', 'yellow']
	color_i = 0
	plt.ion()
	for node in node_vector:
		expl_fig.gca().set_aspect('equal')
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
	
		plt.minorticks_on()
		plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
		plt.grid(which='minor', color='black')
		plt.title('Vector plot', fontsize=25)

		xy = node.getParentXYCoords()
		uv = node.getXYCoords()
		if xy is None or uv is None:
			continue

		x,y = xy
		u,v = uv
		u -= x
		v -= y
		q = plt.quiver(x, y, u, v, units='xy', scale=1, color=colors[color_i%len(colors)])

		# expl_fig.plot(x, y, 'rqo')	

		color_i += 1

		expl_fig.show()
		plt.pause(0.02)

	return expl_fig

