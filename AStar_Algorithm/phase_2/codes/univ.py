from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import node


def function(exploration_node_vector, path_node_vector, goal_node, step_size):

	fig, ax = plt.subplots()
	ax.set(xlim=(0, 300), ylim = (0,200))
	ax.set_aspect('equal')
	a_circle = plt.Circle((225,150), 025, color='b')

	ellipse = Ellipse(xy=(157, 100), width=80, height=40, 
						edgecolor='b', fc='b', lw=2)
	
	# rectangle = plt.Rectangle(xy=(30.04,67.5), width=75, height=20, angle=150)
	# rect_points = np.array((30.04,67.5), (95,30), ())
	# rectangle = Polygon (xy=())
	rectangle = plt.Polygon([(30.04, 67.5), (95, 30), (105, 47.32), (40.04, 84.82)]) 
	polygon = plt.Polygon([(25, 185), (75, 185),(100, 150), (75, 120), (50,150), (20,120)])
	kite = plt.Polygon([(200,25), (225,10), (250,25), (225,40)])

	ax.add_patch(ellipse)
	ax.add_artist(a_circle)
	ax.add_line(polygon)
	ax.add_line(kite)
	ax.add_line(rectangle)

	# Adding the start node
	xs, ys = path_node_vector[0].getXYCoords()
	plt.plot(xs, ys, color='#EE82EE', marker='o', markersize=15)

    	# Adding the goal node
	# xg, yg = path_node_vector[-1].getXYCoords()
	xg, yg = goal_node.getXYCoords()
	plt.plot(xg, yg, color='#7CFC00', marker='o', markersize=15)

	colors = ['red', 'pink', 'green', 'yellow']
	color_i = 0

	plt.ion()
	for node in exploration_node_vector:
		plt.gca().set_aspect('equal')
		plt.xlim(0, 300)
		plt.ylim(0, 200)
	
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
		q = plt.quiver(x, y, u, v, units='xy', scale=1, width=1.5, headwidth=2, headlength=2, color=colors[color_i%len(colors)])

		# plt.plot(x, y, 'rqo')	

		color_i += 1

		plt.show()
		plt.pause(0.0001)


	for node in path_node_vector[:-1]:
		plt.gca().set_aspect('equal')
		plt.xlim(0, 300)
		plt.ylim(0, 200)
	
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
		q = plt.quiver(x, y, u, v, units='xy', scale=1, width=1.5, headwidth=2, headlength=2, color="black")

		plt.show()
		plt.pause(0.01)
	plt.ioff()
	
	plt.gca().set_aspect('equal')
	plt.xlim(0, 300)
	plt.ylim(0, 200)

	plt.minorticks_on()
	plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
	plt.grid(which='minor', color='black')
	plt.title('Vector plot', fontsize=25)

	xy = path_node_vector[-1].getParentXYCoords()
	uv = path_node_vector[-1].getXYCoords()
	if xy is not None and uv is not None:
		x,y = xy
		u,v = uv
		u -= x
		v -= y
		q = plt.quiver(x, y, u, v, units='xy', scale=1, width=1.5, headwidth=2, headlength=2, color="black")

	plt.show()
