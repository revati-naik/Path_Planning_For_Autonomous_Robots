from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import pickle

##
## Defining the obstacle space and checking if the point coordinate is within the obstacle space
##
## :param      x:          x-coordinate in cartesian space
## :type       x:          float
## :param      y:          y-coordinate in cartesian space
## :type       y:          float
## :param      radius:     The radius
## :type       radius:     float
## :param      clearance:  The clearance
## :type       clearance:  float
##
## :returns:   True if the point lies in the obstacle space. Otherwise False
## :rtype:     boolean
##
def withinObstacleSpace((x,y),radius,clearance):

    flag = 0
    flag_1 = 0
    flag_2 = 0
    point = Point(x,y)
    rectangle = Polygon([(35, 76), (100, 39),(95, 30), (30, 68)])
    complex_polygon = Polygon([(25, 185), (75, 185),(100, 150), (75, 120), (50,150), (20,120)])
    kite = Polygon([(225, 40), (250, 25),(225, 10), (200, 25)])
    
    #kite
    kite_line_1 = ((y-25)*25) + ((x-200)*15)
    kite_line_2 = ((y-10)*25) - ((x-225)*15)
    kite_line_3 = ((y-25)*25) + ((x-250)*15)
    kite_line_4 = ((y-40)*25) - ((x-225)*15)
    
    #rectangle
    # rect_line_1 = ((y-76)*65) + ((x-35)*37)
    # rect_line_2 = ((y-39)*5) - ((x-100)*9)
    # rect_line_3 = ((y-30)*65) + ((x-95)*38)
    # rect_line_4 = ((y-68)*5) - ((x-30)*8)

    rect_line_1 = ((y-67.5)*65.04) + ((x-30.04)*37.5)
    rect_line_2 = ((y-30)*10) - ((x-95)*17.32)
    rect_line_3 = ((y-47.32)*(-64.96)) - ((x-105)*37.5)
    rect_line_4 = ((y-84.87)*10) + ((x-40.04)*17.32)
    
    #complex polygon
    quad_1_1 = 5*y+6*x-1050
    quad_1_2 = 5*y-6*x-150
    quad_1_3 = 5*y+7*x-1450
    quad_1_4 = 5*y-7*x-400
    
    quad_2_1 = ((y-185)*5) - ((x-25)*65)
    quad_2_2 = ((y-120)*30) - ((x-20)*30)
    quad_2_3 = ((y-150)*25) - ((x-50)*35)
    quad_2_4 = ((y-185)*(-50))
    
    #check kite
    if kite_line_1 > 0 and kite_line_2 > 0 and kite_line_3 < 0 and kite_line_4 < 0 or point.distance(kite) <= radius+clearance:
        flag = 1
    
    #check rectangle
    if rect_line_1 < 0 and rect_line_2 > 0 and rect_line_3 > 0 and rect_line_4 < 0 or point.distance(rectangle) <= radius+clearance:
        flag = 1
    
    #check polygon
    if quad_1_1>0 and quad_1_2>0 and quad_1_3<0 and quad_1_4<0:
        flag_1 = 1
    else:
        flag_1 = 0

    if quad_2_1 < 0 and quad_2_2 > 0 and quad_2_3 > 0 and quad_2_4 > 0:
        flag_2 = 1
    else:
        flag_2 = 0

    if flag_1 == 1 or flag_2 == 1 or point.distance(complex_polygon) <= radius+clearance:
        flag = 1
    
    #circle
    if(((x - (225))**2 + (y - (150))**2 - (25+radius+clearance)**2) <= 0) :
        flag = 1
        
    #ellipse
    if (((x - (150))/(40+radius+clearance))**2 + ((y - (100))/(20+radius+clearance))**2 - 1) <= 0:
        flag = 1    
    return flag

##
## Genarting the map for the obstacle space 
##
def generateMap():
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

    pickle.dump(ax, file('map.pickle', 'wb'))


def testMain():
    # print(withinObstacleSpace((35,68),0,0))
    # generateMap()
    pass
    

if __name__ == '__main__':
    testMain()