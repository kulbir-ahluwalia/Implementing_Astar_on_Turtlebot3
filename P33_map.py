import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from matplotlib import animation
from matplotlib.animation import FuncAnimation


#color defining
white = (255,255,255)
black = (0,0,0)

red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

scale = 1

# Dimensions of the maze specified in the question.
width= 100
height = 100


coord_square_up = np.array([(23.5, 88.5),
                            (23.5, 73.5),
                            (38.5, 73.5),
                            (38.5 , 88.5)], dtype='int')

coord_square_right = np.array([(83.5, 58.5),
                            (83.5, 43.5),
                            (98.5, 43.5),
                            (98.5 , 58.5)], dtype='int')

coord_square_left = np.array([(35, 58.5),
                            (35, 43.5),
                            (18.5, 43.5),
                            (18.5 , 58.5)], dtype='int')

coord_circle0 = [(10 ), (51, 51)]
coord_circle1 = [(10 ), (31, 21)]
coord_circle2 = [(10 ), (71, 21)]
coord_circle3 = [(10 ), (71, 81)]


fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(3,3)
ax = plt.axes(xlim=(0,width),ylim=(0,height))
circle0 = plt.Circle((coord_circle0[1]),coord_circle0[0],fc='green')
circle1 = plt.Circle((coord_circle1[1]),coord_circle1[0],fc='green')
circle2 = plt.Circle((coord_circle2[1]),coord_circle2[0],fc='green')
circle3 = plt.Circle((coord_circle3[1]),coord_circle3[0],fc='green')
square_up = plt.Polygon(coord_square_up,fc ='green')
square_left = plt.Polygon(coord_square_left,fc ='green')
square_right = plt.Polygon(coord_square_right,fc ='green')
obstacles = [circle0,circle1,circle2,circle3,square_up,square_left,square_right]
for item in obstacles:
    plt.gca().add_patch(item)
plt.show()