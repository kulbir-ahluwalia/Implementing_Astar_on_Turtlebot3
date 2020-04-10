## Draw a circle with center and radius defined
## Also enable the coordinate axes
import matplotlib.pyplot as plt
import numpy as np
# Define limits of coordinate system
x1 = -5.0
x2 = 5.0
y1 = -5.0
y2 = 5.0

circle1 = plt.Circle((0,0),1, color = 'k', fill = False, clip_on = False)
circle2 = plt.Circle((-2,-3),1, color = 'k', fill = False, clip_on = False)
circle3 = plt.Circle((2,-3),1, color = 'k', fill = False, clip_on = False)
circle4 = plt.Circle((2,3),1, color = 'k', fill = False, clip_on = False)

coord_square_up = np.array([(-2.75, 3.75),(-1.25, 3.75),(-1.25 , 2.25),(-2.75, 2.25)], dtype='float')
square_up = plt.Polygon(coord_square_up,color = 'k', fill = False, clip_on = False)

coord_square_left = np.array([(-4.75, 0.75),(-3.25, 0.75),(-3.25, -0.75),(-4.75 , -0.75)], dtype='float')
square_left = plt.Polygon(coord_square_left,color = 'k', fill = False, clip_on = False)

coord_square_right = np.array([(3.25, 0.75),(4.75,0.75),(4.75, -0.75),(3.25 , -0.75)], dtype='float')
square_right = plt.Polygon(coord_square_right, color = 'k', fill = False, clip_on = False)

coord_square_inner = np.array([(-5, 5),(5,5),(5, -5),(-5 , -5)], dtype='float')
square_inner = plt.Polygon(coord_square_inner, color = 'k', fill = False, clip_on = False)



fig, ax = plt.subplots()
ax.add_artist(circle1)
ax.add_artist(circle2)
ax.add_artist(circle3)
ax.add_artist(circle4)
ax.add_artist(square_up)
ax.add_artist(square_left)
ax.add_artist(square_right)
ax.add_artist(square_inner)

plt.axis("equal")

ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')

plt.xlim(left=x1)
plt.xlim(right=x2)
plt.ylim(bottom=y1)
plt.ylim(top=y2)
plt.axhline(linewidth=2, color='k')
plt.axvline(linewidth=2, color='k')

# plt.grid(True)
plt.grid(color='k', linestyle='-.', linewidth=0.5)
plt.show()