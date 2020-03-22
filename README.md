# A* search algorithm for a rigid robot
A* algorithm is an informed search based on which the path planning algorithm for a rigid robot in an obstacle space is implemented

## System and library requirements.
 - Python3
 - Numpy
 - OpenCV
 - Math
 - Sympy
 - Matplotlib
 - Time
 
 
 ## Installation 
 Above mentioned dependencies can be installed using the following links:   
 Numpy -https://docs.scipy.org/doc/numpy-1.10.1/user/install.html   
 OpenCV -https://docs.opencv.org/master/d5/de5/tutorial_py_setup_in_windows.html   
 Math - https://datatofish.com/install-package-python-using-pip/   
 Sympy - https://docs.sympy.org/latest/install.html   
 Matplotlib - https://matplotlib.org/users/installing.html   
 
## How to Run
1. Clone this repo or extract the zip file "proj3p2_18_python". <br>
2. To view the simulation video for the following parameters - 
-Start : (50, 30, 60)
-Goal : (150, 150)
-Radius and clearance : 1 unit each
-Step Size: 1 unit
Open the video "output.mp4"<br>
3. To run the code, navigate to the "codes/Implementing_Astar_on_Turtlebot3" folder. From the terminal, run the command `python3 A_star_final.py` <br>
4. The program will ask for the radius from the obstacles and then the clearance of the robot, provide input in both cases in 'int' format. For eg: 2<br>
5. Next program will ask for start point coordinates x, y and theta. Give input x-->press "Enter",Give input y-->press "Enter", Give input theta-->press "Enter" . <br>
6. You will then be asked the step size for each movement, provide input in integer, For eg: 3 <br>

7. Next program will ask for goal point, follow the same methodology mentioned for start point
If the points provided are in the obstacle space or out of bounds, program will terminate.<br>

## Organisation of code
The code is split into 3 python programs which are:-
1. **A_star_final.py** is the main python file.
2. **intersection_check.py** checks for intersections of vectors with obstacles.
3. **obstacle_check.py** checks whether a point is within an obstacle or not.
We have commented the sympy implementation to find the intersection of each vector with the obstacle space due to its time consuming execution. 
Now we have looked for whether the node is lying in the obstacle space or not as the step size is relatively small as compared to the obstacles.
 
## Output
The simulation video is output as **output.mp4** and the frames are available in the "plots" folder.
The final image is "plots.png" in the plots folder.