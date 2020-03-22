# A* search algorithm for a rigid robot
A* algorithm is an informed search based on which the path planning algorithm for a rigid robot in an obstacle space is implemented

## System and library requirements.
 - Python3
 - Numpy
 - cv2
 - math
 - sympy
 
## How to Run
1. Clone this repo or extract the zip file. <br>
2. Navigate to the folder "Implementing_Astar_on_Rigid_Robot" <br>
3. To view the simulation video for the following parameters - 
Start : (50, 30, 60)
Goal : (150, 150)
Radius and clearance : 1 unit each
Step Size: 1 unit
Open the video "output.avi"<br>
4. To run the code, navigate to the "Test_code" folder. From the terminal, run the command `python A_star_final.py` <br>
5. The program will ask for the clearance from the obstacles and then the radius of the robot, provide input in both cases in 'int' format. For eg: 2<br>
6. Next program will ask for start point coordinates x, y and theta. Give input x-->press "Enter",Give input y-->press "Enter", Give input theta-->press "Enter" . <br>
7. Next program will ask for goal point, follow the same methodology mentioned for start point
If the points provided are in the obstacle space or out of bounds, program will terminate.<br>
8. You will then be asked the step size for each movement, provide input in 'int' format, For eg: 3 <br>

 
