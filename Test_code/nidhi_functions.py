import numpy as np
from sympy import solve
import math

def find_line_slope_and_intercept(test_point_coord, line_point_1, line_point_2):
    slope = (line_point_2[1] - line_point_1[1]) / (line_point_2[0] - line_point_1[0])
    intercept = line_point_1[1] - (slope * line_point_1[0])
    # print(slope,intercept)
    return slope, intercept


def ellipse_intersection_check(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])
    print(line_equation)
    a = 40 + augment_distance
    b = 20 + augment_distance
    center = (150, 100)
    equation_1 = ((x - center[0]) ** 2 / (b ** 2)) + ((y - center[1]) ** 2 / (a ** 2)) - 1
    equation_2 = line_equation
    # the following equation will output solutions in the form of a list
    solution2 = solve([equation_1, equation_2], (x, y))
    print(solution2)

ellipse_intersection_check(1,1,[200,150],[250,150])