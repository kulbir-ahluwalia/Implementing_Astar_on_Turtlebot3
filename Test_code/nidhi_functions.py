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


def rectangle_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    rectangle_point_1 = [100, 38.66025]
    rectangle_point_2 = [35.0481, 76.1603]
    rectangle_point_3 = [30.0481, 67.5]
    rectangle_point_4 = [95, 30]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(None, rectangle_point_1, rectangle_point_2)
    line1 = y - (edge1_m_c[0] * x) - (edge1_m_c[1] + (augment_distance * 2 / (3 ** 0.5)))

    edge2_m_c = find_line_slope_and_intercept(None, rectangle_point_2, rectangle_point_3)
    line2 = y - (edge2_m_c[0] * x) - (edge2_m_c[1] + (augment_distance * 2))

    edge3_m_c = find_line_slope_and_intercept(None, rectangle_point_3, rectangle_point_4)
    line3 = y - (edge3_m_c[0] * x) - (edge3_m_c[1] - (augment_distance * 2 / (3 ** 0.5)))

    edge4_m_c = find_line_slope_and_intercept(None, rectangle_point_4, rectangle_point_1)
    line4 = y - (edge4_m_c[0] * x) - (edge4_m_c[1] - (augment_distance * 2))

    solution3 = solve([line1, line_equation], (x, y))
    print("Intersection points with first line: ", solution3)
    solution4 = solve([line2, line_equation], (x, y))
    print("Intersection points with second line: ", solution4)
    solution5 = solve([line3, line_equation], (x, y))
    print("Intersection points with third line: ", solution5)
    solution6 = solve([line4, line_equation], (x, y))
    print("Intersection points with Fourth line: ", solution6)

def rhombus_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    rhombus_point_1 = [250, 25]
    rhombus_point_2 = [225, 40]
    rhombus_point_3 = [200, 25]
    rhombus_point_4 = [225, 10]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(None, rhombus_point_1, rhombus_point_2)
    line1 = y - (edge1_m_c[0] * x) - (edge1_m_c[1] + (augment_distance / 0.8575))

    edge2_m_c = find_line_slope_and_intercept(None, rhombus_point_2, rhombus_point_3)
    line2 = y - (edge2_m_c[0] * x) - (edge2_m_c[1] + (augment_distance / 0.8575))

    edge3_m_c = find_line_slope_and_intercept(None, rhombus_point_3, rhombus_point_4)
    line3 = y - (edge3_m_c[0] * x) - (edge3_m_c[1] - (augment_distance / 0.8575))

    edge4_m_c = find_line_slope_and_intercept(None, rhombus_point_4, rhombus_point_1)
    line4 = y - (edge4_m_c[0] * x) - (edge4_m_c[1] - (augment_distance / 0.8575))

    solution7 = solve([line1, line_equation], (x, y))
    print("Intersection points with first line: ", solution7)
    solution8 = solve([line2, line_equation], (x, y))
    print("Intersection points with second line: ", solution8)
    solution9 = solve([line3, line_equation], (x, y))
    print("Intersection points with third line: ", solution9)
    solution10 = solve([line4, line_equation], (x, y))
    print("Intersection points with Fourth line: ", solution10)


ellipse_intersection_check(1,1,[200,150],[250,150])
rectangle_intersection(1,1,[200,150],[250,150])
rhombus_intersection(1,1,[200,150],[250,150])