import numpy as np
from sympy import solve, Poly, Eq, Function, exp
from sympy.abc import x, y, z, a, b
import math


def find_line_slope_and_intercept(test_point_coord, line_point_1, line_point_2):
    if (line_point_2[0] - line_point_1[0]) == 0:
        x_intercept = line_point_1[0]
        return math.inf, x_intercept
    else:
        slope = (line_point_2[1] - line_point_1[1]) / (line_point_2[0] - line_point_1[0])
        y_intercept = line_point_1[1] - (slope * line_point_1[0])
        return slope, y_intercept

    # #print(slope,intercept)


def circular_intersection_check(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])
    ##print(line_equation)

    circle_center = (225, 150)
    equation_1 = (x - 225) ** 2 + (y - 150) ** 2 - (25 + augment_distance) ** 2
    equation_2 = line_equation
    # the following equation will output solutions in the form of a list
    solution = solve([equation_1, equation_2], (x, y))
    ##print(solution)

    not_intersecting = True
    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[0] <= x_max) and (y_min <= root[1] <= y_max):
            not_intersecting = False
            break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


def ellipse_intersection_check(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    if line_m_c[0] == math.inf:
        line_equation = x - line_m_c[1]  # x- x_intercept
    else:
        line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # #print(line_equation)
    a = 40 + augment_distance
    b = 20 + augment_distance
    center = (150, 100)
    equation_1 = ((x - center[0]) ** 2 / (a ** 2)) + ((y - center[1]) ** 2 / (b ** 2)) - 1
    equation_2 = line_equation
    # the following equation will output solutions in the form of a list
    solution2 = solve([equation_1, equation_2], (x, y))
    # print(solution2)

    not_intersecting = True
    for root in solution2:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[0] <= x_max) and (y_min <= root[1] <= y_max):
            not_intersecting = False
            break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


def rectangle_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    rectangle_point_1 = [100, 38.66025]
    rectangle_point_2 = [35.0481, 76.1603]
    rectangle_point_3 = [30.0481, 67.5]
    rectangle_point_4 = [95, 30]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    if line_m_c[0] == math.inf:
        line_equation = x - line_m_c[1]  # x- x_intercept
    else:
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
    # print("Intersection points with first line: ", solution3)
    solution4 = solve([line2, line_equation], (x, y))
    # print("Intersection points with second line: ", solution4)
    solution5 = solve([line3, line_equation], (x, y))
    # print("Intersection points with third line: ", solution5)
    solution6 = solve([line4, line_equation], (x, y))
    # print("Intersection points with Fourth line: ", solution6)

    solution = [solution3, solution4, solution5, solution6]

    not_intersecting = True
    for root in solution:
        # print(root)
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[x] <= x_max) and (y_min <= root[y] <= y_max):
            not_intersecting = False
            # break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


def rhombus_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    rhombus_point_1 = [250, 25]
    rhombus_point_2 = [225, 40]
    rhombus_point_3 = [200, 25]
    rhombus_point_4 = [225, 10]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    if line_m_c[0] == math.inf:
        line_equation = x - line_m_c[1]  # x- x_intercept
    else:
        line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(None, rhombus_point_1, rhombus_point_2)
    if edge1_m_c[0] == math.inf:  # not needed btw, since rhombus sides are always at an angle, slope is not infinity
        line1 = x - edge1_m_c[1]
    else:
        line1 = y - (edge1_m_c[0] * x) - (edge1_m_c[1] + (augment_distance / 0.8575))

    edge2_m_c = find_line_slope_and_intercept(None, rhombus_point_2, rhombus_point_3)
    line2 = y - (edge2_m_c[0] * x) - (edge2_m_c[1] + (augment_distance / 0.8575))

    edge3_m_c = find_line_slope_and_intercept(None, rhombus_point_3, rhombus_point_4)
    line3 = y - (edge3_m_c[0] * x) - (edge3_m_c[1] - (augment_distance / 0.8575))

    edge4_m_c = find_line_slope_and_intercept(None, rhombus_point_4, rhombus_point_1)
    line4 = y - (edge4_m_c[0] * x) - (edge4_m_c[1] - (augment_distance / 0.8575))

    solution7 = solve([line1, line_equation], (x, y))
    # print("Intersection points with first line: ", solution7)
    solution8 = solve([line2, line_equation], (x, y))
    # print("Intersection points with second line: ", solution8)
    solution9 = solve([line3, line_equation], (x, y))
    # print("Intersection points with third line: ", solution9)
    solution10 = solve([line4, line_equation], (x, y))
    # print("Intersection points with Fourth line: ", solution10)

    solution = [solution7, solution8, solution9, solution10]

    not_intersecting = True
    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[x] <= x_max) and (y_min <= root[y] <= y_max):
            not_intersecting = False
            # break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


def polygon_right_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [100, 150]
    nonconvex_point_2 = [75, 185]
    nonconvex_point_3 = [60, 185]
    nonconvex_point_4 = [50, 150]
    nonconvex_point_5 = [75, 120]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(None, nonconvex_point_1, nonconvex_point_2)
    line1 = y - (edge1_m_c[0] * x) - (edge1_m_c[1] + (augment_distance / 0.58124))

    edge2_m_c = find_line_slope_and_intercept(None, nonconvex_point_2, nonconvex_point_3)
    line2 = y - (edge2_m_c[0] * x) - (edge2_m_c[1] + (augment_distance / 1))

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(None, nonconvex_point_3, nonconvex_point_4)
    line3 = y - (edge3_m_c[0] * x) - (edge3_m_c[1] + (augment_distance / 0.27472))

    edge4_m_c = find_line_slope_and_intercept(None, nonconvex_point_4, nonconvex_point_5)
    line4 = y - (edge4_m_c[0] * x) - (edge4_m_c[1] - (augment_distance / 0.64018))

    edge5_m_c = find_line_slope_and_intercept(None, nonconvex_point_5, nonconvex_point_1)
    line5 = y - (edge5_m_c[0] * x) - (edge5_m_c[1] - (augment_distance / 0.640184))

    solution11 = solve([line1, line_equation], (x, y))
    # print("Intersection points with first line: ", solution11)
    solution12 = solve([line2, line_equation], (x, y))
    # print("Intersection points with second line: ", solution12)
    solution13 = solve([line3, line_equation], (x, y))
    # print("Intersection points with third line: ", solution13)
    solution14 = solve([line4, line_equation], (x, y))
    # print("Intersection points with Fourth line: ", solution14)
    solution15 = solve([line5, line_equation], (x, y))
    # print("Intersection points with Fourth line: ", solution15)

    solution = [solution11, solution12, solution13, solution14, solution15]

    not_intersecting = True
    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[x] <= x_max) and (y_min <= root[y] <= y_max):
            not_intersecting = False
            break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


def polygon_left_intersection(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [50, 150]
    nonconvex_point_2 = [60, 185]
    nonconvex_point_3 = [25, 185]
    nonconvex_point_4 = [20, 120]

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(None, nonconvex_point_1, nonconvex_point_2)
    line1 = y - (edge1_m_c[0] * x) - (edge1_m_c[1] - (augment_distance / 0.27472))

    edge2_m_c = find_line_slope_and_intercept(None, nonconvex_point_2, nonconvex_point_3)
    line2 = y - (edge2_m_c[0] * x) - (edge2_m_c[1] + (augment_distance / 1))

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(None, nonconvex_point_3, nonconvex_point_4)
    line3 = y - (edge3_m_c[0] * x) - (edge3_m_c[1] + (augment_distance / 0.0767))

    edge4_m_c = find_line_slope_and_intercept(None, nonconvex_point_4, nonconvex_point_1)
    line4 = y - (edge4_m_c[0] * x) - (edge4_m_c[1] - (augment_distance / 0.7071))

    solution16 = solve([line1, line_equation], (x, y))
    # print("Intersection points with first line: ", solution16)
    solution17 = solve([line2, line_equation], (x, y))
    # print("Intersection points with second line: ", solution17)
    solution18 = solve([line3, line_equation], (x, y))
    # print("Intersection points with third line: ", solution18)
    solution19 = solve([line4, line_equation], (x, y))
    # print("Intersection points with Fourth line: ", solution19)

    solution = [solution16, solution17, solution18, solution19]

    not_intersecting = True
    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])


        if (x_min <= root[x] <= x_max) and (y_min <= root[y] <= y_max):
            not_intersecting = False
            break  # intersection present, not valid vector
        else:
            not_intersecting = True

    return not_intersecting


print(circular_intersection_check(1, 1, [197, 150], [253, 150]))  # should give False

print(ellipse_intersection_check(1, 1, [150, 130], [150, 70]))  # False case, vertical vector cuts the ellipse

print(rectangle_intersection(1, 1, [95, 20], [90, 50]))  # False case

print(rhombus_intersection(1, 1, [190, 25], [260, 25]))  # False case, horizontal vector cuts the rhombus
print(rhombus_intersection(1, 1, [225, 5], [225, 50]))  # False case, vertical vector cuts the rhombus

print(polygon_right_intersection(1, 1, [10, 110], [80, 187]))
print(polygon_left_intersection(1, 1, [10, 110], [80, 187]))
