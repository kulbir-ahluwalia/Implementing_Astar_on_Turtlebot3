from sympy import solve, Poly, Eq, Function, exp
from sympy.abc import x, y, z, a, b

def find_line_slope_and_intercept(test_point_coord, line_point_1, line_point_2):
    slope = (line_point_2[1] - line_point_1[1]) / (line_point_2[0] - line_point_1[0])
    y_intercept = line_point_1[1] - (slope * line_point_1[0])
    # print(slope,intercept)
    return slope, y_intercept


def circular_intersection_check(clearance, radius_rigid_robot, parent_coord, child_coord):
    augment_distance = radius_rigid_robot + clearance

    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])
    #print(line_equation)

    circle_center = (225, 150)
    equation_1 = (x - 225) ** 2 + (y - 150) ** 2 - (25 + augment_distance) ** 2
    equation_2 = line_equation
    # the following equation will output solutions in the form of a list
    solution = solve([equation_1, equation_2], (x, y))
    #print(solution)

    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])

        if (x_min <= root[0] <= x_max) and (y_min <= root[1] <= y_max):
            return False  # intersection present, not valid vector
        else:
            return True



circular_intersection_check(1,1,[197,150],[253,150])


def polygon_intersection_check(clearance, radius_rigid_robot, parent_coord, child_coord):

    augment_distance = radius_rigid_robot + clearance
    line_m_c = find_line_slope_and_intercept(None, child_coord, parent_coord)
    line_equation = y - (line_m_c[0] * x) - (line_m_c[1])
    # print(line_equation)

    rectangle_point_1 = [100, 38.66025]
    rectangle_point_2 = [35.0481, 76.1603]
    rectangle_point_3 = [30.0481, 67.5]
    rectangle_point_4 = [95, 30]

    edge1_m_c_rectangle = find_line_slope_and_intercept(None, rectangle_point_1, rectangle_point_2)
    line1 = y - (edge1_m_c_rectangle[0] * x) - (
            edge1_m_c_rectangle[1] + (augment_distance * 2 / (3 ** 0.5)))


    edge2_m_c_rectangle = find_line_slope_and_intercept(None, rectangle_point_2, rectangle_point_3)
    line2 = y - (edge2_m_c_rectangle[0] * x) - (edge2_m_c_rectangle[1] + (augment_distance * 2))


    edge3_m_c_rectangle = find_line_slope_and_intercept(None, rectangle_point_3, rectangle_point_4)
    line3 = y - (edge3_m_c_rectangle[0] * x) - (
            edge3_m_c_rectangle[1] - (augment_distance * 2 / (3 ** 0.5)))


    edge4_m_c_rectangle = find_line_slope_and_intercept(None, rectangle_point_4, rectangle_point_1)
    line4 = y - (edge4_m_c_rectangle[0] * x) - (edge4_m_c_rectangle[1] - (augment_distance * 2))

    lines = [edge1_m_c_rectangle, edge2_m_c_rectangle, edge3_m_c_rectangle, edge4_m_c_rectangle ]



    equation_1 = line1
    equation_2 = line_equation
    # the following equation will output solutions in the form of a list
    # solution = solve([equation_1, equation_2], (x, y), dict=True)
    solution = solve([equation_1, equation_2],  dict=True)

    print(solution)

    for root in solution:
        x_max = max(parent_coord[0], child_coord[0])
        x_min = min(parent_coord[0], child_coord[0])
        y_max = max(parent_coord[1], child_coord[1])
        y_min = min(parent_coord[1], child_coord[1])

        if (x_min <= root[x] <= x_max) and (y_min <= root[y] <= y_max):
            return False  # intersection present, not valid vector
        else:
            return True

polygon_intersection_check(1,1,[95,20],[97,35])