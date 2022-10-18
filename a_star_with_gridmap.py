import numpy as np
import matplotlib.pyplot as plt
import math
import time

show_animation = True


class Node:
    def __init__(self, g=0, h=0, coordinate=None, parent=None):
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.coordinate = coordinate
        
    def reset_f(self):
        self.f = self.g + self.h
        
        
def get_h_cost(node_coordinate, goal):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    # TODO: L1 norm is used as cost, try L2 as an alternative
    h_cost = dx + dy
    return h_cost


def get_g_cost(fixed_node, update_node_coordinate):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    update_cost = math.hypot(dx, dy)
    g_cost = fixed_node.g + update_cost
    return g_cost

def boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, number_of_obstacles):
    """
    :param start: start coordinate
    :param goal: goal coordinate
    :param top_vertex: top right coordinate for boundary
    :param bottom_vertex: bottom left coordinate for boundary
    :param number_of_obstacles:
    :return: boundary_obstacle array, obstacle list
    """
    """
        --------- d ---------
        |                   |
        |                   |
        |                   |
        a                   c
        |                   |
        |                   |
        |                   |
        --------- b ---------
    """
    ay = list(range(bottom_vertex[1], top_vertex[1]))
    ax = [bottom_vertex[0]] * len(ay)
    cy = ay
    cx = [top_vertex[0]] * len(cy)
    bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
    by = [bottom_vertex[1]] * len(bx)
    dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
    dy = [top_vertex[1]] * len(dx)

    # generate random obstacles
    ob_x = np.random.randint(bottom_vertex[0] + 1,
                             top_vertex[0], number_of_obstacles).tolist()
    ob_y = np.random.randint(bottom_vertex[1] + 1,
                             top_vertex[1], number_of_obstacles).tolist()
    # stack x y coordinates
    x = ax + bx + cx + dx
    y = ay + by + cy + dy
    obstacles = np.vstack((ob_x, ob_y)).T.tolist()
    # remove start and goal coordinate in obstacle list
    obstacles = [coord for coord in obstacles if coord != start and coord != goal]
    obs_array = np.array(obstacles)
    boundary = np.vstack((x, y)).T
    bounded_obstacles = np.vstack((boundary, obs_array))
    return bounded_obstacles, obstacles


def find_neighbor(node, obstacles, closed):
    # generate neighbors
    obstacle_list = obstacles.tolist()
    neighbor: list = []
    for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
        for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
            if [x, y] not in obstacle_list:
                neighbor.append([x, y])
    # remove nodes that violate motion rule
    # 1. remove node itself
    neighbor.remove(node.coordinate)

    # 2. remove nodes that are between 2 cross obstacles, (TODO: due to manhattan distance)
    """
    top_neighbor = [node.coordinate[0], node.coordinate[1] + 1]
    bottom_neighbor = [node.coordinate[0], node.coordinate[1] - 1]
    left_neighbor = [node.coordinate[0] - 1, node.coordinate[1]]
    right_neighbor = [node.coordinate[0] + 1, node.coordinate[1]]
    top_left_neighbor = [node.coordinate[0] - 1, node.coordinate[1] + 1]
    top_right_neighbor = [node.coordinate[0] + 1, node.coordinate[1] + 1]
    bottom_left_neighbor = [node.coordinate[0] - 1, node.coordinate[1] - 1]
    bottom_right_neighbor = [node.coordinate[0] + 1, node.coordinate[1] - 1]

    if top_neighbor in obstacle_list and left_neighbor in obstacle_list and top_left_neighbor in neighbor:
        neighbor.remove(top_left_neighbor)
    if top_neighbor in obstacle_list and right_neighbor in obstacle_list and top_right_neighbor in neighbor:
        neighbor.remove(top_right_neighbor)
    if bottom_neighbor in obstacle_list and left_neighbor in obstacle_list and bottom_left_neighbor in neighbor:
        neighbor.remove(bottom_left_neighbor)
    if bottom_neighbor in obstacle_list and right_neighbor in obstacle_list and bottom_right_neighbor in neighbor:
        neighbor.remove(bottom_right_neighbor)
    """
    neighbor = [x for x in neighbor if x not in closed]
    return neighbor


def find_node_index(coordinate, node_list):
    for i in range(len(node_list)):
        if node_list[i].coordinate == coordinate:
            return i
    return 0


def find_path(open_list, closed_list, goal, obstacle):
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coordinate_list = [node.coordinate for node in open_list]
        closed_coordinate_list = [node.coordinate for node in closed_list]
        neighbors = find_neighbor(node, obstacle, closed_coordinate_list)
        for neighbor in neighbors:
            if neighbor in closed_list:
                continue
            elif neighbor in open_coordinate_list:
                idx = open_coordinate_list.index(neighbor)
                new_g_cost = get_g_cost(node, neighbor)
                if new_g_cost < open_list[idx].g:
                    open_list[idx].g = new_g_cost
                    open_list[idx].parent = node
                    open_list[idx].reset_f()
            else:
                new_node = Node(coordinate=neighbor,
                                parent=node,
                                g=get_g_cost(node, neighbor),
                                h=get_h_cost(neighbor, goal))
                open_list.append(new_node)
        closed_list.append(node)
        open_list.remove(node)
        open_list.sort(key=lambda x: x.f)
    return open_list, closed_list


def node_to_coordinate(node_list):
    coordinate_list = []
    for node in node_list:
        coordinate_list.append(node.coordinate)
    return coordinate_list


def check_node_coincide(closed_list_1, closed_list_2):
    """
    :param closed_list_1: 
    :param closed_list_2: 
    :return: the intersecting list of elements 
    """
    cl1 = node_to_coordinate(closed_list_1)
    cl2 = node_to_coordinate(closed_list_2)
    intersecting_list = [node for node in cl1 if node in cl2]
    return intersecting_list


def find_surroundings(coordinate, obstacles):
    boundary: list = []
    for x in range(coordinate[0] - 1, coordinate[0] + 2):
        for y in range(coordinate[1] - 1, coordinate[1] + 2):
            if [x, y] not in obstacles:
                boundary.append([x, y])
    return boundary


def get_borderline(surrounded_node_list, obstacles):
    borderline: list = []
    coordinates = node_to_coordinate(surrounded_node_list)
    for coordinate in coordinates:
        surroundings = find_surroundings(coordinate, obstacles)
        # TODO: might blow up
        for surrounding in surroundings:
            if surrounding not in coordinates:
                borderline.append(surrounding)
    borderline_array = np.array(borderline)
    return borderline_array


def get_path(node_list, coordinate):
    """

    :param node_list:
    :param coordinate:
    :return:
    """
    path = []
    idx = find_node_index(coordinate, node_list)
    node = node_list[idx]
    while node != node_list[0]:
        path.append(node.coordinate)
        node = node.parent
    path.append(node_list[0].coordinate)
    path_array = np.array(path)
    return path_array


def random_coordinate(bottom_vertex, top_vertex):
    x = np.random.randint(bottom_vertex[0] + 1, top_vertex[0] - 1)
    y = np.random.randint(bottom_vertex[1] + 1, top_vertex[1] - 1)
    return [x, y]


def draw(closed, start, goal, boundary):
    # plot the map
    if not closed.tolist():
        closed = np.array([goal])
    plt.cla()
    plt.gcf().set_size_inches(9, 7, forward=True)
    plt.axis('equal')
    plt.plot(closed[:, 0], closed[:, 1], '.g')
    plt.plot(boundary[:, 0], boundary[:, 1], 'sk')
    plt.plot(start[0], start[1], '^b', label='start')
    plt.plot(goal[0], goal[1], 'xb', label='goal')
    plt.legend()
    plt.pause(0.4)


def check_goal_reached(closed_list, goal):
    if goal in closed_list:
        return True
    else:
        return False


def draw_control(closed, flag, start, goal, boundary, obstacle):
    """
    control the plot process, evaluate if the searching finished
    flag == 0 : draw the searching process and plot path
    flag == 1 or 2 : start or end is blocked, draw the border line
    :param closed: 
    :param flag: 
    :param start: 
    :param goal: 
    :param boundary: 
    :param obstacle: 
    :return: 
    """
    
    stop_condition = 0
    closed_list = node_to_coordinate(closed)
    closed_array = np.array(closed_list)
    path = None
    if show_animation:
        draw(closed_array, start, goal, boundary)
    if flag == 0:
        goal_reached = check_goal_reached(closed_list, goal)
        if goal_reached:
            stop_condition = 1
            path = get_path(closed, goal)
            print("Found the path!")
            if show_animation:
                plt.plot(path[:, 0], path[:, 1], '-r')
                plt.title("Path found!", size=20, loc='center')
                plt.pause(0.4)
                plt.show()
    elif flag == 1:
        stop_condition = 1
        print("Start is blocked!")
        if show_animation:
            plt.title("Start is blocked!", size=20, loc='center')
            border = get_borderline(closed, obstacle)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.pause(0.4)
            plt.show()
    return stop_condition, path


def search_control(start_coord, goal_coord, boundary, obstacle):
    start = Node(coordinate=start_coord, parent=None, g=0, h=get_h_cost(start_coord, start_coord))
    goal = Node(coordinate=start_coord, parent=None, g=0, h=0)
    open_list = [start]
    closed_list = []
    flag = 0
    path = None
    target = goal_coord
    while True:
        time.sleep(0.2)
        open_list, closed_list = find_path(open_list, closed_list, goal_coord, boundary)
        if not open_list:
            flag = 1
            draw_control(closed_list, flag, start, goal, boundary, obstacle)
            break
        stop_condition, path = draw_control(closed_list, flag, start_coord, goal_coord, boundary, obstacle)
        if stop_condition:
            break
    return path


def main(number_of_obstacles = 1):
    print(__file__ + " start!!")

    top_vertex = [20, 20]
    bottom_vertex = [0, 0]

    # generate start and goal point randomly
    start = random_coordinate(bottom_vertex, top_vertex)
    goal = random_coordinate(bottom_vertex, top_vertex)

    boundary, obstacle = boundary_and_obstacles(start, goal, top_vertex,
                                                bottom_vertex,
                                                number_of_obstacles)
    path = search_control(start, goal, boundary, obstacle)
    if not show_animation:
        print("Path found:" + path)


if __name__ == '__main__':
    main(number_of_obstacles=200)


