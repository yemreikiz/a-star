# the A* algorithm module
def get_heuristic(node):
    heuristics = {
        'A': 1,
        'B': 1,
        'C': 1,
        'D': 1,
        'E': 1,
        'F': 1
    }
    return heuristics[node]


class Graph:
    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, vertices):
        return self.adjacency_list[vertices]

    def run_planner(self, start, goal):
        open_list = set([start])
        closed_list = set([])
        distance = {}
        for n in self.adjacency_list:
            distance[n] = 100
        distance[start] = 0
        parent_list = {start: start}

        while len(open_list) > 0:
            current_node = None
            # find the node within the open set with the shortest distance from start
            for node in open_list.copy():
                if current_node is None or \
                        (distance[node] + get_heuristic(node) <
                         distance[current_node] + get_heuristic(current_node)):
                    current_node = node
            # if there is no nodes left to investigate, fail
            if current_node is None:
                print("Path does not exist!")
                return None

            # if we have reached goal, return path to goal
            if current_node is goal:
                path = []
                while parent_list[current_node] is not current_node:
                    path.append(current_node)
                    current_node = parent_list[current_node]
                path.append(start)
                path.reverse()
                print('Path found {}'.format(path))
                return path

            # traverse the neighbors of the current node
            for (new_node, cost) in self.get_neighbors(current_node):
                new_distance = distance[current_node] + cost
                if new_distance < distance[new_node]:
                    parent_list[new_node] = current_node
                    distance[new_node] = distance[current_node] + cost
                if new_node not in open_list and new_node not in closed_list:
                    open_list.add(new_node)
                elif new_node in closed_list:
                    closed_list.remove(new_node)
                    open_list.add(new_node)
            # move the node with all neighbors visited node from open lists to closed list
            open_list.remove(current_node)
            closed_list.add(current_node)
        # if the code executes this line, then the input is wrong
        print("Error! Run with a solvable graph...")




