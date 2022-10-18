import algorithm


def app():
    print("Hello World!")
    adjacency_list = {
        'A': [('B', 1), ('C', 3), ('D', 7)],
        'B': [('D', 5)],
        'C': [('D', 12)],
        'D': [('E', 2)],
        'E': []
    }
    g = algorithm.Graph(adjacency_list)
    g.run_planner('A', 'D')


if __name__ == "__main__":
    app()
