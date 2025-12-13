from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        
        start_node.g = 0
        heapq.heappush(pq, (start_node.g, start_node))

        while pq:
            g, node = heapq.heappop(pq)

            if node.closed:
                continue

            if node.get_position() == goal_position:
                goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
                path = self.construct_path(goal_node)
                cost = goal_node.g
                self.node_grid.reset()
                return path, cost

            for successor_position in self.node_grid.get_successors(node.i, node.j):
                successor_node = self.node_grid.get_node(successor_position[0], successor_position[1])
                if successor_node.g > node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position) and not successor_node.closed:
                    successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position)
                    successor_node.parent = node
                    heapq.heappush(pq, (successor_node.g, successor_node))
            
            node.closed = True

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        
        start_node.g = 0
        start_node.f = start_node.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, node = heapq.heappop(pq)

            if node.closed:
                continue

            if node.get_position() == goal_position:
                goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
                path = self.construct_path(goal_node)
                cost = goal_node.g
                self.node_grid.reset()
                return path, cost

            for successor_position in self.node_grid.get_successors(node.i, node.j):
                successor_node = self.node_grid.get_node(successor_position[0], successor_position[1])
                if successor_node.g > node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position) and not successor_node.closed:
                    successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position)
                    successor_node.f = successor_node.distance_to(goal_position[0], goal_position[1])
                    successor_node.parent = node
                    heapq.heappush(pq, (successor_node.f, successor_node))
            
            node.closed = True

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        pq = []
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        
        start_node.g = 0
        start_node.f = start_node.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, node = heapq.heappop(pq)

            if node.closed:
                continue

            if node.get_position() == goal_position:
                goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
                path = self.construct_path(goal_node)
                cost = goal_node.g
                self.node_grid.reset()
                return path, cost

            for successor_position in self.node_grid.get_successors(node.i, node.j):
                successor_node = self.node_grid.get_node(successor_position[0], successor_position[1])
                if successor_node.g > node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position) and not successor_node.closed:
                    successor_node.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor_position)
                    successor_node.f = successor_node.g + successor_node.distance_to(goal_position[0], goal_position[1])
                    successor_node.parent = node
                    heapq.heappush(pq, (successor_node.f, successor_node))
            
            node.closed = True