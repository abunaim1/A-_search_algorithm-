import math
import heapq

# Function to calculate Euclidean distance
def heuristic(node, goal, coords):
    return math.sqrt((coords[node][0] - coords[goal][0])**2 + (coords[node][1] - coords[goal][1])**2)

# Node state class for A* search
class NodeState:
    def __init__(self, node_id, parent, g, f):
        self.node_id = node_id
        self.parent = parent
        self.g = g
        self.f = f
        
    def __lt__(self, other):
        return self.f < other.f

# Function to read the input file and initialize data structures
def read_input(file_name):
    with open(file_name, 'r') as file:
        lines = file.readlines()

    V = int(lines[0].strip())
    coords = {}
    for i in range(1, V + 1):
        parts = lines[i].strip().split()
        node = parts[0]
        x, y = int(parts[1]), int(parts[2])
        coords[node] = (x, y)

    E = int(lines[V + 1].strip())
    adjlist = {}
    for i in range(V + 2, V + 2 + E):
        parts = lines[i].strip().split()
        node1, node2, cost = parts[0], parts[1], int(parts[2])
        if node1 not in adjlist:
            adjlist[node1] = []
        adjlist[node1].append((node2, cost))

    start_node = lines[V + 2 + E].strip()
    goal_node = lines[V + 3 + E].strip()

    return coords, adjlist, start_node, goal_node

# A* search algorithm
def a_star_search(coords, adjlist, start_node, goal_node):
    start_h = heuristic(start_node, goal_node, coords)
    start_state = NodeState(start_node, None, 0, start_h)
    minQ = []
    heapq.heappush(minQ, start_state)
    visited = set()

    while minQ:
        curr_state = heapq.heappop(minQ)
        
        if curr_state.node_id in visited:
            continue
        visited.add(curr_state.node_id)
        
        if curr_state.node_id == goal_node:
            path = []
            cost = curr_state.g  # Save the cost before curr_state becomes None
            while curr_state:
                path.append(curr_state.node_id)
                curr_state = curr_state.parent
            path.reverse()
            return path, cost

        for (adj_node, cost) in adjlist.get(curr_state.node_id, []):
            if adj_node in visited:
                continue
            g = curr_state.g + cost
            h = heuristic(adj_node, goal_node, coords)
            f = g + h
            new_state = NodeState(adj_node, curr_state, g, f)
            heapq.heappush(minQ, new_state)
    
    return None, float('inf')

# Main function to execute the A* search and print the result
def main():
    coords, adjlist, start_node, goal_node = read_input('input.txt')
    path, cost = a_star_search(coords, adjlist, start_node, goal_node)
    
    if path:
        print("Solution path:", "–".join(path))
        print("Solution cost:", cost)
    else:
        print("No path found.")

if __name__ == '__main__':
    main()
