
from collections import deque
import heapq

# -----------------------------
# Input Graph
# -----------------------------
graph = {}

n = int(input("Enter number of roads: "))

print("\nEnter roads in the format:")
print("Source Destination Distance")

for i in range(n):
    src, dest, dist = input(f"Road {i+1}: ").split()
    dist = int(dist)

    if src not in graph:
        graph[src] = {}

    if dest not in graph:
        graph[dest] = {}

    # Undirected graph
    graph[src][dest] = dist
    graph[dest][src] = dist

# -----------------------------
# Input Heuristic Values
# -----------------------------
heuristic = {}

print("\nEnter heuristic value for each node:")

for node in graph:
    h = int(input(f"Heuristic({node}) = "))
    heuristic[node] = h

# -----------------------------
# BFS
# -----------------------------
def bfs(start, goal):
    queue = deque([[start]])
    visited = set()

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == goal:
            return path

        if node not in visited:
            visited.add(node)

            for neighbor in graph[node]:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)

    return None

# -----------------------------
# DFS
# -----------------------------
def dfs(node, goal, visited=None, path=None):
    if visited is None:
        visited = set()

    if path is None:
        path = []

    visited.add(node)
    path.append(node)

    if node == goal:
        return path

    for neighbor in graph[node]:
        if neighbor not in visited:
            result = dfs(neighbor, goal, visited, path.copy())
            if result:
                return result

    return None

# -----------------------------
# Uniform Cost Search
# -----------------------------
def ucs(start, goal):
    pq = [(0, start, [start])]
    visited = set()

    while pq:
        cost, node, path = heapq.heappop(pq)

        if node == goal:
            return path, cost

        if node not in visited:
            visited.add(node)

            for neighbor, weight in graph[node].items():
                heapq.heappush(
                    pq,
                    (cost + weight, neighbor, path + [neighbor])
                )

    return None, None

# -----------------------------
# A* Search
# -----------------------------
def astar(start, goal):
    pq = [(heuristic[start], 0, start, [start])]
    visited = set()

    while pq:
        f, g, node, path = heapq.heappop(pq)

        if node == goal:
            return path, g

        if node not in visited:
            visited.add(node)

            for neighbor, weight in graph[node].items():
                new_g = g + weight
                new_f = new_g + heuristic[neighbor]

                heapq.heappush(
                    pq,
                    (new_f, new_g, neighbor, path + [neighbor])
                )

    return None, None

# -----------------------------
# User Input for Search
# -----------------------------
print("\nAvailable Nodes:", list(graph.keys()))

start = input("Enter Source Node: ")
goal = input("Enter Destination Node: ")

print("\n----- BFS -----")
print("Path:", bfs(start, goal))

print("\n----- DFS -----")
print("Path:", dfs(start, goal))

print("\n----- UCS -----")
path, cost = ucs(start, goal)
print("Path:", path)
print("Cost:", cost)

print("\n----- A* Search -----")
path, cost = astar(start, goal)
print("Path:", path)
print("Cost:", cost)
