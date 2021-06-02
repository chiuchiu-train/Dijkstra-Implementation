from collections import defaultdict
import heapq

# Helper function that makes adjacency list as a dictionary {node:(adjacent node, weight)}
def make_adj_lst(map):
    adj_lst = defaultdict(list)
    for (u,v,weight) in map:
        adj_lst[u].append((weight,v))
        adj_lst[v].append((weight,u))
    return adj_lst

def allKeys(list):
    keyList = []
    for item in list:
        if item[0] not in keyList:
            keyList.append(item[0])
        if item[1] not in keyList:
            keyList.append(item[1])
    return keyList

# Input is (list of tuples, source)
def shortestPath(map, source, goal):
    # Set to store visited nodes
    visited = set()
    # Turns list of tuples into a nested dictionary
    adj_lst = make_adj_lst(map)
    keyList = allKeys(map)
    pathList = []

    # Initialize all vertices besides source to infinity
    # distances keeps track of distance from source
    distances = defaultdict(float)     # dictionary consisting of {vertices:inf}
    for vertex in keyList:
        distances[vertex] = float('inf')
    distances[source] = 0
    Q = []  # empty priority queue
    Q = [(0,source)] # load source into priority queue
    predecessors = {} # to store predecessors {vertex:pred}

    # Main loop
    while Q:
        heapq.heapify(Q)
        (current_distance, current_vertex) = heapq.heappop(Q)
        # don't want to take longer path
        #f current_distance > distances[current_vertex]:
            #continue
        if current_vertex in visited:
            continue
        visited.add(current_vertex)
        #pathList.append(current_vertex)
        if current_vertex == goal:
            break

        # Iterates through adjacent vertices of current_vertex
        for (weight, neighbor) in adj_lst[current_vertex]:
            if neighbor in visited:
                continue
            possible_dist = current_distance + weight
            # New path, executes if it's shorter than current path
            if possible_dist < distances[neighbor]:
                # Change inf to distance from source
                distances[neighbor] = possible_dist
                predecessors[neighbor] = current_vertex
                heapq.heappush(Q, (possible_dist, neighbor))


    shortest_path = []
    last = current_vertex
    while last != source:
        shortest_path.append(last)
        last = predecessors[last]
    shortest_path.reverse()
    shortest_path.insert(0,source)

    ##print(shortest_path)
    #end = goal
    #while end is not None:
        #shortest_path.append(end)
        #end = parents[end]
    return shortest_path

def dijkstra(map, office):
    path_dict = {}
    keyList = allKeys(map)
    for node in keyList:
        path_dict[node] = shortestPath(map, office, node)
    return path_dict
