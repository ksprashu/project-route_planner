import math
import heapq


def get_distance(p1, p2): 
    """Returns the absolute distance between 2 points.
    
    Args:
        p1: [x, y] coords of point 1
        p2: [x, y] coords of point 2
    """
    
    # Find straight line distance between two points = sqrt[(x1-x2)^2 + (y1-y2)^]
    # ref: https://www.cuemath.com/geometry/distance-between-two-points/
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))


def shortest_path(M, start, goal):
    """Computes the shorted distances between the provided intersections.
    
    Args:
        M: Map data that contains the Intersections and available Roads
            M.intersections: A dictionary of intersections with 
                key = intersection number
                value = [x_coord, y_coord]
            M.roads: A list of lists, where each index contains possible connections 
                of that numbered intersection
        start: number of starting intersection
        goal: number of ending intersection
        
    Returns:
        An ordered list of intersections that makes up the shorted path.
    """
    
    # initiatize required data structures

    # this will be a minheap for optimal retrieval of closest node
    # ref: https://docs.python.org/3/library/heapq.html    
    frontiers = []
    
    explored = set()  # this will be a set for best lookup
    path = {}  # store the path as we discover, each node will have it's shortest predecessor
    
    # insert starting point with ideal distance h (which should always be the least)
    h = get_distance(M.intersections[start], M.intersections[goal])
    heapq.heappush(frontiers, (h, 0, start, None))
    
    while frontiers:
        # Pick the closest neighbour from frontiers
        # implement frontiers as a minheap will ensure this happens in O(1)
        distance, current_distance, current_node, previous_node = heapq.heappop(frontiers)
        
        # we are not removing duplicates from frontiers if there are multiple paths
        # so check if it has been explored before proceeding
        if current_node in explored:
            continue
        
        # Mark as explored, set the path, and do a goal test
        explored.add(current_node)
        path[current_node] = previous_node
        if current_node == goal:           
            break

        # Start by finding the immediate neighbours and add them to frontiers
        neighbours = (n for n in M.roads[current_node] if n not in explored)  # generator function
        for neighbour in neighbours:
            # distance to the frontier from current point
            goal_distance = get_distance(M.intersections[current_node], M.intersections[neighbour])  
            # ideal distance to goal from frontier point
            h_distance = get_distance(M.intersections[neighbour], M.intersections[goal])  

            # store total distance in frontiers
            # total distance is actual distance traversed (to current node) + goal distance + h
            total_distance = current_distance + goal_distance + h_distance
            frontier_distance = current_distance + goal_distance
            
            # frontier will store a tuple of (hd, d, n, p), where
            # hd = ideal distance to goal + what we already know
            # d = distance travelled to reach current node, 
            # n = current intersection number
            # p = previous intersection which forms the shortest path            
            heapq.heappush(frontiers, (total_distance, frontier_distance, neighbour, current_node))
            
    # if goal is in path, then we have been able to reach it
    # traverse back from goal to start (value = None) and build the path
    # reverse it since we need the path from start to goal
    if goal in path:
        route = []
        node = goal
        while node:
            route.append(node)
            node = path[node]
        return list(reversed(route))
        
    return None
