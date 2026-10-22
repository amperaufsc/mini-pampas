# Algorithms 
# Dijkstra algorithm 
Dijkstra's algorithm finds the shortest paths from a starting vertex to all other reachable vertices in a weighted graph.

A vertex is a point in a graph where edges connect, and each edge can have a weight representing distance or cost.

The algorithm works by selecting the unvisited vertex with the smallest known distance and checking whether passing through it provides a shorter path to its neighbors.
## Strengths
This algorithm can solve single-source shortest path problems in directed and undirected graphs with non-negative edge weights. It guarantees the shortest path when the graph and its weights are correctly defined.
## Weakness points 
Dijkstra's algorithm does not work correctly with negative edge weights. In these cases, the Bellman-Ford algorithm can be used instead.

It also does not automatically react to sudden obstacles or pedestrians. If the environment changes, the graph may need to be updated and the path recalculated.

## How it works

### 1-
Set the starting vertex distance to zero and all other distances to infinity. Mark all vertices as unvisited.
### 2-
Choose the unvisited vertex with the smallest known distance.
### 3-
Calculate the distance to each unvisited neighboring vertex through the current vertex. If the new distance is shorter, update it.
### 4-
Mark the current vertex as visited and repeat the process with the next closest unvisited vertex.
### 5-
Stop when all reachable vertices have been visited, or when the destination has been reached if only one target is needed. The shortest path can be reconstructed by following the previous vertices.

# Delaunay Triangulation

Faces:Are flat surfaces that form the outer part of a solid.

Edges: Are line segments formed where two faces meet.

Convex: A set is convex if the line segment connecting any two points in the set lies entirely within the set.

Vertex: Are points where two or more edges meet.



                   
```                 
     ^
    / \
   /   \
  /     \       
 /       \           
 ---------   
```      
Normal triangulation is formed by connecting a set of points so that all faces are triangles and the outer edges form the convex hull of the points.

Delaunay triangulation is a special type of triangulation that connects points to form triangles in such a way that no point lies inside the circumcircle of any triangle.
## Strengths
This method tends to produce better-shaped triangles by maximizing the minimum angle and avoiding long, skinny triangles when possible. However, it is more computationally complex and can still produce skinny triangles in some cases.
## Weakness points
It is harder to implement, requires more complex calculations, and may still produce skinny triangles, especially near the outer boundary.
## Application to Autonomous Driving
In an autonomous racing car, the detected cones can be represented as points.

Delaunay triangulation can connect these points to form triangles. The algorithm can then be combined with other methods to identify connections between cones on opposite sides of the track.

The midpoints of suitable cone pairs can be used to generate a path through the track.
# The A* Algorithm
The A algorithm* is used to find paths in weighted graphs, searching for the shortest path between a starting node and a goal node.
## Strengths
Unlike Dijkstra's algorithm, A* does not explore nodes based only on the distance traveled. Instead, it considers both the distance traveled from the starting node and a heuristic estimate of the remaining distance to the goal.
## Weakness points
One weakness of the A* algorithm is that its efficiency depends on the heuristic used. If the heuristic is not accurate enough, the algorithm may explore unnecessary nodes, increasing memory usage and execution time.

D* (Dynamic A*) is a path planning algorithm that helps an autonomous car find a path and adjust it when the environment changes.

The car uses information from its sensors to identify obstacles and create a map of its surroundings.
## 1-
The algorithm calculates an initial path from the car s current position to its destination.
## 2-
As the car moves, its sensors continuously detect changes in the environment.
## 3-
If a new obstacle is detected, the algorithm updates the affected parts of the map.
## 4-
D* recalculates the necessary parts of the path instead of planning everything again from the beginning.
## 5-
The car follows the updated path and continues adjusting it as new information becomes available.

In autonomous racing, D* can help the car adapt its planned route when obstacles or track conditions change. However, the path must still be checked to ensure that it is safe and suitable for the vehicle's steering and movement limits.