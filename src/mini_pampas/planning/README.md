#  Path Planning Algorithms for Autonomous Racing
## 1. Introduction
Documentation for path planning, analyzing potential algorithms to develop code that receives coordinates from the Perception system of the 3D-printed cones that delimit the track, determines the midpoints between these cones, and handles situations where cones are missing.

## 2. Integration With Other Systems
The Planning system receives the coordinates of the cones detected by the Perception system.

Using these coordinates, it determines a path through the track by calculating suitable midpoints between the cones, including situations where some cones are missing.

The planned path is then sent to the Control system, which is responsible for controlling the car's movement along the trajectory.


## 3. Comparison Between Algorithms
### 3.1 Dijkstra Algorithm 
Dijkstra's algorithm finds the shortest paths from a starting vertex to all other reachable vertices in a weighted graph.

A vertex is a point in a graph where edges connect, and each edge can have a weight representing distance or cost.

The algorithm works by selecting the unvisited vertex with the smallest known distance from the starting vertex and checking whether passing through it provides a shorter path to its neighbors.
#### Strengths
This algorithm can solve single-source shortest path problems in directed and undirected graphs with non-negative edge weights. It guarantees the shortest path when the graph and its weights are correctly defined.
#### Weaknesses
Dijkstra's algorithm does not work correctly with negative edge weights. In these cases, the Bellman-Ford algorithm can be used instead.

It also does not automatically react to sudden obstacles or pedestrians. If the environment changes, the graph may need to be updated and the path recalculated.

#### How It Works
 1. Set the starting vertex distance to zero and all other distances to infinity. Mark all vertices as unvisited.
 2. Choose the unvisited vertex with the smallest known distance.
 3. Calculate the distance to each unvisited neighboring vertex through the current vertex. If the new distance is shorter, update it.
 4. Mark the current vertex as visited and repeat the process with the next closest unvisited vertex.
 5. Stop when all reachable vertices have been visited, or when the destination has been reached if only one target is needed. The shortest path can be reconstructed by following the previous vertices.

#### Application to Autonomous Driving

In autonomous driving, Dijkstra's algorithm can be used to find the lowest-cost path between the vehicle's starting position and a destination in a graph representing the environment.

The graph consists of vertices representing possible positions and edges representing valid connections between them. Each edge has a non-negative weight, such as the distance between connected vertices.

Dijkstra explores the graph by selecting the unvisited vertex with the lowest accumulated cost until it reaches the destination.

However, the algorithm does not identify obstacles or generate the graph by itself. It requires information from other systems and may need to recalculate the path when the environment changes.

#### Code Example

The following example demonstrates how to use Dijkstra's algorithm with the NetworkX library.

```python
import networkx as nx

G = nx.path_graph(5)
print(nx.dijkstra_path(G, 0, 4))
```

Output:

```text
[0, 1, 2, 3, 4]
```
### 3.2 Delaunay Triangulation

**Faces:** Triangular regions formed by edges.

**Edges:** Line segments connecting vertices.

**Convex:** A set is convex if the line segment
connecting any two points in the set lies
entirely within the set.

**Vertices:** Points where edges meet.



                   
```                 
     ^
    / \
   /   \
  /     \       
 /       \           
 ---------   
```      
A triangulation of a set of points is formed by connecting the points so that all faces are triangles and the outer edges form the convex hull of the points.

Delaunay triangulation is a special type of triangulation that connects points to form triangles in such a way that no point lies inside the circumcircle of any triangle.
#### Strengths
This method tends to produce better-shaped triangles by maximizing the minimum angle and avoiding long, skinny triangles when possible. However, it can still produce skinny triangles in some cases.
#### Weaknesses
It is harder to implement, requires more complex calculations, and may still produce skinny triangles, especially near the outer boundary.
#### Application to Autonomous Driving
In an autonomous racing car, the detected cones can be represented as points.

Delaunay triangulation can connect these points to form triangles. The algorithm can then be combined with other methods to identify connections between cones on opposite sides of the track.

The midpoints of suitable cone pairs can be used to generate a path through the track.
#### Code Example — Midpoint Calculation

The following MATLAB code demonstrates how to extract edges
from an existing triangulation, filter selected edges,
and calculate their midpoints.

```matlab
xPo = TO.Points(:,1);
yPo = TO.Points(:,2);

E = edges(TO); % triangulation edges

iseven = rem(E, 2) == 0; % neglect boundary edges
Eeven = E(any(iseven,2),:);

isodd = rem(Eeven,2) ~= 0;
Eodd = Eeven(any(isodd,2),:);

xmp = ((xPo(Eodd(:,1)) + xPo(Eodd(:,2)))/2);
ymp = ((yPo(Eodd(:,1)) + yPo(Eodd(:,2)))/2);

Pmp = [xmp ymp]; % midpoint coordinates
```
**Note:** This example assumes that the Delaunay triangulation has already been created and stored in the variable `TO`.

The edge filtering depends on the ordering of the cone vertices in the original example.

The calculated midpoints must also be ordered and validated
before generating a continuous trajectory.

### 3.3 The A* Algorithm
The A* algorithm is used to find paths in weighted graphs, searching for the shortest path between a starting node and a goal node.
#### Strengths
Unlike Dijkstra's algorithm, A* does not explore nodes based only on the distance traveled. Instead, it considers both the distance traveled from the starting node and a heuristic estimate of the remaining distance to the goal.
#### Weaknesses
One weakness of the A* algorithm is that its efficiency depends on the heuristic used. If the heuristic is not accurate enough, the algorithm may explore unnecessary nodes, increasing memory usage and execution time.

#### How It Works
The A* algorithm uses a graph representing the environment to find a path from a starting point to a destination.
1. Set the starting node's cost to zero and calculate its estimated distance to the destination using a heuristic.
2. Select the node with the lowest estimated total cost, considering both the distance traveled and the estimated remaining distance.
3. Examine its neighboring nodes and update their costs if a lower-cost path is found.
4. Repeat the process until the destination is reached or no valid path remains.
5. Reconstruct the path by following the previous nodes.

#### Code Example

The following example demonstrates how to use the A* algorithm
with the NetworkX library, using a grid graph and a heuristic
based on Euclidean distance.

```python
import networkx as nx

G = nx.grid_graph(dim=[3, 3])

nx.set_edge_attributes(G, {e: e[1][0] * 2 for e in G.edges()}, "cost")

def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

print(nx.astar_path(G, (0, 0), (2, 2), heuristic=dist, weight="cost"))
```

Output:

```text
[(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
```
**Note:** The A* algorithm uses the Euclidean distance as a heuristic to estimate the remaining cost to the destination. However, some edges in this example have zero cost, so this heuristic may overestimate the actual remaining cost. Therefore, the algorithm is not guaranteed to find the lowest-cost path in all cases.
#### Application to Autonomous Driving
In autonomous racing, A* can be used to find a path between the car's current position and a destination using a graph built from environmental information.

The algorithm considers both the cost of the path already traveled and an estimate of the remaining cost to the destination.

If new obstacles are detected, the graph can be updated and A* can be executed again to calculate a new path. However, the planned path must still be checked to ensure that it is safe and suitable for the vehicle's steering and movement limits.

### 3.4 Algorithm Comparison

| Algorithm | Main Function | Limitation |
|---|---|---|
| Dijkstra | Finds lowest-cost paths in weighted graphs | Requires a graph |
| Delaunay | Creates a geometric triangulation of points | Requires filtering to identify valid cone connections |
| A* | Finds paths using a heuristic | Requires a graph and an appropriate heuristic |

## 4. Conclusion
After analyzing the three algorithms, it is possible to conclude that both Dijkstra and A* find the lowest-cost paths between vertices in a graph. Therefore, both algorithms require a previously constructed graph.

On the other hand, Delaunay triangulation organizes the received points geometrically and connects them to form triangles using their coordinates.
## 5. Proposed Approach

Delaunay triangulation is proposed as the most suitable initial approach because it does not require a previously constructed graph and can organize the coordinates received from the Perception system geometrically.

The resulting connections can be used to identify suitable cone pairs and calculate their midpoints, helping to generate a path through the track.

When cones are missing, additional geometric rules may be necessary to estimate a valid path using the remaining detected cones.

Possible criteria to investigate include cone spacing,
track continuity, and the removal of invalid connections.

### 5.1 Initial Implementation Plan

The initial implementation will use fictitious cone coordinates to simulate data received from the Perception system.

1. Create coordinate sets representing cones on both sides of the track.
2. Calculate the midpoints between corresponding cone pairs.
3. Visualize the calculated midpoints and the resulting path.
4. Apply Delaunay triangulation to investigate possible connections between cones.
5. Test how missing cones affect the calculated path.

The initial tests will be performed in Python. Integration with ROS 2 will be investigated after the basic calculations have been tested.

## 6. References

- [DataCamp — A* Algorithm](https://www.datacamp.com/pt/tutorial/a-star-algorithm)
- [Delaunay Triangulation](https://ianthehenry.com/posts/delaunay/)
- [W3Schools — Dijkstra](https://www.w3schools.com/dsa/dsa_algo_graphs_dijkstra.php)
- [Wiley — Research Article](https://onlinelibrary.wiley.com/doi/10.1155/2021/8881684)
- [NetworkX — Documentation](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html)
- [MathWorks — Formula Student Driverless](https://blogs.mathworks.com/student-lounge/2022/10/03/path-planning-for-formula-student-driverless-cars-using-delaunay-triangulation/)
- [NetworkX — A* Algorithm](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.astar.astar_path.html)