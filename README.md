# NOTgraphVIZ
QT Program for visualize graph structure and set of algorithms on them. 

**NOTgraphVIZ** (NGV) my implementation of tool for visualize graph and algorithm on them.

## Functions
---
1.  Random generation of:
    * (Un)weighted [graph](https://en.wikipedia.org/wiki/Graph_(discrete_mathematics)) (can be directed or not)
    *   [Flow network](https://en.wikipedia.org/wiki/Flow_network)
2. Visualisation as:
   * [Adjacency matrix](https://en.wikipedia.org/wiki/Adjacency_matrix)
   * [Adjacency matrix](https://en.wikipedia.org/wiki/Adjacency_list)
   * [Kirchoff/Laplacian matrix](https://en.wikipedia.org/wiki/Laplacian_matrix) 
   * Diagrammic form (Schema)
3. Popular algorithms:
   *  Schimbell algorithm for extrema paths
   *  [DFS](https://en.wikipedia.org/wiki/Depth-first_search)
   *  [Dijkstra's algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
   *  [Bellman-Ford algorithm](https://en.wikipedia.org/wiki/Bellman–Ford_algorithm) with queue
4. Flow web algorithms: 
   * [Floyd-Warshall algorithm](https://en.wikipedia.org/wiki/Floyd–Warshall_algorithm)
   *  Max path flow and [Ford-Fulkerson](https://en.wikipedia.org/wiki/Ford–Fulkerson_algorithm)
   *  [Min-cost max-flow algorithm](https://en.wikipedia.org/wiki/Minimum-cost_flow_problem)
5. [Span tree](https://en.wikipedia.org/wiki/Spanning_tree) algorithms:
   * [Kruskal's algorithm](https://en.wikipedia.org/wiki/Kruskal%27s_algorithm) 
   * [Prim's algorithm](https://en.wikipedia.org/wiki/Prim%27s_algorithm)
   * [Prüfer sequence](https://en.wikipedia.org/wiki/Prüfer_sequence)
6. *NP*-tasks:
   * Is [eulerian](https://en.wikipedia.org/wiki/Eulerian_path)
   * Is [hamiltonian](https://en.wikipedia.org/wiki/Hamiltonian_path)
   * [Travelling salesman problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem) (TSP)

## Building
---
### Dependecies
- Qt5.x
- CMake >= 3.0

### To run
---
```bash
git clone https://github.com/timofeevAS/NOTgraphVIZ
cd NOTgraphVIZ
mkdir build
cd build
cmake ..
make
./NOTgraphVIZ
```
### How it works
---
![NGV](./media/ngv.gif)
### Documentation (Russian)
[Link](https://drive.google.com/file/d/1W2YMEHXAQJxBd_w1ZTIgWaDxW7v6yyiW/view?usp=sharing)