
# AI Searches  

This repository contains a Jupyter Notebook that demonstrates the implementation of various AI search algorithms. These algorithms form the backbone of many artificial intelligence applications and are essential for solving optimization and pathfinding problems.  



## Features  

The notebook includes implementations of the following search algorithms:  
- **Breadth-First Search (BFS)**: Explores all nodes at the present depth before moving to nodes at the next depth.  
- **Depth-First Search (DFS)**: Explores as far as possible along each branch before backtracking.  
- **Uniform Cost Search (UCS)**: A variant of BFS that considers the cost of the path.  
- **Greedy Best-First Search**: Prioritizes nodes based on a heuristic.  
- **A* Search**: Combines UCS and heuristic-based search for optimal pathfinding.  

 

## Prerequisites  

Make sure you have the following installed:  
- Python 3.8+  
- Jupyter Notebook  
- Required libraries:  
  - NumPy  
  - Matplotlib  

Install the dependencies using pip:  
```bash  
pip install numpy matplotlib  
```  

 

## How to Use  

1. Clone the repository:  
   ```bash  
   git clone https://github.com/ashithapallath/AI-Searches.git  
   cd AI-Searches  
   ```  

2. Open the Jupyter Notebook:  
   ```bash  
   jupyter notebook search.ipynb  
   ```  

3. Follow the step-by-step implementation of the search algorithms. Modify the sample problems to test the algorithms in various scenarios.  



## Examples  

### Example 1: Breadth-First Search (BFS)  
```python  
from collections import deque  

def bfs(graph, start):
    visited = set()
    queue = deque([start])
    while queue:
        node = queue.popleft()
        if node not in visited:
            visited.add(node)
            queue.extend(graph[node] - visited)
    return visited  
```  

### Example 2: A* Search  
```python  
import heapq  

def a_star_search(graph, start, goal, h):
    open_set = []
    heapq.heappush(open_set, (0 + h[start], start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0  

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break
        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + graph[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                heapq.heappush(open_set, (g_score[neighbor] + h[neighbor], neighbor))
```  



## Contribution  

Contributions are welcome! Feel free to fork the repository and submit pull requests for improvements, bug fixes, or additional features.  



## License  

This project is licensed under the MIT License.  

 

## Acknowledgments  

Thanks to the AI and open-source communities for providing resources and inspiration to build and share implementations of search algorithms.  

