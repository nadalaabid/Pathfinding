//This code is based on Sebastian Lague's code on github
//link: https://github.com/SebLague/Pathfinding/blob/master/Episode%2003%20-%20astar/Assets/Scripts/Pathfinding.cs

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour {

	public Transform seeker, target;
	int cntNode;

	Grid grid;

	void Awake() {
		grid = GetComponent<Grid> ();
	}

	void Update() {
		//we compare between the different searching algorithms
		//and we output the time, and the number of nodes in the path
			var timer = new System.Diagnostics.Stopwatch();
			
			timer.Start();
			FindPathAstar(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by A* algorithm: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
			
			timer.Reset();
			cntNode = 0;
			
			timer.Start();
			FindPathAstarManhattan(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by A* Manhattan Heuristic algorithm: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
          
            timer.Reset();
			cntNode = 0;
			
			timer.Start();
			FindPathAstarEuclidean(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by A* Euclidean Heuristic algorithm: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
			
			timer.Reset();
			cntNode = 0;	

			timer.Start();
			FindPathDFS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by DFS: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
			
			timer.Reset();
			cntNode = 0;
			
			timer.Start();
			FindPathUCS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by UCS: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
          
            timer.Reset();
			cntNode = 0;
			
			timer.Start();
			FindPathBFS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by BFS: {timer.ElapsedMilliseconds} ms, \n path length: {cntNode}");
	
	}

	//This code is the same code as the one by Sebastian Lague github
	// It is the A star algorithm. 
	void FindPathAstar(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePathAstar(startNode,targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void RetracePathAstar(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode = path.Count;
		grid.pathAstar = path;

	}
	// This is the heuristic Sebastian used.
	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}
	//This function is the A star Algorithm with a different heuristic: Manhattan distance
	//The only difference between the two codes are the GetDistance function and the Retrace function
	// Manhattan distance: The distance between two points measured along axes at right angles. 
	//In a plane with p1 at (x1, y1) and p2 at (x2, y2), it is |x1 - x2| + |y1 - y2|.
	void FindPathAstarManhattan(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePathAstarManhattan(startNode,targetNode); //different from the initial code
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceManhattan(neighbour, targetNode); //different from the initial code
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void RetracePathAstarManhattan(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode = path.Count;
		grid.pathManhattan = path ; //to have different paths

	}

	int GetDistanceManhattan(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		return (dstX+dstY);
	}

	//This function is the A star Algorithm with a different heuristic: Euclidean distance
	//The only difference between the two codes are the GetDistance function and the Retrace function
	// Euclidean distance: In a plane with p1 at (x1, y1) and p2 at (x2, y2), it is result = sqrt ((X1-X2)^2 + (Y1-Y2)^2)
	void FindPathAstarEuclidean(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePathAstarEuclidean(startNode,targetNode); //different from the initial code
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceEuclidean(neighbour, targetNode); //different from the initial code
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void RetracePathAstarEuclidean(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode= path.Count;
		grid.pathEuclidean = path;

	}

	int GetDistanceEuclidean(Node nodeA, Node nodeB) {
		float dstX = Mathf.Pow(nodeB.gridX - nodeA.gridX, 2);
		float dstY = Mathf.Pow(nodeB.gridY - nodeA.gridY, 2);
		int result = (int)(Mathf.Sqrt(dstX + dstY)); 
		return (int) result;    
	}

	//Depth First search algorithm
	void FindPathDFS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Stack<Node> stack = new Stack<Node>();//stack for the fringe for LIFO data structure
		List<Node> explored = new List<Node>();//list for the explored nodes
		
		stack.Push(startNode);

		while (stack.Count != 0){ //while stack is not empty
			Node current = stack.Pop();
			explored.Add(current);//add to explored list
			foreach (Node neighbor in grid.GetNeighbours(current))
			{
				if (!neighbor.walkable || explored.Contains(neighbor)) // if the neighbor node is unwalkalbe or if we already explored, we continue exploring
					continue;
				if (neighbor.walkable & !explored.Contains(neighbor)){ // if neighbor is walkable and unexplored
					if (neighbor == targetNode){
						RetracePathDFS(startNode, targetNode); 
						return;
					}
					neighbor.parent = current; //set the parent of the neighbor to our current node to be able to retrace it
					stack.Push(neighbor); // last element is first on stack
				}
			}
		}
	}

	void RetracePathDFS(Node startNode, Node endNode){
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode){
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode = path.Count;
		grid.pathDFS = path;
	}
    //Uniform-cost search: find the optimal path by checking the costs of each move (same cost in all directions)
    void FindPathUCS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> unexplored = new List<Node>(); // fringe: keep track of unexplored nodes
		List<Node> explored = new List<Node>(); // keep track of explored nodes
		unexplored.Add(startNode);

		while (unexplored.Count > 0){ //while fringe is not empty

			// we choose the node with a lower cost than our current node as our current node
			Node current_node = unexplored[0];
			for (int i = 1; i < unexplored.Count; i++){
				if (unexplored[i].fCost <= current_node.fCost){ 
					if (unexplored[i].hCost < current_node.hCost)
						current_node = unexplored[i];
				}
			}

			unexplored.Remove(current_node); 
			explored.Add(current_node);

			if (current_node == targetNode){
				RetracePathUCS(startNode, targetNode);
				return;
			}
			foreach (Node neighbor in grid.GetNeighbours(current_node)){
				if (!neighbor.walkable || explored.Contains(neighbor))
					continue;
				int updatedCost = current_node.gCost; 
				if (updatedCost < neighbor.gCost || !unexplored.Contains(neighbor)){ // if the gcost of the neighbor is less than the current node, 
					neighbor.gCost = updatedCost; 
					neighbor.hCost = 0;
					neighbor.parent = current_node;//to retrace
					if (!unexplored.Contains(neighbor))
						unexplored.Add(neighbor);
				}
			}
		}
	}

	void RetracePathUCS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode = path.Count;
		grid.pathUCS = path;
	}
	//Breadth first search: FIFO implementation with queues
    void FindPathBFS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Queue<Node> queue = new Queue<Node>(); //create queue
		List<Node> exploredNodes = new List<Node>(); //create list of visited nodes
		
		
		queue.Enqueue(startNode); //enqueue the start node position

		while (queue.Count != 0){ // while queue is not empty
			Node currentNode = queue.Dequeue(); // take first node from queue
			if (currentNode == targetNode){ // if it's the target, retrace
				RetracePathBFS(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(currentNode))	{ // expand the current node and store in the queue if they are walkable and unexplored
				if (neighbour.walkable & !exploredNodes.Contains(neighbour)){
					exploredNodes.Add(neighbour);
					queue.Enqueue(neighbour); 
				}
			}
		}
	}

	void RetracePathBFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		this.cntNode = path.Count;
		grid.pathBFS = path;

	}


}
