//This code is based on Sebastian Lague's code on github
//link: https://github.com/SebLague/Pathfinding/blob/master/Episode%2003%20-%20astar/Assets/Scripts/Node.cs
using UnityEngine;
using System.Collections;

public class Node
{

	public bool walkable;
	public Vector3 worldPosition;
	public int gridX;
	public int gridY;

	public int gCost;
	public int hCost;
	public Node parent;

	public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY){
		walkable = _walkable;
		worldPosition = _worldPos;
		gridX = _gridX;
		gridY = _gridY;
	}

	public int fCost{
		get
		{
			return gCost + hCost;
		}
	}

}
