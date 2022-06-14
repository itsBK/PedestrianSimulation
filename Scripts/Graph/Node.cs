using System.Collections.Generic;
using UnityEngine;

public class Node {
    
    public int id;
    public Vector3 position;
    public float radius;
    public Dictionary<Node, Edge> neighbors = new Dictionary<Node, Edge>();

    public Node(int id, Vector3 position, float radius) {
        this.id = id;
        this.position = position;
        this.radius = radius;
    }

    public void AddEdge(Node node, Edge edge) {
        neighbors.Add(node, edge);
        // add the connection both ways
        node.neighbors.Add(this, edge);
    }

    // to use in Adjacency Matrix
    public void AddDirectedEdge(Node node, Edge edge)
    {
        neighbors.Add(node, edge);
    }

    public Edge GetEdgeByNeighbor(Node neighbor)
    {
        if (neighbors.TryGetValue(neighbor, out Edge edge))
            return edge;

        throw new KeyNotFoundException("No edge found between node(" + id + ") and node(" + neighbor.id + ")");
    }
}