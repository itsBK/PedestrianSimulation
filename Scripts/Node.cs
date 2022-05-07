using System.Collections.Generic;
using UnityEngine;

public class Node {
    
    public int id;
    public Vector3 position;
    public Dictionary<Node, float> neighbors = new Dictionary<Node, float>();

    public Node(int id, Vector3 position) {
        this.id = id;
        this.position = position;
    }

    public void AddEdge(Node node, float cost) {
        neighbors.Add(node, cost);
        // add the connection both ways
        node.neighbors.Add(this, cost);
    }

    // to use in Adjacency Matrix
    public void AddDirectedEdge(Node node, float cost)
    {
        neighbors.Add(node, cost);
    }

    /*
     * public override string ToString()
     * {
     *     return "[" + id + "] > ";
     * }
    */

}