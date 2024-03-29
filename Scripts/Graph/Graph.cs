﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class Graph
{
    
    /**
     * To use in A-Star pathfinding algorithm
     */
    private struct NodeCost
    {
        public Node predecessor;
        public float hCost;
        public float gCost;
        public float TotalCost => hCost + gCost;
    }

    public List<Node> nodes;
    public List<Edge> edges;            // an edge can be used multiple times between different nodes

    public Graph()
    {
        Setup();
    }
    
    public void Setup()
    {
        // Node[] nodes = {
        //     new Node( 0, new Vector3(10, 0, 90), 5),
        //     new Node( 1, new Vector3(50, 0, 90), 5),
        //     new Node( 2, new Vector3(90, 0, 50), 5),
        //     new Node( 3, new Vector3(50, 0, 10), 5),
        //     new Node( 4, new Vector3(10, 0, 10), 5)
        // };
        //
        // Edge[] edges = {
        //     Edge.StraightEdge(0, 300, 5),
        //     Edge.StraightEdge(1, 500, 5),
        //     Edge.StraightEdge(2,  20, 5),
        //     Edge.StraightEdge(3, 700, 5),
        //     Edge.StraightEdge(4,  10, 5)
        // };
        // 
        // int[,] adjMatrix = {
        //     {-1,  0, -1,  1,  2},
        //     { 0, -1,  0, -1,  3},
        //     {-1,  0, -1,  4, -1},
        //     { 1, -1,  4, -1,  4},
        //     { 2,  3, -1,  4, -1}
        // };
        // SetGraphFrom(nodes, edges, adjMatrix);

        Node[] nodes2 = {
            new Node( 0, new Vector3(0, 0, 0), 5),
            new Node( 1, new Vector3(200, 0, 0), 5),
            new Node( 2, new Vector3(400, 0, 0), 5),
            new Node( 3, new Vector3(0, 0, 200), 5),
            new Node( 4, new Vector3(200, 0, 200), 5),
            new Node( 5, new Vector3(200, 0, 400), 5)
        };

        Edge[] edges2 = {
            Edge.StraightEdge(0, 200, 5),
            Edge.CurvedEdge(1, 200 * Mathf.PI / 2, 5, 200, new Vector3(200, 0, 0)),
            Edge.CurvedEdge(2, 200 * Mathf.PI / 2, 5, 200, new Vector3(200, 0, 200))
        };
        
        int[,] adjMatrix2 = {
            {-1,  0, -1,  0, -1, -1},
            { 0, -1,  0, -1,  0, -1},
            {-1,  0, -1, -1,  1, -1},
            { 0, -1, -1, -1,  0,  2},
            {-1,  0,  1,  0, -1,  0},
            {-1, -1, -1,  2,  0, -1}
        };
        SetGraphFrom(nodes2, edges2, adjMatrix2);
    }

    /**
     * <param name="adjacencyMatrix">-1 value represent no connection between the nodes</param>
     */
    public void SetGraphFrom(Node[] nodes, Edge[] edges, int[,] adjacencyMatrix)
    {
        this.nodes = nodes.ToList();
        this.edges = edges.ToList();
        
        int dimension = nodes.Length;
        for (int x = 0; x < dimension; x++)
        {
            Node node = GetNodeById(x);
            for (int y = 0; y < dimension; y++)
            {
                if (adjacencyMatrix[x, y] >= 0)
                    node.AddDirectedEdge(GetNodeById(y), GetEdgeById(adjacencyMatrix[x, y]));
            }
        }
    }

    public Node GetNodeById(int id)
    {
        foreach (Node node in nodes)
            if (node.id == id)
                return node;

        throw new KeyNotFoundException("the node(id=" + id + ") could not be found in list of nodes");
    }

    public Edge GetEdgeById(int id)
    {
        foreach (Edge edge in edges)
            if (edge.id == id)
                return edge;
        
        throw new KeyNotFoundException("the edge(id=" + id + ") could not be found in list of edges");
    }

    public int[,] GetAdjacencyMatrix()
    {
        int size = GetHighestId();
        int[,] adjacencyMatrix = new int[size, size];
        for (int x = 0; x < size; x++)
            for (int y = 0; y < size; y++)
                adjacencyMatrix[x, y] = -1;

        foreach (Node node in nodes)
        {
            foreach (KeyValuePair<Node, Edge> neighbor in node.neighbors)
            {
                adjacencyMatrix[node.id, neighbor.Key.id] = neighbor.Value.id;
            }
        }
        
        return adjacencyMatrix;
    }

    private int GetHighestId()
    {
        int id = 0;
        foreach (Node node in nodes)
            id = node.id > id ? node.id : id;

        return id;
    }

    /**
     * implementation of A-Star algorithm
     */
    public static bool FindPath(Node start, Node end, out List<Node> path) {
        path = new List<Node>();
        Dictionary<Node, NodeCost> allNodes = new Dictionary<Node, NodeCost>();
        Dictionary<Node, NodeCost> unvisited = new Dictionary<Node, NodeCost>();
        NodeCost initialNodeCost = new NodeCost {
            hCost = Vector3.Distance(start.position, end.position)
        };
        allNodes.Add(start, initialNodeCost);
        unvisited.Add(start, initialNodeCost);
        
        Node current = start;
        float currentGCost = 0;
        while (unvisited.Count != 0) {
            foreach (KeyValuePair<Node, Edge> pair in current.neighbors) {
                Node neighbor = pair.Key;
                float edgeCost = pair.Value.length;
                
                // newly discovered node ? add to the list
                if (!allNodes.ContainsKey(neighbor)) {
                    NodeCost nodeCost = new NodeCost {
                        predecessor = current,
                        hCost = Vector3.Distance(neighbor.position, end.position),
                        gCost = edgeCost + currentGCost
                    };
                    unvisited.Add(neighbor, nodeCost);
                    allNodes.Add(neighbor, nodeCost);
                    
                // node discovered but not visited yet ? check if this path is cheaper than the one visited    
                } else if (unvisited.ContainsKey(neighbor)) {
                    unvisited.TryGetValue(neighbor, out NodeCost nodeCost);
                    
                    if (nodeCost.gCost > currentGCost + edgeCost) {
                        nodeCost.gCost = currentGCost + edgeCost;
                        nodeCost.predecessor = current;
                        
                        allNodes[neighbor] = nodeCost;
                        unvisited[neighbor] = nodeCost;
                    }

                }

                if (neighbor.Equals(end)) {
                    Node tail = end;
                    while (allNodes.ContainsKey(tail)) {
                        path.Add(tail);
                        allNodes.TryGetValue(tail, out NodeCost body);
                        
                        if (body.predecessor == start)
                            break;
                        
                        tail = body.predecessor;
                    }
                    
                    path.Add(start);
                    path.Reverse();
                    return true;
                }
            }

            unvisited.Remove(current);

            float totalCost = Single.MaxValue;
            foreach (KeyValuePair<Node, NodeCost> node in unvisited) {
                if (totalCost > node.Value.TotalCost) {
                    totalCost = node.Value.TotalCost;
                    
                    current = node.Key;
                    currentGCost = node.Value.gCost;
                }
            }
        }
        
        return false;
    }
    
}