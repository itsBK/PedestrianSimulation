using System;
using System.Collections.Generic;
using UnityEngine;

public class Graph
{
    
    public struct NodeCost
    {
        public Node predecessor;
        public float hCost;
        public float gCost;
        public float TotalCost => hCost + gCost;
    }

    public List<Node> nodes;

    public Graph()
    {
        nodes = new List<Node>();
        Setup();
    }
    
    public void Setup()
    {
        Vector3[] positions = {
            new Vector3(10, 0, 90),
            new Vector3(50, 0, 90),
            new Vector3(90, 0, 50),
            new Vector3(50, 0, 10),
            new Vector3(10, 0, 10),
        };
        
        float[,] adjMatrix = {
            {000, 300, 000, 500,  20},
            {300, 000, 300, 000, 700},
            {000, 300, 000,  10, 000},
            {500, 000,  10, 000,  10},
            { 20, 700, 000,  10, 000}
        };
        SetGraphFrom(positions, adjMatrix);
    }

    /**
     * <param name="adjacencyMatrix">0 value represent no connection between the nodes</param>
     */
    public void SetGraphFrom(Vector3[] positions, float[,] adjacencyMatrix)
    {
        int dimension = positions.Length;
        for (int i = 0; i < dimension; i++)
        {
            nodes.Add(new Node(i, positions[i]));
        }
        
        for (int x = 0; x < dimension; x++)
        {
            Node node = GetNodeByID(x);
            for (int y = 0; y < dimension; y++)
            {
                if (adjacencyMatrix[x, y] > 0)
                    node.AddDirectedEdge(GetNodeByID(y), adjacencyMatrix[x, y]);
            }
        }
    }

    public Node GetNodeByID(int id)
    {
        foreach (Node node in nodes)
        {
            if (node.id == id)
            {
                return node;
            }
        }

        throw new KeyNotFoundException("the node(id=" + id + ") could not be found in list of node");
    }

    public float[,] GetAdjacencyMatrix()
    {
        int size = GetHighestID();
        float[,] adjacencyMatrix = new float[size, size];

        foreach (Node node in nodes)
        {
            foreach (KeyValuePair<Node, float> neighbor in node.neighbors)
            {
                adjacencyMatrix[node.id, neighbor.Key.id] = neighbor.Value;
            }
        }
        
        return adjacencyMatrix;
    }

    private int GetHighestID()
    {
        int id = 0;
        foreach (Node node in nodes)
        {
            id = node.id > id ? node.id : id;
        }

        return id;
    }

    public static bool AStar(Node start, Node end, out List<Node> result) {
        result = new List<Node>();
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
            foreach (KeyValuePair<Node, float> pair in current.neighbors) {
                Node neighbor = pair.Key;
                float edgeCost = pair.Value;
                
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
                        result.Add(tail);
                        allNodes.TryGetValue(tail, out NodeCost body);
                        
                        if (body.predecessor == start)
                            break;
                        
                        tail = body.predecessor;
                    }
                    
                    result.Add(start);
                    result.Reverse();
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