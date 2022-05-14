using System;
using System.Collections.Generic;
using System.Linq;
using Vector3 = UnityEngine.Vector3;

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
    public List<Edge> edges;            // an edge can be used multiple times between different nodes

    public Graph()
    {
        nodes = new List<Node>();
        edges = new List<Edge>();
        Setup();
    }
    
    public void Setup()
    {
        (Vector3, float)[] nodes = {
            (new Vector3(10, 0, 90), 5),
            (new Vector3(50, 0, 90), 5),
            (new Vector3(90, 0, 50), 5),
            (new Vector3(50, 0, 10), 5),
            (new Vector3(10, 0, 10), 5),
        };

        Edge[] edges = {
            Edge.StraightEdge(0, 300),
            Edge.StraightEdge(1, 500),
            Edge.StraightEdge(2,  20),
            Edge.StraightEdge(3, 700),
            Edge.StraightEdge(4,  10),
        };
        
        int[,] adjMatrix = {
            {-1,  0, -1,  1,  2},
            { 0, -1,  0, -1,  3},
            {-1,  0, -1,  4, -1},
            { 1, -1,  4, -1,  4},
            { 2,  3, -1,  4, -1}
        };
        SetGraphFrom(nodes, edges, adjMatrix);
    }

    /**
     * <param name="adjacencyMatrix">0 value represent no connection between the nodes</param>
     */
    public void SetGraphFrom((Vector3, float)[] nodes, Edge[] edges, int[,] adjacencyMatrix)
    {
        int dimension = nodes.Length;
        for (int i = 0; i < dimension; i++)
        {
            (Vector3, float) node = nodes[i];
            this.nodes.Add(new Node(i, node.Item1, node.Item2));
        }
        this.edges = edges.ToList();
        
        for (int x = 0; x < dimension; x++)
        {
            Node node = GetNodeByID(x);
            for (int y = 0; y < dimension; y++)
            {
                if (adjacencyMatrix[x, y] >= 0)
                    node.AddDirectedEdge(GetNodeByID(y), GetEdgeByID(adjacencyMatrix[x, y]));
            }
        }
    }

    public Node GetNodeByID(int id)
    {
        foreach (Node node in nodes)
            if (node.id == id)
                return node;

        throw new KeyNotFoundException("the node(id=" + id + ") could not be found in list of nodes");
    }

    public Edge GetEdgeByID(int id)
    {
        foreach (Edge edge in edges)
            if (edge.id == id)
                return edge;
        
        throw new KeyNotFoundException("the edge(id=" + id + ") could not be found in list of edges");
    }

    public (Vector3, float)[] GetNodesAsVectors()
    {
        List<(Vector3, float)> nodes = new List<(Vector3, float)>();
        foreach (Node node in this.nodes)
        {
            nodes.Add((node.position, node.radius));
        }

        return nodes.ToArray();
    }

    public Edge[] GetEdges()
    {
        return edges.ToArray();
    }

    public int[,] GetAdjacencyMatrix()
    {
        int size = GetHighestID();
        int[,] adjacencyMatrix = new int[size, size];
        for (int x = 0; x < adjacencyMatrix.GetLength(0); x++)
            for (int y = 0; y < adjacencyMatrix.GetLength(1); y++)
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
            foreach (KeyValuePair<Node, Edge> pair in current.neighbors) {
                Node neighbor = pair.Key;
                float edgeCost = pair.Value.cost;
                
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