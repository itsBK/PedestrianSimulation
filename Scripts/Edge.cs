using UnityEngine;

public class Edge
{

    public enum EdgeType
    {
        Straight,
        Curve
    }
    
    public int id;
    public float cost;
    public float width;
    public EdgeType type;

    public float radius;
    public Vector3 center;

    public Edge(int id, float cost, float width, EdgeType type, float radius = 0, Vector3 center = new Vector3())
    {
        this.id = id;
        this.cost = cost;
        this.width = width;
        this.type = type;
        
        this.radius = radius;
        this.center = center;
    }

    public static Edge StraightEdge(int id, float cost, float width)
    {
        return new Edge(id, cost, width, EdgeType.Straight);
    }

    public static Edge CurvedEdge(int id, float cost, float width, float radius, Vector3 center)
    {
        return new Edge(id, cost, width, EdgeType.Curve, radius, center);
    }
}