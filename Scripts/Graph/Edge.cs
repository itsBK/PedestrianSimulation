using UnityEngine;

public class Edge
{

    public enum EdgeType
    {
        Straight,
        Curve
    }
    
    public int id;
    public float length;
    public float width;
    public EdgeType type;

    public float radius;
    public Vector3 center;

    public Edge(int id, float length, float width, EdgeType type, float radius = 0, Vector3 center = new Vector3())
    {
        this.id = id;
        this.length = length;
        this.width = width;
        this.type = type;
        
        this.radius = radius;
        this.center = center;
    }

    public static Edge StraightEdge(int id, float length, float width)
    {
        return new Edge(id, length, width, EdgeType.Straight);
    }

    public static Edge CurvedEdge(int id, float length, float width, float radius, Vector3 center)
    {
        return new Edge(id, length, width, EdgeType.Curve, radius, center);
    }
}