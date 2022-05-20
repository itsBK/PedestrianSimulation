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

    public bool rightCurve;
    public float radius;
    public float angle;

    public Edge(int id, float cost, float width, EdgeType type, float radius = 0, float angle = 0, bool rightCurve = false)
    {
        this.id = id;
        this.cost = cost;
        this.width = width;
        this.type = type;
        
        this.rightCurve = rightCurve;
        this.radius = radius;
        this.angle = angle;
    }

    public static Edge StraightEdge(int id, float cost, float width)
    {
        return new Edge(id, cost, width, EdgeType.Straight);
    }

    public static Edge CurvedEdge(int id, float cost, float width, float radius, float angle, bool rightCurve)
    {
        return new Edge(id, cost, width, EdgeType.Curve, radius, angle, rightCurve);
    }
}