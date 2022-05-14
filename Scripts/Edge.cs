public class Edge
{

    public enum EdgeType
    {
        Straight,
        Curve
    }
    
    public int id;
    public float cost;
    public EdgeType type;

    public float radius;
    public float angle;

    public Edge(int id, float cost, EdgeType type, float radius = 0, float angle = 0)
    {
        this.id = id;
        this.cost = cost;
        this.type = type;
        this.radius = radius;
        this.angle = angle;
    }

    public static Edge StraightEdge(int id, float cost)
    {
        return new Edge(id, cost, EdgeType.Straight);
    }

    public static Edge CurvedEdge(int id, float cost, float radius, float angle)
    {
        return new Edge(id, cost, EdgeType.Curve, radius, angle);
    }
}