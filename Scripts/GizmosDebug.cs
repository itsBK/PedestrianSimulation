using UnityEngine;

public class GizmosDebug : MonoBehaviour
{

    private void OnDrawGizmos()
    {
        PedestrianController controller = FindObjectOfType<PedestrianController>();
        foreach (Node node in controller.graph.nodes)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(node.position, 2);
            foreach (Node neighbor in node.neighbors.Keys)
            {
                Gizmos.DrawLine(node.position, neighbor.position);
            }
        }
    }

}