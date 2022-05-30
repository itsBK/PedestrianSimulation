using UnityEditor;
using UnityEngine;

public class GizmosDebug : MonoBehaviour
{

    private void OnDrawGizmos()
    {
        PedestrianController controller = FindObjectOfType<PedestrianController>();
        Handles.color = Color.green;
        Handles.DrawWireDisc(controller.vehicle.transform.position, new Vector3(0, 1, 0), controller.spawnRadius);
        
        foreach (Node node in controller.graph.nodes)
        {
            Handles.color = Color.red;
            Handles.DrawWireDisc(node.position, Vector3.up, node.radius);
            foreach (Node neighbor in node.neighbors.Keys)
            {
                Gizmos.DrawLine(node.position, neighbor.position);
            }
        }

        foreach (Pedestrian pedestrian in controller.activePedestrians)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(pedestrian.position, pedestrian.velocity);
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(pedestrian.position, pedestrian.acceleration);
            Handles.color = Color.blue;
            Handles.DrawWireDisc(pedestrian.position, Vector3.up, pedestrian.viewRadius);
            Handles.color = Color.red;
            Handles.DrawWireDisc(pedestrian.position, Vector3.up, pedestrian.safeZone);
            
            Vector3 direction = pedestrian.velocity.normalized;
            Gizmos.color = Color.red;
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.safeZone, 11));
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.safeZone, 12));
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.viewRadius, 35));
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.viewRadius, 36));
            Gizmos.color = Color.green;
            Gizmos.DrawLine(pedestrian.position, pedestrian.currentGoal.position);
            // Gizmos.color = Color.red;
            // Gizmos.DrawRay(pedestrian.position, pedestrian.avoidance);
            // Gizmos.color = Color.yellow;
            // Gizmos.DrawRay(pedestrian.position, pedestrian.seek);direction
        }
    }

}