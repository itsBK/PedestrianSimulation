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
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(node.position, 1);
            foreach (Node neighbor in node.neighbors.Keys)
            {
                Gizmos.DrawLine(node.position, neighbor.position);
            }
        }

        //foreach (Pedestrian pedestrian in controller.activePedestrians)
        //{
        //    Gizmos.color = Color.blue;
        //    Gizmos.DrawRay(pedestrian.position, pedestrian.velocity);
        //    Gizmos.color = Color.black;
        //    Gizmos.DrawRay(pedestrian.position, pedestrian.steeringForces);
        //    Gizmos.color = Color.red;
        //    Gizmos.DrawRay(pedestrian.position, pedestrian.avoidance);
        //    Gizmos.color = Color.green;
        //    Gizmos.DrawRay(pedestrian.position, pedestrian.following);
        //}
    }

}