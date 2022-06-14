using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class GizmosDebug : MonoBehaviour
{

    private PedestrianController _controller;

    private void OnDrawGizmos()
    {
        if (_controller == null)
        {
            _controller = FindObjectOfType<PedestrianController>();
            return;
        }

        Handles.color = Color.green;
        Handles.DrawWireDisc(_controller.vehiclePosition, new Vector3(0, 1, 0), _controller.spawnRadius);
        Handles.color = Color.red;
        Handles.DrawWireDisc(_controller.vehiclePosition, new Vector3(0, 1, 0), _controller.vehicleViewRadius);
        
        foreach (Node node in _controller.graph.nodes)
        {
            Handles.color = Color.red;
            Handles.DrawWireDisc(node.position, Vector3.up, node.radius);
            Handles.color = Color.white;
            Gizmos.color = Color.white;
            foreach (KeyValuePair<Node, Edge> neighbor in node.neighbors)
            {
                if (neighbor.Value.type == Edge.EdgeType.Straight)
                    Gizmos.DrawLine(node.position, neighbor.Key.position);
                else
                    Handles.DrawWireArc(neighbor.Value.center, Vector3.up, neighbor.Key.position - neighbor.Value.center, 90, neighbor.Value.radius);
            }
        }

        foreach (Pedestrian pedestrian in _controller.activePedestrians)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(pedestrian.position, pedestrian.velocity);
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(pedestrian.position, pedestrian.acceleration);
            Handles.color = Color.blue;
            Handles.DrawWireDisc(pedestrian.position, Vector3.up, pedestrian.viewRadius);
            Handles.color = Color.red;
            Handles.DrawWireDisc(pedestrian.position, Vector3.up, pedestrian.slowDownRadius);
            
            Vector3 direction = pedestrian.velocity.normalized;
            Gizmos.color = Color.red;
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.slowDownRadius, 17));
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.slowDownRadius, 18));
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.viewRadius, 35));
            Gizmos.DrawRay(pedestrian.position, pedestrian.GetDirection(direction * pedestrian.viewRadius, 36));
        }
    }

}