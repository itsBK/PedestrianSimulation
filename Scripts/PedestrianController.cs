using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;
using Random = System.Random;

public class PedestrianController : MonoBehaviour
{

    public Graph graph;

    public GameObject pedestrianModelPrefab;
    public List<Pedestrian> activePedestrians;
    public int maxPedestriansCount = 40;
    public float minWalkingSpeed = 1;
    public float maxWalkingSpeed = 3;
    public float maxSteerForce = 1;
    public float viewRadius = 4;

    public Vector3 vehiclePosition;
    public float spawnRadius = 100;

    private void Start()
    {
        graph = new Graph();
        Random random = new Random();
        vehiclePosition = new Vector3();

        int nodesCount = graph.nodes.Count;

        for (int i = 0; i < maxPedestriansCount; i++)
        {
            Vector3 position = new Vector3(random.Next(100), 0, random.Next(100));
            GameObject pedestrianGO = Instantiate(pedestrianModelPrefab, transform);

            int a = random.Next(nodesCount - 1);
            int b = a;
            while (b == a)
                b = random.Next(nodesCount - 1);
            
            Pedestrian pedestrian = pedestrianGO.AddComponent<Pedestrian>();
            float walkingSpeed = (float) random.NextDouble() * (maxWalkingSpeed - minWalkingSpeed) + minWalkingSpeed;
            pedestrian.Setup(i, position, graph.nodes[a], graph.nodes[b], walkingSpeed, maxSteerForce, viewRadius);
            activePedestrians.Add(pedestrian);
        }
    }

    private void Update()
    {
        float deltaTime = Time.deltaTime;
        foreach (Pedestrian p in activePedestrians)
        {
            p.UpdateStatus(deltaTime);
        }
    }
}
