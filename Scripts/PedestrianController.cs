using System;
using System.Collections.Generic;
using UnityEngine;
using Random = System.Random;

public class PedestrianController : MonoBehaviour
{

    public Graph graph;
    public int nodeCount;

    public GameObject pedestrianModelPrefab;
    // TODO: change to Vector3 vehiclePosition
    public GameObject vehicle;
    public Vector3 vehiclePosition;
    public List<Pedestrian> activePedestrians;
    public List<int> availableIDs;
    public int maxPedestriansCount = 40;
    public float minWalkingSpeed = 4;
    public float maxWalkingSpeed = 6;
    public float viewRadius = 10;
    public float spawnRadius = 100;

    // TODO: temporal
    [Range(1, 10)]
    public int speedUpTime = 1;

    private Transform _controllerTransform;
    // TODO: temporal
    private Random _random;

    
    private void Start()
    {
        graph = new Graph();
        _random = new Random();
        nodeCount = graph.nodes.Count;
        
        _controllerTransform = transform;
        activePedestrians = new List<Pedestrian>(maxPedestriansCount);
        availableIDs = new List<int>();
        
        for (int i = 0; i < maxPedestriansCount; i++)
            availableIDs.Add(i);
    }

    private void Update()
    {
        vehiclePosition = vehicle.transform.position;
        CheckForSpawning();
        float deltaTime = Time.deltaTime * speedUpTime;
        foreach (Pedestrian pedestrian in activePedestrians)
        {
            pedestrian.UpdateStatus(deltaTime);
        }
        CheckForRemoval();
    }

    /**
     * checks and eventually spawn one pedestrian per frame to limit frame delays
     */
    private void CheckForSpawning()
    {
        if (activePedestrians.Count < maxPedestriansCount)
        {
            // TODO: change spawn process. spawn outside of car field of view
            Node[] nearbyNodes = NearbyNodes(vehiclePosition, spawnRadius);
            if (nearbyNodes.Length == 0)
                throw new ArgumentNullException("no nearby nodes found to spawn pedestrians");
            // TODO: spawn in invisible locations
            Node spawnNode = nearbyNodes[_random.Next(nearbyNodes.Length)];
            SpawnPedestrian(spawnNode);
        }
    }

    /**
     * add all pedestrians out of range in a queue to be removed
     */
    private void CheckForRemoval()
    {
        List<Pedestrian> removalQueue = new List<Pedestrian>();
        foreach (Pedestrian pedestrian in activePedestrians)
        {
            if (Vector3.Distance(pedestrian.position, vehiclePosition) > spawnRadius)
                removalQueue.Add(pedestrian);
            // TODO: remove later
            else if (pedestrian.state == Pedestrian.PedestrianState.ARRIVED)
            {
                removalQueue.Add(pedestrian);
                maxPedestriansCount--;
            }
        }
        
        RemovePedestrians(removalQueue);
    }

    private void RemovePedestrians(List<Pedestrian> removalQueue)
    {
        while (removalQueue.Count != 0)
        {
            Pedestrian pedestrian = removalQueue[0];
            if (RemovePedestrian(pedestrian))
                removalQueue.Remove(pedestrian);
            else
                // TODO: change to pedestrian.position
                Debug.LogWarning("Couldn't remove pedestrian at " + pedestrian.transform.position);
        }
    }
    
    private bool RemovePedestrian(Pedestrian pedestrian)
    {
        if (activePedestrians.Remove(pedestrian))
        {
            availableIDs.Add(pedestrian.id);
            Destroy(pedestrian.gameObject);
            return true;
        }
        
        return false;
    }
    
    private void SpawnPedestrian(Node spawnNode)
    {
        int id = availableIDs[0];
        availableIDs.RemoveAt(0);
        float walkingSpeed = (float) _random.NextDouble() * (maxWalkingSpeed - minWalkingSpeed) + minWalkingSpeed;
        walkingSpeed = KmhToMs(walkingSpeed);

        int b;
        do { b = _random.Next(nodeCount); }
        while (graph.nodes[b] == spawnNode);

        GameObject pedestrianGO = Instantiate(pedestrianModelPrefab, _controllerTransform);
        pedestrianGO.transform.name = "Pedestrian " + id;
        
        Vector2 randomPosition = UnityEngine.Random.insideUnitCircle * spawnNode.radius;
        Vector3 spawnPosition = new Vector3 {
            x = randomPosition.x,
            z = randomPosition.y
        };
        spawnPosition += spawnNode.position;

        Pedestrian pedestrian = pedestrianGO.AddComponent<Pedestrian>();
        pedestrian.Setup(id, spawnPosition, spawnNode, graph.nodes[b], walkingSpeed, viewRadius);
        
        activePedestrians.Add(pedestrian);
    }

    // TODO: change spawn process. include edges too
    private Node[] NearbyNodes(Vector3 center, float radius)
    {
        List<Node> nearbyNodes = new List<Node>();
        foreach (Node node in graph.nodes)
        {
            if (Vector3.Distance(node.position, center) <= radius)
            {
                nearbyNodes.Add(node);
            }
        }

        return nearbyNodes.ToArray();
    }

    /**
     * convert the speed from km/h to m/s
     */
    private float KmhToMs(float speed)
    {
        return speed / 3.6f;
    }
}
