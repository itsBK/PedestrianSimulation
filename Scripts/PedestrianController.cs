using System;
using System.Collections.Generic;
using UnityEngine;
using Random = System.Random;

public class PedestrianController : MonoBehaviour
{

    public Graph graph;
    public int nodeCount;

    public GameObject pedestrianModelPrefab;
    public GameObject vehicle;
    public Vector3 vehiclePosition;
    public List<Pedestrian> activePedestrians;
    public List<int> availableIDs;
    public int maxPedestriansCount = 40;
    public float minWalkingSpeed = 4;
    public float maxWalkingSpeed = 6;
    public float viewRadius = 5;
    public float spawnRadius = 100;
    public float spawnArea = 2;

    // TODO: temporal
    [Range(1, 10)]
    public int speedUpTime = 1;

    // TODO: temporal
    private Random _random;

    
    private void Start()
    {
        graph = new Graph();
        _random = new Random();
        nodeCount = graph.nodes.Count;
        
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
            Node pathStart = GetRandomNearbyNode(vehiclePosition, spawnRadius);
            if (pathStart == null)
                throw new ArgumentNullException("no nearby nodes found to spawn pedestrians");

            SpawnPedestrian(NewPath(pathStart));
        }
    }

    /**
     * add all pedestrians out of range in a queue to be removed
     */
    private void CheckForRemoval()
    {
        List<Pedestrian> removeQueue = new List<Pedestrian>();
        foreach (Pedestrian pedestrian in activePedestrians)
        {
            if (Vector3.Distance(pedestrian.position, vehiclePosition) > spawnRadius)
                removeQueue.Add(pedestrian);
        }
        
        RemovePedestrians(removeQueue);
    }

    public List<Node> NewPath(Node pathStart)
    {
        Node destination;
        do { destination = graph.nodes[_random.Next(nodeCount)]; }
        while (destination == pathStart);

        bool success = Graph.FindPath(pathStart, destination, out List<Node> path);
        if (!success)
            throw new Exception("couldn't find a path between node(" + pathStart.id + ")"
                + " and node(" + destination.id + ")");

        return path;
    }

    private void RemovePedestrians(List<Pedestrian> removeQueue)
    {
        while (removeQueue.Count != 0)
        {
            Pedestrian pedestrian = removeQueue[0];
            if (RemovePedestrian(pedestrian))
                removeQueue.Remove(pedestrian);
            else
                Debug.LogWarning("couldn't remove pedestrian at " + pedestrian.position);
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
    
    private void SpawnPedestrian(List<Node> path)
    {
        int id = availableIDs[0];
        availableIDs.RemoveAt(0);
        float walkingSpeed = (float) _random.NextDouble() * (maxWalkingSpeed - minWalkingSpeed) + minWalkingSpeed;
        walkingSpeed = KmhToMs(walkingSpeed);

        GameObject pedestrianGO = Instantiate(pedestrianModelPrefab, transform);
        pedestrianGO.name = "Pedestrian " + id;

        //TODO: include curved edges
        Node firstGoal = path[0];
        Node secondGoal = path[1];
        Edge edge = firstGoal.GetEdgeByNeighbor(secondGoal);
        float slider = (float) _random.NextDouble();
        Vector3 spawnPosition = (1 - slider) * firstGoal.position + slider * secondGoal.position;
        
        Vector2 randomPosition = UnityEngine.Random.insideUnitCircle * (edge.width / 2);
        spawnPosition.x += randomPosition.x;
        spawnPosition.z += randomPosition.y;

        int i = 0;
        while (Physics.OverlapSphereNonAlloc(spawnPosition, spawnArea, new Collider[1]) > 0 && i < 20)
        {
            slider = (slider + 0.05f) % 1;
            spawnPosition = (1 - slider) * firstGoal.position + slider * secondGoal.position;
            spawnPosition.x += randomPosition.x;
            spawnPosition.z += randomPosition.y;
            i++;
        }
        if (i == 20)
        {
            Debug.Log("couldn't find a free area to spawn pedestrian. retry next frame");
            availableIDs.Add(id);
            Destroy(pedestrianGO);
        } else
        {
            Pedestrian pedestrian = pedestrianGO.AddComponent<Pedestrian>();
            pedestrian.Setup(this, id, spawnPosition, path, walkingSpeed, viewRadius);
            activePedestrians.Add(pedestrian);
        }
    }

    private Node GetRandomNearbyNode(Vector3 center, float radius)
    {
        List<Node> nearbyNodes = new List<Node>();
        foreach (Node node in graph.nodes)
        {
            if (Vector3.Distance(node.position, center) <= radius)
            {
                nearbyNodes.Add(node);
            }
        }

        // TODO: spawn in invisible locations
        Node nearbyNode = nearbyNodes[_random.Next(nearbyNodes.Count)];
        return nearbyNode;
    }

    /**
     * convert the speed from km/h to m/s
     */
    private float KmhToMs(float speed)
    {
        return speed / 3.6f;
    }
}
