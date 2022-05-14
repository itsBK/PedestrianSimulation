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
    public float viewRadius = 4;
    public float spawnRadius = 100;

    // TODO: temporal
    [Range(1, 10)]
    public int speedUpTime = 1;

    private Transform _controllerTransform;
    // TODO: temporal
    private Random _random;

    
    // TODO: for testing. can be removed later
    private void Awake()
    {
        QualitySettings.vSyncCount = 0;         // no vSync
        Application.targetFrameRate = 10;
    }

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
        CheckForRemoval(vehiclePosition);
        CheckForSpawning(vehiclePosition);
        float deltaTime = Time.deltaTime * speedUpTime;
        foreach (Pedestrian pedestrian in activePedestrians)
        {
            pedestrian.UpdateStatus(deltaTime);
        }
    }

    /**
     * checks and eventually spawn one pedestrian per frame to limit frame delays
     */
    private void CheckForSpawning(Vector3 vehiclePosition)
    {
        if (activePedestrians.Count < maxPedestriansCount)
        {
            // TODO: find a procedural process with seed for spawning
            Vector3 position = new Vector3 {
                x = ((float) _random.NextDouble() * 2 - 1) * spawnRadius,
                z = ((float) _random.NextDouble() * 2 - 1) * spawnRadius
            };
            position += vehiclePosition;
            SpawnPedestrian(position);
        }
    }

    /**
     * add all pedestrians out of range in a queue to be removed
     */
    private void CheckForRemoval(Vector3 vehiclePosition)
    {
        List<Pedestrian> removalQueue = new List<Pedestrian>();
        foreach (Pedestrian pedestrian in activePedestrians)
        {
            // TODO: change to pedestrian.position
            if (Vector3.Distance(pedestrian.transform.position, vehiclePosition) > spawnRadius)
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
    
    private void SpawnPedestrian(Vector3 spawnPosition)
    {
        int id = availableIDs[0];
        availableIDs.RemoveAt(0);
        // TODO: find a procedural process with seed
        float walkingSpeed = (float) _random.NextDouble() * (maxWalkingSpeed - minWalkingSpeed) + minWalkingSpeed;
        walkingSpeed = KmhToMs(walkingSpeed);
        
        // TODO: find a procedural process with seed
        // Random.Next() return a value less then maxValue ([0, maxValue[)
        int a = _random.Next(nodeCount);
        int b = a;
        while (b == a)
            b = _random.Next(nodeCount);

        GameObject pedestrianGO = Instantiate(pedestrianModelPrefab, _controllerTransform);
        pedestrianGO.transform.name = "Pedestrian " + id;
        // TODO: change to pedestrian.position;
        pedestrianGO.transform.position = spawnPosition;
        
        Pedestrian pedestrian = pedestrianGO.AddComponent<Pedestrian>();
        pedestrian.Setup(id, spawnPosition, graph.nodes[a], graph.nodes[b], walkingSpeed, viewRadius);
        
        activePedestrians.Add(pedestrian);
    }

    /**
     * convert the speed from km/h to m/s
     */
    private float KmhToMs(float speed)
    {
        return speed / 3.6f;
    }
}
