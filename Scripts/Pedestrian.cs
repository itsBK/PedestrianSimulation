using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class Pedestrian : MonoBehaviour
{

    public enum PedestrianState {
        IDLE,
        WALKING,
        WAITING,
        ARRIVED
    }

    private PedestrianController _controller;

    public int id;
    public PedestrianState state = PedestrianState.IDLE;
    public List<Node> goalList;
    public Vector3 currentGoal;
    public int currentGoalIndex;
    public int finalGoalIndex;
    
    public float maxSteerForce = 1;
    public float maxWalkingSpeed;
    public float slowingDownDistance = 10;
    public float viewRadius;
    //TODO: automatize this
    public float modelWidth = 0.8f;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;


    public void Setup(int id, Vector3 position, Node start, Node destination, float maxWalkingSpeed, float viewRadius)
    {
        _controller = transform.parent.GetComponent<PedestrianController>();
        this.id = id;
        this.maxWalkingSpeed = maxWalkingSpeed;
        this.viewRadius = viewRadius;
        
        bool success = Graph.AStar(start, destination, out goalList);
        if (!success)
            throw new Exception("couldn't find a solution for node");
        
        currentGoal = start.position;
        currentGoalIndex = 0;
        finalGoalIndex = goalList.Count - 1;

        this.position = position;
        Vector3 direction = (currentGoal - position).normalized;
        velocity = direction * maxWalkingSpeed;

        transform.position = position;
        transform.forward = direction;
    }

    /**
     * <param name="deltaTime">in seconds</param>
     */
    public void UpdateStatus(float deltaTime)
    {
        switch (state)
        {
            case PedestrianState.IDLE:
                state = PedestrianState.WALKING;
                break;
                
            case PedestrianState.WALKING:
                // F = m . a     when the mass is constant (1) F = a
                //TODO: different path width
                MovePedestrian(Seek(currentGoal), deltaTime);

                if (Vector3.Distance(position, currentGoal) < 1)
                {
                    state = PedestrianState.WAITING;
                }
                break;
                
            case PedestrianState.WAITING:
                if (currentGoalIndex == finalGoalIndex)
                {
                    state = PedestrianState.ARRIVED;
                } else
                {
                    currentGoal = goalList[++currentGoalIndex].position;
                    state = PedestrianState.WALKING;
                }
                break;
            
            case PedestrianState.ARRIVED:
                Debug.Log("pedestrian has reached their destination");
                break;
        }
    }

    public void MovePedestrian(Vector3 effectingForces, float deltaTime)
    {
        acceleration = Cap(effectingForces, maxSteerForce);
        velocity += acceleration * deltaTime;
        velocity = Cap(velocity, maxWalkingSpeed);
        position += velocity * deltaTime;

        transform.position = position;
        transform.forward = velocity.normalized;
    }

    /**
     * <returns>steering force required to follow the target</returns>
     */
    public Vector3 Seek(Vector3 target)
    {
        Vector3 desiredVelocity = (target - position).normalized * maxWalkingSpeed;
        return desiredVelocity - velocity;
    }

    /**
     * <returns>steering force required to follow the target and slow down when getting close</returns>>
     */
    public Vector3 Arrival(Vector3 target)
    {
        Vector3 targetRelativePosition = target - position;
        float distance = targetRelativePosition.magnitude;
        float rampedSpeed = maxWalkingSpeed * distance / slowingDownDistance;
        float clippedSpeed = Mathf.Min(rampedSpeed, maxWalkingSpeed);
        // divide relative position by the distance to get normalized vector in target direction
        Vector3 desiredVelocity = targetRelativePosition / distance * clippedSpeed;
        return desiredVelocity - velocity;
    }

    /**
     * caps the vector to the specified length if it succeeds it
     */
    public Vector3 Cap(Vector3 vector, float maxLength)
    {
        if (vector.magnitude > maxLength)
            vector = vector.normalized * maxLength;
        return vector;
    }

    public float WalkingSpeed => velocity.magnitude;















    //TODO: include curved paths
    public Vector3 FollowPath(Vector3 pathStart, Vector3 pathEnd, float pathWidth)
    {
        Vector3 ahead = position + velocity;
        if (HandleUtility.DistancePointLine(ahead, pathStart, pathEnd) > pathWidth / 2)
        {
            // same as Vector3.Dot(pathEnd - pathStart, ahead) * (pathEnd - pathStart);
            Vector3 projAheadOnPath = HandleUtility.ProjectPointLine(ahead, pathStart, pathEnd);
            return Seek(projAheadOnPath);
        }

        return Vector3.zero;
    }

    public Vector3 AvoidObstacles()
    {
        if (Physics.SphereCast(position, modelWidth, velocity.normalized, out RaycastHit hit, viewRadius))
        {
            if (hit.transform.TryGetComponent(out Pedestrian pedestrian))
            {
                if (pedestrian.WalkingSpeed < WalkingSpeed)
                {
                    Vector3 hitPosition = transform.InverseTransformPoint(hit.point);
                    // steer in opposite direction 
                    return transform.TransformDirection(Vector3.left * hitPosition.x);
                }
            }
        }
        return Vector3.zero;
    }
}
