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
    
    public float walkingSpeed;
    public float maxSteerForce;
    public float viewRadius;
    //TODO: automatize this
    public float modelWidth = 0.8f;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 steeringForces;

    public Vector3 avoidance;
    public Vector3 following;


    public void Setup(int id, Vector3 position, Node start, Node destination, float walkingSpeed, float maxSteerForce, float viewRadius)
    {
        this.walkingSpeed = walkingSpeed;
        this.maxSteerForce = maxSteerForce;
        this.viewRadius = viewRadius;
    
        bool success = Graph.AStar(start, destination, out goalList);
        if (!success)
            throw new Exception("couldn't find a solution for node");
        
        currentGoal = start.position;
        currentGoalIndex = 0;
        finalGoalIndex = goalList.Count - 1;

        this.position = position;
        Vector3 direction = (currentGoal - position).normalized;
        velocity = direction * walkingSpeed;

        transform.position = position;
        transform.forward = direction;

        avoidance = new Vector3();
        following = new Vector3();
    }

    public void UpdateStatus(float deltaTime)
    {
        switch (state)
        {
            case PedestrianState.IDLE:
                state = PedestrianState.WALKING;
                break;
                
            case PedestrianState.WALKING:
                // F = m . a     when the mass is constant (1) F = a

                //avoidance
                // if (Physics.SphereCast(position, modelWidth, velocity.normalized, out RaycastHit hitInfo, viewRadius))
                // {
                //     avoidance = Vector3.right / 4;
                //     avoidance = transform.TransformDirection(avoidance);
                // } else
                // {
                //     avoidance = Vector3.zero;
                // }
                
                //TODO: different path width
                following = FollowPath(position, currentGoal, 10);
                avoidance = AvoidObstacles();
                steeringForces = following + avoidance;
                steeringForces = Mathf.Min(steeringForces.magnitude, maxSteerForce) * steeringForces.normalized;
                velocity += steeringForces * deltaTime;
                velocity = velocity.normalized * walkingSpeed;
                position += velocity * deltaTime;
                
                transform.position = position;
                transform.forward = velocity.normalized;

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
                    velocity = (currentGoal - position).normalized * walkingSpeed;
                    state = PedestrianState.WALKING;
                }
                break;
            
            case PedestrianState.ARRIVED:
                Debug.Log("pedestrian has reached their destination");
                break;
        }
    }

    /**
     * <returns>steering force required to follow the target</returns>
     */
    public Vector3 Seek(Vector3 target)
    {
        Vector3 desiredVelocity = (position - target).normalized * walkingSpeed;
        return desiredVelocity - velocity;
    }

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
                if (pedestrian.walkingSpeed < walkingSpeed)
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
