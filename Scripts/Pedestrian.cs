using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Pedestrian : MonoBehaviour
{

    public enum PedestrianState {
        IDLE,
        WALKING,
        AVOIDING_COLLISION,
        REDUCING_SPEED,
        ARRIVED
    }

    private PedestrianController _controller;

    public int id;
    public PedestrianState state = PedestrianState.IDLE;
    public List<Node> goalList;
    public Node lastGoal;
    public Node currentGoal;
    public Edge currentEdge;
    public int currentGoalIndex;
    public int finalGoalIndex;
    
    public float maxSteerForce = 1;
    public float maxWalkingSpeed;
    public float minSlowDownSpeed = 0.1f;
    public float viewRadius;
    public float safeZone = 2;
    //TODO: automatize this
    public float modelWidth = 0.5f;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;
    
    public Vector3 avoidance;
    public Vector3 seek;


    public void Setup(int id, Vector3 spawnPosition, out List<Node> goalList, float maxWalkingSpeed, float viewRadius)
    {
        _controller = transform.parent.GetComponent<PedestrianController>();
        this.id = id;
        this.maxWalkingSpeed = maxWalkingSpeed;
        this.viewRadius = viewRadius;
        state = PedestrianState.WALKING;
        
        lastGoal = goalList[0];
        currentGoal = goalList[1];
        currentEdge = lastGoal.GetEdgeByNeighbor(currentGoal);
        currentGoalIndex = 1;
        finalGoalIndex = goalList.Count - 1;

        this.position = spawnPosition;
        Vector3 direction = (currentGoal.position - spawnPosition).normalized;
        velocity = direction * maxWalkingSpeed;

        transform.position = spawnPosition;
        transform.forward = direction;
    }

    /**
     * <param name="deltaTime">in seconds</param>
     */
    public void UpdateStatus(float deltaTime)
    {
        Vector3 effectingForce;
        switch (state)
        {
            case PedestrianState.IDLE:
                return;
            
            case PedestrianState.WALKING:
                //TODO: different path width
                //TODO: switch to follow path
                seek = SeekWithOffset(currentGoal);
                effectingForce = seek;
                break;
            
            case PedestrianState.AVOIDING_COLLISION:
                avoidance = AvoidCollision();
                effectingForce = avoidance;
                break;
            
            case PedestrianState.REDUCING_SPEED:
                effectingForce = ReduceSpeed();
                break;
            
            case PedestrianState.ARRIVED:
                Debug.Log("pedestrian has reached their destination");
                return;
            
            default:
                throw new ArgumentOutOfRangeException("reached an unreachable PedestrianState");
        }

        MovePedestrian(effectingForce, deltaTime);

        // check if current goal is reached
        if (Vector3.Distance(position, currentGoal.position) < currentGoal.radius)
        {
            if (currentGoalIndex == finalGoalIndex)
            {
                state = PedestrianState.ARRIVED;
            } else
            {
                lastGoal = currentGoal;
                currentGoal = goalList[++currentGoalIndex];
                currentEdge = lastGoal.GetEdgeByNeighbor(currentGoal);
                state = PedestrianState.WALKING;
            }
        }

        if (CheckCollisionAhead(out float distance))
        {
            state = distance <= safeZone ?
                PedestrianState.REDUCING_SPEED : PedestrianState.AVOIDING_COLLISION;
        } else
        {
            state = PedestrianState.WALKING;
        }
    }

    public void MovePedestrian(Vector3 effectingForce, float deltaTime)
    {
        acceleration = Clamp(effectingForce, maxSteerForce);
        velocity += acceleration * deltaTime;
        velocity = Clamp(velocity, maxWalkingSpeed);
        position += velocity * deltaTime;

        transform.position = position;
        transform.forward = velocity.normalized;
    }

    /**
     * <returns>steering force required to follow the target</returns>
     */
    public Vector3 Seek(Node target)
    {
        Vector3 desiredVelocity = (target.position - position).normalized * maxWalkingSpeed;
        return desiredVelocity - velocity;
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
     * <returns>steering force required to follow the target</returns>
     */
    public Vector3 SeekWithOffset(Node target)
    {
        float radius = target.radius;
        Vector3 ahead = position + velocity;
        Vector3 projTargetOnAhead = (Vector3.Dot(target.position, ahead) / ahead.magnitude) * ahead;
        if (Vector3.Distance(ahead, projTargetOnAhead) > radius)
        {
            Vector3 desiredVelocity = (target.position - position).normalized * maxWalkingSpeed;
            return desiredVelocity - velocity;
        }
        return Vector3.zero;
    }

    /**
     * <returns>steering force required to follow the target and slow down when getting close</returns>>
     */
    public Vector3 Arrival(Node target)
    {
        Vector3 targetRelativePosition = target.position - position;
        float distance = targetRelativePosition.magnitude;
        float rampedSpeed = maxWalkingSpeed * distance / target.radius;
        float clippedSpeed = Mathf.Min(rampedSpeed, maxWalkingSpeed);
        // divide relative position by the distance to get normalized vector in target direction
        Vector3 desiredVelocity = targetRelativePosition / distance * clippedSpeed;
        return desiredVelocity - velocity;
    }

    /**
     * clamp the vector to the specified length if it succeeds it
     */
    public Vector3 Clamp(Vector3 vector, float maxLength)
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
    
    public Vector3 FollowPath(Node pathStart, Node pathEnd, float pathWidth)
    {
        Vector3 pathStartVec = pathStart.position;
        Vector3 pathEndVec = pathEnd.position;
        Vector3 ahead = position + velocity;
        if (HandleUtility.DistancePointLine(ahead, pathStartVec, pathEndVec) > pathWidth / 2)
        {
            // same as Vector3.Dot(pathEnd - pathStart, ahead) * (pathEnd - pathStart);
            Vector3 projAheadOnPath = HandleUtility.ProjectPointLine(ahead, pathStartVec, pathEndVec);
            return Seek(projAheadOnPath);
        }

        return Vector3.zero;
    }

    public bool CheckCollisionAhead(out float distance)
    {
        bool result = Physics.SphereCast(new Ray(position, velocity.normalized),
            modelWidth / 2, out RaycastHit hitInfo, viewRadius);
        distance = hitInfo.distance;
        return result;
    }

    public Vector3 AvoidCollision()
    {
        Vector3 direction = velocity.normalized;
        Ray ray = new Ray();
        
        int i = 1;
        bool hit = true;
        while (hit && i < 40)
        {
            ray = new Ray(position, GetDirection(direction, i));
            hit = Physics.SphereCast(ray, modelWidth / 2, viewRadius);
            i++;
        }

        return Seek(ray.origin + ray.direction * velocity.magnitude);
    }

    public Vector3 ReduceSpeed()
    {
        return velocity.magnitude > minSlowDownSpeed? -velocity : Vector3.zero;
    }

    public Vector3 GetDirection(Vector3 direction, int i)
    {
        // input:      i = 0,  1,  2,   3,   4,   5,   6
        // output: angle = 0, -5,  5, -10,  10, -15,  15
        int sign = -1 ^ i;
        int angle = (i + 1) / 2 * 5;
        return Quaternion.AngleAxis(sign * angle, Vector3.up) * direction;
    }

    //TODO: temporal for testing.
    public void OnDrawGizmosSelected()
    {
        Vector3 direction = velocity.normalized;

        Gizmos.color = Color.green;
        Gizmos.DrawSphere(position, modelWidth / 2);
        bool hit = Physics.SphereCast(position, modelWidth / 2, direction, out RaycastHit hitInfo, viewRadius);
        
        if (hit)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(hitInfo.point, modelWidth / 2);
        }
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
