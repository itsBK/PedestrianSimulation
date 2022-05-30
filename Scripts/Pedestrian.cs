using System;
using System.Collections.Generic;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class Pedestrian : MonoBehaviour
{

    public enum PedestrianState {
        IDLE,
        WALKING,
        AVOIDING_COLLISION,
        SLOWING_SPEED,
        ARRIVED
    }

    private PedestrianController _controller;

    public int id;
    public PedestrianState state = PedestrianState.IDLE;
    public List<Node> path;
    public Node lastGoal;
    public Node currentGoal;
    public int currentGoalIndex;
    public int finalGoalIndex;
    
    public float maxSteerForce = 3;
    public float maxWalkingSpeed;
    public float viewRadius;
    public float safeZone = 2;
    //TODO: automatize this
    public float modelWidth = 0.5f;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;
    

    public void Setup(PedestrianController controller, int id, Vector3 spawnPosition, List<Node> path, float maxWalkingSpeed, float viewRadius)
    {
        _controller = controller;
        this.id = id;
        this.maxWalkingSpeed = maxWalkingSpeed;
        this.viewRadius = viewRadius;
        SetPath(path);
        
        position = spawnPosition;
        Vector3 direction = (currentGoal.position - lastGoal.position).normalized;
        velocity = direction * maxWalkingSpeed;
        
        transform.position = spawnPosition;
        transform.forward = direction;
    }

    public void SetPath(List<Node> path)
    {
        state = PedestrianState.WALKING;
        this.path = path;
        lastGoal = path[0];
        currentGoal = path[1];
        currentGoalIndex = 1;
        finalGoalIndex = path.Count - 1;
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
                effectingForce = SeekWithOffset(currentGoal);
                break;
            
            case PedestrianState.AVOIDING_COLLISION:
                effectingForce = AvoidCollision();
                break;
            
            case PedestrianState.SLOWING_SPEED:
                effectingForce = AvoidCollision();
                break;
            
            case PedestrianState.ARRIVED:
                Node pathStart = path[finalGoalIndex];
                SetPath(_controller.NewPath(pathStart));
                return;
            
            default:
                throw new ArgumentOutOfRangeException("reached an unreachable PedestrianState");
        }
        
        MovePedestrian(effectingForce, deltaTime);
        
        if (PedestrianWithinViewRadius())
            // if (PedestrianWithinSafeZone2())
            //     state = PedestrianState.SLOWING_SPEED;
            // else
            state = PedestrianState.AVOIDING_COLLISION;
        else
            state = PedestrianState.WALKING;
        
        if (Vector3.Distance(position, currentGoal.position) < currentGoal.radius)
        {
            if (currentGoalIndex == finalGoalIndex)
            {
                state = PedestrianState.ARRIVED;
            } else
            {
                lastGoal = currentGoal;
                currentGoal = path[++currentGoalIndex];
            }
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
        Vector3 ahead = position + velocity.normalized * maxWalkingSpeed;
        Vector3 projTargetOnAhead = (Vector3.Dot(target.position, ahead) / ahead.magnitude) * ahead;
        if (Vector3.Distance(ahead, projTargetOnAhead) > radius)
        {
            return Seek(target.position);
        }
        return Vector3.zero;
    }

    public Vector3 AvoidCollision()
    {
        Vector3 direction = (currentGoal.position - position).normalized;
        Ray ray = new Ray();
        
        int i = 0;
        bool hit = true;
        float multiplier = 1;
        while (hit && i <= 36) // 36 * 5° = 180°
        {
            ray = new Ray(position, GetDirection(direction, i));
            hit = Physics.SphereCast(ray, modelWidth / 2, out RaycastHit hitInfo, viewRadius);
            if (hit) multiplier = viewRadius / hitInfo.distance;
            i++;
        }
        
        Vector3 desiredVelocity = ray.direction * maxWalkingSpeed;
        Vector3 steeringForce = desiredVelocity - velocity;
        
        multiplier = Mathf.Min(multiplier, viewRadius);
        return steeringForce * multiplier;
    }

    public bool PedestrianWithinViewRadius()
    {
        Collider[] colliders = Physics.OverlapSphere(position, viewRadius);

        if (colliders.Length == 1)
            return false;
        
        foreach (Collider collider1 in colliders)
        {
            // collider position in local space
            Vector3 relPos = transform.InverseTransformPoint(collider1.transform.position);
            if (relPos.z >= 0)      // check if pedestrian ahead
                return true;
        }
        return false;
    }

    public bool PedestrianWithinSafeZone()
    {
        Collider[] colliders = Physics.OverlapSphere(position, safeZone);
        
        if (colliders.Length == 1)
            return false;
        
        foreach (Collider collider1 in colliders)
        {
            // collider position in local space
            Vector3 relPos = transform.InverseTransformPoint(collider1.transform.position);
            if (relPos.z < 0) // check if pedestrian ahead
                continue;

            if (Vector3.Angle(relPos, Vector3.forward) <= 30)
            {
                if (Vector3.Angle(transform.forward, collider1.transform.forward) <= 45)
                    return true;
            }
        }
        return false;
    }
    
    public bool PedestrianWithinSafeZone2()
    {
        bool hit = Physics.SphereCast(position, modelWidth / 2,
            velocity.normalized, out RaycastHit hitInfo, safeZone);

        if (hit) {
            if (Vector3.Angle(transform.forward, hitInfo.transform.forward) <= 30)
                return true;
        }
        return false;
    }
    
    /*
    public void OnDrawGizmosSelected()
    {
        if (state == PedestrianState.SLOWING_SPEED)
        {
            Collider[] colliders = Physics.OverlapSphere(position, safeZone);

            foreach (Collider collider1 in colliders)
            {
                Vector3 relPos = transform.InverseTransformPoint(collider1.transform.position);
                if (relPos.z < 0) // skip if behind
                    continue;
                
                if (collider1.GetComponentInParent<Pedestrian>().id == id)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(collider1.transform.position, modelWidth / 2);
                } else if(Vector3.Angle(relPos, Vector3.forward) <= 30 &&
                    Vector3.Angle(transform.forward, collider1.transform.forward) <= 45) {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(collider1.transform.position, modelWidth / 2);
                }
            }
        }
    }
    */

    /**
     * clamp the vector to the specified length if it succeeds it
     */
    public Vector3 Clamp(Vector3 vector, float maxLength)
    {
        if (vector.magnitude > maxLength)
            vector = vector.normalized * maxLength;
        return vector;
    }
    
    public Vector3 GetDirection(Vector3 direction, int i)
    {
        // input:      i = 0,  1,  2,   3,   4,   5,   6
        // output: angle = 0, -5,  5, -10,  10, -15,  15
        int sign = i % 2 == 1? -1 : 1;
        int angle = ((i + 1) / 2) * 5;
        return Quaternion.AngleAxis(sign * angle, Vector3.up) * direction;
    }
    
}
