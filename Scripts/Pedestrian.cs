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
        SLOWING_DOWN,
        SWITCHING_GOAL,
        ARRIVED
    }
    
    private const float MAX_STEER_FORCE = 3;
    private static readonly int ANIMATION_WALKING_SPEED = Animator.StringToHash("walkingSpeed");

    private PedestrianController _controller;
    private Animator _animator;
    
    private List<Node> _path;
    private Node _lastGoal;
    private Node _currentGoal;
    private Edge _currentEdge;
    private int _currentGoalIndex;
    private int _finalGoalIndex;

    public int id;
    public PedestrianState state = PedestrianState.IDLE;
    public float maxWalkingSpeed;
    public float viewRadius;
    public float slowDownRadius;
    public float modelWidth;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;


    public void Setup(int id, Vector3 spawnPosition, List<Node> path, float maxWalkingSpeed, float viewRadius, float slowDownRadius)
    {
        _controller = transform.parent.GetComponent<PedestrianController>();
        _animator = GetComponent<Animator>();
        modelWidth = GetComponent<BoxCollider>().size.x;
        
        this.id = id;
        this.maxWalkingSpeed = maxWalkingSpeed;
        this.viewRadius = viewRadius;
        this.slowDownRadius = slowDownRadius;
        SetPath(path);
        
        position = spawnPosition;
        Vector3 direction = (_currentGoal.position - _lastGoal.position).normalized;
        velocity = direction * maxWalkingSpeed;
        acceleration = Vector3.zero;
        
        transform.position = spawnPosition;
        transform.forward = direction;
    }

    private void SetPath(List<Node> path)
    {
        state = PedestrianState.WALKING;
        _path = path;
        _lastGoal = path[0];
        _currentGoal = path[1];
        _currentEdge = _lastGoal.GetEdgeByNeighbor(_currentGoal);
        _currentGoalIndex = 1;
        _finalGoalIndex = path.Count - 1;
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
                effectingForce = _currentEdge.type == Edge.EdgeType.Straight ?
                    SeekWithOffset(_currentGoal) : FollowCurvedPath(_currentEdge);
                break;
            
            case PedestrianState.AVOIDING_COLLISION:
            case PedestrianState.SLOWING_DOWN:
                effectingForce = AvoidCollision();
                break;
            
            
            case PedestrianState.SWITCHING_GOAL:
                _lastGoal = _currentGoal;
                _currentGoal = _path[++_currentGoalIndex];
                _currentEdge = _lastGoal.GetEdgeByNeighbor(_currentGoal);
                state = PedestrianState.WALKING;
                return;
            
            case PedestrianState.ARRIVED:
                Node pathStart = _path[_finalGoalIndex];
                SetPath(_controller.NewPath(pathStart));
                return;
            
            default:
                throw new ArgumentOutOfRangeException("reached an unreachable PedestrianState");
        }
        
        MovePedestrian(effectingForce, deltaTime, state == PedestrianState.SLOWING_DOWN);
        
        if (PedestrianWithinViewRadius())
            state = PedestrianWithinSafeZone() ?
                PedestrianState.SLOWING_DOWN : PedestrianState.AVOIDING_COLLISION;
        else
            state = PedestrianState.WALKING;
        
        if (Vector3.Distance(position, _currentGoal.position) < _currentGoal.radius)
        {
            state = _currentGoalIndex == _finalGoalIndex ?
                PedestrianState.ARRIVED : PedestrianState.SWITCHING_GOAL;
        }
    }
    
    private void MovePedestrian(Vector3 effectingForce, float deltaTime, bool slowSpeed)
    {
        acceleration = Vector3.ClampMagnitude(effectingForce, MAX_STEER_FORCE);
        velocity += acceleration * deltaTime;
        velocity = Vector3.ClampMagnitude(velocity, maxWalkingSpeed);
        
        float speed = velocity.magnitude;
        if (slowSpeed)
        {
            speed -= speed * deltaTime;
            velocity = velocity.normalized * speed;
        }
        position += velocity * deltaTime;
        
        _animator.SetFloat(ANIMATION_WALKING_SPEED, speed);
        transform.position = position;
        transform.forward = velocity.normalized;
    }
    
    private Vector3 Seek(Vector3 target)
    {
        Vector3 desiredVelocity = (target - position).normalized * maxWalkingSpeed;
        return desiredVelocity - velocity;
    }
    
    /**
     * <returns>steering force required to follow the target</returns>
     */
    private Vector3 SeekWithOffset(Node target)
    {
        float radius = target.radius;
        Vector3 targetPos = target.position;
        Vector3 ahead = velocity.normalized;
        
        Vector3 projTargetOnAhead = Vector3.Dot(targetPos - position, ahead) * ahead + position;
        if (Vector3.Distance(projTargetOnAhead, targetPos) > radius)
        {
            Vector3 edgeOfNode = (projTargetOnAhead - targetPos).normalized * (radius / 5 * 4) + targetPos;
            return Seek(edgeOfNode);
        }

        return Vector3.zero;
    }
    
    private Vector3 FollowCurvedPath(Edge curvedEdge)
    {
        float radius = curvedEdge.radius;
        float halfWidth = curvedEdge.width / 2;
        Vector3 center = curvedEdge.center;
        
        Vector3 ahead = position + velocity;
        Vector3 aheadRelativeToCenter = ahead - center;
        float distToCenter = aheadRelativeToCenter.magnitude;
        
        if (distToCenter < radius - halfWidth || distToCenter > radius + halfWidth)
        {
            Vector3 projAheadOnPath = aheadRelativeToCenter.normalized * radius + center;
            return Seek(projAheadOnPath);
        }

        return Vector3.zero;
    }

    private Vector3 AvoidCollision()
    {
        Vector3 direction = _currentEdge.type == Edge.EdgeType.Curve ?
            (FollowCurvedPath(_currentEdge) + velocity).normalized : (_currentGoal.position - position).normalized;
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

    private bool PedestrianWithinViewRadius()
    {
        Collider[] colliders = Physics.OverlapSphere(position, viewRadius);

        if (colliders.Length == 1)
            return false;
        
        foreach (Collider collider1 in colliders)
        {
            // collider position in local space
            Vector3 relPos = transform.InverseTransformPoint(collider1.transform.position);
            if (relPos.z > 0)      // check if pedestrian ahead
                return true;
        }
        return false;
    }

    private bool PedestrianWithinSafeZone()
    {
        Collider[] colliders = Physics.OverlapSphere(position, slowDownRadius);
        
        if (colliders.Length == 1)
            return false;
        
        foreach (Collider collider1 in colliders)
        {
            // collider position in local space
            Vector3 relPos = transform.InverseTransformPoint(collider1.transform.position);
            if (relPos.z < 0) // check if pedestrian ahead
                continue;

            if (Vector3.Angle(relPos, Vector3.forward) <= 45)
            {
                if (transform.InverseTransformDirection(collider1.transform.forward).z > 0)
                    return true;
            }
        }
        return false;
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
