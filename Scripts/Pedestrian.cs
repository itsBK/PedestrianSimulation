using System;
using System.Collections.Generic;
using UnityEngine;

public class Pedestrian : MonoBehaviour
{

    private PedestrianController _controller;
    
    public float walkingSpeed;
    public float viewRadius;

    public Vector3 position;
    public Vector3 velocity;
    public Vector3 effectingForces;
    public Bounds bounds;

    public Vector3 ahead;
    
    public Vector3 avoidance;
    public Vector3 alignment;
    public Vector3 cohesion;


    /**
     *  <param name="direction">must be normalized</param>
     */
    public void Setup(Node start, Node destination, float walkingSpeed, float viewRadius)
    {
        bool success = Graph.AStar(start, destination, out List<Node> goalList);
        if (!success)
            throw new Exception("couldn't find a solution for node");
        Vector3 direction = goalList[1].position - goalList[0].position;

        this.walkingSpeed = walkingSpeed;
        this.viewRadius = viewRadius;
        
        position = start.position;
        velocity = direction.normalized * walkingSpeed;
        bounds = new Bounds {
            center = position,
            size = Vector3.one
        };

        transform.position = position;
        transform.forward = direction;

        //ahead is a position in real world, not in local space (used to check collisions)
        ahead = position + velocity;
        
        avoidance = new Vector3();
        alignment = new Vector3();
        cohesion = new Vector3();
    }

    public void UpdateStatus(float deltaTime)
    {
        // F = m . a     when the mass is constant (1) F = a
        effectingForces = avoidance + alignment + cohesion;
        velocity += effectingForces;
        velocity = velocity.normalized * walkingSpeed;
        position += velocity * deltaTime;
        
        // temp
        if (position.x < 0) position.x += 100;
        if (position.z < 0) position.z += 100;
        if (position.x > 100) position.x -= 100;
        if (position.z > 100) position.z -= 100;
        
        transform.position = position;
        transform.forward = velocity.normalized;
        
        bounds.center = position;
        ahead = position + velocity;
    }
    
    
    /*
     * FOR REFERENCE
     *
     * 
    public float angle;

    public void setup() {
        this.angle = this.velocity.angle();
    }

    public void update(float deltaTime) {

        this.angle = velocity.angle();
        if (angle < 0) angle += 360f;
        if (angle > 360f) angle -= 360f;
    }
     */
}
