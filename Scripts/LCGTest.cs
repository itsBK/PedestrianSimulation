using System;
using System.Collections.Generic;
using UnityEngine;

public class LCGTest : MonoBehaviour
{
    
    public enum Dimensions
    {
        a2D,
        a3D
    }
    
    private List<Vector3> points = new List<Vector3>();
    private int count = 0;
    private int state = 1;

    public Dimensions dimensions = Dimensions.a2D;
    public int m = 0;
    public int a = 0;
    public int c = 0;

    public int maxCount = 100;
    public bool reset = false;

    private void OnDrawGizmos()
    {
        if (reset)
        {
            points.Clear();
            state = 1;
            count = 0;
            reset = false;
        }

        while (count < maxCount)
        {
            Vector3 point = dimensions == Dimensions.a2D? NextPoint2D() : NextPoint3D();
            points.Add(point);
            count++;
        }
        
        Gizmos.color = Color.black;
        foreach (Vector3 point in points)
        {
            Gizmos.DrawSphere(100 * point, 0.5f);
        }
    }

    public Vector3 NextPoint2D()
    {
        return new Vector3((float) Next() / m, (float) Next() / m, 1);
    }

    public Vector3 NextPoint3D()
    {
        return new Vector3((float) Next() / m, (float) Next() / m, (float) Next() / m);
    }

    public int Next()
    {
        state = (a * state + c) % m;
        if (state < 0)
            state = Math.Abs(state);
        return state;
    }

}