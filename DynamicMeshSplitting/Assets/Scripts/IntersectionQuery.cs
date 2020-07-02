using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IntersectionQuery
{
    public Vector3 intersectionPosition;
    public Vector2 UV;

    public IntersectionQuery(Vector3 intersectionPosition, Vector3 UV)
    {
        this.intersectionPosition = intersectionPosition;
        this.UV = UV;
    }

    public IntersectionQuery()
    {

    }

}

public class IntersectionQueryComparer : IComparer<IntersectionQuery>
{
    private Vector3 baseDirection;
    public Vector3 basePosition;
    private Matrix4x4 world;


    public IntersectionQueryComparer(Vector3 baseDirection, Vector3 basePosition, Matrix4x4 world)
    {
        this.baseDirection = baseDirection;
        this.basePosition = basePosition;
        this.world = world;
    }

    public int Compare(IntersectionQuery intersectionPoint1, IntersectionQuery intersectionPoint2)
    {
        float x = Vector3.Dot(baseDirection, (world.MultiplyPoint(intersectionPoint1.intersectionPosition) - basePosition).normalized);
        float y = Vector3.Dot(baseDirection, (world.MultiplyPoint(intersectionPoint2.intersectionPosition) - basePosition).normalized);

        return x.CompareTo(y);
    }

}
