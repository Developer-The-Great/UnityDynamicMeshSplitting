using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;



public static class MeshSplittingStatics 
{
    /// <summary>
    /// 
    /// </summary>
    /// <param name="meshVertices"></param>
    /// <param name="meshUVs"></param>
    /// <param name="v0"></param>
    /// <param name="v1"></param>
    /// <param name="v2"></param>
    /// <param name="inversePosition"></param>
    /// <param name="inverseNormal"></param>
    /// <returns></returns>
    public static List<IntersectionQuery> FindTriangleToPlaneIntersectionPoint(NativeArray<Vector3> meshVertices, NativeArray<Vector2> meshUVs, int v0, int v1, int v2, Vector3 inversePosition, Vector3 inverseNormal)
    {
        List<IntersectionQuery> result = new List<IntersectionQuery>();

        IntersectionQuery intersection1;
        if (FindLineToPlaneIntersection(meshVertices, meshUVs, v0, v1, inversePosition, inverseNormal, out intersection1))
        {
            result.Add(intersection1);
        }

        IntersectionQuery intersection2;
        if (FindLineToPlaneIntersection(meshVertices, meshUVs, v1, v2, inversePosition, inverseNormal, out intersection2))
        {
            result.Add(intersection2);
        }

        IntersectionQuery intersection3;
        if (FindLineToPlaneIntersection(meshVertices, meshUVs, v2, v0, inversePosition, inverseNormal, out intersection3))
        {
            result.Add(intersection3);
        }

        return result;
    }

    /// <summary>
    /// Given the vertices and UVs of a line in a triangle of a mesh, checks if there is a collision between a plane with a given position and normal
    /// </summary>
    /// <param name="meshVertices"></param>
    /// <param name="meshUVs"></param>
    /// <param name="v0"></param>
    /// <param name="v1"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="intersection"></param>
    /// <returns></returns>
    public static bool FindLineToPlaneIntersection(NativeArray<Vector3> meshVertices, NativeArray<Vector2> meshUVs, int v0, int v1, Vector3 inversePosition, Vector3 inverseNormal, out IntersectionQuery intersection)
    {
        Vector3 objectSpaceV1 = meshVertices[v1];
        Vector3 objectSpaceV0 = meshVertices[v0];

        bool isOnSameSideOfPlane = Utils.IsPointAbovePlane(objectSpaceV0, inversePosition, inverseNormal) ^ Utils.IsPointAbovePlane(objectSpaceV1, inversePosition, inverseNormal);

        intersection = new IntersectionQuery();

        if (!isOnSameSideOfPlane)
        {
            return false;
        }

        Vector2 UVP0 = meshUVs[v0];
        Vector2 UVP1 = meshUVs[v1];

        Vector3 lineToUse = objectSpaceV1 - objectSpaceV0;

        Vector3 P0 = objectSpaceV0;
        Vector3 P1 = lineToUse.normalized;
        Vector3 A = inversePosition;

        float t = (Vector3.Dot(A, inverseNormal) - Vector3.Dot(P0, inverseNormal)) / Vector3.Dot(P1, inverseNormal);

        intersection.intersectionPosition = P0 + P1 * t;
        intersection.UV = Vector2.Lerp(UVP0, UVP1, (t / Vector3.Magnitude(lineToUse)));

        return true;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="meshVertices"></param>
    /// <param name="meshUVs"></param>
    /// <param name="face"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="intersectionPoints"></param>
    public static void GetFaceToPlaneIntersectionPoints(NativeArray<Vector3> meshVertices, NativeArray<Vector2> meshUVs, Face face, Vector3 position, Vector3 normal, out List<IntersectionQuery> intersectionPoints)
    {
        intersectionPoints = new List<IntersectionQuery>();



        List<IntersectionQuery> triangleIntersectionPoints = FindTriangleToPlaneIntersectionPoint
            (meshVertices, meshUVs, face.tri1.v0, face.tri1.v1, face.tri1.v2, position, normal);


        List<IntersectionQuery> secondTriangleIntersectionPoints = FindTriangleToPlaneIntersectionPoint
            (meshVertices, meshUVs, face.tri2.v0, face.tri2.v1, face.tri2.v2, position, normal);


        foreach (var intersectionPoint in triangleIntersectionPoints)
        {
            intersectionPoints.Add(intersectionPoint);
        }

        foreach (var intersectionPoint in secondTriangleIntersectionPoints)
        {
            intersectionPoints.Add(intersectionPoint);
        }

    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="nonUniqueCollection"></param>
    /// <returns></returns>
    public static List<IntersectionQuery> GetUniqueIntersectionQueryCollection(List<IntersectionQuery> nonUniqueCollection)
    {
        List<IntersectionQuery> uniqueCollection = new List<IntersectionQuery>();

        for (int i = 0; i < nonUniqueCollection.Count; i++)
        {
            bool hasSeenElement = false;

            foreach (var vertex in uniqueCollection)
            {

                if (Vector3.Magnitude(nonUniqueCollection[i].intersectionPosition - vertex.intersectionPosition) < 0.001f)
                {
                    hasSeenElement = true;
                    break;
                }
            }

            if (!hasSeenElement)
            {
                uniqueCollection.Add(nonUniqueCollection[i]);
            }

        }

        return uniqueCollection;
    }





}
