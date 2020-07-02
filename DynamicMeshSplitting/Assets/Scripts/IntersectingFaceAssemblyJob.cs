using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Profiling;
using System.Linq;
using Unity.Burst;


struct IntersectingFaceAssemblyJob : IJobParallelFor
{
    [WriteOnly]
    public NativeQueue<JobFace>.ParallelWriter lowerMesh;
    [WriteOnly]
    public NativeQueue<JobFace>.ParallelWriter upperMesh;

    [ReadOnly]
    public NativeArray<Face> intersectingFaces;

    [ReadOnly]
    public NativeArray<int> meshTriangles;
    [ReadOnly]
    public NativeArray<Vector3> meshNormals;
    [ReadOnly]
    public NativeArray<Vector3> meshVertices;
    [ReadOnly]
    public NativeArray<Vector2> meshUVs;

    public Vector3 position;
    public Vector3 normal;

    public Vector3 preCutCentroid;

    public Matrix4x4 worldMatrix;


    public void Execute(int i)
    {
        Vector3[] worldTrianglePointPositions = new Vector3[6];
        worldTrianglePointPositions[0] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri1.v0]);
        worldTrianglePointPositions[1] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri1.v1]);
        worldTrianglePointPositions[2] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri1.v2]);
        worldTrianglePointPositions[3] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri2.v0]);
        worldTrianglePointPositions[4] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri2.v1]);
        worldTrianglePointPositions[5] = worldMatrix.MultiplyPoint(meshVertices[intersectingFaces[i].tri2.v2]);

        intersectingFaceSplit(intersectingFaces[i], worldTrianglePointPositions);

        //intersectingFaces[i].ToLog();
    }

    private void intersectingFaceSplit(Face face, Vector3[] worldTrianglePointPositions)
    {
        List<Vector3> foundIntersectionPoint;
        UnOptimizedGetFaceToPlaneIntersectionPoints(face, position, normal, out foundIntersectionPoint);

        int[] faceIndices = face.ToIntArray();

        bool[] triangleState = CheckIfPointsAreAbovePlane(worldTrianglePointPositions, position, normal);

        List<int> trianglesAboveSplittingPlane = new List<int>();
        List<int> trianglesBelowSplittingPlane = new List<int>();

        //for each triangleState
        for (int i = 0; i < triangleState.Length; i++)
        {
            if (triangleState[i])
            {
                trianglesAboveSplittingPlane.Add(faceIndices[i]);
            }
            else
            {
                trianglesBelowSplittingPlane.Add(faceIndices[i]);
            }
        }

        List<int> uniqueTrianglesAboveSplittingPlane = GetUniqueVertices(trianglesAboveSplittingPlane);
        List<int> uniqueTrianglesBelowSplittingPlane = GetUniqueVertices(trianglesBelowSplittingPlane);
        List<Vector3> uniqueIntersectionPoints = GetUniqueVector3Collection(foundIntersectionPoint);

        //create bottom faces
        Vector3 basePosition;
        Vector3 baseDirection;

        if (uniqueTrianglesBelowSplittingPlane.Count < 2 && uniqueTrianglesAboveSplittingPlane.Count < 2) { return; }
        if (uniqueIntersectionPoints.Count < 2) { return; }

        basePosition = preCutCentroid;
        baseDirection = Vector3.Cross(Vector3.up, (worldTrianglePointPositions[0] - preCutCentroid)).normalized;

        IntersectionComparer ic = new IntersectionComparer
            (baseDirection, basePosition, worldMatrix);

        IndexDirectionComparer idc = new IndexDirectionComparer((uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1] - uniqueIntersectionPoints[0]).normalized
        , basePosition, meshVertices.ToArray(), worldMatrix);

        uniqueIntersectionPoints.Sort(ic);

        Vector3 belowTriangleCentroid = GetWorldTriangleCentroid(uniqueTrianglesBelowSplittingPlane);
        Vector3 aboveTriangleCentroid = GetWorldTriangleCentroid(uniqueTrianglesAboveSplittingPlane);

        idc.basePosition = uniqueIntersectionPoints[0];
        uniqueTrianglesBelowSplittingPlane.Sort(idc);

        idc.basePosition = uniqueIntersectionPoints[0];
        uniqueTrianglesAboveSplittingPlane.Sort(idc);


        List<Vector3> intersectionPoints = new List<Vector3>();

        intersectionPoints.Add(uniqueIntersectionPoints[0]);
        intersectionPoints.Add(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]);


        //------------------------------- create bottom part----------------------------------------------------//

        Vector3 intersectionDirection = intersectionPoints[1] - intersectionPoints[0];

        Vector3 vertexDirection =
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesBelowSplittingPlane[uniqueTrianglesBelowSplittingPlane.Count - 1]]) -
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesBelowSplittingPlane[0]])
           ;

        if (Vector3.Dot(intersectionDirection, baseDirection) < 0)
        {
            intersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection, intersectionDirection) < 0)
        {
            uniqueTrianglesBelowSplittingPlane.Reverse();
        }

        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesBelowSplittingPlane, false, lowerMesh);

        //------------------------------- create above part----------------------------------------------------//

        Vector3 intersectionDirection2 = intersectionPoints[1] - intersectionPoints[0];

        Vector3 vertexDirection2 =
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesAboveSplittingPlane[uniqueTrianglesAboveSplittingPlane.Count - 1]]) -
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesAboveSplittingPlane[0]]);

        if (Vector3.Dot(intersectionDirection2, baseDirection) < 0)
        {
            intersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection2, intersectionDirection2) < 0)
        {
            uniqueTrianglesAboveSplittingPlane.Reverse();
        }

        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesAboveSplittingPlane, true, upperMesh);

        //store vertices restoring the hole that will be generated after cutting the mesh
        IndividualVertex v0 = new IndividualVertex(Matrix4x4.Inverse(worldMatrix).MultiplyPoint(intersectionPoints[0]), Vector3.up, Vector2.zero);
        IndividualVertex v1 = new IndividualVertex(
            Matrix4x4.Inverse(worldMatrix).MultiplyPoint(intersectionPoints[intersectionPoints.Count - 1]),
            Vector3.up, Vector2.zero);
        MeshLidPairing lidPairing = new MeshLidPairing(v0, v1);

        //TODO Impelement threading for lid pairings
        //lidPairings.Add(lidPairing);

    }

    private void UnOptimizedGetFaceToPlaneIntersectionPoints(Face face, Vector3 position, Vector3 normal, out List<Vector3> intersectionPoints)
    {
        intersectionPoints = new List<Vector3>();

        Vector3 worldV0 = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v0]);
        Vector3 worldV1 = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v1]);
        Vector3 worldV2 = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v2]);

        Vector3 worldV3 = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v0]);
        Vector3 worldV4 = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v1]);
        Vector3 worldV5 = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v2]);

        List<Vector3> triangleIntersectionPoints = CuttableTreeScript.UnOptimizedFindTriangleToPlaneIntersectionPoint
            (worldV0, worldV1, worldV2, position, normal);

        List<Vector3> secondTriangleIntersectionPoints = CuttableTreeScript.UnOptimizedFindTriangleToPlaneIntersectionPoint
            (worldV3, worldV4, worldV5, position, normal);


        foreach (var intersectionPoint in triangleIntersectionPoints)
        {

            intersectionPoints.Add(intersectionPoint);
        }

        foreach (var intersectionPoint in secondTriangleIntersectionPoints)
        {
            intersectionPoints.Add(intersectionPoint);

        }

    }

    private bool[] CheckIfPointsAreAbovePlane(Vector3[] worldTrianglePoints, Vector3 position, Vector3 normal)
    {
        bool[] result = new bool[worldTrianglePoints.Length];

        for (int i = 0; i < worldTrianglePoints.Length; i++)
        {
            result[i] = Utils.IsPointAbovePlane(worldTrianglePoints[i], position, normal);
        }

        return result;
    }

    private List<int> GetUniqueVertices(List<int> nonUniqueIndiceList)
    {
        List<int> uniqueVertices = new List<int>();
        List<Vector3> seenVertices = new List<Vector3>();


        for (int i = 0; i < nonUniqueIndiceList.Count; i++)
        {
            bool hasSeenVertex = false;
            Vector3 vertexToTest = meshVertices[nonUniqueIndiceList[i]];

            foreach (Vector3 vertex in seenVertices)
            {
                if (vertexToTest.Equals(vertex))
                {
                    hasSeenVertex = true;
                    break;
                }
            }

            if (!hasSeenVertex)
            {
                //Debug.Log("unique vertex found  ")
                seenVertices.Add(meshVertices[nonUniqueIndiceList[i]]);
                uniqueVertices.Add(nonUniqueIndiceList[i]);
            }



        }

        return uniqueVertices;
    }

    private List<Vector3> GetUniqueVector3Collection(List<Vector3> nonUniqueCollection)
    {
        List<Vector3> uniqueCollection = new List<Vector3>();

        for (int i = 0; i < nonUniqueCollection.Count; i++)
        {
            bool hasSeenElement = false;

            foreach (var vertex in uniqueCollection)
            {
                if ((nonUniqueCollection[i] - vertex).magnitude < 0.001f)
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

    private Vector3 GetWorldTriangleCentroid(List<int> indices)
    {
        Vector3 result = Vector3.zero;
        foreach (var x in indices)
        {
            result += meshVertices[x];
        }

        result /= indices.Count;


        return worldMatrix.MultiplyPoint(result);
    }

    private void assembleFacesFromSplitVertices(List<Vector3> uniqueIntersectionPoints, List<int> trianglesInSplitPlane,
        bool isIntersectionPointBottomLeftVertex, NativeQueue<JobFace>.ParallelWriter meshToPopulate)
    {
        Matrix4x4 inverseWorld = Matrix4x4.Inverse(worldMatrix);

        int iterationCount = uniqueIntersectionPoints.Count > trianglesInSplitPlane.Count ? uniqueIntersectionPoints.Count : trianglesInSplitPlane.Count;

        List<ConnectionTypeToCentroid> types = new List<ConnectionTypeToCentroid>();

        for (int i = 0; i < iterationCount - 1; i++)
        {
            Profiler.BeginSample("single quad/triangle assembly");

            int currentItersectionPointI = CuttableTreeScript.GetCurrentIndex(uniqueIntersectionPoints.Count, i);
            Vector3 objectSpaceItersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(currentItersectionPointI));

            bool nextIntersectionPointExist = i + 1 < uniqueIntersectionPoints.Count;
            bool nextTrianglePointExist = i + 1 < trianglesInSplitPlane.Count;

            JobVertex bottomLeftVertex = new JobVertex();
            JobVertex upperLeftVertex = new JobVertex();

            JobTriangle tri1 = new JobTriangle();
            JobTriangle tri2 = new JobTriangle();

            JobFace jobFace = new JobFace();

            if (nextTrianglePointExist && nextIntersectionPointExist)
            {
                JobVertex bottomRightVertex = new JobVertex();
                JobVertex upperRightVertex = new JobVertex();

                Vector3 nextObjectSpaceIntersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(i + 1));

                //all intersection points are bottom, triangles points are upper
                if (isIntersectionPointBottomLeftVertex)
                {
                    Profiler.BeginSample("Creating Quad - abovePlane");

                    upperLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[i]);

                    upperRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[i + 1]);

                    bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                    bottomRightVertex.Init(nextObjectSpaceIntersectionPoint, upperRightVertex.normal, new Vector2(1, 1));

                    tri1.Init(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                    tri2.Init(bottomLeftVertex, upperRightVertex, bottomRightVertex);

                    tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2);

                    tri2.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2);

                    //store its position and connection type
                    ConnectionTypeToCentroid tri1Type;
                    tri1Type.objectSpaceCentroid = tri1.GetObjectSpaceCentroid();
                    tri1Type.tct = TriangleConnectionType.DoubleOriginalPoint;

                    ConnectionTypeToCentroid tri2Type;
                    tri2Type.objectSpaceCentroid = tri2.GetObjectSpaceCentroid();
                    tri2Type.tct = TriangleConnectionType.DoubleIntersection;

                    types.Add(tri1Type);
                    types.Add(tri2Type);

                    Profiler.EndSample();

                }
                //all intersection points are upper, triangles points are bottom
                else
                {
                    Profiler.BeginSample("Creating Quad - belowPlane");

                    bottomLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[i]);

                    bottomRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[i + 1]);

                    upperLeftVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal, new Vector2(1, 1));

                    upperRightVertex.Init(nextObjectSpaceIntersectionPoint, bottomRightVertex.normal, new Vector2(1, 1));

                    tri1.Init(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                    tri2.Init(bottomLeftVertex, upperRightVertex, bottomRightVertex);

                    tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2);
                    tri2.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2);

                    ConnectionTypeToCentroid tri1Type;
                    tri1Type.objectSpaceCentroid = tri1.GetObjectSpaceCentroid();
                    tri1Type.tct = TriangleConnectionType.DoubleIntersection;

                    ConnectionTypeToCentroid tri2Type;
                    tri2Type.objectSpaceCentroid = tri2.GetObjectSpaceCentroid();
                    tri2Type.tct = TriangleConnectionType.DoubleOriginalPoint;

                    types.Add(tri1Type);
                    types.Add(tri2Type);

                    Profiler.EndSample();
                }


            }
            else
            {
                JobVertex bottomRightVertex = new JobVertex();
                JobVertex upperRightVertex = new JobVertex();

                int currentTriangleIndex = CuttableTreeScript.GetCurrentIndex(trianglesInSplitPlane.Count, i);

                if (nextIntersectionPointExist)
                {
                    Vector3 nextObjectSpaceIntersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(i + 1));

                    //all intersection points are bottom, triangles points are upper
                    if (isIntersectionPointBottomLeftVertex)
                    {
                        Profiler.BeginSample("Creating Triangle - abovePlane");

                        upperLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex]);

                        bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                        bottomRightVertex.Init(nextObjectSpaceIntersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                        tri1.Init(upperLeftVertex, bottomRightVertex, bottomLeftVertex);
                        tri1.AttemptNormalBasedVertexCorrection(upperLeftVertex.normal, 1, 2);

                        Profiler.EndSample();

                    }
                    //all intersection points are upper, triangles points are bottom
                    else
                    {
                        Profiler.BeginSample("Creating Triangle - belowPlane");

                        bottomLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex]);

                        upperLeftVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal, new Vector2(1, 1));

                        upperRightVertex.Init(nextObjectSpaceIntersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                        tri1.Init(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                        tri1.AttemptNormalBasedVertexCorrection(upperLeftVertex.normal, 1, 2);

                        Profiler.EndSample();
                    }
                }
                //
                if (nextTrianglePointExist)
                {
                    //all intersection points are bottom, triangles points are upper
                    if (isIntersectionPointBottomLeftVertex)
                    {
                        Profiler.BeginSample("Creating Triangle - abovePlane");

                        bottomLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex]);

                        bottomRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex + 1]);

                        TriangleConnectionType tct = GetClosestConnectionTypeByCentroidProjection(uniqueIntersectionPoints[0],
                            (uniqueIntersectionPoints[1] - uniqueIntersectionPoints[0]).normalized, types);

                        ConnectionTypeToCentroid tri1Type = new ConnectionTypeToCentroid();

                        if (tct == TriangleConnectionType.DoubleOriginalPoint)
                        {
                            tri1Type.tct = TriangleConnectionType.DoubleIntersection;
                            upperRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex - 1]);
                        }
                        if (tct == TriangleConnectionType.DoubleIntersection)
                        {
                            tri1Type.tct = TriangleConnectionType.DoubleOriginalPoint;
                            upperRightVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal, new Vector2(1, 1));
                        }

                        tri1.Init(bottomLeftVertex, upperRightVertex, bottomRightVertex);
                        tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2);

                        tri1Type.objectSpaceCentroid = tri1.GetObjectSpaceCentroid();
                        types.Add(tri1Type);

                        Profiler.EndSample();
                    }
                    //all intersection points are upper, triangles points are bottom
                    else
                    {
                        Profiler.BeginSample("Creating Triangle - belowPlane");

                        upperLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex]);

                        upperRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex + 1]);

                        bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                        tri1.Init(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                        tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2);

                        Profiler.EndSample();
                    }
                }

            }


            jobFace.jt1 = tri1;
            jobFace.jt2 = tri2;
            jobFace.isFilled = true;
            meshToPopulate.Enqueue(jobFace);

            Profiler.EndSample();
        }
    }

    public TriangleConnectionType GetClosestConnectionTypeByCentroidProjection(Vector3 basePosition, Vector3 desiredDirection, List<ConnectionTypeToCentroid> cttc)
    {
        TriangleConnectionType tct = TriangleConnectionType.DefaultNoType;
        float closestConnectionLength = float.MinValue;

        foreach (ConnectionTypeToCentroid singleTypeToCentroid in cttc)
        {
            Vector3 worldSpaceCentroid = worldMatrix.MultiplyPoint(singleTypeToCentroid.objectSpaceCentroid);
            Vector3 currentDirection = (worldSpaceCentroid - basePosition).normalized;

            float currentFoundClosest = Vector3.Dot(currentDirection, desiredDirection);

            if (currentFoundClosest > closestConnectionLength)
            {
                closestConnectionLength = currentFoundClosest;
                tct = singleTypeToCentroid.tct;
            }

        }
        return tct;
    }


}

