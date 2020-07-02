using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

using UnityEngine;
using UnityEngine.Profiling;

public struct JobFace
{
    public JobTriangle jt1;
    public JobTriangle jt2;
    public bool isFilled;
}

public struct JobTriangle
{
    public JobVertex v0;
    public JobVertex v1;
    public JobVertex v2;
    public bool isFilled;

    public IndividualTriangle ToIndividualTriangle()
    {
        IndividualTriangle result = new IndividualTriangle(v0.ToIndividualVertex(),v1.ToIndividualVertex(),v2.ToIndividualVertex());

        return result;
    }

    public void Init(JobVertex v0,JobVertex v1,JobVertex v2)
    {
        this.v0 = v0;
        this.v1 = v1;
        this.v2 = v2;
        isFilled = true;
    }

    public bool AttemptNormalBasedVertexCorrection(Vector3 actualNormal, int flipIndexA, int flipIndexB)
    {
        if (Vector3.Dot(actualNormal, Vector3.Cross(v1.position - v0.position, v2.position - v0.position)) < 0)
        {
            FlipTriangle(flipIndexA, flipIndexB);
            return true;
        }
        return false;

    }

    public void FlipTriangle(int flipIndexA, int flipIndexB)
    {
        JobVertex[] vertices = new JobVertex[3];
        JobVertex tempVertex;

        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;

        tempVertex = vertices[flipIndexB];

        vertices[flipIndexB] = vertices[flipIndexA];
        vertices[flipIndexA] = tempVertex;

        v0 = vertices[0];
        v1 = vertices[1];
        v2 = vertices[2];
    }


    public Vector3 GetObjectSpaceCentroid()
    {
        return (v0.position + v1.position + v2.position) / 3;
    }

    public void PopulateList(List<Vector3> vertices,List<Vector3> normals,List<Vector2> uvs)
    {
        v0.PopulateList(vertices, normals, uvs);
        v1.PopulateList(vertices, normals, uvs);
        v2.PopulateList(vertices, normals, uvs);
    }

    public void PopulateArray(Vector3[] vertices,Vector3[] normals,Vector2[] uvs,ref int index)
    {
        v0.PopulateArray(vertices, normals, uvs,ref index);
        v1.PopulateArray(vertices, normals, uvs, ref index);
        v2.PopulateArray(vertices, normals, uvs, ref index);

    }


}

public struct JobVertex
{
    public Vector3 position;
    public Vector3 normal;
    public Vector2 uv;

    public IndividualVertex ToIndividualVertex()
    {
        IndividualVertex result = new IndividualVertex(position, normal, uv);
        return result;
    }

    public void Init(Vector3 pPosition, Vector3 pNormal, Vector2 pUV)
    {
        position = pPosition;
        normal = pNormal;
        uv = pUV;
    }

    public void Init(NativeArray<Vector3> meshPosition, NativeArray<Vector3> meshNormals,NativeArray<Vector2> meshUVs,int triangleIndex)
    {
        position = meshPosition[triangleIndex];
        normal = meshNormals[triangleIndex];
        uv = meshUVs[triangleIndex];

    }

    public void PopulateList(List<Vector3> vertices, List<Vector3> normals, List<Vector2> uvs)
    {
        vertices.Add(position);
        normals.Add(normal);
        uvs.Add(uv);
    }

    public void PopulateArray(Vector3[] vertices,Vector3[] normals,Vector2[] uvs,ref int index)
    {
        vertices[index] = position;
        normals[index] = normal;
        uvs[index] = uv;
        index++;

    }
    


}

public struct CutHolePairing
{
    public Vector3 v0;
    public Vector3 v1;

    public void Init(Vector3 v0,Vector3 v1)
    {
        this.v0 = v0;
        this.v1 = v1;
    }
    
}


struct FaceToPrimitiveMeshJob : IJobParallelFor
{
    [WriteOnly]
    public NativeQueue<JobFace>.ParallelWriter lowerMesh;
    [WriteOnly]
    public NativeQueue<JobFace>.ParallelWriter upperMesh;
    [WriteOnly]
    public NativeQueue<CutHolePairing>.ParallelWriter holePairings;

    [ReadOnly]
    public NativeArray<Face> faces;
    [ReadOnly]
    public NativeArray<int> meshTriangles;
    [ReadOnly]
    public NativeArray<Vector3> meshNormals;
    [ReadOnly]
    public NativeArray<Vector3> meshVertices;
    [ReadOnly]
    public NativeArray<Vector2> meshUVs;

    public Vector3 transformedPosition;
    public Vector3 transformedNormal;

    public Vector3 position;
    public Vector3 normal;

    public Matrix4x4 worldMatrix;

    public Vector3 preCutCentroid;

    public void Execute(int i)
    {
        TriangleSplitState tri1CheckResult = TriangleSplitState.DefaultNoTriangle;
        TriangleSplitState tri2CheckResult = TriangleSplitState.DefaultNoTriangle;

        bool hasTriangle1 = faces[i].tri1.v0 != -1;

        if (hasTriangle1)
        {
            tri1CheckResult = triangleToPlaneCheck(
                meshVertices[faces[i].tri1.v0],
                meshVertices[faces[i].tri1.v1],
                meshVertices[faces[i].tri1.v2],
                transformedPosition, transformedNormal);

        }

        bool hasTriangle2 = faces[i].tri2.v0 != -1;

        if (hasTriangle2)
        {
            tri2CheckResult = triangleToPlaneCheck(
                meshVertices[faces[i].tri2.v0],
                meshVertices[faces[i].tri2.v1],
                meshVertices[faces[i].tri2.v2],
                transformedPosition, transformedNormal);
        }

        bool isBothTrianglesExist = hasTriangle1 && hasTriangle2;

        //if both triangles exist and have the same triangle split state
        if (isBothTrianglesExist && tri1CheckResult == tri2CheckResult)
        {
            organizeFaceBasedOnTriangleSplitState(worldMatrix, tri1CheckResult, faces[i]);
        }
        //if both triangles exist but one triangle is intersecting but the other is either above or below the splitting plane
        else if (isBothTrianglesExist && tri1CheckResult != tri2CheckResult)
        {
            Vector3[] worldTrianglePointPositions = new Vector3[6];

            worldTrianglePointPositions[0] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri1.v0]);
            worldTrianglePointPositions[1] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri1.v1]);
            worldTrianglePointPositions[2] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri1.v2]);
            worldTrianglePointPositions[3] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri2.v0]);
            worldTrianglePointPositions[4] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri2.v1]);
            worldTrianglePointPositions[5] = worldMatrix.MultiplyPoint(meshVertices[faces[i].tri2.v2]);


            intersectingFaceSplit(faces[i], worldTrianglePointPositions);

        }
        //one of the triangles in the face do not exist ( this face only has one triangle)
        else if (!isBothTrianglesExist)
        {
           // Debug.Log("!isBothTrianglesExist");
            if (hasTriangle1)
            {
                FindDecisionForSingularTriangle(tri1CheckResult, faces[i].tri1,i);
            }
            if (hasTriangle2)
            {
                FindDecisionForSingularTriangle(tri2CheckResult, faces[i].tri2,i);
            }
        }
    }

    private TriangleSplitState triangleToPlaneCheck(Vector3 transformedV0, Vector3 transformedV1, Vector3 transformedV2, Vector3 position, Vector3 normal)
    {
        bool v0AbovePlane = Utils.IsPointAbovePlane(transformedV0, position, normal);
        bool v1AbovePlane = Utils.IsPointAbovePlane(transformedV1, position, normal);
        bool v2AbovePlane = Utils.IsPointAbovePlane(transformedV2, position, normal);

        if (v0AbovePlane && v1AbovePlane && v2AbovePlane)
        {
            return TriangleSplitState.AbovePlane;
        }
        else if (!v0AbovePlane && !v1AbovePlane && !v2AbovePlane)
        {
            return TriangleSplitState.BelowPlane;
        }
        else
        {
            return TriangleSplitState.IntersectionOnPlane;
        }
        
    }

    private void organizeFaceBasedOnTriangleSplitState(Matrix4x4 world, TriangleSplitState state, Face face)
    {
        switch (state)
        {
            case TriangleSplitState.AbovePlane:
                JobFace jf;
                jf.isFilled = true;
                jf.jt1 = face.tri1.ToJobTriangle(meshVertices, meshNormals, meshUVs);
                jf.jt2 = face.tri2.ToJobTriangle(meshVertices, meshNormals, meshUVs);

                upperMesh.Enqueue(jf);

                break;

            case TriangleSplitState.BelowPlane:
                JobFace jf2;
                jf2.isFilled = true;
                jf2.jt1 = face.tri1.ToJobTriangle(meshVertices, meshNormals, meshUVs);
                jf2.jt2 = face.tri2.ToJobTriangle(meshVertices, meshNormals, meshUVs);

                lowerMesh.Enqueue(jf2);

                break;

            case TriangleSplitState.IntersectionOnPlane:
                Profiler.BeginSample("Object To World ");

                Vector3[] worldTrianglePointPositions = new Vector3[6];
                worldTrianglePointPositions[0] = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v0]);
                worldTrianglePointPositions[1] = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v1]);
                worldTrianglePointPositions[2] = worldMatrix.MultiplyPoint(meshVertices[face.tri1.v2]);
                worldTrianglePointPositions[3] = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v0]);
                worldTrianglePointPositions[4] = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v1]);
                worldTrianglePointPositions[5] = worldMatrix.MultiplyPoint(meshVertices[face.tri2.v2]);

                Profiler.EndSample();

                Profiler.BeginSample("intersectingFaceSplit(face, worldTrianglePointPositions)");

                intersectingFaceSplit(face, worldTrianglePointPositions);

                Profiler.EndSample();

                break;



        }
    }

    private void intersectingFaceSplit( Face face, Vector3[] worldTrianglePointPositions)
    {
        Profiler.BeginSample("Checking intersection Points");

        List<IntersectionQuery> foundIntersectionPoint;
        MeshSplittingStatics.GetFaceToPlaneIntersectionPoints(meshVertices,meshUVs,face
            ,transformedPosition,transformedNormal, out foundIntersectionPoint);

        Profiler.EndSample();


        Profiler.BeginSample("Organize points into list");

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

        Profiler.EndSample();


        Profiler.BeginSample("Get unique Lists");
        List<int> uniqueTrianglesAboveSplittingPlane = GetUniqueVertices(trianglesAboveSplittingPlane);
        List<int> uniqueTrianglesBelowSplittingPlane = GetUniqueVertices(trianglesBelowSplittingPlane);
        List<IntersectionQuery> uniqueIntersectionPoints = MeshSplittingStatics.GetUniqueIntersectionQueryCollection(foundIntersectionPoint);

        Profiler.EndSample();

        //create bottom faces
        Vector3 basePosition;
        Vector3 baseDirection;

        if (uniqueTrianglesBelowSplittingPlane.Count < 2 && uniqueTrianglesAboveSplittingPlane.Count < 2) { return; }
        if (uniqueIntersectionPoints.Count < 2) { return; }


        Profiler.BeginSample("Sort Intersection Points");

        basePosition = preCutCentroid;
        baseDirection = Vector3.Cross(Vector3.up, (worldTrianglePointPositions[0] - preCutCentroid)).normalized;

        IntersectionQueryComparer ic = new IntersectionQueryComparer
            (baseDirection, basePosition, worldMatrix);

        Profiler.BeginSample("Actual sorting intersection points");
        uniqueIntersectionPoints.Sort(ic);

        

        Profiler.EndSample();

        Profiler.EndSample();

        Profiler.BeginSample("Sort Triangle Points");

        Vector3 indexBasedDirection = (worldMatrix.MultiplyPoint(foundIntersectionPoint[foundIntersectionPoint.Count - 1].intersectionPosition)
            - worldMatrix.MultiplyPoint(foundIntersectionPoint[0].intersectionPosition));
        indexBasedDirection.Normalize();

        IndexDirectionComparer idc = new IndexDirectionComparer(indexBasedDirection
        , basePosition, meshVertices.ToArray(), worldMatrix);

        Vector3 belowTriangleCentroid = GetWorldTriangleCentroid(uniqueTrianglesBelowSplittingPlane);
        Vector3 aboveTriangleCentroid = GetWorldTriangleCentroid(uniqueTrianglesAboveSplittingPlane);


        uniqueTrianglesBelowSplittingPlane.Sort(idc);

        uniqueTrianglesAboveSplittingPlane.Sort(idc);

        Profiler.EndSample();


        List<IntersectionQuery> intersectionPoints = new List<IntersectionQuery>();

        intersectionPoints.Add(uniqueIntersectionPoints[0]);
        intersectionPoints.Add(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]);


        //------------------------------- create bottom part----------------------------------------------------//
        Profiler.BeginSample("assembleFacesFromSplitVertices lowerMesh");


        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesBelowSplittingPlane, false, lowerMesh);

        Profiler.EndSample();

        //------------------------------- create above part----------------------------------------------------//
        Profiler.BeginSample("assembleFacesFromSplitVertices upperMesh");

        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesAboveSplittingPlane, true, upperMesh);

        Profiler.EndSample();

        //store vertices restoring the hole that will be generated after cutting the mesh

        CutHolePairing cutHolePairing = new CutHolePairing();
        cutHolePairing.Init(intersectionPoints[0].intersectionPosition, intersectionPoints[intersectionPoints.Count - 1].intersectionPosition);
        holePairings.Enqueue(cutHolePairing);


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
                
                if (Vector3.Magnitude(nonUniqueCollection[i] - vertex) < 0.001f)
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

    private bool UnOptimizedFindLineToPlaneIntersection(Vector3 transformedV0, Vector3 transformedV1, Vector3 position, Vector3 normal, out Vector3 intersection)
    {
        Vector3 lineToUse = transformedV1 - transformedV0;

        Vector3 P0 = transformedV0;
        Vector3 P1 = lineToUse.normalized;
        Vector3 A = position;

        float t = (Vector3.Dot(A, normal) - Vector3.Dot(P0, normal)) / Vector3.Dot(P1, normal);

        intersection = P0 + P1 * t;

        return t > 0.0f && t < lineToUse.magnitude;
    }

    private void FindDecisionForSingularTriangle(TriangleSplitState state, Triangle tri,int executeIndex)
    {
        switch (state)
        {
            case TriangleSplitState.AbovePlane:
                JobFace jobFace;
                jobFace.isFilled = true;
                jobFace.jt1 = tri.ToJobTriangle(meshVertices, meshNormals, meshUVs);

                jobFace.jt2 = jobFace.jt1;
                jobFace.jt2.isFilled = false;

                upperMesh.Enqueue(jobFace);

                break;

            case TriangleSplitState.BelowPlane:
                JobFace jobFace2;
                jobFace2.isFilled = true;
                jobFace2.jt1 = tri.ToJobTriangle(meshVertices, meshNormals, meshUVs);

                jobFace2.jt2 = jobFace2.jt1;
                jobFace2.jt2.isFilled = false;

                lowerMesh.Enqueue(jobFace2);


                break;

            case TriangleSplitState.IntersectionOnPlane:


                Vector3[] worldTrianglePointPositions = new Vector3[3];
                worldTrianglePointPositions[0] = worldMatrix.MultiplyPoint(meshVertices[tri.v0]);
                worldTrianglePointPositions[1] = worldMatrix.MultiplyPoint(meshVertices[tri.v1]);
                worldTrianglePointPositions[2] = worldMatrix.MultiplyPoint(meshVertices[tri.v2]);

                Profiler.BeginSample("intersectingTriangleSplit");
                intersectingTriangleSplit( tri, worldTrianglePointPositions, executeIndex);
                Profiler.EndSample();



                break;
        }
    }

    private void intersectingTriangleSplit(Triangle tri, Vector3[] worldTrianglePointPositions,int executeIndex)
    {
        //find unique intersection points

        Profiler.BeginSample("UnOptimizedFindTriangleToPlaneIntersectionPoint");

        //------------ Find the 
       List<IntersectionQuery> uniqueIntersectionPoints = MeshSplittingStatics.FindTriangleToPlaneIntersectionPoint
            (meshVertices, meshUVs, tri.v0, tri.v1, tri.v2, transformedPosition, transformedNormal);
            //CuttableTreeScript.UnOptimizedFindTriangleToPlaneIntersectionPoint
            //(worldTrianglePointPositions[0], worldTrianglePointPositions[1], worldTrianglePointPositions[2], position, normal);

        Profiler.EndSample();


        if (uniqueIntersectionPoints.Count < 2) { return; }

        //----------- Get Points that are above the splitting plane and below the splitting plane--------------//

        bool[] triangleState = CheckIfPointsAreAbovePlane(worldTrianglePointPositions, position, normal);

        List<int> uniqueTrianglesAboveSplittingPlane = new List<int>();
        List<int> uniqueTrianglesBelowSplittingPlane = new List<int>();

        int[] faceIndices = tri.ToIntArray();
        //for each triangleState
        for (int i = 0; i < triangleState.Length; i++)
        {
            if (triangleState[i])
            {
                uniqueTrianglesAboveSplittingPlane.Add(faceIndices[i]);
            }
            else
            {
                uniqueTrianglesBelowSplittingPlane.Add(faceIndices[i]);
            }

        }


        //sort unique intersection points based on the vector parallel to the vector resulting from the cross product of the world up 
        //(0,1,0) and one of the vertices of the intersection points

        Profiler.BeginSample("Sorting");

        Vector3 basePosition = preCutCentroid;
        Vector3 baseDirection = Vector3.Cross(Vector3.up, (worldTrianglePointPositions[0] - preCutCentroid)).normalized;

        IntersectionQueryComparer ic = new IntersectionQueryComparer(baseDirection, basePosition, worldMatrix);
        uniqueIntersectionPoints.Sort(ic);

        //sort the points above and below the splitting plane based on the vector created by the last element of the intersectionPoint 
        //Collection and the first element in the Vector3 Collection

        Vector3 firstIntersectionPoint = worldMatrix.MultiplyPoint(uniqueIntersectionPoints[0].intersectionPosition);
        Vector3 lastIntersectionPoint = worldMatrix.MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1].intersectionPosition);

        Vector3 indexBasedDirection = lastIntersectionPoint - firstIntersectionPoint;
        indexBasedDirection.Normalize();


        IndexDirectionComparer idc = new IndexDirectionComparer(baseDirection
            , firstIntersectionPoint, meshVertices.ToArray(), worldMatrix);

        //----------- Sort Triangles using IndexDirectionComparer --------------------------------------------------------//
        uniqueTrianglesBelowSplittingPlane.Sort(idc);
        uniqueTrianglesAboveSplittingPlane.Sort(idc);
        

        Profiler.EndSample();

        //check one more time, check if points and intersection points are ordered in the right direction



        Vector3 intersectionDirection = (worldMatrix.MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1].intersectionPosition)
        - worldMatrix.MultiplyPoint(uniqueIntersectionPoints[0].intersectionPosition));

        Vector3 vertexDirection =
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesBelowSplittingPlane[uniqueTrianglesBelowSplittingPlane.Count - 1]]) -
             worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesBelowSplittingPlane[0]])
           ;


        Profiler.BeginSample("Actual assembly lowerMesh");
        assembleFacesFromSplitVertices(uniqueIntersectionPoints, uniqueTrianglesBelowSplittingPlane, false,lowerMesh);
        Profiler.EndSample();


        //use the points above the spllitting plane and the intersection points to create the triangle that will be placed in the upperPrimitiveMesh

        Vector3 intersectionDirection2 = (worldMatrix.MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1].intersectionPosition)
       - worldMatrix.MultiplyPoint(uniqueIntersectionPoints[0].intersectionPosition));

        Vector3 vertexDirection2 =
           worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesAboveSplittingPlane[uniqueTrianglesAboveSplittingPlane.Count - 1]]) -
           worldMatrix.MultiplyPoint(meshVertices[uniqueTrianglesAboveSplittingPlane[0]]);

        Profiler.BeginSample("Actual assembly upperMesh");
        assembleFacesFromSplitVertices(uniqueIntersectionPoints, uniqueTrianglesAboveSplittingPlane, true,  upperMesh);
        Profiler.EndSample();

        //use the points below the splitting plane and the intersection points to create the triangle that will be placed in the lowerPrimitiveMesh
        CutHolePairing cutHolePairing = new CutHolePairing();
        cutHolePairing.Init(uniqueIntersectionPoints[0].intersectionPosition
            , uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1].intersectionPosition);
        holePairings.Enqueue(cutHolePairing);


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

    private void assembleFacesFromSplitVertices(List<IntersectionQuery> uniqueIntersectionPoints, List<int> trianglesInSplitPlane, 
        bool isIntersectionPointBottomLeftVertex, NativeQueue<JobFace>.ParallelWriter meshToPopulate)
    {
        Matrix4x4 inverseWorld = Matrix4x4.Inverse(worldMatrix);

        int iterationCount = uniqueIntersectionPoints.Count > trianglesInSplitPlane.Count ? uniqueIntersectionPoints.Count : trianglesInSplitPlane.Count;

        List<ConnectionTypeToCentroid> types = new List<ConnectionTypeToCentroid>();

        for (int i = 0; i < iterationCount - 1; i++)
        {
            Profiler.BeginSample("single quad/triangle assembly");

            int currentItersectionPointI = CuttableTreeScript.GetCurrentIndex(uniqueIntersectionPoints.Count, i);
            Vector3 objectSpaceItersectionPoint = uniqueIntersectionPoints.ElementAt(currentItersectionPointI).intersectionPosition;
            Vector2 intersectionPointUV = uniqueIntersectionPoints.ElementAt(currentItersectionPointI).UV;

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

                Vector3 nextObjectSpaceIntersectionPoint = uniqueIntersectionPoints.ElementAt(i + 1).intersectionPosition;
                Vector2 nextIntersectionPointUV = uniqueIntersectionPoints.ElementAt(i + 1).UV;

                //all intersection points are bottom, triangles points are upper
                if (isIntersectionPointBottomLeftVertex)
                {
                    Profiler.BeginSample("Creating Quad - abovePlane");

                    upperLeftVertex.Init(meshVertices, meshNormals, meshUVs,trianglesInSplitPlane[i]);

                    upperRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[i + 1]);

                    bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, intersectionPointUV);

                    bottomRightVertex.Init(nextObjectSpaceIntersectionPoint, upperRightVertex.normal, nextIntersectionPointUV);

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

                    bottomLeftVertex.Init(meshVertices, meshNormals, meshUVs,trianglesInSplitPlane[i]);

                    bottomRightVertex.Init(meshVertices, meshNormals, meshUVs,trianglesInSplitPlane[i+1]);

                    upperLeftVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal, intersectionPointUV);

                    upperRightVertex.Init(nextObjectSpaceIntersectionPoint, bottomRightVertex.normal, nextIntersectionPointUV);

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
                    

                    Vector3 nextObjectSpaceIntersectionPoint = uniqueIntersectionPoints.ElementAt(i + 1).intersectionPosition;
                    Vector2 nextIntersectionPointUV = uniqueIntersectionPoints.ElementAt(i + 1).UV;

                    //all intersection points are bottom, triangles points are upper
                    if (isIntersectionPointBottomLeftVertex)
                    {
                        Profiler.BeginSample("Creating Triangle - abovePlane");

                        upperLeftVertex.Init(meshVertices, meshNormals, meshUVs,trianglesInSplitPlane[currentTriangleIndex]);

                        bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, intersectionPointUV);

                        bottomRightVertex.Init(nextObjectSpaceIntersectionPoint, upperLeftVertex.normal, nextIntersectionPointUV);

                        tri1.Init(upperLeftVertex, bottomRightVertex, bottomLeftVertex);
                        tri1.AttemptNormalBasedVertexCorrection(upperLeftVertex.normal, 1, 2);

                        Profiler.EndSample();

                    }
                    //all intersection points are upper, triangles points are bottom
                    else
                    {
                        Profiler.BeginSample("Creating Triangle - belowPlane");

                        bottomLeftVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex]);

                        upperLeftVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal, intersectionPointUV);

                        upperRightVertex.Init(nextObjectSpaceIntersectionPoint, upperLeftVertex.normal, nextIntersectionPointUV);

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

                        bottomRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex+1]);

                        TriangleConnectionType tct = GetClosestConnectionTypeByCentroidProjection(uniqueIntersectionPoints[0].intersectionPosition, 
                            (uniqueIntersectionPoints[1].intersectionPosition - uniqueIntersectionPoints[0].intersectionPosition).normalized, types);

                        ConnectionTypeToCentroid tri1Type = new ConnectionTypeToCentroid();
                        
                        if (tct == TriangleConnectionType.DoubleOriginalPoint)
                        {
                            tri1Type.tct = TriangleConnectionType.DoubleIntersection;
                            upperRightVertex.Init(meshVertices, meshNormals, meshUVs, trianglesInSplitPlane[currentTriangleIndex - 1]);
                        }
                        if (tct == TriangleConnectionType.DoubleIntersection)
                        {
                            tri1Type.tct = TriangleConnectionType.DoubleOriginalPoint;
                            upperRightVertex.Init(objectSpaceItersectionPoint, bottomLeftVertex.normal,intersectionPointUV);
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

                        bottomLeftVertex.Init(objectSpaceItersectionPoint, upperLeftVertex.normal, intersectionPointUV);

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
            Vector3 objectSpaceCentroid = singleTypeToCentroid.objectSpaceCentroid;
            Vector3 currentDirection = (objectSpaceCentroid - basePosition).normalized;

            float currentFoundClosest = Vector3.Dot(currentDirection, desiredDirection);

            if (currentFoundClosest > closestConnectionLength)
            {
                closestConnectionLength = currentFoundClosest;
                tct = singleTypeToCentroid.tct;
            }

        }
        return tct;
    }

    public static void DEBUG_logVertices(List<Vector3> vectors)
    {
        for (int i = 0; i < vectors.Count; i++)
        {
            Debug.Log("vert " + i + "is " + vectors[i]);
        }
    }
}
