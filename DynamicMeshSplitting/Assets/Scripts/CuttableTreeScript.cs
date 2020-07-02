using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.Profiling;
using Unity.Collections;
using Unity.Jobs;
using System;


public struct ForceApplicationInfo
{
    public Vector3 force;
    public Vector3 position;
}
public struct AdditionalSplittingParams
{
    public List<ForceApplicationInfo> additionalForces;
}

public struct ConnectionTypeToCentroid
{
    public TriangleConnectionType tct;
    public Vector3 objectSpaceCentroid;
}

public enum TriangleConnectionType
{
    DefaultNoType,
    DoubleIntersection,
    DoubleOriginalPoint,
    

}

public struct Triangle
{
    public void Init()
    {
        v0 = -1;
        v1 = -1;
        v2 = -1;
    }

    public JobTriangle ToJobTriangle(NativeArray<Vector3> meshVertices,NativeArray<Vector3> meshNormals,NativeArray<Vector2> meshUVs)
    {
        JobTriangle result;

        JobVertex jbv0;
        jbv0.position = meshVertices[v0];
        jbv0.normal = meshNormals[v0];
        jbv0.uv = meshUVs[v0];

        JobVertex jbv1;
        jbv1.position = meshVertices[v1];
        jbv1.normal = meshNormals[v1];
        jbv1.uv = meshUVs[v1];

        JobVertex jbv2;

        jbv2.position = meshVertices[v2];
        jbv2.normal = meshNormals[v2];
        jbv2.uv = meshUVs[v2];

        result.v0 = jbv0;
        result.v1 = jbv1;
        result.v2 = jbv2;
        result.isFilled = true;

        return result;
    }

    public int[] ToIntArray()
    {
        return new int[] {v0,v1,v2 };
    }

    public void InsureVertexOrdering(int indexA,int indexB,Mesh mesh)
    {
        Vector3 trueNormal = mesh.normals[v0];

        Vector3 foundNormal = Vector3.Cross(mesh.vertices[v1] - mesh.vertices[v0], mesh.vertices[v2] - mesh.vertices[v0]);

        Debug.Log("[bfr flipping]Checking for vertex at: ");
        Debug.Log("vert " + mesh.vertices[v0]);
        Debug.Log("vert " + mesh.vertices[v1]);
        Debug.Log("vert " + mesh.vertices[v2]);
        Debug.Log("Vector3.Dot(trueNormal,foundNormal)" + Vector3.Dot(trueNormal, foundNormal));

        if (Vector3.Dot(trueNormal,foundNormal) < 0)
        {
            
            FlipVertices(indexA, indexB);
        }

    }

    public void FlipVertices(int indexA, int indexB)
    {
        int[] indexArray = new int[3];
        indexArray[0] = v0;
        indexArray[1] = v1;
        indexArray[2] = v2;

        int temp = indexArray[indexA];
        indexArray[indexA] = indexArray[indexB];
        indexArray[indexB] = temp;

        v0 = indexArray[0];
        v1 = indexArray[1];
        v2 = indexArray[2];
    }

    public int v0, v1, v2;

    public void LogWorldVertexPositions(NativeArray<Vector3> meshVertices,Matrix4x4 world)
    {
        Debug.Log("-----------------------LogWorldVertexPositions-----------------------------");
        Debug.Log(" v0 " + world.MultiplyPoint(meshVertices[v0]));
        Debug.Log(" v1 " + world.MultiplyPoint(meshVertices[v1]));
        Debug.Log(" v2 " + world.MultiplyPoint(meshVertices[v2]));
    }

}

public struct Face
{ 
    public bool isFilled()
    {
        return tri1.v0 != -1;
    }

    public void MarkUnfilled()
    {
        tri1.v0 = -1;
    }

    public void Init()
    {
        tri1.Init();
        tri2.Init();
    }


    public int[] ToIntArray()
    {
        int[] result = new int[6];

        result[0] = tri1.v0;
        result[1] = tri1.v1;
        result[2] = tri1.v2;

        result[3] = tri2.v0;
        result[4] = tri2.v1;
        result[5] = tri2.v2;

        return result;
    }

    public void ToLog()
    {
        Debug.Log("Tri1 " + tri1.v0 + "," + tri1.v1 + "," + tri1.v2);
        Debug.Log("Tri1 " + tri2.v0 + "," + tri2.v1 + "," + tri2.v2);
    }

    public void ToLogPosition()
    {

    }

    public Triangle tri1;
    public Triangle tri2;
}

public class TreeSplitCollisionBox
{
    public List<Face> faces;

    public Vector3 min;
    public Vector3 max;

}

public enum TriangleSplitState
{
    BelowPlane,
    AbovePlane,
    IntersectionOnPlane,
    DefaultNoTriangle
}

public enum PointSplitState
{
    BelowPlane,
    AbovePlane
}


[RequireComponent(typeof(MeshFilter))]
public class CuttableTreeScript : MonoBehaviour
{
    //public native
    private List<TreeSplitCollisionBox> collisionBoxes;

    [SerializeField] public CuttableMeshPhysicsManager meshPhysicsManager;

    [SerializeField] int debugShowIndex = 2;
    [SerializeField] float debugSphereSize = 1.0f;

    MeshFilter meshFilter;
    Mesh mesh;

    [SerializeField] public GameObject DebugObjectTest;

    private List<MeshLidPairing> lidPairings = new List<MeshLidPairing>();

    private Vector3 preCutCentroid;
    private Vector3 objectSpacePreCutCentroid;

    public int leafParticleIndex;
    public float cutForceMultiplier;
    [Range(-1, 3)] public int treeHP = -1;

    public bool isFirstTree = false;

    public static bool useMultithreadedVersion = true;

    public Material cutMaterial = null;

    public PreAllocator nativeArrayAllocator;

    // a way to identify one cuttable mesh from another
    public int treeTypeID = 0;


    private void Start()
    {
        InitializeCuttableTree();

        if(isFirstTree)
        {
            GameObject allocator = GameObject.FindGameObjectWithTag("PreAllocator");

            if(allocator)
            {
                nativeArrayAllocator = GameObject.FindGameObjectWithTag("PreAllocator").GetComponent<PreAllocator>();
            }
            
        }
    }

    void InitializeCuttableTree()
    {
        collisionBoxes = new List<TreeSplitCollisionBox>();

        //----------- Get Mesh -------------------//
        meshFilter = gameObject.GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        //------------------ Initialize data structures for cutting -----------------------//
        if(useMultithreadedVersion)
        {
            multithreadedBruteForceCollisionBoxInitialize();
        }
        else
        {
            bruteForceCollisionBoxInitialize();
        }

        lidPairings.Clear();

        preCutCentroid = Utils.GetObjectSpaceMeshCentroid(mesh);

        objectSpacePreCutCentroid = preCutCentroid;

        meshPhysicsManager = GetMeshColliderGenerator();
    }

    /// <summary>
    /// Splits the mesh contained in this gameObject's MeshFilter based on an infinite cutting plane 
    /// </summary>
    /// <param name="position"> The position of the cutting plane. </param>
    /// <param name="normal"> The normal of the cutting plane. </param>
    /// <param name="seperationForce"> The scalar value of the force that has a direction parallel to the cutting plane normal which is 
    /// added to the newly created mesh after it is split. </param>
    /// <returns></returns>
    public GameObject CutAt(Vector3 position, Vector3 normal, float seperationForce
        ,AdditionalSplittingParams otherSplittingParams = new AdditionalSplittingParams())
    {
        Profiler.BeginSample("[cut] CutAt");
        normal.Normalize();

        Profiler.BeginSample("[cut] MatrixMath");



        Matrix4x4 worldMatrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.localScale);
        preCutCentroid = worldMatrix.MultiplyPoint(preCutCentroid);

        Matrix4x4 inverseWorldMatrix = Matrix4x4.Inverse(worldMatrix);

        Vector3 transformedPosition = inverseWorldMatrix.MultiplyPoint3x4(position);
        Vector3 transformedNormal = (Matrix4x4.Transpose(worldMatrix)).MultiplyVector(normal);

        Profiler.EndSample();

        Profiler.BeginSample("[cut] Splitting The Mesh Into Primitives");

        Profiler.BeginSample("Create native queues");

        //------------------- Initialize Collections that will be used to store the cut results -----------------------------//
       
        NativeQueue<JobFace> rawLowerMeshQueue;
        NativeQueue<JobFace> rawUpperMeshQueue;
        NativeQueue<CutHolePairing> generatedHoleFilling = new NativeQueue<CutHolePairing>(Allocator.TempJob);

        GetPreAllocatedNativeQueue(out rawLowerMeshQueue, out rawUpperMeshQueue);

        Profiler.EndSample();

        //---------Populate Collections by splitting the meshes that are above,below,and
        // intersecting with the cutting plane ------------------------//

        foreach (TreeSplitCollisionBox collisionBox in collisionBoxes)
        {
            multithreadedPopulatePrimitiveMeshes(rawLowerMeshQueue, rawUpperMeshQueue, generatedHoleFilling, transformedPosition,
                transformedNormal, worldMatrix, position, normal, collisionBox.faces);
        }

        Profiler.EndSample();

        //------- Get center of the intersection points created from the cut------------------------//
        Vector3 centerPoint = Vector3.zero;
        int intersectionVertexCount = generatedHoleFilling.Count;
    
        List<Vector3> holes = new List<Vector3>();

        while (generatedHoleFilling.TryDequeue(out CutHolePairing holePairing ))
        {
            centerPoint += holePairing.v0;
            centerPoint += holePairing.v1;

            holes.Add(holePairing.v0);
            holes.Add(holePairing.v1);
        }

        centerPoint /= (intersectionVertexCount * 2);

        IndividualVertex vertex = new IndividualVertex(centerPoint, Vector3.zero, Vector2.zero);

        List<IndividualTriangle> trianglesBelow = new List<IndividualTriangle>();

        //--------------------- Create a mesh to cover up the hole created by the cut------------------------//

        GameObject bottomHoleCover = InstantiateHole(intersectionVertexCount, holes, centerPoint,  normal);
        GameObject upperHoleCover = InstantiateHole(intersectionVertexCount, holes, centerPoint, normal, false);

        //-------------------------- Reinitialize the original mesh ------------------------------------//
        Profiler.BeginSample("[cut] Reinitialize the original mesh ");
        populateMesh(rawLowerMeshQueue, mesh);
        Profiler.EndSample();

        InitializeCuttableTree();

        if (meshPhysicsManager)
        {
            bool rigidBodyNeeded = !isFirstTree;
            meshPhysicsManager.GenerateMeshColliderFromCut(mesh, rigidBodyNeeded);
        }
        
        //---------------------- Create a new mesh and assign in to the newly created mesh-------------------------------//
        Profiler.BeginSample("[cut] Create a new mesh and assign in to the newly created mesh");
        GameObject newTree;
        var otherMeshPhysicsManager = InstantiateTreePiece(rawUpperMeshQueue, out newTree);
        Profiler.EndSample();



        //---------------------- Re assign the leaves to its correct parent--------------------------//

        //Profiler.BeginSample("[cut] Reinitialize new mesh");
        Profiler.BeginSample("[cut] displace leaves");
        DisplaceLeaves(position, normal, gameObject, otherMeshPhysicsManager.gameObject);
        Profiler.EndSample();

        if(intersectionVertexCount == 0) { centerPoint = transform.position; }
        otherMeshPhysicsManager.AddForceAt(seperationForce * cutForceMultiplier, normal.normalized, centerPoint);

        if(otherSplittingParams.additionalForces != null)
        {
            foreach (var forceInfo in otherSplittingParams.additionalForces)
            {
                otherMeshPhysicsManager.AddForceAt(forceInfo.force.magnitude, forceInfo.force, forceInfo.position);
            }
        }
        

        //---------------------- clear up or dispose of natuve queues-------------------//
        if (nativeArrayAllocator)
        {
            rawLowerMeshQueue.Clear();
            rawUpperMeshQueue.Clear();
        }
        else
        {
            rawLowerMeshQueue.Dispose();
            rawUpperMeshQueue.Dispose();
        }
       
        generatedHoleFilling.Dispose();

        //------------------- Set generated holes to respective parents -----------------------------//

        bottomHoleCover.transform.SetParent(transform);
        upperHoleCover.transform.SetParent(newTree.transform);

        bottomHoleCover.tag = "hole";
        upperHoleCover.tag = "hole";

        Profiler.EndSample();

        return newTree;
    }

    private void GetPreAllocatedNativeQueue(out NativeQueue<JobFace> rawLowerMeshQueue, out NativeQueue<JobFace> rawUpperMeshQueue)
    {
        if (nativeArrayAllocator)
        {
            rawLowerMeshQueue = nativeArrayAllocator.GetLowerMeshQueue();
            rawUpperMeshQueue = nativeArrayAllocator.GetUpperMeshQueue();
        }
        else
        {
            rawLowerMeshQueue = new NativeQueue<JobFace>(Allocator.TempJob);
            rawUpperMeshQueue = new NativeQueue<JobFace>(Allocator.TempJob);
        }
    }

    private GameObject InstantiateHole(int intersectionVertexCount,List<Vector3> holes,Vector3 centerPoint,Vector3 cuttingNormal,bool isBottom = true)
    {

        float normalMultiplier = isBottom ? 1.0f : -1.0f;

        GameObject HoleCover = new GameObject("lowerHoleCover");

        HoleCover.transform.position = transform.position;
        HoleCover.transform.rotation = transform.rotation;
        HoleCover.transform.localScale = transform.localScale;

        Mesh holeMesh = HoleCover.AddComponent<MeshFilter>().mesh;
        MeshRenderer meshRenderer = HoleCover.AddComponent<MeshRenderer>();

        if (cutMaterial)
        {
            meshRenderer.sharedMaterial = cutMaterial;
        }
        else
        {
            meshRenderer.sharedMaterial = GetComponent<MeshRenderer>().sharedMaterial;
        }

        //initialize list of vertices 
        Vector3[] holeVertices = new Vector3[intersectionVertexCount * 3];
        Vector3[] holeNormals = new Vector3[holeVertices.Length];
        Vector2[] holeUVs = new Vector2[holeVertices.Length];
        int[] holeTriangles = new int[holeVertices.Length];



        for (int i = 0, j = 0; i < holeVertices.Length; i += 3, j++)
        {
            holeVertices[i] = isBottom? holes[i - j] : holes[i - j + 1];
            holeVertices[i + 1] = isBottom ?  holes[i - j + 1] : holes[i - j];
            holeVertices[i + 2] = centerPoint;

            //TODO find out why this patch was needed in the first place
            NormalBasedVertexCorrection(holeVertices, i, i + 1, i + 2, cuttingNormal * normalMultiplier);

            holeNormals[i] = cuttingNormal  * normalMultiplier;
            holeNormals[i + 1] = cuttingNormal * normalMultiplier;
            holeNormals[i + 2] = cuttingNormal * normalMultiplier;

            holeUVs[i] = new Vector2(1, 0);
            holeUVs[i + 1] = new Vector2(-1, 0);
            holeUVs[i + 2] = new Vector2(1, 1);


        }

        for (int i = 0; i < holeTriangles.Length; i++)
        {
            holeTriangles[i] = i;
        }

        holeMesh.Clear();
        holeMesh.vertices = holeVertices;
        holeMesh.normals = holeNormals;
        holeMesh.uv = holeUVs;
        holeMesh.triangles = holeTriangles;

        return HoleCover;

    }

    private void NormalBasedVertexCorrection(Vector3[] holeVertices,int index,int indexP1,int indexP2,Vector3 cuttingNormal)
    {
        Vector3 foundNormal = Vector3.Cross(holeVertices[indexP1] - holeVertices[index]
            , holeVertices[indexP2] - holeVertices[index]);

        if(Vector3.Dot(foundNormal,cuttingNormal) < 0)
        {
            Vector3 tempP1 = holeVertices[indexP1];
            holeVertices[indexP1] = holeVertices[index];
            holeVertices[index] = tempP1;
        }


    }
   

    private void multithreadedPopulatePrimitiveMeshes(NativeQueue<JobFace> rawLowerMeshQueue, NativeQueue<JobFace> rawUpperMeshQueue,
        NativeQueue<CutHolePairing> generatedHoleFilling, Vector3 transformedPosition, Vector3 transformedNormal, Matrix4x4 worldMatrix,
       Vector3 position, Vector3 normal, List<Face> meshfaces)
    {

        Profiler.BeginSample("Multhithreaded splitting job");

        FaceToPrimitiveMeshJob primitiveMeshPopulateJob = new FaceToPrimitiveMeshJob();

        NativeQueue<JobFace>.ParallelWriter lowerMesh = rawLowerMeshQueue.AsParallelWriter();
        NativeQueue<JobFace>.ParallelWriter upperMesh = rawUpperMeshQueue.AsParallelWriter();
        NativeQueue<CutHolePairing>.ParallelWriter generatedHoleFillingCollection = generatedHoleFilling.AsParallelWriter();

        NativeArray<Face> faces = Utils.ToNativeArray(meshfaces.ToArray(), Allocator.TempJob);

        primitiveMeshPopulateJob.lowerMesh = lowerMesh;
        primitiveMeshPopulateJob.upperMesh = upperMesh;
        primitiveMeshPopulateJob.holePairings = generatedHoleFillingCollection;

        primitiveMeshPopulateJob.faces = faces;

        Profiler.BeginSample("Native collection alloc");

        primitiveMeshPopulateJob.meshNormals = Utils.ToNativeArray(mesh.normals, Allocator.TempJob);
        primitiveMeshPopulateJob.meshVertices = Utils.ToNativeArray(mesh.vertices, Allocator.TempJob);
        primitiveMeshPopulateJob.meshUVs = Utils.ToNativeArray(mesh.uv, Allocator.TempJob);
        primitiveMeshPopulateJob.meshTriangles = Utils.ToNativeArray(mesh.triangles, Allocator.TempJob);

        Profiler.EndSample();

        primitiveMeshPopulateJob.position = position;
        primitiveMeshPopulateJob.normal = normal;

        primitiveMeshPopulateJob.transformedPosition = transformedPosition;
        primitiveMeshPopulateJob.transformedNormal = transformedNormal;

        primitiveMeshPopulateJob.worldMatrix = worldMatrix;
        primitiveMeshPopulateJob.preCutCentroid = preCutCentroid;
  
        JobHandle jobHandle = primitiveMeshPopulateJob.Schedule(meshfaces.Count, 80);
        jobHandle.Complete();


        primitiveMeshPopulateJob.meshNormals.Dispose();
        primitiveMeshPopulateJob.meshVertices.Dispose();
        primitiveMeshPopulateJob.meshUVs.Dispose();
        primitiveMeshPopulateJob.meshTriangles.Dispose();
       
        faces.Dispose();

        Profiler.EndSample();
    }

    private void populateMesh(NativeQueue<JobFace> jobFaceQueue,Mesh mesh)
    {
        Vector3[] vertices = new Vector3[jobFaceQueue.Count * 6];
        Vector3[] normals = new Vector3[jobFaceQueue.Count * 6];
        Vector2[] uvs = new Vector2[jobFaceQueue.Count * 6];
        int[] triangles;

        Profiler.BeginSample("jobFaceQueue.TryDequeue");


        int finalIndex = 0;

        while(jobFaceQueue.TryDequeue(out JobFace jbf))
        {
            jbf.jt1.PopulateArray(vertices, normals, uvs, ref finalIndex);

            if(jbf.jt2.isFilled)
            {
                jbf.jt2.PopulateArray(vertices, normals, uvs, ref finalIndex);
            }
        }


        triangles = new int[finalIndex];
        for (int i = 0; i < finalIndex; i++)
        {
            triangles[i] = i;
        }

        Profiler.EndSample();


        Profiler.BeginSample("meshClear");
        mesh.Clear();
        Profiler.EndSample();

        Profiler.BeginSample("fill in mesh");

        Vector3[] newVertices = new Vector3[finalIndex];
        Vector3[] newNormals = new Vector3[finalIndex];
        Vector2[] newUVs = new Vector2[finalIndex];

        Array.Copy(vertices, 0, newVertices, 0, finalIndex);
        Array.Copy(normals, 0, newNormals, 0, finalIndex);
        Array.Copy(uvs, 0, newUVs, 0, finalIndex);


        mesh.vertices = newVertices;
        mesh.normals = newNormals;
        mesh.uv = newUVs;
        mesh.triangles = triangles;


        Profiler.EndSample();

    }


    public CuttableMeshPhysicsManager InstantiateTreePiece(NativeQueue<JobFace> faceQueue, out GameObject newTree)
    {
        newTree = new GameObject();

        var meshRenderer = newTree.AddComponent<MeshRenderer>();
        meshRenderer.material = GetComponent<MeshRenderer>().sharedMaterial;

        var newMeshFilter = newTree.AddComponent<MeshFilter>();

        CuttableTreeScript cuttableTree = newTree.AddComponent<CuttableTreeScript>();
        cuttableTree.isFirstTree = false;


        newTree.transform.position = transform.position;
        newTree.transform.rotation = transform.rotation;
        newTree.transform.localScale = transform.localScale;

        populateMesh(faceQueue, newMeshFilter.mesh);

        CuttableMeshPhysicsManager cmpm = newTree.AddComponent<CuttableMeshPhysicsManager>();
        //newTree.AddComponent<TreeFallParticle>();

        cmpm.GenerateMeshColliderFromCut(newMeshFilter.mesh, true);

        cuttableTree.nativeArrayAllocator = nativeArrayAllocator;

        return cmpm;

    }



    //----------------------------------------- non-multithreaded code below--------------------------------------------------------------//
    //

    public GameObject CutAtNoOptimizations(Vector3 position, Vector3 normal, float seperationForce)
    {
        Debug.Log("Starting cut for " + gameObject.name);

        Matrix4x4 worldMatrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.localScale);
        preCutCentroid = worldMatrix.MultiplyPoint(preCutCentroid);

        //will be usefull later for a possible optimization
        Matrix4x4 inverseWorldMatrix = Matrix4x4.Inverse(worldMatrix);

        Vector3 transformedPosition = inverseWorldMatrix.MultiplyPoint3x4(position);
        Vector3 transformedNormal = (Matrix4x4.Transpose(worldMatrix)).MultiplyVector(normal);

        PrimitiveMesh FacesSplitAbove = new PrimitiveMesh();
        PrimitiveMesh FacesSplitBelow = new PrimitiveMesh();
        FacesSplitAbove.individualFaces = new List<IndividualFace>();
        FacesSplitBelow.individualFaces = new List<IndividualFace>();

        Profiler.BeginSample("[cut] Splitting The Mesh Into Primitives");

        foreach (TreeSplitCollisionBox collisionBox in collisionBoxes)
        {
            foreach (Face face in collisionBox.faces)
            {
                populatePrimitiveMeshes(FacesSplitBelow, FacesSplitAbove, face, transformedPosition, transformedNormal, worldMatrix, position, normal);
            }
        }

        Profiler.EndSample();

        List<Vector3> vertexPositions = new List<Vector3>();

        Vector3 centerPoint = Vector3.zero;

        //------- Get center of the intersection points created from the cut------------------------//
        foreach (MeshLidPairing lidPairing in lidPairings)
        {
            centerPoint += lidPairing.v0.position;
            centerPoint += lidPairing.v1.position;

            vertexPositions.Add(lidPairing.v0.position);
            vertexPositions.Add(lidPairing.v1.position);
        }

        centerPoint /= (lidPairings.Count * 2);

        IndividualVertex vertex = new IndividualVertex(centerPoint, Vector3.zero, Vector2.zero);

        List<IndividualTriangle> trianglesBelow = new List<IndividualTriangle>();


        //--------------------- Create triangles using the intersection points and the centerPoint------------------------//
        foreach (MeshLidPairing lidPairing in lidPairings)
        {
            IndividualTriangle tri = lidPairing.CreateTriangle(vertex);

            Vector3 ObjectSpaceCentroid = tri.GetObjectSpaceCentroid();
            Vector3 direction = Vector3.Cross(Vector3.up, ObjectSpaceCentroid - centerPoint).normalized;

            tri.AttemptDirectionOrderingBasedVertexCorrection(ObjectSpaceCentroid, direction);
            tri.SetNormals(normal);
            trianglesBelow.Add(tri);
            FacesSplitBelow.AddFaceFromSingularTriangle(tri);

            IndividualTriangle tri2 = tri.Clone();
            tri2.FlipTriangle(0, 2);
            tri2.SetNormals(-normal);
            FacesSplitAbove.AddFaceFromSingularTriangle(tri2);

        }

        //<<<<<<<<<<<<<<<<<<<<<<<<<<<


        //-------------------------- Reinitialize the original mesh ------------------------------------//

        FacesSplitBelow.PopulateMesh(mesh);
        InitializeCuttableTree();

        if (meshPhysicsManager)
        {
            bool rigidBodyNeeded = !isFirstTree;
            meshPhysicsManager.GenerateMeshColliderFromCut(mesh, rigidBodyNeeded);
        }

        //---------------------- Create a new mesh and assign in to the newly created mesh-------------------------------//

        GameObject newTree;
        var otherMeshPhysicsManager = InstantiateTreePiece(FacesSplitAbove, out newTree);

        float highestPoint = float.MinValue;
        Vector3 point = new Vector3(highestPoint, highestPoint, highestPoint);

        //find highest intersectionPoint
        foreach (MeshLidPairing lidPairing in lidPairings)
        {
            Vector3 worldV0 = worldMatrix.MultiplyPoint(lidPairing.v0.position);
            Vector3 worldV2 = worldMatrix.MultiplyPoint(lidPairing.v1.position);

            if (worldV0.y > highestPoint)
            {
                highestPoint = worldV0.y;
                point = worldV0;
            }

            if (worldV2.y > highestPoint)
            {
                highestPoint = worldV2.y;
                point = worldV2;
            }
        }

        //---------------------- Re assign the leaves to its correct parent--------------------------//

        //Profiler.BeginSample("[cut] Reinitialize new mesh");

        DisplaceLeaves(position, normal, gameObject, otherMeshPhysicsManager.gameObject);

        otherMeshPhysicsManager.AddForceAt(seperationForce * cutForceMultiplier, normal, point);

        return newTree;
    }

    private void populatePrimitiveMeshes(PrimitiveMesh lowerMesh,PrimitiveMesh upperMesh,Face face,Vector3 transformedPosition,Vector3 transformedNormal,Matrix4x4 worldMatrix,Vector3 position,Vector3 normal)
    {
        TriangleSplitState tri1CheckResult = TriangleSplitState.DefaultNoTriangle;
        TriangleSplitState tri2CheckResult = TriangleSplitState.DefaultNoTriangle;

        bool hasTriangle1 = face.tri1.v0 != -1;

        if (hasTriangle1)
        {
            tri1CheckResult = triangleToPlaneCheck(
                mesh.vertices[face.tri1.v0],
                mesh.vertices[face.tri1.v1],
                mesh.vertices[face.tri1.v2],
                transformedPosition, transformedNormal);

        }

        bool hasTriangle2 = face.tri2.v0 != -1;

        if (hasTriangle2)
        {
            tri2CheckResult = triangleToPlaneCheck(
                mesh.vertices[face.tri2.v0],
                mesh.vertices[face.tri2.v1],
                mesh.vertices[face.tri2.v2],
                transformedPosition, transformedNormal);
        }

        bool isBothTrianglesExist = hasTriangle1 && hasTriangle2;

        //if both triangles exist and have the same triangle split state
        if (isBothTrianglesExist && tri1CheckResult == tri2CheckResult)
        {
            organizeFaceBasedOnTriangleSplitState(worldMatrix, tri1CheckResult, face, position, normal,upperMesh,lowerMesh);
        }
        //if both triangles exist but one triangle is intersecting but the other is either above or below the splitting plane
        else if (isBothTrianglesExist && tri1CheckResult != tri2CheckResult)
        {

            Vector3 worldV0 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri1.v0]);
            Vector3 worldV1 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri1.v1]);
            Vector3 worldV2 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri1.v2]);

            Vector3 worldV3 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri2.v0]);
            Vector3 worldV4 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri2.v1]);
            Vector3 worldV5 = worldMatrix.MultiplyPoint(mesh.vertices[face.tri2.v2]);

            Vector3[] worldTrianglePointPositions = new Vector3[6];
            worldTrianglePointPositions[0] = worldV0;
            worldTrianglePointPositions[1] = worldV1;
            worldTrianglePointPositions[2] = worldV2;
            worldTrianglePointPositions[3] = worldV3;
            worldTrianglePointPositions[4] = worldV4;
            worldTrianglePointPositions[5] = worldV5;

            intersectingFaceSplit(worldMatrix, face, position, normal, worldTrianglePointPositions, lowerMesh, upperMesh);

        }
        //one of the triangles in the face do not exist ( this face only has one triangle)
        else if (!isBothTrianglesExist)
        {
            if (hasTriangle1)
            {
                FindDecisionForSingularTriangle(worldMatrix, tri1CheckResult, face.tri1, position, normal, lowerMesh, upperMesh);
            }
            if (hasTriangle2)
            {
                FindDecisionForSingularTriangle(worldMatrix, tri2CheckResult, face.tri2, position, normal, lowerMesh, upperMesh);
            }
        }
    }


   


    /// <summary>
    /// 
    /// </summary>
    /// <param name="state"></param>
    public void SetIsFirstTree(bool state)
    {
        isFirstTree = state;
    }

    public void DisplaceLeaves(Vector3 planePosition, Vector3 planeNormal, GameObject belowCuttingPlaneObj, GameObject aboveCuttingPlaneObj)
    {
        List<Transform> children = new List<Transform>();

        for (int i = 0; i < belowCuttingPlaneObj.gameObject.transform.childCount; i++)
        {
            var child = belowCuttingPlaneObj.gameObject.transform.GetChild(i);

            if(child != null)
            {
                children.Add(child);
            }
        }

        for (int i = 0; i < aboveCuttingPlaneObj.gameObject.transform.childCount; i++)
        {
            var child = aboveCuttingPlaneObj.gameObject.transform.GetChild(i);

            if (child != null)
            {
                children.Add(child);
            }
        }

        belowCuttingPlaneObj.transform.DetachChildren();
        aboveCuttingPlaneObj.transform.DetachChildren();

        for (int i = 0; i < children.Count; i++)
        {
            if(children[i] == null) { continue; }

            Vector3 position = Vector3.zero;
            if (children[i].tag == "Leaves")
            {
                position = children[i].transform.position;
            }
            else if(children[i].tag == "hole")
            {
                position = transform.localToWorldMatrix.MultiplyPoint(Utils.GetObjectSpaceMeshCentroid(
                    children[i].GetComponent<MeshFilter>().mesh));
                    //child.transform.position;
            }
            else
            {
                continue;
            }

            if (Utils.IsPointAbovePlane(position, planePosition,planeNormal))
            {

                children[i].SetParent(aboveCuttingPlaneObj.transform);
                
            }
            else
            {
                children[i].SetParent(belowCuttingPlaneObj.transform);
            }

            //
        }
    }

    public Mesh GetMesh()
    {
        return mesh;
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.C))
        {
            if (useMultithreadedVersion)
            {
                CutAt(DebugObjectTest.transform.position, DebugObjectTest.transform.up, 00.0f);
            }
            else
            {
                CutAtNoOptimizations(DebugObjectTest.transform.position, DebugObjectTest.transform.up, 00.0f);
            }
            
        }
    }

    private void ensureCentroidLeaf()
    {
        foreach (Transform child in transform)
        {
            if (child.tag == "leaf")
            {
                Utils.EnsurePositionIsCentroid(child);
            }
        }
    }

    public CuttableMeshPhysicsManager GetMeshColliderGenerator()
    {
        return GetComponent<CuttableMeshPhysicsManager>();
    }

    /// <summary>
    /// Instantiates a new gameObject with a meshFilter and uses the mesh data given from the given PrimitiveMesh
    /// </summary>
    /// <param name="primitiveMesh"> Contains the mesh data that will be used to populate the mesh</param>
    /// <param name="newTree"> An out parameter that will return the newly created GameObject </param>
    /// <returns>The Mesh Physics Manager owned by the newly created object </returns>
    public CuttableMeshPhysicsManager InstantiateTreePiece(PrimitiveMesh primitiveMesh, out GameObject newTree)
    {
        newTree = new GameObject();

        var meshRenderer = newTree.AddComponent<MeshRenderer>();
        meshRenderer.material = GetComponent<MeshRenderer>().sharedMaterial;

        var newMeshFilter = newTree.AddComponent<MeshFilter>();

        CuttableTreeScript cuttableTree = newTree.AddComponent<CuttableTreeScript>();
        cuttableTree.isFirstTree = false;


        newTree.transform.position = transform.position;
        newTree.transform.rotation = transform.rotation;
        newTree.transform.localScale = transform.localScale;

        primitiveMesh.PopulateMesh(newMeshFilter.mesh);

        CuttableMeshPhysicsManager cmpm  = newTree.AddComponent<CuttableMeshPhysicsManager>();
        //newTree.AddComponent<TreeFallParticle>();
        
        cmpm.GenerateMeshColliderFromCut(newMeshFilter.mesh, true);

        cuttableTree.nativeArrayAllocator = nativeArrayAllocator;
        
        return cmpm;

    }

    /// <summary>
    /// Checks if a triangle is above the splitting plane,below the splitting plane, or intersecting with it
    /// </summary>
    /// <param name="transformedV0"> The  world space of the first vertex of the triangle </param>
    /// <param name="transformedV1"> The  world space of the second vertex of the triangle</param>
    /// <param name="transformedV2"> The world space of the third vertex of the triangle</param>
    /// <param name="position"> the world space position of the cutting plane </param>
    /// <param name="normal"> the world space normal of the triangle </param>
    /// <returns></returns>
    private TriangleSplitState triangleToPlaneCheck(Vector3 transformedV0, Vector3 transformedV1,Vector3 transformedV2,Vector3 position,Vector3 normal)
    {
        bool v0AbovePlane = Utils.IsPointAbovePlane(transformedV0, position, normal);
        bool v1AbovePlane = Utils.IsPointAbovePlane(transformedV1, position, normal);
        bool v2AbovePlane = Utils.IsPointAbovePlane(transformedV2, position, normal);

        if (v0AbovePlane && v1AbovePlane && v2AbovePlane)
        {
            return TriangleSplitState.AbovePlane;
        }
        else if(!v0AbovePlane && !v1AbovePlane && !v2AbovePlane)
        {
            return TriangleSplitState.BelowPlane;
        }
        else
        {
            return TriangleSplitState.IntersectionOnPlane;
        }
        
    }

    /// <summary>
    /// Given a triangle split state, decides if the triangle would be displaced to the lowerPrimitiveMesh,upperPrimitiveMesh, or would need to be split into 2
    /// </summary>
    /// <param name="world"></param>
    /// <param name="state"></param>
    /// <param name="face"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="upperPrimitiveMesh"></param>
    /// <param name="lowerPrimitiveMesh"></param>
    private void organizeFaceBasedOnTriangleSplitState(Matrix4x4 world,TriangleSplitState state, Face face, Vector3 position, 
        Vector3 normal,PrimitiveMesh upperPrimitiveMesh,PrimitiveMesh lowerPrimitiveMesh)
    {


        switch (state)
        {
            case TriangleSplitState.AbovePlane:
                upperPrimitiveMesh.AddFaceFrom(this, face);

                break;

            case TriangleSplitState.BelowPlane:
                lowerPrimitiveMesh.AddFaceFrom(this, face);

                break;

            case TriangleSplitState.IntersectionOnPlane:

                Vector3[] worldTrianglePointPositions = new Vector3[6];
                worldTrianglePointPositions[0] = world.MultiplyPoint(mesh.vertices[face.tri1.v0]);
                worldTrianglePointPositions[1] = world.MultiplyPoint(mesh.vertices[face.tri1.v1]);
                worldTrianglePointPositions[2] = world.MultiplyPoint(mesh.vertices[face.tri1.v2]);
                worldTrianglePointPositions[3] = world.MultiplyPoint(mesh.vertices[face.tri2.v0]);
                worldTrianglePointPositions[4] = world.MultiplyPoint(mesh.vertices[face.tri2.v1]);
                worldTrianglePointPositions[5] = world.MultiplyPoint(mesh.vertices[face.tri2.v2]);

                intersectingFaceSplit(world, face, position, normal, worldTrianglePointPositions, lowerPrimitiveMesh, upperPrimitiveMesh);

                break;



        }
    }


    /// <summary>
    /// Given a face thats known to be intersecting with the cutting plane, creates triangles based on the intersection points of the face 
    /// with the plane
    /// </summary>
    /// <param name="world"> The world space matrix of the GameObject </param>
    /// <param name="face"> The indices of the intersecting face </param>
    /// <param name="position"> the world space Position Of the cutting plane </param>
    /// <param name="normal"> The world space normal of the cutting plane </param>
    /// <param name="worldTrianglePointPositions"> the world space position of the vertices of the triangles of the face </param>
    /// <param name="lowerPrimitiveMesh"> the Primitive Mesh that will store the vertices of the mesh below the cutting plane </param>
    /// <param name="upperPrimitiveMesh"> the Primitice Mesh that will store the vertices of the mesh above the cutting plane </param>
    private void intersectingFaceSplit(Matrix4x4 world,Face face,Vector3 position,Vector3 normal,Vector3[] worldTrianglePointPositions,PrimitiveMesh lowerPrimitiveMesh,PrimitiveMesh upperPrimitiveMesh )
    {
        List<Vector3> foundIntersectionPoint;
        UnOptimizedGetFaceToPlaneIntersectionPoints(world, face, position, normal, out foundIntersectionPoint);

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

        if(uniqueTrianglesBelowSplittingPlane.Count < 2 && uniqueTrianglesAboveSplittingPlane.Count < 2) { return; }
        if (uniqueIntersectionPoints.Count < 2) { return; }

        basePosition = preCutCentroid;
        baseDirection = Vector3.Cross(Vector3.up, (worldTrianglePointPositions[0] - preCutCentroid)).normalized;

        IntersectionComparer ic = new IntersectionComparer
            (baseDirection, basePosition,world);

        IndexDirectionComparer idc = new IndexDirectionComparer((uniqueIntersectionPoints[uniqueIntersectionPoints.Count-1]- uniqueIntersectionPoints[0]).normalized
            , basePosition, mesh.vertices, world);

        uniqueIntersectionPoints.Sort(ic);

        Vector3 belowTriangleCentroid = GetWorldTriangleCentroid(mesh, uniqueTrianglesBelowSplittingPlane, world);
        Vector3 aboveTriangleCentroid = GetWorldTriangleCentroid(mesh, uniqueTrianglesAboveSplittingPlane, world);

        idc.basePosition = uniqueIntersectionPoints[0];
        uniqueTrianglesBelowSplittingPlane.Sort(idc);

        idc.basePosition = uniqueIntersectionPoints[0];
        uniqueTrianglesAboveSplittingPlane.Sort(idc);


        List<Vector3> intersectionPoints = new List<Vector3>();

        intersectionPoints.Add(uniqueIntersectionPoints[0]);
        intersectionPoints.Add(uniqueIntersectionPoints[uniqueIntersectionPoints.Count-1]);


        //------------------------------- create bottom part----------------------------------------------------//

        Vector3 intersectionDirection = intersectionPoints[1] - intersectionPoints[0];

        Vector3 vertexDirection =
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesBelowSplittingPlane[uniqueTrianglesBelowSplittingPlane.Count - 1]]) -
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesBelowSplittingPlane[0]])
           ;

        if (Vector3.Dot(intersectionDirection, baseDirection) < 0)
        {
            intersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection, intersectionDirection) < 0)
        {
            uniqueTrianglesBelowSplittingPlane.Reverse();
        }

        // Debug.Log("------------below splitting plane vertex list");
        // DEBUG_logIndicesList(mesh, world, uniqueTrianglesBelowSplittingPlane);

        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesBelowSplittingPlane, false, world, lowerPrimitiveMesh);

        //------------------------------- create above part----------------------------------------------------//

        Vector3 intersectionDirection2 = intersectionPoints[1] - intersectionPoints[0];

        Vector3 vertexDirection2 =
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesAboveSplittingPlane[uniqueTrianglesAboveSplittingPlane.Count - 1]]) -
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesAboveSplittingPlane[0]]);

        if (Vector3.Dot(intersectionDirection2, baseDirection) < 0)
        {
            intersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection2, intersectionDirection2) < 0)
        {
            uniqueTrianglesAboveSplittingPlane.Reverse();
        }

        // Debug.Log("------------above splitting plane vertex list");
        // DEBUG_logIndicesList(mesh, world, uniqueTrianglesAboveSplittingPlane);


        assembleFacesFromSplitVertices(intersectionPoints, uniqueTrianglesAboveSplittingPlane, true, world, upperPrimitiveMesh);




        //store vertices restoring the hole that will be generated after cutting the mesh
        IndividualVertex v0 = new IndividualVertex(Matrix4x4.Inverse(world).MultiplyPoint(intersectionPoints[0]), Vector3.up, Vector2.zero);
        IndividualVertex v1 = new IndividualVertex(
            Matrix4x4.Inverse(world).MultiplyPoint(intersectionPoints[intersectionPoints.Count - 1]), 
            Vector3.up, Vector2.zero);
        MeshLidPairing lidPairing = new MeshLidPairing(v0, v1);

        lidPairings.Add(lidPairing);

    }

    /// <summary>
    ///  Creates faces from a list of sorted intersection points and vertex positions and adds them to the given PrimitiveMesh
    /// </summary>
    /// <param name="uniqueIntersectionPoints"> A list of sorted and unique intersection Points </param>
    /// <param name="trianglesInSplitPlane"></param>
    /// <param name="isIntersectionPointBottomLeftVertex"></param>
    /// <param name="world"></param>
    /// <param name="meshToPopulate"></param>
    private void assembleFacesFromSplitVertices(List<Vector3> uniqueIntersectionPoints,List<int> trianglesInSplitPlane,bool isIntersectionPointBottomLeftVertex, Matrix4x4 world,PrimitiveMesh meshToPopulate)
    {
        Matrix4x4 inverseWorld = Matrix4x4.Inverse(world);

        int iterationCount = uniqueIntersectionPoints.Count > trianglesInSplitPlane.Count ? uniqueIntersectionPoints.Count : trianglesInSplitPlane.Count;
        
        List<ConnectionTypeToCentroid> types =new List<ConnectionTypeToCentroid>();
        
        for (int i = 0; i < iterationCount-1; i++)
        {
            int currentItersectionPointI = GetCurrentIndex(uniqueIntersectionPoints.Count, i);
            Vector3 objectSpaceItersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(currentItersectionPointI));

            bool nextIntersectionPointExist = i + 1 < uniqueIntersectionPoints.Count;
            bool nextTrianglePointExist = i + 1 < trianglesInSplitPlane.Count;

            IndividualVertex bottomLeftVertex;
            IndividualVertex upperLeftVertex;

            if (nextTrianglePointExist && nextIntersectionPointExist)
            {
                IndividualVertex bottomRightVertex;
                IndividualVertex upperRightVertex;
 
                Vector3 nextObjectSpaceIntersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(i + 1));

                //all intersection points are bottom, triangles points are upper
                if (isIntersectionPointBottomLeftVertex)
                {
                    upperLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(i));
                    upperRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(i + 1));

                    bottomLeftVertex = new IndividualVertex(objectSpaceItersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));
                    bottomRightVertex = new IndividualVertex(nextObjectSpaceIntersectionPoint, upperRightVertex.normal, new Vector2(1, 1));

                    IndividualTriangle tri1 = new IndividualTriangle(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                    IndividualTriangle tri2 = new IndividualTriangle(bottomLeftVertex, upperRightVertex,bottomRightVertex);

                    if (tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2))
                    {

                    }

                    if (tri2.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2))
                    {

                    }

                    meshToPopulate.AddFaceFromTriangles(tri1, tri2);

                    //store its position and connection type
                    ConnectionTypeToCentroid tri1Type;
                    tri1Type.objectSpaceCentroid = tri1.GetObjectSpaceCentroid();
                    tri1Type.tct = TriangleConnectionType.DoubleOriginalPoint;

                    ConnectionTypeToCentroid tri2Type;
                    tri2Type.objectSpaceCentroid = tri2.GetObjectSpaceCentroid();
                    tri2Type.tct = TriangleConnectionType.DoubleIntersection;


                    types.Add(tri1Type);
                    types.Add(tri2Type);

                }
                //all intersection points are upper, triangles points are bottom
                else
                {
                    bottomLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(i));

                    bottomRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(i + 1));

                    upperLeftVertex = new IndividualVertex(objectSpaceItersectionPoint, bottomLeftVertex.normal, new Vector2(1, 1));

                    upperRightVertex = new IndividualVertex(nextObjectSpaceIntersectionPoint, bottomRightVertex.normal, new Vector2(1, 1));

                    IndividualTriangle tri1 = new IndividualTriangle(bottomLeftVertex, upperLeftVertex, upperRightVertex);
                    IndividualTriangle tri2 = new IndividualTriangle(bottomLeftVertex, upperRightVertex, bottomRightVertex);


                    if (tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2))
                    {
                        tri2.V1 = upperLeftVertex;
                    }

                    if (tri2.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2))
                    {
                        tri2.V1 = upperRightVertex;
                    }

                    meshToPopulate.AddFaceFromTriangles(tri1, tri2);
                    
                    ConnectionTypeToCentroid tri1Type;
                    tri1Type.objectSpaceCentroid = tri1.GetObjectSpaceCentroid();
                    tri1Type.tct = TriangleConnectionType.DoubleIntersection;

                    ConnectionTypeToCentroid tri2Type;
                    tri2Type.objectSpaceCentroid = tri2.GetObjectSpaceCentroid();
                    tri2Type.tct = TriangleConnectionType.DoubleOriginalPoint;

                    types.Add(tri1Type);
                    types.Add(tri2Type);

                }


            }
            else
            {
                IndividualVertex bottomRightVertex;
                IndividualVertex upperRightVertex;

                int currentTriangleIndex = GetCurrentIndex(trianglesInSplitPlane.Count, i);

                if (nextIntersectionPointExist)
                {
                    Vector3 nextObjectSpaceIntersectionPoint = inverseWorld.MultiplyPoint(uniqueIntersectionPoints.ElementAt(i + 1));

                    //all intersection points are bottom, triangles points are upper
                    if (isIntersectionPointBottomLeftVertex)
                    {
                        upperLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex));

                        bottomLeftVertex = new IndividualVertex();
                        bottomLeftVertex.position = objectSpaceItersectionPoint;
                        bottomLeftVertex.normal = upperLeftVertex.normal;
                        bottomLeftVertex.UV = new Vector2(1, 1);

                        bottomRightVertex = new IndividualVertex();
                        bottomRightVertex.position = nextObjectSpaceIntersectionPoint;
                        bottomRightVertex.normal = upperLeftVertex.normal;
                        bottomRightVertex.UV = new Vector2(1, 1);

                        IndividualTriangle tri1 = new IndividualTriangle(upperLeftVertex, bottomRightVertex, bottomLeftVertex);

                        tri1.AttemptNormalBasedVertexCorrection(upperLeftVertex.normal, 1, 2);

                        meshToPopulate.AddFaceFromSingularTriangle(tri1);

                    }
                    //all intersection points are upper, triangles points are bottom
                    else
                    {
                        bottomLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex));

                        upperLeftVertex = new IndividualVertex();
                        upperLeftVertex.position = objectSpaceItersectionPoint;
                        upperLeftVertex.normal = bottomLeftVertex.normal;
                        upperLeftVertex.UV = new Vector2(1, 1);

                        upperRightVertex = new IndividualVertex();
                        upperRightVertex.position = nextObjectSpaceIntersectionPoint;
                        upperRightVertex.normal = upperLeftVertex.normal;
                        upperRightVertex.UV = new Vector2(1, 1);

                        IndividualTriangle tri1 = new IndividualTriangle(bottomLeftVertex, upperLeftVertex, upperRightVertex);

                        tri1.AttemptNormalBasedVertexCorrection(upperLeftVertex.normal, 1, 2);

                        meshToPopulate.AddFaceFromSingularTriangle(tri1);
                    }
                }
                //
                if (nextTrianglePointExist)
                {
                    //all intersection points are bottom, triangles points are upper
                    if (isIntersectionPointBottomLeftVertex)
                    {

                        bottomLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex));
                        bottomRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex + 1));

                        TriangleConnectionType tct = GetClosestConnectionTypeByCentroidProjection(uniqueIntersectionPoints[0], 
                            (uniqueIntersectionPoints[1] - uniqueIntersectionPoints[0]).normalized, types);


                        upperRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex - 1));

                        Debug.Log("found tct " + tct.ToString());//tct.ToString()
                       
                        if (tct == TriangleConnectionType.DoubleOriginalPoint)
                        {
                            //connects to intersection
                            upperRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex - 1));
                        }
                        else if(tct == TriangleConnectionType.DoubleIntersection)
                        {
                            //connects to previous triangle
                            upperRightVertex = new IndividualVertex(objectSpaceItersectionPoint, bottomLeftVertex.normal, new Vector2(1, 1));

                        }


                        IndividualTriangle tri1 = new IndividualTriangle(bottomLeftVertex, upperRightVertex, bottomRightVertex);


                        tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 0, 2);

                        meshToPopulate.AddFaceFromSingularTriangle(tri1);

                    }
                    //all intersection points are upper, triangles points are bottom
                    else
                    {

                        upperLeftVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex));
                        upperRightVertex = new IndividualVertex(mesh, trianglesInSplitPlane.ElementAt(currentTriangleIndex+1));

                        bottomLeftVertex = new IndividualVertex(objectSpaceItersectionPoint, upperLeftVertex.normal, new Vector2(1, 1));

                        IndividualTriangle tri1 = new IndividualTriangle(bottomLeftVertex, upperLeftVertex, upperRightVertex);


                        tri1.AttemptNormalBasedVertexCorrection(bottomLeftVertex.normal, 1, 2);
                        meshToPopulate.AddFaceFromSingularTriangle(tri1);

                    }
                }

            }
        }
    }

    private void FindDecisionForSingularTriangle(Matrix4x4 world, TriangleSplitState state, Triangle tri, Vector3 position, Vector3 normal,PrimitiveMesh lowerMesh,PrimitiveMesh upperMesh)
    {
        Vector3 worldV0 = world.MultiplyPoint(mesh.vertices[tri.v0]);
        Vector3 worldV1 = world.MultiplyPoint(mesh.vertices[tri.v1]);
        Vector3 worldV2 = world.MultiplyPoint(mesh.vertices[tri.v2]);

        switch(state)
        {
            case TriangleSplitState.AbovePlane:

                upperMesh.AddTriangleFrom(this, tri);

                break;

            case TriangleSplitState.BelowPlane:

                lowerMesh.AddTriangleFrom(this, tri);

                break;

            case TriangleSplitState.IntersectionOnPlane:

                //List<Vector3> triangleIntersectionPoints = UnOptimizedFindTriangleToPlaneIntersectionPoint(worldV0, worldV1, worldV2, position, normal);

                //foreach (var intersectionPoint in triangleIntersectionPoints)
                //{
                //    DEBUG_intersectionVertices.Add(intersectionPoint);
                //}

                Vector3[] worldTrianglePointPositions = new Vector3[3];
                worldTrianglePointPositions[0] = worldV0;
                worldTrianglePointPositions[1] = worldV1;
                worldTrianglePointPositions[2] = worldV2;

                intersectingTriangleSplit(world, tri, position, normal, worldTrianglePointPositions , lowerMesh, upperMesh);

                break;
        }
    }

    private void intersectingTriangleSplit(Matrix4x4 world, Triangle tri, Vector3 position, Vector3 normal, Vector3[] worldTrianglePointPositions, PrimitiveMesh lowerPrimitiveMesh, PrimitiveMesh upperPrimitiveMesh)
    {
        //find unique intersection points
        List<Vector3> uniqueIntersectionPoints = UnOptimizedFindTriangleToPlaneIntersectionPoint
            (worldTrianglePointPositions[0], worldTrianglePointPositions[1], worldTrianglePointPositions[2], position, normal);


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


        //sort unique intersection points based on the vector parallel to the vector resulting from the cross product of the world up (0,1,0) and one of the vertices of the intersection points

        Vector3 basePosition = preCutCentroid;
        Vector3 baseDirection = Vector3.Cross(Vector3.up, (worldTrianglePointPositions[0] - preCutCentroid)).normalized;

        IntersectionComparer ic = new IntersectionComparer(baseDirection, basePosition, world);
        uniqueIntersectionPoints.Sort(ic);

        //sort the points above and below the splitting plane based on the vector created by the last element of the intersectionPoint Collection and the first element in the Vector3 Collection

        Vector3 indexbaseDirection = (uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1] - uniqueIntersectionPoints[0]).normalized;

        IndexDirectionComparer idc = new IndexDirectionComparer(indexbaseDirection
            , basePosition, mesh.vertices, world);

        uniqueTrianglesAboveSplittingPlane.Sort(idc);


        //check one more time, check if points and intersection points are ordered in the right direction

        Vector3 intersectionDirection = world.MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]) - world.MultiplyPoint(uniqueIntersectionPoints[0]);

        Vector3 vertexDirection =
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesBelowSplittingPlane[uniqueTrianglesBelowSplittingPlane.Count - 1]]) -
             world.MultiplyPoint(mesh.vertices[uniqueTrianglesBelowSplittingPlane[0]])
           ;

        if (Vector3.Dot(intersectionDirection, baseDirection) < 0)
        {
            uniqueIntersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection, intersectionDirection) < 0)
        {
            uniqueTrianglesBelowSplittingPlane.Reverse();
        }


        assembleFacesFromSplitVertices(uniqueIntersectionPoints, uniqueTrianglesBelowSplittingPlane, false, world, lowerPrimitiveMesh);

        //use the points above the spllitting plane and the intersection points to create the triangle that will be placed in the upperPrimitiveMesh
        Vector3 intersectionDirection2 = world.MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]) - world.MultiplyPoint(uniqueIntersectionPoints[0]);

        Vector3 vertexDirection2 =
           world.MultiplyPoint(mesh.vertices[uniqueTrianglesAboveSplittingPlane[uniqueTrianglesAboveSplittingPlane.Count - 1]]) -
           world.MultiplyPoint(mesh.vertices[uniqueTrianglesAboveSplittingPlane[0]]);

        if (Vector3.Dot(intersectionDirection2, baseDirection) < 0)
        {
            uniqueIntersectionPoints.Reverse();
        }

        if (Vector3.Dot(vertexDirection2, intersectionDirection2) < 0)
        {
            uniqueTrianglesAboveSplittingPlane.Reverse();
        }

        assembleFacesFromSplitVertices(uniqueIntersectionPoints, uniqueTrianglesAboveSplittingPlane, true, world, upperPrimitiveMesh);

        //use the points below the splitting plane and the intersection points to create the triangle that will be placed in the lowerPrimitiveMesh


        IndividualVertex v0 = new IndividualVertex(Matrix4x4.Inverse(world).MultiplyPoint(uniqueIntersectionPoints[0]), Vector3.up, Vector2.zero);
        IndividualVertex v1 = new IndividualVertex(
            Matrix4x4.Inverse(world).MultiplyPoint(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]),
            Vector3.up, Vector2.zero);
        MeshLidPairing lidPairing = new MeshLidPairing(v0, v1);

        lidPairings.Add(lidPairing);

        //DEBUG_intersectionVertices.Add(uniqueIntersectionPoints[0]);
        //DEBUG_intersectionVertices.Add(uniqueIntersectionPoints[uniqueIntersectionPoints.Count - 1]);




    }

    //assumes that face contains 2 triangles
    private void UnOptimizedGetFaceToPlaneIntersectionPoints(Matrix4x4 world, Face face, Vector3 position,Vector3 normal, out List<Vector3> intersectionPoints)
    {
        intersectionPoints = new List<Vector3>();

        Vector3 worldV0 = world.MultiplyPoint(mesh.vertices[face.tri1.v0]);
        Vector3 worldV1 = world.MultiplyPoint(mesh.vertices[face.tri1.v1]);
        Vector3 worldV2 = world.MultiplyPoint(mesh.vertices[face.tri1.v2]);

        Vector3 worldV3 = world.MultiplyPoint(mesh.vertices[face.tri2.v0]);
        Vector3 worldV4 = world.MultiplyPoint(mesh.vertices[face.tri2.v1]);
        Vector3 worldV5 = world.MultiplyPoint(mesh.vertices[face.tri2.v2]);

        List<Vector3> triangleIntersectionPoints = UnOptimizedFindTriangleToPlaneIntersectionPoint
            (worldV0, worldV1, worldV2, position, normal);

        List<Vector3> secondTriangleIntersectionPoints = UnOptimizedFindTriangleToPlaneIntersectionPoint
            (worldV3, worldV4, worldV5, position, normal);

   
        foreach(var intersectionPoint in triangleIntersectionPoints)
        {
            
            intersectionPoints.Add(intersectionPoint);
        }

        foreach (var intersectionPoint in secondTriangleIntersectionPoints)
        {
            intersectionPoints.Add(intersectionPoint);
            
        }

    }

    private void bruteForceCollisionBoxInitialize()
    {
        Profiler.BeginSample("[cut] bruteForceCollisionBoxInitialize");

        collisionBoxes.Clear();

        TreeSplitCollisionBox tscb = new TreeSplitCollisionBox();
        tscb.faces = new List<Face>();
        collisionBoxes.Add(tscb);

        for (int i = 0; i < mesh.triangles.Length; i += 6)
        {

            if (i + 5 > mesh.triangles.Length - 1) { break; }

            int v0 = mesh.triangles[i];
            int v1 = mesh.triangles[i + 1];
            int v2 = mesh.triangles[i + 2];

            int v3 = mesh.triangles[i + 3];
            int v4 = mesh.triangles[i + 4];
            int v5 = mesh.triangles[i + 5];

            Triangle tri1 = new Triangle();
            tri1.v0 = v0;
            tri1.v1 = v1;
            tri1.v2 = v2;

            Triangle tri2 = new Triangle();
            tri2.v0 = v3;
            tri2.v1 = v4;
            tri2.v2 = v5;

            Vector3 tri1Normal = mesh.normals[v0];
            Vector3 tri2Normal = mesh.normals[v3];

            if ((tri1Normal - tri2Normal).magnitude < 0.01f)
            {
                Face face = new Face();
                face.Init();

                face.tri1 = tri1;
                face.tri2 = tri2;
                tscb.faces.Add(face);
            }
            else
            {
                Face face1 = new Face();
                face1.Init();
                Face face2 = new Face();
                face2.Init();
                face1.tri1 = tri1;
                face2.tri2 = tri2;

                tscb.faces.Add(face1);
                tscb.faces.Add(face2);
            }
        }

            Profiler.EndSample();
    }

    private void multithreadedBruteForceCollisionBoxInitialize()
    {
        Profiler.BeginSample("[cut] multithreadedBruteForceCollisionBoxInitialize");

        collisionBoxes.Clear();

        TreeSplitCollisionBox tscb = new TreeSplitCollisionBox();
        tscb.faces = new List<Face>();
        collisionBoxes.Add(tscb);
        
        int writeCount = mesh.triangles.Length%6 == 0 ?
            mesh.triangles.Length / 6 : 
            (mesh.triangles.Length / 6) + 1;

        NativeArray<FacePairing> faces = new NativeArray<FacePairing>((mesh.triangles.Length / 6)+1, Allocator.TempJob);

        Profiler.BeginSample("[cut] Array setup");

        NativeArray<int> meshTriangles;
        NativeArray<Vector3> meshNormals;
        meshTriangles = Utils.ToNativeArray(mesh.triangles, Allocator.TempJob);
        meshNormals = Utils.ToNativeArray(mesh.normals, Allocator.TempJob);

        Profiler.EndSample();

        TriangleToFaceJob triangleToFace = new TriangleToFaceJob();
        triangleToFace.faces = faces;
        triangleToFace.normals = meshNormals;
        triangleToFace.triangles = meshTriangles;

        JobHandle jobHandle = triangleToFace.Schedule(writeCount, 50);

        jobHandle.Complete();

        foreach (FacePairing pairing in faces)
        {
            tscb.faces.Add(pairing.f1);

            if (pairing.f2.isFilled())
            {
                tscb.faces.Add(pairing.f2);
            }

        }

        faces.Dispose();
        meshTriangles.Dispose();
        meshNormals.Dispose();


        Profiler.EndSample();
    }

    //---------------------------------- Helper Functions----------------------------------------//

    public static List<Vector3> UnOptimizedFindTriangleToPlaneIntersectionPoint(Vector3 transformedV0, Vector3 transformedV1, Vector3 transformedV2, Vector3 position, Vector3 normal)
    {
        List<Vector3> result = new List<Vector3>();

        Vector3 intersection1;
        if (UnOptimizedFindLineToPlaneIntersection(transformedV0, transformedV1, position, normal, out intersection1) 
            //|| UnOptimizedFindLineToPlaneIntersection(transformedV1, transformedV0, position, normal, out intersection1)
            )
        {
            result.Add(intersection1);
        }

        Vector3 intersection2;
        if (UnOptimizedFindLineToPlaneIntersection(transformedV1, transformedV2, position, normal, out intersection2) 
            //|| UnOptimizedFindLineToPlaneIntersection(transformedV2, transformedV1, position, normal, out intersection2)
            )
        {
            result.Add(intersection2);
        }

        Vector3 intersection3;
        if (UnOptimizedFindLineToPlaneIntersection(transformedV2, transformedV0, position, normal, out intersection3) 
            //|| UnOptimizedFindLineToPlaneIntersection(transformedV0, transformedV2, position, normal, out intersection3)
            )
        {
            result.Add(intersection3);
        }

        return result;
    }

    public static bool UnOptimizedFindLineToPlaneIntersection(Vector3 transformedV0, Vector3 transformedV1, Vector3 position, Vector3 normal, out Vector3 intersection)
    {
        bool isOnSameSideOfPlane = Utils.IsPointAbovePlane(transformedV0, position, normal) ^ Utils.IsPointAbovePlane(transformedV1, position, normal);
        
        if(!isOnSameSideOfPlane)
        {
            intersection = Vector3.zero;
            return false;
        }

        Vector3 lineToUse = transformedV1 - transformedV0;

        Vector3 P0 = transformedV0;
        Vector3 P1 = lineToUse.normalized;
        Vector3 A = position;

        float t = (Vector3.Dot(A, normal) - Vector3.Dot(P0, normal)) / Vector3.Dot(P1, normal);
        
        intersection = P0 + P1 * t;

        return true;

    }

    public static int GetCurrentIndex(int maxIndex, int requestedIndex)
    {
        //Mat
        return requestedIndex > maxIndex - 1 ? maxIndex - 1 : requestedIndex; 
    }

    private List<int> GetUniqueVertices(List<int> nonUniqueIndiceList)
    {
        List<int> uniqueVertices = new List<int>();
        List<Vector3> seenVertices = new List<Vector3>();


        for (int i = 0; i < nonUniqueIndiceList.Count; i++)
        {
            bool hasSeenVertex = false;
            Vector3 vertexToTest = mesh.vertices[nonUniqueIndiceList[i]];

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
                seenVertices.Add(mesh.vertices[nonUniqueIndiceList[i]]);
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

    /// <summary>
    /// Given the indices of a triangle, find its worldSpaceCentroid
    /// </summary>
    /// <param name="mesh"></param>
    /// <param name="indices"></param>
    /// <param name="world"></param>
    /// <returns></returns>
    private Vector3 GetWorldTriangleCentroid(Mesh mesh, List<int> indices,Matrix4x4 world)
    {
        Vector3 result = Vector3.zero;
        foreach(var x in indices)
        {
            result += mesh.vertices[x];
        }

        result /= indices.Count;


        return world.MultiplyPoint(result);
    }

    //base position and desiredDirection is expressed in world space
    public TriangleConnectionType GetClosestConnectionTypeByCentroidProjection(Vector3 basePosition,Vector3 desiredDirection, List<ConnectionTypeToCentroid> cttc)
    {
        TriangleConnectionType tct = TriangleConnectionType.DefaultNoType;
        float closestConnectionLength = float.MinValue;

        Matrix4x4 world = Matrix4x4.TRS(transform.position, transform.rotation, transform.localScale);



        foreach (ConnectionTypeToCentroid singleTypeToCentroid in cttc)
        {
            Vector3 worldSpaceCentroid = world.MultiplyPoint(singleTypeToCentroid.objectSpaceCentroid);
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

    /// <summary>
    /// Given an array of Vector3, checks if the given points are above a given infinite splitting plane
    /// </summary>
    /// <param name="worldTrianglePoints"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <returns></returns>
    private bool[] CheckIfPointsAreAbovePlane(Vector3[] worldTrianglePoints, Vector3 position, Vector3 normal)
    {
        bool[] result = new bool[worldTrianglePoints.Length];

        for (int i = 0; i < worldTrianglePoints.Length; i++)
        {
            result[i] = Utils.IsPointAbovePlane(worldTrianglePoints[i], position, normal);
        }

        return result;
    }


    //------------------------ Debugging related functions --------------------------//



    private void OnDrawGizmos()
    {
        //initialization testing
        if(collisionBoxes != null)
        {
            DEBUG_drawMeshInitialize();
        }


        Gizmos.color = Color.red;



    }

    private void DEBUG_drawMeshInitialize()
    {
        List<Color> colorList = new List<Color>();
        colorList.Add(Color.red);
        colorList.Add(Color.blue);
        colorList.Add(Color.green);
        colorList.Add(Color.yellow);
        colorList.Add(Color.gray);

        if(debugShowIndex > collisionBoxes.Count) { return; }

        for (int k = 0; k < debugShowIndex; k++)
        {
            Gizmos.color = colorList[k];

            for (int i = 0; i < collisionBoxes[k].faces.Count; i++)
            {
                DEBUG_drawCollisionBox(collisionBoxes[k]);
            }
        }
    }

    private void DEBUG_drawCollisionBox(TreeSplitCollisionBox box)
    {
        for(int i = 0; i < box.faces.Count; i++)
        {
            Face face = box.faces[i];

            if (face.tri1.v0 != -1)
            {
                DEBUG_drawTrianglePoints(face.tri1);

            }
            if (face.tri2.v1 != -1)
            {
                DEBUG_drawTrianglePoints(face.tri2);
            }
        }
    }

    private void DEBUG_drawTrianglePoints(Triangle tri)
    {
        Matrix4x4 worldM = Matrix4x4.TRS(transform.position, transform.rotation, transform.localScale);

        Vector3 pos1 = worldM.MultiplyPoint3x4(mesh.vertices[tri.v0]);
        Vector3 pos2 = worldM.MultiplyPoint3x4(mesh.vertices[tri.v1]);
        Vector3 pos3 = worldM.MultiplyPoint3x4(mesh.vertices[tri.v2]);

        Gizmos.DrawSphere(pos1, debugSphereSize);
        Gizmos.DrawSphere(pos2, debugSphereSize);
        Gizmos.DrawSphere(pos3, debugSphereSize);
    }

    private void DEBUG_logIndicesList(Mesh mesh,Matrix4x4 world,List<int> indices)
    {
        for(int i =0; i < indices.Count;i++)
        {
            Debug.Log("vert " + i + "is " + world.MultiplyPoint(mesh.vertices[indices[i]]) );
        }

    }

    public static void DEBUG_logVertices(List<Vector3> vectors)
    {
        for (int i = 0; i < vectors.Count; i++)
        {
            Debug.Log("vert " + i + "is " + vectors[i]);
        }
    }

}

