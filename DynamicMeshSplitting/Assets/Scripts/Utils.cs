using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;

public class Utils : MonoBehaviour
{
   

    static public T SelectRandomObjectFromCollection<T, TCollection>(TCollection Collection,out int selectIndex) where TCollection : ICollection<T>
    {
        T[] collectionArray = new T[Collection.Count];

        Collection.CopyTo(collectionArray, 0);

        int selectedObject = Random.Range(0, collectionArray.Length);

        selectIndex = selectedObject;

        return collectionArray[selectedObject];

    }

    static public void GetMinMaxOfVertices<T>(out Vector3 min, out Vector3 max, T collection) where T : ICollection<Vector3>
    {
        Vector3 minResult = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 maxResult = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        foreach (Vector3 vertex in collection)
        {
            //check for minResult
            if (vertex.x < minResult.x)
            {
                minResult.x = vertex.x;
            }

            if (vertex.y < minResult.y)
            {
                minResult.y = vertex.y;
            }

            if (vertex.z < minResult.z)
            {
                minResult.z = vertex.z;
            }



            //check for maxResult
            if (vertex.x > maxResult.x)
            {
                maxResult.x = vertex.x;
            }

            if (vertex.y > maxResult.y)
            {
                maxResult.y = vertex.y;
            }

            if (vertex.z > maxResult.z)
            {
                maxResult.z = vertex.z;
            }

        }

        min = minResult;
        max = maxResult;
    }

    static public void CustomLineToPlaneIntersection(Vector3 start, Vector3 end, Vector3 planePosition, Vector3 planeNormal, out Vector3 intersection,out float t)
    {
        Vector3 lineToUse = start - end;

        Vector3 P0 = start;
        Vector3 P1 = lineToUse.normalized;
        Vector3 A = planePosition;

        t = (Vector3.Dot(A, planeNormal) - Vector3.Dot(P0, planeNormal)) / Vector3.Dot(P1, planeNormal);

        intersection = P0 + P1 * t;


    }

    static public bool IsPointAbovePlane(Vector3 pointPosition, Vector3 planePosition, Vector3 normal)
    {
        return Vector3.Dot(pointPosition - planePosition, normal) > 0;
    }

    static public bool IsPointAbovePlane(Vector3 pointPosition, Vector3 planePosition, Vector3 normal, out PointSplitState state)
    {
        bool isPointAbovePlane = Utils.IsPointAbovePlane(pointPosition, planePosition, normal);

        if (isPointAbovePlane)
        {
            state = PointSplitState.AbovePlane;
        }
        else
        {
            state = PointSplitState.BelowPlane;
        }

        return isPointAbovePlane;

    }

    static public void EnsurePositionIsCentroid(Transform obj)
    {
        //get mesh of objects
        

        MeshFilter meshFilter = obj.GetComponent<MeshFilter>();

        if(meshFilter)
        {
            Mesh mesh = meshFilter.mesh;

            if(mesh)
            {
                Vector3 centroid = new Vector3();

                for (int i = 0; i < mesh.triangles.Length; i++)
                {
                    centroid += mesh.vertices[mesh.triangles[i]];
                }

                centroid /= mesh.triangles.Length;

                Vector3 worldSpaceCentroid = obj.localToWorldMatrix.MultiplyPoint(centroid);
                Vector3 oldPosition = obj.transform.position;

                obj.transform.position = worldSpaceCentroid;


                Vector3 centroidShiftDirection = obj.worldToLocalMatrix.MultiplyVector( oldPosition - worldSpaceCentroid);

                Vector3[] vertices = mesh.vertices;


                for (int i = 0; i < mesh.vertices.Length; i++)
                {
                    vertices[i] += centroidShiftDirection;
                }

                mesh.vertices = vertices;

                mesh.MarkModified(); 
            }
        }
        else
        {
            Debug.LogError("Utils.EnsurePositionIsCentroid failed because a mesh filter of the mesh itself is missing!");
        }


    }

    static public void EnsureCentroidIsPosition(Transform obj)
    {
        MeshFilter meshFilter = obj.GetComponent<MeshFilter>();

        if (meshFilter)
        {
            Mesh mesh = meshFilter.mesh;

            if (mesh)
            {
                Vector3 worldSpaceCentroid = obj.localToWorldMatrix.MultiplyPoint(GetObjectSpaceMeshCentroid( mesh));

                Vector3 oldPosition = obj.transform.position;
                obj.transform.position = worldSpaceCentroid;


                Vector3 shiftDirection = oldPosition - worldSpaceCentroid;
                Vector3[] vertices = mesh.vertices;

                for (int i = 0; i < mesh.vertices.Length; i++)
                {
                    vertices[i] -= shiftDirection;
                }

                mesh.vertices = vertices;

                mesh.MarkModified();


            }
        }

        
    }

    public static Vector3 GetObjectSpaceMeshCentroid(Mesh mesh)
    {
        Vector3 centroid = new Vector3();

        foreach(Vector3 position in mesh.vertices)
        {
            centroid += position;
        }

        centroid /= mesh.triangles.Length;
        return centroid;
    }

    //TODO :I have no idea how this actually works, and I will find out later
    public static unsafe NativeArray<T> ToNativeArray<T>(T[] TArray,Allocator allocator = Allocator.Persistent) where T : unmanaged
    {
        NativeArray<T> verts = new NativeArray<T>(TArray.Length, allocator,
            NativeArrayOptions.UninitializedMemory);
        #if UNITY_WEBGL

            for (int i = 0; i < TArray.Length; i++)
            {
                verts[i] = TArray[i];
            }

            return verts;
        #else
        // create a destination NativeArray to hold the vertices


        // pin the mesh's vertex buffer in place...
        fixed (void* vertexBufferPointer = TArray)
        {
            // ...and use memcpy to copy the Vector3[] into a NativeArray<floar3> without casting. whould be fast!
            UnsafeUtility.MemCpy(NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(verts),
                vertexBufferPointer, TArray.Length * (long)UnsafeUtility.SizeOf<T>());
        }
        // we only hve to fix the .net array in place, the NativeArray is allocated in the C++ side of the engine and
        // wont move arround unexpectedly. We have a pointer to it not a reference! thats basically what fixed does,
        // we create a scope where its 'safe' to get a pointer and directly manipulate the array

        return verts;
        #endif
    }

    

}
