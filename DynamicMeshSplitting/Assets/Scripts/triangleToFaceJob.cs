using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;

public struct FacePairing
{
    public Face f1;
    public Face f2;


}



struct TriangleToFaceJob : IJobParallelFor
{
    [WriteOnly]
    public NativeArray<FacePairing> faces;
    [ReadOnly]
    public NativeArray<int> triangles;
    [ReadOnly]
    public NativeArray<Vector3> normals;

    public void Execute(int i)
    {
        int index = i * 6;

        bool canAddOneMoreTriangle = index + 2 < triangles.Length - 1;
        bool canAddOneMoreQuad = index + 5 < triangles.Length - 1;

        FacePairing pairing;

        Face face1 = new Face();
        Face face2 = new Face();
        face1.Init();
        face2.Init();

        Triangle tri1 = new Triangle();
        Triangle tri2 = new Triangle();


        if (!canAddOneMoreQuad && canAddOneMoreTriangle)
        {
            tri1.v0 = triangles[index];
            tri1.v1 = triangles[index + 1];
            tri1.v2 = triangles[index + 2];

            face1.tri1 = tri1;
            face2.MarkUnfilled();

            pairing.f1 = face1;
            pairing.f2 = face2;

            faces[i] = pairing;


            return;
        }
        else if (!canAddOneMoreQuad)
        {
            return;
        }

        int v0 = triangles[index];
        int v1 = triangles[index + 1];
        int v2 = triangles[index + 2];

        int v3 = triangles[index + 3];
        int v4 = triangles[index + 4];
        int v5 = triangles[index + 5];

        
        tri1.v0 = v0;
        tri1.v1 = v1;
        tri1.v2 = v2;

        
        tri2.v0 = v3;
        tri2.v1 = v4;
        tri2.v2 = v5;

        Vector3 tri1Normal = normals[v0];
        Vector3 tri2Normal = normals[v3];

        

       


        ////tri1 and tri2 
        //if ((tri1Normal - tri2Normal).magnitude < 0.001f)
        //{
        //    face1.tri1 = tri1;
        //    face1.tri2 = tri2;
        //    face2.MarkUnfilled();
        //}
        ////both triangles need to be in 2 different faces
        //else
        //{
        //    face1.tri1 = tri1;
        //    face2.tri1 = tri2;
        //}

        face1.tri1 = tri1;
        face2.tri1 = tri2;

        pairing.f1 = face1;
        pairing.f2 = face2;

        faces[i] = pairing;
    }
}




