using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IndividualVertex
{
    public Vector3 position;
    public Vector3 normal;
    public Vector2 UV;

    public IndividualVertex()
    {

    }

    public IndividualVertex(Vector3 position,Vector3 normal,Vector2 uv)
    {
        this.position = position;
        this.normal = normal;
        UV = uv;
    }

    public IndividualVertex(Mesh mesh, int i)
    {
        position = mesh.vertices[i];
        normal = mesh.normals[i];
        UV = mesh.uv[i];
    }

    public void SetNormal(Vector3 normal)
    {
        this.normal = normal;
    }
}


public class IndividualTriangle
{
    //vertices are stored in object space
    public IndividualVertex V0;
    public IndividualVertex V1;
    public IndividualVertex V2;

    public IndividualTriangle()
    {

    }

    public IndividualTriangle(IndividualVertex v0, IndividualVertex v1, IndividualVertex v2)
    {
        V0 = v0;
        V1 = v1;
        V2 = v2;
    }

    public void RecalculateNormals()
    {
        Vector3 normal = Vector3.Cross(V1.position - V0.position, V2.position - V0.position).normalized;

        V0.normal = normal;
        V1.normal = normal;
        V2.normal = normal;
    }

    public bool AttemptNormalBasedVertexCorrection(Vector3 actualNormal, int flipIndexA, int flipIndexB)
    {
        if (Vector3.Dot(actualNormal, Vector3.Cross(V1.position - V0.position, V2.position - V0.position)) < 0)
        {
            FlipTriangle(flipIndexA, flipIndexB);
            return true;
        }
        return false;

    }

    public void AttemptDirectionOrderingBasedVertexCorrection(Vector3 basePosition,Vector3 direction)
    {
        if(Vector3.Dot(V2.position - basePosition,direction) > Vector3.Dot(V0.position - basePosition,direction))
        {
            FlipTriangle(0, 2);
        }
    }

    public Vector3 GetObjectSpaceCentroid()
    {
        return (V0.position + V1.position + V2.position) / 3;
    }

    public void FlipTriangle(int flipIndexA, int flipIndexB)
    {
        IndividualVertex[] vertices = new IndividualVertex[3];
        IndividualVertex tempVertex;

        vertices[0] = V0;
        vertices[1] = V1;
        vertices[2] = V2;

        tempVertex = vertices[flipIndexB];

        vertices[flipIndexB] = vertices[flipIndexA];
        vertices[flipIndexA] = tempVertex;

        V0 = vertices[0];
        V1 = vertices[1];
        V2 = vertices[2];
    }

    public void SetNormals(Vector3 normal)
    {
        V0.SetNormal(normal);
        V1.SetNormal(normal);
        V2.SetNormal(normal);
    }

    public IndividualTriangle Clone()
    {
        IndividualVertex v0 = new IndividualVertex(V0.position, V0.normal, V0.UV);
        IndividualVertex v1 = new IndividualVertex(V1.position, V1.normal, V1.UV);
        IndividualVertex v2 = new IndividualVertex(V2.position, V2.normal, V2.UV);

        return new IndividualTriangle(v0,v1,v2);
    }



}

public class IndividualFace
{
    public IndividualFace(IndividualTriangle tri1, IndividualTriangle tri2)
    {
        this.tri1 = tri1;
        this.tri2 = tri2;
    }

    public IndividualTriangle tri1;
    public IndividualTriangle tri2;

    public bool isFaceComplete = false;
}

public class PrimitiveMesh
{
    public List<IndividualFace> individualFaces;

    public void AddFaceFromSingularTriangle(IndividualTriangle tri1)
    {
        IndividualFace newFace = new IndividualFace(tri1, null);
        newFace.isFaceComplete = false;

        individualFaces.Add(newFace);
    }

    public void AddFaceFromTriangles(IndividualTriangle tri1, IndividualTriangle tri2)
    {
        IndividualFace newFace = new IndividualFace(tri1, tri2);
        newFace.isFaceComplete = true;

        individualFaces.Add(newFace);
    }

    public void AddFaceFrom(CuttableTreeScript cts, Face face)
    {
        Mesh mesh = cts.GetMesh();

        IndividualVertex V0 = new IndividualVertex(mesh, face.tri1.v0);
        IndividualVertex V1 = new IndividualVertex(mesh, face.tri1.v1);
        IndividualVertex V2 = new IndividualVertex(mesh, face.tri1.v2);

        IndividualVertex V3 = new IndividualVertex(mesh, face.tri2.v0);
        IndividualVertex V4 = new IndividualVertex(mesh, face.tri2.v1);
        IndividualVertex V5 = new IndividualVertex(mesh, face.tri2.v2);

        IndividualTriangle tri1 = new IndividualTriangle(V0, V1, V2);
        IndividualTriangle tri2 = new IndividualTriangle(V3, V4, V5);


        IndividualFace newFace = new IndividualFace(tri1, tri2);
        newFace.isFaceComplete = true;

        individualFaces.Add(newFace);
    }

    public void AddFaceFrom(Face face)
    {

    }

    public void AddTriangleFrom(CuttableTreeScript cts, Triangle triangle)
    {
        Mesh mesh = cts.GetMesh();

        IndividualVertex V0 = new IndividualVertex(mesh, triangle.v0);
        IndividualVertex V1 = new IndividualVertex(mesh, triangle.v1);
        IndividualVertex V2 = new IndividualVertex(mesh, triangle.v2);

        IndividualVertex V3 = new IndividualVertex();
        IndividualVertex V4 = new IndividualVertex();
        IndividualVertex V5 = new IndividualVertex();

        IndividualTriangle tri1 = new IndividualTriangle(V0, V1, V2);
        IndividualTriangle tri2 = new IndividualTriangle(V3, V4, V5);

        IndividualFace newFace = new IndividualFace(tri1, tri2);
        newFace.isFaceComplete = false;

        individualFaces.Add(newFace);


    }

    public void PopulateMesh(Mesh mesh)
    {

        List<Vector3> verticesList = new List<Vector3>();
        List<Vector3> normalList = new List<Vector3>();
        List<Vector2> UVList = new List<Vector2>();
        List<int> indicesList = new List<int>();

        for (int i = 0; i < individualFaces.Count; i++)
        {
            //fill in first triangle
            verticesList.Add(individualFaces[i].tri1.V0.position);
            verticesList.Add(individualFaces[i].tri1.V1.position);
            verticesList.Add(individualFaces[i].tri1.V2.position);

            normalList.Add(individualFaces[i].tri1.V0.normal);
            normalList.Add(individualFaces[i].tri1.V1.normal);
            normalList.Add(individualFaces[i].tri1.V2.normal);

            UVList.Add(individualFaces[i].tri1.V0.UV);
            UVList.Add(individualFaces[i].tri1.V1.UV);
            UVList.Add(individualFaces[i].tri1.V2.UV);

            //fill in second triangle if it exist
            if (individualFaces[i].isFaceComplete)
            {
                verticesList.Add(individualFaces[i].tri2.V0.position);
                verticesList.Add(individualFaces[i].tri2.V1.position);
                verticesList.Add(individualFaces[i].tri2.V2.position);

                normalList.Add(individualFaces[i].tri2.V0.normal);
                normalList.Add(individualFaces[i].tri2.V1.normal);
                normalList.Add(individualFaces[i].tri2.V2.normal);

                UVList.Add(individualFaces[i].tri2.V0.UV);
                UVList.Add(individualFaces[i].tri2.V1.UV);
                UVList.Add(individualFaces[i].tri2.V2.UV);
            }

        }

        for (int i = 0; i < verticesList.Count; i++)
        {
            indicesList.Add(i);
        }


        mesh.Clear();
  

        mesh.vertices = verticesList.ToArray();
        mesh.triangles = indicesList.ToArray();
        mesh.normals = normalList.ToArray();
        mesh.uv = UVList.ToArray();

    }

    public Vector3 GetCentroid()
    {
        foreach(IndividualFace face in individualFaces)
        {

        }




        return Vector3.zero;
    }

}
