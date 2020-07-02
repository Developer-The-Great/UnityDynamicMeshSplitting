using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CuttableTreeScript))]
public class CuttableMeshPhysicsManager : MonoBehaviour
{

    private MeshCollider cutCollider;
    private MeshFilter meshFilter;

    private Rigidbody rb;

    public void GenerateMeshColliderFromCut(Mesh newCutMesh,bool rigidbodyNeeded = false)
    {
        //check if it already has a mesh collider
        cutCollider = gameObject.GetComponent<MeshCollider>();

        if(cutCollider == null)
        {
            cutCollider = gameObject.AddComponent<MeshCollider>();
        }
        else
        {
            cutCollider.sharedMesh = null;
            cutCollider.sharedMesh = gameObject.GetComponent<MeshFilter>().mesh;

        }

        //gameObject.GetComponent<BoxCollider>();

        cutCollider.convex = true;

        if(rigidbodyNeeded)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            //rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        }
    }

    public void AddForceAt(float magnitude,Vector3 direction,Vector3 position)
    {
        rb.AddForceAtPosition(direction * magnitude, position);
    }

    //public void Upd

    public float GetMaxCylinderRadius(Mesh mesh,Vector3 centerPoint,Vector3 centerMinPoint,Vector3 meshDirection)
    {
        //squish center point into plane

        Vector3 centerPointIntersection;
        float tCenter;
        Utils.CustomLineToPlaneIntersection(centerPoint, centerPoint - meshDirection, centerMinPoint, Vector3.up, out centerPointIntersection, out tCenter);

        float largestSquaredDistance = float.MinValue;
        foreach (Vector3 vertex in mesh.vertices)
        {
            //float foundSqrDistance = Vector3.SqrMagnitude(vertex - )

            //squish vertex into plane
            Vector3 vertexIntersection;
            float t;
            Utils.CustomLineToPlaneIntersection(vertex, vertex - meshDirection, centerMinPoint, Vector3.up, out vertexIntersection, out t);

            float foundSqrLength = Vector3.SqrMagnitude(vertexIntersection - centerPointIntersection);

            if(foundSqrLength > largestSquaredDistance)
            {
                largestSquaredDistance = foundSqrLength;
            }





        }



        return Mathf.Sqrt(largestSquaredDistance);
    }




}
