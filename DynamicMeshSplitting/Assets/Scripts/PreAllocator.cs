using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;




public class PreAllocator : MonoBehaviour
{
    NativeQueue<JobFace> rawLowerMeshQueue;
    NativeQueue<JobFace> rawUpperMeshQueue;
    // Start is called before the first frame update
    void Start()
    {
        rawLowerMeshQueue = new NativeQueue<JobFace>(Allocator.Persistent);
        rawUpperMeshQueue = new NativeQueue<JobFace>(Allocator.Persistent);
    }

    public NativeQueue<JobFace> GetLowerMeshQueue()
    {
        return rawLowerMeshQueue;
    }

    public NativeQueue<JobFace> GetUpperMeshQueue()
    {
        return rawUpperMeshQueue;
    }

    public void OnDestroy()
    {
        rawUpperMeshQueue.Dispose();
        rawLowerMeshQueue.Dispose();

    }

}
