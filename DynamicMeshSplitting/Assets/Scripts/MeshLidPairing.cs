using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public struct MeshLidPairing
{

    public IndividualVertex v0;
    public IndividualVertex v1;

    public MeshLidPairing(IndividualVertex v0, IndividualVertex v1)
    {
        this.v0 = v0;
        this.v1 = v1;
    }

    public IndividualTriangle CreateTriangle(IndividualVertex centerVertex)
    {
        return new IndividualTriangle(v0, centerVertex, v1);
    }




}
