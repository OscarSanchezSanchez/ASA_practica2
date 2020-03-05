using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);

        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        // TO BE COMPLETED
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // TO BE COMPLETED
    }

    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
    }

    #endregion

}
