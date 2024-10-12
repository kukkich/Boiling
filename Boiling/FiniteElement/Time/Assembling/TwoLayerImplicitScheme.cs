using SharpMath;
using SharpMath.Matrices.Sparse;
using SharpMath.Vectors;

namespace Boiling.FiniteElement.Time.Assembling;

public class TwoLayerImplicitScheme
{
    private readonly SparseMatrix _stiffnessAndVelocityMatrix;
    private readonly SparseMatrix _massMatrix;
    private readonly SparseMatrix _bufferMatrix;
    private readonly Vector _bufferVector;

    public TwoLayerImplicitScheme(SparseMatrix stiffnessAndVelocityMatrix, SparseMatrix massMatrix)
    {
        _stiffnessAndVelocityMatrix = stiffnessAndVelocityMatrix;
        _massMatrix = massMatrix;
        _bufferMatrix = new SparseMatrix(_massMatrix.RowsIndexes, _massMatrix.ColumnsIndexes);
        _bufferVector = new Vector(new double[stiffnessAndVelocityMatrix.RowsCount]);
    }

    public Equation<SparseMatrix> UseScheme(Vector rightPart, Vector previousSolution, double currentTime, double previousTime)
    {
        _bufferVector.Nullify();
        var delta01 = currentTime - previousTime;

        LinAl.Multiply(1 / delta01, _massMatrix, _bufferMatrix);
        LinAl.Multiply(_bufferMatrix, previousSolution, _bufferVector);
        //LinAl.Sum(rightPart, _bufferVector, _bufferVector);
        LinAl.Sum(_stiffnessAndVelocityMatrix, _bufferMatrix, _bufferMatrix);

        return new Equation<SparseMatrix>(_bufferMatrix, Vector.Create(_bufferVector.Length), _bufferVector);
    }
}