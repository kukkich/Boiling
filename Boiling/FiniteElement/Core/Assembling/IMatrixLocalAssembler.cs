using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.Matrices;

namespace Boiling.FiniteElement.Core.Assembling;

public interface IMatrixLocalAssembler<in TElement>
{
    public LocalMatrix AssembleMatrix(TElement element);
}

public interface IMatrixStackLocalAssembler<in TElement>
{
    public void AssembleMatrix(TElement element, StackMatrix matrix, StackIndexPermutation indexes);
}