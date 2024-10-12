using SharpMath.FiniteElement.Core.Assembling;

namespace Boiling.FiniteElement.Core.Assembling;

public interface IVectorLocalAssembler<in TElement>
{
    public LocalVector AssembleVector(TElement element);
}

public interface IVectorStackLocalAssembler<in TElement>
{
    public void AssembleVector(TElement element, Span<double> vector, StackIndexPermutation indexes);
    public void AssembleVector(TElement element, double time, Span<double> vector, StackIndexPermutation indexes);
}