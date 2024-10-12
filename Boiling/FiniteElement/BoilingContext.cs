using SharpMath.FiniteElement.Core.Assembling.Boundary.First;
using SharpMath.FiniteElement.Core.Assembling.Boundary.Second;
using SharpMath.FiniteElement.Core.Assembling.Boundary.Third;
using SharpMath.FiniteElement.Core.Assembling.Params;
using SharpMath.FiniteElement.Materials.HarmonicWithoutChi;
using SharpMath.Geometry;
using SharpMath;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.Matrices.Sparse;
using SharpMath.Vectors;

namespace Boiling.FiniteElement;

public class BoilingContext<TPoint, TElement, TMatrix> : Context<TPoint, TElement, TMatrix>
{
    public required double[] TimeLayers { get; set; }
    public required SparseMatrix StiffnessAndVelocityMatrix { get; set; }
    public required SparseMatrix MassMatrix { get; set; }
    public required Vector RightPart { get; set; }
    public required Vector[] TimeSolutions { get; set; }
}