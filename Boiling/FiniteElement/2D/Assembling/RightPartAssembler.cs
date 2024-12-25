using Boiling.FiniteElement.Core.Assembling;
using SharpMath;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement._2D.Parameters;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement.Core.Assembling.Params;
using SharpMath.FiniteElement.Core.Assembling.TemplateMatrices;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.Geometry._2D;
using SharpMath.Geometry._2D.Сylinder;
using SharpMath.Matrices;
using SharpMath.Matrices.Sparse;
using SharpMath.Vectors;

namespace Boiling.FiniteElement._2D.Assembling;

public class RightPartAssembler : IVectorStackLocalAssembler<Element>
{
    private readonly Context<Point, Element, SparseMatrix> _context;
    private readonly Matrix _matrix = new(new double[4, 4]);
    private readonly ConvectionVelocity _velocity;

    public RightPartAssembler(Context<Point, Element, SparseMatrix> context, ConvectionVelocity velocity)
    {
        _context = context;
        _velocity = velocity;
    }

    public void AssembleVector(Element element, Span<double> vector, StackIndexPermutation indexes)
    {
        throw new NotImplementedException();
    }

    public void AssembleVector(Element element, double time, Span<double> vector, StackIndexPermutation indexes)
    {
        var leftRCoordinate = _context.Grid.Nodes[element.NodeIndexes[0]].R();

        var massRTemplate = CylinderTemplateMatrices.MassR1D(leftRCoordinate, element.Length);
        var massZTemplate = CylinderTemplateMatrices.MassZ1D(element.Width);

        for (var i = 0; i < element.NodeIndexes.Length; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                _matrix[i, j] = massRTemplate[Mu(i), Mu(j)] * massZTemplate[Nu(i), Nu(j)];
                _matrix[j, i] = _matrix[i, j];
            }
        }

        var densityFunction = new Func<Point, double, double>((p, t) => -4 + 1);

        Span<double> densityFunctionValues = stackalloc double[element.NodeIndexes.Length];

        for (var i = 0; i < densityFunctionValues.Length; i++)
        {
            densityFunctionValues[i] = densityFunction(_context.Grid.Nodes[element.NodeIndexes[i]], time) - _velocity.Get(_context.Grid.Nodes[element.NodeIndexes[i]]) * new Point(2 * _context.Grid.Nodes[element.NodeIndexes[i]].R(), 1d);
            vector[i] = 0;
        }

        LinAl.Multiply(_matrix, densityFunctionValues, vector);

        FillIndexes(element, indexes);
    }

    private static void FillIndexes(Element element, StackIndexPermutation indexes)
    {
        for (var i = 0; i < element.NodeIndexes.Length; i++)
        {
            indexes.Permutation[i] = element.NodeIndexes[i];
        }
    }

    private static int Mu(int i)
    {
        return i % 2;
    }

    private static int Nu(int i)
    {
        return i / 2;
    }
}