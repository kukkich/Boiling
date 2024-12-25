using Boiling.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement.Core.Assembling.Params;
using SharpMath.FiniteElement.Core.Assembling.TemplateMatrices;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.FiniteElement.Materials.HarmonicWithoutChi;
using SharpMath.Geometry._2D;
using SharpMath.Geometry._2D.Сylinder;
using SharpMath.Matrices;
using SharpMath.Matrices.Sparse;

namespace Boiling.FiniteElement._2D.Assembling;

public class MassMatrixLocalAssembler : IMatrixStackLocalAssembler<Element>
{
    private readonly Context<Point, Element, SparseMatrix> _context;
    private readonly IMaterialProvider<BoilingMaterial> _materials;

    public MassMatrixLocalAssembler(Context<Point, Element, SparseMatrix> context, IMaterialProvider<BoilingMaterial> materials)
    {
        _context = context;
        _materials = materials;
    }

    public void AssembleMatrix(Element element, StackMatrix matrix, StackIndexPermutation indexes)
    {
	    var material = _materials.GetById(element.MaterialId);

        var leftRCoordinate = _context.Grid.Nodes[element.NodeIndexes[0]].R();

        var massRTemplate = CylinderTemplateMatrices.MassR1D(leftRCoordinate, element.Length);
        var massZTemplate = CylinderTemplateMatrices.MassZ1D(element.Width);

        for (var i = 0; i < element.NodeIndexes.Length; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                matrix[i, j] = material.Cp * material.Rho * (massRTemplate[Mu(i), Mu(j)] * massZTemplate[Nu(i), Nu(j)]);
                matrix[j, i] = matrix[i, j];
            }
        }

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