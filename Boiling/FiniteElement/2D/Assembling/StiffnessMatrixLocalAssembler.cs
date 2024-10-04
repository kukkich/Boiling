﻿using Boiling.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement.Core.Assembling.Params;
using SharpMath.FiniteElement.Core.Assembling.TemplateMatrices;
using SharpMath.FiniteElement.Core.Harmonic;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.Geometry._2D;
using SharpMath.Geometry._2D.Сylinder;
using SharpMath.Matrices;
using SharpMath.Matrices.Sparse;

namespace Boiling.FiniteElement._2D.Assembling;

public class StiffnessMatrixLocalAssembler : IMatrixStackLocalAssembler<Element>
{
    private readonly Context<Point, Element, SparseMatrix> _context;
    private readonly IMaterialProvider<BoilingMaterial> _materials;

    public StiffnessMatrixLocalAssembler(Context<Point, Element, SparseMatrix> context, IMaterialProvider<BoilingMaterial> materials)
    {
        _context = context;
        _materials = materials;
    }

    public void AssembleMatrix(Element element, StackMatrix matrix, StackIndexPermutation indexes)
    {
	    var material = _materials.GetById(element.MaterialId);
        var leftRCoordinate = _context.Grid.Nodes[element.NodeIndexes[0]].R();

        var stiffnessRTemplate = CylinderTemplateMatrices.StiffnessR1D(leftRCoordinate, element.Width);
        var stiffnessZTemplate = CylinderTemplateMatrices.StiffnessZ1D(element.Length);
        var massRTemplate = CylinderTemplateMatrices.MassR1D(leftRCoordinate, element.Width);
        var massZTemplate = CylinderTemplateMatrices.MassZ1D(element.Length);

        for (var i = 0; i < element.NodeIndexes.Length; i++)
        {
            for (var j = 0; j <= i; j++)
            {
                matrix[i, j] = material.Lambda *
                               (stiffnessRTemplate[Mu(i), Mu(j)] * massZTemplate[Nu(i), Nu(j)] +
                                massRTemplate[Mu(i), Mu(j)] * stiffnessZTemplate[Nu(i), Nu(j)]);
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