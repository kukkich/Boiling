using SharpMath.FiniteElement.Core.Assembling.Boundary.First;
using SharpMath.FiniteElement.Core.Assembling.Boundary.Second;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D;
using SharpMath.Geometry._2D;
using SharpMath.Matrices.Sparse;
using Boiling.FiniteElement.Core.Assembling;
using Boiling.FiniteElement.Time.Assembling;
using SharpMath.FiniteElement.Core.Assembling.Boundary.Third;
using SharpMath.Matrices;
using SharpMath;
using SharpMath.Vectors;

namespace Boiling.FiniteElement._2D;

public class BoilingEquationAssembler
{
    public Equation<SparseMatrix> CurrentTimeLayerEquation => _context.Equation;

    private readonly BoilingContext<Point, Element, SparseMatrix> _context;
    private readonly IMatrixStackLocalAssembler<Element> _localStiffnessMatrixAssembler;
    private readonly IMatrixStackLocalAssembler<Element> _localMassMatrixAssembler;
    private readonly IMatrixStackLocalAssembler<Element> _localVelocityMatrixAssembler;
    private readonly IStackInserter<SparseMatrix> _inserter;
    private readonly ISecondBoundaryApplier<SparseMatrix> _secondBoundaryApplier;
    private readonly IThirdBoundaryApplier<SparseMatrix> _thirdBoundaryApplier;
    private TwoLayerImplicitScheme _timeScheme;

    public BoilingEquationAssembler(
        BoilingContext<Point, Element, SparseMatrix> context,
        IMatrixStackLocalAssembler<Element> localStiffnessMatrixAssembler,
        IMatrixStackLocalAssembler<Element> localMassMatrixAssembler,
        IMatrixStackLocalAssembler<Element> localVelocityMatrixAssembler,
        IStackInserter<SparseMatrix> inserter,
        ISecondBoundaryApplier<SparseMatrix> secondBoundaryApplier,
        IThirdBoundaryApplier<SparseMatrix> thirdBoundaryApplier
    )
    {
        _context = context;
        _localStiffnessMatrixAssembler = localStiffnessMatrixAssembler;
        _localMassMatrixAssembler = localMassMatrixAssembler;
        _localVelocityMatrixAssembler = localVelocityMatrixAssembler;
        _inserter = inserter;
        _secondBoundaryApplier = secondBoundaryApplier;
        _thirdBoundaryApplier = thirdBoundaryApplier;
    }

    public BoilingEquationAssembler BuildEquation(Vector previousSolution, double currentTime, double previousTime)
    {
        if (_timeScheme == null)
        {
            var matrix = new StackMatrix(stackalloc double[4 * 4], 4);
            var indexes = new StackIndexPermutation(stackalloc int[4]);

            var i = 0;
            foreach (var element in _context.Grid.Elements)
            {
                i++;
                var localMatrix = new StackLocalMatrix(matrix, indexes);

                if (i % 100 == 0)
                {
                    Console.WriteLine(i);
                }
                
                _localStiffnessMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.StiffnessAndVelocityMatrix, localMatrix);

                _localMassMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.MassMatrix, localMatrix);

                _localVelocityMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.StiffnessAndVelocityMatrix, localMatrix);
            }

            _timeScheme = new TwoLayerImplicitScheme(_context.StiffnessAndVelocityMatrix, _context.MassMatrix);
        }

        _context.Equation = _timeScheme.UseScheme(previousSolution, currentTime, previousTime);

        return this;
    }

    public BoilingEquationAssembler ApplySecondBoundary(BoilingContext<Point, Element, SparseMatrix> context)
    {
        var equation = context.Equation;

        foreach (var condition in _context.SecondConditions)
        {
            _secondBoundaryApplier.Apply(equation, condition);
        }

        return this;
    }

    public BoilingEquationAssembler ApplyThirdBoundary(BoilingContext<Point, Element, SparseMatrix> context)
    {
        foreach (var condition in context.ThirdConditions)
        {
            _thirdBoundaryApplier.Apply(context.Equation, condition);
        }

        return this;
    }
}