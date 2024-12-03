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
    private readonly IVectorStackLocalAssembler<Element> _rightPartAssembler;
    private readonly IStackInserter<SparseMatrix> _inserter;
    private readonly IFirstBoundaryApplier<SparseMatrix> _firstBoundaryApplier;
    private readonly ISecondBoundaryApplier<SparseMatrix> _secondBoundaryApplier;
    private readonly IThirdBoundaryApplier<SparseMatrix> _thirdBoundaryApplier;
    private TwoLayerImplicitScheme _timeScheme;

    public BoilingEquationAssembler(
        BoilingContext<Point, Element, SparseMatrix> context,
        IMatrixStackLocalAssembler<Element> localStiffnessMatrixAssembler,
        IMatrixStackLocalAssembler<Element> localMassMatrixAssembler,
        IMatrixStackLocalAssembler<Element> localVelocityMatrixAssembler,
        IVectorStackLocalAssembler<Element> rightPartAssembler,
        IStackInserter<SparseMatrix> inserter,
        IFirstBoundaryApplier<SparseMatrix> firstBoundaryApplier,
        ISecondBoundaryApplier<SparseMatrix> secondBoundaryApplier,
        IThirdBoundaryApplier<SparseMatrix> thirdBoundaryApplier
    )
    {
        _context = context;
        _localStiffnessMatrixAssembler = localStiffnessMatrixAssembler;
        _localMassMatrixAssembler = localMassMatrixAssembler;
        _localVelocityMatrixAssembler = localVelocityMatrixAssembler;
        _rightPartAssembler = rightPartAssembler;
        _inserter = inserter;
        _firstBoundaryApplier = firstBoundaryApplier;
        _secondBoundaryApplier = secondBoundaryApplier;
        _thirdBoundaryApplier = thirdBoundaryApplier;
    }

    public BoilingEquationAssembler BuildEquation(Vector previousSolution, double currentTime, double previousTime)
    {
        if (_timeScheme == null)
        {
            var matrix = new StackMatrix(stackalloc double[4 * 4], 4);
            var indexes = new StackIndexPermutation(stackalloc int[4]);

            foreach (var element in _context.Grid.Elements)
            {
                var localMatrix = new StackLocalMatrix(matrix, indexes);
                
                _localStiffnessMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.StiffnessAndVelocityMatrix, localMatrix);

                _localMassMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.MassMatrix, localMatrix);

                _localVelocityMatrixAssembler.AssembleMatrix(element, matrix, indexes);
                _inserter.InsertMatrix(_context.StiffnessAndVelocityMatrix, localMatrix);
            }

            _timeScheme = new TwoLayerImplicitScheme(_context.StiffnessAndVelocityMatrix, _context.MassMatrix);
        }

        //var vectorIndexes = new StackIndexPermutation(stackalloc int[4]);

        //Span<double> vector = stackalloc double[4];
        //var j = 0;
        //foreach (var element in _context.Grid.Elements)
        //{
        //    j++;
            
        //    var localVector = new StackLocalVector(vector, vectorIndexes);

        //    if (j % 100 == 0)
        //    {
        //        Console.WriteLine(j);
        //    }

        //    _rightPartAssembler.AssembleVector(element, currentTime, vector, vectorIndexes);
        //    _inserter.InsertVector(_context.RightPart, localVector);
        //}

        _context.Equation = _timeScheme.UseScheme(null, previousSolution, currentTime, previousTime);

        return this;
    }

    public BoilingEquationAssembler ApplyFirstBoundary(Context<Point, Element, SparseMatrix> context)
    {
        var equation = context.Equation;

        foreach (var condition in _context.FirstConditions)
        {
            _firstBoundaryApplier.Apply(equation, condition);
        }

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