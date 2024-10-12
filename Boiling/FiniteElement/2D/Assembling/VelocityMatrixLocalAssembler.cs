using Boiling.FiniteElement.Core.Assembling;
using SharpMath.Computations.Integration;
using SharpMath.FiniteElement;
using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement._2D.Parameters;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.FiniteElement.Core.Assembling.Params;
using SharpMath.FiniteElement.Core.BasisFunctions;
using SharpMath.FiniteElement.Materials.Boiling;
using SharpMath.Geometry;
using SharpMath.Geometry._2D;
using SharpMath.Geometry._2D.Сylinder;
using SharpMath.Matrices;
using SharpMath.Matrices.Sparse;

namespace Boiling.FiniteElement._2D.Assembling;

public class VelocityMatrixLocalAssembler : IMatrixStackLocalAssembler<Element>
{
    private readonly Context<Point, Element, SparseMatrix> _context;
    private readonly IMaterialProvider<BoilingMaterial> _materials;
    private readonly ConvectionVelocity _velocity;
    private readonly DoubleIntegration _doubleIntegration;
    private readonly IBasisFunctionsProvider<Element, Point> _basisFunctionsProvider;
    private readonly Func<double, double>[] _derivativeByRFunctions;
    private readonly Func<double, double>[] _derivativeByZFunctions;

    public VelocityMatrixLocalAssembler(
	    Context<Point, Element, SparseMatrix> context, 
	    IMaterialProvider<BoilingMaterial> materials,
        ConvectionVelocity velocity,
		DoubleIntegration doubleIntegration,
		IBasisFunctionsProvider<Element, Point> basisFunctionsProvider
	)
    {
        _context = context;
        _materials = materials;
        _velocity = velocity;
		_doubleIntegration = doubleIntegration;
		_basisFunctionsProvider = basisFunctionsProvider;
        _derivativeByRFunctions = new Func<double, double>[4];
		_derivativeByZFunctions = new Func<double, double>[4];
    }

    public void AssembleMatrix(Element element, StackMatrix matrix, StackIndexPermutation indexes)
    {
        var material = _materials.GetById(element.MaterialId);

	    var rInterval = new Interval(_context.Grid.Nodes[element.NodeIndexes[0]].R(), _context.Grid.Nodes[element.NodeIndexes[1]].R());
	    var zInterval = new Interval(_context.Grid.Nodes[element.NodeIndexes[0]].Z(), _context.Grid.Nodes[element.NodeIndexes[2]].Z());

	    var basisFunctions = _basisFunctionsProvider.GetFunctions(element);
        var derivativeByRFunctions = GetDerivativeByRFunctions(element);
        var derivativeByZFunctions = GetDerivativeByZFunctions(element);

        var point = new Point();

	    for (var i = 0; i < element.NodeIndexes.Length; i++)
	    {
		    for (var j = 0; j < element.NodeIndexes.Length; j++)
		    {
			    matrix[i, j] = -material.Cp * material.Rho * _doubleIntegration.Integrate(rInterval, zInterval,
				    (r, z) =>
				    {
					    point.X = r;
					    point.Y = z;
					    var velocity = _velocity.Get(point);
					    return (velocity.R() * derivativeByRFunctions[i](z) +
					            velocity.Z() * derivativeByZFunctions[i](r)) *
					           basisFunctions[j].Evaluate(point) * r;
				    });
		    }
	    }
    }

    private Func<double, double>[] GetDerivativeByRFunctions(Element element)
    {
        var leftCoordinate = _context.Grid.Nodes[element.NodeIndexes[0]].Z();
        var rightCoordinate = _context.Grid.Nodes[element.NodeIndexes[2]].Z();

        _derivativeByRFunctions[0] = coordinate => -(rightCoordinate - coordinate) / (element.Width * element.Length);
        _derivativeByRFunctions[1] = coordinate => (rightCoordinate - coordinate) / (element.Width * element.Length);
        _derivativeByRFunctions[2] = coordinate => -(coordinate - leftCoordinate) / (element.Width * element.Length);
        _derivativeByRFunctions[3] = coordinate => (coordinate - leftCoordinate) / (element.Width * element.Length);

        return _derivativeByRFunctions;
    }

    private Func<double, double>[] GetDerivativeByZFunctions(Element element)
    {
        var leftCoordinate = _context.Grid.Nodes[element.NodeIndexes[0]].R();
        var rightCoordinate = _context.Grid.Nodes[element.NodeIndexes[1]].R();

        _derivativeByZFunctions[0] = coordinate => -(rightCoordinate - coordinate) / (element.Width * element.Length);
        _derivativeByZFunctions[1] = coordinate => -(coordinate - leftCoordinate) / (element.Width * element.Length);
        _derivativeByZFunctions[2] = coordinate => (rightCoordinate - coordinate) / (element.Width * element.Length);
        _derivativeByZFunctions[3] = coordinate => (coordinate - leftCoordinate) / (element.Width * element.Length);

        return _derivativeByZFunctions;
    }
}