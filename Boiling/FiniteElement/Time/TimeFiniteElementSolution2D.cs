using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement._2D.BasisFunctions;
using SharpMath.Geometry;
using SharpMath.Geometry._2D;
using SharpMath.Vectors;

namespace Boiling.FiniteElement.Time;

public class TimeFiniteElementSolution2D
{
    private readonly BilinearBasisFunctionsProvider _basisFunctionsProvider;
    private readonly Grid<Point, Element> _grid;
    private readonly Vector[] _solutions;
    private readonly double[] _timeLayers;

    public TimeFiniteElementSolution2D(
        BilinearBasisFunctionsProvider basisFunctionsProvider,
        Grid<Point, Element> grid,
        Vector[] solutions,
        double[] timeLayers
    )
    {
        _basisFunctionsProvider = basisFunctionsProvider;
        _grid = grid;
        _solutions = solutions;
        _timeLayers = timeLayers;
    }

    public double Calculate(Point point, double time)
    {
        var currentTimeLayerIndex = FindCurrentTimeLayer(time);

        var element = _grid.Elements.First(x => ElementHas(x, point));

        var basisFunctions = _basisFunctionsProvider.GetFunctions(element);

        var polynomials = CreateLagrangePolynomials(currentTimeLayerIndex);

        return polynomials
            .Select((polynomial, i) => element.NodeIndexes
                .Select((nodeIndex, j) => _solutions[currentTimeLayerIndex - i][nodeIndex] * basisFunctions[j].Evaluate(point))
                .Sum() * polynomial(time))
            .Sum();
    }

    private Func<double, double>[] CreateLagrangePolynomials(int timeLayerIndex)
    {
        var currentTime = _timeLayers[timeLayerIndex];
        var previousTime = _timeLayers[timeLayerIndex - 1];
        var polynomials = new Func<double, double>[]
        {
            t => (t - currentTime) / (previousTime - currentTime),
            t => (t - previousTime) / (currentTime - previousTime),
        };

        return polynomials;
    }

    private bool ElementHas(Element element, Point node)
    {
        var leftBottom = _grid.Nodes[element.NodeIndexes[0]];
        var rightTop = _grid.Nodes[element.NodeIndexes[^1]];

        return leftBottom.X <= node.X && node.X <= rightTop.X &&
               leftBottom.Y <= node.Y && node.Y <= rightTop.Y;
    }

    private int FindCurrentTimeLayer(double time)
    {
        return Array.FindIndex(_timeLayers, x => Math.Abs(time - x) <= 1e-15 || time <= x);
    }
}