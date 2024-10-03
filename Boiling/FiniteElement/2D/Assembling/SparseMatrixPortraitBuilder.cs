using SharpMath.FiniteElement._2D;
using SharpMath.FiniteElement.Core.Assembling;
using SharpMath.Matrices.Sparse;

namespace Boiling.FiniteElement._2D.Assembling;

public class SparseMatrixPortraitBuilder : IMatrixPortraitBuilder<SparseMatrix, Element>
{
    private List<SortedSet<int>> _adjacencyList = null!;

    public SparseMatrix Build(IEnumerable<Element> elements, int nodesCount)
    {
        BuildAdjacencyList(elements, nodesCount);

        var amount = 0;
        var buf = _adjacencyList.Select(nodeSet => amount += nodeSet.Count).ToList();
        buf.Insert(0, 0);

        var rowsIndexes = buf.ToArray();
        var columnsIndexes = _adjacencyList.SelectMany(nodeList => nodeList).ToArray();

        return new SparseMatrix(rowsIndexes, columnsIndexes);
    }

    private void BuildAdjacencyList(IEnumerable<Element> elements, int nodesCount)
    {
        _adjacencyList = new List<SortedSet<int>>(nodesCount);

        for (var i = 0; i < nodesCount; i++)
        {
            _adjacencyList.Add([]);
        }

        foreach (var element in elements)
        {
            var nodeIndexes = element.NodeIndexes;

            foreach (var currentNode in nodeIndexes)
            {
                foreach (var nodeIndex in nodeIndexes)
                {
                    if (currentNode > nodeIndex) _adjacencyList[currentNode].Add(nodeIndex);
                }
            }
        }
    }
}