#include "ECM.h"

#include "ECMDataTypes.h"

#include <memory>
#include <math.h>
#include <random>

ECM::ECM()
{
	_medialAxis = std::make_shared<MedialAxis>();
}


// ECM Graph

// TODO: make into KD-tree insert
int ECMGraph::AddVertex(ECMVertex vertex)
{
	m_Vertices.push_back(vertex);

	int index = m_NextVertexIndex;
	m_NextVertexIndex++;

	return index;
}

// TODO: make into KD-tree insert
int ECMGraph::AddEdge(ECMEdge edge)
{
	m_Edges.push_back(edge);

	int index = m_NextEdgeIndex;
	m_NextEdgeIndex++;

	return index;
}

void ECMGraph::AddAdjacency(int v0, int v1, int edge)
{
	if (m_VertAdjacency.size() < (v0 + 1)) m_VertAdjacency.resize(v0 + 1);
	if (m_VertAdjacency.size() < (v1 + 1)) m_VertAdjacency.resize(v1 + 1);

	m_VertAdjacency[v0].push_back(edge);
	m_VertAdjacency[v1].push_back(edge);
}


// TODO: make KD-tree query
int ECMGraph::GetVertexIndex(float x, float y) const
{
	float xtest = x;
	float ytest = y;
	int index = 0;
	for (const ECMVertex& vert : m_Vertices)
	{
		const Point& pos = vert.Position();
		bool isSamePosition = abs(x - vert.Position().x) < ECM_EPSILON && abs(y - vert.Position().y) < ECM_EPSILON;

		if (isSamePosition)
		{
			return index;
		}

		index++;
	}

	int hallo = 0;
	// not found
	return -1;
}

// TESTY TESTY
std::vector<Segment> ECMGraph::GetRandomTestPath(int startVertIndex) const
{
	std::vector<Segment> path;
	std::vector<int> indicesVisited;
	indicesVisited.push_back(startVertIndex);

	int pathLength = 3;
	int currentVertex = startVertIndex;

	for (int i = 0; i < pathLength; i++)
	{
		// check adjacent edges and make sure we don't walk back
		int nextVertex = -1;
		for (EdgeIndex i : m_VertAdjacency[currentVertex])
		{
			const ECMEdge& edge = m_Edges[i];
			int vertToConsider = edge.V1() == currentVertex ? edge.V2() : edge.V1(); // pick the vertex on the other side of the edge

			// check if we've already visited this node
			bool alreadyVisited = false;
			for (int index : indicesVisited) {
				if (vertToConsider == index)
				{
					alreadyVisited = true;
					break;
				}
			}

			if (!alreadyVisited)
			{
				nextVertex = vertToConsider;
				break;
			}
		}

		if (nextVertex == -1) break;

		path.push_back(Segment(m_Vertices[currentVertex].Position(), m_Vertices[nextVertex].Position()));
		currentVertex = nextVertex;
	}

	return path;
}


// TESTY TESTY
std::vector<Segment> ECM::GetRandomTestPath() const
{
	int maxVerts = _ecmGraph.GetNextFreeVertexIndex();

	srand((unsigned int)time(NULL));
	int startIdx = rand() % maxVerts;

	return _ecmGraph.GetRandomTestPath(startIdx);
}
