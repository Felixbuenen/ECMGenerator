#include "ECM.h"

#include "ECMDataTypes.h"
#include "UtilityFunctions.h"
#include "ECMCellCollection.h"

#include <memory>
#include <math.h>
#include <random>

ECM::ECM()
{
	_medialAxis = std::make_shared<MedialAxis>();
}


// ECM Graph

ECMGraph::ECMGraph() : m_NextVertexIndex(0), m_NextEdgeIndex(0)
{
	m_Cells = std::make_unique<ECMCellCollection>();
}

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
	int index = m_NextEdgeIndex;
	m_NextEdgeIndex++;
	edge.SetIndex(index);
	
	m_Edges.push_back(edge);

	return index;
}

void ECMGraph::ConstructECMCells()
{
	m_Cells->Construct(*this);
}



//void ECMGraph::AddAdjacency(int vertexIndex, int edgeIndex)
//{
//	m_Vertices[vertexIndex].AddIncidentEdge(edgeIndex);
//}



void ECMGraph::AddAdjacency(int v0, int v1, int edge)
{
	if (m_VertAdjacency.size() < (v0 + 1)) m_VertAdjacency.resize(v0 + 1);
	m_VertAdjacency[v0].push_back(edge);

	//printf("m_VertAdjacency.size(): %d\n", m_VertAdjacency.size());
}


// TODO: make KD-tree query
int ECMGraph::GetVertexIndex(float x, float y) const
{
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

	// not found
	return -1;
}

const std::vector<EdgeIndex>& ECMGraph::GetIncidentEdges(int vertex_index) const
{
	if (vertex_index >= m_VertAdjacency.size())
	{
		return std::vector<EdgeIndex>();
	}

	return m_VertAdjacency[vertex_index];
}

const ECMCell& ECMGraph::GetCell(float x, float y) const
{
	return m_Cells->PointLocationQuery(Point(x, y));
}


//// TESTY TESTY
//std::vector<Segment> ECMGraph::GetRandomTestPath(int startVertIndex) const
//{
//	std::vector<Segment> path;
//	std::vector<int> indicesVisited;
//	indicesVisited.push_back(startVertIndex);
//
//	int pathLength = 3;
//	int currentVertex = startVertIndex;
//
//	for (int i = 0; i < pathLength; i++)
//	{
//		// check adjacent edges and make sure we don't walk back
//		int nextVertex = -1;
//		for (EdgeIndex i : m_VertAdjacency[currentVertex])
//		{
//			const ECMEdge& edge = m_Edges[i];
//			int vertToConsider = edge.V0() == currentVertex ? edge.V1() : edge.V0(); // pick the vertex on the other side of the edge
//
//			// check if we've already visited this node
//			bool alreadyVisited = false;
//			for (int index : indicesVisited) {
//				if (vertToConsider == index)
//				{
//					alreadyVisited = true;
//					break;
//				}
//			}
//
//			if (!alreadyVisited)
//			{
//				nextVertex = vertToConsider;
//				break;
//			}
//		}
//
//		if (nextVertex == -1) break;
//
//		path.push_back(Segment(m_Vertices[currentVertex].Position(), m_Vertices[nextVertex].Position()));
//		currentVertex = nextVertex;
//	}
//
//	return path;
//}
//

// TESTY TESTY
std::vector<Segment> ECM::GetRandomTestPath() const
{
	return std::vector<Segment>();
	//int maxVerts = _ecmGraph.GetNextFreeVertexIndex();
	//
	//srand((unsigned int)time(NULL));
	//int startIdx = rand() % maxVerts;
	//
	//return _ecmGraph.GetRandomTestPath(startIdx);
}

const ECMCell& ECM::GetECMCell(float x, float y) const
{
	return _ecmGraph.GetCell(x, y);
}
