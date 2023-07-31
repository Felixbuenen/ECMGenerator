#include "ECM.h"

#include "ECMDataTypes.h"
#include "UtilityFunctions.h"

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

std::vector<Segment> ECM::GetECMCell(float x, float y) const
{
	// TODO: right now we check for the nearest edge and draw that cell. However, this is not correct. 
	// We must do a point location query, where the ecm edges and closest obstacles form the planar subdivision.
	// We could for now simply test against all cells manually (eg loop through all half edges, check if point inside
	// left face). But eventually we must create a point location query datastructure for this.

	const auto& edges = _ecmGraph.GetEdges();
	Point p(x, y);

	int counter = 0;
	float closestDist = 1000000;
	int closestIdx = -1;
	for (const auto& edge : edges)
	{
		ECMVertex v1 = _ecmGraph.GetVertex(edge.V0());
		ECMVertex v2 = _ecmGraph.GetVertex(edge.V1());

		Segment s(v1.Position(), v2.Position());

		Point pOnSeg = MathUtility::GetClosestPointOnSegment(p, s);
		float distance = MathUtility::Distance(p, pOnSeg);

		if (distance < closestDist)
		{
			closestDist = distance;
			closestIdx = counter;
		}

		counter++;
	}

	ECMEdge closestEdge = edges[closestIdx];
	ECMVertex v1 = _ecmGraph.GetVertex(closestEdge.V0());
	ECMVertex v2 = _ecmGraph.GetVertex(closestEdge.V1());
	Point edgePoint(v2.Position() - v1.Position());
	Vec2 edgeVec(edgePoint.x, edgePoint.y);
	Vec2 edgeRight = MathUtility::Right(edgeVec);

	bool isLeft = MathUtility::Dot(edgeRight, Vec2(x - v1.Position().x, y - v1.Position().y)) < 0.0f;
	
	std::vector<Segment> result;
	if (isLeft)
	{
		result.push_back(Segment(v1.Position(), v2.Position()));
		result.push_back(Segment(v2.Position(), closestEdge.NearestLeftV1()));
		result.push_back(Segment(closestEdge.NearestLeftV1(), closestEdge.NearestLeftV0()));
		result.push_back(Segment(closestEdge.NearestLeftV0(), v1.Position()));
	}
	else
	{
		result.push_back(Segment(v1.Position(), v2.Position()));
		result.push_back(Segment(v2.Position(), closestEdge.NearestRightV1()));
		result.push_back(Segment(closestEdge.NearestRightV1(), closestEdge.NearestRightV0()));
		result.push_back(Segment(closestEdge.NearestRightV0(), v1.Position()));
	}

	return result;
}
