#include "ECM.h"

#include "ECMDataTypes.h"
#include "UtilityFunctions.h"
#include "ECMCellCollection.h"

#include <memory>
#include <math.h>
#include <random>

namespace ECM {

	ECM::ECM()
	{
		m_MedialAxis = std::make_shared<MedialAxis>();
	}

	const void ECM::RetractPoint(Point location, Point& outRetractedLocation, ECMEdge& outEdge) const
	{
		// TODO:
		// Retract point on arc is currently not implemented!


		const ECMCell* cell = m_EcmGraph.GetCell(location.x, location.y);
		outEdge = m_EcmGraph.GetEdge(cell->ecmEdge);
		const ECMVertex& p1 = m_EcmGraph.GetVertex(outEdge.V0());
		const ECMVertex& p2 = m_EcmGraph.GetVertex(outEdge.V1());
		
		outRetractedLocation = Utility::MathUtility::GetClosestPointOnSegment(location, Segment(p1.Position(), p2.Position()));
	}

	const void ECM::RetractPoint(Point location, const ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge) const
	{
		outEdge = m_EcmGraph.GetEdge(cell.ecmEdge);
		const ECMVertex& p1 = m_EcmGraph.GetVertex(outEdge.V0());
		const ECMVertex& p2 = m_EcmGraph.GetVertex(outEdge.V1());

		outRetractedLocation = Utility::MathUtility::GetClosestPointOnSegment(location, Segment(p1.Position(), p2.Position()));
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
		using Utility::EPSILON;

		int index = 0;
		for (const ECMVertex& vert : m_Vertices)
		{
			const Point& pos = vert.Position();
			bool isSamePosition = abs(x - vert.Position().x) < EPSILON && abs(y - vert.Position().y) < EPSILON;

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


	const ECMCell* ECMGraph::GetCell(float x, float y) const
	{
		return m_Cells->PointLocationQuery(Point(x, y));
	}




	std::vector<Segment> ECMGraph::GetSampledEdge(const ECMEdge& edge, int samples, bool inverseDirection) const
	{
		std::vector<Segment> result;

		const ECMVertex& v1 = inverseDirection ? m_Vertices[edge.V1()] : m_Vertices[edge.V0()];
		const ECMVertex& v2 = inverseDirection ? m_Vertices[edge.V0()] : m_Vertices[edge.V1()];
		Point p1 = v1.Position();
		Point p2 = v2.Position();

		if (!edge.IsArc())
		{
			result.push_back(Segment(p1, p2));
			return result;
		}
	}


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

	const ECMCell* ECM::GetECMCell(float x, float y) const
	{
		return m_EcmGraph.GetCell(x, y);
	}

}