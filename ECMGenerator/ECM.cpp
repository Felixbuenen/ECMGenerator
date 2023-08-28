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

	const bool ECM::RetractPoint(Point location, Point& outRetractedLocation, ECMEdge& outEdge) const
	{
		// TODO:
		// Retract point on arc is currently not implemented!


		const ECMCell* cell = m_EcmGraph.GetCell(location.x, location.y);
		outEdge = m_EcmGraph.GetEdge(cell->ecmEdge);
		const ECMVertex& p1 = m_EcmGraph.GetVertex(outEdge.V0());
		const ECMVertex& p2 = m_EcmGraph.GetVertex(outEdge.V1());

		// get closest obstacle based on which side of the ECM edge the location is
		Point obstacleP1, obstacleP2;
		Vec2 rayDir;
		if (Utility::MathUtility::IsLeftOfSegment(Segment(p1.Position(), p2.Position()), location))
		{
			obstacleP1 = outEdge.NearestLeftV0();
			obstacleP2 = outEdge.NearestLeftV1();
			Vec2 v = obstacleP2 - obstacleP1;
			rayDir.x = v.y;
			rayDir.y = -v.x;
			rayDir.Normalize();
		}
		else
		{
			obstacleP1 = outEdge.NearestRightV0();
			obstacleP2 = outEdge.NearestRightV1();
			Vec2 v = obstacleP2 - obstacleP1;
			rayDir.x = -v.y;
			rayDir.y = v.x;
			rayDir.Normalize();
		}

		return Utility::MathUtility::GetRayToLineSegmentIntersection(location, rayDir, p1.Position(), p2.Position(), outRetractedLocation);
	}

	const bool ECM::RetractPoint(Point location, const ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge) const
	{
		outEdge = m_EcmGraph.GetEdge(cell.ecmEdge);
		const ECMVertex& p1 = m_EcmGraph.GetVertex(outEdge.V0());
		const ECMVertex& p2 = m_EcmGraph.GetVertex(outEdge.V1());

		// get closest obstacle based on which side of the ECM edge the location is
		Point obstacleP1, obstacleP2;
		Vec2 rayDir;

		// TODO: deze code houdt geen rekening met point obstacles, dit gaat nu fout.
		if (Utility::MathUtility::IsLeftOfSegment(Segment(p1.Position(), p2.Position()), location))
		{
			obstacleP1 = outEdge.NearestLeftV0();
			obstacleP2 = outEdge.NearestLeftV1();

			//point obstacle
			if (obstacleP1.x == obstacleP2.x && obstacleP1.y == obstacleP2.y)
			{
				Vec2 v1 = p1.Position() - obstacleP1;
				Vec2 v2 = p2.Position() - obstacleP1;
				rayDir = v1 + v2;

			}
			else
			{
				Vec2 v = obstacleP2 - obstacleP1;
				rayDir.x = v.y;
				rayDir.y = -v.x;

			}
			
			rayDir.Normalize();
		}
		else
		{
			obstacleP1 = outEdge.NearestRightV0();
			obstacleP2 = outEdge.NearestRightV1();
			//point obstacle
			if (obstacleP1.x == obstacleP2.x && obstacleP1.y == obstacleP2.y)
			{
				Vec2 v1 = p1.Position() - obstacleP1;
				Vec2 v2 = p2.Position() - obstacleP1;
				rayDir = v1 + v2;
			}
			else
			{
				Vec2 v = obstacleP2 - obstacleP1;
				rayDir.x = -v.y;
				rayDir.y = v.x;

			}
			rayDir.Normalize();
		}

		return Utility::MathUtility::GetRayToLineSegmentIntersection(location, rayDir, p1.Position(), p2.Position(), outRetractedLocation);
	}


	// ECM Graph

	ECMGraph::ECMGraph() : m_NextVertexIndex(0), m_NextEdgeIndex(0)
	{
		m_Cells = std::make_unique<ECMCellCollection>();
	}

	// TODO: make into KD-tree insert
	int ECMGraph::AddVertex(ECMVertex vertex)
	{
		int index = m_NextVertexIndex;

		vertex.SetIndex(index);
		m_Vertices.push_back(vertex);

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

	const std::vector<int> ECMGraph::GetNeighboringVertices(int vertex_index) const
	{
		std::vector<int> result;

		if (vertex_index >= m_VertAdjacency.size())
		{
			return result;
		}

		const std::vector<EdgeIndex>& edges = m_VertAdjacency[vertex_index];
		for (const EdgeIndex& edge : edges)
		{
			const ECMEdge& ecmEdge = GetEdge(edge);
			int neighbor = ecmEdge.V0() == vertex_index ? ecmEdge.V1() : ecmEdge.V0();

			result.push_back(neighbor);
		}

		return result;
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