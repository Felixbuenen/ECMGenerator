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

	bool ECM::RetractPoint(Point location, ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge, float clearance)
	{
		outEdge = *m_EcmGraph.GetEdge(cell.ecmEdge);
		const ECMVertex* p1 = m_EcmGraph.GetVertex(outEdge.half_edges[1].v_target_idx);
		const ECMVertex* p2 = m_EcmGraph.GetVertex(outEdge.half_edges[0].v_target_idx);

		// get closest obstacle based on which side of the ECM edge the location is
		Point obstacleP1, obstacleP2;
		Vec2 rayDir;
		int halfEdgeToP2Idx = outEdge.half_edges[0].v_target_idx == p2->idx ? 0 : 1;
		int halfEdgeToP1Idx = (halfEdgeToP2Idx + 1) % 2;

		if (Utility::MathUtility::IsLeftOfSegment(Segment(p1->position, p2->position), location))
		{
			obstacleP1 = outEdge.half_edges[halfEdgeToP2Idx].closest_left;
			obstacleP2 = outEdge.half_edges[halfEdgeToP1Idx].closest_right;

			//point obstacle
			if (obstacleP1.Approximate(obstacleP2))
			{
				Vec2 v1 = p1->position - obstacleP1;
				Vec2 v2 = p2->position - obstacleP1;
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
			obstacleP1 = outEdge.half_edges[halfEdgeToP2Idx].closest_right;
			obstacleP2 = outEdge.half_edges[halfEdgeToP1Idx].closest_left;
			//point obstacle
			if (obstacleP1.Approximate(obstacleP2))
			{
				Vec2 v1 = p1->position - obstacleP1;
				Vec2 v2 = p2->position - obstacleP1;
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

		// first check if enough clearance from obstacle
		float distFromObstacle;
		Point obstacleIntersect;
		Vec2 invDir(-rayDir.x, -rayDir.y);
		if (Utility::MathUtility::GetRayToLineSegmentIntersection(location, invDir, obstacleP1, obstacleP2, obstacleIntersect, distFromObstacle))
		{
			if (distFromObstacle < clearance)
			{
				printf("ERROR: not enough clearance from obstacle.\n");
				return false;
			}
		}

		float outDist;
		return Utility::MathUtility::GetRayToLineSegmentIntersection(location, rayDir, p1->position, p2->position, outRetractedLocation, outDist);
	}


	// ECM Graph

	ECMGraph::ECMGraph() : m_NextVertexIndex(0), m_NextEdgeIndex(0)
	{
		m_Cells = std::make_unique<ECMCellCollection>();
	}

	ECMVertex* ECMGraph::AddVertex(Point position)
	{
		int index = m_NextVertexIndex;

		// TODO: can we make an assumption based on the environment how many edges/vertices there will be?
		m_Vertices.push_back(ECMVertex());
		m_Vertices[index].idx = index;
		m_Vertices[index].position = position;

		m_NextVertexIndex++;

		return &m_Vertices[index];
	}

	ECMEdge* ECMGraph::AddEdge()
	{
		int index = m_NextEdgeIndex;

		// TODO: can we make an assumption based on the environment how many edges/vertices there will be?
		m_Edges.push_back(ECMEdge());
		m_Edges[index].idx = index;

		m_NextEdgeIndex++;

		return &m_Edges[index];
	}

	ECMHalfEdge* ECMGraph::AddHalfEdge(int edgeIdx, int targetIdx, Point closestLeft, Point closestRight, short idx)
	{
		if (idx != 0 && idx != 1) {
			printf("ERROR in AddHalfEdge: idx represents the local half-edge index for the edge and should be 0 or 1");
			return nullptr;
		}

		ECMEdge& edge = m_Edges[edgeIdx];
		edge.half_edges[idx].closest_left = closestLeft;
		edge.half_edges[idx].closest_right = closestRight;
		edge.half_edges[idx].v_target_idx = targetIdx;

		return &(edge.half_edges[idx]);
	}


	void ECMGraph::ConstructECMCells()
	{
		m_Cells->Construct(*this);
	}

	// TODO: make KD-tree query (?)
	int ECMGraph::FindVertex(float x, float y) const
	{
		using Utility::EPSILON;

		int index = 0;
		for (const ECMVertex& vert : m_Vertices)
		{
			const Point& pos = vert.position;
			bool isSamePosition = abs(x - vert.position.x) < EPSILON && abs(y - vert.position.y) < EPSILON;

			if (isSamePosition)
			{
				return index;
			}

			index++;
		}

		// not found
		return -1;
	}


	ECMCell* ECMGraph::FindCell(float x, float y)
	{
		return m_Cells->PointLocationQuery(Point(x, y));
	}

	bool ECMGraph::IsArc(const ECMEdge& edge, int& outPtLeftOfIdx) const
	{
		bool leftBoundIsPoint = edge.half_edges[0].closest_left == edge.half_edges[1].closest_right;
		bool rightBoundIsPoint = edge.half_edges[0].closest_right == edge.half_edges[1].closest_left;

		if (leftBoundIsPoint) outPtLeftOfIdx = 0;
		else if (rightBoundIsPoint) outPtLeftOfIdx = 1;
		else outPtLeftOfIdx = -1;

		// the edge is only an arc if either one of the two boundaries is a point
		return (leftBoundIsPoint || rightBoundIsPoint) && (leftBoundIsPoint != rightBoundIsPoint);
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

	ECMCell* ECM::GetECMCell(float x, float y)
	{
		return m_EcmGraph.FindCell(x, y);
	}


}