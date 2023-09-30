#pragma once

#include <memory>
#include <limits>

#include "ECMDataTypes.h"
#include "ECMCellCollection.h"


namespace ECM {

	class ECM;
	class ECMVertex;
	class ECMCellCollection;
	
	struct ECMCell;

	typedef int EdgeIndex;

	struct ECMHalfEdge
	{
		int next_idx = -1;
		int v_target_idx;
		Point closest_left;
		Point closest_right;
	};

	struct ECMEdge
	{
		int idx;
		ECMHalfEdge half_edges[2];

		bool operator==(const ECMEdge& other) const
		{
			return idx == other.idx;
		}
	};

	struct ECMVertex
	{
		int idx;
		int half_edge_idx; // index of incident half edge
		float clearance;
		Point position;

		bool operator==(const ECMVertex& other) const
		{
			return idx == other.idx;
		}
	};

	class ECMGraph
	{
	public:

		ECMGraph();

		// getters
		inline ECMVertex* GetVertex(int idx) { return &(m_Vertices[idx]); }
		inline ECMEdge* GetEdge(int idx) { return &(m_Edges[idx]); }
		inline ECMHalfEdge* GetHalfEdge(int idx) { return &(m_Edges[idx >> 1].half_edges[idx & 1]); };

		inline ECMVertex* GetSource(ECMHalfEdge* halfEdge)
		{
			// get the byte position of the half edge pointer in memory (relative to the start of the edge list)
			const char* firstEdge = reinterpret_cast<const char*>(&m_Edges[0]);
			const char* thisEdge = reinterpret_cast<const char*>(halfEdge);
			int diff = int(thisEdge - firstEdge);

			// get the edge by dividing this relative byte position to the size of ECMEdge. this way you get a pointer to the edge
			//  that holds the half edge
			ECMEdge* edge = &m_Edges[0] + diff / sizeof(ECMEdge);
			int sourceIdx = &edge->half_edges[0] == halfEdge ? edge->half_edges[1].v_target_idx : edge->half_edges[0].v_target_idx;

			return GetVertex(sourceIdx);
		}

		inline std::vector<ECMVertex>& GetVertices() { return m_Vertices; }
		inline std::vector<ECMEdge>& GetEdges() { return m_Edges; }

		// queries
		int FindVertex(float x, float y) const;
		ECMCell* FindCell(float x, float y);
		bool IsArc(const ECMEdge&, int& outPtLeftOfIdx) const;

		// construction methods
		ECMVertex* AddVertex(Point position);
		ECMEdge* AddEdge();
		ECMHalfEdge* AddHalfEdge(int edgeIdx, int targetIdx, Point closestLeftV1, Point closestRightV1, short idx);

		void ConstructECMCells();

	private:
		std::vector<ECMVertex> m_Vertices;
		std::vector<ECMEdge> m_Edges;
		std::unique_ptr<ECMCellCollection> m_Cells;

		int m_NextVertexIndex;
		int m_NextEdgeIndex;
	};

	class ECM
	{
		// contains ECM (generation) logic and ECM model

	public:
		ECM();

		// querying
		ECMCell* GetECMCell(float x, float y);
		bool RetractPoint(Point location, ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge, float clearance);

		inline std::shared_ptr<MedialAxis> GetMedialAxis() { return m_MedialAxis; }
		inline ECMGraph& GetECMGraph() { return m_EcmGraph; }

		// ------- TESTING -----------
		std::vector<Segment> GetRandomTestPath() const;

	private:
		std::shared_ptr<MedialAxis> m_MedialAxis; // TODO: waarom is dit een shared ptr en niet gewoon een obj?
		ECMGraph m_EcmGraph;
	};

}