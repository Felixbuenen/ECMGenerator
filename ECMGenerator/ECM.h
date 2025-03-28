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
		friend class ECMGenerator;
		friend class ECM;

	public:

		ECMGraph();

		void Clear();

		// getters
		inline const ECMVertex* GetVertex(int idx) const 
		{
			return &(m_Vertices[idx]); 
		}
		inline ECMEdge* GetEdge(int idx) { return &(m_Edges[idx]); } // TODO: make const
		inline const ECMHalfEdge* GetHalfEdge(int idx) const { return &(m_Edges[idx >> 1].half_edges[idx & 1]); };

		inline const ECMVertex* GetSource(const ECMHalfEdge* halfEdge) const
		{
			// get the byte position of the half edge pointer in memory (relative to the start of the edge list)
			const char* firstEdge = reinterpret_cast<const char*>(&m_Edges[0]);
			const char* thisEdge = reinterpret_cast<const char*>(halfEdge);
			int diff = int(thisEdge - firstEdge);

			// get the edge by dividing this relative byte position to the size of ECMEdge. this way you get a pointer to the edge
			//  that holds the half edge
			const ECMEdge* edge = &m_Edges[0] + diff / sizeof(ECMEdge);
			int sourceIdx = &edge->half_edges[0] == halfEdge ? edge->half_edges[1].v_target_idx : edge->half_edges[0].v_target_idx;

			return GetVertex(sourceIdx);
		}

		inline std::vector<ECMVertex>& GetVertices() { return m_Vertices; }
		inline std::vector<ECMEdge>& GetEdges() { return m_Edges; }

		// DEBUG
		ECMCellCollection* GetCells();
		// DEBUG

		// queries
		int FindVertex(float x, float y) const;
		const ECMCell* FindCell(float x, float y) const;
		bool IsArc(const ECMEdge&, int& outPtLeftOfIdx) const;

		// construction methods
		ECMVertex* AddVertex(Point position);
		ECMEdge* AddEdge();
		ECMHalfEdge* AddHalfEdge(int edgeIdx, int targetIdx, Point closestLeftV1, Point closestRightV1, short idx);

		void ConstructECMCells();

	private:
		void SetVertexClearance(int vertID, float clearance) { m_Vertices[vertID].clearance = clearance; }
		void SetVertexHalfEdge(int vertID, int halfEdge) { m_Vertices[vertID].half_edge_idx = halfEdge; }
		void SetNextEdge(int edgeID, int offset, int nextEdge) { m_Edges[edgeID].half_edges[offset].next_idx = nextEdge; }

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
		void Clear();

		// querying
		const ECMCell* GetECMCell(float x, float y) const;
		bool RetractPoint(Point location, Point& outRetractedLocation, ECMEdge& outEdge) const;

		inline MedialAxis* GetMedialAxis() { return &m_MedialAxis; }
		inline ECMGraph& GetECMGraph() { return m_EcmGraph; }

		// ------- TESTING -----------
		std::vector<Segment> GetRandomTestPath() const;

	private:
		MedialAxis m_MedialAxis;
		ECMGraph m_EcmGraph;
	};

}