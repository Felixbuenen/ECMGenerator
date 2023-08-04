#pragma once

#include <memory>
#include <limits>

#include "ECMDataTypes.h"
#include "ECMCellCollection.h"


namespace ECM {

	class ECM;
	class ECMCellCollection;
	
	struct ECMCell;

	typedef int EdgeIndex;


	class ECMVertex
	{
	public:
		static const int INVALID_ECM_VERTEX_INDEX = -1;

	public:
		ECMVertex(float x, float y) : m_Position(Point(x, y)) { }
		ECMVertex(Point point) : m_Position(point) { }
		ECMVertex() { }

		inline int Index() const { return m_Index; }
		inline Point Position() const { return m_Position; }
		inline float Clearance() const { return m_Clearance; }
		inline std::vector<Point> GetClosestPoints() const { return m_ClosestPoints; }

		void SetIndex(int index) { m_Index = index; }
		void SetPosition(Point position) { m_Position = position; }
		void SetClearance(float clearance) { m_Clearance = clearance; }
		void AddClosestObstaclePoint(Point point) { m_ClosestPoints.push_back(point); }

	private:
		int m_Index;
		Point m_Position;
		float m_Clearance;
		std::vector<Point> m_ClosestPoints;
	};


	// An ECMEdge is bi-directional, so we have 1 edge between two ECM vertices. 
	class ECMEdge
	{
	public:
		ECMEdge(int v0, int v1, float cost) : m_V0(v0), m_V1(v1), m_Cost(cost) { }
		ECMEdge(int v0, int v1) : m_V0(v0), m_V1(v1), m_Cost(1.0f) { }
		ECMEdge() : m_V0(ECMVertex::INVALID_ECM_VERTEX_INDEX), m_V1(ECMVertex::INVALID_ECM_VERTEX_INDEX), m_Cost(1.0f) { }

		inline int V0() const { return m_V0; }
		inline int V1() const { return m_V1; }
		inline int Index() const { return m_Index; }
		inline float Cost() const { return m_Cost; }
		inline bool IsArc() const { return m_IsArc; }
		inline Point NearestLeftV0() const { return m_Nearest_left_V0; }
		inline Point NearestRightV0() const { return m_Nearest_right_V0; }
		inline Point NearestLeftV1() const { return m_Nearest_left_V1; }
		inline Point NearestRightV1() const { return m_Nearest_right_V1; }

		void SetVertices(int v1, int v2);
		void SetIndex(int idx) { m_Index = idx; }
		void SetCost(float cost);
		void SetNearestLeftV0(Point p) { m_Nearest_left_V0 = p; }
		void SetNearestRightV0(Point p) { m_Nearest_right_V0 = p; }
		void SetNearestLeftV1(Point p) { m_Nearest_left_V1 = p; }
		void SetNearestRightV1(Point p) { m_Nearest_right_V1 = p; }
		void SetIsArc(bool isArc) { m_IsArc = isArc; }

	private:
		int m_V0;
		int m_V1;
		int m_Index;
		float m_Cost;

		bool m_IsArc;

		// these are the four closest points to adjacent obstacle. Essentially, this - along with the edge - defines the corridor.
		Point m_Nearest_left_V0;
		Point m_Nearest_left_V1;
		Point m_Nearest_right_V0;
		Point m_Nearest_right_V1;
	};


	// the ECMGraph implements the typical edge adjacency list structure, as a bi-directional graph
	class ECMGraph
	{
	public:

		ECMGraph();

		// NOTE: it's nicer to use the addvertex/addedge methods and build the graph in the ecm generator class
		// so we don't need this construct method here

		inline int GetNextFreeVertexIndex() const { return m_NextVertexIndex; }
		inline int GetNextFreeEdgeIndex() const { return m_NextEdgeIndex; }

		int AddVertex(ECMVertex vertex);
		void RemoveVertex(int vertex);

		int AddEdge(ECMEdge edge);
		void RemoveEdge(int start, int end);

		void ConstructECMCells();

		void AddAdjacency(int v0, int v1, int edge);

		int GetVertexIndex(float x, float y) const;
		const std::vector<ECMVertex>& GetVertices() const { return m_Vertices; }
		const std::vector<ECMEdge>& GetEdges() const { return m_Edges; }
		const ECMVertex& GetVertex(int idx) const { return m_Vertices[idx]; }
		const ECMEdge& GetEdge(int idx) const { return m_Edges[idx]; }
		const std::vector<EdgeIndex>& GetIncidentEdges(int vertex_index) const;
		const ECMCell* GetCell(float x, float y) const;
		std::vector<Segment> GetSampledEdge(const ECMEdge& edge, int samples = 10, bool inverseDirection = false) const;

		// ------------- TESTING -------------
		std::vector<Segment> GetRandomTestPath(int startIndex) const;

	private:
		std::vector<ECMVertex> m_Vertices; // TODO: make KD-tree
		std::vector<ECMEdge> m_Edges; // TODO: make KD-tree
		std::unique_ptr<ECMCellCollection> m_Cells;

		std::vector<std::vector<EdgeIndex>> m_VertAdjacency;

		int m_NextVertexIndex;
		int m_NextEdgeIndex;
	};


	class ECM
	{
		// contains ECM (generation) logic and ECM model

	public:
		ECM();

		void AddObstacle();
		void RemoveObstacle();
		void Clear();

		// querying
		void GetECMBounds(/*point refs*/) const;
		void GetWalkableArea(/*point refs*/) const;
		void GetObstacles(/*point refs*/) const;
		const ECMCell* GetECMCell(float x, float y) const; // pointer because it can be null
		const bool RetractPoint(Point location, Point& outRetractedLocation, ECMEdge& outEdge) const;
		const bool RetractPoint(Point location, const ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge) const;

		inline std::shared_ptr<MedialAxis> GetMedialAxis() { return m_MedialAxis; }
		inline ECMGraph& GetECMGraph() { return m_EcmGraph; }

		// ------- TESTING -----------
		std::vector<Segment> GetRandomTestPath() const;

	private:
		/* datastructure, kd-tree, boost data.. */
		std::shared_ptr<MedialAxis> m_MedialAxis; // waarom is dit een shared ptr en niet gewoon een obj?
		ECMGraph m_EcmGraph;
	};

}