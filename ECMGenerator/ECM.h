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

	// TODO:
	// Inspiratie opgedaan uit https://github.com/alexshafranov/corridormap/blob/master/source/build.cpp
	// De graaf bestaat uit een aantal ECM edges.
	// Iedere ECM edge heeft twee half-edges (1 voor iedere richting). half_edge[0]==links, half_edge[1]==rechts
	// Iedere half edge heeft een target vertex (source hebben we niet nodig).
	// Iedere half edge slaat de linker en rechter dichtsbijzijnde obstacle points op voor de target vertex.
	// Een ECMVertex heeft N half_edges -> de incident edges. Deze edges staan gesorteerd CCW. 
	//
	// Voor pathfinding gaan we eerst een lijst van ECM vertices retourneren (zoals we dat nu hebben).
	// De ECMEdge pad kunnen we vervolgens maken door voor iedere vertex te lopen door incident edges. Vergelijk de target
	//  met de volgende ECMvertex in de lijst. Als deze gelijk is hebben we de half edge gevonden.
	// Uiteindelijk retourneer je een lijst van half_edges. Hiermee heb je dus ook genoeg info voor het maken van corridor.
	//
	// Nu we toch de datastructuur gaan aanpassen, kunnen we alvast nadenken hoe we de structuur efficienter kunnen inrichten. Nu zijn de structuren
	//  vrij grote classes met methodes en veel variabelen. Beter kunnen we kleinere structs maken en deze onderverdelen in meerdere arrays met data.
	// Dit bevordert data locality.
	// > Pool<Vertex>. Vertex is een struct met vec2 positie, int index en int half_edge (index naar half edge)
	// > Pool<Edge>. Edge is een struct met index en half_edge[2]. 
	// > Half_edge is een struct met int next (idx volgende incident edge CCW), int target (idx van target vertex), vec2 posL en posR
	// Links en rechts informatie kan simpelweg met determinant berekend worden tijdens constructie. 
	// Dingen als 'isArc' hoeven we niet op te slaan. Dit is alleen relevant voor het maken van de corridor en is een simpele vergelijking. 
	// static functies kunnen allerlei memory operaties doen. bijvoorbeeld "next_incident" neemt een half edge en de edge memory en retourneert een
	//  pointer naar de volgende incident edge. Dit houdt de structs klein en overzichtelijk.
	// 
	// Als bovenstaande geimplementeerd is hebben we een structuur waar we niet voor het berekenen voor een pad nog eens left/right obstacles moeten
	//  berekenen. Alles is ook een stuk compacter. Dit is niet alleen netter, maar ook 

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
		std::vector<ECMVertex> m_Vertices; // TODO: maybe change to ECMVertex* array
		std::vector<ECMEdge> m_Edges;
		std::unique_ptr<ECMCellCollection> m_Cells;

		int m_NextVertexIndex;
		int m_NextEdgeIndex;
	};



	/*class ECMEdge
	{
	public:
		ECMEdge(ECMVertex* v0, ECMVertex* v1) : m_V0(v0), m_V1(v1), m_Cost(1.0f) { }

		inline ECMVertex* V0() const { return m_V0; }
		inline ECMVertex* V1() const { return m_V1; }
		inline ECMEdge* Next() { return m_Next; }
		inline ECMEdge* Prev() { return m_Prev; }
		inline ECMEdge* Twin() { return m_Twin; }

		inline int Index() const { return m_Index; }
		inline float Cost() const { return m_Cost; }
		inline bool IsArc() const { return m_IsArc; }
		inline Point NearestLeftV0() const { return m_Nearest_left_V0; }
		inline Point NearestRightV0() const { return m_Nearest_right_V0; }
		inline Point NearestLeftV1() const { return m_Nearest_left_V1; }
		inline Point NearestRightV1() const { return m_Nearest_right_V1; }

		void SetIndex(int idx) { m_Index = idx; }
		void SetCost(float cost) { m_Cost = cost; }
		void SetNext(ECMEdge* edge) { m_Next = edge; }
		void SetPrev(ECMEdge* edge) { m_Prev = edge; }
		void SetTwin(ECMEdge* edge) { m_Twin = edge; }

		void SetNearestLeftV0(Point p) { m_Nearest_left_V0 = p; }
		void SetNearestRightV0(Point p) { m_Nearest_right_V0 = p; }
		void SetNearestLeftV1(Point p) { m_Nearest_left_V1 = p; }
		void SetNearestRightV1(Point p) { m_Nearest_right_V1 = p; }
		void SetIsArc(bool isArc) { m_IsArc = isArc; }

	private:
		int m_Index;
		ECMVertex* m_V0;
		ECMVertex* m_V1;
		ECMEdge* m_Next;
		ECMEdge* m_Prev;
		ECMEdge* m_Twin;

		float m_Cost;
		bool m_IsArc;

		Point m_Nearest_left_V0;
		Point m_Nearest_left_V1;
		Point m_Nearest_right_V0;
		Point m_Nearest_right_V1;
	};


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
		std::vector<ECMEdge*> IncidentEdges() { return m_IncidentEdges; }
		inline std::vector<Point> GetClosestPoints() const { return m_ClosestPoints; } // do we need this?


		void SetIndex(int index) { m_Index = index; }
		void SetPosition(Point position) { m_Position = position; }
		void SetClearance(float clearance) { m_Clearance = clearance; }
		void AddClosestObstaclePoint(Point point) { m_ClosestPoints.push_back(point); }
		void AddIncidentEdge(ECMEdge* edge) { m_IncidentEdges.push_back(edge); }

		void SortIncidentEdges();

	private:
		int m_Index;
		Point m_Position;
		float m_Clearance;
		std::vector<ECMEdge*> m_IncidentEdges;
		std::vector<Point> m_ClosestPoints;
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
		ECMEdge* AddEdge(ECMEdge edge);

		void ConstructECMCells();

		int GetVertexIndex(float x, float y) const;
		std::vector<ECMVertex>& GetVertices() { return m_Vertices; }
		const std::vector<ECMEdge>& GetEdges() const { return m_Edges; }
		ECMVertex& GetVertex(int idx) { return m_Vertices[idx]; }
		ECMEdge& GetEdge(int idx) { return m_Edges[idx]; }
		const std::vector<EdgeIndex>& GetIncidentEdges(int vertex_index) const;
		const std::vector<int> GetNeighboringVertices(int vertex_index) const;
		const ECMCell* GetCell(float x, float y) const;
		std::vector<Segment> GetSampledEdge(const ECMEdge& edge, int samples = 10, bool inverseDirection = false) const;

	private:
		std::vector<ECMVertex> m_Vertices; // TODO: make KD-tree
		std::vector<ECMEdge> m_Edges; // TODO: make KD-tree
		std::unique_ptr<ECMCellCollection> m_Cells;

		std::vector<std::vector<EdgeIndex>> m_VertAdjacency;

		int m_NextVertexIndex;
		int m_NextEdgeIndex;
	};
	*/

	class ECM
	{
		// contains ECM (generation) logic and ECM model

	public:
		ECM();

		// querying
		ECMCell* GetECMCell(float x, float y);
		bool RetractPoint(Point location, Point& outRetractedLocation, ECMEdge* outEdge);
		bool RetractPoint(Point location, ECMCell& cell, Point& outRetractedLocation, ECMEdge& outEdge);

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