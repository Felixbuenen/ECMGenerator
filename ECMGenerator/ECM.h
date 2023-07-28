#pragma once

#include <memory>

#include "ECMDataTypes.h"

#define INVALID_ECM_VERTEX_INDEX -1
#define ECM_EPSILON 0.02f // TODO: decide what is a good epsilon
#define ECM_HALF_EPSILON ECM_EPSILON * 0.5f

class MedialAxis;
class Point;
class Segment;


class ECMEdge
{
public:
	ECMEdge(int start, int end, float cost) : m_StartVert(start), m_EndVert(end), m_Cost(cost) { }
	ECMEdge(int start, int end) : m_StartVert(start), m_EndVert(end), m_Cost(1.0f) { }
	ECMEdge() : m_StartVert(INVALID_ECM_VERTEX_INDEX), m_EndVert(INVALID_ECM_VERTEX_INDEX), m_Cost(1.0f) { }

	inline int StartIndex() const { return m_StartVert; }
	inline int EndIndex() const { return m_EndVert; }
	inline float Cost() const { return m_Cost; }
	inline Point NearestLeftV0() const { return m_Nearest_left_V0; }
	inline Point NearestRightV0() const { return m_Nearest_right_V0; }
	inline Point NearestLeftV1() const { return m_Nearest_left_V1; }
	inline Point NearestRightV1() const { return m_Nearest_right_V1; }

	void SetVertices(int v1, int v2);
	void SetCost(float cost);
	void SetNearestLeftV0(Point p) { m_Nearest_left_V0 = p; }
	void SetNearestRightV0(Point p) { m_Nearest_right_V0 = p; }
	void SetNearestLeftV1(Point p) { m_Nearest_left_V1 = p; }
	void SetNearestRightV1(Point p) { m_Nearest_right_V1 = p; }

private:
	int m_StartVert;
	int m_EndVert;
	float m_Cost;

	// these are the four closest points to adjacent obstacle. Essentially, this - along with the edge - defines the corridor.
	Point m_Nearest_left_V0;
	Point m_Nearest_left_V1;
	Point m_Nearest_right_V0;
	Point m_Nearest_right_V1;
};

class ECMVertex
{
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
	void AddIncidentEdge(int edge) { m_IncidentEdges.push_back(edge); }

private:
	int m_Index;
	Point m_Position;
	float m_Clearance;
	std::vector<Point> m_ClosestPoints;
	std::vector<int> m_IncidentEdges;
};


// for now, let's assume that an ECM graph is always bidirectional. Note that this doesn't have to be the case (e.g. cars that must travel in one direction on a lane).
// the ECMGraph implements the typical edge adjacency list structure
class ECMGraph
{
public:
	typedef int EdgeIndex;

	ECMGraph() : m_NextVertexIndex(0), m_NextEdgeIndex(0) { }

	// NOTE: it's nicer to use the addvertex/addedge methods and build the graph in the ecm generator class
	// so we don't need this construct method here
	//int Construct(); // provide environment and voronoi diagram to construct ecm graph

	inline int GetNextFreeVertexIndex() const { return m_NextVertexIndex; }
	inline int GetNextFreeEdgeIndex() const { return m_NextEdgeIndex; }
	
	int AddVertex(ECMVertex vertex);
	void RemoveVertex(int vertex);

	int AddEdge(ECMEdge edge);
	void RemoveEdge(int start, int end);

	void AddAdjacency(int vertexIndex, int edgeIndex);

	int GetVertexIndex(float x, float y) const;
	const std::vector<ECMVertex>& GetVertices() const { return m_Vertices; }
	const std::vector<ECMEdge>& GetEdges() const { return m_Edges; }
	const ECMVertex& GetVertex(int idx) const { return m_Vertices[idx]; }
	const ECMEdge& GetEdge(int idx) const { return m_Edges[idx]; }

	// ------------- TESTING -------------
	std::vector<Segment> GetRandomTestPath(int startIndex) const;

private:
	std::vector<ECMVertex> m_Vertices; // TODO: make KD-tree
	std::vector<ECMEdge> m_Edges; // TODO: make KD-tree

	//std::vector<std::vector<EdgeIndex>> m_VertAdjacency;

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
	void GetRetractionPoint() const;
	void GetStartAndGoalRetractionPoint() const; // maybe for optimization
	void GetECMBounds(/*point refs*/) const;
	void GetWalkableArea(/*point refs*/) const;
	void GetObstacles(/*point refs*/) const;
	std::vector<Segment> GetECMCell(float x, float y) const;

	inline std::shared_ptr<MedialAxis> GetMedialAxis() { return _medialAxis; }
	inline ECMGraph& GetECMGraph() { return _ecmGraph; }

	// ------- TESTING -----------
	std::vector<Segment> GetRandomTestPath() const;

private:
	/* datastructure, kd-tree, boost data.. */
	std::shared_ptr<MedialAxis> _medialAxis; // waarom is dit een shared ptr en niet gewoon een obj?
	ECMGraph _ecmGraph;
};