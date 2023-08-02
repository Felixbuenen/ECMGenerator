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

   // // Apply the linear transformation to move start point of the segment to
   // // the point with coordinates (0, 0) and the direction of the segment to
   // // coincide the positive direction of the x-axis.
   // CT segm_vec_x = cast(x(high(segment))) - cast(x(low(segment)));
   // CT segm_vec_y = cast(y(high(segment))) - cast(y(low(segment)));
   // CT sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;
   //
   // // Compute x-coordinates of the endpoints of the edge
   // // in the transformed space.
   // CT projection_start = sqr_segment_length *
   //     get_point_projection((*discretization)[0], segment);
   // CT projection_end = sqr_segment_length *
   //     get_point_projection((*discretization)[1], segment);
   //
   // // Compute parabola parameters in the transformed space.
   // // Parabola has next representation:
   // // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
   // CT point_vec_x = cast(x(point)) - cast(x(low(segment)));
   // CT point_vec_y = cast(y(point)) - cast(y(low(segment)));
   // CT rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
   // CT rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;
   //
   // // Save the last point.
   // Point<CT> last_point = (*discretization)[1];
   // discretization->pop_back();
   //
   // // Use stack to avoid recursion.
   // std::stack<CT> point_stack;
   // point_stack.push(projection_end);
   // CT cur_x = projection_start;
   // CT cur_y = parabola_y(cur_x, rot_x, rot_y);
   //
   // // Adjust max_dist parameter in the transformed space.
   // const CT max_dist_transformed = max_dist * max_dist * sqr_segment_length;
   // while (!point_stack.empty()) {
   //     CT new_x = point_stack.top();
   //     CT new_y = parabola_y(new_x, rot_x, rot_y);
   //
   //     // Compute coordinates of the point of the parabola that is
   //     // furthest from the current line segment.
   //     CT mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
   //     CT mid_y = parabola_y(mid_x, rot_x, rot_y);
   //
   //     // Compute maximum distance between the given parabolic arc
   //     // and line segment that discretize it.
   //     CT dist = (new_y - cur_y) * (mid_x - cur_x) -
   //         (new_x - cur_x) * (mid_y - cur_y);
   //     dist = dist * dist / ((new_y - cur_y) * (new_y - cur_y) +
   //         (new_x - cur_x) * (new_x - cur_x));
   //     if (dist <= max_dist_transformed) {
   //         // Distance between parabola and line segment is less than max_dist.
   //         point_stack.pop();
   //         CT inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) /
   //             sqr_segment_length + cast(x(low(segment)));
   //         CT inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) /
   //             sqr_segment_length + cast(y(low(segment)));
   //         discretization->push_back(Point<CT>(inter_x, inter_y));
   //         cur_x = new_x;
   //         cur_y = new_y;
   //     }
   //     else {
   //         point_stack.push(mid_x);
   //     }
   // }
   //
   // // Update last point.
   // discretization->back() = last_point;
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

const ECMCell* ECM::GetECMCell(float x, float y) const
{
	return _ecmGraph.GetCell(x, y);
}
