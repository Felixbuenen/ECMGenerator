#include "ECMPathPlanner.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECM.h"
#include "AStar.h"
#include "UtilityFunctions.h"

namespace ECM {
	namespace PathPlanning {

		ECMPathPlanner::ECMPathPlanner() { }

		ECMPathPlanner::~ECMPathPlanner()
		{
			delete m_AStar;
		}

		bool ECMPathPlanner::Initialize(ECMGraph& graph)
		{
			m_AStar = new AStar(graph);
			m_AStar->Initialize();

			return true;
		}


		Path ECMPathPlanner::GetPath(const Environment& environment, Point start, Point goal, float clearance, Corridor& outCorridor, std::vector<Segment>& outPortals)
		{
			// TODO:
			// First check if start and goal are valid positions.
			
			Path result;

			// an ECM path is planned as follows:
			// 1. query the cell location of the start / goal position.
			auto ecmStart = environment.QueryECM(start);
			auto ecmGoal = environment.QueryECM(goal);
			
			// for now we require that we only have 1 ECM in our environment
			if (ecmStart != ecmGoal)
			{
				return result;
			}

			auto ecm = ecmStart;
			auto startCell = ecm->GetECMGraph().FindCell(start.x, start.y);
			auto goalCell = ecm->GetECMGraph().FindCell(goal.x, goal.y);

			if (!startCell || !goalCell)
			{
				printf("PathPlanning error: Could not find the cell of start and/or goal position.\n");
				return result;
			}
			
			// 2. retract the start / goal position on the ECM graph
			int startEdgeIdx = startCell->ecmEdge;
			int goalEdgeIdx = goalCell->ecmEdge;

			// if start and goal in the same corridor, simply return a straight line path
			if (startEdgeIdx == goalEdgeIdx)
			{
				result.push_back(start);
				result.push_back(goal);

				return result;
			}

			Point retrStart, retrGoal;
			ECMEdge startEdge;
			ECMEdge goalEdge;
			if (!ecm->RetractPoint(start, *startCell, retrStart, startEdge))
			{
				printf("retraction for start point failed");
			}
			if (!ecm->RetractPoint(goal, *goalCell, retrGoal, goalEdge))
			{
				printf("retraction for end point failed");
			}

			// 3. Plan a path on the medial axis using the clearance and A*.
			std::vector<int> astarPath;
			if(!m_AStar->FindPath(retrStart, retrGoal, &startEdge, &goalEdge, 1.0f, astarPath))
			{
				printf("couldn't find path!\n");
			}

			std::vector<ECMHalfEdge*> edgePath;
			for (int i = 0; i < astarPath.size() - 1; i++)
			{
				int index1 = astarPath[i];
				int index2 = astarPath[i+1];
				ECMVertex* v = ecm->GetECMGraph().GetVertex(index1);
				ECMHalfEdge* incEdge = ecm->GetECMGraph().GetHalfEdge(v->half_edge_idx);

				ECMHalfEdge* incEdgeStart = incEdge;
				do {
					if (incEdge->v_target_idx == index2) {
						edgePath.push_back(incEdge);
						break;
					}
					incEdge = ecm->GetECMGraph().GetHalfEdge(incEdge->next_idx);
				} while (incEdgeStart != incEdge);
			}

			for (const auto& e : edgePath)
			{
				printf("closest left: (%f, %f). closest right: (%f, %f)\n", e->closest_left.x, e->closest_left.y, e->closest_right.x, e->closest_right.y);
			}

			// DEBUG
			//result.push_back(start);
			//result.push_back(retrStart);
			//for (int i = 0; i < edgePath.size(); i++)
			//{
			//	result.push_back(ecm->GetECMGraph().GetVertex(edgePath[i]->v_target_idx)->position);
			//}
			//result.push_back(retrGoal);
			//result.push_back(goal);
			//return result;

			// 4. Create and shrink the corridor
			CreateCorridor(edgePath, outCorridor, ecm);
			ShrinkCorridor(outCorridor, clearance);

			// 5. Triangulate the ECM cells through which the path goes.
			TriangulateCorridor(outCorridor, outPortals, clearance);

			// 7. Use the funnel algorithm to generate a path over these triangles
			int firstPortal, lastPortal;
			FindFirstAndLastPortal(outPortals, start, goal, firstPortal, lastPortal);
			std::vector<Point> shortestPath;
			Funnel(outPortals, firstPortal, lastPortal, start, goal, shortestPath);
			

			for (const auto& p : shortestPath)
			{
				result.push_back(p);
			}


			return result;
		}

		// TODO: currently we're adding the half edges that contain the start and end position. I don't think this is necessary. E.g. we make triangles until we
		//  reach the final node. then we create a triangle using the goal position as end point.
		void ECMPathPlanner::CreateCorridor(const std::vector<ECMHalfEdge*>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm)
		{
			auto& graph = ecm->GetECMGraph();
			for (ECMHalfEdge* edge : maPath)
			{
				Point pos = graph.GetSource(edge)->position;
				outCorridor.diskCenters.push_back(pos);
				//outCorridor.diskRadii.push_back(graph.GetSource(edge)->clearance);
				// TODO: use disk clearance
				outCorridor.diskRadii.push_back(Utility::MathUtility::Length(edge->closest_left - pos));
				outCorridor.leftBounds.push_back(edge->closest_left);
				outCorridor.rightBounds.push_back(edge->closest_right);
			}
		}

		void ECMPathPlanner::ShrinkCorridor(Corridor& corridor, float clearance)
		{
			for (int i = 0; i < corridor.diskCenters.size()-1; i++)
			{
				if (corridor.diskRadii[i] < clearance)
				{
					corridor.leftCorridorBounds.push_back(corridor.diskCenters[i]);
					corridor.rightCorridorBounds.push_back(corridor.diskCenters[i]);
				}
				else
				{
					Vec2 moveDirectionLeft = (corridor.diskCenters[i] - corridor.leftBounds[i]);
					Vec2 moveDirectionRight = (corridor.diskCenters[i] - corridor.rightBounds[i]);
					moveDirectionLeft.Normalize();
					moveDirectionRight.Normalize();

					corridor.leftCorridorBounds.push_back(corridor.leftBounds[i] + moveDirectionLeft * clearance);
					corridor.rightCorridorBounds.push_back(corridor.rightBounds[i] + moveDirectionRight * clearance);
				}


				if (corridor.leftBounds[i] == corridor.leftBounds[i + 1])
				{
					corridor.curveTypes.push_back(CorridorBoundCurve::LEFT_ARC);
				}
				else if (corridor.rightBounds[i] == corridor.rightBounds[i + 1])
				{
					corridor.curveTypes.push_back(CorridorBoundCurve::RIGHT_ARC);
				}
				else
				{
					corridor.curveTypes.push_back(CorridorBoundCurve::LINEAR);
				}
			}

			// add last point
			if (corridor.diskRadii.back() < clearance)
			{
				corridor.leftCorridorBounds.push_back(corridor.diskCenters.back());
				corridor.rightCorridorBounds.push_back(corridor.diskCenters.back());
			}

			else 
			{
				Vec2 moveDirectionLeft = (corridor.diskCenters.back() - corridor.leftBounds.back());
				Vec2 moveDirectionRight = (corridor.diskCenters.back() - corridor.rightBounds.back());
				moveDirectionLeft.Normalize();
				moveDirectionRight.Normalize();

				corridor.leftCorridorBounds.push_back(corridor.leftBounds.back() + moveDirectionLeft * clearance);
				corridor.rightCorridorBounds.push_back(corridor.rightBounds.back() + moveDirectionRight * clearance);
			}
		}

		void ECMPathPlanner::TriangulateCorridor(const Corridor& corridor, std::vector<Segment>& outPortals, float clearance)
		{
			// create the funnel portals by connecting all pairs of left and right bounds.
			// make sure to sample the arcs


			for (int i = 0; i < corridor.diskCenters.size()-1; i++)
			{
				switch (corridor.curveTypes[i])
				{
				case(CorridorBoundCurve::LINEAR):
					//outPortalsLeft.push_back(corridor.leftCorridorBounds[i]);
					//outPortalsLeft.push_back(corridor.leftCorridorBounds[i + 1]);
					//outPortalsRight.push_back(corridor.rightCorridorBounds[i]);
					outPortals.push_back(Segment(corridor.leftCorridorBounds[i], corridor.rightCorridorBounds[i]));
					outPortals.push_back(Segment(corridor.leftCorridorBounds[i + 1], corridor.rightCorridorBounds[i]));
					break;

				case(CorridorBoundCurve::LEFT_ARC):
					SampleCorridorArc(corridor.leftCorridorBounds[i], corridor.leftCorridorBounds[i + 1], corridor.rightCorridorBounds[i], corridor.rightCorridorBounds[i+1], corridor.leftBounds[i], clearance, true, outPortals);
					break;

				case(CorridorBoundCurve::RIGHT_ARC):
					SampleCorridorArc(corridor.rightCorridorBounds[i], corridor.rightCorridorBounds[i+1], corridor.leftCorridorBounds[i], corridor.leftCorridorBounds[i+1], corridor.rightBounds[i], clearance, false, outPortals);
					break;

				default: 
					break;
				}
			}

			// add final portal
			outPortals.push_back(Segment(corridor.leftCorridorBounds.back(), corridor.rightCorridorBounds.back()));
		}

		void ECMPathPlanner::SampleCorridorArc(const Point& p1, const Point& p2, const Point& o1, const Point& o2, const Point& c, float radius, bool leftArc, std::vector<Segment>& portals)
		{
			const float maxCurveSampleLength = 10.0f;
			if (leftArc) portals.push_back(Segment(p1, o1));
			else portals.push_back(Segment(o1, p1));

			float edgeLength = Utility::MathUtility::Length(p2 - p1);
			int numSamples = std::ceil(edgeLength / maxCurveSampleLength);
			float sampleLength = edgeLength / (float)numSamples;
			Vec2 edgeDirection = (p2 - p1) / edgeLength;
			
			for (int i = 0; i < numSamples; i++)
			{
				Point p = p1 + edgeDirection * sampleLength * i;
				Vec2 arcDirection = p - c;
				arcDirection.Normalize();
				
				p = c + arcDirection * radius;
				
				if (leftArc) portals.push_back(Segment(p, o2));
				else portals.push_back(Segment(o2, p));
			}

			// don't push back last edge segment -> this is sampled in next corridor
			// 
			//if (leftArc) portals.push_back(Segment(p2, o2));
			//else portals.push_back(Segment(o2, p2));
		}

		void ECMPathPlanner::FindFirstAndLastPortal(const std::vector<Segment>& portals, const Point& start, const Point& goal, int& outFirst, int& outLast)
		{
			// start portal
			outFirst = 0;
			for (int i = 0; i < portals.size(); i++)
			{
				const Segment& portal = portals[i];
				if (!Utility::MathUtility::IsLeftOfSegment(portal, start))
				{
					outFirst = i;
					break;
				}
			}

			// end portal
			outLast = portals.size() - 1;
			for (int i = portals.size() - 1; i >= 0; i--)
			{
				const Segment& portal = portals[i];
				if (Utility::MathUtility::IsLeftOfSegment(portal, goal))
				{
					outLast = i;
					break;
				}
			}
		}

		// Reference: https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
		void ECMPathPlanner::Funnel(const std::vector<Segment>& portals, int firstPortal, int lastPortal, const Point& start, const Point& goal, std::vector<Point>& outShortestPath)
		{
			std::printf("we're gonna work with %d portals\n", portals.size());
			for (const auto& p : portals)
			{
				std::printf("(%f, %f) -> (%f, %f)\n", p.p0.x, p.p0.y, p.p1.x, p.p1.y);
			}

			// init vectors: left, right, apex (= left)
			// init indices: left, right, apex
			Point portalLeft, portalRight, portalApex;
			int leftIdx = 0;
			int rightIdx = 0;
			int apexIdx = 0;

			portalApex = start;
			portalLeft = start;
			portalRight = start;

			int numPortals = portals.size();

			// add start point
			outShortestPath.push_back(start);
			
			// <begin loop, i=1 en i < numPortals>
			for (int i = firstPortal; i <= lastPortal; i++)
			{
				Point left = portals[i].p0;
				Point right = portals[i].p1;

				// 1) first update RIGHT
				// first check: does this right point make the current funnel (portalApex, portalRight, right) wider? Then skip right for this iteration...
				if (Utility::MathUtility::TriangleArea(portalApex, portalRight, right) <= 0.0f)
				{
					if (portalApex == portalRight || Utility::MathUtility::TriangleArea(portalApex, portalLeft, right) > 0.0f)
					{
						portalRight = right;
						rightIdx = i;
						std::printf("right: update right index\n");
					}
					else
					{
						std::printf("right: found new apex, reset\n");

						outShortestPath.push_back(portalLeft);

						// Make current left the new apex.
						portalApex = portalLeft;
						apexIdx = leftIdx;

						// Reset portal
						portalLeft = portalApex;
						portalRight = portalApex;
						leftIdx = apexIdx;
						rightIdx = apexIdx;

						// Restart scan
						i = apexIdx;
						continue;
					}
				}

				// 2) then update LEFT
				// of course the same checks apply
				if (Utility::MathUtility::TriangleArea(portalApex, portalLeft, left) >= 0.0f)
				{
					if (portalApex == portalLeft || Utility::MathUtility::TriangleArea(portalApex, portalRight, left) < 0.0f)
					{
						std::printf("left: update left index\n");

						portalLeft = left;
						leftIdx = i;
					}
					else
					{
						std::printf("left: found new apex, reset\n");

						outShortestPath.push_back(portalRight);

						// Make current right the new apex.
						portalApex = portalRight;
						apexIdx = rightIdx;

						// Reset portal
						portalLeft = portalApex;
						portalRight = portalApex;
						leftIdx = apexIdx;
						rightIdx = apexIdx;

						// Restart scan
						i = apexIdx;
						continue;
					}
				}
			}

			// append last point
			outShortestPath.push_back(goal);
		}

	}
}
