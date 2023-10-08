#include "ECMPathPlanner.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECM.h"
#include "AStar.h"
#include "UtilityFunctions.h"
#include "Timer.h"

namespace ECM {
	namespace PathPlanning {

		ECMPathPlanner::ECMPathPlanner(ECMGraph& graph) : m_AStar(AStar(graph)) 
		{
			m_AStar.Initialize();
		}

		ECMPathPlanner::~ECMPathPlanner()
		{
		}

		bool ECMPathPlanner::GetPath(const Environment& environment, Point start, Point goal, float clearance, float preferredAdditionalClearance, Corridor& outCorridor, std::vector<Segment>& outPortals, Path& outPath)
		{
			clearance += preferredAdditionalClearance;
			
			//Timer timer("ECMPathPlanner::GetPath");

			// 1. query the cell location of the start / goal position.
			auto ecmStart = environment.QueryECM(start);
			auto ecmGoal = environment.QueryECM(goal);
			
			// for now we require that we only have 1 ECM in our environment
			if (ecmStart != ecmGoal)
			{
				return false;
			}

			//printf("query start/goal locations...\n");
			auto ecm = ecmStart;
			ECMCell* startCell;
			ECMCell* goalCell;

			{
				//Timer findCellTime("Find cell...");
				startCell = ecm->GetECMGraph().FindCell(start.x, start.y);
				goalCell = ecm->GetECMGraph().FindCell(goal.x, goal.y);
			}

			if (!startCell || !goalCell)
			{
				//printf("PathPlanning error: Could not find the cell of start and/or goal position.\n");
				return false;
			}
			
			// 2. retract the start / goal position on the ECM graph
			
			//printf("retract points...\n");
			Point retrStart, retrGoal;
			ECMEdge startEdge;
			ECMEdge goalEdge;
			if (!ecm->RetractPoint(start, *startCell, retrStart, startEdge, clearance))
			{
				//printf("retraction for start point failed");
				return false;
			}
			if (!ecm->RetractPoint(goal, *goalCell, retrGoal, goalEdge, clearance))
			{
				//printf("retraction for end point failed");
				return false;
			}

			// if start and goal in the same corridor, simply return a straight line path
			// we do this after retraction, because if retraction fails, we cannot create a path (e.g. due to not sufficient clearance from obstacle)
			if (startEdge.idx == goalEdge.idx)
			{
				outPath.push_back(start);
				outPath.push_back(goal);

				return true;
			}

			//printf("find a star path...\n");
			// 3. Plan a path on the medial axis using the clearance and A*.
			std::vector<int> astarPath;
			if(!m_AStar.FindPath(retrStart, retrGoal, &startEdge, &goalEdge, clearance, astarPath))
			{
				//printf("couldn't find path!\n");
				return false;
			}


			//printf("generate edge path...\n");
			std::vector<ECMHalfEdge*> edgePath;

			{
				//Timer timer("generate edge path..");
				for (int i = 0; i < astarPath.size() - 1; i++)
				{
					int index1 = astarPath[i];
					int index2 = astarPath[i + 1];
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
			}

			//printf("create and shrink corridor...\n");
			// 4. Create and shrink the corridor
			CreateCorridor(edgePath, outCorridor, ecm);
			ShrinkCorridor(outCorridor, clearance);

			//printf("triangulate corridor...\n");
			// 5. Triangulate the ECM cells through which the path goes.
			TriangulateCorridor(start, goal, outCorridor, outPortals, clearance);

			//printf("smooth path...\n");
			// 7. Use the funnel algorithm to generate a path over these triangles
			std::vector<Point> shortestPath;
			Funnel(outPortals, start, goal, shortestPath);
			
			//printf("done\n");
			for (const auto& p : shortestPath)
			{
				outPath.push_back(p);
			}

			return true;
		}

		bool ECMPathPlanner::ValidStartGoalLocation(const Point& start, const Point& goal, const ECMEdge& startEdge, const ECMEdge& goalEdge, float clearance) const
		{


			return false;
		}


		void ECMPathPlanner::CreateCorridor(const std::vector<ECMHalfEdge*>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm)
		{
			//Timer timer("ECMPathPlanner::CreateCorridor");
			auto& graph = ecm->GetECMGraph();
			for (ECMHalfEdge* edge : maPath)
			{
				Point pos = graph.GetSource(edge)->position;
				outCorridor.diskCenters.push_back(pos);
				outCorridor.diskRadii.push_back(graph.GetSource(edge)->clearance);
				outCorridor.leftBounds.push_back(edge->closest_left);
				outCorridor.rightBounds.push_back(edge->closest_right);
			}
		}

		void ECMPathPlanner::ShrinkCorridor(Corridor& corridor, float clearance)
		{
			//Timer timer("ECMPathPlanner::ShrinkCorridor");

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

		void ECMPathPlanner::TriangulateCorridor(const Point& start, const Point& goal, const Corridor& corridor, std::vector<Segment>& outPortals, float clearance)
		{
			//Timer timer("ECMPathPlanner::TriangulateCorridor");

			// create the funnel portals by connecting all pairs of left and right bounds.
			// make sure to sample the arcs

			// then add all left/right closest obstacles (shrunk corridor vertices)
			for (int i = 0; i < corridor.diskCenters.size()-1; i++)
			{
				switch (corridor.curveTypes[i])
				{
				case(CorridorBoundCurve::LINEAR):
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

			// make sure the range of portals is correct. remove portals that fall outside of the actual corridor.
			FitPortalRange(outPortals, start, goal);

			// add the goal point as the last portal. This is required for the path smoothing algorithm.
			outPortals.push_back(Segment(goal, goal));
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
		}

		void ECMPathPlanner::FitPortalRange(std::vector<Segment>& portals, const Point& start, const Point& goal)
		{
			//Timer timer("ECMPathPlanner::FitPortalRange");

			// start portal
			int first;
			for (int i = 0; i < portals.size(); i++)
			{
				const Segment& portal = portals[i];
				if (!Utility::MathUtility::IsLeftOfSegment(portal, start))
				{
					first = i;
					break;
				}
			}

			// remove unnecessary begin portals
			for (int i = 0; i < first; i++)
			{
				portals.erase(portals.begin());
			}

			// end portal
			int last;
			for (int i = portals.size() - 1; i >= 0; i--)
			{
				const Segment& portal = portals[i];
				if (Utility::MathUtility::IsLeftOfSegment(portal, goal))
				{
					last = i;
					break;
				}
			}

			int toRemove = portals.size() - last;
			// remove unnecessary last portals
			for (int i = 0; i < toRemove; i++)
			{
				portals.pop_back();
			}
		}

		// Reference: https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
		void ECMPathPlanner::Funnel(const std::vector<Segment>& portals, const Point& start, const Point& goal, std::vector<Point>& outShortestPath)
		{
			//Timer timer("ECMPathPlanner::Funnel");

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
			for (int i = 0; i < portals.size(); i++)
			{
				Point left = portals[i].p0;
				Point right = portals[i].p1;

				// 1) first update RIGHT
				// first check: does this right point make the current funnel (portalApex, portalRight, right) wider? Then skip right for this iteration...
				if (Utility::MathUtility::TriangleArea(portalApex, portalRight, right) <= 0.0f)
				{
					if (portalApex.Approximate(portalRight) || Utility::MathUtility::TriangleArea(portalApex, portalLeft, right) > 0.0f)
					{
						portalRight = right;
						rightIdx = i;
					}
					else
					{
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
					if (portalApex.Approximate(portalLeft) || Utility::MathUtility::TriangleArea(portalApex, portalRight, left) < 0.0f)
					{
						portalLeft = left;
						leftIdx = i;
					}
					else
					{
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
			if (!outShortestPath.back().Approximate(goal))
			{
				outShortestPath.push_back(goal);
			}
		}

	}
}
