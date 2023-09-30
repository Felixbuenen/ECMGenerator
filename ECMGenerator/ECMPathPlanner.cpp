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


		bool ECMPathPlanner::GetPath(const Environment& environment, Point start, Point goal, float clearance, Corridor& outCorridor, std::vector<Segment>& outPortals, Path& outPath)
		{
			// TODO:
			// First check if start and goal are valid positions.
			//std::printf("Start: (%f, %f). End: (%f, %f).\n", start.x, start.y, goal.x, goal.y);

			// 1. query the cell location of the start / goal position.
			auto ecmStart = environment.QueryECM(start);
			auto ecmGoal = environment.QueryECM(goal);
			
			// for now we require that we only have 1 ECM in our environment
			if (ecmStart != ecmGoal)
			{
				return false;
			}

			printf("query start/goal locations...\n");
			auto ecm = ecmStart;
			auto startCell = ecm->GetECMGraph().FindCell(start.x, start.y);
			auto goalCell = ecm->GetECMGraph().FindCell(goal.x, goal.y);

			if (!startCell || !goalCell)
			{
				printf("PathPlanning error: Could not find the cell of start and/or goal position.\n");
				return false;
			}
			
			// 2. retract the start / goal position on the ECM graph
			int startEdgeIdx = startCell->ecmEdge;
			int goalEdgeIdx = goalCell->ecmEdge;

			// if start and goal in the same corridor, simply return a straight line path
			if (startEdgeIdx == goalEdgeIdx)
			{
				outPath.push_back(start);
				outPath.push_back(goal);

				return true;
			}

			printf("retract points...\n");
			Point retrStart, retrGoal;
			ECMEdge startEdge;
			ECMEdge goalEdge;
			if (!ecm->RetractPoint(start, *startCell, retrStart, startEdge))
			{
				printf("retraction for start point failed");
				return false;
			}
			if (!ecm->RetractPoint(goal, *goalCell, retrGoal, goalEdge))
			{
				printf("retraction for end point failed");
				return false;
			}

			printf("find a star path...\n");
			// 3. Plan a path on the medial axis using the clearance and A*.
			std::vector<int> astarPath;
			if(!m_AStar->FindPath(retrStart, retrGoal, &startEdge, &goalEdge, 1.0f, astarPath))
			{
				printf("couldn't find path!\n");
				return false;
			}

			printf("generate edge path...\n");
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

			printf("create and shrink corridor...\n");
			// 4. Create and shrink the corridor
			CreateCorridor(edgePath, outCorridor, ecm);
			ShrinkCorridor(outCorridor, clearance);

			printf("triangulate corridor...\n");
			// 5. Triangulate the ECM cells through which the path goes.
			TriangulateCorridor(start, goal, outCorridor, outPortals, clearance);

			printf("smooth path...\n");
			// 7. Use the funnel algorithm to generate a path over these triangles
			std::vector<Point> shortestPath;
			Funnel(outPortals, start, goal, shortestPath);
			
			printf("done\n");
			for (const auto& p : shortestPath)
			{
				outPath.push_back(p);
			}



			return true;
		}

		void ECMPathPlanner::CreateCorridor(const std::vector<ECMHalfEdge*>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm)
		{
			auto& graph = ecm->GetECMGraph();
			for (ECMHalfEdge* edge : maPath)
			{
				Point pos = graph.GetSource(edge)->position;
				outCorridor.diskCenters.push_back(pos);
				// TODO: use disk clearance
				//outCorridor.diskRadii.push_back(graph.GetSource(edge)->clearance);
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

		void ECMPathPlanner::TriangulateCorridor(const Point& start, const Point& goal, const Corridor& corridor, std::vector<Segment>& outPortals, float clearance)
		{
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

			// add the goal point as the last portal. This is required for the path smoothing algorithm.
			outPortals.push_back(Segment(goal, goal));
		}

		void ECMPathPlanner::FindFirstAndLastPortal(const Point& start, const Point& goal, const std::vector<Segment>& portals, int& outFirst, int& outLast)
		{
			// find first portal
			for (int i = 0; i < portals.size(); i++)
			{
				if (!Utility::MathUtility::IsLeftOfSegment(portals[i], start))
				{
					outFirst = i;
					break;
				}
			}

			for (int i = portals.size()-1; i >= 0; i--)
			{
				if (Utility::MathUtility::IsLeftOfSegment(portals[i], start))
				{
					outLast = i;
					break;
				}
			}
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
		void ECMPathPlanner::Funnel(const std::vector<Segment>& portals, const Point& start, const Point& goal, std::vector<Point>& outShortestPath)
		{
			int firstPortal, lastPortal;
			FindFirstAndLastPortal(portals, start, goal, firstPortal, lastPortal);

			Point portalLeft, portalRight, portalApex;
			int leftIdx = firstPortal;
			int rightIdx = firstPortal;
			int apexIdx = firstPortal;

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
					// TODO: make equality function for points/vecs

					bool apexEqualToRight = (portalApex.x < portalRight.x + Utility::EPSILON) && (portalApex.x > portalRight.x - Utility::EPSILON) &&
						(portalApex.y < portalRight.y + Utility::EPSILON) && (portalApex.y > portalRight.y - Utility::EPSILON);
					if (apexEqualToRight || Utility::MathUtility::TriangleArea(portalApex, portalLeft, right) > 0.0f)
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
					// TODO: replace with function
					bool apexEqualToLeft = (portalApex.x < portalLeft.x + Utility::EPSILON && portalApex.x > portalLeft.x - Utility::EPSILON &&
						portalApex.y < portalLeft.y + Utility::EPSILON && portalApex.y > portalLeft.y - Utility::EPSILON);

					if (portalApex == portalLeft || Utility::MathUtility::TriangleArea(portalApex, portalRight, left) < 0.0f)
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
			outShortestPath.push_back(goal);
		}

	}
}
