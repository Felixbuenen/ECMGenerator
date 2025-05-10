#include "IRMPathFollower.h"
#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECM.h"
#include "ECMCellCollection.h"

#include <iostream>

namespace ECM {

	namespace Simulation {

        // TODO: ecm/ecmGraph should probably be a member of this class
		bool IRMPathFollower::FindAttractionPoint(const ECM& ecm, const ECMGraph& ecmGraph, const Point& position, const PathComponent& path, int ecmCell, Point& outPoint)
		{
			// retract point on the medial axis
			Point retractedLoc;
			ECMEdge edge;
			if (!ecm.RetractPoint(ecmCell, position, retractedLoc, edge))
			{
				std::cout << "couldn't find attraction point.. aborting path following" << std::endl;
                return false;
			}

			// calculate clearance at retracted point location
            // this is not always a simple linear interpolation, so we calculate the distance to the closest obstacle.
			const ECMVertex& vertA = *ecmGraph.GetVertex(edge.half_edges[0].v_target_idx);
			const ECMVertex& vertB = *ecmGraph.GetVertex(edge.half_edges[1].v_target_idx);

            // calculate distance to nearest obstacle
            Point obstA = edge.half_edges[0].closest_left;
            Point obstB = edge.half_edges[1].closest_right;

            Point closestObst = Utility::MathUtility::GetClosestPointOnSegment(retractedLoc, obstA, obstB);
            float clearance = ((Vec2)(retractedLoc - closestObst)).Length();
            float retrLocClearanceSqr = clearance * clearance;

			// now we have a disk centered at the retraction point with the max clearance as its radius.
			// we check where this disk intersects with the indicative path: these become our candiate attraction points

            int resultIndex = -1;

            // first check if goal position lies in clearance disk. if so, set attraction point to be the goal point
            Vec2 posToGoal = (Point(path.x[path.numPoints - 1], path.y[path.numPoints - 1]) - retractedLoc);
            if (posToGoal.LengthSquared() < retrLocClearanceSqr)
            {
                outPoint = Point(path.x[path.numPoints - 1], path.y[path.numPoints - 1]);
                return true;
            }

            bool success = false;

			// calculate candidate attraction points
            for (int i = 0; i < path.numPoints - 1; i++) {
                // Edge points relative to clearance circle
                Point p1 = Point(path.x[i], path.y[i]) - retractedLoc;
                Point p2 = Point(path.x[i + 1], path.y[i + 1]) - retractedLoc;

                Vec2 edgeDir = p2 - p1;
                float edgeLengthSqr = edgeDir.LengthSquared();
                
                float determinant = Utility::MathUtility::Determinant(p1, p2);

                float discriminant = retrLocClearanceSqr * edgeLengthSqr - determinant * determinant;

                // Skip this line segment if its line does not intersect with clearance circle
                if (discriminant < Utility::EPSILON)
                    continue;

                success = true;

                int dySign = edgeDir.y < 0.0f ? -1 : 1;

                // Calculate the intersection points
                float sqrtDiscriminant = sqrtf(discriminant);
                Point intersect1 = {
                    (determinant * edgeDir.y + dySign * edgeDir.x * sqrtDiscriminant) / edgeLengthSqr,
                    (-determinant * edgeDir.x + fabs(edgeDir.y) * sqrtDiscriminant) / edgeLengthSqr
                };
                Point intersect2 = {
                    (determinant * edgeDir.y - dySign * edgeDir.x * sqrtDiscriminant) / edgeLengthSqr,
                    (-determinant * edgeDir.x - fabs(edgeDir.y) * sqrtDiscriminant) / edgeLengthSqr
                };

                // Transform back to global coordinates
                Point globalP1 = p1 + retractedLoc;
                Point globalP2 = p2 + retractedLoc;
                Point globalIntersect1 = intersect1 + retractedLoc;
                Point globalIntersect2 = intersect2 + retractedLoc;

                // Calculate t-values for each intersect point
                Vec2 edge = globalP2 - globalP1;
                float t1 = Utility::MathUtility::Dot((globalIntersect1 - globalP1), edge) / edgeLengthSqr;
                float t2 = Utility::MathUtility::Dot((globalIntersect2 - globalP1), edge) / edgeLengthSqr;

                float maxT = -1.0f;
                // choose the furthest intersection point down the path
                if(t1 >= 0.0f && t1 <= 1.0f)
                {
                    outPoint = globalIntersect1;
                    maxT = t1;
                    resultIndex = i;
                }
                if (t2 >= 0.0f && t2 <= 1.0f)
                {
                    if (t2 > maxT)
                    {
                        outPoint = globalIntersect2;
                        resultIndex = i;
                    }
                }
            }

			return success;
		}


	}
}