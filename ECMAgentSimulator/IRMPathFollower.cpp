#include "IRMPathFollower.h"
#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECM.h"
#include "ECMCellCollection.h"

#include <iostream>

namespace ECM {

	namespace Simulation {

        // TODO: ecm/ecmGraph should probably be a member of this class
		Point IRMPathFollower::FindAttractionPoint(const ECM& ecm, const ECMGraph& ecmGraph, const Point& position, const PathComponent& path)
		{
			// retract point on the medial axis
			Point retractedLoc;
			ECMEdge edge;
			if (!ecm.RetractPoint(position, retractedLoc, edge))
			{
				std::cout << "couldn't find attraction point.. aborting path following" << std::endl;
				return Point();
			}

			// calculate clearance at retracted point location
			// this is simply a linear interpolation between the two vertex clearances
			const ECMVertex& vertA = *ecmGraph.GetVertex(edge.half_edges[0].v_target_idx);
			const ECMVertex& vertB = *ecmGraph.GetVertex(edge.half_edges[1].v_target_idx);

			float sqLengthTotal = Utility::MathUtility::SquareDistance(vertA.position, vertB.position);
			float sqLengthSeg = Utility::MathUtility::SquareDistance(vertA.position, retractedLoc);
			float t = sqLengthSeg / sqLengthTotal;

			float retrLocClearance = vertA.clearance + (vertB.clearance - vertA.clearance) * t;
            float retrLocClearanceSqr = retrLocClearance * retrLocClearance;

			// now we have a disk centered at the retraction point with the max clearance as its radius.
			// we check where this disk intersects with the indicative path: these become our candiate attraction points

			Point result;
            int resultIndex = -1;

            // first check if goal position lies in clearance disk. if so, set attraction point to be the goal point
            Vec2 posToGoal = (Point(path.x[path.numPoints - 1], path.y[path.numPoints - 1]) - retractedLoc);
            if (posToGoal.LengthSquared() < retrLocClearanceSqr)
            {
                return Point(path.x[path.numPoints - 1], path.y[path.numPoints - 1]);
            }

			// calculate candidate attraction points
            for (int i = 0; i < path.numPoints - 1; i++) {
                // Edge points relative to clearance circle
                Point p1 = Point(path.x[i], path.y[i]) - retractedLoc;
                Point p2 = Point(path.x[i + 1], path.y[i + 1]) - retractedLoc;

                Vec2 edgeDir = p2 - p1;
                float edgeLengthSqr = edgeDir.x * edgeDir.x + edgeDir.y * edgeDir.y;
                float determinant = Utility::MathUtility::Determinant(p1, p2);

                float discriminant = retrLocClearanceSqr * edgeLengthSqr - determinant * determinant;

                // Skip this line segment if its line does not intersect with clearance circle
                if (discriminant < Utility::EPSILON) continue;

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
                // choose the furthest intersection point down the path edge
                if(t1 >= 0.0f && t1 <= 1.0f)
                {
                    result = globalIntersect1;
                    maxT = t1;
                    resultIndex = i;
                }
                if (t2 >= 0.0f && t2 <= 1.0f)
                {
                    if (t2 > maxT)
                    {
                        result = globalIntersect2;
                        resultIndex = i;
                    }
                }
            }

			return result;
		}


	}
}