#include "ECMDataTypes.h"

#include "UtilityFunctions.h"

#include <iostream>

namespace ECM {

	// TODO: better way to manage obstacle memory?
	Obstacle::Obstacle(const Obstacle& c)
	{
		Initialize(c.verts);
	}

	Obstacle::~Obstacle()
	{
		for (int i = 0; i < verts.size(); i++)
		{
			delete verts[i];
		}
	}

	void Obstacle::Initialize(const std::vector<Point>& vertexPositions)
	{
		int numVerts = vertexPositions.size();

		// add obstacle data
		for (int i = 0; i < numVerts; i++)
		{
			ObstacleVertex* o = new ObstacleVertex();
			o->p = vertexPositions[i];
			verts.push_back(o);
		}

		// add neighbor information
		for (int i = 0; i < numVerts; i++)
		{
			ObstacleVertex* obstacle = verts[i];

			if (i == 0)
			{
				obstacle->prevObstacle = verts[numVerts - 1];
				obstacle->nextObstacle = verts[1];
			}
			else
			{
				obstacle->prevObstacle = verts[i - 1];
				obstacle->nextObstacle = verts[((i + 1) % numVerts)];
			}

			// calculate if convex
			if (numVerts == 2)
			{
				obstacle->isConvex = true;
			}
			else
			{
				obstacle->isConvex = Utility::MathUtility::Determinant(obstacle->prevObstacle->p - obstacle->nextObstacle->p, obstacle->p - obstacle->prevObstacle->p) >= 0.0f;
			}
		}
	}

	void Obstacle::Initialize(const std::vector<ObstacleVertex*>& vertices)
	{
		if (verts.size() != 0)
		{
			std::cout << "Cannot initialize already initialized obstacle" << std::endl;
		}

		int numVerts = vertices.size();
		verts.resize(numVerts);

		for (int i = 0; i < numVerts; i++)
		{
			verts[i] = new ObstacleVertex();
			verts[i]->isConvex = vertices[i]->isConvex;
			verts[i]->p = vertices[i]->p;
		}

		for (int i = 0; i < numVerts; i++)
		{
			ObstacleVertex* ov = verts[i];
			if (i == 0)
			{
				ov->prevObstacle = verts[numVerts - 1];
				ov->nextObstacle = verts[1];
			}
			else
			{
				ov->prevObstacle = verts[i - 1];
				ov->nextObstacle = verts[((i + 1) % numVerts)];
			}
		}
	}


	bool Point::Approximate(const Point& a) const
	{
		return x < (a.x + Utility::EPSILON) && x >(a.x - Utility::EPSILON) && y < (a.y + Utility::EPSILON) && y >(a.y - Utility::EPSILON);
	}

}