#include "Environment.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "ECMGenerator.h"

#include <iostream>

namespace ECM {

	void Environment::Initialize(TestEnvironment type)
	{
		// setup environment
		if (type == Environment::TestEnvironment::CLASSIC)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-500, -500, 500, -500));
			walkableArea.push_back(Segment(500, -500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, -500, -500));

			std::vector<Segment> obstacle{
				Segment(-200, 250, -200, -250),
					Segment(-200, -250, 200, -250),
					Segment(200, -250, 200, 250),
					Segment(200, 250, 100, 250),
					Segment(100, 250, 100, -150),
					Segment(100, -150, -100, -150),
					Segment(100, -150, -100, -150),
					Segment(-100, -150, -100, 250),
					Segment(-100, 250, -200, 250)
			};

			AddWalkableArea(walkableArea);
			AddObstacle(obstacle);
		}
		if (type == Environment::TestEnvironment::BIG)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-2500, -2500, 2500, -2500));
			walkableArea.push_back(Segment(2500, -2500, 2500, 2500));
			walkableArea.push_back(Segment(-2500, 2500, 2500, 2500));
			walkableArea.push_back(Segment(-2500, 2500, -2500, -2500));
			AddWalkableArea(walkableArea);

			const float obstacleSize = 200.0f;
			const float padding = 200.0f;

			int numObstaclesRow = 5000.0f / (obstacleSize + padding);
			int numObstaclesCol = 5000.0f / (obstacleSize + padding);

			for (int r = 0; r < numObstaclesRow; r++)
			{
				float y = 2500 - padding - r * (obstacleSize + padding);

				for (int c = 0; c < numObstaclesCol; c++)
				{
					float x = -2500 + padding + c * (obstacleSize + padding);

					std::vector<Segment> obstacle{
						Segment(Point(x, y), Point(x + obstacleSize, y)),
							Segment(Point(x + obstacleSize, y), Point(x + obstacleSize, y - obstacleSize)),
							Segment(Point(x + obstacleSize, y - obstacleSize), Point(x, y - obstacleSize)),
							Segment(Point(x, y - obstacleSize), Point(x, y))
					};

					AddObstacle(obstacle);
				}
			}
		}

		if (type == Environment::TestEnvironment::NARROW)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-100, -100, 100, -100));
			walkableArea.push_back(Segment(100, -100, 100, 100));
			walkableArea.push_back(Segment(-100, 100, 100, 100));
			walkableArea.push_back(Segment(-100, 100, -100, -100));

			std::vector<Segment> obstacle1{
				Segment(-20, -60, -20, 60),
					Segment(-20, 60, -10, 60),
					Segment(-10, 60, -10, -60),
					Segment(-10, -60, -20, -60)
			};
			std::vector<Segment> obstacle2{
				Segment(20, -60, 20, 60),
					Segment(20, 60, 10, 60),
					Segment(10, 60, 10, -60),
					Segment(10, -60, 20, -60)
			};

			AddWalkableArea(walkableArea);
			AddObstacle(obstacle1);
			AddObstacle(obstacle2);
		}

		if (type == Environment::TestEnvironment::LINES)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-100, -100, 100, -100));
			walkableArea.push_back(Segment(100, -100, 100, 100));
			walkableArea.push_back(Segment(-100, 100, 100, 100));
			walkableArea.push_back(Segment(-100, 100, -100, -100));

			std::vector<Segment> obstacle1{
				Segment(-20, -60, -10, 70)
			};
			std::vector<Segment> obstacle2{
				Segment(60, -60, 20, 60)
			};

			AddWalkableArea(walkableArea);
			AddObstacle(obstacle1);
			AddObstacle(obstacle2);
		}

		if (type == Environment::TestEnvironment::EMPTY)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-500, -500, 500, -500));
			walkableArea.push_back(Segment(500, -500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, -500, -500));


			AddWalkableArea(walkableArea);
		}

		// generate ECM from environment
		ComputeECM();
	}


	void Environment::AddWalkableArea(std::vector<Segment> waEdges)
	{
		m_WalkableArea = waEdges;

		for (const Segment& s : waEdges) m_EnvironmentObstacleUnion.push_back(s);

		UpdateBbox(waEdges);
	}

	void Environment::AddObstacle(std::vector<Segment> obstacleEdges)
	{
		// we don't allow edge obstacles in our simulation
		if (obstacleEdges.size() < 3)
		{
			std::cout << "ERROR: obstacles must have size of at least 3" << std::endl;
			return;
		}

		int obstacleSize = obstacleEdges.size();

		// add segments to obstacle union
		for (int i = 0; i < obstacleSize; i++)
		{
			m_EnvironmentObstacleUnion.push_back(obstacleEdges[i]);
		}

		// all obstacles
		int currentSize = m_Obstacles.size();
		for (int i = 0; i < obstacleSize; i++)
		{
			Obstacle obstacle;
			Segment& s1 = obstacleEdges[i];

			obstacle.p = s1.p0;
			m_Obstacles.push_back(obstacle);
		}

		// all neighboring obstacles + calculate if convex
		for (int i = 0; i < obstacleSize; i++)
		{
			Obstacle& obstacle = m_Obstacles[currentSize + i];

			if (i == 0)
			{
				obstacle.prevObstacle = &m_Obstacles[currentSize + obstacleSize - 1];
				obstacle.nextObstacle = &m_Obstacles[currentSize + 1];
			}
			else
			{
				obstacle.prevObstacle = &m_Obstacles[currentSize + i - 1];
				obstacle.nextObstacle = &m_Obstacles[currentSize + ((i + 1) % obstacleSize)];
			}

			// calculate if convex
			float areaSum = 0;

			areaSum += obstacle.prevObstacle->p.x * (obstacle.nextObstacle->p.y - obstacle.p.y);
			areaSum += obstacle.p.x * (obstacle.prevObstacle->p.y - obstacle.nextObstacle->p.y);
			areaSum += obstacle.nextObstacle->p.x * (obstacle.p.y - obstacle.prevObstacle->p.y);

			obstacle.isConvex = areaSum < 0.0f;
		}

		m_ObstaclesDeprecated.push_back(obstacleEdges);

		UpdateBbox(obstacleEdges);
	}

	void Environment::ComputeECM()
	{		
		// for now just support generation of 1 ecm. However, an environment may contain various ECM graphs.
		m_EcmList.push_back(ECMGenerator::GenerateECM(*this));
	}

	void Environment::UpdateBbox(const std::vector<Segment>& newEdges)
	{
		for (const Segment& edge : newEdges)
		{
			// update min values
			if (edge.p0.x < m_Bbox.min.x) m_Bbox.min.x = edge.p0.x;
			if (edge.p0.y < m_Bbox.min.y) m_Bbox.min.y = edge.p0.y;
			if (edge.p1.x < m_Bbox.min.x) m_Bbox.min.x = edge.p1.x;
			if (edge.p1.y < m_Bbox.min.y) m_Bbox.min.y = edge.p1.y;

			// update max values
			if (edge.p0.x > m_Bbox.max.x) m_Bbox.max.x = edge.p0.x;
			if (edge.p0.y > m_Bbox.max.y) m_Bbox.max.y = edge.p0.y;
			if (edge.p1.x > m_Bbox.max.x) m_Bbox.max.x = edge.p1.x;
			if (edge.p1.y > m_Bbox.max.y) m_Bbox.max.y = edge.p1.y;
		}
	}

	// TODO: change to new m_Obstacle structure
	bool Environment::InsideObstacle(const Point& p) const
	{
		// todo: something clever with kd-tree

		for (const auto& obs : m_ObstaclesDeprecated)
		{
			if (obs.size() == 1) continue; // not a polygon

			if (Utility::MathUtility::Contains(p, obs)) return true;
		}

		return false;
	}

	// for now just return the only ECM we have. However, this method can be expanded to be used with multiple ECM graphs.
	std::shared_ptr<ECM> Environment::QueryECM(Point position) const
	{
		return m_EcmList[0];
	}
}