#include "Environment.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "ECMGenerator.h"

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

		// generate ECM from environment
		ComputeECM();
	}


	void Environment::AddWalkableArea(std::vector<Segment> waEdges)
	{
		_walkableArea = waEdges;

		for (Segment s : waEdges) _environmentObstacleUnion.push_back(s);

		UpdateBbox(waEdges);
	}

	void Environment::AddObstacle(std::vector<Segment> obstacleEdges)
	{
		_obstacles.push_back(obstacleEdges);
		for (Segment s : obstacleEdges) _environmentObstacleUnion.push_back(s);

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
			if (edge.p0.x < _bbox.min.x) _bbox.min.x = edge.p0.x;
			if (edge.p0.y < _bbox.min.y) _bbox.min.y = edge.p0.y;
			if (edge.p1.x < _bbox.min.x) _bbox.min.x = edge.p1.x;
			if (edge.p1.y < _bbox.min.y) _bbox.min.y = edge.p1.y;

			// update max values
			if (edge.p0.x > _bbox.max.x) _bbox.max.x = edge.p0.x;
			if (edge.p0.y > _bbox.max.y) _bbox.max.y = edge.p0.y;
			if (edge.p1.x > _bbox.max.x) _bbox.max.x = edge.p1.x;
			if (edge.p1.y > _bbox.max.y) _bbox.max.y = edge.p1.y;
		}
	}

	bool Environment::InsideObstacle(const Point& p) const
	{
		// todo: something clever with kd-tree

		for (const auto& obs : _obstacles)
		{
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