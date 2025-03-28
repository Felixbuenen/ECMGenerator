#include "Environment.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "ECMGenerator.h"
#include "../ECMApplication/Application.h"

#include <iostream>

namespace ECM {

	Environment::~Environment()
	{
		for (ECM* ecm : m_Ecms)
		{
			delete ecm;
		}

		m_Ecms.clear();
	}


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

			std::vector<Point> obstacle{
				Point(200, -250),
				Point(200, 250),
				Point(100, 250),
				Point(100, -150),
				Point(-100, -150),
				Point(-100, 250),
				Point(-200, 250),
				Point(-200, -250)
			};

			AddWalkableArea(walkableArea);
			AddObstacle(obstacle);
		}
		if (type == Environment::TestEnvironment::SQUARE)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-500, -500, 500, -500));
			walkableArea.push_back(Segment(500, -500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, -500, -500));

			std::vector<Point> obstacle{
				Point(200, -250),
					Point(200, 250),
					Point(-200, 250),
					Point(-200, -250)
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

					//AddObstacle(obstacle);
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
			//AddObstacle(obstacle1);
			//AddObstacle(obstacle2);
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
			//AddObstacle(obstacle1);
			//AddObstacle(obstacle2);
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

		if (type == Environment::TestEnvironment::DEBUG1)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-500, -500, 500, -500));
			walkableArea.push_back(Segment(500, -500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, -500, -500));

			std::vector<Point> obstacle{
				Point(-50, 50),
					Point(-150, 50),
					Point(-150, -50),
					Point(-50, -50)
			};
			std::vector<Point> obstacle2{
				Point(150, 50),
					Point(50, 50),
					Point(50, -50),
					Point(150, -50)
			};

			AddWalkableArea(walkableArea);
			AddObstacle(obstacle);
			AddObstacle(obstacle2);
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

	// adds a static obstacle to the scene, defined by its vertices.
	// NOTE: it's important that the vertices are passed in anti-clockwise order!
	void Environment::AddObstacle(const std::vector<Point>& obstacleVerts, bool updateECM)
	{
		// we don't allow edge obstacles in our simulation
		if (obstacleVerts.size() < 2)
		{
			std::cout << "ERROR: obstacles must have size of at least 3" << std::endl;
			return;
		}


		int obstacleSize = obstacleVerts.size();
		int obstacleIdx = m_Obstacles.size();

		// add segments to obstacle union
		for (int i = 0; i < obstacleSize; i++)
		{
			Segment s;
			s.p0 = obstacleVerts[i];
			s.p1 = obstacleVerts[(i + 1) % obstacleSize];
			m_EnvironmentObstacleUnion.push_back(s);
		}

		Obstacle o;
		m_Obstacles.emplace_back(o);
		m_Obstacles[obstacleIdx].Initialize(obstacleVerts);

		m_ObstacleIndices.push_back(obstacleIdx);

		UpdateBbox(m_EnvironmentObstacleUnion);
		m_Dirty = updateECM;
	}


	void Environment::ComputeECM()
	{		
		// for now just support generation of 1 ecm. However, an environment may contain various ECM graphs.
		if (m_Ecms.size() == 0)
		{
			m_Ecms.push_back(ECMGenerator::GenerateECM(*this));
		}
	}

	void Environment::UpdateECM()
	{
		// for now just support generation of 1 ecm. However, an environment may contain various ECM graphs.
		for (ECM* ecm : m_Ecms)
		{
			ecm->Clear();
			ECMGenerator::GenerateECM(*this, ecm);
		}

		//m_Ecms.clear();
		//m_Ecms.push_back(ECMGenerator::GenerateECM(*this));


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
		for (const Obstacle& o : m_Obstacles)
		{
			if (Utility::MathUtility::Contains(p, o)) return true;
		}

		return false;
	}

	// for now just return the only ECM we have. However, this method can be expanded to be used with multiple ECM graphs.
	ECM* Environment::QueryECM(Point position) const
	{
		if (m_Ecms.size() == 0) return nullptr;

		return m_Ecms[0];
	}
}