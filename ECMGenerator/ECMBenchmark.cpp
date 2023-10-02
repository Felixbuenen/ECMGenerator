#include "ECMBenchmark.h"
#include "Environment.h"
#include "ECM.h"

#include <iostream>

namespace ECM {

	bool ECMBenchmark::InitTestEnvironment(Environment::TestEnvironment environment)
	{
		// setup environment
		if (environment == Environment::TestEnvironment::CLASSIC)
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

			m_Environment.AddWalkableArea(walkableArea);
			m_Environment.AddObstacle(obstacle);
		}
		if (environment == Environment::TestEnvironment::BIG)
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-2500, -2500, 2500, -2500));
			walkableArea.push_back(Segment(2500, -2500, 2500, 2500));
			walkableArea.push_back(Segment(-2500, 2500, 2500, 2500));
			walkableArea.push_back(Segment(-2500, 2500, -2500, -2500));
			m_Environment.AddWalkableArea(walkableArea);

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

					m_Environment.AddObstacle(obstacle);
				}
			}
		}

		if (environment == Environment::TestEnvironment::NARROW)
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

			m_Environment.AddWalkableArea(walkableArea);
			m_Environment.AddObstacle(obstacle1);
			m_Environment.AddObstacle(obstacle2);
		}

		// generate ECM from environment
		m_Environment.ComputeECM();

		m_Planner.Initialize(m_Environment.GetECM()->GetECMGraph());

		return true;
	}

	bool ECMBenchmark::PerformPathPlanTest(Point start, Point end, float clearance, int numRepetitions, bool writeToFile)
	{
		float* values = new float[numRepetitions];

		for (int i = 0; i < numRepetitions; i++)
		{
			BenchmarkTimerMS timer;
			Corridor c;
			std::vector<Segment> portals;
			Path path;
			
			timer.Start();

			m_Planner.GetPath(m_Environment, start, end, clearance, c, portals, path);

			values[i] = timer.Stop();
		}

		if (!writeToFile)
		{
			WriteStatsToScreen(values, numRepetitions);
		}

		delete[] values;
	}

	void ECMBenchmark::WriteStatsToScreen(float* values, int numValues)
	{
		float average = 0.0f;
		for (int i = 0; i < numValues; i++)
		{
			average += values[i];
		}

		average /= (float)numValues;
		
		std::cout << "Average: " << average << "ms.\n";
	}


	void ECMBenchmark::InitFile(const char* path, std::vector<const char*> columns, std::ofstream& outFile)
	{
		
	}

	void ECMBenchmark::WriteToFile(std::ofstream& file, std::vector<const char*> columns, std::vector<float> values)
	{

	}

}