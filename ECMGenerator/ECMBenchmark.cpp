#include "ECMBenchmark.h"
#include "Environment.h"
#include "ECM.h"

#include <iostream>

namespace ECM {

	bool ECMBenchmark::Initialize(Environment& environment)
	{
		m_Environment = environment;
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