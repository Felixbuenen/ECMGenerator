#pragma once

#include <vector>
#include <fstream>
#include <chrono>

#include "Environment.h"
#include "ECMPathPlanner.h"

namespace ECM
{
	using namespace PathPlanning;

	class BenchmarkTimerMS
	{
	public:
		void Start()
		{
			m_StartTimePoint = std::chrono::high_resolution_clock::now();
		}

		float Stop()
		{
			auto endTimePoint = std::chrono::high_resolution_clock::now();

			long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
			long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePoint).time_since_epoch().count();

			return (end - start) * 0.001;
		}

	private:
		std::chrono::time_point<std::chrono::steady_clock> m_StartTimePoint;
	};

	class BenchmarkFile
	{
	public:
		BenchmarkFile(const char* path) : m_Path(path), m_Stream(std::ofstream(path))
		{

		}

	private:
		const char* m_Path;
		std::ofstream m_Stream;
	};

	class ECMBenchmark
	{
	public:
		bool Initialize(Environment& environment);
		bool PerformPathPlanTest(Point start, Point end, float clearance, int numRepetitions, bool writeToFile = false);

	private:
		void InitFile(const char* path, std::vector<const char*> columns, std::ofstream& outFile);
		void WriteToFile(std::ofstream& file, std::vector<const char*> columns, std::vector<float> values);
		void WriteStatsToScreen(float* values, int numValues);

		Environment m_Environment;
		ECMPathPlanner m_Planner;
	};
}
