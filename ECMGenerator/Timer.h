#pragma once

#include <chrono>
#include <iostream>

namespace ECM {

	class Timer
	{
	public:
		Timer(const char* name) :
			m_Name(name), m_Stopped(false)
		{
			m_StartTimePoint = std::chrono::high_resolution_clock::now();
		}

		void Stop()
		{
			auto endTimePoint = std::chrono::high_resolution_clock::now();

			long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
			long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePoint).time_since_epoch().count();

			std::cout << m_Name << ": " << (end - start) * 0.001f << "ms\n";

			m_Stopped = true;
		}

		~Timer()
		{
			if (!m_Stopped)
			{
				Stop();
			}
		}

	private:
		const char* m_Name;
		std::chrono::time_point<std::chrono::steady_clock> m_StartTimePoint;
		bool m_Stopped;
	};

}

