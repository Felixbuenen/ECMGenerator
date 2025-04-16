#pragma once

#include <chrono>
#include <iostream>
#include <unordered_map>

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

	class MultipassTimer {
	public:
		MultipassTimer(const std::string& label)
			: m_Label(label), m_Start(std::chrono::high_resolution_clock::now()) {}

		~MultipassTimer() {
			auto end = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - m_Start).count();
			Accumulate(m_Label, duration);
		}

		static bool PrintAverages(int everyNFrames = 100) {
			m_FrameCount++;
			if (m_FrameCount % everyNFrames == 0) {
				std::cout << std::endl;
				std::cout << "---- Timing (avg over " << everyNFrames << " frames) ----\n";
				for (const auto& [m_Label, data] : m_Timings) {
					//double avgMs = (data.totalMicroseconds / 1000.0) / data.count;
					double avgMs = (data.totalMicroseconds / 1000.0) / (double)everyNFrames;
					std::cout << m_Label << ": " << avgMs << " ms\n";
				}
				std::cout << "--------------------------------------\n";
				m_Timings.clear();

				return true;
			}

			return false;
		}

	private:
		struct TimingData {
			long long totalMicroseconds = 0;
			int count = 0;
		};

		inline static std::unordered_map<std::string, TimingData> m_Timings;
		inline static int m_FrameCount = 0;

		std::string m_Label;
		std::chrono::high_resolution_clock::time_point m_Start;

		static void Accumulate(const std::string& label, long long microseconds) {
			m_Timings[label].totalMicroseconds += microseconds;
			m_Timings[label].count += 1;
		}
	};

}

