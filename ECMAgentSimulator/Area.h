#pragma once

#include "ECMDataTypes.h"

#include <random>

namespace ECM {

	namespace Simulation {

		struct Area
		{
			int ID;
			Point Position;
			float HalfHeight;
			float HalfWidth;

			Point GetRandomPositionInArea()
			{
				std::random_device rd;
				std::mt19937 gen(rd());
				std::uniform_real_distribution<float> xDistribution(Position.x - HalfWidth, Position.x + HalfWidth);
				std::uniform_real_distribution<float> yDistribution(Position.y - HalfHeight, Position.y + HalfHeight);

				float randomX = xDistribution(gen);
				float randomY = yDistribution(gen);

				return Point(randomX, randomY);
			}
		};

		struct SpawnArea : public Area
		{
			float spawnRate; // number of agents per second
		};

		struct GoalArea : public Area
		{
		};

		struct AreaConnector
		{
			int fromID;
			int toID;
		};
	}

}