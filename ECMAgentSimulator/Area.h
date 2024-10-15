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
				float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

				float xMin = Position.x - HalfWidth;
				float xMax = Position.x + HalfWidth;
				float yMin = Position.y - HalfHeight;
				float yMax = Position.y + HalfHeight;

				float randX = xMin + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (xMax - xMin)));
				float randY = yMin + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (yMax - yMin)));


				return Point(randX, randY);
			}
		};

		struct GoalArea : public Area
		{
		};

		struct AreaConnector
		{
			int goalID;
		};

		struct SpawnConfiguration
		{
			float preferredSpeedMin = 20.0f;
			float preferredSpeedMax = 80.0f;
			float clearanceMin = 10.0f;
			float clearanceMax = 20.0f;
			int spawnRate = 10;
		};

		struct SpawnArea : public Area
		{
			float spawnRate; // number of agents per second
			float timeSinceLastSpawn = 0.0f;

			// spawn configuration
			SpawnConfiguration spawnConfiguration;
			std::vector<AreaConnector> connectors;
		};
	}

}