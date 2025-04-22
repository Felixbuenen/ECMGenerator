#pragma once

#include "ECMDataTypes.h"

#include <random>

namespace ECM {

	namespace Simulation {

		enum SimAreaType
		{
			NONE,
			WALKABLE,
			SPAWN,
			GOAL,
			OBSTACLE
		};

		struct AreaConnection
		{
			AreaConnection(int _spawnID, int _goalID)
				: spawnID(_spawnID), goalID(_goalID) { }

			AreaConnection() { }

			int spawnID = -1;
			int goalID = -1;
		};

		struct Area
		{
			int ID;
			Point Position;
			float HalfHeight;
			float HalfWidth;
			SimAreaType Type;

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

			void Scale(Vec2 delta)
			{
				HalfHeight += delta.y;
				HalfWidth += delta.x;
			}

			void Translate(Vec2 delta)
			{
				Position = Position + delta;
			}

			bool Intersects(const Point position) const
			{
				bool result = position.x <= (Position.x + HalfWidth);
				result &= position.x >= (Position.x - HalfWidth);
				result &= position.y <= (Position.y + HalfHeight);
				result &= position.y >= (Position.y - HalfHeight);

				return result;
			}
		};

		struct GoalArea : public Area
		{
			GoalArea() { Type = GOAL; }
		};

		struct SpawnConfiguration
		{
			float preferredSpeedMin = 5.0f;
			float preferredSpeedMax = 10.0f;
			float clearanceMin = 5.0f;
			float clearanceMax = 10.0f;
		};

		struct SpawnArea : public Area
		{
			SpawnArea() { Type = SPAWN; }

			SpawnConfiguration spawnConfiguration;

			std::vector<int> connectedGoalAreas;
			std::vector<float> spawnRate; // number of agents per second, per connected goal area
			std::vector<float> timeSinceLastSpawn;
		};

		struct ObstacleArea : public Area
		{
			ObstacleArea() { Type = OBSTACLE; }
			std::vector<Point> obstacleVerts;
		};
	}

}