#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"
#include "ECMBenchmark.h"
#include "Simulator.h"
#include "KDTree.h"
#include "Timer.h"
#include "RVO.h"

#include <stdio.h>

int main()
{
	using namespace ECM;
	using namespace Simulation;
	using namespace WindowApplication;

	// test

	// AGENT A
	PositionComponent position;
	position.x = -2;
	position.y = 2;
	VelocityComponent preferredVelocity;
	preferredVelocity.dx = 3;
	preferredVelocity.dy = 0;
	ClearanceComponent clearance;
	clearance.clearance = 1.0f;

	// NEIGHBORS (AGENT B)
	PositionComponent* nPositions = new PositionComponent[1];
	VelocityComponent* nPreferredVelocities = new VelocityComponent[1];
	ClearanceComponent* nClearances = new ClearanceComponent[1];
	nPositions[0].x = 2;
	nPositions[0].y = 0;
	nPreferredVelocities[0].dx = 0;
	nPreferredVelocities[0].dy = 1;
	nClearances[0].clearance = 1;
	
	std::vector<Constraint> outConstraints(1);

	RVO rvo;
	rvo.GenerateConstraints(1, 1, position, preferredVelocity, clearance, nPositions, nPreferredVelocities, nClearances, outConstraints);

	delete[] nPositions;
	delete[] nPreferredVelocities;
	delete[] nClearances;
	// end test
	int gridSize = 10;
	PositionComponent* positions = new PositionComponent[gridSize* gridSize];
	for (int y = 0; y < gridSize; y++)
	{
		for(int x = 0; x < gridSize; x++)
		{
			positions[y * gridSize + x].x = x;
			positions[y * gridSize + x].y = y;
		}
	}


	KDTree tree;
	tree.TestConstruct(positions, gridSize * gridSize);
	
	const float range = 2.0f;
	std::vector<int> agents;
	tree.AgentsInRangeTest(positions, 25, range, 5, agents);

	// BRUTE FORCE NNEIGHBOR METHOD
	/*
	std::vector<float> kNearest;
	std::vector<int> kNearestIdx;
	{
		Timer timer("SIMPLE METHOD");

		kNearest.resize(k, Utility::MAX_FLOAT);
		kNearestIdx.resize(k, -1);

		Vec2 target(positions[550].x, positions[550].y);
		for (int i = 0; i < gridSize * gridSize; i++)
		{
			Vec2 diff(target.x - positions[i].x, target.y - positions[i].y);
			float dist = diff.x * diff.x + diff.y * diff.y;

			if (i < (k - 1))
			{
				kNearest[i] = dist;
				kNearestIdx[i] = i;
			}
			else
			{
				// found new nearest
				if (kNearest[k - 1] > dist)
				{
					kNearest[k - 1] = dist;
					kNearestIdx[k - 1] = i;


					// go through list of current k nearest to find the new largest distance and update the k nearest list accordingly.
					float largestDistance = dist;
					int largetIndexTree = k - 1;
					for (int j = 0; j < (k - 1); j++)
					{
						if (kNearest[j] > largestDistance)
						{
							largestDistance = kNearest[j];
							largetIndexTree = j;
						}
					}

					// update list to swap the highest distance with the last element
					kNearest[k - 1] = largestDistance;
					kNearestIdx[k - 1] = kNearestIdx[largetIndexTree];
					kNearest[largetIndexTree] = dist;
					kNearestIdx[largetIndexTree] = i;
				}
			}
		}
	}

	delete[] dummy;
	delete[] positions;

	*/

	Environment env;
	env.Initialize(Environment::TestEnvironment::CLASSIC);
	ECMPathPlanner planner(env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 50);
	sim.Initialize();

	// TODO:
	// 1. encapsulate this in SpawnArea and GoalArea
	// 2. in the Application update method, implement a spawnRate (debug for now) and do a timed SpawnAgent,
	//     using the SpawnArea and GoalArea data to get a valid start and goal position.
	//const float minX = -400;
	//const float maxX = 400;
	//const float minStartY = -475;
	//const float maxStartY = -350;
	//const float minEndY = 300;
	//const float maxEndY = 470;
	//
	//for (int i = 0; i < 1; i++)
	//{
	//	float xStart = minX + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxX - minX)));
	//	float yStart = minStartY + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxStartY - minStartY)));
	//
	//	float xEnd = minX + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxX - minX)));
	//	float yEnd = minEndY + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxEndY - minEndY)));
	//
	//	sim.SpawnAgent(Point(xStart, yStart), Point(xEnd, yEnd), 10.0f, 3.0f);
	//}

	Application app(&planner, &env, &sim);
	if (!app.InitializeApplication("ECM generation tool", 1080, 720))
	{
		printf("ERROR: could not initialize ECM applciation.\n");
		return -1;
	}
	app.Run();
	app.Clear();

	return 0;
}