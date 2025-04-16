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
#include "ORCA.h"

#include <stdio.h>

int main()
{
	using namespace ECM;
	using namespace Simulation;
	using namespace WindowApplication;

	//PositionComponent* positions = new PositionComponent[100];
	//for (int i = 0; i < 10; i++)
	//{
	//	for (int j = 0; j < 10; j++)
	//	{
	//		positions[j * 10 + i].x = i * 5.0f;
	//		positions[j * 10 + i].y = j * 5.0f;
	//	}
	//}
	
	//KDTree tree;
	//tree.TestConstruct(positions, 100);
	//tree.TestConstruct(positions, 100);
	////int* neighbors = new int[5];
	//std::vector<int> neighbors(5);
	//int numNeighbors = 0;
	//tree.KNearestAgentsTest(positions, 55, 5, neighbors, numNeighbors);
	//std::vector<int> nn;
	
	// test seems to work
	//tree.AgentsInRangeTest(positions, 55, 6.0f, 5, nn);
	
	//for (int i = 0; i < 5; i++) nn.push_back(neighbors[i]);
	
	int stop = 0;
	//delete[] neighbors;
	//delete[] positions;
	
	Environment env;
	env.Initialize(Environment::TestEnvironment::CLASSIC);
	ECMPathPlanner planner(&env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 10000, 0.1f);

	// DEBUG
	SpawnConfiguration config;
	sim.AddSpawnArea(Point(-340.479, 364.999), Vec2(50, 50), config);
	sim.AddSpawnArea(Point(-142.917, 363.598), Vec2(50, 50), config);
	sim.AddSpawnArea(Point(25.2207, 362.197), Vec2(50, 50), config);
	sim.AddSpawnArea(Point(183.551, 362.197), Vec2(50, 50), config);
	sim.AddSpawnArea(Point(344.683, 359.395), Vec2(50, 50), config);
	sim.AddGoalArea(Point(0.0, -369.203), Vec2(400, 75));

	sim.ConnectSpawnGoalAreas(0, 0, 0.5);
	sim.ConnectSpawnGoalAreas(1, 0, 0.5);
	sim.ConnectSpawnGoalAreas(2, 0, 0.5);
	sim.ConnectSpawnGoalAreas(3, 0, 0.5);
	sim.ConnectSpawnGoalAreas(4, 0, 0.5);
	// DEBUG

	sim.Initialize();

	Application app(env.GetECM(), & planner, &env, &sim);
	if (!app.InitializeApplication("Crowd Simulator", true))
	{
		printf("ERROR: could not initialize ECM applciation.\n");
		return -1;
	}
	app.Run();
	app.Clear();

	return 0;
}