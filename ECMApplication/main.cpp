#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"
#include "Simulator.h"
#include "KDTree.h"
#include "Timer.h"
#include "ORCA.h"
#include "ECMPathPlanner.h"

#include <stdio.h>

int main()
{
	using namespace ECM;
	using namespace Simulation;
	using namespace WindowApplication;
	using namespace PathPlanning;

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
	env.Initialize(Environment::TestEnvironment::BIG);
	ECMPathPlanner planner(&env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 10000, 0.1f);
	sim.Initialize();

	//// DEBUG
	SpawnConfiguration config;
	const int COLS = 150;
	const int ROWS = 5000 / COLS;
	
	int goalID = sim.AddGoalArea(Point(0.0, -800), Vec2(1300, 400));
	sim.Initialize();
	
	for (int j = 0; j < ROWS; j++)
	{
		for (int i = 0; i < COLS; i++)
		{
			Point start(-1950 + 25 * i, 1950 - 25 * j);
			Point goal = sim.GetGoalArea(goalID)->GetRandomPositionInArea();
	
			sim.SpawnAgent(start, goal, 10.0f, 5.0f);
		}
	}
	// DEBUG


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