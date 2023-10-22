#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"
#include "ECMBenchmark.h"
#include "Simulator.h"

#include <stdio.h>

int main()
{
	using namespace ECM;
	using namespace Simulation;
	using namespace WindowApplication;

	Environment env;
	env.Initialize(Environment::TestEnvironment::CLASSIC);
	ECMPathPlanner planner(env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 20000);
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