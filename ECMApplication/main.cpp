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
	
	Environment env;
	env.Initialize(Environment::TestEnvironment::BIG);
	ECMPathPlanner planner(&env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 10000, 0.1f);
	sim.Initialize();

	//// DEBUG
	SpawnConfiguration config;
	const int COLS = 150;
	const int ROWS = 10000 / COLS;
	//const int ROWS = 832 / COLS;
	
	int goalID = sim.AddGoalArea(Point(0.0, -800), Vec2(1300, 400));
	sim.Initialize();
	
	int offset = 25;
	
	for (int j = 0; j < ROWS; j++)
	{
		for (int i = 0; i < COLS; i++)
		{
			Point start(-1950 + offset * i, 1950 - offset * j);
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