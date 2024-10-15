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

	/*
	// AGENT A
	PositionComponent position;
	position.x = 0;
	position.y = 0;
	VelocityComponent preferredVelocity;
	preferredVelocity.dx = 0;
	preferredVelocity.dy = 4;
	ClearanceComponent clearance;
	clearance.clearance = 1.0f;

	// NEIGHBORS (AGENT B)
	PositionComponent* nPositions = new PositionComponent[1];
	VelocityComponent* nPreferredVelocities = new VelocityComponent[1];
	ClearanceComponent* nClearances = new ClearanceComponent[1];
	nPositions[0].x = -3;
	nPositions[0].y = 3;
	nPreferredVelocities[0].dx = 4;
	nPreferredVelocities[0].dy = 0;
	nClearances[0].clearance = 1;
	
	std::vector<Constraint> outConstraints(1);

	RVO rvo;
	rvo.GenerateConstraints(1, 1, position, preferredVelocity, clearance, nPositions, nPreferredVelocities, nClearances, outConstraints);
	Vec2 outVel;
	if (rvo.RandomizedLP(outConstraints, Vec2(preferredVelocity.dx, preferredVelocity.dy), 6.0f, outVel))
	{
		std::cout << "updated velocity: " << outVel.x << ", " << outVel.y << ")" << std::endl;
	}
	else
	{
		std::cout << "couldn't find solution" << std::endl;
	}

	delete[] nPositions;
	delete[] nPreferredVelocities;
	delete[] nClearances;
	*/
	// end test
	//int gridSize = 10;
	//PositionComponent* positions = new PositionComponent[gridSize* gridSize];
	//for (int y = 0; y < gridSize; y++)
	//{
	//	for(int x = 0; x < gridSize; x++)
	//	{
	//		positions[y * gridSize + x].x = x;
	//		positions[y * gridSize + x].y = y;
	//	}
	//}
	//
	//
	//KDTree tree;
	//tree.TestConstruct(positions, gridSize * gridSize);
	//
	//const float range = 2.0f;
	//std::vector<int> agents;
	//tree.AgentsInRangeTest(positions, 25, range, 5, agents);


	Environment env;
	env.Initialize(Environment::TestEnvironment::EMPTY);
	ECMPathPlanner planner(env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM(), &planner, &env, 20, 0.1f);
	sim.Initialize();


	// CIRCLE AGENTS
	const int N = 10;
	const float R = 200.0f;
	const float PI = 3.14159265358979323846;
	float angleStep = 2 * PI / N; // Hoek tussen opeenvolgende punten
	for (int i = 0; i < N; ++i) {
		float theta = i * angleStep; // Hoek voor het i-de punt
		float x = R * cos(theta + 0.5f) + 1.0f;
		float y = R * sin(theta + 0.5f) + 1.0f;
	
		sim.SpawnAgent(Point(x, y), Point(-x, -y), 10, 20);
	}
	
	
	//sim.SpawnAgent(Point(0, 400), Point(0, 200), 10, 30);
	//sim.SpawnAgent(Point(0, 200), Point(0, 400), 10, 30);
	//sim.SpawnAgent(Point(-100, 300), Point(100, 300), 10, 30);
	
	//sim.SpawnAgent(Point(-100, 280), Point(100, 300), 10, 30);
	//sim.SpawnAgent(Point(-100, 340), Point(100, 280), 10, 30);

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