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
	env.Initialize(Environment::TestEnvironment::SQUARE);
	ECMPathPlanner planner(env.GetECM()->GetECMGraph());
	Simulator sim(env.GetECM().get(), &planner, &env, 50, 0.25f);
	sim.Initialize();

	//sim.SpawnAgent(Point(-227, -138), Point(127, -300), 10.0f, 20.0f);

	//std::vector<Point> obst;
	//obst.push_back(Point(5, 0));
	//obst.push_back(Point(100,0));
	//env.AddObstacle(obst);
	//
	//sim.SpawnAgent(Point(0, 100), Point(0, -100), 10.0f, 2.0f);


	// CIRCLE AGENTS
	//const int N = 20;
	//const float R = 250.0f;
	//const float PI = 3.14159265358979323846;
	//float angleStep = 2 * PI / N; // Hoek tussen opeenvolgende punten
	//for (int i = 0; i < N; ++i) {
	//	float theta = i * angleStep; // Hoek voor het i-de punt
	//	float x = R * cos(theta + 0.5f) + 1.0f;
	//	float y = R * sin(theta + 0.5f) + 1.0f;
	//	
	//	sim.SpawnAgent(Point(x, y), Point(-x, -y), 10, 2);
	//}

	//RVO rvo;
	
	//Constraint c;
	//c.Init(Point(9.64906502f, 5.27687311f), Point(0.877230167f, 0.480070025f));
	//std::vector<Constraint> constraints;
	//constraints.push_back(c);
	//Vec2 testResult(-0.87569201f, -0.482870013f);
	//rvo.RandomizedLP3D(1, constraints, 2, 0, testResult);
	
	//Constraint c1;
	//Constraint c2;
	//Constraint c3;
	//Vec2 n1 = Utility::MathUtility::Left(Vec2(0.600814700, -0.799388230));
	//Vec2 n2 = Utility::MathUtility::Left(Vec2(-0.584312618, -0.811528683));
	//Vec2 n3 = Utility::MathUtility::Left(Vec2(0.968583643, -0.248688012));
	//Vec2 p1 = Vec2(-3.81258988, 5.18064976);
	//Vec2 p2 = Vec2(1.89454556, -0.667740703);
	//Vec2 p3 = Vec2(0.525458097, -4.39200497);
	//std::vector<Constraint> constraints;
	//
	//c1.Init(p1, n1);
	//c2.Init(p2, n2);
	//c3.Init(p3, n3);
	//constraints.push_back(c1);
	//constraints.push_back(c2);
	//constraints.push_back(c3);
	//Vec2 result(1.22587729, -1.52306712);
	//
	//int error = rvo.RandomizedLP(3, constraints, Vec2(5,5), 20.0f, false, result);

	//sim.SpawnAgent(Point(0, 400), Point(0, 200), 10, 30);
	//sim.SpawnAgent(Point(0, 200), Point(0, 400), 10, 30);
	//sim.SpawnAgent(Point(-100, 300), Point(100, 300), 10, 30);

	//sim.SpawnAgent(Point(-100, 280), Point(100, 300), 10, 30);
	//sim.SpawnAgent(Point(-100, 340), Point(100, 280), 10, 30);

	//for (int i = 0; i < 20; i++)
	//{
	//	sim.SpawnAgent(Point(-100 + 50 * i, 200), Point(-100 + 50 * i, -200), 10, 30);
	//	sim.SpawnAgent(Point(-100 + 55 * i, -200), Point(-100 + 55 * i, 200), 10, 30);
	//}

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