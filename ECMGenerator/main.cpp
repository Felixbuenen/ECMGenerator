#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"
#include "ECMBenchmark.h"

#include <stdio.h>

int main()
{
	//using ECM::ECM;
	//using namespace ECM;
	//
	//WindowApplication::Application app;
	//if (!app.InitializeApplication("ECM generation tool", Environment::TestEnvironment::BIG, 1080, 720))
	//{
	//	printf("ERROR: could not initialize ECM applciation.\n");
	//	return -1;
	//}
	//app.Run();
	//app.Clear();

	using ECM::ECM;
	using namespace ECM;

	ECMBenchmark benchmark;
	//Environment env;
	benchmark.InitTestEnvironment(Environment::TestEnvironment::BIG);
	
	Point start(-2404.970947f, -2353.801025f);
	Point end(1527.777832f, 2404.971191f);
	benchmark.PerformPathPlanTest(start, end, 15.0f, 50000);


	//WindowApplication::Application app;
	//if (!app.InitializeApplication("ECM generation tool", env, 1080, 720))
	//{
	//	printf("ERROR: could not initialize ECM applciation.\n");
	//	return -1;
	//}
	//app.Run();
	//app.Clear();

	return 0;
}