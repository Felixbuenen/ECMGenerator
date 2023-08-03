#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"

#include <stdio.h>

int main()
{
	using ECM::ECM;
	using namespace ECM;

	WindowApplication::Application app;
	app.InitializeApplication("ECM generation tool", Environment::TestEnvironment::CLASSIC, 1080, 720);
	app.Run();
	app.Clear();

	return 0;
}