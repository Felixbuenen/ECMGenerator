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
	if (!app.InitializeApplication("ECM generation tool", Environment::TestEnvironment::BIG, 1080, 720))
	{
		printf("ERROR: could not initialize ECM applciation.\n");
		return -1;
	}
	app.Run();
	app.Clear();

	return 0;
}