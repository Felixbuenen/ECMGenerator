#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECM.h"

#include <stdio.h>

int main()
{
	// initialize the environment
	std::vector<ECM::Segment> walkableArea;
	walkableArea.push_back(ECM::Segment(-500, -500, 500, -500));
	walkableArea.push_back(ECM::Segment(500, -500, 500, 500));
	walkableArea.push_back(ECM::Segment(-500, 500, 500, 500));
	walkableArea.push_back(ECM::Segment(-500, 500, -500, -500));

	std::vector<ECM::Segment> obstacle{
		ECM::Segment(-200, -250, -200, 250),
			ECM::Segment(-200, 250, 200, 250),
			ECM::Segment(200, 250, 200, -250),
			ECM::Segment(200, -250, 100, -250),
			ECM::Segment(100, -250, 100, 150),
			ECM::Segment(100, 150, -100, 150),
			ECM::Segment(100, 150, -100, 150),
			ECM::Segment(-100, 150, -100, -250),
			ECM::Segment(-100, -250, -200, -250)
	};

	ECM::Environment env;
	env.AddWalkableArea(walkableArea);
	env.AddObstacle(obstacle);

	ECM::ECMGenerator gen;
	std::shared_ptr<ECM::ECM> ecm = gen.GenerateECM(env);

	printf("Starting render loop... \n");
	ECM::WindowApplication::ECMRenderer ecmRenderer;
	ECM::WindowApplication::ECMRendererColorSettings colorSettings;
	ecmRenderer.Initialize(1080, 720, "ECM generation tool", ecm, &env, colorSettings, 0.65f);
	printf("ECM application stopped.\n");

	return 0;
}