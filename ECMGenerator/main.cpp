#include "ECMGenerator.h"
#include "ECMRenderer.h"
#include "Environment.h"
#include "UtilityFunctions.h"

#include <stdio.h>

int main()
{
	// initialize the environment
	std::vector<Segment> walkableArea;
	walkableArea.push_back(Segment(-500, -500, 500, -500));
	walkableArea.push_back(Segment(500, -500, 500, 500));
	walkableArea.push_back(Segment(-500, 500, 500, 500));
	walkableArea.push_back(Segment(-500, 500, -500, -500));

	std::vector<Segment> obstacle{
		Segment(-200, -250, -200, 250),
		Segment(-200, 250, 200, 250),
		Segment(200, 250, 200, -250),
		Segment(200, -250, 100, -250),
		Segment(100, -250, 100, 150),
		Segment(100, 150, -100, 150),
		Segment(100, 150, -100, 150),
		Segment(-100, 150, -100, -250),
		Segment(-100, -250, -200, -250)
	};

	Environment env;
	env.AddWalkableArea(walkableArea);
	env.AddObstacle(obstacle);

	ECMGenerator gen;
	std::shared_ptr<ECM> ecm = gen.GenerateECM(env);

	printf("Starting render loop... \n");
	ECMRenderer ecmRenderer;
	ECMRendererColorSettings colorSettings;
	ecmRenderer.Initialize(1080, 720, "ECM generation tool", ecm, &env, colorSettings, 0.65f);
	printf("ECM application stopped.\n");

	return 0;
}