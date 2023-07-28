#pragma once

#include <memory>
#include <vector>
#include <stdint.h>

class ECM;
class Environment;
struct Segment;

class SDL_Window;
class SDL_Renderer;
class SDL_Surface;

struct ECMRendererColorSettings
{
	int background_R = 0x22, background_G = 0x22, background_B = 0x22;
	int walkableArea_R = 0xff, walkableArea_G = 0xff, walkableArea_B = 0xff;
	int obstacle_R = 0xaa, obstacle_G = 0x11, obstacle_B = 0x11;
	// etc....
};

class ECMRenderer
{
public:
	void Initialize(int screenWidth, int screenHeight, const char* title, std::shared_ptr<ECM> ecm, Environment* env, ECMRendererColorSettings colorSettings, float zoomFactor);
	void Refresh();

private:
	void InitializeRenderContext(int width, int height, const char* title);
	void Render(); // (re-)render the window

	// different draw calls
	void SetupSDLWindow();
	void DrawBackground();
	void DrawWalkableArea();
	void DrawObstacles();
	void DrawMedialAxis();
	void DrawECMVertices();
	void DrawPath(); // bij deze even kijken hoe ik dat ga doen, want het pad is in principe geen onderdeel van ECM, wellicht met ref van pad
	void DrawRandomTestPath();
	void DrawInsideVerts();
	void DrawClosestObstaclePoints();

	void DebugSetDrawECMCell(float screenX, float screenY);
	void DebugDrawECMCell();

private:
	std::shared_ptr<ECM> _ecm;
	Environment* _env;
	ECMRendererColorSettings _colorSettings;

	float _zoomFactor;
	int _offsetX, _offsetY;

	// SDL member variables
private:
	SDL_Window* _window;
	SDL_Renderer* _renderer;
	SDL_Surface* _screenSurface;

	// testy testy
	std::vector<Segment> cellToDraw;
};

