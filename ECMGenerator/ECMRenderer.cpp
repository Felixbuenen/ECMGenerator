#include "ECMRenderer.h"
#include "ECM.h"
#include "Environment.h"
#include "ECMCellCollection.h"

#include "SDL.h"
#include "boost/polygon/voronoi.hpp"

#include <stdio.h>

void ECMRenderer::Initialize(int screenWidth, int screenHeight, const char* title, std::shared_ptr<ECM> ecm, Environment* env, ECMRendererColorSettings colorSettings, float zoomFactor)
{
	// set member vars
	_ecm = ecm;
	_env = env;
	_zoomFactor = zoomFactor;

	int bboxW = (env->GetBBOX().max.x - env->GetBBOX().min.x) * zoomFactor;
	int bboxH = (env->GetBBOX().max.y - env->GetBBOX().min.y) * zoomFactor;

	_offsetX = -env->GetBBOX().min.x * zoomFactor + screenWidth * 0.5f - bboxW * 0.5f;
	_offsetY = -env->GetBBOX().min.y * zoomFactor + screenHeight * 0.5f - bboxH * 0.5f;

	_colorSettings = colorSettings;

	//The window we'll be rendering to
	_window = NULL;
	_renderer = NULL;

	//The surface contained by the window
	_screenSurface = NULL;

	InitializeRenderContext(screenWidth, screenHeight, title);

	//Destroy window
	SDL_DestroyWindow(_window);
	SDL_DestroyRenderer(_renderer);

	//Quit SDL subsystems
	SDL_Quit();
}

void ECMRenderer::InitializeRenderContext(int width, int height, const char* title)
{
	//Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
	}
	else
	{
		//Create window
		_window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);
		if (_window == NULL)
		{
			printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		}
		else
		{
			_renderer = SDL_CreateRenderer(_window, -1, 0);

			//Get window surface
			_screenSurface = SDL_GetWindowSurface(_window);


			//Update the surface
			SDL_UpdateWindowSurface(_window);

			Render();

			//Hack to get window to stay up
			SDL_Event e; bool quit = false; while (quit == false) 
			{ 
				while (SDL_PollEvent(&e)) { 
					if (e.type == SDL_QUIT) quit = true; 

					if (e.type == SDL_MOUSEBUTTONDOWN)
					{
						DebugSetDrawECMCell(e.button.x, e.button.y);
					}

					else
					{
						Render();
					}
				} 
			}
		}
	}
}

void ECMRenderer::Render()
{
	DrawBackground();
	DrawWalkableArea();
	DrawObstacles();
	DrawMedialAxis();
	DrawECMVertices();
	//DrawClosestObstaclePoints();
	
	// TEST
	//DrawRandomTestPath();
	//DrawInsideVerts();
	DebugDrawECMCell();
	//DebugDrawSecondaryLines();
	//DebugDrawCellValues();

	// render window
	SDL_RenderPresent(_renderer);
}

void ECMRenderer::DrawBackground()
{
	SDL_SetRenderDrawColor(_renderer, _colorSettings.background_R, _colorSettings.background_G, _colorSettings.background_B, 0xff);
	SDL_RenderClear(_renderer);
}

void ECMRenderer::DrawWalkableArea()
{
	SDL_SetRenderDrawColor(_renderer, _colorSettings.walkableArea_R, _colorSettings.walkableArea_G, _colorSettings.walkableArea_B, 0xff);
	const std::vector<Segment>& walkableArea = _env->GetWalkableArea();

	// this code only works for this specific test case! make sure to encapsulate it in a walkable area struct 
	int x = walkableArea[0].p0.x * _zoomFactor + _offsetX;
	int y = walkableArea[0].p0.y* _zoomFactor + _offsetY;
	int w =  (walkableArea[0].p1.x - walkableArea[0].p0.x);
	int h = (walkableArea[1].p1.y - walkableArea[1].p0.y);

	w *= _zoomFactor;
	h *= _zoomFactor;

	SDL_Rect testWalkableArea;
	testWalkableArea.x = x;
	testWalkableArea.y = y;
	testWalkableArea.w = w;
	testWalkableArea.h = h;

	SDL_RenderDrawRect(_renderer, &testWalkableArea);
	SDL_RenderFillRect(_renderer, &testWalkableArea);
}

void ECMRenderer::DrawObstacles()
{
	SDL_SetRenderDrawColor(_renderer, _colorSettings.obstacle_R, _colorSettings.obstacle_G, _colorSettings.obstacle_B, 0xff);

	const std::vector<std::vector<Segment>>& obstacles = _env->GetObstacles();
	for (const std::vector<Segment>& obstacle : obstacles) {
		for (const Segment& edge : obstacle) {

			int x1 = edge.p0.x * _zoomFactor + _offsetX;
			int y1 = edge.p0.y * _zoomFactor + _offsetY;
			int x2 = edge.p1.x * _zoomFactor + _offsetX;
			int y2 = edge.p1.y * _zoomFactor + _offsetY;

			SDL_RenderDrawLine(_renderer, x1, y1, x2, y2);
		}
	}

	// _ecm->GetObstacles();
}

void ECMRenderer::DrawMedialAxis()
{
	// TODO: loop through all edges. For non-linear edges (arcs), sample the arc.
	auto ma = _ecm->GetMedialAxis();
	ECMGraph& graph = _ecm->GetECMGraph();

	// TODO: add colour to properties
	SDL_SetRenderDrawColor(_renderer, 0x00,0x00, 0x00, 0xff);

	const std::vector<ECMEdge>& edges = graph.GetEdges();
	const std::vector<ECMVertex>& verts = graph.GetVertices();

	for (auto edge : edges)
	{
		if (edge.IsArc())
		{
			SDL_SetRenderDrawColor(_renderer, 0x00, 0xff, 0x00, 0xff);
		}
		else
		{
			SDL_SetRenderDrawColor(_renderer, 0x00, 0x00, 0x00, 0xff);
		}
		int x1 = verts[edge.V0()].Position().x * _zoomFactor + _offsetX;
		int y1 = verts[edge.V0()].Position().y * _zoomFactor + _offsetY;
		int x2 = verts[edge.V1()].Position().x * _zoomFactor + _offsetX;
		int y2 = verts[edge.V1()].Position().y * _zoomFactor + _offsetY;

		SDL_RenderDrawLine(_renderer, x1, y1, x2, y2);
	}
}

void ECMRenderer::DrawRandomTestPath()
{
	SDL_SetRenderDrawColor(_renderer, 0x00, 0xff, 0x00, 0xff);

	std::vector<Segment> path = _ecm->GetRandomTestPath();

	for (const Segment& s : path)
	{
		float x0 = s.p0.x * _zoomFactor + _offsetX;
		float x1 = s.p1.x * _zoomFactor + _offsetX;
		float y0 = s.p0.y * _zoomFactor + _offsetY;
		float y1 = s.p1.y * _zoomFactor + _offsetY;

		SDL_RenderDrawLine(_renderer, x0, y0, x1, y1);
	}
}

void ECMRenderer::DrawECMVertices()
{
	auto verts = _ecm->GetECMGraph().GetVertices();

	SDL_SetRenderDrawColor(_renderer, 0xff, 0x00, 0xff, 0xff);
	//SDL_RenderSetScale(_renderer, 2.0f, 2.0f);

	for (auto vert : verts)
	{
		float x = vert.Position().x * _zoomFactor + _offsetX;
		float y = vert.Position().y * _zoomFactor + _offsetY;

		SDL_RenderDrawPoint(_renderer, x, y);
	}

	int result = SDL_RenderDrawPoint(_renderer, _offsetX, _offsetY);


	//SDL_RenderSetScale(_renderer, 1.0f, 1.0f);

}

void ECMRenderer::DrawInsideVerts()
{
	auto verts = _ecm->GetECMGraph().GetVertices();

	SDL_SetRenderDrawColor(_renderer, 0xff, 0x00, 0x00, 0xff);
	//SDL_RenderSetScale(_renderer, 2.0f, 2.0f);

	for (auto vert : verts)
	{
		if (_env->InsideObstacle(vert.Position()))
		{
			float x = vert.Position().x * _zoomFactor + _offsetX;
			float y = vert.Position().y * _zoomFactor + _offsetY;

			const int size = 5;

			SDL_RenderDrawLine(_renderer, (x - size), y, (x + size), y);
			SDL_RenderDrawLine(_renderer, (x), (y- size), x, (y+ size));
		}
	}

	//SDL_RenderSetScale(_renderer, 1.0f, 1.0f);
}

void ECMRenderer::DrawClosestObstaclePoints()
{
	auto verts = _ecm->GetECMGraph().GetVertices();
	SDL_SetRenderDrawColor(_renderer, 0xff, 0x8c, 0x00, 0xff);

	for (const auto& vert : verts)
	{
		float startX = vert.Position().x * _zoomFactor + _offsetX;
		float startY = vert.Position().y * _zoomFactor + _offsetY;

		const auto& clPoints = vert.GetClosestPoints();
		for (const auto& clPoint : clPoints)
		{
			float x = clPoint.x * _zoomFactor + _offsetX;
			float y = clPoint.y * _zoomFactor + _offsetY;

			SDL_RenderDrawLine(_renderer, startX, startY, x, y);
		}
	}
}

void ECMRenderer::DebugDrawECMCell()
{
	if (!cellToDraw) return;

	SDL_SetRenderDrawColor(_renderer, 0xc4, 0x77, 0x02, 0xff);

	const std::vector<Segment>& segs = cellToDraw->boundary;

	for (const Segment& s : segs)
	{
		float x0 = s.p0.x * _zoomFactor + _offsetX;
		float x1 = s.p1.x * _zoomFactor + _offsetX;
		float y0 = s.p0.y * _zoomFactor + _offsetY;
		float y1 = s.p1.y * _zoomFactor + _offsetY;
		SDL_RenderDrawLine(_renderer, x0, y0, x1, y1);
	}
}


void ECMRenderer::DebugSetDrawECMCell(float screenX, float screenY)
{
	float worldX = (screenX - _offsetX) / _zoomFactor;
	float worldY = (screenY - _offsetY) / _zoomFactor;

	cellToDraw = _ecm->GetECMCell(worldX, worldY);
}

void ECMRenderer::DebugDrawSecondaryLines()
{
	using boost::polygon::voronoi_diagram;
	const auto& vd = _ecm->GetMedialAxis()->VD;
	
	SDL_SetRenderDrawColor(_renderer, 0x55, 0x55, 0xff, 0xff);

	for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
		vd.edges().begin(); it != vd.edges().end(); ++it)
	{
		if (it->is_finite() && it->is_secondary())
		{
			float x0 = it->vertex0()->x() * _zoomFactor + _offsetX;
			float x1 = it->vertex1()->x() * _zoomFactor + _offsetX;
			float y0 = it->vertex0()->y() * _zoomFactor + _offsetY;
			float y1 = it->vertex1()->y() * _zoomFactor + _offsetY;

			SDL_RenderDrawLine(_renderer, x0, y0, x1, y1);
		}
	}
}

void ECMRenderer::DebugDrawCellValues()
{
	using boost::polygon::voronoi_diagram;
	const auto& vd = _ecm->GetMedialAxis()->VD;

	SDL_SetRenderDrawColor(_renderer, 0x55, 0x55, 0xff, 0xff);

	for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
		vd.edges().begin(); it != vd.edges().end(); ++it)
	{
		if (it->cell()->contains_point()) continue;

		if (it->is_finite() && it->is_secondary())
		{
			// TODO:
			// > this demonstrates that we can find the closest segments in constant time.
			// > update the ecm construction algorithm such that it utilizes this information.
			int segmentIndex = it->cell()->source_index();

			const Segment& s = _env->GetEnvironmentObstacleUnion()[segmentIndex];

			float x0 = s.p0.x * _zoomFactor + _offsetX;
			float x1 = s.p1.x * _zoomFactor + _offsetX;
			float y0 = s.p0.y * _zoomFactor + _offsetY;
			float y1 = s.p1.y * _zoomFactor + _offsetY;
			
			SDL_RenderDrawLine(_renderer, x0, y0, x1, y1);
		}
	}
}
