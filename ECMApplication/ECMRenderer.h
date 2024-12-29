#pragma once

#include <memory>
#include <vector>
#include <stdint.h>

class SDL_Window;
class SDL_Renderer;
class SDL_Surface;

namespace ECM {

	class ECM;
	class Environment;
	struct Segment;
	struct ECMCell;
	struct Point;

	namespace WindowApplication {

		class Application;
		struct ApplicationState;

		enum SimAreaDrag;

		class ECMRenderer
		{
		public:
			struct ECMRendererColorSettings
			{
				int background_R = 0x22, background_G = 0x22, background_B = 0x22;
				int walkableArea_R = 0xff, walkableArea_G = 0xff, walkableArea_B = 0xff;
				int obstacle_R = 0xaa, obstacle_G = 0x11, obstacle_B = 0x11;
				// etc....
			};

		public:
			void Initialize(Application* app);
			void Render();
			void Clear();

			// TODO: these methods are not used everywhere. Refactor.
			Point ScreenToWorldCoordinates(float x, float y);
			Point WorldToScreenCoordinates(float x, float y);

			void RenderDragSimulationArea(float x, float y, SimAreaDrag areaType);
			void StopRenderDragSimulationArea();

		private:
			void UpdateRenderState();

			// different draw calls
			void SetupSDLWindow();
			void DrawBackground();
			void DrawWalkableArea();
			void DrawObstacles();
			void DrawMedialAxis();
			void DrawECMVertices();
			void HighlightECMVertex(int idx);
			void DrawPath(); // bij deze even kijken hoe ik dat ga doen, want het pad is in principe geen onderdeel van ECM, wellicht met ref van pad
			void DrawRandomTestPath();
			void DrawInsideVerts();
			void DrawClosestObstaclePoints();
			void DrawCorridor();
			void DrawPortals();
			void DrawHalfEdge(int idx);
			void DrawAttractionPoints();

			void DebugDrawECMCell();
			void DebugDrawSecondaryLines();
			void DebugDrawCellValues();
			void DebugDrawBoostVoronoiDiagram();
			void DrawSimulationAreas();
			void InternalDrawDragSimulationArea();
			void DebugDrawKNearestNeighbors(int idx);

			void DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius);

			// SIMULATION
			void DrawAgents();
			void DrawPaths();

		private:
			std::shared_ptr<ECM> m_Ecm;
			Environment* m_Env;
			ECMRendererColorSettings m_ColorSettings;

			float m_CamZoomFactor;
			float m_YRotation;
			int m_CamOffsetX, m_CamOffsetY;

			ApplicationState* m_AppState;
			SDL_Renderer* m_Renderer;

		};


	}
}