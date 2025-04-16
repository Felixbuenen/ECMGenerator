#pragma once

#include <memory>
#include <vector>
#include <stdint.h>

#include "ECMDataTypes.h"

class SDL_Window;
class SDL_Renderer;
class SDL_Surface;
class SDL_Texture;

namespace ECM {

	class ECM;
	class Environment;
	struct Segment;
	struct ECMCell;
	struct Point;

	namespace Simulation
	{
		enum SimAreaType;
		struct Area;
	}

	namespace WindowApplication {

		class Application;
		class Gizmo;
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

			void RenderDragArea(Simulation::SimAreaType type, const Point& screenCoordinates, const Vec2& halfSize);
			void RenderDragConnection(const Point& sourceAreaWorldPosition, const Simulation::Area* hoverArea);
			void RenderGizmo(Gizmo* gizmo);
			void DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius);

			void GetTextureFromBMP(const char* pathToBMP, SDL_Texture** outTexture) const;

			float GetCamZoomFactor() const { return m_CamZoomFactor; }

			Point ScreenToWorldCoordinates(float x, float y) const;
			Point WorldToScreenCoordinates(float x, float y) const;

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
			void DrawPath();
			void DrawRandomTestPath();
			void DrawInsideVerts();
			void DrawClosestObstaclePoints();
			void DrawCorridor();
			void DrawPortals();
			void DrawHalfEdge(int idx);
			void DrawAttractionPoints();
			void DrawAreaSelectionBounds();
			void DrawSimulationAreaConnections();

			void DebugDrawECMCell();
			void DebugDrawSecondaryLines();
			void DebugDrawCellValues();
			void DebugDrawBoostVoronoiDiagram();
			void DebugDrawVertices();
			void DebugDrawRetractionPoint();

			void DrawSimulationAreas();

			// SIMULATION
			void DrawAgents();
			void DrawPaths();

		private:
			Environment* m_Env;
			ECM* m_Ecm;
			ECMRendererColorSettings m_ColorSettings;

			float m_CamZoomFactor;
			float m_YRotation;
			int m_CamOffsetX, m_CamOffsetY;

			ApplicationState* m_AppState;
			SDL_Renderer* m_Renderer;

		};


	}
}