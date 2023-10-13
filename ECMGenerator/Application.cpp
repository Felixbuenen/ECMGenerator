#include "Application.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECMGenerator.h"
#include "ECM.h"
#include "ECMPathPlanner.h"
#include "Timer.h"

#include "SDL.h"

namespace ECM
{
	namespace WindowApplication
	{
		bool Application::InitializeApplication(const char* title, int screenWidth, int screenHeight)
		{
			m_ApplicationState.ecm = m_ApplicationState.environment->GetECM();

			if (!InitializeWindow(title, screenWidth, screenHeight)) return false;
			if (!InitializeRenderer()) return false;

			return true;
		}

		void Application::Run()
		{
			//Update the surface
			SDL_UpdateWindowSurface(m_Window);

			Uint64 currentTime = SDL_GetPerformanceCounter();
			Uint64 lastTime = 0;
			double deltaTime = 0.0;
			const double timeScale = 1.0 / SDL_GetPerformanceFrequency();

			// init simulation
			{
				//Timer timer("Simulator::Initialize()");
				m_ApplicationState.simulator->Initialize();
			}

			// start main loop
			SDL_Event e; bool quit = false; while (quit == false)
			{
				// update timing information
				lastTime = currentTime;
				currentTime = SDL_GetPerformanceCounter();
				deltaTime = (currentTime - lastTime) * timeScale;

				// HANDLE INPUT
				while (SDL_PollEvent(&e)) {

					if (e.type == SDL_QUIT) quit = true;

					if (e.type == SDL_KEYDOWN)
					{
						if (e.key.keysym.sym == SDLK_a)
						{
							m_ApplicationState.camOffsetX += 10.0f;
						}
						if (e.key.keysym.sym == SDLK_d)
						{
							m_ApplicationState.camOffsetX -= 10.0f;
						}
						if (e.key.keysym.sym == SDLK_w)
						{
							m_ApplicationState.camOffsetY += 10.0f;
						}
						if (e.key.keysym.sym == SDLK_s)
						{
							m_ApplicationState.camOffsetY -= 10.0f;
						}
					}

					if (e.type == SDL_MOUSEWHEEL)
					{
						m_ApplicationState.camZoomFactor += 0.05f * e.wheel.y;
					}

					// EVENT: CLICK CELL
					if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT)
					{
						float worldX = (e.button.x - m_ApplicationState.camOffsetX) / m_ApplicationState.camZoomFactor;
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor * -1.0f;

						m_ApplicationState.cellToDraw = m_ApplicationState.ecm->GetECMCell(worldX, worldY);
					}

					// EVENT: CLICK START/GOAL/RESET PATH
					if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
					{
						float worldX = (e.button.x - m_ApplicationState.camOffsetX) / m_ApplicationState.camZoomFactor;
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor * -1.0f; // -1.0 because inversed y-axis (TODO: refactor)

						printf("(%f, %f)\n", worldX, worldY);

						if (!m_ApplicationState.startPointSelected)
						{
							m_ApplicationState.startPointSelected = true;
							m_ApplicationState.pathToDraw.clear();
							m_ApplicationState.pathStartPoint = Point(worldX, worldY);
						}
						else
						{
							m_ApplicationState.pathGoalPoint = Point(worldX, worldY);
							m_ApplicationState.corridorToDraw = PathPlanning::Corridor();
							m_ApplicationState.portalsToDraw = std::vector<Segment>();

							m_ApplicationState.pathToDraw = PathPlanning::Path();
							m_Planner->GetPath(*m_ApplicationState.environment, m_ApplicationState.pathStartPoint, m_ApplicationState.pathGoalPoint, 20.0f, 40.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw, m_ApplicationState.pathToDraw);

							m_ApplicationState.startPointSelected = false;
						}
					}
				}
			
				// UPDATE SIMULATION
				// todo.. create a simulation object and update all positions
				{
					//Timer timer("SIMULATION");
					m_ApplicationState.simulator->Update(deltaTime);
				}
				
				// RENDER
				m_Renderer.Render();

				// TODO:
				// add user interface and real-time stats information.
				//printf("FPS: %f\n", 1.0f/deltaTime);
			}


		}

		void Application::Clear()
		{
			//Destroy window
			SDL_DestroyWindow(m_Window);
			m_Renderer.Clear();

			//Quit SDL subsystems
			SDL_Quit();
		}

		bool Application::InitializeRenderer()
		{
			m_Renderer.Initialize(this);
			return true;
		}

		bool Application::InitializeWindow(const char* title, int screenWidth, int screenHeight)
		{
			const Environment& env = *m_ApplicationState.environment;

			float bboxW = (env.GetBBOX().max.x - env.GetBBOX().min.x);
			float bboxH = (env.GetBBOX().max.y - env.GetBBOX().min.y);
			float zoomW = screenWidth / bboxW;
			float zoomH = screenHeight / bboxH;
			m_ApplicationState.camZoomFactor = zoomW < zoomH ? zoomW : zoomH;
			m_ApplicationState.camZoomFactor *= 0.95f;

			m_ApplicationState.camZoomFactor = m_ApplicationState.camZoomFactor;

			bboxW *= m_ApplicationState.camZoomFactor;
			bboxH *= m_ApplicationState.camZoomFactor;

			m_ApplicationState.camOffsetX = -env.GetBBOX().min.x * m_ApplicationState.camZoomFactor + screenWidth * 0.5f - bboxW * 0.5f;
			m_ApplicationState.camOffsetY = -env.GetBBOX().min.y * m_ApplicationState.camZoomFactor + screenHeight * 0.5f - bboxH * 0.5f;

			//Initialize SDL
			if (SDL_Init(SDL_INIT_VIDEO) < 0)
			{
				printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
				return false;
			}
			else
			{
				//Create window
				m_Window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, SDL_WINDOW_SHOWN);
				if (m_Window == NULL)
				{
					printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
					return false;
				}
			}

			return true;
		}


	} // Visualisation
} // ExplicitCorridorMap