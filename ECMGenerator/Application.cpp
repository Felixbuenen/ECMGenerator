#include "Application.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECMGenerator.h"
#include "ECM.h"
#include "ECMPathPlanner.h"

#include "SDL.h"

namespace ECM
{
	namespace WindowApplication
	{
		bool Application::InitializeApplication(const char* title, Environment::TestEnvironment environment, int screenWidth, int screenHeight, float zoomFactor)
		{
			if (!InitializeEnvironment(environment)) return false;
			if (!InitializeWindow(title, screenWidth, screenHeight, zoomFactor)) return false;
			if (!InitializeRenderer()) return false;

			m_Planner.Initialize(m_ApplicationState.ecm->GetECMGraph());

			return true;
		}

		void Application::Run()
		{

			//Update the surface
			SDL_UpdateWindowSurface(m_Window);

			m_Renderer.Render();

			// start main loop
			SDL_Event e; bool quit = false; while (quit == false)
			{
				while (SDL_PollEvent(&e)) {
					if (e.type == SDL_QUIT) quit = true;

					// EVENT: CLICK CELL
					if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT)
					{
						float worldX = (e.button.x - m_ApplicationState.camOffsetX) / m_ApplicationState.camZoomFactor;
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor;

						printf("world X: %f\n", worldX);
						printf("world Y: %f\n", worldY);

						m_ApplicationState.cellToDraw = m_ApplicationState.ecm->GetECMCell(worldX, worldY);
					}

					// EVENT: CLICK START/GOAL/RESET PATH
					if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
					{
						float worldX = (e.button.x - m_ApplicationState.camOffsetX) / m_ApplicationState.camZoomFactor;
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor * -1.0f; // -1.0 because inversed y-axis (TODO: refactor)

						if (!m_ApplicationState.pathToDraw.empty())
						{
							m_ApplicationState.pathToDraw.clear();
							m_ApplicationState.pathStartPoint = Point(worldX, worldY);
						}
						else
						{
							m_ApplicationState.pathGoalPoint = Point(worldX, worldY);
							m_ApplicationState.corridorToDraw = PathPlanning::Corridor();
							m_ApplicationState.portalsToDraw = std::vector<Segment>();		
							m_ApplicationState.pathToDraw = m_Planner.GetPath(m_ApplicationState.environment, m_ApplicationState.pathStartPoint, m_ApplicationState.pathGoalPoint, 25.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw);
						}
					}

					else
					{
						m_Renderer.Render();
					}
				}
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

		bool Application::InitializeWindow(const char* title, int screenWidth, int screenHeight, float zoomFactor)
		{
			m_ApplicationState.camZoomFactor = zoomFactor;

			const Environment& env = m_ApplicationState.environment;
			int bboxW = (env.GetBBOX().max.x - env.GetBBOX().min.x) * zoomFactor;
			int bboxH = (env.GetBBOX().max.y - env.GetBBOX().min.y) * zoomFactor;

			m_ApplicationState.camOffsetX = -env.GetBBOX().min.x * zoomFactor + screenWidth * 0.5f - bboxW * 0.5f;
			m_ApplicationState.camOffsetY = -env.GetBBOX().min.y * zoomFactor + screenHeight * 0.5f - bboxH * 0.5f;

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



		bool Application::InitializeEnvironment(Environment::TestEnvironment environment)
		{
			// setup environment
			if (environment == Environment::TestEnvironment::CLASSIC)
			{
				std::vector<Segment> walkableArea;
				walkableArea.push_back(Segment(-500, -500, 500, -500));
				walkableArea.push_back(Segment(500, -500, 500, 500));
				walkableArea.push_back(Segment(-500, 500, 500, 500));
				walkableArea.push_back(Segment(-500, 500, -500, -500));

				std::vector<Segment> obstacle{
					Segment(-200, 250, -200, -250),
						Segment(-200, -250, 200, -250),
						Segment(200, -250, 200, 250),
						Segment(200, 250, 100, 250),
						Segment(100, 250, 100, -150),
						Segment(100, -150, -100, -150),
						Segment(100, -150, -100, -150),
						Segment(-100, -150, -100, 250),
						Segment(-100, 250, -200, 250)
				};

				m_ApplicationState.environment.AddWalkableArea(walkableArea);
				m_ApplicationState.environment.AddObstacle(obstacle);
			}

			// generate ECM from environment
			m_ApplicationState.environment.ComputeECM();
			m_ApplicationState.ecm = m_ApplicationState.environment.GetECM();

			return true;
		}



	} // Visualisation
} // ExplicitCorridorMap