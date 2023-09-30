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
		bool Application::InitializeApplication(const char* title, Environment::TestEnvironment environment, int screenWidth, int screenHeight)
		{
			if (!InitializeEnvironment(environment)) return false;
			if (!InitializeWindow(title, screenWidth, screenHeight)) return false;
			if (!InitializeRenderer()) return false;

			m_Planner.Initialize(m_ApplicationState.ecm->GetECMGraph());

			//Point start(716.374329f, 884.503235f);
			//Point end(-387.426941f, 1213.450562f);
			//m_Planner.GetPath(m_ApplicationState.environment, start, end, 25.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw, m_ApplicationState.pathToDraw);
			//m_Planner.GetPath(m_ApplicationState.environment, end, start, 25.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw, m_ApplicationState.pathToDraw);

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
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor;

						m_ApplicationState.cellToDraw = m_ApplicationState.ecm->GetECMCell(worldX, worldY);
					}

					// EVENT: CLICK START/GOAL/RESET PATH
					if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
					{
						float worldX = (e.button.x - m_ApplicationState.camOffsetX) / m_ApplicationState.camZoomFactor;
						float worldY = (e.button.y - m_ApplicationState.camOffsetY) / m_ApplicationState.camZoomFactor * -1.0f; // -1.0 because inversed y-axis (TODO: refactor)

						printf("World (%f, %f)\n", worldX, worldY);

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
							m_Planner.GetPath(m_ApplicationState.environment, m_ApplicationState.pathStartPoint, m_ApplicationState.pathGoalPoint, 25.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw, m_ApplicationState.pathToDraw);

							m_ApplicationState.startPointSelected = false;
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

		bool Application::InitializeWindow(const char* title, int screenWidth, int screenHeight)
		{
			const Environment& env = m_ApplicationState.environment;

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
			if (environment == Environment::TestEnvironment::BIG)
			{
				std::vector<Segment> walkableArea;
				walkableArea.push_back(Segment(-2500, -2500, 2500, -2500));
				walkableArea.push_back(Segment(2500, -2500, 2500, 2500));
				walkableArea.push_back(Segment(-2500, 2500, 2500, 2500));
				walkableArea.push_back(Segment(-2500, 2500, -2500, -2500));
				m_ApplicationState.environment.AddWalkableArea(walkableArea);

				const float obstacleSize = 200.0f;
				const float padding = 200.0f;

				int numObstaclesRow = 5000.0f / (obstacleSize + padding);
				int numObstaclesCol = 5000.0f / (obstacleSize + padding);

				for (int r = 0; r < numObstaclesRow; r++)
				{
					float y = 2500 - padding - r * (obstacleSize + padding);

					for (int c = 0; c < numObstaclesCol; c++)
					{
						float x = -2500 + padding + c * (obstacleSize + padding);

						std::vector<Segment> obstacle{
							Segment(Point(x, y), Point(x + obstacleSize, y)),
								Segment(Point(x + obstacleSize, y), Point(x + obstacleSize, y - obstacleSize)),
								Segment(Point(x + obstacleSize, y - obstacleSize), Point(x, y - obstacleSize)),
								Segment(Point(x, y - obstacleSize), Point(x, y))
						};

						m_ApplicationState.environment.AddObstacle(obstacle);
					}
				}
			}

			// generate ECM from environment
			m_ApplicationState.environment.ComputeECM();
			m_ApplicationState.ecm = m_ApplicationState.environment.GetECM();
		

			return true;
		}



	} // Visualisation
} // ExplicitCorridorMap