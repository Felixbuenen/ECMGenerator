#include "Application.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECMGenerator.h"
#include "ECM.h"
#include "ECMPathPlanner.h"
#include "Timer.h"

#include "SDL.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"

namespace ECM
{
	namespace WindowApplication
	{
		bool Application::InitializeApplication(const char* title, int screenWidth, int screenHeight)
		{
			m_ApplicationState.ecm = m_ApplicationState.environment->GetECM();

			if (!InitializeWindow(title, screenWidth, screenHeight)) return false;
			if (!InitializeRenderer()) return false;

			// Setup Dear ImGui context
			IMGUI_CHECKVERSION();
			ImGui::CreateContext();
			ImGuiIO& io = ImGui::GetIO(); (void)io;
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
			
			// Setup Dear ImGui style
			ImGui::StyleColorsDark();
			//ImGui::StyleColorsLight();
			
			// Setup Platform/Renderer backends
			ImGui_ImplSDL2_InitForSDLRenderer(m_Window, m_Renderer.GetSDLRenderer());
			ImGui_ImplSDLRenderer2_Init(m_Renderer.GetSDLRenderer());

			return true;
		}

		void Application::Run()
		{
			Uint64 currentTime = SDL_GetPerformanceCounter();
			Uint64 lastTime = 0;
			double deltaTime = 0.0;
			const double timeScale = 1.0 / SDL_GetPerformanceFrequency();

			// start main loop
			SDL_Event e; bool quit = false; while (quit == false)
			{
				// update timing information
				lastTime = currentTime;
				currentTime = SDL_GetPerformanceCounter();
				deltaTime = (currentTime - lastTime) * timeScale;

				// HANDLE INPUT
				while (SDL_PollEvent(&e)) {
					ImGui_ImplSDL2_ProcessEvent(&e);

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
							m_Planner->GetPath(*m_ApplicationState.environment, m_ApplicationState.pathStartPoint, m_ApplicationState.pathGoalPoint, 20.0f, 0.0f, m_ApplicationState.corridorToDraw, m_ApplicationState.portalsToDraw, m_ApplicationState.pathToDraw);

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
				
				// Start the Dear ImGui frame
				ImGui_ImplSDLRenderer2_NewFrame();
				ImGui_ImplSDL2_NewFrame();
				ImGui::NewFrame();
				ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

				bool j = true;
				ImGui::ShowDemoWindow(&j);

				// 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
				{
					static float f = 0.0f;
					static int counter = 0;

					ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

					ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

					ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
					ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

					if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
						counter++;
					ImGui::SameLine();
					ImGui::Text("counter = %d", counter);

					ImGui::End();
				}


				// Rendering
				SDL_RenderClear(m_Renderer.GetSDLRenderer());
				//SDL_RenderSetScale(m_Renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
				//SDL_SetRenderDrawColor(renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));

				m_Renderer.Render();
				ImGui::Render();

				ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

				SDL_RenderPresent(m_Renderer.GetSDLRenderer());

				// TODO:
				// add user interface and real-time stats information.
				//printf("FPS: %f\n", 1.0f/deltaTime);
			}

			// Cleanup
			ImGui_ImplSDLRenderer2_Shutdown();
			ImGui_ImplSDL2_Shutdown();
			ImGui::DestroyContext();
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
			if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) < 0)
			{
				printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
				return false;
			}
			else
			{
				//Create window
				SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
				m_Window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, window_flags);
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