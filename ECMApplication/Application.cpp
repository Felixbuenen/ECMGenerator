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
		bool Application::InitializeApplication(const char* title, bool fullScreen, int screenWidth, int screenHeight)
		{
			m_ApplicationState.ecm = m_ApplicationState.environment->GetECM();

			if (!InitializeWindow(title, fullScreen, screenWidth, screenHeight)) return false;
			
			m_Renderer = SDL_CreateRenderer(m_Window, -1, SDL_RENDERER_ACCELERATED);
			if (!InitializeRenderer()) return false;

			m_UI.Initialize(this);

			return true;
		}

		void Application::Run()
		{
			Uint64 currentTime = SDL_GetPerformanceCounter();
			Uint64 lastTime = 0;
			m_DeltaTime = 0.0;
			const double timeScale = 1.0 / SDL_GetPerformanceFrequency();

			// start main loop
			SDL_Event e; while (m_Quit == false)
			{
				// update timing information
				lastTime = currentTime;
				currentTime = SDL_GetPerformanceCounter();
				float oldDT = m_DeltaTime;
				m_DeltaTime = (currentTime - lastTime) * timeScale;

				// HANDLE INPUT
				while (SDL_PollEvent(&e)) {
					if (!HandleInput(e)) break;
				}
				
				// UPDATE APPLICATION
				Update(e);
				
				// RENDER APPLICATION
				Render();
			}
		}

		void Application::Clear()
		{
			m_UI.CleanUp();
			m_ECMRenderer.Clear();

			//Destroy window
			SDL_DestroyWindow(m_Window);
			SDL_DestroyRenderer(m_Renderer);

			//Quit SDL subsystems
			SDL_Quit();
		}

		bool Application::InitializeRenderer()
		{
			m_ECMRenderer.Initialize(this);

			return true;
		}

		bool Application::HandleInput(SDL_Event& e)
		{
			// first input layer: UI
			m_UI.HandleInput(e);
			m_EnvEditor.HandleInput(e);

			// second input layer: Application
			if (ImGui::GetIO().WantCaptureMouse) return false;
			HandleMouseEvent(e);

			if (ImGui::GetIO().WantCaptureKeyboard) return false;
			HandleKeyEvent(e);

			return true;
		}

		void Application::Update(SDL_Event& event)
		{
			// UPDATE SIMULATION
			{
				//Timer timer("SIMULATION");
				if (m_ApplicationState.simulationPlaying)
				{
					m_ApplicationState.simulator->Update(m_DeltaTime);
				}
			}
			m_EnvEditor.Update();
			m_UI.Update(event);
		}

		void Application::Render()
		{
			SDL_RenderClear(m_Renderer);

			m_ECMRenderer.Render();
			m_EnvEditor.Render();
			m_UI.Render();
			
			SDL_RenderPresent(m_Renderer);
		}


		bool Application::InitializeWindow(const char* title, bool fullScreen, int screenWidth, int screenHeight)
		{
			//Initialize SDL
			if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) < 0)
			{
				printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
				return false;
			}
			else
			{
				//Create window
				SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
				m_Window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, window_flags);

				if (m_Window == NULL)
				{
					printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
					return false;
				}

			}

			if (fullScreen)
			{
				if (fullScreen) SDL_MaximizeWindow(m_Window);

				SDL_Surface* surface = SDL_GetWindowSurface(m_Window);
				screenWidth = surface->w;
				screenHeight = surface->h;
			}
			
			// setup environment variables
			const Environment& env = *m_ApplicationState.environment;

			float bboxW = (env.GetBBOX().max.x - env.GetBBOX().min.x);
			float bboxH = (env.GetBBOX().max.y - env.GetBBOX().min.y);
			float zoomW = screenWidth / bboxW;
			float zoomH = screenHeight / bboxH;
			m_ApplicationState.camZoomFactor = zoomW < zoomH ? zoomW : zoomH;
			m_ApplicationState.camZoomFactor *= 0.9;

			m_ApplicationState.camZoomFactor = m_ApplicationState.camZoomFactor;

			bboxW *= m_ApplicationState.camZoomFactor;
			bboxH *= m_ApplicationState.camZoomFactor;

			m_ApplicationState.camOffsetX = -env.GetBBOX().min.x * m_ApplicationState.camZoomFactor + screenWidth * 0.5f - bboxW * 0.5f;
			m_ApplicationState.camOffsetY = -env.GetBBOX().min.y * m_ApplicationState.camZoomFactor + screenHeight * 0.5f - bboxH * 0.5f;


			return true;
		}

		void Application::HandleMouseEvent(SDL_Event& e)
		{
			if (e.type == SDL_QUIT) m_Quit = true;

			if (e.type == SDL_MOUSEWHEEL)
			{
				m_ApplicationState.camZoomFactor += 0.05f * e.wheel.y;
			}

			// EVENT: CLICK CELL
			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT)
			{
				Point worldCoords = m_ECMRenderer.ScreenToWorldCoordinates(e.button.x, e.button.y);

				m_ApplicationState.cellToDraw = m_ApplicationState.ecm->GetECMCell(worldCoords.x, worldCoords.y);
			}

		}

		void Application::HandleKeyEvent(SDL_Event& e)
		{
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
		}



	} // Visualisation
} // ExplicitCorridorMap