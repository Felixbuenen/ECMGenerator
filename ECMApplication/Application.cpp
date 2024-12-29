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
			if (!InitializeRenderer()) return false;

			// Setup Dear ImGui context
			IMGUI_CHECKVERSION();
			ImGui::CreateContext();
			ImGuiIO& io = ImGui::GetIO(); (void)io;
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
			
			// Setup Dear ImGui style
			ImGui::StyleColorsDark();
			
			// Setup Platform/Renderer backends
			ImGui_ImplSDL2_InitForSDLRenderer(m_Window, m_Renderer.GetSDLRenderer());
			ImGui_ImplSDLRenderer2_Init(m_Renderer.GetSDLRenderer());

			return true;
		}

		void Application::Run()
		{
			Uint64 currentTime = SDL_GetPerformanceCounter();
			Uint64 lastTime = 0;
			m_DeltaTime = 0.0;
			m_DeltaTimeSmooth = 0.0;
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

		bool Application::HandleInput(SDL_Event& e)
		{
			// first input layer: UI
			ImGui_ImplSDL2_ProcessEvent(&e);

			// second input layer: Application
			if (ImGui::GetIO().WantCaptureMouse) return false;
			HandleMouseEvent(e);

			if (ImGui::GetIO().WantCaptureKeyboard) return false;
			HandleKeyEvent(e);

			return true;
		}

		void Application::Update(SDL_Event& event)
		{
			m_DeltaTimeSmooth += (m_DeltaTime - m_DeltaTimeSmooth) * 0.01f;

			// UPDATE SIMULATION
			{
				//Timer timer("SIMULATION");
				if (m_ApplicationState.simulationPlaying)
				{
					m_ApplicationState.simulator->Update(m_DeltaTime);
				}
			}

			// UI update call
			// TODO: make general UI class that takes care of rendering and tracking UI logic
			m_AreaPanel.Update(event, *this);
		}

		void Application::Render()
		{
			SDL_RenderClear(m_Renderer.GetSDLRenderer());

			m_Renderer.Render();
			CreateUI();
			ImGui::Render();

			ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

			SDL_RenderPresent(m_Renderer.GetSDLRenderer());
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
				Point worldCoords = m_Renderer.ScreenToWorldCoordinates(e.button.x, e.button.y);

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

		void Application::CreateUI()
		{
			// Start the Dear ImGui frame
			ImGui_ImplSDLRenderer2_NewFrame();
			ImGui_ImplSDL2_NewFrame();
			ImGui::NewFrame();

			float screenHeight = ImGui::GetIO().DisplaySize.y;
			float screenWidth = ImGui::GetIO().DisplaySize.x;

			// create playback panel
			float playbackPanelHeight = 30.0f;
			float playbackPanelWidth = 150.0f;
			ImGuiWindowFlags playbackPanelFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
				| ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings
				| ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
				| ImGuiWindowFlags_NoScrollWithMouse;

			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.f, 0.f));

			ImGui::SetNextWindowPos(ImVec2(screenWidth/2.0f, screenHeight), 0, ImVec2(0.5f, 1.0f));
			ImGui::SetNextWindowSize(ImVec2(playbackPanelWidth, playbackPanelHeight));
			ImGui::Begin("Playback", nullptr, playbackPanelFlags);
			if (ImGui::Button("Play", ImVec2(playbackPanelWidth / 3.0f, playbackPanelHeight)))
			{
				m_ApplicationState.simulationPlaying = true;
				m_ApplicationState.simulationPaused = false;
			}
			ImGui::SameLine();
			if (ImGui::Button("Pause", ImVec2(playbackPanelWidth / 3.0f, playbackPanelHeight)))
			{
				m_ApplicationState.simulationPaused = true;
				m_ApplicationState.simulationPlaying = false;
			}
			ImGui::SameLine();
			if (ImGui::Button("Stop", ImVec2(playbackPanelWidth / 3.0f, playbackPanelHeight)))
			{
				m_ApplicationState.simulationPaused = true;
				m_ApplicationState.simulationPlaying = false;
				m_ApplicationState.simulator->Reset();
			}

			ImGui::End();
			ImGui::PopStyleVar(4);

			// DEBUGGING: create FPS panel
			float fpsPanelHeight = 30.0f;
			float fpsPanelWidth = 150.0f;
			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(5.f, 5.f));
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.f, 5.f));
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.f, 0.f));
			ImGui::SetNextWindowPos(ImVec2(screenWidth / 2.0f, 0), 0, ImVec2(0.5f, 0.0f));
			ImGui::SetNextWindowSize(ImVec2(fpsPanelWidth, fpsPanelHeight));

			ImGui::Begin("FPS", nullptr, playbackPanelFlags);

			float windowWidth = ImGui::GetWindowSize().x;
			float windowHeight = ImGui::GetWindowSize().y;
			
			float fps = m_DeltaTimeSmooth == 0.0f ? 0.0f : 1.0f / m_DeltaTimeSmooth;
			std::string fpsString = "FPS: ";
			fpsString.append(std::to_string(fps));
			int numDecimals = fpsString.size() - fpsString.find('.');
			if (numDecimals > 3)
			{
				fpsString.resize(fpsString.size() - numDecimals + 3);
			}

			float textWidth = ImGui::CalcTextSize(fpsString.c_str()).x;
			float textHeigh = ImGui::CalcTextSize(fpsString.c_str()).y;
			ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
			ImGui::SetCursorPosY((windowHeight - textHeigh) * 0.5f);
			//std::string s = std::format("{:.2f}", 3.14159265359); // s == "3.14"

			ImGui::Text(fpsString.c_str());
			ImGui::End();
			ImGui::PopStyleVar(4);

			// create simulation area panel
			m_AreaPanel.Render();
		}


	} // Visualisation
} // ExplicitCorridorMap