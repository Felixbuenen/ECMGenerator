#include "UI.h"

#include "Application.h"
#include "SimAreaPanel.h"
#include "PlaybackPanel.h"
#include "MainMenu.h"
#include "EditorModePanel.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

namespace ECM {
	namespace WindowApplication {

		UI::~UI()
		{
			for (UIWidget* w : m_Widgets)
			{
				delete w;
			}
		}

		void UI::Initialize(Application* application)
		{
			// Setup Dear ImGui context
			IMGUI_CHECKVERSION();
			ImGui::CreateContext();
			ImGuiIO& io = ImGui::GetIO(); (void)io;
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

			// Setup Dear ImGui style
			ImGui::StyleColorsDark();

			// Setup Platform/Renderer backends
			ImGui_ImplSDL2_InitForSDLRenderer(application->GetWindow(), application->GetApplicationRenderer());
			ImGui_ImplSDLRenderer2_Init(application->GetApplicationRenderer());

			InitializeWidgets(application);

			m_Application = application;
		}

		void UI::CleanUp()
		{
			ImGui_ImplSDLRenderer2_Shutdown();
			ImGui_ImplSDL2_Shutdown();
			ImGui::DestroyContext();
		}

		void UI::InitializeWidgets(Application* application)
		{
			m_Widgets.push_back(new PlaybackPanel(application));
			m_Widgets.push_back(new EditorModePanel(application));
			m_Widgets.push_back(new MainMenu(application));
		}

		void UI::HandleInput(SDL_Event& e)
		{
			// set mouse position in application state
			ImGuiIO& io = ImGui::GetIO();
			ImVec2 mousePos = io.MousePos;
			m_Application->GetApplicationState()->mousePosition = Point(io.MousePos.x, io.MousePos.y);

			ImGui_ImplSDL2_ProcessEvent(&e);
		}

		void UI::Update(SDL_Event& e)
		{
			for (UIWidget* w : m_Widgets)
			{
				w->Update(e);
			}
		}

		void UI::Render()
		{
			ImGui_ImplSDLRenderer2_NewFrame();
			ImGui_ImplSDL2_NewFrame();
			ImGui::NewFrame();

			for (UIWidget* w : m_Widgets)
			{
				w->Render();
			}

			ImGui::EndFrame();

			ImGui::Render();
			ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
		}

	}
}