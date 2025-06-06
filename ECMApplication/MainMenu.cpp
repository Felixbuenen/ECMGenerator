#include "MainMenu.h"

#include "Application.h"
#include "Area.h"

#include "SDL.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

namespace ECM {

	namespace WindowApplication {

		MainMenu::MainMenu(Application* application)
			: UIWidget(application)
		{
			m_ApplicationState = application->GetApplicationState();
		}

        static void ShowExampleMenuFile()
        {
            ImGui::MenuItem("(demo menu)", NULL, false, false);
            if (ImGui::MenuItem("New")) {}
            if (ImGui::MenuItem("Open", "Ctrl+O")) {}
            if (ImGui::BeginMenu("Open Recent"))
            {
                ImGui::MenuItem("fish_hat.c");
                ImGui::MenuItem("fish_hat.inl");
                ImGui::MenuItem("fish_hat.h");
                if (ImGui::BeginMenu("More.."))
                {
                    ImGui::MenuItem("Hello");
                    ImGui::MenuItem("Sailor");
                    if (ImGui::BeginMenu("Recurse.."))
                    {
                        ShowExampleMenuFile();
                        ImGui::EndMenu();
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();
            }
            if (ImGui::MenuItem("Save", "Ctrl+S")) {}
            if (ImGui::MenuItem("Save As..")) {}

            ImGui::Separator();

            if (ImGui::BeginMenu("Options"))
            {
                static bool enabled = true;
                ImGui::MenuItem("Enabled", "", &enabled);
                ImGui::BeginChild("child", ImVec2(0, 60), true);
                for (int i = 0; i < 10; i++)
                    ImGui::Text("Scrolling Text %d", i);
                ImGui::EndChild();
                static float f = 0.5f;
                static int n = 0;
                ImGui::SliderFloat("Value", &f, 0.0f, 1.0f);
                ImGui::InputFloat("Input", &f, 0.1f);
                ImGui::Combo("Combo", &n, "Yes\0No\0Maybe\0\0");
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Colors"))
            {
                float sz = ImGui::GetTextLineHeight();
                for (int i = 0; i < ImGuiCol_COUNT; i++)
                {
                    const char* name = ImGui::GetStyleColorName((ImGuiCol)i);
                    ImVec2 p = ImGui::GetCursorScreenPos();
                    ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32((ImGuiCol)i));
                    ImGui::Dummy(ImVec2(sz, sz));
                    ImGui::SameLine();
                    ImGui::MenuItem(name);
                }
                ImGui::EndMenu();
            }

            // Here we demonstrate appending again to the "Options" menu (which we already created above)
            // Of course in this demo it is a little bit silly that this function calls BeginMenu("Options") twice.
            // In a real code-base using it would make senses to use this feature from very different code locations.
            if (ImGui::BeginMenu("Options")) // <-- Append!
            {
                static bool b = true;
                ImGui::Checkbox("SomeOption", &b);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Disabled", false)) // Disabled
            {
                IM_ASSERT(0);
            }
            if (ImGui::MenuItem("Checked", NULL, true)) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Alt+F4")) {}
        }


		void MainMenu::Render()
		{
            if (ImGui::BeginMainMenuBar())
            {
                if (ImGui::BeginMenu("Project"))
                {
                    ShowExampleMenuFile();
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Edit"))
                {
                    if (ImGui::MenuItem("Undo", "CTRL+Z")) {}
                    if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
                    ImGui::Separator();
                    if (ImGui::MenuItem("Cut", "CTRL+X")) {}
                    if (ImGui::MenuItem("Copy", "CTRL+C")) {}
                    if (ImGui::MenuItem("Paste", "CTRL+V")) {}
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Simulation"))
                {
                    if (ImGui::MenuItem("Build simulation")) {}
                    ImGui::Separator();
                    if (ImGui::MenuItem("Add Walkable Area")) 
                    {}

                    if (ImGui::MenuItem("Add Spawn Area")) 
                    {
                        m_Application->GetEnvironmentEditor()->StartDragArea(Simulation::SimAreaType::SPAWN);
                        m_Application->GetApplicationState()->mode = ApplicationMode::BUILD;
                    }
                    if (ImGui::MenuItem("Add Goal Area"))
                    {
                        m_Application->GetEnvironmentEditor()->StartDragArea(Simulation::SimAreaType::GOAL);
                        m_Application->GetApplicationState()->mode = ApplicationMode::BUILD;
                    }
                    if (ImGui::MenuItem("Add Obstacle Area"))
                    {
                        m_Application->GetEnvironmentEditor()->StartDragArea(Simulation::SimAreaType::OBSTACLE);
                        m_Application->GetApplicationState()->mode = ApplicationMode::BUILD;
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("Connect areas"))
                    {
                        m_Application->GetApplicationState()->mode = ApplicationMode::CONNECT;
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("Settings")) {}
                    ImGui::EndMenu();
                }
                ImGui::EndMainMenuBar();
            }
		}
	}
}