#pragma once

#include "ECMDataTypes.h"
#include "UI.h"
#include "Area.h"

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		class EnvironmentEditor;

		class ICommand
		{
		public:
			virtual void Execute() = 0;
			virtual void Undo() = 0;
		};

		class CMD_SelectArea : public ICommand
		{
		public:
			CMD_SelectArea(EnvironmentEditor* editor, Simulation::Area* prevArea, Simulation::Area* area)
				: m_EnvEditor(editor), m_Area(area), m_PrevArea(prevArea) { }

			virtual void Execute() override;
			virtual void Undo() override;

		private:
			EnvironmentEditor* m_EnvEditor;
			Simulation::Area* m_Area;
			Simulation::Area* m_PrevArea;
		};

		class CMD_SelectAreaConnection : public ICommand
		{
		public:
			CMD_SelectAreaConnection(EnvironmentEditor* editor, Simulation::AreaConnection prevAreaConnection, Simulation::AreaConnection areaConnection)
				: m_EnvEditor(editor), m_AreaConnection(areaConnection), m_PrevAreaConnection(prevAreaConnection) { }

			virtual void Execute() override;
			virtual void Undo() override;

		private:
			EnvironmentEditor* m_EnvEditor;
			Simulation::AreaConnection m_AreaConnection;
			Simulation::AreaConnection m_PrevAreaConnection;
		};

		class CMD_AddSimulationArea : public ICommand
		{
		public:
			CMD_AddSimulationArea(Simulation::SimAreaType areaType, Point screenPos, Vec2 halfSize, Application* application) 
				: m_AreaType(areaType), m_Application(application), m_ScreenPos(screenPos), m_HalfSize(halfSize) { }

			void Execute() override;
			void Undo() override;

		private:
			Application* m_Application;
			Point m_ScreenPos;
			Vec2 m_HalfSize;
			Simulation::SimAreaType m_AreaType;
			int m_ID;
		};

		class CMD_AddSimulationAreaConnection : public ICommand
		{
		public:
			CMD_AddSimulationAreaConnection(Application* application, int spawnID, int goalID, float spawnRate)
				: m_Application(application), m_SpawnID(spawnID), m_GoalID(goalID), m_SpawnRate(spawnRate) { }

			void Execute() override;
			void Undo() override;

		private:
			Application* m_Application;
			int m_SpawnID;
			int m_GoalID;
			float m_SpawnRate;
		};

		class CMD_RemoveSimulationArea : public ICommand
		{
		public:
			CMD_RemoveSimulationArea(const Simulation::Area& area, Application* application);

			void Execute() override;
			void Undo() override;

		private:
			//void RemoveArea(const Simulation::Area& area);

			Application* m_Application;
		};

		class CMD_TransformSimulationArea : public ICommand
		{
		public:
			CMD_TransformSimulationArea(Simulation::Area* area, Vec2 translationDelta, Vec2 scaleDelta)
				: m_Area(area), m_TranslationDelta(translationDelta), m_ScaleDelta(scaleDelta) { }

			void Execute() override;
			void Undo() override;

		private:
			//void TransformArea(Simulation::Area& area, Vec2 translation, Vec2 scale);

			Simulation::Area* m_Area;
			Vec2 m_TranslationDelta;
			Vec2 m_ScaleDelta;
		};
	}
}