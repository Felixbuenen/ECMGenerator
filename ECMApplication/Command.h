#pragma once

#include "ECMDataTypes.h"
#include "UI.h"
#include "Area.h"

typedef union SDL_Event;

namespace ECM {

	namespace Simulation {
		struct Area;
	}

	namespace WindowApplication {

		class ICommand
		{
		public:
			virtual void Execute() = 0;
			virtual void Undo() = 0;
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
			CMD_TransformSimulationArea(Simulation::Area& area, Vec2 translation, Vec2 scale, Application* application);

			void Execute() override;
			void Undo() override;

		private:
			//void TransformArea(Simulation::Area& area, Vec2 translation, Vec2 scale);

			Application* m_Application;
			Vec2 m_Translation;
			Vec2 m_Scale;
		};
	}
}