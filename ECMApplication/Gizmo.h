#pragma once

#include "ECMDataTypes.h"

class SDL_Renderer;
class SDL_Texture;
struct SDL_Rect;
typedef union SDL_Event;

namespace ECM {

	namespace Simulation {
		struct Area;
	}

	namespace WindowApplication {

		class ECMRenderer;
		class UndoRedoManager;

		enum TransformMode;

		enum GizmoAxis {
			X,
			Y,
			BOTH
		};

		class Gizmo
		{
		public:
			virtual void Initialize(UndoRedoManager* undoRedo, ECMRenderer* ecmRenderer) { m_UndoRedo = undoRedo; }

			// returns true if a gizmo input event was handled
			virtual bool HandleInput(SDL_Event& e, ECMRenderer* ecmRenderer) = 0;
			virtual void Render(SDL_Renderer* sdlRenderer, ECMRenderer* ecmRenderer) = 0;

			void SetActiveArea(Simulation::Area* area) { m_ActiveArea = area; }

			Simulation::Area* GetArea() const { return m_ActiveArea; }

		protected:
			bool CheckClickHandle(const SDL_Rect& handle, const Point& screenPos) const;
			bool CheckIsStartingDragging(const SDL_Event& e);
			bool CheckIsDragging(const SDL_Event& e);
			void HandleStopDragging(const TransformMode& mode, const Vec2& delta);

			SDL_Rect* m_HorHandle;
			SDL_Rect* m_VertHandle;
			SDL_Rect* m_GenHandle;

			Simulation::Area* m_ActiveArea;
			UndoRedoManager* m_UndoRedo;
			
			Point m_StartDragPosition;
			Point m_StartDragWorldPosition;
			Vec2 m_DragDir;

			bool m_IsStartingDrag = false;
			bool m_IsDragging = false;
			const float m_MinDragLength = 0.05f;
			GizmoAxis m_ActiveAxis;
		};

		class TranslateGizmo : public Gizmo
		{
		public:
			TranslateGizmo();
			~TranslateGizmo();

			void Initialize(UndoRedoManager* undoRedo, ECMRenderer* ecmRenderer) override;

			bool HandleInput(SDL_Event& e, ECMRenderer* ecmRenderer) override;
			void Render(SDL_Renderer* sdlRenderer, ECMRenderer* ecmRenderer) override;

		private:
			Point m_OldWorldPosition;
			const float m_ArrowLength = 80.0f;
			const float m_ArrowSize = 15.0f;

			SDL_Texture* m_VertHandleTexture;
			SDL_Texture* m_HorHandleTexture;
		};

		class ScaleGizmo : public Gizmo
		{
		public:
			ScaleGizmo();
			~ScaleGizmo();

			void Initialize(UndoRedoManager* undoRedo, ECMRenderer* ecmRenderer) override;

			bool HandleInput(SDL_Event& e, ECMRenderer* ecmRenderer) override;
			void Render(SDL_Renderer* sdlRenderer, ECMRenderer* ecmRenderer) override;

		private:

			const float m_ArrowLength = 80.0f;
			const float m_ArrowSize = 15.0f;
			const float m_ScaleMultiplier = 1.0f;
			Vec2 m_OldSize;

			SDL_Texture* m_VertHandleTexture;
			SDL_Texture* m_HorHandleTexture;
		};

	}
}