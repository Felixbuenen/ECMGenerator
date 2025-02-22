#include "Gizmo.h"

#include "Area.h"
#include "ECMRenderer.h"
#include "UndoRedoManager.h"
#include "Command.h"
#include "UtilityFunctions.h"
#include "EnvironmentEditor.h"

#include "SDL.h"

#include <iostream>

namespace ECM {

	namespace WindowApplication {

		bool Gizmo::CheckClickHandle(const SDL_Rect& handle, const Point& screenPos) const
		{
			bool collision = true;

			collision = collision && handle.x <= screenPos.x;
			collision = collision && (handle.x + handle.w) >= screenPos.x;
			collision = collision && handle.y <= screenPos.y;
			collision = collision && (handle.y + handle.h) >= screenPos.y;

			return collision;
		}

		bool Gizmo::CheckIsStartingDragging(const SDL_Event& e)
		{
			Point screenPos = Point(e.button.x, e.button.y);
			bool click = false;

			if (CheckClickHandle(*m_GenHandle, screenPos))
			{
				m_ActiveAxis = GizmoAxis::BOTH;
				click = true;
			}
			else if (CheckClickHandle(*m_VertHandle, screenPos))
			{
				m_ActiveAxis = GizmoAxis::Y;
				click = true;
			}
			else if (CheckClickHandle(*m_HorHandle, screenPos))
			{
				m_ActiveAxis = GizmoAxis::X;
				click = true;
			}

			if (click)
			{
				m_IsStartingDrag = true;
				m_StartDragPosition = screenPos;
			}

			return click;
		}


		bool Gizmo::CheckIsDragging(const SDL_Event& e)
		{
			if (!m_IsStartingDrag && m_IsDragging) return true;

			if (m_IsStartingDrag)
			{
				Point screenPos(e.button.x, e.button.y);
				float dragDist = Utility::MathUtility::Distance(m_StartDragPosition, screenPos);

				if (dragDist >= m_MinDragLength)
				{
					m_DragDir = screenPos - m_StartDragPosition;
					m_DragDir.Normalize();

					m_IsStartingDrag = false;
					return true;
				}
			}
			
			return false;
		}

		void Gizmo::HandleStopDragging(const TransformMode& mode, const Vec2& delta)
		{
			if (m_IsDragging)
			{
				m_IsDragging = false;

				if (mode == TRANSLATE)
				{
					// no need to execute command, position already set...
					m_UndoRedo->Invoke(new CMD_TransformSimulationArea(m_ActiveArea, delta, Vec2(0, 0)), true); 
				}
				else if (mode == SCALE)
				{
					m_UndoRedo->Invoke(new CMD_TransformSimulationArea(m_ActiveArea, Vec2(0, 0), delta), true);
				}
			}

			m_IsStartingDrag = false;
		}


		TranslateGizmo::TranslateGizmo()
		{
			m_GenHandle = new SDL_Rect();
			m_HorHandle = new SDL_Rect();
			m_VertHandle = new SDL_Rect();
		}

		TranslateGizmo::~TranslateGizmo()
		{
			delete m_VertHandle;
			delete m_HorHandle;
			delete m_GenHandle;

			SDL_DestroyTexture(m_VertHandleTexture);
			SDL_DestroyTexture(m_HorHandleTexture);
		}

		void TranslateGizmo::Initialize(UndoRedoManager* undoRedo, ECMRenderer* ecmRenderer) 
		{
			Gizmo::Initialize(undoRedo, ecmRenderer);
			
			ecmRenderer->GetTextureFromBMP("../assets/trans_gizmo_yaxis.bmp", &m_VertHandleTexture);
			ecmRenderer->GetTextureFromBMP("../assets/trans_gizmo_xaxis.bmp", &m_HorHandleTexture);
		}


		bool TranslateGizmo::HandleInput(SDL_Event& e, ECMRenderer* ecmRenderer)
		{
			bool eventHandled = false;

			if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
			{
				Vec2 delta = m_ActiveArea->Position - m_OldWorldPosition;
				HandleStopDragging(TRANSLATE, delta);

				eventHandled = true;
			}

			// check click gizmo
			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
			{
				if (CheckIsStartingDragging(e))
				{
					m_StartDragWorldPosition = ecmRenderer->ScreenToWorldCoordinates(m_StartDragPosition.x, m_StartDragPosition.y);
					m_OldWorldPosition = m_ActiveArea->Position;

					eventHandled = true;
				}
				else
				{
					m_IsDragging = false;
				}
			}

			m_IsDragging = CheckIsDragging(e);

			if (m_IsDragging)
			{
				Vec2 offset = m_StartDragWorldPosition - m_OldWorldPosition;
				Point newPos = ecmRenderer->ScreenToWorldCoordinates(e.button.x, e.button.y) - offset;
				if (m_ActiveAxis == GizmoAxis::BOTH) m_ActiveArea->Position = newPos;
				else if (m_ActiveAxis == GizmoAxis::X) m_ActiveArea->Position.x = newPos.x;
				else if (m_ActiveAxis == GizmoAxis::Y) m_ActiveArea->Position.y = newPos.y;

				eventHandled = true;
			}

			return eventHandled;
		}

		void TranslateGizmo::Render(SDL_Renderer* sdlRenderer, ECMRenderer* ecmRenderer)
		{
			const float zoomFactor = ecmRenderer->GetCamZoomFactor();
		
			// render vertical handle
			Point handlePos = ecmRenderer->WorldToScreenCoordinates(m_ActiveArea->Position.x, m_ActiveArea->Position.y);
			
			m_VertHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_VertHandle->y = handlePos.y - (m_ArrowLength + m_ArrowSize * 0.5f) * zoomFactor;
			m_VertHandle->w = m_ArrowSize * zoomFactor;
			m_VertHandle->h = m_ArrowLength * zoomFactor;

			SDL_SetRenderDrawColor(sdlRenderer, 230, 0, 0, 255);
			SDL_RenderCopy(sdlRenderer, m_VertHandleTexture, NULL, m_VertHandle);
			//SDL_RenderFillRect(sdlRenderer, m_VertHandle);
		
			// render horizontal handle
			m_HorHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_HorHandle->y = handlePos.y - m_ArrowSize * 0.5f * zoomFactor;
			m_HorHandle->w = m_ArrowLength * zoomFactor;
			m_HorHandle->h = m_ArrowSize * zoomFactor;
			SDL_SetRenderDrawColor(sdlRenderer, 0, 150, 0, 255);
			SDL_RenderCopy(sdlRenderer, m_HorHandleTexture, NULL, m_HorHandle);
			//SDL_RenderFillRect(sdlRenderer, m_HorHandle);
		
			// render free handle
			m_GenHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_GenHandle->y = handlePos.y - m_ArrowSize * 0.5f * zoomFactor;
			m_GenHandle->w = m_ArrowSize * zoomFactor;
			m_GenHandle->h = m_ArrowSize * zoomFactor;
			SDL_SetRenderDrawColor(sdlRenderer, 230, 230, 0, 255);
			SDL_RenderFillRect(sdlRenderer, m_GenHandle);
		}

		ScaleGizmo::ScaleGizmo()
		{
			m_HorHandle = new SDL_Rect();
			m_VertHandle = new SDL_Rect();
			m_GenHandle = new SDL_Rect();
		}

		ScaleGizmo::~ScaleGizmo()
		{
			delete m_HorHandle;
			delete m_VertHandle;
			delete m_GenHandle;

			SDL_DestroyTexture(m_VertHandleTexture);
			SDL_DestroyTexture(m_HorHandleTexture);
		}

		void ScaleGizmo::Initialize(UndoRedoManager* undoRedo, ECMRenderer* ecmRenderer)
		{
			Gizmo::Initialize(undoRedo, ecmRenderer);

			ecmRenderer->GetTextureFromBMP("../assets/scale_gizmo_yaxis.bmp", &m_VertHandleTexture);
			ecmRenderer->GetTextureFromBMP("../assets/scale_gizmo_xaxis.bmp", &m_HorHandleTexture);
		}

		bool ScaleGizmo::HandleInput(SDL_Event& e, ECMRenderer* ecmRenderer)
		{
			bool eventHandled = false;

			if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
			{
				Vec2 delta = Vec2(m_ActiveArea->HalfWidth, m_ActiveArea->HalfHeight) - m_OldSize;
				HandleStopDragging(SCALE, delta);
				
				eventHandled = true;
			}

			// check click gizmo
			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
			{
				if (CheckIsStartingDragging(e))
				{
					m_StartDragWorldPosition = ecmRenderer->ScreenToWorldCoordinates(m_StartDragPosition.x, m_StartDragPosition.y);
					m_OldSize = Vec2(m_ActiveArea->HalfWidth, m_ActiveArea->HalfHeight);

					eventHandled = true;
				}
				else
				{
					m_IsDragging = false;
				}
			}
			
			m_IsDragging = CheckIsDragging(e);

			if (m_IsDragging)
			{
				// calculate delta
				Point screenPos = Point(e.button.x, e.button.y);
				Vec2 delta = screenPos - m_StartDragPosition;

				if (m_ActiveAxis == GizmoAxis::BOTH)
				{
					// calculate scale factor in direction of initial drag movement
					float scaleFactor = Utility::MathUtility::Dot(m_DragDir, delta);
					scaleFactor = scaleFactor * m_ScaleMultiplier;
					Vec2 scaleDelta = Vec2(scaleFactor, scaleFactor);
					Vec2 newSize = m_OldSize + scaleDelta;

					m_ActiveArea->HalfWidth = newSize.x;
					m_ActiveArea->HalfHeight = newSize.y;
				}
				else if (m_ActiveAxis == GizmoAxis::X)
				{
					float scaleFactor = Utility::MathUtility::Dot(Vec2(1.0f, 0.0f), delta);
					scaleFactor = scaleFactor * m_ScaleMultiplier;
					float newSizeX = m_OldSize.x + scaleFactor;

					m_ActiveArea->HalfWidth = newSizeX;
				}
				else if (m_ActiveAxis == GizmoAxis::Y)
				{
					float scaleFactor = Utility::MathUtility::Dot(Vec2(0.0f, -1.0f), delta);
					scaleFactor = scaleFactor * m_ScaleMultiplier;
					float newSizeY = m_OldSize.y + scaleFactor;

					m_ActiveArea->HalfHeight = newSizeY;
				}
				
				eventHandled = true;
			}

			return eventHandled;
		}

		void ScaleGizmo::Render(SDL_Renderer* sdlRenderer, ECMRenderer* ecmRenderer)
		{
			if (m_IsDragging) return;

			const float zoomFactor = ecmRenderer->GetCamZoomFactor();

			// render vertical handle
			Point handlePos = ecmRenderer->WorldToScreenCoordinates(m_ActiveArea->Position.x, m_ActiveArea->Position.y);

			m_VertHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_VertHandle->y = handlePos.y - (m_ArrowLength + m_ArrowSize * 0.5f) * zoomFactor;
			m_VertHandle->w = m_ArrowSize * zoomFactor;
			m_VertHandle->h = m_ArrowLength * zoomFactor;

			SDL_SetRenderDrawColor(sdlRenderer, 230, 0, 0, 255);
			SDL_RenderCopy(sdlRenderer, m_VertHandleTexture, NULL, m_VertHandle);
			//SDL_RenderFillRect(sdlRenderer, m_VertHandle);

			// render horizontal handle
			m_HorHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_HorHandle->y = handlePos.y - m_ArrowSize * 0.5f * zoomFactor;
			m_HorHandle->w = m_ArrowLength * zoomFactor;
			m_HorHandle->h = m_ArrowSize * zoomFactor;
			SDL_SetRenderDrawColor(sdlRenderer, 0, 150, 0, 255);
			SDL_RenderCopy(sdlRenderer, m_HorHandleTexture, NULL, m_HorHandle);
			//SDL_RenderFillRect(sdlRenderer, m_HorHandle);

			// render free handle
			m_GenHandle->x = handlePos.x - m_ArrowSize * 0.5f * zoomFactor;
			m_GenHandle->y = handlePos.y - m_ArrowSize * 0.5f * zoomFactor;
			m_GenHandle->w = m_ArrowSize * zoomFactor;
			m_GenHandle->h = m_ArrowSize * zoomFactor;
			SDL_SetRenderDrawColor(sdlRenderer, 230, 230, 0, 255);
			SDL_RenderFillRect(sdlRenderer, m_GenHandle);
		}
	}

}