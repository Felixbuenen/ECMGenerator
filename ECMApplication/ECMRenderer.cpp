#include "ECMRenderer.h"
#include "ECM.h"
#include "Environment.h"
#include "ECMCellCollection.h"
#include "UtilityFunctions.h"
#include "Application.h"
#include "ECMDataTypes.h"
#include "SimAreaPanel.h"
#include "Area.h"
#include "KDTree.h"

#include "SDL.h"
#include "boost/polygon/voronoi.hpp"
#include <boost/polygon/polygon.hpp>
#include "BoostVisualizeUtils.h"

#include <stdio.h>

namespace ECM {
	namespace WindowApplication {

		void ECMRenderer::Initialize(Application* app)
		{
			m_Renderer = SDL_CreateRenderer(app->GetWindow(), -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
			m_AppState = app->GetApplicationState();
		}

		void ECMRenderer::Clear()
		{
			SDL_DestroyRenderer(m_Renderer);
		}

		void ECMRenderer::Render()
		{
			UpdateRenderState();

			DrawBackground();
			DrawWalkableArea();
			DrawObstacles();
			DrawMedialAxis();
			//DrawECMVertices();
			//DrawClosestObstaclePoints();
			//DrawCorridor();
			//DrawPortals();
			//HighlightECMVertex(11);
			//HighlightECMVertex(21);
			//HighlightECMVertex(18);
			
			//DrawHalfEdge(22);

			// TEST
			//DrawRandomTestPath();
			//DrawInsideVerts();
			DebugDrawECMCell();
			//DebugDrawSecondaryLines();
			//DebugDrawCellValues();
			//DebugDrawBoostVoronoiDiagram();

			DrawSimulationAreas();
			DrawPath();

			DrawAgents();
			//DrawPaths();

			InternalDrawDragSimulationArea();
		}

		Point ECMRenderer::ScreenToWorldCoordinates(float x, float y)
		{
			float worldX = (x - m_AppState->camOffsetX) / m_AppState->camZoomFactor;
			float worldY = (y - m_AppState->camOffsetY) / m_AppState->camZoomFactor * -1; // -1.0 because inversed y-axis

			return Point(worldX, worldY);
		}

		Point ECMRenderer::WorldToScreenCoordinates(float x, float y)
		{
			float screenX = x * m_CamZoomFactor + m_CamOffsetX;
			float screenY = y * m_CamZoomFactor * m_YRotation + m_CamOffsetY;

			return Point(screenX, screenY);
		}

		void ECMRenderer::RenderDragSimulationArea(float x, float y, SimAreaDrag areaType)
		{
			m_AppState->dragAreaPosition.x = x;
			m_AppState->dragAreaPosition.y = y;
			m_AppState->dragAreaType = areaType;
		}

		void ECMRenderer::StopRenderDragSimulationArea()
		{
			m_AppState->dragAreaType = SimAreaDrag::NONE;
		}

		void ECMRenderer::InternalDrawDragSimulationArea()
		{
			if (m_AppState->dragAreaType == SimAreaDrag::NONE) return;

			// TODO: place in variables
			float halfWidth = 50;
			float halfHeight = 50;
			float spawnX = m_AppState->dragAreaPosition.x;
			float spawnY = m_AppState->dragAreaPosition.y;
			SDL_Rect spawnRect;

			Point pos = ScreenToWorldCoordinates(spawnX, spawnY);
			pos.x = pos.x - halfWidth;
			pos.y = pos.y + halfHeight;
			
			Point screenPos = WorldToScreenCoordinates(pos.x, pos.y);
			
			spawnRect.x = screenPos.x;
			spawnRect.y = screenPos.y;
			spawnRect.w = halfWidth * 2 * m_CamZoomFactor;
			spawnRect.h = halfHeight * 2 * m_CamZoomFactor;
			SDL_SetRenderDrawBlendMode(m_Renderer, SDL_BLENDMODE_BLEND);
			if (m_AppState->dragAreaType == SimAreaDrag::SPAWN) SDL_SetRenderDrawColor(m_Renderer, 120.0f, 200.0f, 120.0f, 150);
			if(m_AppState->dragAreaType == SimAreaDrag::GOAL) SDL_SetRenderDrawColor(m_Renderer, 200.0, 120.0f, 120.0f, 150);
			SDL_RenderFillRect(m_Renderer, &spawnRect);
		}

		void ECMRenderer::DebugDrawKNearestNeighbors(int idx)
		{
			if (!m_AppState->simulator->GetActiveFlags()[idx]) return;

			const int k = 4;
			
			std::vector<int> agents;
			agents.resize(k);
			//m_AppState->simulator->GetKDTree()->KNearestAgents(idx, k, agents);

			agents.push_back(idx);

			const auto positions = m_AppState->simulator->GetPositionData();
			const auto clearances = m_AppState->simulator->GetClearanceData();

			const float recip = sqrtf(2.0f);

			for (int i = 0; i < k + 1; i++)
			{
				int index = agents[i];
				const auto& pos = positions[index];
				const auto& clearance = clearances[index];

				float x = pos.x * m_CamZoomFactor + m_CamOffsetX;
				float y = pos.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				float size = (clearance.clearance / recip) * m_CamZoomFactor;

				SDL_Rect rect;
				rect.x = x - size;
				rect.y = y - size;
				rect.w = size * 2;
				rect.h = size * 2;

				if(i < k) SDL_SetRenderDrawColor(m_Renderer, 255, 125, 125, 255);
				else SDL_SetRenderDrawColor(m_Renderer, 255, 0, 0, 255);
				
				SDL_RenderFillRect(m_Renderer, &rect);
			}
		}



		// Cache render-state variables, easier to reference the app state everywhere
		void ECMRenderer::UpdateRenderState()
		{
			m_CamZoomFactor = m_AppState->camZoomFactor;
			m_CamOffsetX = m_AppState->camOffsetX, 
			m_CamOffsetY = m_AppState->camOffsetY;
			m_YRotation = -1.0f;

			m_Env = m_AppState->environment;
			m_Ecm = m_AppState->ecm;
		}


		void ECMRenderer::DrawBackground()
		{
			SDL_SetRenderDrawColor(m_Renderer, m_ColorSettings.background_R, m_ColorSettings.background_G, m_ColorSettings.background_B, 0xff);
			SDL_RenderClear(m_Renderer);
		}

		void ECMRenderer::DrawWalkableArea()
		{
			SDL_SetRenderDrawColor(m_Renderer, m_ColorSettings.walkableArea_R, m_ColorSettings.walkableArea_G, m_ColorSettings.walkableArea_B, 0xff);
			const std::vector<Segment>& walkableArea = m_Env->GetWalkableArea();

			// this code only works for this specific test case! make sure to encapsulate it in a walkable area struct 
			int x = walkableArea[0].p0.x * m_CamZoomFactor + m_CamOffsetX;
			int y = walkableArea[0].p0.y * m_CamZoomFactor + m_CamOffsetY;
			int w = (walkableArea[0].p1.x - walkableArea[0].p0.x);
			int h = (walkableArea[1].p1.y - walkableArea[1].p0.y);

			w *= m_CamZoomFactor;
			h *= m_CamZoomFactor;

			SDL_Rect testWalkableArea;
			testWalkableArea.x = x;
			testWalkableArea.y = y;
			testWalkableArea.w = w;
			testWalkableArea.h = h;

			SDL_RenderDrawRect(m_Renderer, &testWalkableArea);
			SDL_RenderFillRect(m_Renderer, &testWalkableArea);
		}

		void ECMRenderer::DrawObstacles()
		{
			SDL_SetRenderDrawColor(m_Renderer, m_ColorSettings.obstacle_R, m_ColorSettings.obstacle_G, m_ColorSettings.obstacle_B, 0xff);

			const std::vector<std::vector<Segment>>& obstacles = m_Env->GetObstacles();
			for (const std::vector<Segment>& obstacle : obstacles) {
				for (const Segment& edge : obstacle) {

					int x1 = edge.p0.x * m_CamZoomFactor + m_CamOffsetX;
					int y1 = edge.p0.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					int x2 = edge.p1.x * m_CamZoomFactor + m_CamOffsetX;
					int y2 = edge.p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
				}
			}

			// _ecm->GetObstacles();
		}

		void ECMRenderer::DrawMedialAxis()
		{
			auto ma = m_Ecm->GetMedialAxis();
			ECMGraph& graph = m_Ecm->GetECMGraph();

			// TODO: add colour to properties
			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0x00, 0x00, 0xff);

			const std::vector<ECMEdge>& edges = graph.GetEdges();
			const std::vector<ECMVertex>& verts = graph.GetVertices();

			// TODO: 
			// > figure out how we can use the point_type / segment_type for all calculations
			// > refactor code to prevent code duplication
			using namespace boost::polygon;
			typedef double coordinate_type;
			typedef boost::polygon::point_data<coordinate_type> point_type;
			typedef boost::polygon::segment_data<coordinate_type> segment_type;

			for (const ECMEdge& edge : edges)
			{
				const ECMHalfEdge& edge0 = edge.half_edges[0];
				const ECMHalfEdge& edge1 = edge.half_edges[1];
				int v1idx = edge0.v_target_idx;
				int v2idx = edge1.v_target_idx;

				int ptLeftOfIdx;
				if (graph.IsArc(edge, ptLeftOfIdx))
				{
					std::vector<point_type> discr;

					point_type pt1(graph.GetVertex(v1idx)->position.x, graph.GetVertex(v1idx)->position.y);
					point_type pt2(graph.GetVertex(v2idx)->position.x, graph.GetVertex(v2idx)->position.y);
					discr.push_back(pt1);
					discr.push_back(pt2);

					const double maxDist = 0.5;

					if (ptLeftOfIdx == 0)
					{
						point_type p(edge0.closest_left.x, edge0.closest_left.y);
						point_type nr0(edge0.closest_right.x, edge0.closest_right.y);
						point_type nr1(edge1.closest_left.x, edge1.closest_left.y);
						segment_type seg(nr0, nr1);

						boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
					}
					else
					{
						point_type p(edge0.closest_right.x, edge0.closest_right.y);
						point_type nl0(edge0.closest_left.x, edge0.closest_left.y);
						point_type nl1(edge1.closest_right.x, edge1.closest_right.y);
						segment_type seg(nl0, nl1);

						boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
					}

					int numEdges = discr.size() - 1;
					for (int i = 0; i < numEdges; i++)
					{
						int x1 = discr[i].x() * m_CamZoomFactor + m_CamOffsetX;
						int y1 = discr[i].y()* m_YRotation * m_CamZoomFactor + m_CamOffsetY;
						int x2 = discr[i + 1].x() * m_CamZoomFactor + m_CamOffsetX;
						int y2 = discr[i + 1].y() * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

						SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
					}
				}
				else
				{
					int x1 = verts[v1idx].position.x * m_CamZoomFactor + m_CamOffsetX;
					int y1 = verts[v1idx].position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					int x2 = verts[v2idx].position.x * m_CamZoomFactor + m_CamOffsetX;
					int y2 = verts[v2idx].position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
				}

			}
		}

		void ECMRenderer::DrawRandomTestPath()
		{
			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0xff, 0x00, 0xff);

			std::vector<Segment> path = m_Ecm->GetRandomTestPath();

			for (const Segment& s : path)
			{
				float x0 = s.p0.x * m_CamZoomFactor + m_CamOffsetX;
				float x1 = s.p1.x * m_YRotation * m_CamZoomFactor + m_CamOffsetX;
				float y0 = s.p0.y * m_CamZoomFactor + m_CamOffsetY;
				float y1 = s.p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
			}
		}

		void ECMRenderer::DrawECMVertices()
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();
			for (const ECMVertex& vertex : verts)
			{
				Point p = vertex.position;

				const int w = 12;
				const int h = 12;

				int x1 = p.x * m_CamZoomFactor + m_CamOffsetX - w/2;
				int y1 = p.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY - h/2;

				SDL_Rect rect;
				rect.x = x1;
				rect.y = y1;
				rect.w = w;
				rect.h = h;

				SDL_SetRenderDrawColor(m_Renderer, 255, 0, 0, 255);
				SDL_RenderDrawRect(m_Renderer, &rect);
			}

		}

		void ECMRenderer::HighlightECMVertex(int index)
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();
			const ECMVertex& vertex = verts[index];

			Point p = vertex.position;

			const int w = 30;
			const int h = 30;

			int x1 = p.x * m_CamZoomFactor + m_CamOffsetX - w / 2;
			int y1 = p.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY - h / 2;

			SDL_Rect rect;
			rect.x = x1;
			rect.y = y1;
			rect.w = w;
			rect.h = h;

			SDL_SetRenderDrawColor(m_Renderer, 255, 0, 0, 255);
			SDL_RenderDrawRect(m_Renderer, &rect);
		}

		void ECMRenderer::DrawInsideVerts()
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();

			SDL_SetRenderDrawColor(m_Renderer, 0xff, 0x00, 0x00, 0xff);
			//SDL_RenderSetScale(_renderer, 2.0f, 2.0f);

			for (auto vert : verts)
			{
				if (m_Env->InsideObstacle(vert.position))
				{
					float x = vert.position.x * m_CamZoomFactor + m_CamOffsetX;
					float y = vert.position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					const int size = 5;

					SDL_RenderDrawLine(m_Renderer, (x - size), y, (x + size), y);
					SDL_RenderDrawLine(m_Renderer, (x), (y - size), x, (y + size));
				}
			}

			//SDL_RenderSetScale(_renderer, 1.0f, 1.0f);
		}

		void ECMRenderer::DrawClosestObstaclePoints()
		{
			ECMGraph& graph = m_Ecm->GetECMGraph();
			auto verts = graph.GetVertices();
			SDL_SetRenderDrawColor(m_Renderer, 0xff, 0x8c, 0x00, 0xff);

			for (const auto& vert : verts)
			{
				if (vert.idx == 4)
				{
					bool stop = true;
				}
				float startX = vert.position.x * m_CamZoomFactor + m_CamOffsetX;
				float startY = vert.position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				int firstHalfEdge = vert.half_edge_idx;
				int currentHalfEdge = firstHalfEdge;
				ECMHalfEdge* edge = graph.GetHalfEdge(firstHalfEdge);
				do
				{
					edge = graph.GetHalfEdge(currentHalfEdge);

					float xl = edge->closest_left.x * m_CamZoomFactor + m_CamOffsetX;
					float yl = edge->closest_left.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					SDL_RenderDrawLine(m_Renderer, startX, startY, xl, yl);

					float xr = edge->closest_right.x * m_CamZoomFactor + m_CamOffsetX;
					float yr = edge->closest_right.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					SDL_RenderDrawLine(m_Renderer, startX, startY, xr, yr);

					currentHalfEdge = edge->next_idx;
				} while (firstHalfEdge != currentHalfEdge);
			}
		}

		void ECMRenderer::DrawCorridor()
		{
			const PathPlanning::Corridor& c = m_AppState->corridorToDraw;
			//SDL_SetRenderDrawColor(m_Renderer, 215, 152, 0x00, 0xff);
			SDL_SetRenderDrawColor(m_Renderer, 215, 0x00, 0x00, 0xff);

			int leftSize = c.leftCorridorBounds.size() - 1;
			// left bounds
			for (int i = 0; i < leftSize; ++i)
			{
				float x1 = c.leftCorridorBounds[i].x * m_CamZoomFactor + m_CamOffsetX;
				float y1 = c.leftCorridorBounds[i].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float x2 = c.leftCorridorBounds[i+1].x * m_CamZoomFactor + m_CamOffsetX;
				float y2 = c.leftCorridorBounds[i+1].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			
				SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
			}
			
			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0x00, 215, 0xff);
			int rightSize = c.rightCorridorBounds.size() - 1;
			// right bounds
			for (int i = 0; i < rightSize; i++)
			{
				float x1 = c.rightCorridorBounds[i].x * m_CamZoomFactor + m_CamOffsetX;
				float y1 = c.rightCorridorBounds[i].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float x2 = c.rightCorridorBounds[i + 1].x * m_CamZoomFactor + m_CamOffsetX;
				float y2 = c.rightCorridorBounds[i + 1].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			
				SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
			}
		}

		void ECMRenderer::DrawPortals()
		{
			const std::vector<Segment>& portals = m_AppState->portalsToDraw;
			//SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 0xff);
			//
			//for (const Segment& portal : portals)
			//{
			//	float x1 = portal.p0.x * m_CamZoomFactor + m_CamOffsetX;
			//	float y1 = portal.p0.y * m_CamZoomFactor + m_CamOffsetY;
			//	float x2 = portal.p1.x * m_CamZoomFactor + m_CamOffsetX;
			//	float y2 = portal.p1.y * m_CamZoomFactor + m_CamOffsetY;
			//
			//	SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
			//}


			for (const Segment& portal : portals)
			{
				float x1 = portal.p0.x * m_CamZoomFactor + m_CamOffsetX;
				float y1 = portal.p0.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float x2 = portal.p1.x * m_CamZoomFactor + m_CamOffsetX;
				float y2 = portal.p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float midx = x1 + (x2 - x1) * 0.5f;
				float midy = y1 + (y2 - y1) * 0.5f;

				SDL_SetRenderDrawColor(m_Renderer, 215, 0x00, 0x00, 0xff);
				SDL_RenderDrawLine(m_Renderer, x1, y1, midx, midy);
				SDL_SetRenderDrawColor(m_Renderer, 0x00, 0x00, 215, 0xff);
				SDL_RenderDrawLine(m_Renderer, x2, y2, midx, midy);
			}

		}

		void ECMRenderer::DrawHalfEdge(int idx)
		{
			auto edge = m_Ecm->GetECMGraph().GetHalfEdge(idx);
			auto vert = m_Ecm->GetECMGraph().GetSource(edge);
			auto targetVert = m_Ecm->GetECMGraph().GetVertex(edge->v_target_idx);

			float x1 = edge->closest_left.x * m_CamZoomFactor + m_CamOffsetX;
			float y1 = edge->closest_left.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			float x2 = edge->closest_right.x * m_CamZoomFactor + m_CamOffsetX;
			float y2 = edge->closest_right.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			float midx = vert->position.x * m_CamZoomFactor + m_CamOffsetX;
			float midy = vert->position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			float targetX = targetVert->position.x * m_CamZoomFactor + m_CamOffsetX;
			float targetY = targetVert->position.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

			SDL_SetRenderDrawColor(m_Renderer, 215, 0x00, 0x00, 0xff);
			SDL_RenderDrawLine(m_Renderer, x1, y1, midx, midy);
			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0x00, 215, 0xff);
			SDL_RenderDrawLine(m_Renderer, x2, y2, midx, midy);
			SDL_SetRenderDrawColor(m_Renderer, 215, 215, 0x00, 0xff);
			SDL_RenderDrawLine(m_Renderer, targetX, targetY, midx, midy);

			HighlightECMVertex(vert->idx);
		}

		void ECMRenderer::DebugDrawECMCell()
		{
			if (!m_AppState->cellToDraw) return;

			// TODO: wrap in function
			const ECMCell* cell = m_AppState->cellToDraw;
			const Segment& obstacle = cell->boundary;
			Point p1 = m_AppState->ecm->GetECMGraph().GetVertex(cell->edge->half_edges[1].v_target_idx)->position;
			Point p2 = obstacle.p0;
			Point p3 = obstacle.p1;
			Point p4 = m_AppState->ecm->GetECMGraph().GetVertex(cell->edge->half_edges[0].v_target_idx)->position;

			p1.x = p1.x * m_CamZoomFactor + m_CamOffsetX;
			p2.x = p2.x * m_CamZoomFactor + m_CamOffsetX;
			p3.x = p3.x * m_CamZoomFactor + m_CamOffsetX;
			p4.x = p4.x * m_CamZoomFactor + m_CamOffsetX;
			p1.y = p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			p2.y = p2.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			p3.y = p3.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
			p4.y = p4.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

			SDL_SetRenderDrawColor(m_Renderer, 0xc4, 0x77, 0x02, 0xff);
			SDL_RenderDrawLine(m_Renderer, p1.x, p1.y, p2.x, p2.y);
			SDL_RenderDrawLine(m_Renderer, p2.x, p2.y, p3.x, p3.y);
			SDL_RenderDrawLine(m_Renderer, p3.x, p3.y, p4.x, p4.y);
			SDL_RenderDrawLine(m_Renderer, p4.x, p4.y, p1.x, p1.y);
		}


		void ECMRenderer::DebugDrawSecondaryLines()
		{
			using boost::polygon::voronoi_diagram;
			const auto& vd = m_Ecm->GetMedialAxis()->VD;

			SDL_SetRenderDrawColor(m_Renderer, 0x55, 0x55, 0xff, 0xff);

			for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
				vd.edges().begin(); it != vd.edges().end(); ++it)
			{
				if (it->is_finite() && it->is_secondary())
				{
					float x0 = it->vertex0()->x() * m_CamZoomFactor + m_CamOffsetX;
					float x1 = it->vertex1()->x() * m_CamZoomFactor + m_CamOffsetX;
					float y0 = it->vertex0()->y() * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					float y1 = it->vertex1()->y()* m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
				}
			}
		}

		void ECMRenderer::DebugDrawCellValues()
		{
			using boost::polygon::voronoi_diagram;
			const auto& vd = m_Ecm->GetMedialAxis()->VD;

			SDL_SetRenderDrawColor(m_Renderer, 0x55, 0x55, 0xff, 0xff);

			for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
				vd.edges().begin(); it != vd.edges().end(); ++it)
			{
				if (it->cell()->contains_point()) continue;

				if (it->is_finite() && it->is_secondary())
				{
					int segmentIndex = it->cell()->source_index();

					const Segment& s = m_Env->GetEnvironmentObstacleUnion()[segmentIndex];

					float x0 = s.p0.x * m_CamZoomFactor + m_CamOffsetX;
					float x1 = s.p1.x * m_CamZoomFactor + m_CamOffsetX;
					float y0 = s.p0.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					float y1 = s.p1.y* m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
				}
			}
		}

		void ECMRenderer::DebugDrawBoostVoronoiDiagram()
		{
			const auto& boostVD = m_AppState->ecm->GetMedialAxis()->VD;

			SDL_SetRenderDrawColor(m_Renderer, 0x55, 0x55, 0xff, 0xff);

			for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
				boostVD.edges().begin(); it != boostVD.edges().end(); ++it)
			{
				if (it->cell()->contains_point()) continue;

				if (it->is_finite() && it->is_primary())
				{
					//int segmentIndex = it->cell()->source_index();

					//const Segment& s = m_Env->GetEnvironmentObstacleUnion()[segmentIndex];

					Point p1(it->vertex0()->x(), it->vertex0()->y());
					Point p2(it->vertex1()->x(), it->vertex1()->y());

					float x0 = p1.x * m_CamZoomFactor + m_CamOffsetX;
					float x1 = p2.x * m_CamZoomFactor + m_CamOffsetX;
					float y0 = p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					float y1 = p2.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
				}
			}
		}

		// TODO: dynamically add 
		void ECMRenderer::DrawSimulationAreas()
		{
			// spawn
			auto spawnAreas = m_AppState->simulator->GetSpawnAreas();

			for (const Simulation::SpawnArea& sa : spawnAreas)
			{
				float halfWidth = sa.HalfWidth;
				float halfHeight = sa.HalfHeight;

				Point pos = WorldToScreenCoordinates(sa.Position.x - halfWidth, sa.Position.y + halfHeight);
				//float spawnX = m_CamOffsetX - halfWidth * m_CamZoomFactor;
				//float spawnY = (-375.0f + halfHeight) * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				SDL_Rect spawnRect;
				spawnRect.x = pos.x;
				spawnRect.y = pos.y;
				spawnRect.w = halfWidth * 2 * m_CamZoomFactor;
				spawnRect.h = halfHeight * 2 * m_CamZoomFactor;
				SDL_SetRenderDrawColor(m_Renderer, 120.0f, 200.0f, 120.0f, 255);
				SDL_RenderFillRect(m_Renderer, &spawnRect);
			}

			// goal
			auto goalAreas = m_AppState->simulator->GetGoalAreas();

			for (const Simulation::GoalArea& ga : goalAreas)
			{
				float halfWidth = ga.HalfWidth;
				float halfHeight = ga.HalfHeight;

				Point pos = WorldToScreenCoordinates(ga.Position.x - halfWidth, ga.Position.y + halfHeight);
				//float spawnX = m_CamOffsetX - halfWidth * m_CamZoomFactor;
				//float spawnY = (-375.0f + halfHeight) * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				SDL_Rect goalRect;
				goalRect.x = pos.x;
				goalRect.y = pos.y;
				goalRect.w = halfWidth * 2 * m_CamZoomFactor;
				goalRect.h = halfHeight * 2 * m_CamZoomFactor;
				SDL_SetRenderDrawColor(m_Renderer, 200.0f, 120.0f, 120.0f, 255);
				SDL_RenderFillRect(m_Renderer, &goalRect);
			}

		}

		void ECMRenderer::DrawPath()
		{
			PathPlanning::Path& path = m_AppState->pathToDraw;

			if (path.empty()) return;

			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0xaa, 0x00, 0xff);
			for (int i = 0 ; i < path.size() - 1; i++)
			{
				float x0 = path[i].x * m_CamZoomFactor + m_CamOffsetX;
				float x1 = path[i+1].x * m_CamZoomFactor + m_CamOffsetX;
				float y0 = path[i].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float y1 = path[i+1].y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
			}
		}

		void ECMRenderer::DrawAgents()
		{
			SDL_Rect rect;
			rect.x = -5;
			rect.y = -5;
			rect.w = 5;
			rect.h = 5;
			SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 255);
			SDL_RenderFillRect(m_Renderer, &rect);

			int lastIdx = m_AppState->simulator->GetLastIndex();
			const auto positions = m_AppState->simulator->GetPositionData();
			const auto velocities = m_AppState->simulator->GetVelocityData();
			const auto clearances = m_AppState->simulator->GetClearanceData();
			const auto activeFlags = m_AppState->simulator->GetActiveFlags();

			const float recip = sqrtf(2.0f);

			for (int i = 0; i <= lastIdx; i++)
			{
				if (!activeFlags[i]) continue;

				const auto& pos = positions[i];
				const auto& vel = velocities[i];
				const auto& clearance = clearances[i];

				float x = pos.x * m_CamZoomFactor + m_CamOffsetX;
				float y = pos.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				
				float vx = (pos.x + vel.dx) * m_CamZoomFactor + m_CamOffsetX;
				float vy = (pos.y + vel.dy) * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				float size = (clearance.clearance / recip) * m_CamZoomFactor;
				//float size = clearance.clearance * 2;

				//SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 255);
				//DrawCircle(m_Renderer, x, y, clearance.clearance);

				SDL_Rect rect;
				rect.x = x - size;
				rect.y = y - size;
				rect.w = size * 2;
				rect.h = size * 2;
				
				SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 255);
				//SDL_RenderFillRect(m_Renderer, &rect);

				DrawCircle(m_Renderer, x, y, clearance.clearance * m_CamZoomFactor);

				//SDL_SetRenderDrawColor(m_Renderer, 255, 0, 0, 255);
				//SDL_RenderDrawLineF(m_Renderer, x, y, vx, vy);
			}
		}

		void ECMRenderer::DrawPaths()
		{
			
			const auto paths = m_AppState->simulator->GetPathData();
			int numPaths = m_AppState->simulator->GetNumAgents();

			SDL_SetRenderDrawColor(m_Renderer, 0x00, 0xaa, 0x00, 0xff);
			for (int i = 0; i < numPaths; i++)
			{
				int numPoints = paths[i].numPoints;
				for (int j = 0; j < numPoints-1; j++)
				{
					float x0 = paths[i].x[j] * m_CamZoomFactor + m_CamOffsetX;
					float x1 = paths[i].x[j + 1] * m_CamZoomFactor + m_CamOffsetX;
					float y0 = paths[i].y[j] * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
					float y1 = paths[i].y[j + 1] * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
				}
			}
			
		}

		// https://stackoverflow.com/questions/38334081/how-to-draw-circles-arcs-and-vector-graphics-in-sdl
		void ECMRenderer::DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius)
		{
			const int32_t diameter = (radius * 2);

			int32_t x = (radius - 1);
			int32_t y = 0;
			int32_t tx = 1;
			int32_t ty = 1;
			int32_t error = (tx - diameter);

			while (x >= y)
			{
				//  Each of the following renders an octant of the circle
				SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
				SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
				SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
				SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
				SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
				SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
				SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
				SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

				if (error <= 0)
				{
					++y;
					error += ty;
					ty += 2;
				}

				if (error > 0)
				{
					--x;
					tx += 2;
					error += (tx - diameter);
				}
			}
		}
	}


}