#include "ECMRenderer.h"
#include "ECM.h"
#include "Environment.h"
#include "ECMCellCollection.h"
#include "UtilityFunctions.h"
#include "Application.h"

#include "SDL.h"
#include "boost/polygon/voronoi.hpp"
#include <boost/polygon/polygon.hpp>
#include "BoostVisualizeUtils.h"

#include <stdio.h>

namespace ECM {
	namespace WindowApplication {

		void ECMRenderer::Initialize(Application* app)
		{
			m_Renderer = SDL_CreateRenderer(app->GetWindow(), -1, 0);
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
			//DebugDrawECMCell();
			//DebugDrawSecondaryLines();
			//DebugDrawCellValues();

			DrawPath();



			// render window
			SDL_RenderPresent(m_Renderer);
		}

		// Cache render-state variables, easier to reference the app state everywhere
		void ECMRenderer::UpdateRenderState()
		{
			m_CamZoomFactor = m_AppState->camZoomFactor;
			m_CamOffsetX = m_AppState->camOffsetX, 
			m_CamOffsetY = m_AppState->camOffsetY;
			m_YRotation = -1.0f;

			m_Env = &m_AppState->environment;
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
			// TODO: loop through all edges. For non-linear edges (arcs), sample the arc.
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

			SDL_SetRenderDrawColor(m_Renderer, 0xc4, 0x77, 0x02, 0xff);

			const std::vector<Segment>& segs = m_AppState->cellToDraw->boundary;

			for (const Segment& s : segs)
			{
				float x0 = s.p0.x * m_CamZoomFactor + m_CamOffsetX;
				float x1 = s.p1.x * m_CamZoomFactor + m_CamOffsetX;
				float y0 = s.p0.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;
				float y1 = s.p1.y * m_YRotation * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
			}
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
					// TODO:
					// > this demonstrates that we can find the closest segments in constant time.
					// > update the ecm construction algorithm such that it utilizes this information.
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

	}
}