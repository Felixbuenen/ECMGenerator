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
			DrawECMVertices();
			//DrawClosestObstaclePoints();
			
			// TEST
			//DrawRandomTestPath();
			//DrawInsideVerts();
			DebugDrawECMCell();
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
					int y1 = edge.p0.y * m_CamZoomFactor + m_CamOffsetY;
					int x2 = edge.p1.x * m_CamZoomFactor + m_CamOffsetX;
					int y2 = edge.p1.y * m_CamZoomFactor + m_CamOffsetY;

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
				if (edge.IsArc())
				{
					std::vector<point_type> discr;
					point_type pt1(graph.GetVertex(edge.V0()).Position().x, graph.GetVertex(edge.V0()).Position().y);
					point_type pt2(graph.GetVertex(edge.V1()).Position().x, graph.GetVertex(edge.V1()).Position().y);
					discr.push_back(pt1);
					discr.push_back(pt2);

					const double maxDist = 0.5;

					if (Utility::MathUtility::SquareDistance(edge.NearestLeftV0(), edge.NearestLeftV1()) < Utility::EPSILON)
					{
						point_type p(edge.NearestLeftV0().x, edge.NearestLeftV0().y);
						point_type nr0(edge.NearestRightV0().x, edge.NearestRightV0().y);
						point_type nr1(edge.NearestRightV1().x, edge.NearestRightV1().y);
						segment_type seg(nr0, nr1);

						boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
					}
					else
					{
						point_type p(edge.NearestRightV0().x, edge.NearestRightV0().y);
						point_type nl0(edge.NearestLeftV0().x, edge.NearestLeftV0().y);
						point_type nl1(edge.NearestLeftV1().x, edge.NearestLeftV1().y);
						segment_type seg(nl0, nl1);

						boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
					}

					int numEdges = discr.size() - 1;
					for (int i = 0; i < numEdges; i++)
					{
						int x1 = discr[i].x() * m_CamZoomFactor + m_CamOffsetX;
						int y1 = discr[i].y() * m_CamZoomFactor + m_CamOffsetY;
						int x2 = discr[i + 1].x() * m_CamZoomFactor + m_CamOffsetX;
						int y2 = discr[i + 1].y() * m_CamZoomFactor + m_CamOffsetY;

						SDL_RenderDrawLine(m_Renderer, x1, y1, x2, y2);
					}
				}
				else
				{
					int x1 = verts[edge.V0()].Position().x * m_CamZoomFactor + m_CamOffsetX;
					int y1 = verts[edge.V0()].Position().y * m_CamZoomFactor + m_CamOffsetY;
					int x2 = verts[edge.V1()].Position().x * m_CamZoomFactor + m_CamOffsetX;
					int y2 = verts[edge.V1()].Position().y * m_CamZoomFactor + m_CamOffsetY;

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
				float x1 = s.p1.x * m_CamZoomFactor + m_CamOffsetX;
				float y0 = s.p0.y * m_CamZoomFactor + m_CamOffsetY;
				float y1 = s.p1.y * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
			}
		}

		void ECMRenderer::DrawECMVertices()
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();

			SDL_SetRenderDrawColor(m_Renderer, 0xff, 0x00, 0xff, 0xff);
			//SDL_RenderSetScale(_renderer, 2.0f, 2.0f);

			for (auto vert : verts)
			{
				float x = vert.Position().x * m_CamZoomFactor + m_CamOffsetX;
				float y = vert.Position().y * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawPoint(m_Renderer, x, y);
			}

			int result = SDL_RenderDrawPoint(m_Renderer, m_CamOffsetX, m_CamOffsetY);


			//SDL_RenderSetScale(_renderer, 1.0f, 1.0f);

		}

		void ECMRenderer::DrawInsideVerts()
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();

			SDL_SetRenderDrawColor(m_Renderer, 0xff, 0x00, 0x00, 0xff);
			//SDL_RenderSetScale(_renderer, 2.0f, 2.0f);

			for (auto vert : verts)
			{
				if (m_Env->InsideObstacle(vert.Position()))
				{
					float x = vert.Position().x * m_CamZoomFactor + m_CamOffsetX;
					float y = vert.Position().y * m_CamZoomFactor + m_CamOffsetY;

					const int size = 5;

					SDL_RenderDrawLine(m_Renderer, (x - size), y, (x + size), y);
					SDL_RenderDrawLine(m_Renderer, (x), (y - size), x, (y + size));
				}
			}

			//SDL_RenderSetScale(_renderer, 1.0f, 1.0f);
		}

		void ECMRenderer::DrawClosestObstaclePoints()
		{
			auto verts = m_Ecm->GetECMGraph().GetVertices();
			SDL_SetRenderDrawColor(m_Renderer, 0xff, 0x8c, 0x00, 0xff);

			for (const auto& vert : verts)
			{
				float startX = vert.Position().x * m_CamZoomFactor + m_CamOffsetX;
				float startY = vert.Position().y * m_CamZoomFactor + m_CamOffsetY;

				const auto& clPoints = vert.GetClosestPoints();
				for (const auto& clPoint : clPoints)
				{
					float x = clPoint.x * m_CamZoomFactor + m_CamOffsetX;
					float y = clPoint.y * m_CamZoomFactor + m_CamOffsetY;

					SDL_RenderDrawLine(m_Renderer, startX, startY, x, y);
				}
			}
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
				float y0 = s.p0.y * m_CamZoomFactor + m_CamOffsetY;
				float y1 = s.p1.y * m_CamZoomFactor + m_CamOffsetY;

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
					float y0 = it->vertex0()->y() * m_CamZoomFactor + m_CamOffsetY;
					float y1 = it->vertex1()->y() * m_CamZoomFactor + m_CamOffsetY;

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
					float y0 = s.p0.y * m_CamZoomFactor + m_CamOffsetY;
					float y1 = s.p1.y * m_CamZoomFactor + m_CamOffsetY;

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
				float y0 = path[i].y * m_CamZoomFactor + m_CamOffsetY;
				float y1 = path[i+1].y * m_CamZoomFactor + m_CamOffsetY;

				SDL_RenderDrawLine(m_Renderer, x0, y0, x1, y1);
			}
		}

	}
}