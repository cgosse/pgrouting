/******************************************************************************
 * $Id$
 *
 * Project:  pgRouting bdsp and bdastar algorithms
 * Purpose:
 * Author:   Razequl Islam <ziboncsedu@gmail.com>
 *

 ******************************************************************************
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies of this Software or works derived from this Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.

 *****************************************************************************/

#include <exception>
#include "BiDirAStar_bulk.h"
#include "bdastar_bulk.h"

using namespace std;


// same idea as the basic wrapper, but takes an array of input and output vertices and returns a bulk path
//  which has an additional value for which route is being computed
int bdastar_bulk_wrapper(edge_astar_t *p_edges, unsigned int edge_count, int maxNode,
		routeset_t *p_routes, int route_count,
		bool directed, bool has_reverse_cost,
		path_element_bulk_t **path, int *path_count, char **err_msg)
{
	int res = 0;
	DBG("\n\ntrying the C++ code\n");
	try {
		DBG("fifth edge id now %i\n", p_edges[4].id);
		// allocating c++ memory for the C arrays since edges at least seems to get clobbered 
		edge_astar_t *edges = new edge_astar_t[edge_count];
		for (int e=0; e < edge_count; ++e)
			edges[e] = p_edges[e];
		// routes can be a vector since they're not passed around as pointers
		std::vector<routeset_t> routes(p_routes, p_routes + route_count);
		// instantiate our class to hold the graph, we'll copy it for each route
		BiDirAStarBulk baseAStar(maxNode);

		DBG("constructing the graph\n");
		DBG("edges pointer %i\n", edges);
		DBG("last edge source now %i\n", edges[edge_count - 1].source);
		DBG("fifth edge id now %i\n", edges[4].id);

		baseAStar.construct_graph(edges, edge_count, maxNode);
		// we're done with edges now, free the memory
		delete[] edges;
		DBG("Constructed the graph\n");

		std::vector< std::vector<path_element_t> > paths;
		std::vector<int> rs;
		std::vector<int> path_count_per_row;
		std::vector<bool> routeOK;
		int path_row_count = 0;

		// then route each pair of verticies and build, we've set this to not worry about the path stuff 
		//DBG("starting the parallel section with %i threads", omp_get_num_threads() );
//#pragma omp parallel for schedule(static) num_threads(4)
		for (int r = 0; r < route_count; ++r) {
			DBG("starting route %i in the wrapper\n", r);
			// make a thread-local copy of the astar class to use the pre-built graph
			BiDirAStarBulk childAStar(baseAStar);
			// fetch the source and target from the routes array
			int source_vertex_id = routes[r].source, target_vertex_id = routes[r].target;
			// we'll need some dummy pointers to use the same function declaration
			// use our class copy to route an individual trip 
			DBG("calling bdastar from %i to %i\n", source_vertex_id, target_vertex_id);
			res = childAStar.bidir_astar_bulk(maxNode, source_vertex_id, target_vertex_id, err_msg);
			DBG("and the result is %i\n", res);

			// critical section so that each thread can write to the output vector
//#pragma omp critical(combinepaths)
			{
				if (res == 0)
				{
					DBG("begin critical result aggregation for route %i\n", r);
					// fetch the resulting graph into a vector of path vectors. 
					paths.push_back(childAStar.m_vecPath);
					// keep track of the route id that this path represents
					rs.push_back(routes[r].route_id);
					routeOK.push_back(true);
					path_count_per_row.push_back(childAStar.m_vecPath.size());
					path_row_count += childAStar.m_vecPath.size();
				} else {
					DBG("Route %i had a problem", routes[r].route_id);
					routeOK.push_back(false);
				}
			}
		}
		// allocate memory for and return the resulting multi-paths
		//  we apparently don't use palloc here, since it doesn't work
		DBG("allocating result memory for %i path rows\n", path_row_count);
		*path = (path_element_bulk_t *) malloc( path_row_count * sizeof(path_element_bulk_t) );
		*path_count = path_row_count;
		// put the results back out into the C vector toward the db
		int p = 0; // the output vector index
		for (int r = 0; r < route_count; ++r) {
			// only return results from ok routes
			if (! routeOK.at(r))
				continue;
			//DBG("pushing out the results for r %i\n",r);
			for(int i = 0; i < path_count_per_row.at(r); ++i, ++p) {
				//DBG("  %i, %i, %i, %0.5f\n",paths.at(r).at(i).vertex_id, paths.at(r).at(i).edge_id,paths.at(r).at(i).cost, rs.at(r));
				(*path)[p].vertex_id = paths.at(r).at(i).vertex_id;
				(*path)[p].edge_id =  paths.at(r).at(i).edge_id;
				(*path)[p].cost =  paths.at(r).at(i).cost;
				// real route id for each record, they may have been processed out of order
				(*path)[p].route_id = rs.at(r);
			}
			//DBG("\nfinished the %ith route\n", r);
		}

	}
	catch(exception& e) {
		*err_msg = (char *) e.what();
		return -1;
	}
	catch(...) {
		*err_msg = (char *) "Caught unknown exception!";
		return -1;
	}

	if (res < 0)
		return res;
	else
		return EXIT_SUCCESS;
}


