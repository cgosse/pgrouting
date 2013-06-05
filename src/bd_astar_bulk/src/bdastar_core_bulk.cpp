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
int bdastar_bulk_wrapper(edge_astar_t *edges, unsigned int edge_count, int maxNode,
		routeset_t *routes, int route_count,
		bool directed, bool has_reverse_cost,
		path_element_bulk_t **path, int *path_count, char **err_msg)
{
	int res = 0;
	DBG("\n\ntrying the C++ code\n");
	try {
		// instantiate our class to hold the graph, we'll copy it for each route
		BiDirAStarBulk base(maxNode);

		DBG("constructing the graph\n");
		DBG("edges pointer %i\n", edges);
		DBG("last edge source now %i\n", edges[edge_count - 1].source);
		DBG("fifth edge id now %i\n", edges[4].id);

		base.construct_graph(edges, edge_count, maxNode);
		DBG("Constructed the graph\n");

		std::vector< std::vector<path_element_t> > paths;
		std::vector<int> rs;
		int path_row_count = 0;

		// then route each pair of verticies and build, we've set this to not worry about the path stuff 
		//for (int r = 0; r < route_count; ++r) {
		for (int r = 0; r < 1; ++r) {
			DBG("starting route %i in the wrapper\n", r);
			// make a local copy of the astar class with it's graph already built
			//  we're using the default copy constructor here, which should work since the class is all std:: or plain types
			//BiDirAStarBulk routeClass = base;
			// fetch the source and target from the routes array
			int source_vertex_id = routes[r].source, target_vertex_id = routes[r].target;
			// we'll need some dummy pointers to use the same function declaration
			// use our class copy to route an individual trip 
			// TODO check res for each individual run
			DBG("calling bdastar from %i to %i\n", source_vertex_id, target_vertex_id);
			res = base.bidir_astar_bulk(edges, edge_count, maxNode, source_vertex_id, target_vertex_id, err_msg);
			DBG("and the result is %i\n", res);

			// critical section
			{
				DBG("begin critical result aggregation for route %i\n", r);
				// fetch the resulting graph into a vector of path vectors. 
				paths.push_back(base.m_vecPath);
				// keep track of the route id that this path represents
				rs.push_back(routes[r].route_id);
				path_row_count += base.m_vecPath.size();
			}
		}
		// allocate memory for and return the resulting multi-paths
		//  we apparently don't use palloc here, since it doesn't work
		DBG("allocating result memory for %i path rows", path_row_count + 1);
		*path = (path_element_bulk_t *) malloc( (path_row_count + 1) * sizeof(path_element_bulk_t) );
		*path_count = path_row_count;

		for (int r = 0; r < route_count; ++r) {
			for(int i = 0; i < *path_count; i++) {
				(*path)[i].vertex_id = paths.at(r).at(i).vertex_id;
				(*path)[i].edge_id =  paths.at(r).at(i).edge_id;
				(*path)[i].cost =  paths.at(r).at(i).cost;
				// save the requested route id for each record, they may have been processed out of order
				(*path)[i].route_id = routes[ rs[r] ].route_id;
			}
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


