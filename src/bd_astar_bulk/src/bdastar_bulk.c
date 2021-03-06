/*
 * Bi Directional A* Shortest path algorithm for PostgreSQL
 *
 * Copyright (c) 2006 Anton A. Patrushev, Orkney, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include "postgres.h"
#include "executor/spi.h"
#include "funcapi.h"
#include "catalog/pg_type.h"

#include <stdio.h>
#include <stdlib.h>
#include <search.h>

#include "bdastar_bulk.h"


//-------------------------------------------------------------------------

Datum bidir_astar_shortest_path_bulk(PG_FUNCTION_ARGS);

#undef DEBUG
//#define DEBUG 1

#ifdef DEBUG
#define DBG(format, arg...)                     \
	elog(NOTICE, format , ## arg)
#else
#define DBG(format, arg...) do { ; } while (0)
#endif

// The number of tuples to fetch from the SPI cursor at each iteration
#define TUPLIMIT 1000

// this needs to be done once per library, assume somewhere else
//#ifdef PG_MODULE_MAGIC
//PG_MODULE_MAGIC;
//#endif

	static char *
text2char(text *in)
{
	char *out = palloc(VARSIZE(in));

	memcpy(out, VARDATA(in), VARSIZE(in) - VARHDRSZ);
	out[VARSIZE(in) - VARHDRSZ] = '\0';
	return out;
}

	static int
finish(int code, int ret)
{
	code = SPI_finish();
	if (code == SPI_ERROR_UNCONNECTED)
		DBG("tried to finish but wasn't connected\n");
	if (code  != SPI_OK_FINISH ) {
		elog(ERROR,"couldn't disconnect from SPI");
		return -1 ;
	}

	return ret;
}

typedef struct edge_astar_columns
{
	int id;
	int source;
	int target;
	int cost;
	int reverse_cost;
	int s_x;
	int s_y;
	int t_x;
	int t_y;
} edge_astar_columns_t;


	static int
fetch_edge_astar_columns(SPITupleTable *tuptable,
		edge_astar_columns_t *edge_columns,
		bool has_reverse_cost)
{
	edge_columns->id = SPI_fnumber(SPI_tuptable->tupdesc, "id");
	edge_columns->source = SPI_fnumber(SPI_tuptable->tupdesc, "source");
	edge_columns->target = SPI_fnumber(SPI_tuptable->tupdesc, "target");
	edge_columns->cost = SPI_fnumber(SPI_tuptable->tupdesc, "cost");

	if (edge_columns->id == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->source == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->target == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->cost == SPI_ERROR_NOATTRIBUTE) {

		elog(ERROR, "Error, query must return columns "
				"'id', 'source', 'target' and 'cost'");
		return -1;
	}

	if (SPI_gettypeid(SPI_tuptable->tupdesc,
				edge_columns->source) != INT4OID ||
			SPI_gettypeid(SPI_tuptable->tupdesc,
				edge_columns->target) != INT4OID ||
			SPI_gettypeid(SPI_tuptable->tupdesc, edge_columns->cost) != FLOAT8OID) {

		elog(ERROR, "Error, columns 'source', 'target' must be of type int4, "
				"'cost' must be of type float8");
		return -1;
	}

	DBG("columns: id %i source %i target %i cost %i",
			edge_columns->id, edge_columns->source,
			edge_columns->target, edge_columns->cost);

	if (has_reverse_cost) {
		edge_columns->reverse_cost = SPI_fnumber(SPI_tuptable->tupdesc,
				"reverse_cost");

		if (edge_columns->reverse_cost == SPI_ERROR_NOATTRIBUTE) {
			elog(ERROR, "Error, reverse_cost is used, but query did't return "
					"'reverse_cost' column");
			return -1;
		}

		if (SPI_gettypeid(SPI_tuptable->tupdesc,
					edge_columns->reverse_cost) != FLOAT8OID) {
			elog(ERROR, "Error, columns 'reverse_cost' must be of type float8");
			return -1;
		}

		DBG("columns: reverse_cost cost %i", edge_columns->reverse_cost);
	}

	edge_columns->s_x = SPI_fnumber(SPI_tuptable->tupdesc, "x1");
	edge_columns->s_y = SPI_fnumber(SPI_tuptable->tupdesc, "y1");
	edge_columns->t_x = SPI_fnumber(SPI_tuptable->tupdesc, "x2");
	edge_columns->t_y = SPI_fnumber(SPI_tuptable->tupdesc, "y2");

	if (edge_columns->s_x == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->s_y == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->t_x == SPI_ERROR_NOATTRIBUTE ||
			edge_columns->t_y == SPI_ERROR_NOATTRIBUTE) {

		elog(ERROR, "Error, query must return columns "
				"'x1', 'x2', 'y1' and 'y2'");
		return -1;
	}

	DBG("columns: x1 %i y1 %i x2 %i y2 %i",
			edge_columns->s_x, edge_columns->s_y,
			edge_columns->t_x,edge_columns->t_y);

	return 0;
}

	static void
fetch_edge_astar(HeapTuple *tuple, TupleDesc *tupdesc,
		edge_astar_columns_t *edge_columns,
		edge_astar_t *target_edge)
{
	Datum binval;
	bool isnull;

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->id, &isnull);
	if (isnull) elog(ERROR, "id contains a null value");
	target_edge->id = DatumGetInt32(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->source, &isnull);
	if (isnull) elog(ERROR, "source contains a null value");
	target_edge->source = DatumGetInt32(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->target, &isnull);
	if (isnull) elog(ERROR, "target contains a null value");
	target_edge->target = DatumGetInt32(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->cost, &isnull);
	if (isnull) elog(ERROR, "cost contains a null value");
	target_edge->cost = DatumGetFloat8(binval);

	if (edge_columns->reverse_cost != -1) {
		binval = SPI_getbinval(*tuple, *tupdesc,
				edge_columns->reverse_cost, &isnull);
		if (isnull)
			elog(ERROR, "reverse_cost contains a null value");
		target_edge->reverse_cost =  DatumGetFloat8(binval);
	}

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->s_x, &isnull);
	if (isnull) elog(ERROR, "source x contains a null value");
	target_edge->s_x = DatumGetFloat8(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->s_y, &isnull);
	if (isnull) elog(ERROR, "source y contains a null value");
	target_edge->s_y = DatumGetFloat8(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->t_x, &isnull);
	if (isnull) elog(ERROR, "target x contains a null value");
	target_edge->t_x = DatumGetFloat8(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, edge_columns->t_y, &isnull);
	if (isnull) elog(ERROR, "target y contains a null value");
	target_edge->t_y = DatumGetFloat8(binval);
}


/*// break out the fetching of edges and the routing verticies into separate functions
  static int query_edges(char* sql, bool directed, bool has_reverse_cost, edge_astar_t **edges, 
  int* edge_count, int* v_min_id, int* v_max_id)
  {
  }*/

	static int
fetch_route_columns(SPITupleTable *tuptable, routeset_t *route_columns)
{
	route_columns->route_id = SPI_fnumber(SPI_tuptable->tupdesc, "route_id");
	route_columns->source = SPI_fnumber(SPI_tuptable->tupdesc, "source");
	route_columns->target = SPI_fnumber(SPI_tuptable->tupdesc, "target");
	if (route_columns->route_id == SPI_ERROR_NOATTRIBUTE ||
			route_columns->source == SPI_ERROR_NOATTRIBUTE ||
			route_columns->target == SPI_ERROR_NOATTRIBUTE ) {
		elog(ERROR, "Error, query must return columns "
				"'route_id', 'source', 'target'");
		return -1;
	}
	if (SPI_gettypeid(SPI_tuptable->tupdesc,
				route_columns->source) != INT4OID ||
			SPI_gettypeid(SPI_tuptable->tupdesc,
				route_columns->target) != INT4OID ||
			SPI_gettypeid(SPI_tuptable->tupdesc, route_columns->route_id) != INT4OID) {

		elog(ERROR, "Error, columns 'source', 'target', and 'route_id' must be of type int4");
		return -1;
	}

	DBG("columns: route_id %i source %i target %i",
			route_columns->route_id, route_columns->source, route_columns->target);

	return 0;
}

	static void
fetch_route_row(HeapTuple *tuple, TupleDesc *tupdesc,
		routeset_t *route_columns, routeset_t *target_route)
{
	Datum binval;
	bool isnull;

	binval = SPI_getbinval(*tuple, *tupdesc, route_columns->route_id, &isnull);
	if (isnull) elog(ERROR, "route_id contains a null value");
	target_route->route_id = DatumGetInt32(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, route_columns->source, &isnull);
	if (isnull) elog(ERROR, "source contains a null value");
	target_route->source = DatumGetInt32(binval);

	binval = SPI_getbinval(*tuple, *tupdesc, route_columns->target, &isnull);
	if (isnull) elog(ERROR, "target contains a null value");
	target_route->target = DatumGetInt32(binval);
}

/*
static int query_routes(char* sql, routeset_t** routes, int* route_count)
{
	int SPIcode;
	void *SPIplan;
	Portal SPIportal;
	bool moredata = TRUE;
	int ntuples, ret = -1;
	int total_tuples = 0;

	routeset_t route_columns = {route_id: -1, source: -1, target: -1};

	DBG("fetching route information for bulk astar\n");

	SPIcode = SPI_connect();
	if (SPIcode  != SPI_OK_CONNECT) {
		elog(ERROR, "shortest_path_astar_bulk: couldn't open a connection to SPI");
		return -1;
	}

	SPIplan = SPI_prepare(sql, 0, NULL);
	if (SPIplan  == NULL) {
		elog(ERROR, "shortest_path_astar_bulk: couldn't create query plan via SPI");
		return -1;
	}

	if ((SPIportal = SPI_cursor_open(NULL, SPIplan, NULL, NULL, true)) == NULL) {
		elog(ERROR, "shortest_path_astar_bulk: SPI_cursor_open('%s') returns NULL",
				sql);
		return -1;
	}

	while (moredata == TRUE) {
		SPI_cursor_fetch(SPIportal, TRUE, TUPLIMIT);

		if (route_columns.route_id == -1) {
			if (fetch_route_columns(SPI_tuptable, &route_columns) == -1)
				return finish(SPIcode, ret);
		}

		ntuples = SPI_processed;
		total_tuples += ntuples;
		if (!*routes)
			*routes = palloc(total_tuples * sizeof(routeset_t));
		else
			*routes = repalloc(*routes, total_tuples * sizeof(routeset_t));

		if (*routes == NULL) {
			elog(ERROR, "Out of memory");
			return finish(SPIcode, ret);
		}

		if (ntuples > 0) {
			int t;
			SPITupleTable *tuptable = SPI_tuptable;
			TupleDesc tupdesc = SPI_tuptable->tupdesc;

			for (t = 0; t < ntuples; t++) {
				HeapTuple tuple = tuptable->vals[t];
				// use pointer arith to send a pointer to the row to add
				fetch_route_row(&tuple, &tupdesc, &route_columns,
						*routes + total_tuples - ntuples + t);
			}
			SPI_freetuptable(tuptable);
		}
		else {
			moredata = FALSE;
		}
	}
	*route_count = total_tuples;
	return finish(SPIcode, ret);
}
*/



static int compute_shortest_path_astar_bulk(char* edge_sql, char* route_sql, 
		bool directed, bool has_reverse_cost,
		path_element_bulk_t **path, int *path_count)
{

	edge_astar_t *edges = NULL;
	routeset_t *routes = NULL;
	int v_max_id=0;
	int v_min_id=INT_MAX;
	register int z, r;
	char *err_msg;
	int ret = -1;
	int route_count = 0, edge_count = 0;

	// fetch the edges from the supplied query
	//query_edges(edge_sql, directed, has_reverse_cost, &edges, &edge_count, &v_min_id, &v_max_id);
	int SPIcode;
	void *SPIplan;
	Portal SPIportal;
	bool moredata = TRUE;
	int ntuples;
	int total_tuples = 0;

	edge_astar_columns_t edge_columns = {id: -1, source: -1, target: -1,
		cost: -1, reverse_cost: -1,
		s_x: -1, s_y: -1, t_x: -1, t_y: -1};

	DBG("start shortest_path_astar\n");

	SPIcode = SPI_connect();
	if (SPIcode  != SPI_OK_CONNECT) {
		elog(ERROR, "shortest_path_astar: couldn't open a connection to SPI");
		return -1;
	}

	SPIplan = SPI_prepare(edge_sql, 0, NULL);
	if (SPIplan  == NULL) {
		elog(ERROR, "shortest_path_astar: couldn't create query plan via SPI");
		return -1;
	}

	if ((SPIportal = SPI_cursor_open(NULL, SPIplan, NULL, NULL, true)) == NULL) {
		elog(ERROR, "shortest_path_astar: SPI_cursor_open('%s') returns NULL",
				edge_sql);
		return -1;
	}

	while (moredata == TRUE) {
		SPI_cursor_fetch(SPIportal, TRUE, TUPLIMIT);

		if (edge_columns.id == -1) {
			if (fetch_edge_astar_columns(SPI_tuptable, &edge_columns,
						has_reverse_cost) == -1)
				return finish(SPIcode, ret);
		}

		ntuples = SPI_processed;
		total_tuples += ntuples;
		if (edges == NULL)
			edges = palloc(total_tuples * sizeof(edge_astar_t));
		else
			edges = repalloc(edges, total_tuples * sizeof(edge_astar_t));

		if (edges == NULL) {
			elog(ERROR, "Out of memory");
			return finish(SPIcode, ret);
		}

		if (ntuples > 0) {
			int t;
			SPITupleTable *tuptable = SPI_tuptable;
			TupleDesc tupdesc = SPI_tuptable->tupdesc;

			for (t = 0; t < ntuples; t++) {
				HeapTuple tuple = tuptable->vals[t];
				// since edges is a pointer to a pointer, deref to get a pointer then add, and it's still a pointer
				fetch_edge_astar(&tuple, &tupdesc, &edge_columns,
						&edges[total_tuples - ntuples + t]);
				//DBG("added edge id %i\n", edges[total_tuples - ntuples + t].id);
			}
			SPI_freetuptable(tuptable);
		}
		else {
			moredata = FALSE;
		}
	}
	// free the plan
	if (SPI_freeplan(SPIplan) != 0)
		DBG("trouble freeing the edge fetching plan\n");
	
	edge_count = total_tuples;
	DBG("got %i edges\n", total_tuples);
	// determine the min and max ids
	for(z=0; z < edge_count; ++z)
	{
		if(edges[z].source < v_min_id) v_min_id=edges[z].source;
		if(edges[z].source > v_max_id) v_max_id=edges[z].source;
		if(edges[z].target < v_min_id) v_min_id=edges[z].target;
		if(edges[z].target > v_max_id) v_max_id=edges[z].target;
		//DBG("%i <-> %i", *v_min_id, *v_max_id);
	}

	//if (finish(SPIcode, ret) != 0)
	//	DBG("error disconnecting from the DB after fetching edges\n");

	DBG("fetched %i edges. Min id %i, max id %i\n", edge_count, v_min_id, v_max_id);
	DBG("last edge id %i\n", edges[edge_count - 1].id);
	DBG("last edge source %i\n", edges[edge_count - 1].source);


	// fetch start and end locations from the supplied query
/*	if (query_routes(route_sql, &routes, &route_count) != 0)
		DBG("error fetching routes\n");
	else
		DBG("fetched %i routes\n", route_count);
*/

	DBG("fetching route information for bulk astar\n");
	routeset_t route_columns = {route_id: -1, source: -1, target: -1};
	// reset our counters 
	total_tuples = 0;
	moredata = TRUE;
	// we freed this plan pointer so we can use it again
	void *SPIRoutePlan = SPI_prepare(route_sql, 0, NULL);
	if (SPIRoutePlan  == NULL) {
		elog(ERROR, "shortest_path_astar_bulk: couldn't create query plan via SPI");
		return -1;
	}
	// use a new cursor to be safe
	Portal SPIRoutePortal;
	
	if ((SPIRoutePortal = SPI_cursor_open(NULL, SPIRoutePlan, NULL, NULL, true)) == NULL) {
		elog(ERROR, "shortest_path_astar_bulk: SPI_cursor_open('%s') returns NULL",
				route_sql);
		return -1;
	}
DBG("about to get data\n");

	while (moredata == TRUE) {
		DBG("fetching from the route cursor\n");
		SPI_cursor_fetch(SPIRoutePortal, TRUE, TUPLIMIT);
		DBG("fetched, but now checking if itwas good\n");
		if (route_columns.route_id == -1) {
			if (fetch_route_columns(SPI_tuptable, &route_columns) == -1)
				return finish(SPIcode, ret);
		}

		ntuples = SPI_processed;
		DBG("got %i route tuples\n", SPI_processed);
		total_tuples += ntuples;
		if (!routes)
			routes = palloc(total_tuples * sizeof(routeset_t));
		else
			routes = repalloc(routes, total_tuples * sizeof(routeset_t));

		if (routes == NULL) {
			elog(ERROR, "Out of memory");
			return finish(SPIcode, ret);
		}

		if (ntuples > 0) {
			int t;
			SPITupleTable *tuptable = SPI_tuptable;
			TupleDesc tupdesc = SPI_tuptable->tupdesc;

			for (t = 0; t < ntuples; t++) {
				HeapTuple tuple = tuptable->vals[t];
				// use pointer arith to send a pointer to the row to add
				fetch_route_row(&tuple, &tupdesc, &route_columns,
						&routes[total_tuples - ntuples + t]);
			}
			SPI_freetuptable(tuptable);
		}
		else {
			moredata = FALSE;
		}
	}
	route_count = total_tuples;

	// check that the sources and targets are in the graph
	for (r=0; r < route_count; ++r) {
		bool s_found = false, t_found = false;
		int source_vertex_id = routes[r].source, target_vertex_id = routes[r].target;
		DBG("Checking for start %i and target %i of route %i\n",source_vertex_id, target_vertex_id, r);
		for(z=0; z<edge_count; z++) {
			if(edges[z].source == source_vertex_id || edges[z].target == source_vertex_id)
				s_found = true;
			if(edges[z].source == target_vertex_id || edges[z].target == target_vertex_id)
				t_found = true;
		}
		if(! s_found) {
			elog(ERROR, "Start vertex was not found.");
			return -1;
		}

		if(! t_found) {
			elog(ERROR, "Target vertex was not found.");
			return -1;
		}
		//DBG("%i - %i", edges[z].source, edges[z].target);
	}

	//::::::::::::::::::::::::::::::::::::
	//:: reducing vertex id (renumbering)
	//::::::::::::::::::::::::::::::::::::
	for(z=0; z<edge_count; z++) {
		edges[z].source-=v_min_id;
		edges[z].target-=v_min_id;
	}

	for(r = 0; r < route_count; ++r) {
		// we don't need routes again, so just modify it here
		routes[r].source -= v_min_id;
		routes[r].target -= v_min_id;
	}

	DBG("Total %i tuples", edge_count);

	DBG("Calling bidir_astar <%i>\n", edge_count);
	DBG("last edge source now %i\n", edges[edge_count - 1].source);

	// calling C++ A* function
	ret = bdastar_bulk_wrapper(edges, edge_count, v_max_id + 1, 
			routes, route_count,
			directed, has_reverse_cost,
			path, path_count, &err_msg );

	DBG("SIZE %i\n",*path_count);

	DBG("ret =  %i\n",ret);

	//::::::::::::::::::::::::::::::::
	//:: restoring original vertex id
	//::::::::::::::::::::::::::::::::
	for(z=0; z < *path_count; z++) {
		//DBG("vetex %i\n",(*path)[z].vertex_id);
		(*path)[z].vertex_id+=v_min_id;
	}


	if (ret < 0) {
		elog(ERROR, "Error computing path: %s", err_msg);
	}

	return finish(SPIcode, ret);
}


PG_FUNCTION_INFO_V1(bidir_astar_shortest_path_bulk);
	Datum
bidir_astar_shortest_path_bulk(PG_FUNCTION_ARGS)
{
	FuncCallContext     *funcctx;
	int                  call_cntr;
	int                  max_calls;
	TupleDesc            tuple_desc;
	path_element_bulk_t      *path;

	/* stuff done only on the first call of the function */
	if (SRF_IS_FIRSTCALL()) {
		MemoryContext   oldcontext;
		int path_count = 0;
		int ret;

		/* create a function context for cross-call persistence */
		funcctx = SRF_FIRSTCALL_INIT();

		/* switch to memory context appropriate for multiple function calls */
		oldcontext = MemoryContextSwitchTo(funcctx->multi_call_memory_ctx);


		ret = compute_shortest_path_astar_bulk(
				text2char(PG_GETARG_TEXT_P(0)),
				text2char(PG_GETARG_TEXT_P(1)),
				PG_GETARG_BOOL(2),
				PG_GETARG_BOOL(3),
				&path, &path_count);

#ifdef DEBUG
		DBG("Ret is %i", ret);
		if (ret >= 0) {
			int i;
			for (i = 0; i < path_count; i++) {
				DBG("Step # %i vertex_id  %i ", i, path[i].vertex_id);
				DBG("        edge_id    %i ", path[i].edge_id);
				DBG("        cost       %f ", path[i].cost);
			}
		}
#endif

		/* total number of tuples to be returned */
		DBG("Conting tuples number\n");
		funcctx->max_calls = path_count;
		funcctx->user_fctx = path;

		DBG("Path count %i", path_count);

		funcctx->tuple_desc =
			BlessTupleDesc(RelationNameGetTupleDesc("pgr_cost3Result"));

		MemoryContextSwitchTo(oldcontext);
	}

	/* stuff done on every call of the function */
	DBG("Strange stuff doing\n");

	funcctx = SRF_PERCALL_SETUP();

	call_cntr = funcctx->call_cntr;
	max_calls = funcctx->max_calls;
	tuple_desc = funcctx->tuple_desc;
	path = (path_element_bulk_t*) funcctx->user_fctx;

	DBG("Trying to allocate some memory\n");

	if (call_cntr < max_calls) {   /* do when there is more left to send */
		HeapTuple    tuple;
		Datum        result;
		Datum *values;
		char* nulls;

		// cost3Result has 5 fields
		values = palloc(5 * sizeof(Datum));
		nulls = palloc(5 * sizeof(char));

		values[0] = Int32GetDatum(call_cntr);
		nulls[0] = ' ';
		values[1] = Int32GetDatum(path[call_cntr].vertex_id);
		nulls[1] = ' ';
		values[2] = Int32GetDatum(path[call_cntr].edge_id);
		nulls[2] = ' ';
		values[3] = Int32GetDatum(path[call_cntr].route_id);
		nulls[3] = ' ';
		values[4] = Float8GetDatum(path[call_cntr].cost);
		nulls[4] = ' ';

		DBG("should say %i %i %i %f\n", path[call_cntr].vertex_id, path[call_cntr].edge_id, path[call_cntr].route_id, path[call_cntr].cost);

		DBG("Heap making\n");

		tuple = heap_formtuple(tuple_desc, values, nulls);

		DBG("Datum making\n");

		/* make the tuple into a datum */
		result = HeapTupleGetDatum(tuple);

		DBG("Trying to free some memory\n");

		/* clean up (this is not really necessary) */
		pfree(values);
		pfree(nulls);
		DBG("trying to return next\n");
		SRF_RETURN_NEXT(funcctx, result);
	}
	else {   /* do when there is no more left */
		DBG("Freeing path since it was allocated upstream with malloc");
		if (path) free(path);
		DBG("returning final\n");
		SRF_RETURN_DONE(funcctx);
	}
}
