-----------------------------------------------------------------------
-- Core function for bi_directional_astar_shortest_path_bulk computation
-- See README for description
-----------------------------------------------------------------------
--
--
 CREATE OR REPLACE FUNCTION pgr_bdAstar_bulk(
		sql text, 
		route_sql text,
	        directed boolean, 
        	has_reverse_cost boolean)
        RETURNS SETOF pgr_cost3Result
        AS '$libdir/librouting_bd', 'bidir_astar_shortest_path_bulk'
        LANGUAGE 'c' IMMUTABLE STRICT;

