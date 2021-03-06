.. 
   ****************************************************************************
    pgRouting Manual
    Copyright(c) pgRouting Contributors

    This documentation is licensed under a Creative Commons Attribution-Share  
    Alike 3.0 License: http://creativecommons.org/licenses/by-sa/3.0/
   ****************************************************************************

.. _sampledata:

Sample Data
===============================================================================

The documentation provides very simple example queries based on a small sample network.
To be able to execute the sample queries, run the following SQL commands to create a table with a small network dataset.

.. image:: images/trsp-test-image.png

.. rubric:: Create table

.. code-block:: sql

	CREATE TABLE edge_table (
	    id serial,
	    dir character varying,
	    source integer,
	    target integer,
	    cost double precision,
	    reverse_cost double precision,
	    x1 double precision,
	    y1 double precision,
	    x2 double precision,
	    y2 double precision,
	    to_cost double precision,
	    rule text,
	    the_geom geometry(Linestring)
	);

.. code-block:: sql

	CREATE TABLE vertex_table (
	    id serial,
	    x double precision,
	    y double precision
	);


.. rubric:: Insert network data

.. code-block:: sql

	INSERT INTO edge_table VALUES (1, 'B', 1, 2, 1, 1, 2, 0, 2, 1, NULL, NULL, '010200000002000000000000000000004000000000000000000000000000000040000000000000F03F');
	INSERT INTO edge_table VALUES (2, 'TF', 2, 3, -1, 1, 2, 1, 3, 1, NULL, NULL, '0102000000020000000000000000000040000000000000F03F0000000000000840000000000000F03F');
	INSERT INTO edge_table VALUES (3, 'TF', 3, 4, -1, 1, 3, 1, 4, 1, NULL, NULL, '0102000000020000000000000000000840000000000000F03F0000000000001040000000000000F03F');
	INSERT INTO edge_table VALUES (4, 'B', 2, 7, 1, 1, 2, 1, 2, 2, NULL, NULL, '0102000000020000000000000000000040000000000000F03F00000000000000400000000000000040');
	INSERT INTO edge_table VALUES (5, 'FT', 3, 8, 1, -1, 3, 1, 3, 2, NULL, NULL, '0102000000020000000000000000000840000000000000F03F00000000000008400000000000000040');
	INSERT INTO edge_table VALUES (6, 'B', 5, 6, 1, 1, 0, 2, 1, 2, NULL, NULL, '01020000000200000000000000000000000000000000000040000000000000F03F0000000000000040');
	INSERT INTO edge_table VALUES (7, 'B', 6, 7, 1, 1, 1, 2, 2, 2, NULL, NULL, '010200000002000000000000000000F03F000000000000004000000000000000400000000000000040');
	INSERT INTO edge_table VALUES (8, 'B', 7, 8, 1, 1, 2, 2, 3, 2, NULL, NULL, '0102000000020000000000000000000040000000000000004000000000000008400000000000000040');
	INSERT INTO edge_table VALUES (9, 'B', 8, 9, 1, 1, 3, 2, 4, 2, NULL, NULL, '0102000000020000000000000000000840000000000000004000000000000010400000000000000040');
	INSERT INTO edge_table VALUES (10, 'B', 7, 10, 1, 1, 2, 2, 2, 3, NULL, NULL, '0102000000020000000000000000000040000000000000004000000000000000400000000000000840');
	INSERT INTO edge_table VALUES (11, 'FT', 8, 11, 1, -1, 3, 2, 3, 3, NULL, NULL, '0102000000020000000000000000000840000000000000004000000000000008400000000000000840');
	INSERT INTO edge_table VALUES (12, 'FT', 10, 11, 1, -1, 2, 3, 3, 3, NULL, NULL, '0102000000020000000000000000000040000000000000084000000000000008400000000000000840');
	INSERT INTO edge_table VALUES (13, 'FT', 11, 12, 1, -1, 3, 3, 4, 3, NULL, NULL, '0102000000020000000000000000000840000000000000084000000000000010400000000000000840');
	INSERT INTO edge_table VALUES (14, 'B', 10, 13, 1, 1, 2, 3, 2, 4, NULL, NULL, '0102000000020000000000000000000040000000000000084000000000000000400000000000001040');
	INSERT INTO edge_table VALUES (15, 'B', 9, 12, 1, 1, 4, 2, 4, 3, NULL, NULL, '0102000000020000000000000000001040000000000000004000000000000010400000000000000840');
	INSERT INTO edge_table VALUES (16, 'B', 4, 9, 1, 1, 4, 1, 4, 2, NULL, NULL, '0102000000020000000000000000001040000000000000F03F00000000000010400000000000000040');

.. code-block:: sql

	INSERT INTO vertex_table VALUES 
		(1,2,0), (2,2,1), (3,3,1), (4,4,1), (5,0,2), (6,1,2), (7,2,2),
		(8,3,2), (9,4,2), (10,2,3), (11,3,3), (12,4,3), (13,2,4);
