.. 
   ****************************************************************************
    pgRouting Manual
    Copyright(c) pgRouting Contributors

    This documentation is licensed under a Creative Commons Attribution-Share  
    Alike 3.0 License: http://creativecommons.org/licenses/by-sa/3.0/
   ****************************************************************************

.. _index:

Table of Contents
===============================================================================

pgRouting extends the `PostGIS <http://postgis.net>`_/`PostgreSQL <http://postgresql.org>`_ geospatial database to provide geospatial routing and other network analysis functionality.

This is the manual for pgRouting |release|.

.. image:: static/images/ccbysa.png
	:align: left
	:alt: Creative Commons Attribution-Share Alike 3.0 License

The pgRouting Manual is licensed under a `Creative Commons Attribution-Share Alike 3.0 License <http://creativecommons.org/licenses/by-sa/3.0/>`_. Feel free to use this material any way you like, but we ask that you attribute credit to the pgRouting Project and wherever possible, a link back to http://pgrouting.org. For other licenses used in pgRouting see the :ref:`license` page.

*******************************************************************************
General
*******************************************************************************

.. toctree::
	:maxdepth: 1

	src/introduction/index
	src/introduction/support
	src/installation/index


*******************************************************************************
Tutorial
*******************************************************************************

.. toctree::
	:maxdepth: 1

	src/tutorial/index
	src/tutorial/topology
	src/tutorial/analytics
	src/tutorial/custom_query
	src/tutorial/custom_wrapper
	src/tutorial/recipes
	src/tutorial/performance

For a more complete introduction how to build a routing application read the `pgRouting Workshop <http://workshop.pgrouting.org>`_ (current version written for pgRouting 1.x).

*******************************************************************************
Reference
*******************************************************************************

pgRouting provides several :ref:`common functions <common>`:

.. toctree::
	:maxdepth: 1
	:hidden:

	../src/common/doc/index

.. toctree::
	:maxdepth: 1
	:glob: 

	../src/common/doc/functions/*

pgRouting defines a few :ref:`custom data types <common_types>`:

.. toctree::
	:maxdepth: 1
	:hidden:

	../src/common/doc/types

.. toctree::
	:maxdepth: 1
	:glob: 

	../src/common/doc/types/*

pgRouting functions in alphabetical order:

.. toctree::
	:maxdepth: 1

	../src/apsp_johnson/doc/index
	../src/apsp_warshall/doc/index
	../src/astar/doc/index
	../src/bd_astar/doc/index
	../src/bd_dijkstra/doc/index
	../src/dijkstra/doc/index
	../src/kdijkstra/doc/index
	../src/ksp/doc/index
	../src/tsp/doc/index
	../src/trsp/doc/index

If pgRouting is compiled with "Driving Distance" enabled:

.. toctree::
	:maxdepth: 1

	../src/driving_distance/doc/dd_alphashape
	../src/driving_distance/doc/index
	../src/driving_distance/doc/dd_points_as_polygon

Some functions from previous releases may have been removed. 

.. toctree::
	:maxdepth: 1

	../src/common/doc/legacy
	src/developer/discontinued


*******************************************************************************
Developer
*******************************************************************************

.. toctree::
	:maxdepth: 1

	src/installation/build
	src/developer/index
	src/developer/sampledata


.. rubric:: Release Notes

.. toctree::
	:maxdepth: 1

	src/changelog/2.0
	src/changelog/1.x
	

.. rubric:: Indices and tables

* :ref:`genindex`
* :ref:`search`

