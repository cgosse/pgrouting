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

#ifndef BiDirAStarBulk_H
#define BiDirAStarBulk_H

#include <vector>
#include <map>
#include <queue>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
// for openmp informational functions
//#include <omp.h>

#include "MinHeapBulk.h"
#include "bdastar_bulk.h"

#define INF 1e15

//#define DEBUG 
#ifdef DEBUG
#include <stdio.h>
static FILE *dbg;
#define DBG(format, arg...) \
    dbg = fopen("/tmp/sew-debug", "a"); \
    fprintf(dbg, format,  ## arg); \
    fclose(dbg);
#else
#define DBG(format, arg...) do { ; } while (0)
#endif


typedef std::vector<long> LongVector;
typedef std::vector<LongVector> VectorOfLongVector;
//typedef std::pair<int, bool> PIB;
typedef std::pair<double, int> PDI;
//typedef std::pair<double, std::vector<int> > PDVI;
/*
typedef struct edge 
{
	int id;
	int source;
	int target;
	double s_x;
	double s_y;
	double t_x;
	double t_y;
	double cost;
	double reverse_cost;
} edge_astar_t;

typedef struct path_element 
{
	int vertex_id;
	int edge_id;
	double cost;
}path_element_t;
*/

typedef struct{
	int par_Node;
	int par_Edge;
}PARENT_PATH;

typedef struct{
	int NodeID;
	double xpos;
	double ypos;
	std::vector<int> Connected_Nodes;
	std::vector<int> Connected_Edges_Index;
}GraphNodeInfo;

struct GraphEdgeInfo
{
public:
	int EdgeID;
	int EdgeIndex;
	int Direction;
	double Cost;
	double ReverseCost;
	int StartNode;
	int EndNode;
};

typedef std::vector<GraphEdgeInfo> GraphEdgeVector;
typedef std::map<long,LongVector> Long2LongVectorMap;
typedef std::map<long,long> Long2LongMap;
typedef std::vector<GraphNodeInfo> GraphNodeVector;


class BiDirAStarBulk
{
public:
	BiDirAStarBulk(void);
	// allow instantiation with maxNode to allocate memory
	BiDirAStarBulk(int);
	// copy constructor to enable multi-threading
	BiDirAStarBulk(const BiDirAStarBulk &a);
	// destructor to clean up dynamic memory
	~BiDirAStarBulk(void);
	
	int bidir_astar_bulk(unsigned int maxNode, unsigned int start_vertex, unsigned int end_vertex, char **err_msg);

	// these are public to call them early and pre-build the graph
	std::vector <path_element_t> m_vecPath;
	bool construct_graph(edge_astar_t *edges, unsigned int edge_count, unsigned int maxNode);

private:
	void fconstruct_path(int node_id);
	void rconstruct_path(int node_id);
	bool addEdge(edge_astar_t edgeIn);
	bool connectEdge(GraphEdgeInfo& firstEdge, GraphEdgeInfo& secondEdge, bool bIsStartNodeSame);
	void initall(int maxNode);
	void explore(int cur_node, double cur_cost, int dir, MinHeapBulk &que);
	double getcost(int node_id, int dir);
	void setcost(int node_id, int dir, double c);
	void setparent(int node_id, int dir, int parnode, int paredge);
	double gethcost(int node_id, int dir);
	double dist(double x1, double y1, double x2, double y2);

private:
	GraphEdgeVector m_vecEdgeVector;
	Long2LongMap m_mapEdgeId2Index;
	Long2LongVectorMap m_mapNodeId2Edge;
	GraphNodeVector m_vecNodeVector;
	int max_node_id;
	int max_edge_id;
	int m_lStartNodeId;
	int m_lEndNodeId;

	double m_MinCost;
	int m_MidNode;
	PARENT_PATH *m_pFParent;
	PARENT_PATH *m_pRParent;
	double *m_pFCost;
	double *m_pRCost;
};

#endif
