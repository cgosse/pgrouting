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

#include "BiDirAStar_bulk.h"

BiDirAStarBulk::BiDirAStarBulk(void)
{
}

BiDirAStarBulk::~BiDirAStarBulk(void)
{
	// any reason not to do this?
	deleteall();
}

void BiDirAStarBulk::init()
{
	//max_edge_id = 0;
	//max_node_id = 0;
	
}

/*
	Initialization and allocation of memories.
*/

void BiDirAStarBulk::initall(int maxNode)
{
	int i;
	m_pFParent = new PARENT_PATH[maxNode + 1];
	m_pRParent = new PARENT_PATH[maxNode + 1];

	m_pFCost = new double[maxNode + 1];
	m_pRCost = new double[maxNode + 1];

	for(i = 0; i <= maxNode; i++)
	{
		m_pFParent[i].par_Node = -2;
		m_pRParent[i].par_Node = -2;
		m_pFCost[i] = INF;
		m_pRCost[i] = INF;
		
	}
	m_MinCost = INF;
	m_MidNode = -1;
}

/*
	Delete the allocated memories to avoid memory leak.
*/

void BiDirAStarBulk::deleteall()
{
	delete [] m_pFParent;
	delete [] m_pRParent;
	delete [] m_pFCost;
	delete [] m_pRCost;
}

/*
	Get the current cost from source to the current node if direction is 1 else the cost to reach target from the current node.
*/

double BiDirAStarBulk::getcost(int node_id, int dir)
{
	if(dir == 1)
	{
		return(m_pFCost[node_id]);
	}
	else
	{
		return(m_pRCost[node_id]);
	}
}


double BiDirAStarBulk::dist(double x1, double y1, double x2, double y2)
{
	double ret = fabs((x1 - x2) + fabs(y1 - y2));
	//double ret = sqrt(x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	return(ret * 10);
}

/*
	Get the heuristic cost of the node depending on dir (1 for forward search and -1 for reverse search).
*/
double BiDirAStarBulk::gethcost(int node_id, int dir)
{
	if(dir == -1)
	{
		return(dist(m_vecNodeVector[node_id].xpos, m_vecNodeVector[node_id].ypos, m_vecNodeVector[m_lStartNodeId].xpos, m_vecNodeVector[m_lStartNodeId].ypos));
	}
	else
	{
		return(dist(m_vecNodeVector[node_id].xpos, m_vecNodeVector[node_id].ypos, m_vecNodeVector[m_lEndNodeId].xpos, m_vecNodeVector[m_lEndNodeId].ypos));
	}
}

/*
	Set the forward or reverse cost list depending on dir (1 for forward search and -1 for reverse search).
*/


void BiDirAStarBulk::setcost(int node_id, int dir, double c)
{
	if(dir == 1)
	{
		m_pFCost[node_id] = c;
	}
	else
	{
		m_pRCost[node_id] = c;
	}
}

void BiDirAStarBulk::setparent(int node_id, int dir, int parnode, int paredge)
{
	if(dir == 1)
	{
		m_pFParent[node_id].par_Node = parnode;
		m_pFParent[node_id].par_Edge = paredge;
	}
	else
	{
		m_pRParent[node_id].par_Node = parnode;
		m_pRParent[node_id].par_Edge = paredge;
	}
}

/*
	Reconstruct path for forward search. It is like normal dijkstra. The parent array contains the parent of the current node and there is a -1 in the source.
	So one need to recurse upto the source and then add the current node and edge to the list.
*/

void BiDirAStarBulk::fconstruct_path(int node_id)
{
	if(m_pFParent[node_id].par_Node == -1)
		return;
	fconstruct_path(m_pFParent[node_id].par_Node);
	path_element_t pt;
	pt.vertex_id = m_pFParent[node_id].par_Node;
	pt.edge_id = m_pFParent[node_id].par_Edge;
	pt.cost = m_pFCost[node_id] - m_pFCost[m_pFParent[node_id].par_Node];
	m_vecPath.push_back(pt);
}

/*
	Reconstruct path for the reverse search. In this case the subsequent node is stored in the parent and the target contains a -1. So one need to add the node
	and edge to the list and then recurse through the parent upto hitting a -1.
*/

void BiDirAStarBulk::rconstruct_path(int node_id)
{
	path_element_t pt;
	if(m_pRParent[node_id].par_Node == -1)
	{
		pt.vertex_id = node_id;
		pt.edge_id = -1;
		pt.cost = 0.0;
		return;
	}
	pt.vertex_id = node_id;
	pt.cost = m_pRCost[node_id] - m_pRCost[m_pRParent[node_id].par_Node];
	pt.edge_id = m_pRParent[node_id].par_Edge;
	m_vecPath.push_back(pt);
	rconstruct_path(m_pRParent[node_id].par_Node);
}

/*
	This is the main exploration module. The parameter dir indicates whether the exploration will be in forward or reverser direction. The reference to the corresponding
	que is also passed as parameter que. The current node and the current costs are also available as parameter.
*/

//void BiDirAStarBulk::explore(int cur_node, double cur_cost, int dir, std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > &que)
void BiDirAStarBulk::explore(int cur_node, double cur_cost, int dir, MinHeapBulk &que)
{
	int i;
	// Number of connected edges
	int con_edge = m_vecNodeVector[cur_node].Connected_Edges_Index.size();
	double edge_cost;
	for(i = 0; i < con_edge; i++)
	{
		int edge_index = m_vecNodeVector[cur_node].Connected_Edges_Index[i];
		// Get the edge from the edge list.
		GraphEdgeInfo edge = m_vecEdgeVector[edge_index];
		// Get the connected node
		int new_node = m_vecNodeVector[cur_node].Connected_Nodes[i];
		int mult;
		
		if(edge.Direction == 0)
			mult = 1;
		else
			mult = dir;
		if(cur_node == edge.StartNode)
		{
			// Current node is the startnode of the edge. For forward search it should use forward cost, otherwise it should use the reverse cost,
			// i.e. if the reverse direction is valid then this node may be visited from the end node.
			if(dir > 0)
				edge_cost = edge.Cost;
			else
				edge_cost = edge.ReverseCost;
			// Check if the direction is valid for exploration
			if(edge.Direction == 0 || edge_cost >= 0.0)
			{
				//edge_cost = edge.Cost * mult;
				// Check if the current edge gives better result
				if(cur_cost + edge_cost < getcost(new_node, dir))
				{
					// explore the node, and push it in the queue. the value in the queue will also contain the heuristic cost
					setcost(new_node, dir, cur_cost + edge_cost);
					setparent(new_node, dir, cur_node, edge.EdgeID);
					que.push(std::make_pair(cur_cost + edge_cost + gethcost(new_node, dir), new_node));

					// Update the minimum cost found so far.
					if(getcost(new_node, dir) + getcost(new_node, dir * -1) < m_MinCost)
					{
						m_MinCost = getcost(new_node, dir) + getcost(new_node, dir * -1);
						m_MidNode = new_node;
					}
				}
			}
		}
		else
		{
			// Current node is the endnode of the edge. For forward search it should use reverse cost, otherwise it should use the forward cost,
			// i.e. if the forward direction is valid then this node may be visited from the start node.
			if(dir > 0)
				edge_cost = edge.ReverseCost;
			else
				edge_cost = edge.Cost;
			// Check if the direction is valid for exploration
			if(edge.Direction == 0 || edge_cost >= 0.0)
			{
				//edge_cost = edge.ReverseCost * mult;
				
				// Check if the current edge gives better result
				if(cur_cost + edge_cost < getcost(new_node, dir))
				{
					// explore the node, and push it in the queue. the value in the queue will also contain the heuristic cost
					setcost(new_node, dir, cur_cost + edge_cost);
					setparent(new_node, dir, cur_node, edge.EdgeID);
					que.push(std::make_pair(cur_cost + edge_cost + gethcost(new_node, dir), new_node));
					// Update the minimum cost found so far.
					if(getcost(new_node, dir) + getcost(new_node, dir * -1) < m_MinCost)
					{
						m_MinCost = getcost(new_node, dir) + getcost(new_node, dir * -1);
						m_MidNode = new_node;
					}
				}
			}
		}
	}
}

/* 
	This is the entry function that the wrappers should call. Most of the parameters are trivial. maxNode refers to Maximum
	node id. As we run node based exploration cost, parent etc will be based on maximam node id.
*/

int BiDirAStarBulk:: bidir_astar_bulk(edge_astar_t *edges, unsigned int edge_count, 
		unsigned int maxNode, unsigned int start_vertex, unsigned int end_vertex, 
		char **err_msg)
{
	m_lStartNodeId = start_vertex;
	m_lEndNodeId = end_vertex;

	int nodeCount = m_vecNodeVector.size();
	
	MinHeapBulk fque(maxNode + 2);
	MinHeapBulk rque(maxNode + 2);
	//std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > fque;
	//std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > rque;
	
	m_vecPath.clear();

	// Initialize the forward search
	m_pFParent[start_vertex].par_Node = -1;
	m_pFParent[start_vertex].par_Edge = -1;
	m_pFCost[start_vertex] = 0.0;
	fque.push(std::make_pair(0.0, start_vertex));

	// Initialize the reverse search
	m_pRParent[end_vertex].par_Node = -1;
	m_pRParent[end_vertex].par_Edge = -1;
	m_pRCost[end_vertex] = 0.0;
	rque.push(std::make_pair(0.0, end_vertex));


	int i;
	int new_node;
	int cur_node;
	int dir;
/*
	The main loop. The algorithm is as follows:
	1. IF the sum of the current minimum of both heap is greater than so far found path, we cannot get any better, so break the loop.
	2. IF the reverse heap minimum is lower than the forward heap minimum, explore from reverse direction.
	3. ELSE explore from the forward directtion.
*/
	DBG("Starting Iterations");
	while(!fque.empty() && !rque.empty())
	{
		PDI fTop = fque.top();
		PDI rTop = rque.top();
		if(m_pFCost[fTop.second] + m_pRCost[rTop.second] > m_MinCost) //We are done, there is no path with lower cost
			break;

		if(rTop.first < fTop.first) // Explore from reverse queue
		{
			if(rTop.first > m_MinCost)
				break;
			cur_node = rTop.second;
			int dir = -1;
			rque.pop();
			explore(cur_node, m_pRCost[rTop.second], dir, rque);
		}
		else                        // Explore from forward queue
		{
			if(fTop.first > m_MinCost)
				break;
			cur_node = fTop.second;
			int dir = 1;
			fque.pop();
			explore(cur_node, m_pFCost[fTop.second], dir, fque);
		}
	}

/*
	Path reconstruction part. m_MidNode is the joining point where two searches meet to make a shortest path. It is updated in explore.
	If it contains -1, then no path is found. Other wise we have a shortest path and that is reconstructed in the m_vecPath.
*/

	if(m_MidNode == -1)
	{
		*err_msg = (char *)"Path Not Found";
		deleteall();
		return -1;
	}
	else 	
	{
		// reconstruct path from forward search
		fconstruct_path(m_MidNode);
		// reconstruct path from backward search
		rconstruct_path(m_MidNode);

		// insert the last row in the path trace (having edge_id = -1 and cost = 0.0)
		path_element_t pelement;
		pelement.vertex_id = end_vertex;
		pelement.edge_id = -1;
		pelement.cost = 0.0;
		m_vecPath.push_back(pelement);

	}
	return 0;
}

/*
	Populate the member variables of the class using the edge list. Basically there is a node list and an edge list. Each node contains the list of adjacent nodes and 
	corresponding edge indices from edge list that connect this node with the adjacent nodes.
*/

bool BiDirAStarBulk::construct_graph(edge_astar_t* edges, unsigned int edge_count, unsigned int maxNode)
{
	DBG("edges pointer %i\n", edges);
	DBG("last edge source now %i\n", edges[edge_count - 1].source);
	DBG("fifth edge id now %i\n", edges[4].id);
	DBG("last edge id now %i\n", edges[edge_count - 1].id);
	max_node_id = maxNode;
	max_edge_id = -1;

	// allocate memory
	initall(maxNode);

	int i;
	// Create a dummy node for building the node structure
	GraphNodeInfo nodeInfo;
	nodeInfo.Connected_Edges_Index.clear();
	nodeInfo.Connected_Nodes.clear();

	// Insert the dummy node into the node list. This acts as place holder. Also change the nodeId so that nodeId and node index in the vector are same.
	// There may be some nodes here that does not appear in the edge list. The size of the list is upto maxNode which is equal to maximum node id.
	// no reason that we can't fill the vector first, then assign id, though hard to say if faster
	m_vecNodeVector.assign(maxNode + 1, nodeInfo);
	for(i = 0; i <= maxNode; ++i)
	{
		m_vecNodeVector.at(i).NodeID = i;
	}
	DBG("processed up to %i nodes, about to process %i edges\n", maxNode, edge_count);
	m_vecEdgeVector.reserve(edge_count);
	DBG("fifth edge id now %i\n", edges[4].id);
	// Process each edge from the edge list and update the member data structures accordingly.
	for(i = 0; i < edge_count; ++i)
	{
		addEdge(edges[i]);
	}
	DBG("added %i edges\n", edge_count);
	return true;
}

/*
	Process the edge and populate the member nodelist and edgelist. The nodelist already contains upto maxNode dummy entries with nodeId same as index. Now the
	connectivity information needs to be updated.
*/

bool BiDirAStarBulk::addEdge(edge_astar_t edgeIn)
{
	//DBG("Adding edge id %i \n", edgeIn.id);
	long lTest;
	// Check if the edge is already processed.
	Long2LongMap::iterator itMap = m_mapEdgeId2Index.find(edgeIn.id);
	if(itMap != m_mapEdgeId2Index.end())	
		return false;

	// Create a GraphEdgeInfo using the information of the current edge
	GraphEdgeInfo newEdge;
	newEdge.EdgeID = edgeIn.id;
	newEdge.EdgeIndex = m_vecEdgeVector.size();	
	newEdge.StartNode = edgeIn.source;
	newEdge.EndNode = edgeIn.target;
	newEdge.Cost = edgeIn.cost;
	newEdge.ReverseCost = edgeIn.reverse_cost;

	// Set the direction. If both cost and reverse cost has positive value the edge is bidirectional and direction field is 0. If cost is positive and reverse cost
	// negative then the edge is unidirectional with direction = 1 (goes from source to target) otherwise it is unidirectional with direction = -1 (goes from target
	// to source). Another way of creating unidirectional edge is assigning big values in cost or reverse_cost. In that case the direction is still zero and this case
	// is handled in the algorithm automatically.
	if(newEdge.Cost >= 0.0 && newEdge.ReverseCost >= 0)
	{
		newEdge.Direction = 0;
	}
	else if(newEdge.Cost >= 0.0)
	{
		newEdge.Direction = 1;
	}
	else
	{
		newEdge.Direction = -1;
	}

	if(edgeIn.id > max_edge_id)
	{
		max_edge_id = edgeIn.id;
	}

	// Update max_edge_id
	if(newEdge.StartNode > max_node_id)
	{
		return false;//max_node_id = newEdge.StartNode;
	}
	if(newEdge.EndNode > max_node_id)
	{
		return false;//max_node_id = newEdge.EdgeIndex;
	}
	//DBG("Setting new edge start position for %i", edgeIn.id);
	m_vecNodeVector.at(newEdge.StartNode).xpos = edgeIn.s_x;
	m_vecNodeVector.at(newEdge.StartNode).ypos = edgeIn.s_y;

	m_vecNodeVector.at(newEdge.EndNode).xpos = edgeIn.t_x;
	m_vecNodeVector.at(newEdge.EndNode).ypos = edgeIn.t_y;

	// update connectivity information for the start node.
	m_vecNodeVector.at(newEdge.StartNode).Connected_Nodes.push_back(newEdge.EndNode);
	m_vecNodeVector.at(newEdge.StartNode).Connected_Edges_Index.push_back(newEdge.EdgeIndex);

	// update connectivity information for the end node.
	m_vecNodeVector.at(newEdge.EndNode).Connected_Nodes.push_back(newEdge.StartNode);
	m_vecNodeVector.at(newEdge.EndNode).Connected_Edges_Index.push_back(newEdge.EdgeIndex);

	//Adding edge to the list
	m_mapEdgeId2Index.insert(std::make_pair(newEdge.EdgeID, m_vecEdgeVector.size()));
	m_vecEdgeVector.push_back(newEdge);
	//DBG(" done and added to the list");

	//
	return true;
}
