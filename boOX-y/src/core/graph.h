/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             boOX 1-224_leibniz                             */
/*                                                                            */
/*                  (C) Copyright 2018 - 2020 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* graph.h / 1-224_leibniz                                                    */
/*----------------------------------------------------------------------------*/
//
// Graph related data structures and algorithms.
//
/*----------------------------------------------------------------------------*/

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <vector>
#include <list>
#include <set>
#include <map>

#include "result.h"

#include "common/types.h"


using namespace boOX;


/*----------------------------------------------------------------------------*/

namespace boOX
{

    class sEdge;
    class sArc;
    class sVertex;

   
/*----------------------------------------------------------------------------*/
// sVertex

    class sVertex
    {
    public:
	static const sInt_32 ORDER_UNDEFINED = -1;

    public:
	typedef std::list<sArc*> Neighbors_list;
	typedef std::vector<sArc*> Neighbors_vector;
	typedef std::vector<sInt_32> VertexIDs_vector;

    public:
	sVertex();
	sVertex(sInt_32 id);
	sVertex(sInt_32 id, sInt_32 capacity);	
	sVertex(const sVertex &vertex);
	const sVertex& operator=(const sVertex &vertex);

	sInt_32 calc_NeighborCount(void) const;
	sInt_32 calc_NeighborOrder(sInt_32 vertex_id) const;
	sInt_32 calc_NeighborID(sInt_32 order) const;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	sInt_32 m_id;
	
	Neighbors_list m_Neighbors;
	Neighbors_list m_in_Neighbors;
	Neighbors_list m_out_Neighbors;

	bool m_visited;
	sInt_32 m_distance;
	sInt_32 m_prev_id;

	sInt_32 m_capacity;

	VertexIDs_vector m_Conflicts;
    };


/*----------------------------------------------------------------------------*/
// sArc

    class sArc
    {
    public:
	sArc();
	sArc(sEdge *edge, sVertex *source, sVertex *target);
	sArc(const sArc &arc);
	const sArc& operator=(const sArc &arc);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	sEdge *m_edge;

	sVertex *m_source;
	sVertex *m_target;
    };


/*----------------------------------------------------------------------------*/
// sEdge

    class sEdge
    {
    public:
	sEdge(sInt_32 id, sVertex *vertex_u, sVertex *vertex_v, bool directed = false);
	sEdge(const sEdge &edge);
	const sEdge& operator=(const sEdge &edge);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	sInt_32 m_id;
	bool m_directed;

	sArc m_arc_uv;
	sArc m_arc_vu;
    };


/*----------------------------------------------------------------------------*/
// sUndirectedGraph

    class sUndirectedGraph
    {
    public:
	struct Edge
	{
	    Edge(sInt_32 u_id, sInt_32 v_id)
	    {
		m_u_id = u_id;
		m_v_id = v_id;
	    }
	    
	    sInt_32 m_u_id;
	    sInt_32 m_v_id;
	};

	struct Coordinates
	{
	    Coordinates() { }
	    Coordinates(sInt_32 row, sInt_32 column)
	    {
		m_row = row;
		m_column = column;
	    }
	    
	    sInt_32 m_row;
	    sInt_32 m_column;
	};

	typedef std::vector<Edge> Edges_vector;
	typedef std::vector<Coordinates> Coordinates_vector;

	typedef std::vector<sInt_32> Distances_vector; 
	typedef std::vector<Distances_vector> Distances_2d_vector;
	typedef std::vector<Distances_2d_vector> Distances_3d_vector;
	typedef std::vector<Distances_3d_vector> Distances_4d_vector;

	typedef std::vector<sInt_32> VertexIDs_vector;
	typedef std::list<sInt_32> VertexIDs_list;
	typedef std::set<sInt_32> VertexIDs_set;

	typedef std::vector<sVertex> Vertices_vector;
	typedef std::list<sVertex*> Vertices_list;	

	typedef std::list<sEdge> Edges_list;
	typedef std::multimap<sInt_32, sInt_32, std::less<sInt_32> > VertexQueue_multimap;

	typedef std::pair<sInt_32, sInt_32> Vertex_pair;
	typedef std::vector<Vertex_pair> VertexPairs_vector;
	typedef std::multimap<sInt_32, Vertex_pair, std::less<sInt_32> > VertexPairQueue_multimap;
	/*----------------------------------------------------------------------------*/	

    public:
	sUndirectedGraph();
	sUndirectedGraph(bool directed);

	sUndirectedGraph(sInt_32 N_vertices, double edge_probability);
	
	sUndirectedGraph(sInt_32 x_size, sInt_32 y_size);
	sUndirectedGraph(sInt_32 x_size, sInt_32 y_size, double obstacle_prob);
	sUndirectedGraph(sInt_32 x_size, sInt_32 y_size, sInt_32 N_obstacles);
	sUndirectedGraph(sInt_32 base_cycle_size, sInt_32 ear_min_size, sInt_32 ear_max_size, sInt_32 graph_size);

	sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size);
	sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size, double obstacle_prob);
	sUndirectedGraph(bool directed, sInt_32 x_size, sInt_32 y_size, sInt_32 N_obstacles);
	sUndirectedGraph(bool directed, sInt_32 base_cycle_size, sInt_32 ear_min_size, sInt_32 ear_max_size, sInt_32 graph_size);
	
	sUndirectedGraph(const sUndirectedGraph &undirected_graph);
	sUndirectedGraph(const sUndirectedGraph &undirected_graph, bool opposite);
			 
	const sUndirectedGraph& operator=(const sUndirectedGraph &undirected_graph);
	~sUndirectedGraph();
	/*----------------------------------------------------------------------------*/

	void generate_Star(sInt_32 N_vertices);
	void generate_Clique(sInt_32 N_vertices);
	void generate_Path(sInt_32 N_vertices);	
	/*----------------------------------------------------------------------------*/

	void initialize_InverseMatrix(void);
	void set_Capacities(sInt_32 capacity);
	/*----------------------------------------------------------------------------*/	
	
	void add_Vertex(void);
	void add_Vertex(sInt_32 id, sInt_32 capacity);
	void add_Vertices(sInt_32 Vertex_cnt = 1);

	sInt_32 get_VertexCount(void) const;
	sVertex* get_Vertex(sInt_32 id);
	const sVertex* get_Vertex(sInt_32 id) const;

	bool is_Grid(void) const;
	sInt_32 get_GridHeight(void) const;
	sInt_32 get_GridWidth(void) const;
	sInt_32 get_GridCell(sInt_32 x, sInt_32 y) const;

	sInt_32 calc_GridRow(sInt_32 vertex_id) const;
	sInt_32 calc_GridColumn(sInt_32 vertex_id) const;
	sInt_32 calc_GridVertexID(sInt_32 grid_row, sInt_32 grid_column) const;
	sInt_32 calc_GridNeighborVertexID(sInt_32 vertex_id, sInt_32 delta_row, sInt_32 delta_column) const;
	sInt_32 get_GridNeighborVertexID(sInt_32 vertex_id, sInt_32 delta_row, sInt_32 delta_column) const;	

	void add_Arrow(sInt_32 u_id, sInt_32 v_id);
	void add_Edge(sInt_32 u_id, sInt_32 v_id);
	void add_RandomArrow(sInt_32 u_id, sInt_32 v_id);
	sInt_32 get_EdgeCount(void) const;

	bool is_Adjacent(sInt_32 u_id, sInt_32 v_id) const;
	bool is_LinkedTo(sInt_32 u_id, sInt_32 v_id) const;

	sInt_32 calc_ShortestPath(sInt_32 u_id, sInt_32 v_id) const;
	void calc_SingleSourceShortestPaths(sInt_32 s_id, Distances_vector &Distances) const;
	
	void calc_SingleSourceShortestPathsBreadth(sInt_32 s_id);
	void calc_SingleSourceShortestPathsBreadth(sInt_32 s_id, Distances_vector &Distances) const;

	void calc_SingleSourceShortestPathsBreadth_opposite(sInt_32 s_id);	
	void calc_SingleSourceShortestPathsBreadth_opposite(sInt_32 s_id, Distances_vector &Distances) const;
	
	void find_ShortestPathBreadth(sInt_32 source_id, sInt_32 dest_id, VertexIDs_vector &shortest_Path);

	void calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances);
	void calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void calc_SourceGoalShortestPaths(Distances_2d_vector &source_Distances, Distances_2d_vector &goal_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void collect_EquidistantVertices(sInt_32 s_id, sInt_32 distance, VertexIDs_vector &equidistant_IDs);

	void calc_AllPairsShortestPaths(void);
	void calc_AllPairsShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void calc_SourceGoalShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	const Distances_2d_vector& get_AllPairsShortestPaths(void) const;

	const Distances_2d_vector& get_SourceShortestPaths(void) const;
	const Distances_2d_vector& get_GoalShortestPaths(void) const;

	void find_ShortestPath(sInt_32 u_id, sInt_32 v_id, VertexIDs_list &path);
	void find_ShortestPathBFS(sInt_32 u_id, sInt_32 v_id);
	/*----------------------------------------------------------------------------*/

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Screen_vertices(const sString &indent = "") const;

	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_vertices(FILE *fw, const sString &indent = "") const;
	/*----------------------------------------------------------------------------*/

	virtual sResult to_File_cpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_cpf(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_ccpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_ccpf(FILE *fw, const sString &indent = "") const;	

	virtual sResult to_File_mpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mpf(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_cmpf(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_cmpf(FILE *fw, const sString &indent = "") const;	

	virtual sResult from_File_cpf(const sString &filename);
	virtual sResult from_Stream_cpf(FILE *fr);

	virtual sResult from_File_ccpf(const sString &filename);
	virtual sResult from_Stream_ccpf(FILE *fr);	

	virtual sResult from_File_mpf(const sString &filename);
	virtual sResult from_Stream_mpf(FILE *fr);

	virtual sResult from_File_cmpf(const sString &filename);
	virtual sResult from_Stream_cmpf(FILE *fr);		

	virtual sResult from_File_map(const sString &filename);
	virtual sResult from_Stream_map(FILE *fr);

	virtual sResult from_File_bgu(const sString &filename);
	virtual sResult from_Stream_bgu(FILE *fr);

	virtual sResult to_File_usc(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_usc(FILE *fw, const sString &indent = "") const;			
	
	virtual sResult from_File_usc(const sString &filename);
	virtual sResult from_Stream_usc(FILE *fr);      

	virtual sResult from_File_lusc(const sString &filename);
	virtual sResult from_Stream_lusc(FILE *fr);	

	virtual sResult to_File_dibox(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_dibox(FILE *fw, const sString &indent = "") const;	

	virtual sResult from_File_dibox(const sString &filename);
	virtual sResult from_Stream_dibox(FILE *fr);

	virtual sResult to_File_xml(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_xml(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_movi(const sString &filename);
	virtual sResult from_Stream_movi(FILE *fr);		
	/*----------------------------------------------------------------------------*/
	
	virtual sResult to_File_mapR(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mapR(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_mapR(const sString &filename);
	virtual sResult from_Stream_mapR(FILE *fr);		
	/*----------------------------------------------------------------------------*/	
		
    public:
	bool m_directed;
	
	sInt_32 m_Edge_cnt;
	sInt_32 m_x_size;
	sInt_32 m_y_size;
	
	sInt_32 *m_Matrix;
	Coordinates_vector m_inverse_Matrix;

	Vertices_vector m_Vertices;
	Edges_list m_Edges;

	bool m_all_pairs_distances_calculated;
	Distances_2d_vector m_all_pairs_Distances;

	bool m_source_goal_distances_calculated;
	Distances_2d_vector m_source_Distances;
	Distances_2d_vector m_goal_Distances;

	VertexIDs_vector m_Queue;
	Distances_vector m_Distances;
    }; 


/*----------------------------------------------------------------------------*/
// sVectorGraph

    class sVectorGraph
    {
    public:
	typedef std::vector<sInt_32> Connections_vector;
	typedef std::vector<Connections_vector> Connections_2d_vector;

    public:
	sVectorGraph();
	sVectorGraph(const sUndirectedGraph &undirected_graph);
	~sVectorGraph();

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:	
	Connections_2d_vector m_Connections;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __GRAPH_H__ */
