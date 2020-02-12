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
/* mapR.h / 1-224_leibniz                                                     */
/*----------------------------------------------------------------------------*/
//
// Semi-continuous maps for semi-continuous version of MAPF (MAPF-R).
//
/*----------------------------------------------------------------------------*/


#ifndef __MAP_R_H__
#define __MAP_R_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>

#include "result.h"

#include "common/types.h"
#include "core/graph.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{
   
    const sDouble s_RADIANT_RADIUS_2K_NEIHBORHOOD[] = { -1.0,
							-1.0,
							1.1,
							1.5,
							2.3,
							3.7,
							6.45
    };
    

/*----------------------------------------------------------------------------*/
// s2DMap

    class s2DMap
    {
    public:
	struct Location
	{
	    Location()
	    { /* nothing */}
	    Location(sInt_32 id, sDouble x, sDouble y)
	    : m_id(id)
	    , m_x(x)
	    , m_y(y) { /* nothing */ }
	    
	    sInt_32 m_id;
	    sDouble m_x, m_y;
	};

	typedef std::vector<sInt_32> LocationIDs_vector;
	typedef std::vector<Location> Locations_vector;

	typedef std::vector<sDouble> Distances_vector;
	typedef std::vector<Distances_vector> Distances_2d_vector;

	typedef std::multimap<sDouble, sInt_32, std::less<sDouble> > DistanceLocationIDs_mmap;

    public:
	s2DMap();
	s2DMap(sInt_32 N_locations);

	void add_Location(sDouble x, sDouble y);
	void add_Location(sInt_32 id, sDouble x, sDouble y);

	void populate_Network(void);
	void populate_Network(sDouble max_distance);

	void populate_NetworkCircular(sDouble max_distance);
	void populate_NetworkRadiant(sDouble max_distance);	
	/*----------------------------------------------------------------------------*/

	sDouble calc_StraightDistance(sInt_32 source_id, sInt_32 target_id) const;
	
	void calc_SingleSourceStraightDistances(sInt_32 s_id);
	void calc_SingleSourceStraightDistances(sInt_32 s_id, Distances_vector &straight_Distances) const;

	void calc_SingleSourceStraightNeighborDistances(sInt_32 s_id);
	void calc_SingleSourceStraightNeighborDistances(sInt_32 s_id, Distances_vector &straight_Distances) const;	

	void calc_AllPairsStraightDistances(void);
	void calc_AllPairsStraightDistances(Distances_2d_vector &straight_Distances) const;

	void calc_NetworkPairsStraightDistances(void);
	void calc_NetworkPairsStraightDistances(Distances_2d_vector &straight_Distances) const;	

	void calc_SelectedPairsStraightDistances(const LocationIDs_vector &selection_IDs);
	void calc_SelectedPairsStraightDistances(const LocationIDs_vector &selection_IDs, Distances_2d_vector &straight_Distances) const;

	void calc_SourceGoalStraightDistances(const LocationIDs_vector &source_IDs, const LocationIDs_vector &goal_IDs);
	void calc_SourceGoalStraightDistances(const LocationIDs_vector &source_IDs, const LocationIDs_vector &goal_IDs, Distances_2d_vector &straight_Distances) const;
	/*----------------------------------------------------------------------------*/	

	void calc_SingleSourceShortestDistances(sInt_32 s_id);
	void calc_SingleSourceShortestDistances(sInt_32 s_id, Distances_vector &shortest_Distances) const;

	void calc_SingleSourceShortestNeighborDistances(sInt_32 s_id);
	void calc_SingleSourceShortestNeighborDistances(sInt_32 s_id, Distances_vector &shortest_Distances) const;	

	void calc_AllPairsShortestDistances(void);
	void calc_AllPairsShortestDistances(Distances_2d_vector &shortest_Distances) const;

	void calc_NetworkPairsShortestDistances(void);
	void calc_NetworkPairsShortestDistances(Distances_2d_vector &shortest_Distances) const;	

	void calc_SelectedPairsShortestDistances(const LocationIDs_vector &selection_IDs);	
	void calc_SelectedPairsShortestDistances(const LocationIDs_vector &selection_IDs, Distances_2d_vector &shortest_Distances) const;

	void calc_SourceGoalShortestDistances(const LocationIDs_vector &source_IDs, const LocationIDs_vector &goal_IDs);
	void calc_SourceGoalShortestDistances(const LocationIDs_vector &source_IDs, const LocationIDs_vector &goal_IDs, Distances_2d_vector &shortest_Distances) const;	
	/*----------------------------------------------------------------------------*/	

	sDouble calc_PointDistance(sInt_32 point_1_id, sInt_32 point_2_id) const;
	sDouble calc_PointDistance(sDouble x1, sDouble y1, sDouble x2, sDouble y2) const;
	
	sDouble calc_LineDistance(sInt_32 line_1_u_id,
				  sInt_32 line_1_v_id,
				  sInt_32 line_2_u_id,
				  sInt_32 line_2_v_id) const;
	
	sDouble calc_LineDistance(sDouble px1, sDouble py1, sDouble px2, sDouble py2,
				  sDouble qx1, sDouble qy1, sDouble qx2, sDouble qy2) const;

	sDouble calc_PointDistance(sInt_32 line_1_u_id,
				   sInt_32 line_1_v_id,
				   sInt_32 point_id) const;
	
	sDouble calc_PointDistance(sDouble px1, sDouble py1, sDouble px2, sDouble py2,
				   sDouble x, sDouble y) const;	
	/*----------------------------------------------------------------------------*/

	sInt_32 find_NearestLocation(sInt_32 location_id) const;	
	sInt_32 find_NearestLocation(sInt_32 location_id, sDouble &distance) const;
	/*----------------------------------------------------------------------------*/

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_mapR(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_mapR(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_mapR(const sString &filename);
	virtual sResult from_Stream_mapR(FILE *fr);

	virtual sResult from_File_map(const sString &filename);
	virtual sResult from_Stream_map(FILE *fr);

	virtual sResult from_File_movi(const sString &filename);
	virtual sResult from_Stream_movi(FILE *fr);	

	virtual sResult to_File_xml(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_xml(FILE *fw, const sString &indent = "") const;	

	virtual sResult from_File_xml(const sString &filename);
	virtual sResult from_Stream_xml(FILE *fr);
	/*----------------------------------------------------------------------------*/	
	
    public:
	Locations_vector m_Locations;
	
	Distances_2d_vector m_straight_Distances;
	Distances_2d_vector m_shortest_Distances;	

	sUndirectedGraph m_Network;
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __MAP_R_H__ */
