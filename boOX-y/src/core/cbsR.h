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
/* cbsR.h / 1-224_leibniz                                                     */
/*----------------------------------------------------------------------------*/
//
// Conflict based search for a semi-continuous version of MAPF.
//
/*----------------------------------------------------------------------------*/


#ifndef __CBSR_H__
#define __CBSR_H__

#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "result.h"

#include "common/types.h"
#include "core/mapR.h"
#include "core/kruhoR.h"


using namespace std;


/*----------------------------------------------------------------------------*/

namespace boOX
{



    
/*----------------------------------------------------------------------------*/
// sRealCBS

    class sRealCBSBase
    {
    public:
	struct Event
	{
	    Event() { /* nothing */ }
	    Event(sInt_32 from_loc_id,
		  sInt_32 to_loc_id,
		  sDouble start_time,
		  sDouble finish_time)
	    : m_from_loc_id(from_loc_id)
	    , m_to_loc_id(to_loc_id)	    
	    , m_start_time(start_time)
	    , m_finish_time(finish_time)		
	    { /* nothing */ }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d --> %d [%.3f, %.3f]\n", indent.c_str(), m_from_loc_id, m_to_loc_id, m_start_time, m_finish_time);
	    }
	    
	    sInt_32 m_from_loc_id, m_to_loc_id;
	    sDouble m_start_time, m_finish_time;
	};

	struct Interval
	{
	    Interval() { /* nothing */ }
	    Interval(sDouble lower, sDouble upper)
	    : m_lower(lower)
	    , m_upper(upper)
	    { /* nothing */ }

	    /*--------------------------------*/
	    bool compare_nonoverlapping(const Interval &interval) const
	    {
		return (m_upper <= interval.m_lower);
	    }
	    
	    bool compare_lexicographic(const Interval &interval) const
	    {
		return (m_lower < interval.m_lower || (m_lower == interval.m_lower && m_upper < interval.m_upper));
	    }

	    bool compare_colexicographic(const Interval &interval) const
	    {
		return (m_upper < interval.m_upper || (m_upper == interval.m_upper && m_lower < interval.m_lower));
	    }	    
	   
	    bool compare_lower_less(const Interval &interval) const
	    {
		return (m_lower < interval.m_lower);		
	    }

	    bool compare_upper_less(const Interval &interval) const
	    {
		return (m_upper < interval.m_upper);
	    }

	    bool compare_lower_lteq(const Interval &interval) const
	    {
		return (m_lower <= interval.m_lower);		
	    }

	    bool compare_upper_lteq(const Interval &interval) const
	    {
		return (m_upper <= interval.m_upper);		
	    }
	    /*--------------------------------*/	    

	    bool operator<(const Interval &interval) const
	    {
		return compare_nonoverlapping(interval);
	    }	    

	    bool operator==(const Interval &interval) const
	    {
		return (m_lower == interval.m_lower && m_upper == interval.m_upper);		
	    }

	    bool overlaps(const Interval &interval) const
	    {
		return (m_upper > interval.m_lower && m_lower < interval.m_upper);
	    }

	    bool empty(void) const
	    {		
		return (m_lower > m_upper);
	    }

	    sDouble size(void) const
	    {		
		return (m_upper - m_lower);
	    }	    

	    Interval intersect(const Interval &interval) const
	    {
		return Interval(sMAX(m_lower, interval.m_lower), sMIN(m_upper, interval.m_upper));
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s[%.3f,%.3f)\n", indent.c_str(), m_lower, m_upper);
	    }	    
	    
	    sDouble m_lower, m_upper;

	    struct CompareLowerLess : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_lower_less(interval_B);
		}
	    };

	    struct CompareUpperLess : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_upper_less(interval_B);
		}
	    };

	    struct CompareLowerLteq : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_lower_lteq(interval_B);
		}
	    };

	    struct CompareUpperLteq : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_upper_lteq(interval_B);
		}
	    };

	    struct CompareLexicographic : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_lexicographic(interval_B);
		}
	    };

	    struct CompareColexicographic : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_colexicographic(interval_B);
		}
	    };	    

	    struct CompareNonoverlapping : public std::binary_function<Interval, Interval, bool>
	    {
		bool operator()(const Interval& interval_A, const Interval& interval_B) const
		{
		    return interval_A.compare_nonoverlapping(interval_B);
		}
	    };	    
	};
	
	struct Traversal
	{
	    Traversal() { /* nothing */ }
	    Traversal(sInt_32 kruhobot_id, sInt_32 u_loc_id, sInt_32 v_loc_id, const Interval &interval)
	    : m_kruhobot_id(kruhobot_id)
	    , m_u_loc_id(u_loc_id)
	    , m_v_loc_id(v_loc_id)
	    , m_interval(interval) { /* nothing */ }
	    
	    sInt_32 m_kruhobot_id;
	    sInt_32 m_u_loc_id, m_v_loc_id;
	    Interval m_interval;

	    bool operator==(const Traversal &traversal) const
	    {
		return (   m_kruhobot_id == traversal.m_kruhobot_id
			&& m_u_loc_id == traversal.m_u_loc_id
			&& m_v_loc_id == traversal.m_v_loc_id
			&& m_interval == traversal.m_interval);
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d: %d <-> %d [%.3f,%.3f]\n", indent.c_str(), m_kruhobot_id, m_u_loc_id, m_v_loc_id, m_interval.m_lower, m_interval.m_upper);
	    }	    
	};

	typedef std::vector<Event> Schedule_vector;
	typedef std::vector<Schedule_vector> KruhobotSchedules_vector;

	struct KruhobotCollision
	{
	    KruhobotCollision() { /* nothig */ }
	
	    KruhobotCollision(const Traversal &traversal_A, const Traversal &traversal_B)
	    : m_importance(-1.0)
	    , m_traversal_A(traversal_A)
	    , m_traversal_B(traversal_B) { /* nothing */ }    
	    
	    KruhobotCollision(sDouble importance, const Traversal &traversal_A, const Traversal &traversal_B)
	    : m_importance(importance)
	    , m_traversal_A(traversal_A)
	    , m_traversal_B(traversal_B) { /* nothing */ }

	    bool operator<(const KruhobotCollision &kruhobot_collision) const
	    {
		Interval intersection_1 = m_traversal_A.m_interval.intersect(m_traversal_B.m_interval);
		Interval intersection_2 = kruhobot_collision.m_traversal_A.m_interval.intersect(kruhobot_collision.m_traversal_B.m_interval);

//		return (intersection_1 < intersection_2);
//		return (m_importance < kruhobot_collision.m_importance);		
		return intersection_1.compare_lexicographic(intersection_2);
	    }

	    bool operator==(const KruhobotCollision &kruhobot_collision) const
	    {
		return (   (   m_traversal_A == kruhobot_collision.m_traversal_A
			    && m_traversal_B == kruhobot_collision.m_traversal_B)
			|| (   m_traversal_B == kruhobot_collision.m_traversal_A
			    && m_traversal_A == kruhobot_collision.m_traversal_B));
	    }

	    sDouble m_importance;

	    Traversal m_traversal_A;
	    Traversal m_traversal_B;

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%sKruhobot collision [\n", indent.c_str());
		m_traversal_A.to_Stream(fw, indent + s_INDENT);
		m_traversal_B.to_Stream(fw, indent + s_INDENT);		
		fprintf(fw, "%s]\n", indent.c_str());		
	    }	    
	};

	/*
	typedef std::unordered_map<sInt_32, sInot_32> Occupation_umap;
	typedef std::vector<Occupation_umap> Occupations_vector;
	*/
	typedef std::unordered_set<sInt_32> KruhobotIDs_uset;
	typedef std::unordered_map<sInt_32, sInt_32> KruhobotIDs_umap;	
	
	typedef std::unordered_map<sInt_32, KruhobotIDs_uset> Cooccupation_umap;
	typedef std::vector<Cooccupation_umap> Cooccupations_vector;

	typedef std::unordered_map<sInt_32, KruhobotIDs_umap> Cooccupation__umap;
	typedef std::vector<Cooccupation_umap> Cooccupations__vector;

	typedef std::multiset<KruhobotCollision, std::less<KruhobotCollision> > KruhobotCollisions_mset;

	struct LocationConflict
	{
	    LocationConflict() { /* nothing */ }
	    LocationConflict(sInt_32 location_id, const Interval &interval, bool infinity = false)
	    : m_infinity(infinity)
	    , m_location_id(location_id)
	    , m_interval(interval) { /* nothing */ }

	    LocationConflict(sInt_32 conflict_id, sInt_32 location_id, const Interval &interval, bool infinity = false)
	    : m_conflict_id(conflict_id)
	    , m_infinity(infinity)
	    , m_location_id(location_id)
	    , m_interval(interval) { /* nothing */ }	    

	    bool operator<(const LocationConflict &location_conflict) const
	    {
		return (m_interval < location_conflict.m_interval);
	    }

	    bool operator==(const LocationConflict &location_conflict) const
	    {
		return (m_interval == location_conflict.m_interval);
	    }

	    bool overlaps(const Interval &interval) const
	    {
		return m_interval.overlaps(interval);
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		if (m_infinity)
		{
		    fprintf(fw, "%s%d:%d+: [%.3f,%.3f]\n", indent.c_str(), m_conflict_id, m_location_id, m_interval.m_lower, m_interval.m_upper);
		}
		else
		{
		    fprintf(fw, "%s%d:%d: [%.3f,%.3f]\n", indent.c_str(), m_conflict_id, m_location_id, m_interval.m_lower, m_interval.m_upper);
		}
	    }
	    
	    sInt_32 m_conflict_id;
	    
	    bool m_infinity;
	    sInt_32 m_location_id;
	    Interval m_interval;
	};

	struct LinearConflict
	{
	    LinearConflict() { /* nothing */ }
	    LinearConflict(sInt_32 line_u_id, sInt_32 line_v_id, const Interval &interval)
	    : m_line_u_id(line_u_id)
	    , m_line_v_id(line_v_id)	    
	    , m_interval(interval) { /* nothing */ }

	    LinearConflict(sInt_32 conflict_id, sInt_32 line_u_id, sInt_32 line_v_id, const Interval &interval)
	    : m_conflict_id(conflict_id)
	    , m_line_u_id(line_u_id)
	    , m_line_v_id(line_v_id)	    
	    , m_interval(interval) { /* nothing */ }	    

	    bool operator<(const LinearConflict &linear_conflict) const
	    {
		return (m_interval < linear_conflict.m_interval);
	    }

	    bool operator==(const LinearConflict &linear_conflict) const
	    {
		return (m_interval == linear_conflict.m_interval);
	    }

	    bool overlaps(const Interval &interval) const
	    {
		return m_interval.overlaps(interval);
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d:%d<->%d: [%.3f,%.3f]\n", indent.c_str(), m_conflict_id, m_line_u_id, m_line_v_id, m_interval.m_lower, m_interval.m_upper);
	    }
	    
	    sInt_32 m_conflict_id;	    

	    sInt_32 m_line_u_id;
	    sInt_32 m_line_v_id;	    
	    Interval m_interval;
	};

	struct Occupation
	{
	    Occupation() { /* nothing */ }
	    Occupation(sInt_32 kruhobot_id, const Interval &interval)
	    : m_kruhobot_id(kruhobot_id)
	    , m_interval(interval) { /* nothing */ }
	    
	    sInt_32 m_kruhobot_id;
	    Interval m_interval;	    
	};

	struct Uline
	{
	    Uline() { /* nothing */ }
	    Uline(sInt_32 u_id, sInt_32 v_id)
	    {
		if (u_id < v_id)
		{
		    m_lower_id = u_id;
		    m_upper_id = v_id;
		}
		else
		{
		    m_lower_id = v_id;
		    m_upper_id = u_id;		    
		}
	    }

	    bool operator<(const Uline &uline) const
	    {
		return (m_lower_id < uline.m_lower_id || (m_lower_id ==  uline.m_lower_id && m_upper_id < uline.m_upper_id));
	    }

	    bool operator==(const Uline &uline) const
	    {
		return (m_lower_id ==  uline.m_lower_id && m_upper_id == uline.m_upper_id);
	    }

	    std::pair<sInt_32, sInt_32> get_Pair(void) const
	    {
		return std::pair<sInt_32, sInt_32>(m_lower_id, m_upper_id);
	    }

	    sInt_32 m_lower_id;
	    sInt_32 m_upper_id;
	};

	
	struct Line
	{
	    Line() { /* nothing */ }
	    Line(sInt_32 u_id, sInt_32 v_id)
	    {
		m_u_id = u_id;
		m_v_id = v_id;
	    }

	    bool operator<(const Line &line) const
	    {
		return (m_u_id < line.m_u_id || (m_u_id ==  line.m_u_id && m_v_id < line.m_v_id));
	    }

	    bool operator==(const Line &line) const
	    {
		return (m_u_id ==  line.m_u_id && m_v_id == line.m_v_id);
	    }

	    std::pair<sInt_32, sInt_32> get_Pair(void) const
	    {
		return std::pair<sInt_32, sInt_32>(m_u_id, m_v_id);
	    }

	    sInt_32 m_u_id;
	    sInt_32 m_v_id;
	};
	

	typedef std::map<Interval, LocationConflict, Interval::CompareNonoverlapping> LocationConflicts_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareNonoverlapping> LinearConflicts_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareNonoverlapping> UlinearConflicts_map;	

	typedef std::map<Interval, LocationConflict, Interval::CompareLexicographic > LocationConflicts_lexicographic_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareLexicographic > LinearConflicts_lexicographic_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareLexicographic > UlinearConflicts_lexicographic_map;	
	
	typedef std::map<Interval, LocationConflict, Interval::CompareLowerLess> LocationConflicts_lower_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareLowerLess> LinearConflicts_lower_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareLowerLess> UlinearConflicts_lower_map;	

	typedef std::map<Interval, LocationConflict, Interval::CompareUpperLess> LocationConflicts_upper_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareUpperLess> LinearConflicts_upper_map;
	typedef std::map<Interval, LinearConflict, Interval::CompareUpperLess> UlinearConflicts_upper_map;	

	typedef std::map<Interval, LocationConflicts_map::iterator, Interval::CompareLowerLess> LocationConflicts_lowerProjection_map;
	typedef std::map<Interval, LinearConflicts_map::iterator, Interval::CompareLowerLess> LinearConflicts_lowerProjection_map;
	typedef std::map<Interval, UlinearConflicts_map::iterator, Interval::CompareLowerLess> UlinearConflicts_lowerProjection_map;

	typedef std::map<Interval, LocationConflicts_map::iterator, Interval::CompareUpperLess> LocationConflicts_upperProjection_map;
	typedef std::map<Interval, LinearConflicts_map::iterator, Interval::CompareUpperLess> LinearConflicts_upperProjection_map;
	typedef std::map<Interval, UlinearConflicts_map::iterator, Interval::CompareUpperLess> UlinearConflicts_upperProjection_map;	

	struct LocationConflicts_projection_map
	{
	    LocationConflicts_projection_map() { }
	    
	    void insert(const Interval &interval, const LocationConflict &location_conflict)
	    {
		LocationConflicts_lexicographic_map::iterator conflict_iter = m_Conflicts.insert(LocationConflicts_lexicographic_map::value_type(interval, location_conflict)).first;

		m_lower_Projection.insert(LocationConflicts_lowerProjection_map::value_type(interval, conflict_iter));
		m_upper_Projection.insert(LocationConflicts_upperProjection_map::value_type(interval, conflict_iter));		
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		
		fprintf(fw, "%sLocation conflicts [projection map] {\n", indent.c_str());

		if (!m_Conflicts.empty())
		{
		    fprintf(fw, "%s%slex [\n", indent.c_str(), s_INDENT.c_str());

		    for (LocationConflicts_lexicographic_map::const_iterator lex_conflict = m_Conflicts.begin(); lex_conflict != m_Conflicts.end(); ++lex_conflict)
		    {
			lex_conflict->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%slow [\n", indent.c_str(), s_INDENT.c_str());

		    for (LocationConflicts_lowerProjection_map::const_iterator low_conflict = m_lower_Projection.begin(); low_conflict != m_lower_Projection.end(); ++low_conflict)
		    {
			low_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%sup [\n", indent.c_str(), s_INDENT.c_str());

		    for (LocationConflicts_upperProjection_map::const_iterator up_conflict = m_upper_Projection.begin(); up_conflict != m_upper_Projection.end(); ++up_conflict)
		    {
			up_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());		    		    		    
		}
		fprintf(fw, "%s}\n", indent.c_str());		
	    }	    	    	    

	    LocationConflicts_lexicographic_map m_Conflicts;

	    LocationConflicts_lowerProjection_map m_lower_Projection;
	    LocationConflicts_upperProjection_map m_upper_Projection;	    
	};

	struct LinearConflicts_projection_map
	{
	    LinearConflicts_projection_map() { }
	    
	    void insert(const Interval &interval, const LinearConflict &linear_conflict)
	    {
		LinearConflicts_lexicographic_map::iterator conflict_iter = m_Conflicts.insert(LinearConflicts_lexicographic_map::value_type(interval, linear_conflict)).first;

		m_lower_Projection.insert(LinearConflicts_lowerProjection_map::value_type(interval, conflict_iter));
		m_upper_Projection.insert(LinearConflicts_upperProjection_map::value_type(interval, conflict_iter));		
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		
		fprintf(fw, "%sLinear conflicts [projection map] {\n", indent.c_str());

		if (!m_Conflicts.empty())
		{
		    fprintf(fw, "%s%slex [\n", indent.c_str(), s_INDENT.c_str());

		    for (LinearConflicts_lexicographic_map::const_iterator lex_conflict = m_Conflicts.begin(); lex_conflict != m_Conflicts.end(); ++lex_conflict)
		    {
			lex_conflict->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%slow [\n", indent.c_str(), s_INDENT.c_str());

		    for (LinearConflicts_lowerProjection_map::const_iterator low_conflict = m_lower_Projection.begin(); low_conflict != m_lower_Projection.end(); ++low_conflict)
		    {
			low_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%slow [\n", indent.c_str(), s_INDENT.c_str());

		    for (LinearConflicts_upperProjection_map::const_iterator up_conflict = m_upper_Projection.begin(); up_conflict != m_upper_Projection.end(); ++up_conflict)
		    {
			up_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());		    		    		    
		}
		fprintf(fw, "%s}\n", indent.c_str());		
	    }	    	    	    	    

	    LinearConflicts_lexicographic_map m_Conflicts;

	    LinearConflicts_lowerProjection_map m_lower_Projection;
	    LinearConflicts_upperProjection_map m_upper_Projection;	    
	};
	
	struct UlinearConflicts_projection_map
	{
	    UlinearConflicts_projection_map() { }
	    
	    void insert(const Interval &interval, const LinearConflict &linear_conflict)
	    {
		UlinearConflicts_lexicographic_map::iterator conflict_iter = m_Conflicts.insert(UlinearConflicts_lexicographic_map::value_type(interval, linear_conflict)).first;

		m_lower_Projection.insert(UlinearConflicts_lowerProjection_map::value_type(interval, conflict_iter));
		m_upper_Projection.insert(UlinearConflicts_upperProjection_map::value_type(interval, conflict_iter));		
	    }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		
		fprintf(fw, "%sLinear conflicts [projection map] {\n", indent.c_str());

		if (!m_Conflicts.empty())
		{
		    fprintf(fw, "%s%slex [\n", indent.c_str(), s_INDENT.c_str());

		    for (UlinearConflicts_lexicographic_map::const_iterator lex_conflict = m_Conflicts.begin(); lex_conflict != m_Conflicts.end(); ++lex_conflict)
		    {
			lex_conflict->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%slow [\n", indent.c_str(), s_INDENT.c_str());

		    for (UlinearConflicts_lowerProjection_map::const_iterator low_conflict = m_lower_Projection.begin(); low_conflict != m_lower_Projection.end(); ++low_conflict)
		    {
			low_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());

		    fprintf(fw, "%s%slow [\n", indent.c_str(), s_INDENT.c_str());

		    for (UlinearConflicts_upperProjection_map::const_iterator up_conflict = m_upper_Projection.begin(); up_conflict != m_upper_Projection.end(); ++up_conflict)
		    {
			up_conflict->second->second.to_Screen(indent + s2_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s_INDENT.c_str());		    		    		    
		}
		fprintf(fw, "%s}\n", indent.c_str());		
	    }	    	    	    	    

	    UlinearConflicts_lexicographic_map m_Conflicts;

	    UlinearConflicts_lowerProjection_map m_lower_Projection;
	    UlinearConflicts_upperProjection_map m_upper_Projection;	    
	};	
	
	typedef std::unordered_map<sInt_32, LocationConflicts_map> LocationConflicts__umap;
	typedef std::map<Uline, LinearConflicts_map, std::less<Uline> > LinearConflicts__map;
	typedef std::map<Uline, UlinearConflicts_map, std::less<Uline> > UlinearConflicts__map;

	typedef std::vector<LocationConflicts__umap> KruhobotLocationConflicts_vector;
	typedef std::vector<LinearConflicts__map> KruhobotLinearConflicts_vector;
	typedef std::vector<UlinearConflicts__map> KruhobotUlinearConflicts_vector;
	
	typedef std::unordered_map<sInt_32, LocationConflicts_projection_map> LocationConflicts_projection__umap;
	typedef std::map<Uline, LinearConflicts_projection_map, std::less<Uline> > LinearConflicts_projection__map;
	typedef std::map<Uline, UlinearConflicts_projection_map, std::less<Uline> > UlinearConflicts_projection__map;	

	typedef std::vector<LocationConflicts_projection__umap> KruhobotLocationConflicts_projection_vector;
	typedef std::vector<LinearConflicts_projection__map> KruhobotLinearConflicts_projection_vector;
	typedef std::vector<UlinearConflicts_projection__map> KruhobotUlinearConflicts_projection_vector;	

	typedef std::unordered_map<sInt_32, LocationConflicts_lexicographic_map> LocationConflicts_lexicographic__umap;
	typedef std::map<Uline, LinearConflicts_lexicographic_map, std::less<Uline> > LinearConflicts_lexicographic__map;
	typedef std::map<Uline, UlinearConflicts_lexicographic_map, std::less<Uline> > UlinearConflicts_lexicographic__map;	

	typedef std::vector<LocationConflicts_lexicographic__umap> KruhobotLocationConflicts_lexicographic_vector;
	typedef std::vector<LinearConflicts_lexicographic__map> KruhobotLinearConflicts_lexicographic_vector;
	typedef std::vector<UlinearConflicts_lexicographic__map> KruhobotUlinearConflicts_lexicographic_vector;	

	typedef std::unordered_map<sInt_32, LocationConflicts_lower_map> LocationConflicts_lower__umap;
	typedef std::map<Uline, LinearConflicts_lower_map, std::less<Uline> > LinearConflicts_lower__map;
	typedef std::map<Uline, UlinearConflicts_lower_map, std::less<Uline> > UlinearConflicts_lower__map;	

	typedef std::vector<LocationConflicts_lower__umap> KruhobotLocationConflicts_lower_vector;
	typedef std::vector<LinearConflicts_lower__map> KruhobotLinearConflicts_lower_vector;
	typedef std::vector<UlinearConflicts_lower__map> KruhobotUlinearConflicts_lower_vector;			

	typedef std::unordered_map<sInt_32, LocationConflicts_upper_map> LocationConflicts_upper__umap;
	typedef std::map<Uline, LinearConflicts_upper_map, std::less<Uline> > LinearConflicts_upper__map;
	typedef std::map<Uline, UlinearConflicts_upper_map, std::less<Uline> > UlinearConflicts_upper__map;	

	typedef std::vector<LocationConflicts_upper__umap> KruhobotLocationConflicts_upper_vector;
	typedef std::vector<LinearConflicts_upper__map> KruhobotLinearConflicts_upper_vector;
	typedef std::vector<UlinearConflicts_upper__map> KruhobotUlinearConflicts_upper_vector;		

	struct Transition
	{
	    Transition() { /* nothing */ }
	    Transition(sInt_32 trans_id, sDouble time, sDouble cost, sDouble makespan, sInt_32 location_id, sInt_32 prev_trans_id)
	    : m_trans_id(trans_id)
	    , m_time(time)
	    , m_cost(cost)
	    , m_makespan(makespan)
	    , m_location_id(location_id)
	    , m_prev_trans_id(prev_trans_id)
	    , m_corr_dec_id(-1) { /* nothing */ }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%s%d [%.3f {cost: %.3f, make:%.3f}] (%d <-- %d) @%d\n", indent.c_str(), m_location_id, m_time, m_cost, m_makespan, m_trans_id, m_prev_trans_id, m_corr_dec_id);
	    }	    	    

	    sInt_32 m_trans_id;
	    
	    sDouble m_time;
	    sDouble m_cost;
	    sDouble m_makespan;
	    sInt_32 m_location_id;
	    sInt_32 m_prev_trans_id;
	    sInt_32 m_corr_dec_id;
	};

	typedef std::vector<sInt_32> LocationIDs_vector;	
	typedef std::unordered_set<sInt_32> LocationIDs_uset;
	
	typedef std::map<sDouble, LocationIDs_uset> Transitions_map;
	typedef std::multimap<sDouble, Transition, std::less<sDouble> > Transitions_mmap;
	typedef std::vector<Transition> Transitions_vector;

	typedef std::map<Interval, KruhobotIDs_uset, Interval::CompareColexicographic> Cooccupations_map;
	typedef std::unordered_map<sInt_32, Cooccupations_map> LocationCooccupations_umap;
	typedef std::map<Uline, Cooccupations_map, std::less<Uline> > LinearCooccupations_map;
	typedef std::map<Uline, Cooccupations_map, std::less<Uline> > UlinearCooccupations_map;

	typedef std::map<Interval, KruhobotIDs_umap, Interval::CompareColexicographic> Cooccupations__map;
	typedef std::unordered_map<sInt_32, Cooccupations_map> LocationCooccupations__umap;
	typedef std::map<Uline, Cooccupations__map, std::less<Uline> > LinearCooccupaions__map;
	typedef std::map<Uline, Cooccupations__map, std::less<Uline> > UlinearCooccupations__map;		

	typedef std::vector<sInt_32> KruhobotAffections_vector;
	typedef std::pair<sInt_32, sInt_32> KruhobotAffection_pair;
	
    public:
	sRealCBSBase(sRealInstance *real_Instance);
	sRealCBSBase(sRealInstance *real_Instance, sDouble timeout);
        /*----------------------------------------------------------------------------*/

	sDouble analyze_NonconflictingSchedules(const sRealInstance            &real_Instance,
						const KruhobotSchedules_vector &kruhobot_Schedules,
						KruhobotCollisions_mset        &kruhobot_Collisions) const;

	sDouble analyze_NonconflictingSchedules_nonprioritized(const sRealInstance            &real_Instance,
							       const KruhobotSchedules_vector &kruhobot_Schedules,
							       KruhobotCollisions_mset        &kruhobot_Collisions) const;

	sDouble analyze_NonconflictingSchedules_exactNonprioritized(const sRealInstance            &real_Instance,
								    const KruhobotSchedules_vector &kruhobot_Schedules,
								    KruhobotCollisions_mset        &kruhobot_Collisions) const;	
        /*----------------------------------------------------------------------------*/

	KruhobotAffection_pair resolve_KruhobotCollision(const sRealInstance              &real_Instance,
							 const Traversal                  &kruhobot_traversal_A,
							 const Traversal                  &kruhobot_traversal_B,				       
							 KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
							 KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
							 sInt_32                          &last_conflict_id,
							 bool                              infinity = false) const;

	KruhobotAffection_pair resolve_KruhobotCollision(const sRealInstance                    &real_Instance,
							 const Traversal                        &kruhobot_traversal_A,
							 const Traversal                        &kruhobot_traversal_B,				       
							 KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
							 KruhobotLinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
							 sInt_32                                &last_conflict_id,
							 bool                                    infinity = false) const;
	
	KruhobotAffection_pair resolve_KruhobotCollision(const sRealInstance                    &real_Instance,
							 const Traversal                        &kruhobot_traversal_A,
							 const Traversal                        &kruhobot_traversal_B,				       
							 KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
							 KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
							 sInt_32                                &last_conflict_id,
							 bool                                    infinity = false) const;		
	
	KruhobotAffection_pair resolve_KruhobotCollision(const sRealInstance                            &real_Instance,
							 const Traversal                                &kruhobot_traversal_A,
							 const Traversal                                &kruhobot_traversal_B,				       
							 KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
							 KruhobotLinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
							 sInt_32                                        &last_conflict_id,
							 bool                                            infinity = false) const;

	template<class T1, class T2>
	KruhobotAffection_pair resolve_KruhobotCollision(const sRealInstance &real_Instance,
							 const Traversal     &kruhobot_traversal_A,
							 const Traversal     &kruhobot_traversal_B,
							 T1                  &kruhobot_location_Conflicts,
							 T2                  &kruhobot_linear_Conflicts,
							 sInt_32             &last_conflict_id,
							 bool                 infinity = false) const;	

	template<class T1, class T2>
	KruhobotAffection_pair resolve_KruhobotCollision_location_X_location(const sRealInstance   &real_Instance,
									     const Traversal       &kruhobot_traversal_A,
									     const Traversal       &kruhobot_traversal_B,
									     T1                    &kruhobot_location_Conflicts,
									     T2                    &kruhobot_linear_Conflicts,
									     sInt_32               &last_conflict_id,
									     bool                   infinity = false) const;	

	template<class T1, class T2>
	KruhobotAffection_pair resolve_KruhobotCollision_location_X_linear(const sRealInstance &real_Instance,
									   const Traversal     &kruhobot_traversal_A,
									   const Traversal     &kruhobot_traversal_B,
									   T1                  &kruhobot_location_Conflicts,
									   T2                  &kruhobot_linear_Conflicts,
									   sInt_32             &last_conflict_id,
									   bool                 infinity = false) const;

	template<class T1, class T2>
	KruhobotAffection_pair resolve_KruhobotCollision_linear_X_location(const sRealInstance &real_Instance,
									   const Traversal     &kruhobot_traversal_A_linear,
									   const Traversal     &kruhobot_traversal_B_location,
									   T1                  &kruhobot_location_Conflicts,
									   T2                  &kruhobot_linear_Conflicts,
									   sInt_32             &last_conflict_id,
									   bool                 infinity) const;		

	template<class T1, class T2>
	KruhobotAffection_pair resolve_KruhobotCollision_linear_X_linear(const sRealInstance &real_Instance,
									 const Traversal     &kruhobot_traversal_A,
									 const Traversal     &kruhobot_traversal_B,
									 T1                  &kruhobot_location_Conflicts,
									 T2                  &kruhobot_linear_Conflicts,
									 sInt_32             &last_conflict_id,
									 bool                 infinity = false) const;			
        /*----------------------------------------------------------------------------*/		

	void introduce_KruhobotConflict(const Traversal                  &kruhobot_traversal,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					bool                              infinity = false) const;

	void introduce_KruhobotConflict(const Traversal                                &kruhobot_traversal,
					KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
					bool                                            infinity = false) const;
	
	void introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
					KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
					bool                                    infinity = false) const;

	void introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					bool                                    infinity = false) const;
        /*----------------------------------------------------------------------------*/

	void introduce_KruhobotConflict(const Traversal                  &kruhobot_traversal,
					KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					sInt_32                          &last_conflict_id,
					bool                              infinity = false) const;

	void introduce_KruhobotConflict(const Traversal                                &kruhobot_traversal,
					KruhobotLocationConflicts_lexicographic_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_lexicographic_vector   &kruhobot_linear_Conflicts,
					sInt_32                                        &last_conflict_id,
					bool                                            infinity = false) const;
	
	void introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
					KruhobotLocationConflicts_lower_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_lower_vector   &kruhobot_linear_Conflicts,
					sInt_32                                &last_conflict_id,					
					bool                                    infinity = false) const;

	void introduce_KruhobotConflict(const Traversal                        &kruhobot_traversal,
					KruhobotLocationConflicts_upper_vector &kruhobot_location_Conflicts,
					KruhobotLinearConflicts_upper_vector   &kruhobot_linear_Conflicts,
					sInt_32                                &last_conflict_id,
					bool                                    infinity = false) const;
        /*----------------------------------------------------------------------------*/		

	sDouble calc_KruhobotCollisionImportance(const Traversal            &traversal_A,
						 const Traversal            &traversal_B,
						 LocationCooccupations_umap &location_Cooccupations,
						 UlinearCooccupations_map    &linear_Cooccupations) const;

	sDouble calc_KruhobotCollisionImportance(const Traversal            &traversal,
						 LocationCooccupations_umap &location_Cooccupations,
						 UlinearCooccupations_map    &linear_Cooccupations) const;
        /*----------------------------------------------------------------------------*/
	// TODO

        /*----------------------------------------------------------------------------*/	

	static void smooth_Schedule(const Schedule_vector &Schedule, Schedule_vector &smooth_Schedule);
	static sDouble augment_Schedules(const sRealInstance &real_Instance, KruhobotSchedules_vector &kruhobot_Schedules);	
        /*----------------------------------------------------------------------------*/

	static void to_Screen(const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent = "");
	static void to_Stream(FILE *fw, const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent = "");
	static sResult to_File(const sString &filename, const KruhobotSchedules_vector &kruhobot_Schedules, const sString &indent = "");	
        /*----------------------------------------------------------------------------*/	

    public:
	sRealInstance *m_real_Instance;
	sDouble m_timeout;
    };

    
/*----------------------------------------------------------------------------*/
// sRealCBS

    class sRealCBS
	: public sRealCBSBase
    {
    public:
	struct Node
	{
	    Node(sInt_32 node_id)
	    : m_node_id(node_id) { /* nothing */ }

	    void to_Screen(const sString &indent = "") const
	    {
		to_Stream(stdout, indent);
	    }
	    
	    void to_Stream(FILE *fw, const sString &indent = "") const
	    {
		fprintf(fw, "%sNode (node_id = %d, upper_id = %d, left_id = %d, right_id = %d, udp_kruhobot_id = %d, {cost = %.3f, make = %.3f, conf = %.3f}) [\n",
			indent.c_str(),
			m_node_id,
			m_upper_node_id,
			m_left_node_id,
			m_right_node_id,
			m_update_kruhobot_id,
			m_cost,
			m_makespan,
			m_conflicting);

		sInt_32 N_kruhobots_1 = m_kruhobot_location_Conflicts.size();
		fprintf(fw, "%s%slocation conflicts: {\n", indent.c_str(), s_INDENT.c_str());
		for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
		{
		    if (!m_kruhobot_location_Conflicts[kruhobot_id].empty())
		    {
			fprintf(fw, "%s%s%d: [\n", indent.c_str(), s2_INDENT.c_str(), kruhobot_id);

			for (LocationConflicts__umap::const_iterator location_Conflicts = m_kruhobot_location_Conflicts[kruhobot_id].begin(); location_Conflicts != m_kruhobot_location_Conflicts[kruhobot_id].end(); ++location_Conflicts)
			{
			    fprintf(fw, "%s%slocation: %d\n", indent.c_str(), s3_INDENT.c_str(), location_Conflicts->first);
			    for (LocationConflicts_map::const_iterator location_conflict = location_Conflicts->second.begin(); location_conflict != location_Conflicts->second.end(); ++location_conflict)
			    {
				location_conflict->second.to_Screen(indent + s4_INDENT);
			    }
			}
			fprintf(fw, "%s%s]\n", indent.c_str(), s2_INDENT.c_str());		    
		    }
		}
		fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());

		N_kruhobots_1 = m_kruhobot_linear_Conflicts.size();		
		fprintf(fw, "%s%slinear conflicts: {\n", indent.c_str(), s_INDENT.c_str());
		for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
		{
		    if (!m_kruhobot_linear_Conflicts[kruhobot_id].empty())
		    {
			fprintf(fw, "%s%s%d: [\n", indent.c_str(), s2_INDENT.c_str(), kruhobot_id);

			for (LinearConflicts__map::const_iterator linear_Conflicts = m_kruhobot_linear_Conflicts[kruhobot_id].begin(); linear_Conflicts != m_kruhobot_linear_Conflicts[kruhobot_id].end(); ++linear_Conflicts)
			{
			    fprintf(fw, "%s%sline: %d<->%d\n", indent.c_str(), s3_INDENT.c_str(), linear_Conflicts->first.m_lower_id, linear_Conflicts->first.m_upper_id);
			    for (LinearConflicts_map::const_iterator linear_conflict = linear_Conflicts->second.begin(); linear_conflict != linear_Conflicts->second.end(); ++linear_conflict)
			    {
				linear_conflict->second.to_Screen(indent + s4_INDENT);
			    }
			}
			fprintf(fw, "%s%s]\n", indent.c_str(), s2_INDENT.c_str());		    
		    }
		}
		fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());

		N_kruhobots_1 = m_kruhobot_Schedules.size();		
		fprintf(fw, "%s%sschedules: {\n", indent.c_str(), s_INDENT.c_str());
		for (sInt_32 kruhobot_id = 1; kruhobot_id < N_kruhobots_1; ++kruhobot_id)
		{
		    fprintf(fw, "%s%s%d: [\n", indent.c_str(), s2_INDENT.c_str(), kruhobot_id);

		    for (Schedule_vector::const_iterator event = m_kruhobot_Schedules[kruhobot_id].begin(); event != m_kruhobot_Schedules[kruhobot_id].end(); ++event)
		    {
			event->to_Screen(indent + s3_INDENT);
		    }
		    fprintf(fw, "%s%s]\n", indent.c_str(), s2_INDENT.c_str());		    
		}
		fprintf(fw, "%s%s}\n", indent.c_str(), s_INDENT.c_str());		

		fprintf(fw, "%s]\n", indent.c_str());
	    }	    

	    sInt_32 m_node_id;

	    sDouble m_cost;
	    sDouble m_makespan;
	    sDouble m_conflicting;

	    sInt_32 m_upper_node_id;
	    sInt_32 m_left_node_id;
	    sInt_32 m_right_node_id;

	    sInt_32 m_update_kruhobot_id;	    

	    KruhobotLocationConflicts_vector m_kruhobot_location_Conflicts;
	    KruhobotLinearConflicts_vector   m_kruhobot_linear_Conflicts;
	    KruhobotSchedules_vector         m_kruhobot_Schedules;
	};

	typedef std::vector<Node> Nodes_vector;

	struct NodeReference
	{
	    NodeReference(sInt_32 node_id, const Nodes_vector *nodes_Store)
	    : m_node_id(node_id)
	    , m_nodes_Store(nodes_Store)
	    {
		// nothing
	    }

	    bool operator<(const NodeReference &node_ref) const
	    {
//		return m_node_id < node_ref.m_node_id;		
		return ((*m_nodes_Store)[m_node_id].m_makespan < (*m_nodes_Store)[node_ref.m_node_id].m_makespan);	
//		return ((*m_nodes_Store)[m_node_id].m_cost < (*m_nodes_Store)[node_ref.m_node_id].m_cost);
/*
		return ((*m_nodes_Store)[m_node_id].m_cost < (*m_nodes_Store)[node_ref.m_node_id].m_cost
			|| ((*m_nodes_Store)[m_node_id].m_cost == (*m_nodes_Store)[node_ref.m_node_id].m_cost
			    && (*m_nodes_Store)[m_node_id].m_conflicting < (*m_nodes_Store)[node_ref.m_node_id].m_conflicting));
*/
		
		return ((*m_nodes_Store)[m_node_id].m_cost < (*m_nodes_Store)[node_ref.m_node_id].m_cost
			|| ((*m_nodes_Store)[m_node_id].m_cost == (*m_nodes_Store)[node_ref.m_node_id].m_cost
			    && (*m_nodes_Store)[m_node_id].m_makespan < (*m_nodes_Store)[node_ref.m_node_id].m_makespan));
		
/*		
		return ((*m_nodes_Store)[m_node_id].m_makespan < (*m_nodes_Store)[node_ref.m_node_id].m_makespan
			|| ((*m_nodes_Store)[m_node_id].m_makespan == (*m_nodes_Store)[node_ref.m_node_id].m_makespan
			    && (*m_nodes_Store)[m_node_id].m_conflicting < (*m_nodes_Store)[node_ref.m_node_id].m_conflicting));
*/
	    }

	    sInt_32 m_node_id;
	    const Nodes_vector *m_nodes_Store;
	};

	typedef std::multimap<sInt_32, Node, std::less<sInt_32>> Nodes_mmap;

	typedef std::set<Node, std::less<Node> > Nodes_set;
	typedef std::multiset<Node, std::less<Node> > Nodes_mset;

	typedef std::set<NodeReference, std::less<NodeReference> > NodeReferences_set;
	typedef std::multiset<NodeReference, std::less<NodeReference> > NodeReferences_mset;
	typedef std::vector<NodeReference> NodeReferences_vector;

	typedef std::vector<sInt_32> NodeIDs_vector;	
	
    public:
	sRealCBS(sRealInstance *real_Instance);
	sRealCBS(sRealInstance *real_Instance, sDouble timeout);
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules(sRealSolution &real_Solution, sDouble cost_limit);
	sDouble find_ShortestNonconflictingSchedules(const  sRealInstance &real_Instance,
						     sRealSolution        &real_Solution,
						     sDouble               cost_limit);
	
	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit);	
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   cost_limit);

	sDouble find_ShortestNonconflictingSchedules(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost);
	sDouble find_ShortestNonconflictingSchedules(const sRealInstance      &real_Instance,
						     KruhobotSchedules_vector &kruhobot_Schedules,
						     sDouble                   cost_limit,
						     sDouble                   extra_cost);
	/*----------------------------------------------------------------------------*/	

	sDouble find_ShortestNonconflictingSchedules_smart(sRealSolution &real_Solution, sDouble cost_limit);
	sDouble find_ShortestNonconflictingSchedules_smart(const  sRealInstance &real_Instance,
							     sRealSolution        &real_Solution,
							     sDouble               cost_limit);
	
	sDouble find_ShortestNonconflictingSchedules_smart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit);	
	sDouble find_ShortestNonconflictingSchedules_smart(const sRealInstance      &real_Instance,
							     KruhobotSchedules_vector &kruhobot_Schedules,
							     sDouble                   cost_limit);

	sDouble find_ShortestNonconflictingSchedules_smart(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost);
	sDouble find_ShortestNonconflictingSchedules_smart(const sRealInstance      &real_Instance,
							     KruhobotSchedules_vector &kruhobot_Schedules,
							     sDouble                   cost_limit,
							     sDouble                   extra_cost);	
	/*----------------------------------------------------------------------------*/

	sDouble find_ShortestNonconflictingSchedules_strong(sRealSolution &real_Solution, sDouble cost_limit);
	sDouble find_ShortestNonconflictingSchedules_strong(const  sRealInstance &real_Instance,
							    sRealSolution        &real_Solution,
							    sDouble               cost_limit);
	
	sDouble find_ShortestNonconflictingSchedules_strong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit);	
	sDouble find_ShortestNonconflictingSchedules_strong(const sRealInstance      &real_Instance,
							    KruhobotSchedules_vector &kruhobot_Schedules,
							    sDouble                   cost_limit);

	sDouble find_ShortestNonconflictingSchedules_strong(KruhobotSchedules_vector &kruhobot_Schedules, sDouble cost_limit, sDouble extra_cost);
	sDouble find_ShortestNonconflictingSchedules_strong(const sRealInstance      &real_Instance,
							    KruhobotSchedules_vector &kruhobot_Schedules,
							    sDouble                   cost_limit,
							    sDouble                   extra_cost);	
	/*----------------------------------------------------------------------------*/		

	sDouble find_NonconflictingSchedules(const sRealInstance              &real_Instance,
					     KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					     KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					     KruhobotSchedules_vector         &kruhobot_Schedules,
					     sDouble                           cost_limit,
					     sDouble                           extra_cost);

	sDouble find_NonconflictingSchedules_smart(const sRealInstance              &real_Instance,
						     KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						     KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						     KruhobotSchedules_vector         &kruhobot_Schedules,
						     sDouble                           cost_limit,
						     sDouble                           extra_cost);

	sDouble find_NonconflictingSchedules_strong(const sRealInstance              &real_Instance,
						    KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						    KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						    KruhobotSchedules_vector         &kruhobot_Schedules,
						    sDouble                           cost_limit,
						    sDouble                           extra_cost);		
	
	sDouble update_NonconflictingSchedule(sInt_32                           upd_kruhobot_id,
					      const sRealInstance              &real_Instance,
					      KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
					      KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
					      KruhobotSchedules_vector         &kruhobot_Schedules,
					      sDouble                           makespan_bound,
					      sDouble                           cost_limit,
					      sDouble                           extra_cost) const;

	sDouble update_NonconflictingSchedule_smart(sInt_32                           upd_kruhobot_id,
						    const sRealInstance              &real_Instance,
						    KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						    KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						    KruhobotSchedules_vector         &kruhobot_Schedules,
						    sDouble                           makespan_bound,
						    sDouble                           cost_limit,
						    sDouble                           extra_cost) const;

	sDouble update_NonconflictingSchedule_strong(sInt_32                           upd_kruhobot_id,
						     const sRealInstance              &real_Instance,
						     KruhobotLocationConflicts_vector &kruhobot_location_Conflicts,
						     KruhobotLinearConflicts_vector   &kruhobot_linear_Conflicts,
						     KruhobotSchedules_vector         &kruhobot_Schedules,
						     sDouble                           makespan_bound,
						     sDouble                           cost_limit,
						     sDouble                           extra_cost) const;		

	sDouble examine_NonconflictingSchedules(const sRealInstance              &real_Instance,
						sInt_32                           upper_node_id,
						KruhobotLocationConflicts_vector  kruhobot_location_Conflicts,
						KruhobotLinearConflicts_vector    kruhobot_linear_Conflicts,
						KruhobotSchedules_vector          kruhobot_Schedules,
						sDouble                           cost_limit,
						sDouble                           extra_cost,
						Nodes_vector                     &search_Store,
						NodeReferences_mset              &search_Queue) const;
	/*----------------------------------------------------------------------------*/
	
	sDouble find_KruhobotNonconflictingSchedule(const sKruhobot               &kruhobot,
						    const s2DMap                  &map,
						    sInt_32                        source_loc_id,
						    sInt_32                        sink_loc_id,
						    sDouble                        cost_limit,
						    sDouble                        extra_cost,
						    const LocationConflicts__umap &location_Conflicts,
						    const LinearConflicts__map    &linear_Conflicts,
						    sDouble                        makespan_bound,
						    Schedule_vector               &Schedule) const;

	sDouble find_KruhobotNonconflictingSchedule_smart(const sKruhobot               &kruhobot,
							  const s2DMap                  &map,
							  sInt_32                        source_loc_id,
							  sInt_32                        sink_loc_id,
							  sDouble                        cost_limit,
							  sDouble                        extra_cost,
							  const LocationConflicts__umap &location_Conflicts,
							  const LinearConflicts__map    &linear_Conflicts,
							  sDouble                        makespan_bound,
							  Schedule_vector               &Schedule) const;

	sDouble find_KruhobotNonconflictingSchedule_strong(const sKruhobot               &kruhobot,
							   const s2DMap                  &map,
							   sInt_32                        source_loc_id,
							   sInt_32                        sink_loc_id,
							   sDouble                        cost_limit,
							   sDouble                        extra_cost,
							   const LocationConflicts__umap &location_Conflicts,
							   const LinearConflicts__map    &linear_Conflicts,
							   sDouble                        makespan_bound,
							   Schedule_vector               &Schedule) const;	

	void generate_KruhobotImportantPoints(const sKruhobot               &kruhobot,
					      const s2DMap                  &map,
					      sInt_32                        source_loc_id,
					      sInt_32                        sink_loc_id,
					      sDouble                        cost_limit,
					      sDouble                        extra_cost,
					      const LocationConflicts__umap &location_Conflicts,
					      const LinearConflicts__map    &linear_Conflicts,
					      sDouble                        makespan_bound) const;

	bool check_KruhobotNonconflictingSchedule(const Schedule_vector         &Schedule,
						  const LocationConflicts__umap &location_Conflicts,
						  const LinearConflicts__map    &linear_Conflicts) const;
	/*----------------------------------------------------------------------------*/

	static sInt_32 calc_NodeDepth(const Node &node, const Nodes_vector &nodes_Store);

	static sDouble calc_ScheduleMakespan(const sRealInstance &real_Instance, const KruhobotSchedules_vector &kruhobot_Schedules);		
	static sDouble calc_ScheduleCost(const sRealInstance &real_Instance, const KruhobotSchedules_vector &kruhobot_Schedules);
        /*----------------------------------------------------------------------------*/

	static void to_Screen(const Node &node, const Nodes_vector &nodes_Store, const sString &indent = "");
	static void to_Stream(FILE *fw, const Node &node, const Nodes_vector &nodes_Store, const sString &indent = "");
    };


/*----------------------------------------------------------------------------*/

} // namespace boOX

#endif /* __CBSR_H__ */
    
