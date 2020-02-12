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
/* safe_intervals.cpp / 1-224_leibniz                                         */
/*----------------------------------------------------------------------------*/
//
// Exact safe time interval calculation for circular robots
// with constant velocities.
//
/*----------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vector>


#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define ABS(x) (((x) < 0) ? -(x) : (x))    


namespace boOX
{    
    struct Interval
    {
	Interval() {  }
	Interval(double lower, double upper)
	    : m_lower(lower)
	    , m_upper(upper)
	{ }
	
	    double size(void) const
	    {		
		return (m_upper - m_lower);
	    }	    	
	    Interval intersect(const Interval &interval) const
	    {
		return Interval(MAX(m_lower, interval.m_lower), MIN(m_upper, interval.m_upper));
	    }
	    void to_Screen() const
	    {
		printf("[%.3f,%.3f)\n", m_lower, m_upper);
	    }	    	
	
	    double m_lower, m_upper;
    };
    

    const double EPSILON = 0.000000001;
    const double DELTA = 0.0001;

    std::pair<bool, bool> calculate_UnsafeIntervals(
	double               A_x1,  /* origin of agent A */
	double               A_y1,
	double               A_x2,  /* destination of agent A */
	double               A_y2,
	const Interval      &A_time, /* time interval of agent A */
	double               B_x1,  /* origin of agent B */
	double               B_y1,
	double               B_x2,  /* destination of agent B */
	double               B_y2,
	const Interval      &B_time,  /* time interval of agent B */
	double               A_v,   /* velocity of agent A */
	double               A_r,   /* radius of agent A */
	double               B_v,   /* velocity of agent B */
	double               B_r,   /* radius of agent B */				   
	Interval            &unsafe_A, /* unsafe time interval for A */
	Interval            &unsafe_B) /* unsafe time interval for B */
    /* returns true in some element of the pair if some avoidance needed, false/false if it is safe to go ahead for both agents */
    /* time intervals, origins and destinations must be consistent together with velocities */
    {
	bool avoid_A = false;
	bool avoid_B = false;
	
	double vA = A_v;
	double rA = A_r;
	
	double vB = B_v;
	double rB = B_r;

	double x1A_ = A_x1;
	double x2A_ = A_x2;
	double y1A_ = A_y1;
	double y2A_ = A_y2;
	
	double x1B_ = B_x1;
	double x2B_ = B_x2;
	double y1B_ = B_y1;
	double y2B_ = B_y2;

	double dxA = x2A_ - x1A_;
	double dyA = y2A_ - y1A_;

	double dxB = x2B_ - x1B_;
	double dyB = y2B_ - y1B_;

	double d2A = dxA * dxA + dyA * dyA;
	double dA = sqrt(d2A);

	double d2B = dxB * dxB + dyB * dyB;
	double dB = sqrt(d2B);

	double x1A = x1A_ + (vA * (-A_time.m_lower) * dxA) / dA;
	double y1A = y1A_ + (vA * (-A_time.m_lower) * dyA) / dA;

	double x1B = x1B_ + (vB * (-B_time.m_lower) * dxB) / dB;
	double y1B = y1B_ + (vB * (-B_time.m_lower) * dyB) / dB;

	double dxAB = x1A - x1B;
	double dyAB = y1A - y1B;
	double rAB = rA + rB;

	{
	    double gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    double beta_x = vA * dxA / dA - vB * dxB / dB;
	    double beta_y = vA * dyA / dA - vB * dyB / dB;
	    double beta = 2 * (dxAB * beta_x + dyAB * beta_y);
	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double discriminant = beta * beta - 4 * alpha * gamma;
		
	    if (discriminant <= EPSILON)
	    {
		return std::pair<bool, bool>(false, false);
	    }
	    else
	    {
		double tau_1 = (-beta - sqrt(discriminant)) / (2 * alpha); 
		double tau_2 = (-beta + sqrt(discriminant)) / (2 * alpha);

		Interval check_intersection_A = A_time.intersect(Interval(tau_1, tau_2));
		Interval check_intersection_B = B_time.intersect(Interval(tau_1, tau_2));

		if (check_intersection_A.size() <= DELTA || check_intersection_B.size() <= DELTA)
		{
		    return std::pair<bool, bool>(false, false);
		}
	    }
	}
	    
	{
	    double gamma_0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
	    double gamma_1 = 2 * (vA * (dxAB * dxA + dyAB * dyA)) / dA;	
	    double gamma_2 = vA * vA;

	    double beta_0 = 2 * ((vA * dxAB * dxA / dA) - ((vB * dxAB * dxB) / dB) + (vA * dyAB * dyA / dA) - ((vB * dyAB * dyB) / dB));
	    double beta_x = (vA * dxA / dA) - (vB * dxB / dB);
	    double beta_y = (vA * dyA / dA) - (vB * dyB / dB);	
	    double beta_1 = 2 * ((vA * dxA * beta_x) / dA + (vA * dyA * beta_y) / dA);

	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double lower_unsafe_A, upper_unsafe_A;
	
	    double A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    double B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    double C = beta_0 * beta_0 - 4 * alpha * gamma_0;
	    
	    if (ABS(A) > EPSILON)
	    {
		double D = B * B - 4 * A * C;

		if (D > EPSILON)
		{
		    double t0_1 = (-B - sqrt(D)) / (2 * A); 
		    double t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_A = -t0_1 + A_time.m_lower;		    
		    upper_unsafe_A = -t0_2 + A_time.m_lower;
		    
		    Interval avoid_interval_A(MAX(0, MIN(lower_unsafe_A, upper_unsafe_A)), MAX(0, MAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > DELTA)
		    {
			Interval intersection_A = B_time.intersect(avoid_interval_A);
			
			if (intersection_A.size() > DELTA)
			{
			    unsafe_A = intersection_A;
			    avoid_A = true;
			}
		    }
		}
	    }
	    else
	    {
		double gamma = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
		double beta_x = vA * dxA / dA - vB * dxB / dB;
		double beta_y = vA * dyA / dA - vB * dyB / dB;
		double beta = 2 * (dxAB * beta_x + dyAB * beta_y);
		double alpha = beta_x * beta_x + beta_y * beta_y;
		
		double disc = beta * beta - 4 * alpha * gamma;

		if (disc > EPSILON)
		{
		    double time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    double time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    lower_unsafe_A = time0_1;
		    upper_unsafe_A = time0_2;

		    Interval avoid_interval_A(MAX(0, MIN(lower_unsafe_A, upper_unsafe_A)), MAX(0, MAX(lower_unsafe_A, upper_unsafe_A)));

		    if (avoid_interval_A.size() > DELTA)
		    {
			Interval intersection_A = A_time.intersect(avoid_interval_A);
		    
			if (intersection_A.size() > DELTA)
			{
			    double AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double BB0 = 2 * (vA * (dxAB * dxA + dyAB * dyA) / dA - vB * (dxAB * dxB + dyAB * dyB) / dB);
			    double BB1 = 2 * vA * vA - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double CC0 = dxAB * dxAB + dyAB * dyAB - rAB * rAB;
			    double CC1 = 2 * vA * (dxAB * dxA + dyAB * dyA) / dA;
			    double CC2 = vA * vA;

			    double tB2 = B_time.m_upper;

			    double A = AA * CC2;
			    double B = AA * BB1 * tB2 + AA * CC1;
			    double C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    double DD = B * B - 4 * A * C;

			    if (DD > EPSILON)
			    {
				double T0_1 = (-B - sqrt(DD)) / (2 * A); 
				double T0_2 = (-B + sqrt(DD)) / (2 * A);
				
				double lower_unsafe_A_ = A_time.m_lower - MIN(0, T0_1);
				double upper_unsafe_A_ = A_time.m_lower - MIN(0, T0_2);

				Interval avoid_interval_A_(MAX(0, MIN(lower_unsafe_A_, upper_unsafe_A_)), MAX(0, MAX(lower_unsafe_A_, upper_unsafe_A_)));
				
				if (avoid_interval_A_.size() > DELTA)
				{
				    Interval intersection_A_ = A_time.intersect(avoid_interval_A_);
				    
				    if (intersection_A_.size() > DELTA)
				    {
					unsafe_A = intersection_A_;
					avoid_A = true;
				    }
				}
			    }
			    else
			    {
				unsafe_A = A_time;
				avoid_A = true;
			    }
			}
		    }		    
		}		
	    }
	}

	double dxBA = x1B - x1A;
	double dyBA = y1B - y1A;

	{
	    double gamma_0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
	    double gamma_1 = 2 * (vB * (dxBA * dxB + dyBA * dyB)) / dB;	
	    double gamma_2 = vB * vB;

	    double beta_0 = 2 * ((vB * dxBA * dxB / dB) - ((vA * dxBA * dxA) / dA) + (vB * dyBA * dyB / dB) - ((vA * dyBA * dyA) / dA));
	    double beta_x = (vB * dxB / dB) - (vA * dxA / dA);
	    double beta_y = (vB * dyB / dB) - (vA * dyA / dA);
	    double beta_1 = 2 * ((vB * dxB * beta_x) / dB + (vB * dyB * beta_y) / dB);	    

	    double alpha = beta_x * beta_x + beta_y * beta_y;
	    
	    double lower_unsafe_B, upper_unsafe_B;
	
	    double A = beta_1 * beta_1 - 4 * alpha * gamma_2;
	    double B = 2 * beta_0 * beta_1 - 4 * alpha * gamma_1;
	    double C = beta_0 * beta_0 - 4 * alpha * gamma_0;

	    if (ABS(A) > EPSILON)
	    {
		double D = B * B - 4 * A * C;

		if (D > EPSILON)
		{
		    double t0_1 = (-B - sqrt(D)) / (2 * A); 
		    double t0_2 = (-B + sqrt(D)) / (2 * A);

		    lower_unsafe_B = -t0_1 + B_time.m_lower;
		    upper_unsafe_B = -t0_2 + B_time.m_lower;

		    Interval avoid_interval_B(MAX(0, MIN(lower_unsafe_B, upper_unsafe_B)), MAX(0, MAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > DELTA)
		    {
			Interval intersection_B = A_time.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > DELTA)
			{
			    unsafe_B = intersection_B;
			    avoid_B = true;
			}
		    }
		}
	    }
	    else
	    {
		double gamma = dxBA * dxBA + dyBA * dyBA - rAB * rAB;
		double beta_x = vB * dxB / dB - vA * dxA / dA;
		double beta_y = vB * dyB / dB - vA * dyA / dA;
		double beta = 2 * (dxBA * beta_x + dyBA * beta_y);
		double alpha = beta_x * beta_x + beta_y * beta_y;		

		double disc = beta * beta - 4 * alpha * gamma;

		if (disc > EPSILON)
		{
		    double time0_1 = (-beta - sqrt(disc)) / (2 * alpha); 
		    double time0_2 = (-beta + sqrt(disc)) / (2 * alpha);

		    lower_unsafe_B = time0_1;
		    upper_unsafe_B = time0_2;		    

		    Interval avoid_interval_B(MAX(0, MIN(lower_unsafe_B, upper_unsafe_B)), MAX(0, MAX(lower_unsafe_B, upper_unsafe_B)));

		    if (avoid_interval_B.size() > DELTA)
		    {
			Interval intersection_B = B_time.intersect(avoid_interval_B);
		    
			if (intersection_B.size() > DELTA)
			{
			    double AA = vA * vA + vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);
			    double BB0 = 2 * (vB * (dxBA * dxB + dyBA * dyB) / dB - vA * (dxBA * dxA + dyBA * dyA) / dA);
			    double BB1 = 2 * vB * vB - 2 * vA * vB * (dxA * dxB + dyA * dyB) / (dA * dB);			    
			    double CC0 = dxBA * dxBA + dyBA * dyBA - rAB * rAB;			    
			    double CC1 = 2 * vB * (dxBA * dxB + dyBA * dyB) / dB;
			    double CC2 = vB * vB;

			    double tB2 = A_time.m_upper;

			    double A = AA * CC2;
			    double B = AA * BB1 * tB2 + AA * CC1;
			    double C = AA * BB0 * tB2 + AA * CC0 + AA * AA * tB2 * tB2;

			    double DD = B * B - 4 * A * C;

			    if (DD > EPSILON)
			    {
				double T0_1 = (-B - sqrt(DD)) / (2 * A); 
				double T0_2 = (-B + sqrt(DD)) / (2 * A);
			    
				double lower_unsafe_B_ = B_time.m_lower - MIN(0, T0_1);
				double upper_unsafe_B_ = B_time.m_lower - MIN(0, T0_2);
				
				Interval avoid_interval_B_(MAX(0, MIN(lower_unsafe_B_, upper_unsafe_B_)), MAX(0, MAX(lower_unsafe_B_, upper_unsafe_B_)));
				
				if (avoid_interval_B_.size() > DELTA)
				{
				    Interval intersection_B_ = A_time.intersect(avoid_interval_B_);
				    
				    if (intersection_B_.size() > DELTA)
				    {
					unsafe_B = intersection_B_;
					avoid_B = true;
				    }
				}
			    }
			    else
			    {
				unsafe_B = B_time;
				avoid_B = true;
			    }
			}
		    }		    
		}		
	    }
	}
	return std::pair<bool, bool>(avoid_A, avoid_B);
    } 


} // namespace boOX

using namespace boOX;

int main(void)
{
    std::pair<bool, bool> result;
    
    Interval unsafe_A(-1.0, -1.0);
    Interval unsafe_B(-1.0, -1.0);

    result = calculate_UnsafeIntervals(0.0,
				       1.0,
				       2.0,
				       1.0,
				       Interval(0.0, 2.0),
				       1.0,
				       0.0,
				       1.0,
				       2.0,
				       Interval(0.0, 2.0),
				       1.0,
				       0.25,
				       1.0,
				       0.25,
				       unsafe_A,
				       unsafe_B);
    
    printf("Result: %d,%d\n", result.first, result.second);
    
    unsafe_A.to_Screen();
    unsafe_B.to_Screen();
    
    return 0;
}
