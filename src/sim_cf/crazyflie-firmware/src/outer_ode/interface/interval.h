#ifndef __INTERVAL_H__
#define __INTERVAL_H__

#include "config.h"

class Interval
{
public:
	real l_elem;
	real r_elem;

public:
	Interval(real val = 0.0f);
	Interval(const Interval&);
	Interval(real val1 , real val2);

	Interval & operator = (const Interval &);

	Interval operator + (const Interval &) const;
	Interval operator - (const Interval &) const;
	Interval operator * (const Interval &) const;
	Interval operator / (const Interval &) const;

	Interval & operator += (const Interval &);
	Interval & operator -= (const Interval &);
	Interval & operator *= (const Interval &);
	Interval & operator /= (const Interval &);

	bool operator == (const Interval &) const;

	real getCenter() const;
	real getMin() const;
	real getMax() const;
	real getRadius() const;
	real getRange() const;
	
#ifdef VERBOSE
	void print_it(FILE *file = NULL);
#endif

};

uint8_t subseteq(const Interval& , const Interval&);

#endif //__INTERVAL_H__