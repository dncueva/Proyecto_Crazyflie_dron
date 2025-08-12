#ifndef __AA_MOD_H__
#define __AA_MOD_H__

#include "config.h"
#include "interval.h"

class MAF2 {

private:
	real center;
	real deviations[N_NOISE];
	unsigned int indexes[N_NOISE];
	unsigned int nbIndexes;

	static unsigned int index_last;

public:
	// Constructor
	MAF2(real center = 0.0f);
	MAF2(const MAF2 &);
	MAF2(const Interval&);

	// Destructor
	~MAF2();

	// Overloading operators
	MAF2 & operator = (const MAF2 &);
	MAF2 & operator = (const real);

	real & operator[](uint16_t );
	real operator[](uint16_t ) const;

	MAF2 operator + (const MAF2 &) const;
	MAF2 operator - (const MAF2 &) const;
	MAF2 operator * (const MAF2 &) const;
	MAF2 operator / (const MAF2 &) const;

	MAF2 & operator += (const MAF2 &);
	MAF2 & operator -= (const MAF2 &);
	MAF2 & operator *= (const MAF2 &);

	bool operator == (const MAF2 &) const;
	// MAF2 operator ^ (const uint8_t) const;

	// unary operator minus
	MAF2 operator - () const;

	MAF2 operator + (const real ) const;
	MAF2 & operator += (const real );

	MAF2 operator - (const real ) const;
	MAF2 & operator -= (const real );

	MAF2 operator * (const real) const;
	MAF2 & operator *= (const real);

	MAF2 operator / (const real) const;

	real getCenter() const;
	Interval getInterval() const;

	real getMax() const;
	real getMin() const;
	real getRadius() const;

	void compress_af(real tol);

#ifdef VERBOSE
	void print_af(FILE *f = NULL);
#endif

	friend MAF2 cos(const MAF2 &);
	friend MAF2 sin(const MAF2 &);
	friend MAF2 tan(const MAF2 &);
	friend MAF2 inv(const MAF2 &);
};

MAF2 operator * (real , const MAF2 &);
MAF2 operator / (real , const MAF2 &);
MAF2 operator + (real , const MAF2 &);
MAF2 operator - (real , const MAF2 &);

#endif //__AA_MOD_H__