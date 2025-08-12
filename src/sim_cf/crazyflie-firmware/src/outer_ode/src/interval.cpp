#include "interval.h"
#include <algorithm>

using namespace std;

Interval::Interval(real val)
{
	this->l_elem = val;
	this->r_elem = val;
}

Interval::Interval(real lo , real hi)
{
	assert_af(lo <= hi);
	this->l_elem = lo;
	this->r_elem = hi;
}

Interval::Interval(const Interval &it)
{
	this->l_elem = it.l_elem;
	this->r_elem = it.r_elem;
}

real Interval::getCenter() const
{
	return (this->l_elem + this->r_elem) / 2;
}

real Interval::getMin() const
{
	return this->l_elem;
}

real Interval::getMax() const
{
	return this->r_elem;
}

real Interval::getRadius() const
{
	return (this->r_elem - this->l_elem)/2;
}

real Interval::getRange() const
{
	return (this->r_elem - this->l_elem);
}

Interval & Interval::operator = (const Interval &it)
{

	this->l_elem = it.l_elem;
	this->r_elem = it.r_elem;
	return *this;
}

Interval Interval::operator + (const Interval &it) const
{
	Interval temp(*this);
	temp.r_elem += it.r_elem;
	temp.l_elem += it.l_elem;
	return  temp;
}

Interval Interval::operator - (const Interval &it) const
{
	Interval temp(*this);
	temp.r_elem -= it.l_elem;
	temp.l_elem -= it.r_elem;
	return  temp;
}

Interval Interval::operator * (const Interval &it) const
{
	real x1 = this->l_elem * it.l_elem;
	real x2 = this->l_elem * it.r_elem;
	real x3 = this->r_elem * it.l_elem;
	real x4 = this->r_elem * it.r_elem;
	return Interval(min(x1 , min(x2 , min(x3 , x4))), max(x1 , max(x2 , max(x3 , x4))));
}

Interval Interval::operator / (const Interval &it) const
{
	assert_af(it.l_elem * it.r_elem > 0);
	return *this * Interval(1.0f/it.r_elem , 1.0f/it.l_elem);
}

Interval & Interval::operator += (const Interval &it)
{
	this->l_elem += it.l_elem;
	this->r_elem += it.r_elem;
	return *this;
}

Interval & Interval::operator -= (const Interval &it)
{
	this->l_elem -= it.r_elem;
	this->r_elem -= it.l_elem;
	return *this;
}

Interval & Interval::operator *= (const Interval &it)
{
	real x1 = this->l_elem * it.l_elem;
	real x2 = this->l_elem * it.r_elem;
	real x3 = this->r_elem * it.l_elem;
	real x4 = this->r_elem * it.r_elem;
	this->l_elem = min(x1 , min(x2 , min(x3 , x4)));
	this->r_elem = max(x1 , max(x2 , max(x3 , x4)));
	return *this;
}

Interval & Interval::operator /= (const Interval &it)
{
	assert_af(it.l_elem * it.r_elem > 0);
	real x1 = this->l_elem / it.r_elem;
	real x2 = this->l_elem / it.l_elem;
	real x3 = this->r_elem / it.r_elem;
	real x4 = this->r_elem / it.l_elem;
	this->l_elem = min(x1 , min(x2 , min(x3 , x4)));
	this->r_elem = max(x1 , max(x2 , max(x3 , x4)));
	return *this;
}

bool Interval::operator == (const Interval &it) const
{
	return it.l_elem == this->l_elem && it.r_elem == this->r_elem;
}

#ifdef VERBOSE
void Interval::print_it(FILE *file)
{
	if (file != NULL){
		fprintf(file ,"%f\t%f\n", this->getMin() , this->getMax());
		return ;
	}
	printf("it = [%f , %f] \n", this->getMin() , this->getMax());
}
#endif

uint8_t subseteq(const Interval &it1 , const Interval &it2)
{
	if (it1.r_elem <= it2.r_elem && it1.l_elem >= it2.l_elem)
		return 1;
	return 0;
}