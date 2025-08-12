#include "aa_mod.h"
#include <cstring>
#include <algorithm>

using namespace std;

/* This function takes last element as pivot, places
   the pivot element at its correct position in sorted
    array, and places all smaller (smaller than pivot)
   to left of pivot and all greater elements to right
   of pivot */
// int partition_dev (real arr[], unsigned int indexes[], int low, int high);

// void ksmallest(int arr)
/*bool compare(unsigned int a , unsigned int b , real *data)
{
	return data[a] < data[b];
}*/

//  The main function that implements QuickSort
//  arr[] --> Array to be sorted,
//   low  --> Starting index,
//   high  --> Ending index 
// void quickSort (real arr[], unsigned int indexes[], int low, int high);

unsigned int MAF2::index_last = 1;

/************************************************************/
/* Common definition of constructor + getter of affine form */
/************************************************************/
MAF2::MAF2(real center)
{
	this->center = center;;
	// for (uint8_t index = 0; index < N_NOISE ; index++){
	// 	this->deviations[index] = 0.0f;
	// 	this->indexes[index] = 0;
	// }
	this->nbIndexes = 0;
}

MAF2::MAF2(const MAF2 &af)
{
	this->center = af.center;
	this->nbIndexes = af.nbIndexes;
	memcpy(this->deviations , af.deviations , this->nbIndexes * sizeof *this->deviations);
	memcpy(this->indexes , af.indexes , this->nbIndexes * sizeof *this->indexes);
}

MAF2::MAF2(const Interval &it)
{
	this->center = it.getCenter();
	// for (uint8_t index = 0; index < N_NOISE ; index++){
	// 	this->deviations[index] = 0.0f;
	// 	this->indexes[index] = 0;
	// }
	this->deviations[0] = it.getRadius();
	this->nbIndexes = 1;
	this->indexes[0] = index_last;
	index_last++;
}

MAF2::~MAF2(){}

MAF2 & MAF2::operator = (const real center)
{
	this->center = center;
	// for (uint8_t index = 0; index < N_NOISE ; index++){
	// 	this->deviations[index] = 0.0f;
	// 	this->indexes[index] = 0;
	// }
	this->nbIndexes = 0;
	return *this;
}

MAF2 & MAF2::operator = (const MAF2 &af)
{
	this->center = af.center;
	this->nbIndexes = af.nbIndexes;
	memcpy(this->deviations , af.deviations , this->nbIndexes * sizeof *this->deviations);
	memcpy(this->indexes , af.indexes , this->nbIndexes * sizeof *this->indexes);
	return *this;
}

real & MAF2::operator[](uint16_t index) 
{
	assert_af(index>=0 && index < this->nbIndexes);
	return this->deviations[index];
}

real MAF2::operator[](uint16_t index) const
{
	assert_af(index>=0 && index < this->nbIndexes);
	return this->deviations[index];
}

real MAF2::getCenter() const
{
	return this->center;
}

Interval MAF2::getInterval() const
{
	real radius = this->getRadius();
	return Interval(this->center - radius , this->center + radius);
}

real MAF2::getRadius() const
{
	real radius = 0.0f;
	for (uint16_t index = 0 ; index < this->nbIndexes ; index++){
		radius += abs(this->deviations[index]);
	}
	return radius;
}

real MAF2::getMax() const
{
	return this->getCenter() + this->getRadius();
}

real MAF2::getMin() const
{
	return this->getCenter() - this->getRadius();
}

void MAF2::compress_af(real tol)
{
	real r = tol * this->getRadius() / this->nbIndexes;
	real rest = 0.0f;
	uint16_t index = 0;
	for(uint16_t i=0 ; i< this->nbIndexes ; i++){
		real temp = abs(this->deviations[i]);
		if( temp > r){
			this->deviations[index] = this->deviations[i];
			this->indexes[index] = this->indexes[i];
			index++;
		} else {
			rest += temp;
		}
	}
	if (index < this->nbIndexes){
		if (rest == 0){
			this->nbIndexes = index;
			return;
		}
		this->deviations[index] = rest;
		this->indexes[index] = index_last;
		this->nbIndexes = index + 1;
		index_last++;
	}
}

#ifdef VERBOSE
void MAF2::print_af(FILE *file)
{
	if (file != NULL){
		fprintf(file,"Center = %f \n", this->center);
		fprintf(file,"nbIndexes = %d \n", this->nbIndexes);
		for (uint16_t i=0 ; i < this->nbIndexes ; i++){
			fprintf(file,"---> eps[%d] = %f \n", (int) this->indexes[i] , this->deviations[i]);
		}
		fprintf(file,"-------------------------------\n");
		return ;
	}
	printf("Center = %f \n", this->center);
	printf("nbIndexes = %d \n", this->nbIndexes);
	for (uint16_t i=0 ; i < this->nbIndexes ; i++){
		printf("---> eps[%d] = %f \n", (int) this->indexes[i] , this->deviations[i]);
	}
	printf("-------------------------------\n");
}
#endif

/************************************************************/
/* 	Affine form basic arithmetic operations definition   	*/
/************************************************************/

MAF2 MAF2::operator + (const MAF2 &other) const
{
	if (this->nbIndexes + other.nbIndexes == 0){
		MAF2 temp(*this);
		temp.center += other.center;
		return temp;
	}

	if(this->nbIndexes == 0){
		MAF2 temp(other);
		temp.center += this->center;
		return temp; 
	}

	if(other.nbIndexes == 0){
		MAF2 temp(*this);
		temp.center += other.center;
		return temp;
	}

	const unsigned  *id1 = this->indexes;
	const unsigned  *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	//real * va1 = this->deviations;
	//real * va2 = other.deviations;

	unsigned int idTemp[2*N_NOISE];
	real devTemp[2*N_NOISE];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;

	unsigned a , b ;
	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = this->deviations[a] + other.deviations[b];
		pu1++;
		pu2++;
	}

	MAF2 temp(this->center + other.center);
	if (ltemp <= N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		temp.nbIndexes = ltemp;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			temp.indexes[ind] = idTemp[ind];
			temp.deviations[ind] = devTemp[ind];
		}
		return temp;
	}

	temp.nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		temp.indexes[i] = idTemp[i];
		temp.deviations[i] = devTemp[i];
	}
	real new_noise = 0.0f;
	for(uint16_t i=N_NOISE-1 ; i<ltemp ; i++){
		new_noise += abs(devTemp[i]);
	}
	temp.deviations[N_NOISE-1] = new_noise;
	temp.indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , bind(compare, _1, _2, devTemp));

	// temp.nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	temp.indexes[iter] = idTemp[i];
	// 	temp.deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// temp.deviations[iter] = new_noise;
	// temp.indexes[iter] = index_last;
	// index_last++;

	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// temp.nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	temp.indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	temp.deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// temp.deviations[0] = new_noise;
	// temp.indexes[0] = index_last;
	// index_last++;
	return temp;
}


MAF2 MAF2::operator - (const MAF2 &other) const 
{
	if (this->nbIndexes + other.nbIndexes == 0){
		MAF2 temp(*this);
		temp.center -= other.center;
		return temp;
	}

	if(this->nbIndexes == 0){
		MAF2 temp(other);
		temp.center *= -1;
		temp.center += this->center;
		for(uint16_t i=0 ; i < other.nbIndexes ; i++){
			temp.deviations[i] *= -1;
		}
		return temp; 
	}

	if(other.nbIndexes == 0){
		MAF2 temp(*this);
		temp.center -= other.center;
		return temp;
	}

	const unsigned int *id1 = this->indexes;
	const unsigned int *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	//real * va1 = this->deviations;
	//real * va2 = other.deviations;

	unsigned int idTemp[2*N_NOISE];
	real devTemp[2*N_NOISE];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;

	unsigned a , b ;
	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = -other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = this->deviations[a] - other.deviations[b];
		pu1++;
		pu2++;
	}

	MAF2 temp(this->center - other.center);
	if (ltemp <= N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		temp.nbIndexes = ltemp;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			temp.indexes[ind] = idTemp[ind];
			temp.deviations[ind] = devTemp[ind];
		}
		return temp;
	}

	temp.nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		temp.indexes[i] = idTemp[i];
		temp.deviations[i] = devTemp[i];
	}
	real new_noise = 0.0;
	for(uint16_t i=N_NOISE-1 ; i<ltemp ; i++){
		new_noise += abs(devTemp[i]);
	}
	temp.deviations[N_NOISE-1] = new_noise;
	temp.indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , std::bind(compare, _1, _2, devTemp));

	// temp.nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	temp.indexes[iter] = idTemp[i];
	// 	temp.deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// temp.deviations[iter] = new_noise;
	// temp.indexes[iter] = index_last;
	// index_last++;

	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// temp.nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	temp.indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	temp.deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// temp.deviations[0] = new_noise;
	// temp.indexes[0] = index_last;
	// index_last++;
	return temp;
}

MAF2 MAF2::operator - () const
{
	MAF2 temp(*this);
	temp.center *= -1;
	for (uint16_t index=0 ; index< this->nbIndexes ; index++){
		temp.deviations[index] *= -1;
	}
	// err_term doesn't change
	return temp;
}

MAF2 MAF2::operator + (const real val) const
{
	MAF2 temp(*this);
	temp.center += val;
	return temp;
}

MAF2 & MAF2::operator += (const real val)
{
	this->center += val;
	return *this;
}

MAF2 MAF2::operator * (const real val) const
{
	MAF2 temp(*this);
	temp.center *= val;
	for (uint16_t index=0 ; index< this->nbIndexes ; index++){
		temp.deviations[index] *= val;
	}
	return temp;
}

MAF2 & MAF2::operator *= (const real val)
{
	this->center *= val;
	for (uint16_t index=0 ; index < this->nbIndexes ; index++){
		this->deviations[index] *= val;
	}
	return *this;
}

MAF2 MAF2::operator / (const real val) const
{
	assert_af(val != 0);

	MAF2 temp(*this);
	temp.center /= val;
	for (uint16_t index=0 ; index< this->nbIndexes ; index++){
		temp.deviations[index] /= val;
	}
	return temp;
}

MAF2 & MAF2::operator += (const MAF2 &other)
{
	if (this->nbIndexes + other.nbIndexes == 0){
		this->center += other.center;
		return *this;
	}

	if(this->nbIndexes == 0){
		this->center += other.center;
		for(uint16_t i=0 ; i<other.nbIndexes ; i++){
			this->deviations[i] = other.deviations[i];
			this->indexes[i] = other.indexes[i];
		}
		this->nbIndexes = other.nbIndexes;
		return *this; 
	}

	if(other.nbIndexes == 0){
		this->center += other.center;
		return *this;
	}

	const unsigned int *id1 = this->indexes;
	const unsigned int *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	//real * va1 = this->deviations;
	//real * va2 = other.deviations;

	unsigned int idTemp[2*N_NOISE];
	real devTemp[2*N_NOISE];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;

	unsigned a , b ;

	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = this->deviations[a] + other.deviations[b];
		pu1++;
		pu2++;
	}

	this->center += other.center;

	if (ltemp <= N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		this->nbIndexes = ltemp;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			this->indexes[ind] = idTemp[ind];
			this->deviations[ind] = devTemp[ind];
		}
		return *this;
	}

	this->nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		this->indexes[i] = idTemp[i];
		this->deviations[i] = devTemp[i];
	}
	real new_noise = 0.0;
	for(uint16_t i=N_NOISE-1 ; i<ltemp ; i++){
		new_noise += abs(devTemp[i]);
	}
	this->deviations[N_NOISE-1] = new_noise;
	this->indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , std::bind(compare, _1, _2, devTemp));

	// this->nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	this->indexes[iter] = idTemp[i];
	// 	this->deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// this->deviations[iter] = new_noise;
	// this->indexes[iter] = index_last;
	// index_last++;

	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// this->nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	this->indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	this->deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// this->deviations[0] = new_noise;
	// this->indexes[0] = index_last;
	// index_last++;
	return *this;
}

MAF2 & MAF2::operator -= (const MAF2 &other)
{
	if (this->nbIndexes + other.nbIndexes == 0){
		this->center -= other.center;
		return *this;
	}

	if(this->nbIndexes == 0){
		this->center -= other.center;
		for(uint16_t i=0 ; i<other.nbIndexes ; i++){
			this->deviations[i] = -other.deviations[i];
			this->indexes[i] = other.indexes[i];
		}
		this->nbIndexes = other.nbIndexes;
		return *this; 
	}

	if(other.nbIndexes == 0){
		this->center -= other.center;
		return *this;
	}

	const unsigned int *id1 = this->indexes;
	const unsigned int *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	//real * va1 = this->deviations;
	//real * va2 = other.deviations;

	unsigned int idTemp[2*N_NOISE];
	real devTemp[2*N_NOISE];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;

	unsigned a , b ;

	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = -other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = this->deviations[a] - other.deviations[b];
		pu1++;
		pu2++;
	}

	this->center -= other.center;

	if (ltemp <= N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		this->nbIndexes = ltemp;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			this->indexes[ind] = idTemp[ind];
			this->deviations[ind] = devTemp[ind];
		}
		return *this;
	}

	this->nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		this->indexes[i] = idTemp[i];
		this->deviations[i] = devTemp[i];
	}
	real new_noise = 0.0;
	for(uint16_t i=N_NOISE-1 ; i<ltemp ; i++){
		new_noise += abs(devTemp[i]);
	}
	this->deviations[N_NOISE-1] = new_noise;
	this->indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , std::bind(compare, _1, _2, devTemp));

	// this->nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	this->indexes[iter] = idTemp[i];
	// 	this->deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// this->deviations[iter] = new_noise;
	// this->indexes[iter] = index_last;
	// index_last++;
	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// this->nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	this->indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	this->deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// this->deviations[0] = new_noise;
	// this->indexes[0] = index_last;
	// index_last++;
	return *this;
}

/************************************************************/
/* 	Affine form basic arithmetic operations approximation  	*/
/************************************************************/

MAF2 MAF2::operator * (const MAF2 &other) const
{
	if (this->nbIndexes + other.nbIndexes == 0){
		return MAF2(this->center * other.center);
	}

	if(this->nbIndexes == 0){
		MAF2 temp(other);
		temp.center *= this->center;
		for (uint16_t i=0 ; i < other.nbIndexes ; i++){
			temp.deviations[i] *= this->center;
		}
		return temp;
	}

	if(other.nbIndexes == 0){
		MAF2 temp(*this);
		temp.center *= other.center;
		for (uint16_t i=0 ; i < this->nbIndexes ; i++){
			temp.deviations[i] *= other.center;
		}
		return temp;
	}

	const unsigned int *id1 = this->indexes;
	const unsigned int *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	unsigned int idTemp[2*N_NOISE + 1];
	real devTemp[2*N_NOISE + 1];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;
	// for(unsigned i=0 ; i<ltemp ; i++){
	// 	printf("%d ", idTemp[i]);
	// }
	// printf("\n");
	unsigned a , b ;

	real commonTermCenter = 0.0f;
	real commonTermDeviation = 0.0f;

	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = this->center * other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = other.center * this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = other.center * this->deviations[a] + this->center * other.deviations[b];
		commonTermCenter += other.deviations[b]*this->deviations[a];
		commonTermDeviation += abs(other.deviations[b]*this->deviations[a]);
		pu1++;
		pu2++;
	}

	commonTermDeviation *= 0.5f;
	commonTermCenter *= 0.5f;

	MAF2 temp(this->center * other.center);
	temp.center += commonTermCenter;

	if (ltemp < N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		temp.nbIndexes = ltemp+1;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			temp.indexes[ind] = idTemp[ind];
			temp.deviations[ind] = devTemp[ind];
		}
		temp.deviations[ltemp] = this->getRadius() * other.getRadius() - commonTermDeviation;
		temp.indexes[ltemp] = index_last;
		index_last++;
		return temp;
	}

	devTemp[ltemp] = this->getRadius() * other.getRadius() - commonTermDeviation;
	idTemp[ltemp] = index_last;
	index_last++;
	ltemp++;

	temp.nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		temp.indexes[i] = idTemp[i];
		temp.deviations[i] = devTemp[i];
	}
	real new_noise = 0.0;
	for(uint16_t i= N_NOISE-1 ; i < ltemp; i++){
		new_noise += abs(devTemp[i]);
	}
	temp.deviations[N_NOISE-1] = new_noise;
	temp.indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE + 1];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , std::bind(compare, _1, _2, devTemp));

	// // for(uint8_t i=0 ; i<ltemp ; i++){
	// // 	printf("mult : dev : %f , indexes : %d , ind : %d \n", devTemp[i] , idTemp[i] , ind_indexes[i]);
	// // }

	// temp.nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	temp.indexes[iter] = idTemp[i];
	// 	temp.deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// temp.deviations[iter] = new_noise;
	// temp.indexes[iter] = index_last;
	// index_last++;

	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// temp.nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	temp.indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	temp.deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// temp.deviations[0] = new_noise;
	// temp.indexes[0] = index_last;
	// index_last++;
	return temp;
}

MAF2 & MAF2::operator *= (const MAF2 &other)
{
	if (this->nbIndexes + other.nbIndexes == 0){
		this->center *= other.center;
		return *this;
	}

	if(this->nbIndexes == 0){
		real t_center = this->center;
		*this =  other;
		this->center *= t_center;
		for (uint16_t i=0 ; i < other.nbIndexes ; i++){
			this->deviations[i] *= t_center;
		}
		return *this;
	}

	if(other.nbIndexes == 0){
		this->center *= other.center;
		return *this;
	}

	const unsigned int *id1 = this->indexes;
	const unsigned int *id2 = other.indexes;

	const unsigned * pu1=id1;
	const unsigned * pu2=id2;

	unsigned int idTemp[2*N_NOISE + 1];
	real devTemp[2*N_NOISE + 1];

	unsigned * fin = set_union(id1 , id1 + this->nbIndexes , id2 , id2 + other.nbIndexes , idTemp);
	unsigned int ltemp = fin - idTemp;
	// for(unsigned i=0 ; i<ltemp ; i++){
	// 	printf("%d ", idTemp[i]);
	// }
	// printf("\n");
	unsigned a , b ;

	real commonTermCenter = 0.0f;
	real commonTermDeviation = 0.0f;

	for (unsigned i = 0; i< ltemp ; i++)
	{
		a = pu1 - id1;
		b = pu2 - id2;

		if ( a == this->nbIndexes || id1[a] != idTemp[i])
		{
			devTemp[i] = this->center * other.deviations[b];
			pu2++;
			continue;
		}

		if ( b == other.nbIndexes || id2[b] != idTemp[i])
		{
			devTemp[i] = other.center * this->deviations[a];
			pu1++;
			continue;
		}

		devTemp[i] = other.center * this->deviations[a] + this->center * other.deviations[b];
		commonTermCenter += other.deviations[b]*this->deviations[a];
		commonTermDeviation += abs(other.deviations[b]*this->deviations[a]);
		pu1++;
		pu2++;
	}

	commonTermDeviation *= 0.5f;
	commonTermCenter *= 0.5f;

	this->center *= other.center;
	this->center += commonTermCenter;
	real t_radius = this->getRadius();
	if (ltemp < N_NOISE){
		// sizeof(arr)/sizeof(arr[0])
		this->nbIndexes = ltemp+1;
		for(uint16_t ind=0 ; ind < ltemp ; ind++){
			this->indexes[ind] = idTemp[ind];
			this->deviations[ind] = devTemp[ind];
		}
		this->deviations[ltemp] = t_radius * other.getRadius() - commonTermDeviation;
		this->indexes[ltemp] = index_last;
		index_last++;
		return *this;
	}

	devTemp[ltemp] = t_radius * other.getRadius() - commonTermDeviation;
	idTemp[ltemp] = index_last;
	index_last++;
	ltemp++;

	this->nbIndexes = N_NOISE;

	for(uint16_t i=0 ; i<N_NOISE-1 ; i++){
		this->indexes[i] = idTemp[i];
		this->deviations[i] = devTemp[i];
	}
	real new_noise = 0.0;
	for(uint16_t i= N_NOISE-1 ; i < ltemp; i++){
		new_noise += abs(devTemp[i]);
	}
	this->deviations[N_NOISE-1] = new_noise;
	this->indexes[N_NOISE-1] = index_last;
	index_last++;

	// unsigned int ind_indexes[2*N_NOISE + 1];
	// for (uint16_t i=0 ; i< ltemp ; i++)
	// 	ind_indexes[i] = i;
	// sort(&(ind_indexes[0]) , &(ind_indexes[0]) + ltemp , std::bind(compare, _1, _2, devTemp));

	// // for(uint8_t i=0 ; i<ltemp ; i++){
	// // 	printf("mult : dev : %f , indexes : %d , ind : %d \n", devTemp[i] , idTemp[i] , ind_indexes[i]);
	// // }

	// temp.nbIndexes = N_NOISE;

	// // Sum up smallest term and remove their index from the list of index
	// real new_noise = 0; 
	// for(uint16_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind_indexes[ind]]);
	// 	idTemp[ind_indexes[ind]] = 0;
	// }
	// uint16_t iter = 0;
	// for(uint16_t i=0 ; i< ltemp ; i++){
	// 	if(idTemp[i] == 0)
	// 		continue;
	// 	temp.indexes[iter] = idTemp[i];
	// 	temp.deviations[iter] = devTemp[i];
	// 	iter++;
	// }

	// temp.deviations[iter] = new_noise;
	// temp.indexes[iter] = index_last;
	// index_last++;

	// quickSort(devTemp , idTemp , 0 , ltemp-1);

	// temp.nbIndexes = N_NOISE;
	// for(uint8_t ind = 1; ind < N_NOISE; ind++){
	// 	temp.indexes[ind] = idTemp[ind + ltemp-N_NOISE];
	// 	temp.deviations[ind] = devTemp[ind + ltemp-N_NOISE];
	// }

	// real new_noise = 0; 
	// for(uint8_t ind = 0; ind < ltemp-N_NOISE+1; ind++){
	// 	new_noise += abs(devTemp[ind]);
	// }
	// temp.deviations[0] = new_noise;
	// temp.indexes[0] = index_last;
	// index_last++;
	return *this;
}

MAF2 MAF2::operator / (const MAF2 &other) const
{
	if (this == &other)
		return MAF2(1.0f);
	else
		return (*this) * inv(other);
}

/*MAF2 MAF2::operator ^ (const uint8_t n) const
{
	if (n == 0)
		return MAF2(1.0);
	else if (n == 1)
		return *this;

	real a , b;
	real r;

	r = this->getRadius();

	if ( r == 0){
		a = 1;
		for(uint8_t i =0 ; i< n ; i++){
			a *= this->center;
		}
		return MAF2(a);
	}

	a = this->center - r;
	b = this->center + r;

	real fa , fb;
	real alpha , dzeta , delta;

	fa = 1;
	fb = 1;
	for (uint8_t i = 0 ; i< n ; i++){
		fa *= a;
		fb *= b;
	}

	if (r > MIN_RAD){
		alpha = (fb - fa)/(b-a);
	} else {
		alpha = n*fa/(a+EPS_ZERO);
	}

	real x_1;
	if ( n == 2)
		x_1 = - abs(alpha/2);
	else
		x_1 = -pow((real) (abs(alpha/n)), (real) (1.0/(n-1)));
	real x_2 = -x_1;
	real fx_1, fx_2;
	// check valid points
	if (x_1 > a){
		// x_1 is valid
		fx_1 = 1;
		for (uint8_t i = 0 ; i< n ; i++){
			fx_1 *= x_1;
		}
	}
	else {
		x_1 = a;
		fx_1 = fa;
	}

	if (x_2 < b) {
		fx_2 = 1;
		for (uint8_t i = 0 ; i< n ; i++){
			fx_2 *= x_2;
		}
	}
	else {
		x_2 = b;
		fx_2 = fb;
	}

	double y_2 = fx_2 - alpha*x_2;
	double y_1 = fx_1 - alpha*x_1;

	delta = 0.5*(y_1 - y_2);
	dzeta = 0.5*(y_1 + y_2);

	// inverse of other
	MAF2 temp;
	temp.center = alpha * this->center + dzeta;
	for (uint8_t index = 0 ; index < N_NOISE ; index++){
		temp[index] = this->deviations[index] * alpha;
	}
	temp.err_term = this->err_term + delta;

	return temp;
}*/

MAF2 inv(const MAF2 &other)
{
	if (other.nbIndexes == 0)
		return MAF2(1.0f / other.center);

	real a , b;
	real r;

	r = other.getRadius();

	a = other.center - r;
	b = other.center + r;

	real fa , fb;
	real alpha ,dzeta , delta;

	assert_af(a*b > 0.0f);

	fa = 1/a;
	fb = 1/b;

	alpha = - fa * fb;
	if (a > 0){
		delta = (fa + fb)/2  - (1/sqrt(a*b));
		dzeta = fa + fb - delta;
	}else {
		delta = -((fa + fb)/2  + (1/sqrt(a*b)));
		dzeta = fa + fb + delta;
	}

	// inverse of other
	real min_noise = abs(other.deviations[0]);
	uint16_t index_min = 0;

	MAF2 temp;
	temp.center = alpha * other.center + dzeta;
	for (uint16_t index = 0 ; index < other.nbIndexes ; index++){
		temp.deviations[index] = other.deviations[index] * alpha;
		temp.indexes[index] = other.indexes[index];
		if (min_noise < abs(other.deviations[index])){
			index_min = index;
			min_noise = abs(other.deviations[index]);
		}
	}

	if (other.nbIndexes < N_NOISE){
		temp.nbIndexes = other.nbIndexes + 1;
		temp.deviations[other.nbIndexes] = delta;
		temp.indexes[other.nbIndexes] = MAF2::index_last;
		MAF2::index_last++;
	}else {
		temp.nbIndexes = N_NOISE;
		// move left all elements after index min
		for(uint16_t i = index_min ; i < N_NOISE-1 ; i++){
			temp.deviations[i] = temp.deviations[i+1];
			temp.indexes[i] = temp.indexes[i+1];
		}
		temp.deviations[N_NOISE-1] = delta + min_noise;
		temp.indexes[N_NOISE-1] = MAF2::index_last;
		MAF2::index_last++;
	}
	return temp;
}

MAF2 operator * (real val, const MAF2 &af)
{
	MAF2 temp(af);
	temp *= val;
	return temp;
}

MAF2 operator / (real val, const MAF2 &af)
{
	MAF2 temp(inv(af));
	temp *= val;
	return temp;
}

MAF2 operator + (real val , const MAF2 &af)
{
	MAF2 temp(af);
	temp += val;
	return temp;
}

MAF2 operator - (real val, const MAF2 &af)
{
	MAF2 temp(-af);
	temp += val;
	return temp;
}

bool MAF2::operator == (const MAF2 &af) const
{
	if (this->center != af.center)
		return false;
	if (this->nbIndexes != af.nbIndexes)
		return false;
	for (uint8_t i=0 ; i< this->nbIndexes ; i++){
		if (this->deviations[i] != af.deviations[i])
			return false;
	}
	return true;	
}

/************************************************************/
/* 	Affine form approximation of trigonometry functions  	*/
/************************************************************/


/* TODO: NEED to optimize PI/2 PI/4 2PI by pre calculating them */
MAF2 sin(const MAF2 &other)
{
	if (other.nbIndexes == 0)
		return MAF2(sin(other.center));

	real a , b;
	real r;

	r = other.getRadius();
	// assert_af( r < M_PI_VER/4); // For the moment

	/*if ( r >= 2*M_PI_VER ){
		MAF1 res;
		res.err_term = 1.0f;
		return res;
	}*/

	a = other.center - r;
	b = other.center + r;

	if (a> M_PI_VER || a < -M_PI_VER){
		real temp = floor(a / (2*M_PI_VER));
		a -= temp * 2 * M_PI_VER;
		a -= (a > M_PI_VER) ? 2*M_PI_VER : 0;
		b = a + 2*r;
	}

	real fa , fb;
	real alpha ,dzeta , delta;

	fa = sin(a);
	fb = sin(b);

	if( b < 0 || (b < M_PI_VER  && a > 0)){ // chebyshev approx in  this case
		if (r > MIN_RAD){
			alpha = (fb - fa) / (b - a);
			real sol = ((a > 0) - (a < 0)) * acos(alpha);
			real fsol = sqrt(1 - alpha*alpha); // fast computation of sin(acos)
			dzeta = (fa + fsol - alpha * (a + sol)) / 2;
			delta = abs(fsol - fa - alpha * (sol - a)) / 2;
		}
		else{
			alpha = cos(a);
			dzeta = fa - alpha*a;
			delta = 0.0f;
		}

	} else { // min range optimization since the derivative cant be zero because of the radius constraints
		if ( a <= 0 ){
			alpha = 1.0f;
			delta = (-fb + fa - alpha * (a - b)) / 2;
		} else {
			alpha = -1.0f;
			delta = (fb - fa + alpha * (a - b)) / 2;
		}
		dzeta = (fa + fb - alpha * (a + b)) / 2;
	}

	MAF2 temp;
	temp.nbIndexes = other.nbIndexes;
	temp.center = alpha * other.center + dzeta;
	for (uint8_t index = 0 ; index < other.nbIndexes ; index++){
		temp.deviations[index] = other[index] * alpha;
		temp.indexes[index] = other.indexes[index];
	}
	if (other.nbIndexes < N_NOISE){
		temp.nbIndexes++;
		temp.deviations[other.nbIndexes] = delta;
		temp.indexes[other.nbIndexes] = MAF2::index_last;
		MAF2::index_last++;
		return temp;
	}
	// We fuse the last symbol with this error
	temp.deviations[N_NOISE-1] = abs(temp.deviations[other.nbIndexes-1]) + delta;
	temp.indexes[N_NOISE-1] = MAF2::index_last;
	MAF2::index_last++;

	return temp;
}

MAF2 cos(const MAF2 &other)
{
	return sin(other + M_PI_VER/2);
}

MAF2 tan(const MAF2 &other)
{

	if (other.nbIndexes == 0)
		return MAF2(tan(other.center));

	real a , b;
	real r;

	r = other.getRadius();

	a = other.center - r;
	b = other.center + r;

	// In case the angles are not between -PI/2 and PI/2
	if (a>= M_PI_VER/2 || a <= -M_PI_VER/2){
		real temp = floor(a / (M_PI_VER));
		a -= temp *  M_PI_VER;
		a -= (a > (M_PI_VER/2)) ? M_PI_VER : 0;
		b = a + 2*r;
	}

	assert_af(! ( (a <= M_PI_VER/2 && b >= M_PI_VER/2 ) || (a <= -M_PI_VER/2 && b >= -M_PI_VER/2 ) ));

	real fa , fb;
	real alpha ,dzeta , delta;

	fa = tan(a);
	fb = tan(b);

	if (r > MIN_RAD)
		alpha = (fb - fa) / (b-a);
	else
		alpha =  1 + fa * fb;

	real aux = sqrt(alpha - 1);
	real eps = atan(aux);
	// real eps2 = -eps1;

	if ( eps >= a && eps <= b && -eps >= a && -eps <= b){
		dzeta = 0;
		delta = abs(aux - alpha * eps);
	} else {
		if (-eps >= a && -eps <= b){
			eps = -eps;
			aux = -aux;
		}
		dzeta = (fa + aux - alpha*(a + eps)) / 2;
		delta = abs(aux - fa - alpha * (eps - a)) / 2;
	}

	MAF2 temp;
	temp.nbIndexes = other.nbIndexes;
	temp.center = alpha * other.center + dzeta;
	for (uint8_t index = 0 ; index < other.nbIndexes ; index++){
		temp.deviations[index] = other[index] * alpha;
		temp.indexes[index] = other.indexes[index];
	}
	if (other.nbIndexes < N_NOISE){
		temp.nbIndexes++;
		temp.deviations[other.nbIndexes] = delta;
		temp.indexes[other.nbIndexes] = MAF2::index_last;
		MAF2::index_last++;
		return temp;
	}
	// We fuse the last symbol with this error
	temp.deviations[N_NOISE-1] = abs(temp.deviations[other.nbIndexes-1]) + delta;
	temp.indexes[N_NOISE-1] = MAF2::index_last;
	MAF2::index_last++;

	return temp;

}

/* TODO: NEED to optimize PI/2 PI/4 2PI by pre calculating them */
// MAF2 sin(const MAF2 &other)
// {
// 	real a , b;
// 	real r;

// 	r = other.getRadius();

// 	if (r == 0)
// 		return MAF2(sin(other.center));

// 	assert_af( r < M_PI_VER/4); // For the moment

// 	/*if ( r >= 2*M_PI_VER ){
// 		MAF2 res;
// 		res.err_term = 1.0f;
// 		return res;
// 	}*/

// 	a = other.center - r;
// 	b = other.center + r;

// 	if (a> M_PI_VER || a < -M_PI_VER){
// 		real temp = floor(a / (2*M_PI_VER));
// 		a -= temp * 2 * M_PI_VER;
// 		a -= (a > M_PI_VER) ? 2*M_PI_VER : 0;
// 		b = a + 2*r;
// 	}

// 	real fa , fb;
// 	real alpha ,dzeta , delta;

// 	fa = sin(a);
// 	fb = sin(b);

// 	if( b < 0 || (b < M_PI_VER  && a > 0)){ // chebyshev approx in  this case
// 		if (r > MIN_RAD){
// 			alpha = (fb - fa) / (b - a);
// 			real sol = ((a > 0) - (a < 0)) * acos(alpha);
// 			real fsol = sqrt(1 - alpha*alpha); // fast computation of sin(acos)
// 			dzeta = (fa + fsol - alpha * (a + sol)) / 2;
// 			delta = abs(fsol - fa - alpha * (sol - a)) / 2;
// 		}
// 		else{
// 			alpha = cos(a);
// 			dzeta = fa - alpha*a;
// 			delta = 0.0f;
// 		}

// 	} else { // min range optimization since the derivative cant be zero because of the radius constraints
// 		if ( a <= 0 ){
// 			alpha = 1.0f;
// 			delta = (-fb + fa - alpha * (a - b)) / 2;
// 		} else {
// 			alpha = -1.0f;
// 			delta = (fb - fa + alpha * (a - b)) / 2;
// 		}
// 		dzeta = (fa + fb - alpha * (a + b)) / 2;
// 	}

// 	MAF2 temp;
// 	temp.center = alpha * other.center + dzeta;
// 	for (uint8_t index = 0 ; index < N_NOISE ; index++){
// 		temp[index] = other[index] * alpha;
// 	}
// 	temp.err_term = other.err_term + delta;

// 	return temp;
// }

// MAF2 cos(const MAF2 &other)
// {
// 	return sin(other + M_PI_VER/2);
// }

// MAF2 tan(const MAF2 &other)
// {
// 	real a , b;
// 	real r;

// 	r = other.getRadius();

// 	if (r == 0)
// 		return MAF2(tan(other.center));

// 	a = other.center - r;
// 	b = other.center + r;

// 	// In case the angles are not between -PI/2 and PI/2
// 	if (a>= M_PI_VER/2 || a <= -M_PI_VER/2){
// 		real temp = floor(a / (M_PI_VER));
// 		a -= temp *  M_PI_VER;
// 		a -= (a > (M_PI_VER/2)) ? M_PI_VER : 0;
// 		b = a + 2*r;
// 	}

// 	assert_af(! ( (a <= M_PI_VER/2 && b >= M_PI_VER/2 ) || (a <= -M_PI_VER/2 && b >= -M_PI_VER/2 ) ));

// 	real fa , fb;
// 	real alpha ,dzeta , delta;

// 	fa = tan(a);
// 	fb = tan(b);

// 	if (r > MIN_RAD)
// 		alpha = (fb - fa) / (b-a);
// 	else
// 		alpha =  1 + fa * fb;

// 	real aux = sqrt(alpha - 1.0);
// 	real eps = atan(aux);
// 	// real eps2 = -eps1;

// 	if ( eps >= a && eps <= b && -eps >= a && -eps <= b){
// 		dzeta = 0;
// 		delta = abs(aux - alpha * eps);
// 	} else {
// 		if (-eps >= a && -eps <= b){
// 			eps = -eps;
// 			aux = -aux;
// 		}
// 		dzeta = (fa + aux - alpha*(a + eps)) / 2;
// 		delta = abs(aux - fa - alpha * (eps - a)) / 2;
// 	}

// 	MAF2 temp;
// 	temp.center = alpha * other.center + dzeta;
// 	for (uint8_t index = 0 ; index < N_NOISE ; index++){
// 		temp[index] = other[index] * alpha;
// 	}
// 	temp.err_term = other.err_term + delta;

// 	return temp;
// }

/* Useful functions */
// int partition (real arr[], unsigned int indexes[], int low, int high)
// {
// 	real pivot = abs(arr[high]);    // pivot
// 	int i = (low - 1);  // Index of smaller element
// 	for (int j = low; j <= high-1; j++)
// 	{
// 		// If current element is smaller than or
// 		// equal to pivot
// 		if (abs(arr[j]) <= pivot)
// 		{	
// 			i++;    // increment index of smaller element
// 			swap(arr[i], arr[j]);
// 			swap(indexes[i], indexes[j]);
// 		}
// 	}
// 	swap(arr[i + 1], arr[high]);
// 	swap(indexes[i + 1], indexes[high]);
// 	return (i + 1);
// }

// void quickSort(real arr[], unsigned int indexes[], int low, int high)
// {
// 	if (low < high)
// 	{
// 		/* pi is partitioning index, arr[p] is now at right place */
// 		int pi = partition(arr, indexes, low, high);
// 		// Separately sort elements before
// 		// partition and after partition
// 		quickSort(arr, indexes, low, pi - 1);
// 		quickSort(arr, indexes, pi + 1, high);
// 	}
// }