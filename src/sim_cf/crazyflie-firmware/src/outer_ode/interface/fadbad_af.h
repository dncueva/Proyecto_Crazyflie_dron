#ifndef FADBAD_AF_H
#define FADBAD_AF_H

#include "config_outer.h"
#include "tadiff.h"

using namespace fadbad;

namespace fadbad
{

template <> struct Op<Interval>
{
    
    typedef Interval Base;
    
    static Base myInteger(const int i) { return Base(double(i)); }
    static Base myZero() { return myInteger(0); }
    static Base myOne() { return myInteger(1);}
    static Base myTwo() { return myInteger(2); }
    
    static Base myPos(const Base& x) { return x; }
    static Base myNeg(const Base& x) { return Base(-x.getMax(),-x.getMin()); }
    
    static Base& myCadd(Base& x, const Base& y) { return x+=y; }
    static Base& myCsub(Base& x, const Base& y) { return x-=y; }
    static Base& myCmul(Base& x, const Base& y) { return x*=y; }
    static Base& myCdiv(Base& x, const Base& y) { return x/=y; }

    static Base myInv(const Base& x) { return myOne()/x; }
    
    static Base mySqr(const Base& x) { assert_af(false); return myInteger(0);}// sqr(x); }
    static Base myPow(const Base& x, const int n) { assert_af(false); return myInteger(0);}
    static Base myPow(const Base& x, const Base& y) { assert_af(false); return myInteger(0);}
    
    static Base mySqrt(const Base& x) { assert_af(false); return myInteger(0);}
    static Base myLog(const Base& x) { assert_af(false); return myInteger(0);}
    static Base myExp(const Base& x) { assert_af(false); return myInteger(0);}
    static Base mySin(const Base& x) { assert_af(false); return myInteger(0); }
    static Base myCos(const Base& x) { assert_af(false); return myInteger(0); }
    static Base myTan(const Base& x) { assert_af(false); return myInteger(0); }
    static Base myAsin(const Base& x) { assert_af(false); return myInteger(0);}
    static Base myAcos(const Base& x) { assert_af(false); return myInteger(0);}
    static Base myAtan(const Base& x) { assert_af(false); return myInteger(0);}
    
    static bool myEq(const Base& x, const Base& y) { return x==y; }
    static bool myNe(const Base& x, const Base& y) { return !(x==y); }
};


template <> struct Op<AF1>
{
  
  typedef AF1 Base;

  static Base myInteger(const int i) { return Base(double(i)); }
  static Base myZero() { return myInteger(0); }
  static Base myOne() { return myInteger(1);}
  static Base myTwo() { return myInteger(2); }

  static Base myPos(const Base& x) { return  x; }
  static Base myNeg(const Base& x) { return  -x;  }
  
  static Base& myCadd(Base& x, const Base& y) { return x+=y;   }
  static Base& myCsub(Base& x, const Base& y) { return x-=y;   }
  static Base& myCmul(Base& x, const Base& y) { return x*=y;  }
  static Base& myCdiv(Base& x, const Base& y) { return (x= x/y); }

  static Base myInv(const Base& x) { return inv(x);  }

  static Base mySqr(const Base& x) { return x*x; }
  static Base myPow(const Base& x, const int n) { assert_af(false); return myInteger(0); }
  static Base myPow(const Base& x, const Base& y) { assert_af(false); return myInteger(0); }

  static Base mySqrt(const Base& x) { assert_af(false); return myInteger(0);  }
  static Base myLog(const Base& x) { assert_af(false); return myInteger(0);  }
  static Base myExp(const Base& x) { assert_af(false); return myInteger(0);  }
  static Base mySin(const Base& x) { return sin(x);  }
  static Base myCos(const Base& x) { return cos(x);  }
  static Base myTan(const Base& x) { return tan(x);  }
  static Base myAsin(const Base& x) { assert_af(false); return myInteger(0);}
  static Base myAcos(const Base& x) { assert_af(false); return myInteger(0); }
  static Base myAtan(const Base& x) { assert_af(false); return myInteger(0); }
  
  /*  static Base myPos(const Base& x) { return +x; }
  static bool myNe(const Base& x, const Base& y) { return x!=y; }
  */ 
  static bool myEq(const Base& x, const Base& y) { return x==y; }
  static bool myNe(const Base& x, const Base& y) { return !(x==y); }
};

}

#endif
