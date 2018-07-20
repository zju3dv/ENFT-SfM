//-*- c++ -*-
#ifndef __IRLS_H
#define __IRLS_H

#include <cvd/wls.h>
#include <assert.h>
#include <cmath>

namespace CVD {

/// Robust reweighting (type I) for IRLS.
/// A reweighting class with \f$w(x)=\frac{1}{\sigma + |x|}\f$.
/// This structure can be passed as the second template argument in IRLS.
/// @ingroup gMaths
struct RobustI {
  double sd_inlier; ///< The inlier standard deviation, \f$\sigma\f$.
  inline double reweight(double x) {return 1/(sd_inlier+fabs(x));}  ///< Returns \f$w(x)\f$.
  inline double true_scale(double x) {return reweight(x) - fabs(x)*reweight(x)*reweight(x);}  ///< Returns \f$w(x) + xw'(x)\f$.
  inline double objective(double x) {return fabs(x) + sd_inlier*log(sd_inlier*reweight(x));}  ///< Returns \f$\int xw(x)dx\f$.
};

/// Robust reweighting (type II) for IRLS.
/// A reweighting class with \f$w(x)=\frac{1}{\sigma + x^2}\f$.
/// This structure can be passed as the second template argument in IRLS.
/// @ingroup gMaths
struct RobustII {
  double sd_inlier; ///< The inlier standard deviation, \f$\sigma\f$.
  inline double reweight(double d){return 1/(sd_inlier+d*d);} ///< Returns \f$w(x)\f$.
  inline double true_scale(double d){return d - 2*d*reweight(d);} ///< Returns \f$w(x) + xw'(x)\f$.
  inline double objective(double d){return 0.5 * log(1 + d*d/sd_inlier);} ///< Returns \f$\int xw(x)dx\f$.
};

/// A reweighting class representing no reweighting in IRLS.
/// \f$w(x)=1\f$
/// This structure can be passed as the second template argument in IRLS.
/// @ingroup gMaths
struct ILinear {
  inline double reweight(double d){return 1;} ///< Returns \f$w(x)\f$.
  inline double true_scale(double d){return 1;} ///< Returns \f$w(x) + xw'(x)\f$.
  inline double objective(double d){return d*d;} ///< Returns \f$\int xw(x)dx\f$.
};


/// Performs iterative reweighted least squares.
/// @param Size the size
/// @param Reweight The reweighting functor. This structure must provide reweight(), 
/// true-scale() and objective() methods. Existing examples are  Robust I, Robust II and ILinear.
/// @ingroup gMaths
template <int Size, class Reweight>
class IRLS
  : public Reweight,
    public WLS<Size>
{
public:
  IRLS(){Identity(my_true_C_inv,0);my_residual=0;}

  inline void add_df(double d, const Vector<Size>& f) {
    double scale = reweight(d);
    double ts = true_scale(d);
    my_residual += objective(d);

    WLS<Size>::add_df(d,f,scale);

    for(int i=0; i<Size; i++){
      for(int j=0; j<Size; j++){
	my_true_C_inv[i][j]+=f[i]*f[j]*ts;
      }
    }
  }

  void operator += (const IRLS& meas){
    WLS<Size>::operator+=(meas);
    my_true_C_inv += meas.my_true_C_inv;
  }


  Matrix<Size,Size,RowMajor>& get_true_C_inv() {return my_true_C_inv;}
  const Matrix<Size,Size,RowMajor>& get_true_C_inv()const {return my_true_C_inv;}

  double get_residual() {return my_residual;}

private:

  double my_residual;

  Matrix<Size,Size,RowMajor> my_true_C_inv;

  // comment out to allow bitwise copying
  IRLS( IRLS& copyof );
  int operator = ( IRLS& copyof );
};

}

#endif
