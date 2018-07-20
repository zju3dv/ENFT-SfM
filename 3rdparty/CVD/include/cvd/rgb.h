//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  rgb.h                                                               //
//                                                                      //
//  Contains definitions of Rgb template class                          //
//                                                                      //
//  derived from IPRS_* developed by Tom Drummond                       //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
#ifndef CVD_RGB_H
#define CVD_RGB_H

#include <iostream>

#include <cvd/byte.h>

#include <cvd/internal/is_pod.h>

namespace CVD {

  /// A colour consisting of red, green and blue components.
/// Often used to store 24-bit colour information, i.e. <code>CVD::Rgb<CVD::byte></code>
/// @param T The datatype of each component
/// @ingroup gImage
template <class T>
class Rgb
{
public:
  /// Default constructor. Sets everything to 0.
  explicit Rgb() : red(0), green(0), blue(0) {}

  /// Constructs a colour as specified
  /// @param r The red component
  /// @param g The green component
  /// @param b The blue component
  explicit inline Rgb(T r, T g, T b) : red(r),green(g),blue(b) {}

  template <class S> inline explicit Rgb(const Rgb<S>& rgb) : red(static_cast<T>(rgb.red)), green(static_cast<T>(rgb.green)), blue(static_cast<T>(rgb.blue)) {}

  T red;   ///< The red component
  T green; ///< The green component
  T blue;  ///< The blue component
   
  /// Assignment operator
  /// @param c The colour to copy from
    inline Rgb<T>& operator=(const Rgb<T>& c)
    {red = c.red; green = c.green; blue = c.blue; return *this;}
	    
  /// Logical equals operator. Returns true if each component is the same.
  /// @param c Rgb to compare with
  inline bool operator==(const Rgb<T>& c) const
  {return red == c.red && green == c.green && blue == c.blue;}
	      
  /// Logical not-equals operator. Returns true unless each component is the same.
  /// @param c Rgb to compare with
  inline bool operator!=(const Rgb<T>& c) const
  {return red != c.red || green != c.green || blue != c.blue;}

  /// Assignment operator between two different storage types, using the standard casts as necessary
  /// @param c The colour to copy from
  template <class T2>
  inline Rgb<T>& operator=(const Rgb<T2>& c){
    red = static_cast<T>(c.red);
    green = static_cast<T>(c.green);
    blue = static_cast<T>(c.blue);
    return *this;
  }

  //   T to_grey() {return 0.3*red + 0.6*green + 0.1*blue;}
};
  
  /// Write the colour to a stream in the format "(red,green,blue)"
/// @param os The stream
/// @param x The colour object
/// @relates Rgb
template <class T>
std::ostream& operator <<(std::ostream& os, const Rgb<T>& x)
{
  return os << "(" << x.red << "," << x.green << ","
	    << x.blue << ")";
}

  /// Read a colour from a stream, interpreting three numbers as <code>char</code>s
/// @param os The stream
/// @param x The colour object
/// @relates Rgb
inline std::ostream& operator <<(std::ostream& os, const Rgb<char>& x)
{
  return os << "(" << (int)(unsigned char)x.red << ","
	    << (int)(unsigned char)x.green << ","
	    << (int)(unsigned char)x.blue << ")";
}

  /// Read a colour from a stream, interpreting three numbers as <code>byte</code>s
/// @param os The stream
/// @param x The colour object
/// @relates Rgb
inline std::ostream& operator <<(std::ostream& os, const Rgb<byte>& x)
{
  return os << "(" << static_cast<int>(x.red) << ","
	    << static_cast<int>(x.green) << ","
	    << static_cast<int>(x.blue) << ")";
}

#ifndef DOXYGEN_IGNORE_INTERNAL
namespace Internal
{
  template<class C> struct is_POD<Rgb<C> >
  {
    enum { is_pod = is_POD<C>::is_pod };
  };
}
#endif


} // end namespace
#endif
