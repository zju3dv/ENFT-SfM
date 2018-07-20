inline ImageRef::ImageRef()
{
	x=y=0;
}

inline ImageRef::ImageRef(int xp, int yp)
:x(xp),y(yp)
{}

inline ImageRef::ImageRef(std::istream& is)
{
	is.read((char*)&x,sizeof(int));
	is.read((char*)&y,sizeof(int));
}

	//////////////////////////////////////////////////////////////////////////
	// the following cryptic pieces of rubbish are because inline functions //
	// must have their one and only return function as the last call        //
	// so this code makes use of the fact that expressions to the right     //
	// of || are only evaluated when the left hand side is false            //
	//////////////////////////////////////////////////////////////////////////

inline bool ImageRef::next(const ImageRef& max)	// move on to the next value
{
	return(++x < max.x || (x=0, ++y < max.y) || (y=0, false));
}

inline bool ImageRef::next(const ImageRef& min, const ImageRef& max)
{
	return (++x < max.x || (x=min.x, ++y < max.y) || (y=min.y, false));
}

inline bool ImageRef::prev(const ImageRef& max)	// move back to the previous value
{
	return(--x > -1 || (x=max.x-1, --y > -1) || (y=max.y-1, false));
}

inline bool ImageRef::prev(const ImageRef& min, const ImageRef& max)
{
	return (--x > min.x-1 || (x=max.x-1, --y > min.y-1) || (y=max.y-1, false));
}

inline void ImageRef::home()
{
	x=y=0;
}

inline void ImageRef::end(const ImageRef& size)
{
	x=size.x-1;
	y=size.y-1;
}

inline ImageRef& ImageRef::operator=(const ImageRef& ref)
{
	x=ref.x;
	y=ref.y;
	return *this;
}

inline bool ImageRef::operator ==(const ImageRef& ref) const
{
	return (x==ref.x && y==ref.y);
}

inline bool ImageRef::operator !=(const ImageRef& ref) const
{
	return (x!=ref.x || y!=ref.y);
}

inline ImageRef ImageRef::operator-() const
{
  ImageRef v(-x, -y);
  return v;
}

inline ImageRef& ImageRef::operator*=(const double scale)
{
  x=(int)(x*scale);
  y=(int)(y*scale);
  return *this;
}

inline ImageRef& ImageRef::operator/=(const double scale)
{
	x=(int)(x/scale);
	y=(int)(y/scale);
	return *this;
}

inline ImageRef& ImageRef::operator+=(const ImageRef rhs)
{
	x+=rhs.x;
	y+=rhs.y;
	return *this;
}

inline ImageRef& ImageRef::operator-=(const ImageRef rhs)
{
	x-=rhs.x;
	y-=rhs.y;
	return *this;
}

inline ImageRef ImageRef::operator*(const double scale) const
{
	ImageRef v((int)(x*scale),(int)(y*scale));
	return v;
}

inline ImageRef ImageRef::operator/(const double scale) const
{
	ImageRef v((int)(x/scale),(int)(y/scale));
	return v;
}

inline ImageRef ImageRef::operator+(const ImageRef rhs) const
{
	ImageRef v(x+rhs.x, y+rhs.y);
	return v;
}

inline ImageRef ImageRef::operator-(const ImageRef rhs) const
{
	ImageRef v(x-rhs.x, y-rhs.y);
	return v;
}

inline ImageRef& ImageRef::operator<<=(int i)
{
	x = x << i;
	y=y << i;
	return *this;
}

inline ImageRef& ImageRef::operator>>=(int i)
{
	x = x >> i;
	y=y >> i;
	return *this;
}

inline ImageRef ImageRef::shiftl(int i) const
{
	ImageRef result;
	result.x = x << i;
	result.y=y << i;
	return result;
}

inline ImageRef ImageRef::shiftr(int i) const
{
	ImageRef result;
	result.x = x >> i;
	result.y=y >> i;
	return result;
}

inline ImageRef ImageRef::operator<<(int i) const
{
	return shiftl(i);
}

inline ImageRef ImageRef::operator>>(int i) const
{
	return shiftr(i);
}


inline ImageRef operator*(const int scale, const ImageRef&  ref)
{
	return ImageRef(ref.x*scale, ref.y*scale);
}

inline bool ImageRef::operator<(const ImageRef & other) const
{
  return y < other.y || ( y == other.y && x < other.x);
}

inline bool ImageRef::operator>(const ImageRef & other) const
{
  return y > other.y || ( y == other.y && x > other.x);
}


inline int& ImageRef::operator[](int i)
{
  if(i==0)
    return x;
  if(i==1)
    return y;
  throw Exceptions::BadSubscript();
}

inline int ImageRef::operator[](int i) const
{
  if(i==0)
    return x;
  if(i==1)
    return y;
  throw Exceptions::BadSubscript();
}

inline unsigned int ImageRef::mag_squared() const
{
  typedef unsigned int uint;
  return uint(x*x) + uint(y*y);
}

inline int ImageRef::area() const
{
  return x * y;
}

inline ImageRef ImageRef::dot_times(const ImageRef &ref) const
{
  return ImageRef(x * ref.x, y * ref.y);
}
