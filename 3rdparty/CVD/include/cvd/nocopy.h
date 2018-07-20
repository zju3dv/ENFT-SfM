
#ifndef CVD_INCLUDE_NOCOPY_H
#define CVD_INCLUDE_NOCOPY_H

namespace CVD {

/** Conveniently block the copy constructor and assignment operator of subclasses. */
class NoCopy {
protected:

    NoCopy() {}
   ~NoCopy() {}

private:

   NoCopy(const NoCopy &);
   const NoCopy & operator=(const NoCopy &);
};

}

#endif /* CVD_INCLUDE_NOCOPY_H */
