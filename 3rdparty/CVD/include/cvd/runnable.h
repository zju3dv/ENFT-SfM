#ifndef RUNNABLE_H_
#define RUNNABLE_H_

namespace CVD {
//! This is an abstract base class for anything with a run() method.
class Runnable 
{
 public:
  //! Perform the function of this object.
   virtual void run()=0;
   virtual ~Runnable(){};
};

}
#endif
