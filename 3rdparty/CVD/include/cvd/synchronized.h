#ifndef SYNCHRONIZED_H_
#define SYNCHRONIZED_H_

// POSIX threads
#include <pthread.h>

#include <cvd/nocopy.h>

namespace CVD {

/** A Synchronized object encapsulates a basic mutex.
    Only one thread can own the lock at a time.
    Classes should subclass from Synchronized if they
    want to be able to lock themselves.
*/
class Synchronized : private NoCopy
{
 public:
  //! Create an initially unlocked mutex
  Synchronized();
  //! The lock is acquired before deconstruction.
  virtual ~Synchronized();
  //! Acquire the lock; this blocks until the lock is available.

  //! This can be called multiple times by the same thread without deadlocking.
  //! Each call should have a matching unlock() call.
  void lock() const;
   
  //! Release the lock.
  void unlock() const;
   
 protected:
   pthread_mutexattr_t     myAttr;
   mutable pthread_mutex_t myMutex;
};

/** A utility class for locking and unlocking Synchronized objects automatically.
    A Lock object should be declared on the stack in the same scope as the object to be locked.
    When the Lock object goes out of scope, the mutex is released.  This is especially useful
    in code that throws exceptions. */
class Lock : private NoCopy {
public:

    explicit Lock(const Synchronized& obj)
    : myObject(obj) {
        myObject.lock();
    }

    ~Lock() {
        myObject.unlock();
    }

private:

    const Synchronized& myObject;
};

}  
#endif
