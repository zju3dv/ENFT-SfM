#ifndef THREAD_H_
#define THREAD_H_

//POSIX threads
#include <pthread.h>
#include <cvd/runnable.h>

namespace CVD {
/** The Thread class encapsulates a thread of execution.  It is implemented with POSIX threads.
    Code that uses this class should link with libpthread and librt (for nanosleep).
*/

class Thread : public Runnable
{
 public:
   //! Construct a thread.  If runnable != 0, use that runnable, else use our own "run" method.
   Thread();

   //! This does not destroy the object until the thread has been terminated.
   virtual ~Thread();

   //! Start execution of "run" method in separate thread.
   void start(Runnable* runnable=0);
   
   //! Tell the thread to stop.  
   /** This doesn't make the thread actually stop, it just causes shouldStop() to return true. */
   void stop();
   
   //! Returns true if the stop() method been called, false otherwise.
   bool shouldStop() const;
   
   //! Returns true if the thread is still running.
   bool isRunning() const;

   //! This blocks until the thread has actually terminated.  
   /** If the thread is infinite looping, this will block forever! */
   void join();
   
   //! Get the ID of this thread.
   pthread_t getID();
   
   //! Override this method to do whatever it is the thread should do.
   virtual void run(){};
   
   //Static methods:
   
   //! Returns how many threads are actually running, not including the main thread.
   static unsigned int count();
   
   //! Returns a pointer to the currently running thread.
   static Thread* getCurrent();
   
   //! Tell the current thread to sleep for milli milliseconds
   static void sleep(unsigned int milli);

   //! Tell the current thread to yield the processor.
   static void yield();

 private:
   static bool init();
   static bool ourInitializedFlag;
   static void* threadproc(void* param);
   static pthread_key_t ourKey;
   static unsigned int ourCount;
   Runnable* myRunnable;
   pthread_t myID;
   bool myRunningFlag;
   bool myStopFlag;
};

}
#endif
