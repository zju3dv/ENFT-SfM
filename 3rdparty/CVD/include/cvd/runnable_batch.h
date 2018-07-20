#ifndef CVD_INCLUDE_RUNNABLE_BATCH_H
#define  CVD_INCLUDE_RUNNABLE_BATCH_H

#include <cvd/runnable.h>
#include <cvd/internal/shared_ptr.h>
#include <vector>
namespace CVD
{
	class Thread;

//Forward declaration
template<class C> class MessageQueue;

///This class provides a simple job manager for 
///tasks derived from CVD:Runnable. The class operates using a small pool of
///threads, so new tasks are run in existing free threads, not new ones. Overhead will
///only be due to locking in MessageQueue and shared_ptr.
class RunnableBatch
{
	private:
		bool joined;
		unsigned int  parallelism;

		class RunMessageInThread;

		std::vector<CVD::STD::shared_ptr<RunMessageInThread> > threads;
		CVD::STD::shared_ptr<MessageQueue<CVD::STD::shared_ptr<Runnable> > > queue;
	

	public:
		///Construct a job manager
		///@parm pallelism Number of threads to use. Note that 1 thread will cause 
		///                jobs to be executed in one new thread. 0 threads will
		///                cause the jobs to be executed in the current thread. This
		///                will prevent thread spawning and should make debugging easier.
		///                This is the behaviour if CVD is compiled without threading
		///                support.
		RunnableBatch(unsigned int p);
				
		///Wait until all pending tasks have been run, then terminate all 
		///threads.
		void join();
		
		///Put a task on the queue. This will be run at some point
		///in the future. Job lifetime is managed by shared_ptr, so they
		///may be managed wither by RunnableBatch or by the caller.
		void schedule(CVD::STD::shared_ptr<Runnable> r);
	
		///Destruct the job manager. This will wait for all threads
		///to finish and terminate.
		~RunnableBatch();
};

}
#endif
