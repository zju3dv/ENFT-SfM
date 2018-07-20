#ifndef CVD_INCLUDE_MESSAGE_QUEUE_H
#define CVD_INCLUDE_MESSAGE_QUEUE_H

#include <cvd/synchronized.h>
#include <semaphore.h>
#include <deque>

namespace CVD
{

///This class provides a simple, thread safe FIFO message queue.
template<class C> class MessageQueue
{
	public:
		///Construct a message queue
		MessageQueue()
		{
			sem_init(&empty_slots, 0, 0);
		}

		///Destruct a message queue
		~MessageQueue()
		{
			sem_destroy(&empty_slots);
		}

		///Write a message to the queue.
		///@param message The message to write to the queue
		void write(const C& message)
		{	
			//Lock the queue, so it can be safely used.
			queue_mutex.lock();
			queue.push_back(message);
			queue_mutex.unlock();

			sem_post(&empty_slots);
		}
		
		///Read a message from the queue.
		///Wait if the queue is empty.
		///@return  The message read from the queue
		C read()
		{
			sem_wait(&empty_slots);
			C ret;

			queue_mutex.lock();
			ret = queue.front();
			queue.pop_front();
			queue_mutex.unlock();

			return ret;
		}

		int size()
		{
			int s;
			sem_getvalue(&empty_slots, &s);
			return s;
		}

	private:
		Synchronized queue_mutex;
		std::deque<C> queue;
		sem_t empty_slots;
};

}
#endif
