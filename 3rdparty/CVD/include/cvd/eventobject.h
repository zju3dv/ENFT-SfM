#ifndef EVENTOBJECT_H
#define EVENTOBJECT_H

#include <pthread.h>
#include <cvd/synchronized.h>

namespace CVD {
    //! Encapsulation of a condition variable and its boolean condition.
    class EventObject : public Synchronized
    {
    public:
	//! Construct an initially untriggered event.
	EventObject();
	virtual ~EventObject();
	//! Set the condition boolean to true and wake up one thread waiting on the event.
	void trigger();
	//! Set the condition boolean to true and wake up all threads waiting on the event.
	void triggerAll();

	//! Block until the condition is true.  Reset the condition before returning.
	void wait();
	//! Block until the condition is true or the specified timeout has elapsed.  Reset the condition before returning if the condition became true.
	bool wait(unsigned int milli);
    protected:
	pthread_cond_t myCondVar;
    };
    
}
#endif
