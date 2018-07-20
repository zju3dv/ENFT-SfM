////////////////////////////////////////////////////////////////////////////
//  Copyright 2017-2018 Computer Vision Group of State Key Lab at CAD&CG, 
//  Zhejiang University. All Rights Reserved.
//
//  For more information see <https://github.com/ZJUCVG/ENFT-SfM>
//  If you use this code, please cite the corresponding publications as 
//  listed on the above website.
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes only.
//  Any modification based on this work must be open source and prohibited
//  for commercial use.
//  You must retain, in the source form of any derivative works that you 
//  distribute, all copyright, patent, trademark, and attribution notices 
//  from the source form of this work.
//   
//
////////////////////////////////////////////////////////////////////////////

#ifndef _MUTEX_H_
#define _MUTEX_H_

#include <list>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/xtime.hpp>

#define SIGNAL_FLAG_DEFAULT		0
#define SIGNAL_FLAG_WAIT		1
#define SIGNAL_FLAG_RESET		2

template<class Data>
class MutexBuffer
{
public:

	MutexBuffer(const uint &bufferSize = 1) : m_bufferSize(bufferSize) {}

	virtual void Push(Data *data) = 0;
	virtual bool PushNoWaiting(Data *data) = 0;
	virtual Data* Pop() = 0;
	virtual Data* PopNoWaiting() = 0;
	virtual void WaitWhileEmpty() = 0;
	virtual void WaitWhileNotFull() = 0;
	virtual uint Size() = 0;

protected:

	uint m_bufferSize;
	boost::mutex m_mutex;
};

template<class Data>
class MutexBufferPool : public MutexBuffer<Data>
{
public:

	MutexBufferPool(const uint &bufferSize = 1) : MutexBuffer<Data>(bufferSize)
	{
		Data *data;
		for(uint i = 0; i < bufferSize; ++i)
		{
			data = new Data();
			m_dataPool.push_back(data);
			m_dataAvailable.push_back(data);
		}
	}
	~MutexBufferPool()
	{
		for(typename std::list<Data *>::iterator it = m_dataPool.begin(); it != m_dataPool.end(); ++it)
			delete *it;
	}

	virtual void Push(Data *data)
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		m_dataAvailable.push_back(data);
		m_conditionEmpty.notify_one();
	}

	virtual bool PushNoWaiting(Data *data)
	{
		Push(data);
		return true;
	}
	
	virtual Data* Pop()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(m_dataAvailable.empty())
			m_conditionEmpty.wait(lock);
		Data *data = m_dataAvailable.back();
		m_dataAvailable.pop_back();
		return data;
	}

	virtual Data* PopNoWaiting()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		if(m_dataAvailable.empty())
			return NULL;
		Data *data = m_dataAvailable.back();
		m_dataAvailable.pop_back();
		return data;
	}

	virtual void WaitWhileEmpty()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(m_dataAvailable.empty())
			m_conditionEmpty.wait(lock);
	}

	virtual void WaitWhileNotFull()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(uint(m_dataAvailable.size()) < this->m_bufferSize)
			m_conditionEmpty.wait(lock);
	}

	virtual uint Size()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		return uint(m_dataAvailable.size());
	}

protected:

	boost::condition m_conditionEmpty;
	std::list<Data *> m_dataPool, m_dataAvailable;
};

template<class Data>
class MutexBufferQueue : public MutexBuffer<Data>
{
public:

	MutexBufferQueue(const uint &bufferSize = 1) : MutexBuffer<Data>(bufferSize) {}

	virtual void Push(Data *data)
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(this->m_data.size() == this->m_bufferSize)
			m_conditionFull.wait(lock);
		m_data.push(data);
		m_conditionEmpty.notify_one();
	}

	virtual bool PushNoWaiting(Data *data)
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		if(this->m_data.size() == this->m_bufferSize)
			return false;
		m_data.push(data);
		m_conditionEmpty.notify_one();
		return true;
	}

	virtual Data* Pop()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(m_data.empty())
			m_conditionEmpty.wait(lock);
		Data *data = m_data.front();
		m_data.pop();
		m_conditionFull.notify_one();
		//if(m_data.empty())
		//{
		//	++m_bufferSize;
		//	m_conditionFull.notify_one();

		//	boost::xtime duration;
		//	boost::xtime_get(&duration, boost::TIME_UTC_);
		//	duration.sec += 2;

		//	const bool scc = m_conditionEmpty.timed_wait(lock, duration);
		//	//while(m_data.empty())
		//	//	m_conditionEmpty.wait(lock);
		//	//const bool scc = true;

		//	--m_bufferSize;
		//	if(!scc)
		//		return NULL;
		//}
		//if(m_data.empty())
		//	return NULL;
		//Data *data = m_data.front();
		//m_data.pop();
		//m_conditionFull.notify_one();
		return data;
	}

	virtual Data* PopNoWaiting()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		if(m_data.empty())
			return NULL;
		Data *data = m_data.front();
		m_data.pop();
		m_conditionFull.notify_one();
		return data;
	}

	virtual void WaitWhileEmpty()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(m_data.empty())
			m_conditionEmpty.wait(lock);
	}

	virtual void WaitWhileNotFull()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		while(uint(m_data.size()) < this->m_bufferSize)
			m_conditionEmpty.wait(lock);
	}

	virtual uint Size()
	{
		boost::mutex::scoped_lock lock(this->m_mutex);
		return uint(m_data.size());
	}

protected:

	boost::condition m_conditionFull, m_conditionEmpty;
	std::queue<Data *> m_data;
};

template<typename TYPE>
class MutexSignal
{
public:

	MutexSignal() : m_signal(0) {}

	inline bool Set(const TYPE signal, bool waitWhileNoneZero)
	{
		boost::mutex::scoped_lock lock(m_mutex);
		if(!waitWhileNoneZero && m_signal != 0)
			return false;
		while(m_signal != 0)
			m_conditionNonZero.wait(lock);
		m_signal = signal;
		m_conditionZero.notify_one();
		return true;
	}

	inline TYPE Get(const bool waitWhileZero)
	{
		boost::mutex::scoped_lock lock(m_mutex);
		while(waitWhileZero && m_signal == 0)
			m_conditionZero.wait(lock);
		return m_signal;
	}

	inline void SetZero()
	{
		boost::mutex::scoped_lock lock(m_mutex);
		m_signal = 0;
		m_conditionNonZero.notify_one();
	}

protected:

	TYPE m_signal;
	boost::condition m_conditionNonZero, m_conditionZero;
	boost::mutex m_mutex;
};

#endif