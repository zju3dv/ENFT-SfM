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

#ifndef _THREAD_H_
#define _THREAD_H_

#include "Mutex.h"
#include "Timer.h"
#include <boost/thread/thread.hpp>

//class ThreadSynchronizer
//{
//
//public:
//
//	inline void Initialize(const uint &nThreads) { m_nThreads = nThreads; m_nThreadsFinished = 0; }
//	inline void Synchronize()
//	{
//		boost::mutex::scoped_lock lock(m_mutex);
//		++m_nThreadsFinished;
//		while(m_nThreadsFinished < m_nThreads)
//			m_condition.wait(lock);
//		m_condition.notify_all();
//	}
//
//protected:
//
//	uint m_nThreads, m_nThreadsFinished;
//	boost::mutex m_mutex;
//	boost::condition m_condition;
//
//};

class ThreadSerializer
{

public:

	inline void Initialize() { m_idx = 0; }
	inline void Serialize(const uint idx)
	{
		boost::mutex::scoped_lock lock(m_mutex);
		while(m_idx != idx)
			m_condition.wait(lock);
		m_idx = idx + 1;
		m_condition.notify_all();
	}

//protected:
public:

	uint m_idx;
	boost::mutex m_mutex;
	boost::condition m_condition;

};

template<class Function, class Data>
class ForegroundThread
{

public:

	ForegroundThread(Function *pFunc, MutexBuffer<Data> *pInputBuffer, MutexBuffer<Data> *pOutputBuffer, ThreadSerializer *pSerializer1, 
					 ThreadSerializer *pSerializer2, const uint idx) : m_pFunc(pFunc), m_pInputBuffer(pInputBuffer), m_pOutputBuffer(pOutputBuffer), 
					 m_pSerializer1(pSerializer1), m_pSerializer2(pSerializer2), m_idx(idx) {}

	void operator()()
	{
		{
			boost::mutex::scoped_lock lock(m_pSerializer1->m_mutex);
			while(m_pSerializer1->m_idx != m_idx)
				m_pSerializer1->m_condition.wait(lock);
			m_pFunc->Start();
			m_pSerializer1->m_idx = m_idx + 1;
			m_pSerializer1->m_condition.notify_all();
		}

		Timer timer;

		//uint idx = 0;
		Data *data;
		bool running = true;
		while(running)
		{
			if((data = m_pInputBuffer->Pop()) == NULL)
				continue;
			timer.Start();
			running = m_pFunc->Run(data);
			timer.Stop();
			if(m_pOutputBuffer)
				m_pOutputBuffer->Push(data);
		}

		//char fileName[MAX_LINE_LENGTH];
		//sprintf(fileName, "F:/tmp/timing_%d.txt", m_idx);
		//FILE *fp = fopen( fileName, "w");
		//timer.PrintTotalAndStableMeanTiming(0, fp);
		//fclose(fp);

		{
			boost::mutex::scoped_lock lock(m_pSerializer2->m_mutex);
			while(m_pSerializer2->m_idx != m_idx)
				m_pSerializer2->m_condition.wait(lock);
			m_pFunc->Stop();
			printf("[%d] ", m_idx);
			m_pSerializer2->m_idx = m_idx + 1;
			m_pSerializer2->m_condition.notify_all();
		}
	}

	inline static void Print(char *format, ...)
	{
		va_list args;
		char buf[MAX_LINE_LENGTH];

		va_start(args, format);
		vsprintf(buf, format, args);
		va_end(args);

		static boost::mutex g_mutex;
		{
			boost::mutex::scoped_lock lock(g_mutex);
			printf(buf);
		}
	}

protected:

	uint m_idx;
	Function *m_pFunc;
	MutexBuffer<Data> *m_pInputBuffer, *m_pOutputBuffer;
	ThreadSerializer *m_pSerializer1, *m_pSerializer2;

};

template<class Function, class Data>
class BackgroundThread
{

public:

	BackgroundThread(Function *pFunc, MutexBuffer<Data> *pInputBuffer, MutexBuffer<Data> *pOutputBuffer, const uint idx) : 
	  m_pFunc(pFunc), m_pInputBuffer(pInputBuffer), m_pOutputBuffer(pOutputBuffer), m_idx(idx) {}

	void Start()
	{
		m_running = true;
		CloseHandle(CreateThread(NULL, 0, Run, this, 0, NULL));
	}
	void Stop()
	{
		m_running = false;
		while(m_pFunc->IsRunning())
			Sleep(20);
		m_pInputBuffer->PushNoWaiting(NULL);
		while(!m_stopped)
			Sleep(20);
	}

	static DWORD WINAPI Run(LPVOID pParam)
	{
		BackgroundThread* pThread = (BackgroundThread* ) pParam;
		pThread->m_stopped = false;

		Data *data;
		while(pThread->m_running)
		{
			if((data = pThread->m_pInputBuffer->Pop()) == NULL)
			//if((data = pThread->m_pInputBuffer->PopNoWaiting()) == NULL)
				continue;
			//data = pThread->m_pInputBuffer->Pop();
			pThread->m_pFunc->Run(*data);
			if(pThread->m_pOutputBuffer)
				pThread->m_pOutputBuffer->Push(data);
		}
		pThread->m_stopped = true;
		printf("Stop %d\n", pThread->m_idx);
		return 0;
	}

protected:

	uint m_idx;
	Function *m_pFunc;
	MutexBuffer<Data> *m_pInputBuffer, *m_pOutputBuffer;
	bool m_running, m_stopped;
};

template<class Function, typename TYPE>
class BackgroundThreadSignal
{

public:

	BackgroundThreadSignal(Function *pFunc, MutexSignal<TYPE> *pSignal, const uint idx)	: m_pFunc(pFunc), m_pSignal(pSignal), m_idx(idx) {}

	void Start()
	{
		m_running = true;
		CreateThread(NULL, 0, Run, this, 0, NULL);
	}
	void Stop()
	{
		m_running = false;
	}

	static DWORD WINAPI Run(LPVOID pParam)
	{
		BackgroundThreadSignal* pThread = (BackgroundThreadSignal* ) pParam;
		pThread->m_pFunc->Start();
		
		while(pThread->m_running)
		{
			const TYPE signal = pThread->m_pSignal->Get(true);
			pThread->m_pFunc->Run(signal);
			pThread->m_pSignal->SetZero();
		}

		pThread->m_pFunc->Stop();
		return 0;
	}

protected:

	uint m_idx;
	Function *m_pFunc;
	MutexSignal<TYPE> *m_pSignal;
	bool m_running;
};

#endif