
#ifndef CVD_READAHEADVIDEOBUFFER_H
#define CVD_READAHEADVIDEOBUFFER_H

#include <cvd/config.h>
#include <cvd/videobuffer.h>

#ifdef CVD_HAVE_PTHREAD
#include <deque>
#include <cvd/thread.h>
#include <cvd/synchronized.h>
#include <cvd/eventobject.h>
#endif

namespace CVD {
    #ifndef CVD_HAVE_PTHREAD
    #ifndef _WIN32
    #warning ReadAheadVideoBuffer will not do any read-ahead because threads are not supported in this build
    #endif
    template <class T> 
    class ReadAheadVideoBuffer : public VideoBuffer<T>
    {
    private:
	VideoBuffer<T>& vbuffer;
    public:
	virtual ~ReadAheadVideoBuffer() {}
	ReadAheadVideoBuffer(VideoBuffer<T>& vb, size_t maxReadAhead=10) 
	: VideoBuffer<T>(vb.type()),vbuffer(vb) {}
	/// The size of the VideoFrames returned by this buffer
	ImageRef size() { return vbuffer.size(); }
	/// Returns the next frame from the buffer. This function blocks until a frame is ready.
	VideoFrame<T>* get_frame() { return vbuffer.get_frame(); }
	/// Tell the buffer that you are finished with this frame. Typically the VideoBuffer then destroys the frame.
	/// \param f The frame that you are finished with.
	void put_frame(VideoFrame<T>* f) { vbuffer.put_frame(f); }
	/// Is there a frame waiting in the buffer? This function does not block. 
	bool frame_pending() { return vbuffer.frame_pending(); }
	/// What is the (expected) frame rate of this video buffer, in frames per second?
	double frame_rate() { return vbuffer.frame_rate(); }
	/// Go to a particular point in the video buffer (only implemented in buffers of recorded video)
	/// \param t The frame time in seconds
	void seek_to(double t){ vbuffer.seek_to(t); }
    };
#else    
    /// Decorator video buffer that preloads frames asynchronously in a separate thread.
    /// @param T The pixel type of the video frames
    /// @param vb The video buffer to wrap/preload
    /// @param maxReadAhead The maximum number of frames to read ahead asynchronously; 
    ///                     the underlying VideoBuffer must support this many concurrently existing VideoFrame's
    /// @ingroup gVideoBuffer
    template <class T> 
    class ReadAheadVideoBuffer : public VideoBuffer<T>, public Runnable
    {
    private:
	VideoBuffer<T>& vbuffer;
	size_t maxRA;
	std::deque<VideoFrame<T>*> q, putq;
	Synchronized vblock;
	EventObject qevent;
	Thread thread;


	static VideoBufferType::Type type_update(VideoBufferType::Type t)
	{
		if(t== VideoBufferType::NotLive)
			return t; 
		else
			return VideoBufferType::Flushable;
	}

    public:
	virtual ~ReadAheadVideoBuffer() {
	    qevent.lock();
	    thread.stop();
	    qevent.trigger();
	    qevent.unlock();
	    thread.join();	    
	}
	ReadAheadVideoBuffer(VideoBuffer<T>& vb, size_t maxReadAhead=10) 
	: VideoBuffer<T>(type_update(vb.type())), vbuffer(vb), maxRA(maxReadAhead) {
	    thread.start(this);	    
	}

	void run() {
	    while (!thread.shouldStop()) 
	    {
		// lock queue
		{ Lock l(qevent);
		    for (size_t i=0; i<putq.size(); i++)
			vbuffer.put_frame(putq[i]);
		    putq.resize(0);
		    while (q.size() >= maxRA) {		    
			qevent.wait();
			if (thread.shouldStop())
			    return;
		    }
		}
		// lock videobuffer
		{ Lock l(vblock);
		    VideoFrame<T>* frame = vbuffer.get_frame();
		    // lock queue
		    { Lock l(qevent);
			q.push_back(frame);
			qevent.trigger();
		    }
		}
	    }
	}
	/// The size of the VideoFrames returned by this buffer
	ImageRef size() { return vbuffer.size(); }
	/// Returns the next frame from the buffer. This function blocks until a frame is ready.
	VideoFrame<T>* get_frame() {
	    VideoFrame<T>* frame = 0;
	    Lock l(qevent);
	    while (q.empty())
		qevent.wait();
	    frame = q.front();
	    q.pop_front();
	    qevent.trigger(); 
	    return frame;
	}
	/// Tell the buffer that you are finished with this frame. Typically the VideoBuffer then destroys the frame.
	/// \param f The frame that you are finished with.
	void put_frame(VideoFrame<T>* f) { Lock l(qevent); putq.push_back(f); }
	/// Is there a frame waiting in the buffer? This function does not block. 
	bool frame_pending() {
	    return !q.empty();
	}
	/// What is the (expected) frame rate of this video buffer, in frames per second?		
	double frame_rate() { return vbuffer.frame_rate(); }
	/// Go to a particular point in the video buffer (only implemented in buffers of recorded video)
	/// \param t The frame time in seconds
	void seek_to(double t){
	    Lock vbl(vblock);
	    Lock ql(qevent);
	    for (size_t i=0; i<q.size(); i++)
		putq.push_back(q[i]);
	    q.resize(0);
	    qevent.triggerAll();
	    vbuffer.seek_to(t);
	}
    };
    #endif
}

#endif
