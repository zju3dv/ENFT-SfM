//-*- c++ -*-

//////////////////////////////////////////////////////////////////////
//                                                                  //
//   V4L1Frame - An image with a timestamp and an index             //
//                                                                  //
//   G Reitmayr APRIL 2005                                          //
//                                                                  //
//////////////////////////////////////////////////////////////////////

#ifndef __CVD_V4L1FRAME_H
#define __CVD_V4L1FRAME_H

#include <cvd/videoframe.h>


#ifdef CVD_INTERNAL_HAVE_STRANGE_V4L2
	#include <videodevx/videodev.h>
#else
	#include <linux/videodev.h>
#endif



namespace CVD {


template <class T> class V4L1Buffer;

/// A frame from a V4L1Buffer
/// @param T the pixel type of the buffer
/// @ingroup gVideoFrame
template <class T>
class V4L1Frame : public VideoFrame<T>
{
	friend class V4L1Buffer<T>;

private:
	/// (Used internally) Construct a video frame
	/// @param t The timestamp
	/// @param size The image size
	/// @param index The index
	/// @param data The image data
        V4L1Frame(double t, T * data, const ImageRef& size) : VideoFrame<T>(t, data, size)
        {
	}

        ~V4L1Frame()
        {
            this->my_data = NULL;
        }
};

}

#endif
