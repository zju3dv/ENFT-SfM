//-*- c++ -*-

//////////////////////////////////////////////////////////////////////
//                                                                  //
//   v4l2Frame - An image with a timestamp and an index             //
//                                                                  //
//   C Kemp Nov 2002                                                //
//                                                                  //
//////////////////////////////////////////////////////////////////////

#ifndef CVD_V4L2FRAME_H
#define CVD_V4L2FRAME_H

#include <cvd/videoframe.h>
#include <cvd/config.h>

#ifdef CVD_INTERNAL_HAVE_STRANGE_V4L2
	#include <videodevx/videodev.h>
#else
	#include <linux/videodev2.h>
#endif

namespace CVD {

/// A frame from a V4L2Buffer
/// This is an 8-bit greyscale video frame
/// @ingroup gVideoFrame
template <class T>	
class V4L2FrameT : public VideoFrame<T>
{
	friend class V4L2Buffer_Base;

	protected:
		/// (Used internally) Construct a video frame
		/// @param t The timestamp
		/// @param size The image size
		/// @param index The index
		/// @param data The image data
		/// @param f The field
	  V4L2FrameT(double t, const ImageRef& size, int index, T *data, VideoFrameFlags::FieldType f) 
		: VideoFrame<T>(t, data, size, f),my_index(index)
		{
		}
		

		//Only 2.6 needs this, but it does no harm in 2.4
		struct v4l2_buffer* m_buf;


	  int my_index;

	  ~V4L2FrameT() 
	  {}

	public:

};

/// Nontemplated video frame type for backwards compatibility
typedef V4L2FrameT<unsigned char> V4L2Frame;


}

#endif
