/*
 *  qtframe.h
 *  libcvd
 */

#ifndef CVD_QTFRAME_H
#define CVD_QTFRAME_H

#include <cvd/videoframe.h>

namespace CVD {
	
	template <class T> class QTBuffer;
	
	/// A frame from a QTBuffer
	/// @param T the pixel type of the buffer
	/// @ingroup gVideoFrame
	template <class T> class QTFrame : public VideoFrame<T> {
		friend class QTBuffer<T>;
		
	private:
		/// (Used internally) Construct a video frame
		/// @param t The timestamp
		/// @param size The image size
		/// @param data The image data
        QTFrame(double t, T * data, const ImageRef& size) : VideoFrame<T>(t, data, size, VideoFrameFlags::Progressive) 
		{}
		
        ~QTFrame() {
            this->my_data = NULL;
		}
	};
}

#endif
