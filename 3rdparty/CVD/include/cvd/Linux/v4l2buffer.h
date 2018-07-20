//-*- c++ -*-
#ifndef __CVD_V4L2BUFFER_H
#define __CVD_V4L2BUFFER_H

#include <cvd/yc.h>

#include <string>

#include <cvd/videobuffer.h>
#include <cvd/Linux/v4l2frame.h>

#define V4L2BUFFERS 3

namespace CVD {

/// Which buffer block method shall I use?
enum V4L2BufferBlockMethod{
  V4L2BBMselect,
  V4L2BBMsleep,
  V4L2BBMchew
};

/*
enum V4L2BufferFrameOrder{
  MostRecent,
  InOrder,
};*/

namespace Exceptions
{
	/// %Exceptions specific to V4L2Buffer
	/// @ingroup gException
	namespace V4L2Buffer
	{
		/// Base class for all V4L2 exceptions
		/// @ingroup gException
		struct All: public CVD::Exceptions::VideoBuffer::All{};
		/// Error opening the device
		/// @ingroup gException
		struct DeviceOpen: public All {DeviceOpen(std::string dev); ///< Construct from the device name
		};
		/// Error setting up the device
		/// @ingroup gException
		struct DeviceSetup: public All {DeviceSetup(std::string dev, std::string action);  ///< Construct from the device string and an error string
		};
		/// Error in a put_frame() call
		/// @ingroup gException
		struct PutFrame: public All {PutFrame(std::string dev); ///< Construct from the device name
		};
		/// Error in a get_frame() call
		/// @ingroup gException
		struct GetFrame: public All {GetFrame(std::string dev); ///< Construct from the device name
		};
	}
}

template<class T>
struct V4L2_Traits;

template<>
struct V4L2_Traits<unsigned char>{
  static const unsigned int pix_code=V4L2_PIX_FMT_GREY;
};

template<>
struct V4L2_Traits<CVD::YC>{
  static const unsigned int pix_code=V4L2_PIX_FMT_YUYV;
};



/// A live video buffer from a the framegrabber (using the Video for Linux 2 API).
/// This provides 8-bit greyscale video frames of type CVD::V4L2Frame and throws exceptions
/// of type CVD::Exceptions::V4L2Buffer
/// @ingroup gVideoBuffer
class V4L2Buffer_Base
{
	public:
		/// Construct a video buffer
		/// @param devname The device name
		/// @param fields
		/// @param block Which buffer block method to use
		/// @param input Which card input?
		/// @param numbufs How many buffers?
	  V4L2Buffer_Base(const char *devname, bool fields, V4L2BufferBlockMethod block, int input, int numbufs, unsigned long int pixtpe);
		~V4L2Buffer_Base();

		ImageRef size() 
		{
			return my_image_size;
		}

		V4L2FrameT<unsigned char>* get_frame(); 
		/// Tell the buffer that you are finished with this frame. Overloaded version of VideoBuffer<T>::put_frame()
		void put_frame(VideoFrame<unsigned char>* f);
		/// Tell the buffer that you are finished with this frame. Overloaded version of VideoBuffer<T>::put_frame()
		void put_frame(V4L2FrameT<unsigned char>* f);
		bool frame_pending();

		double frame_rate() 
		{
			return my_frame_rate;
		}

	private:
		std::string device;
		int my_dropped_frames;
		int my_prev_frame_no;
		int num_buffers;
		bool i_am_using_fields;
		double my_frame_rate;
		ImageRef my_image_size;
		V4L2BufferBlockMethod my_block_method;
		struct v4l2_buffer* m_sv4l2Buffer;
		void** m_pvVideoBuffer;
		int m_nVideoFileDesc;
		//struct v4l2_performance m_sv4l2Performance;

		int my_fd;

		V4L2Buffer_Base( V4L2Buffer_Base& copyof );
		int operator = ( V4L2Buffer_Base& copyof );

};

template<class T>
class V4L2BufferT : public VideoBuffer<T>,
                    public V4L2Buffer_Base 
{
	public:
		V4L2BufferT(const char *devname, bool fields, V4L2BufferBlockMethod block, int input=1, int numbufs=V4L2BUFFERS)
		:VideoBuffer<T>(VideoBufferType::Flushable),V4L2Buffer_Base(devname, fields, block, input, numbufs, V4L2_Traits<T>::pix_code)
		{}

		virtual ImageRef size() 
		{
			return V4L2Buffer_Base::size();
		}

		virtual VideoFrame<T>* get_frame() {return reinterpret_cast<V4L2FrameT<T>*>(V4L2Buffer_Base::get_frame());}

		/// Tell the buffer that you are finished with this frame. Overloaded version of VideoBuffer<T>::put_frame()
		virtual void put_frame(VideoFrame<T>* f) {V4L2Buffer_Base::put_frame(reinterpret_cast<V4L2FrameT<unsigned char>*>(f));}

		virtual bool frame_pending() {return V4L2Buffer_Base::frame_pending();}
		virtual double frame_rate() {return V4L2Buffer_Base::frame_rate();}
};

/// Nontemplated V4L2Buffer type for backwards compatibility.
typedef V4L2BufferT<unsigned char> V4L2Buffer;

}

#endif
















