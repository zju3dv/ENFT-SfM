//-*- c++ -*-
#ifndef CVD_V4LBUFFER_H
#define CVD_V4LBUFFER_H

#include <cvd/config.h>

#include <vector>

#ifdef CVD_INTERNAL_HAVE_STRANGE_V4L2
		#include <videodevx/videodev.h>
#else
		#include <linux/videodev2.h>
#endif



#include <cvd/videobuffer.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/rgb8.h>
#include <cvd/timer.h>
#include <cvd/colourspaces.h>
#include <fcntl.h>

#ifndef VIDEO_PALETTE_GREY
/* These are restored from videodev.h, removed in recent v4l2. */
#define VIDEO_PALETTE_GREY      1       /* Linear greyscale */
#define VIDEO_PALETTE_HI240     2       /* High 240 cube (BT848) */
#define VIDEO_PALETTE_RGB565    3       /* 565 16 bit RGB */
#define VIDEO_PALETTE_RGB24     4       /* 24bit RGB */
#define VIDEO_PALETTE_RGB32     5       /* 32bit RGB */
#define VIDEO_PALETTE_RGB555    6       /* 555 15bit RGB */
#define VIDEO_PALETTE_YUV422    7       /* YUV422 capture */
#define VIDEO_PALETTE_YUYV      8
#define VIDEO_PALETTE_UYVY      9       /* The great thing about standards is ... */
#define VIDEO_PALETTE_YUV420    10
#define VIDEO_PALETTE_YUV411    11      /* YUV411 capture */
#define VIDEO_PALETTE_RAW       12      /* RAW capture (BT848) */
#define VIDEO_PALETTE_YUV422P   13      /* YUV 4:2:2 Planar */
#define VIDEO_PALETTE_YUV411P   14      /* YUV 4:1:1 Planar */
#define VIDEO_PALETTE_YUV420P   15      /* YUV 4:2:0 Planar */
#define VIDEO_PALETTE_YUV410P   16      /* YUV 4:1:0 Planar */
#define VIDEO_PALETTE_PLANAR    13      /* start of planar entries */
#define VIDEO_PALETTE_COMPONENT 7       /* start of component entries */
#endif

namespace CVD {

namespace Exceptions
{
	/// @ingroup gException
	namespace V4LBuffer
	{
		/// @ingroup gException
		struct All: public CVD::Exceptions::VideoBuffer::All
		{
		};
		/// Error opening the device
		/// @ingroup gException
		struct DeviceOpen: public All {DeviceOpen(std::string dev); ///< Construct from the device name
		};

		/// Device was OK, but could not provide the requested colourspace
		/// @ingroup gException
		struct NoColourspace: public All{ NoColourspace(std::string dev, std::string space); ///< Construct from the device name and requested colourspace
		};

		/// Error setting up the device
		/// @ingroup gException
		struct DeviceSetup: public All {DeviceSetup(std::string dev, std::string action);  ///< Construct from the device string and an error string
		};
		/// Error in a put_frame() call
		/// @ingroup gException
		struct PutFrame: public All {PutFrame(std::string dev, std::string msg); ///< Construct from the device name
		};
		/// Error in a get_frame() call
		/// @ingroup gException
		struct GetFrame: public All {GetFrame(std::string dev, std::string msg); ///< Construct from the device name
		};
	}
}

namespace V4L
{
#ifndef DOXYGEN_IGNORE_INTERNAL
	template<class C> struct format;

	template<> struct format<byte>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_GREY;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_GREY;
	};
		
	#ifdef V4L2_PIX_FMT_SBGGR8
	template<> struct format<bayer_grbg>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_SBGGR8;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_RAW;
	};
	template<> struct format<bayer_bggr>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_SBGGR8;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_RAW;
	};
	#endif

	template<> struct format<yuv422>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_YUYV;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_YUV422;
	};

	template<> struct format<vuy422>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_UYVY;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_UYVY;
	};

	template<> struct format<yuv420p>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_YUV420;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_YUV420P;
	};

	template<> struct format<Rgb<byte> >
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_RGB24;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_RGB24;
	};

	template<> struct format<Rgb8>
	{
		static const unsigned int v4l2_fmt = V4L2_PIX_FMT_RGB32;
		static const unsigned int v4l1_palette = VIDEO_PALETTE_RGB32;
	};
#endif
	
	class RawV4LBuffer: public virtual RawVideoBuffer
	{
		public:
			struct Buffer {
				int id;
				unsigned char* data;
				double when;
			};

			RawV4LBuffer(const std::string& dev, unsigned int fmt, ImageRef size, int input, bool fields, int frame_per_second, bool verbose);
			ImageRef getSize();
			Buffer getFrame();
			void releaseFrame(int id);
			double getRate();
			bool pendingFrame();
			virtual ~RawV4LBuffer();

			int num_buffers()
			{
					return num_bufs;
			}

			const std::string & device_name() const 
			{ 
				return dev; 
			}

		private:
			int num_bufs;
			struct State; 
			State* state;
			std::string dev;
	};
	
	class V4L1Client;
	
};


/// A live video buffer which uses the Video for Linux 2 (V4L2) API.
/// A replacement for the (deprecated?) V4L2Buffer
/// @ingroup gVideoBuffer
template <class T> class V4LBuffer : public VideoBuffer<T>, public V4L::RawV4LBuffer
{
	public:
		V4LBuffer(const std::string & dev, ImageRef size, int input=-1, bool fields=false, int frames_per_second=0, bool verbose=0) 
		:VideoBuffer<T>(VideoBufferType::Flushable), RawV4LBuffer(dev, V4L::format<T>::v4l2_fmt, size, input, fields, frames_per_second, verbose)
		{
		}

		virtual ImageRef size() 
		{ 
			return getSize();
		}


		virtual VideoFrame<T> * get_frame() {
			V4L::RawV4LBuffer::Buffer buffer = getFrame();
			return new V4LFrame(buffer.id, buffer.when, buffer.data, getSize());
		}

		virtual void put_frame(VideoFrame<T>* f) {
			V4LFrame* vf = dynamic_cast<V4LFrame*>(f);
			if (vf == 0)
				throw Exceptions::V4LBuffer::PutFrame(device_name(), "Invalid VideoFrame");
			int id = vf->id;
			delete vf;

			releaseFrame(id);
		}

		virtual bool frame_pending() 
		{ 
			return pendingFrame(); 
		}

		virtual double frame_rate() 
		{ 
			return getRate();
		}

		int num_buffers()
		{
			return num_buffers();
		}

	
 private:

		struct V4LFrame : public VideoFrame<T> {
			V4LFrame(int i, double t, void* data, ImageRef size) 
			:VideoFrame<T>(t,reinterpret_cast<T*>(data),size), id(i) 
			{}

			int id;
			friend class V4LBuffer<T>;
		};

		V4LBuffer( V4LBuffer&);
		void operator=( V4LBuffer&);

};

};
#endif
