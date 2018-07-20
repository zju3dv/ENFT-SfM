/*
 *  qtbuffer.h
 *  libcvd
 */

#ifndef CVD_QTBUFFER_H
#define CVD_QTBUFFER_H

#include <cvd/videobuffer.h>
#include <cvd/OSX/qtframe.h>

#include <cvd/timer.h>
#include <cvd/colourspaces.h>

#include <string>

namespace CVD {
	namespace Exceptions {
		/// Exceptions specific to QTBUFFER
		/// @ingroup gException
		namespace QTBUFFER {
		/// Base class for all QTBuffer exceptions
		/// @ingroup gException
		struct All: public CVD::Exceptions::VideoBuffer::All{};
		/// Error opening the device
		/// @ingroup gException
		struct DeviceOpen: public All {DeviceOpen(std::string msg); ///< Construct from the device name
		};
	}
}

/// Internal QT helpers
namespace QT
{
#ifndef DOXYGEN_IGNORE_INTERNAL
	/*
    template<class C> struct cam_type
    {
		static const int mode = C::Error__type_not_valid_for_camera___Use_byte_or_yuv411_or_rgb_of_byte;
    };
	
    template<> struct cam_type<byte>
    {
        static const unsigned int mode = VIDEO_PALETTE_GREY;
    };
	
    template<> struct cam_type<bayer>
    {
        static const unsigned int mode = VIDEO_PALETTE_RAW;
    };
	
    template<> struct cam_type<yuv422>
    {
        static const unsigned int mode = VIDEO_PALETTE_YUV422;
    };
	
    template<> struct cam_type<Rgb<byte> >
    {
        static const unsigned int mode = VIDEO_PALETTE_RGB24;
    };
	*/
#endif
	
	/// encapsulates internal data structures of @ref RawQT specific to Quicktime
	class RawQTPimpl;
	
    /** Internal (non type-safe) class used by qtbuffer classes to access video devices with
	 * QuickTime support.
	 */
    class RawQT
    {
	public:
        /** constructor
		* @param dev file name of the device to open
		* @param mode color palette to use (not supported yet)
		* @param num which camera to open
		*/
		RawQT(const ImageRef & size, unsigned int mode, unsigned int num = 0, bool showSettingsDialog=false, bool verbose = false);
		virtual ~RawQT();
        
		/** Get the width in pixels of the captured frames. */
		ImageRef get_size() const;
		
		/** returns the data of the next captured frame */
		unsigned char* get_frame();
		/** returns the frame data to be used for further capturing */
		void put_frame( unsigned char * );
		/// Get the camera frame rate
		double frame_rate();
		/// Is there a new frame ready?
		bool frame_pending();
		/// Get the video format string
		std::string get_frame_format_string();
			
	private:
		RawQTPimpl * pimpl;
		std::string frame_format_string;
    };
};

/// A video buffer from a QuickTime video device on Mac OSX.
/// @param T The pixel type of the frames. The supported type are <code><CVD::vuy422></code>
/// which returns 16-bit yuv422 encoded data where v,u and y are flipped. See Apple developer
/// documentation at http://developer.apple.com/quicktime/icefloe/dispatch020.html for details.
///
/// The optimal way to use this class is to call frame_pending() in the event handling loop until 
/// a frame is ready and then call get_frame() to retrieve the frame. Also note that the frame data
/// is not owned by you and may be overriden by QuickTime. Therefore don't use it too long. A future
/// could execute the necessary internal calls in a thread and make better use of processor time even
/// with different usage patterns of the buffer.
///
/// @ingroup gVideoBuffer
template <class T> class QTBuffer : public VideoBuffer<T>, public QT::RawQT
{
public:
    /// Construct a video buffer
    /// @param dev file name of the device to use
    QTBuffer(const ImageRef & size, unsigned int number = 0, bool showSettingsDialog=false, bool verbose = false ) : VideoBuffer<T>(VideoBufferType::Live), RawQT( size, 0, number, showSettingsDialog, verbose ) {}
	
    virtual ImageRef size()
    {
        return RawQT::get_size();
    }
    virtual VideoFrame<T> * get_frame()
    {
        return new QTFrame<T>(get_time_of_day(), (T *)RawQT::get_frame(), RawQT::get_size());
    }
    virtual void put_frame(VideoFrame<T>* f)
    {
        RawQT::put_frame((unsigned char *)f->data());
        delete reinterpret_cast<QTFrame<T> *>(f);
    }
    virtual bool frame_pending()
    {
        return RawQT::frame_pending();
    }
    virtual double frame_rate()
    {
        return RawQT::frame_rate();
    }
	
private:
    QTBuffer( QTBuffer& copyof );
    int operator = ( QTBuffer& copyof );
};

};
#endif
