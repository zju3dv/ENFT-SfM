//-*- c++ -*-
#ifndef __CVD_V4L1BUFFER_H
#define __CVD_V4L1BUFFER_H

#include <vector>

#include <cvd/Linux/v4l1frame.h>
#include <cvd/videobuffer.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/timer.h>
#include <cvd/colourspaces.h>

namespace CVD {

namespace Exceptions
{
    /// %Exceptions specific to V4L1Buffer
    /// @ingroup gException
    namespace V4L1Buffer
    {
        /// Base class for all V4L1 exceptions
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
        struct PutFrame: public All {PutFrame(std::string dev, unsigned int number); ///< Construct from the device name
        };
        /// Error in a get_frame() call
        /// @ingroup gException
        struct GetFrame: public All {GetFrame(std::string dev, unsigned int number); ///< Construct from the device name
        };
    }
}

/// Internal V4L1 helpers
namespace V4L1
{
    #ifndef DOXYGEN_IGNORE_INTERNAL
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

	template<> struct cam_type<yuv420p>
    {
        static const unsigned int mode = VIDEO_PALETTE_YUV420P;
    };


    template<> struct cam_type<Rgb<byte> >
    {
        static const unsigned int mode = VIDEO_PALETTE_RGB24;
    };
    #endif

    /** Internal (non type-safe) class used by V4L1Buffer classes to access video devices with
     * v4l1 drivers. Use V4L1Buffer to get the 8-bit greyscale or 24-bit color.
     */
    class RawV4L1
    {
    public:
        /** constructor
         * @param dev file name of the device to open
         * @param mode color palette to use (see linux/video.h for possible modes)
         */
        RawV4L1(const std::string & dev, unsigned int mode, const ImageRef&);
        virtual ~RawV4L1();
        /** Get the width in pixels of the captured frames. */
        const ImageRef& get_size() const;
        /** Set the size of the captured frames. */
        void set_size(const ImageRef& size);
        /** Set the color palette to use */
        void set_palette(unsigned int palette);
        /** Set the brightness in [0,1] of the captured frames. */
        void set_brightness(double brightness);
        /** returns current brightness setting. */
        double get_brightness(void) { return myBrightness; };
        /** Set the whiteness in [0,1] of the captured frames. */
        void set_whiteness(double whiteness);
        /** returns current whiteness setting. */
        double get_whiteness(void) { return myWhiteness; };
        /** Set the hue in [0,1] of the captured frames. */
        void set_hue(double hue);
        /** returns current hue setting. */
        double get_hue(void) { return myHue; };
        /** Set the contrast in [0,1] of the captured frames. */
        void set_contrast(double contrast);
        /** returns current contrast setting. */
        double get_contrast(void) { return myContrast; };
        /** Set the saturation in [0,1] of the captured frames. */
        void set_saturation(double saturation);
        /** returns current saturation setting. */
        double get_saturation(void) { return mySaturation; };
        /** sets autoexposure */
        void set_auto_exp(bool on);
        /** gets autoexposure */
        bool get_auto_exp(void);
        /** Get current settings from the camera device. */
        void retrieveSettings();
        /** Commit the settings to the camera device. */
        void commitSettings();
        /** marks a frame as ready for capturing */
        void captureFrame(unsigned int buffer);
        /** returns the data of the next captured frame */
        unsigned char* get_frame();
        /** returns the frame data to be used for further capturing */
        void put_frame( unsigned char * );
        /// Get the camera frame rate
        double frame_rate();
        /// Is there a frame waiting in the buffer? This function does not block.
        bool frame_pending();
        /// return underlying file handle for select operations etc
        int get_handle() const { return myDevice; };

    private:
        std::string deviceName;
        int myDevice;
        ImageRef mySize;
        unsigned int myPalette;
        double myBrightness, myWhiteness, myContrast, myHue, mySaturation;
        unsigned int myBpp;
        int autoexp;

		void *mmaped_memory;
		size_t mmaped_len;

        std::vector<unsigned char*> myFrameBuf;
        std::vector<bool> myFrameBufState;
        unsigned int myNextRetrieveBuf;
    };
};

/// A video buffer from a v4l1 video device.
/// @param T The pixel type of the frames. The supported type are <code><CVD::byte></code>
/// which returns 8-bit grey scale and <code><CVD::bayer></code> which returns an
/// 8-bit grey scale image containing the raw intensities of the sensor. You have to convert
/// the returned image to a byte image or something else to use it in a meaningful way.
/// <code><CVD::Rgb<CVD::byte> ></code> should work to but
/// crashes with my current driver ?! The pixel type used will automatically configure the
/// underlying RawV4L1 object to use the right video palette.
///
/// @note The grey images are returned upside down unless the kernel module is loaded with
/// the option flipvert.
/// @note frame_rate currently returns fixed 30 fps and frame_pending is untested.
/// @ingroup gVideoBuffer
template <class T> class V4L1Buffer : public VideoBuffer<T>, public V4L1::RawV4L1
{
public:
    /// Construct a video buffer
    /// @param dev file name of the device to use
    V4L1Buffer(const std::string & dev) 
	:VideoBuffer<T>(VideoBufferType::Flushable),
	 RawV4L1( dev, V4L1::cam_type<T>::mode, ImageRef(0,0)) 
	{}

    /// Construct a video buffer
    /// @param dev file name of the device to use
    /// @param size Size of the video stream to grab
    V4L1Buffer(const std::string & dev, ImageRef size) 
	:VideoBuffer<T>(VideoBufferType::Flushable),
	 RawV4L1( dev, V4L1::cam_type<T>::mode,size ) 
	{}

    virtual ImageRef size()
    {
        return RawV4L1::get_size();
    }
    virtual VideoFrame<T> * get_frame()
    {
        return new V4L1Frame<T>(timer.get_time(), (T *)RawV4L1::get_frame(), RawV4L1::get_size());
    }
    virtual void put_frame(VideoFrame<T>* f)
    {
        RawV4L1::put_frame((unsigned char *)f->data());
        delete reinterpret_cast<V4L1Frame<T> *>(f);
    }
    virtual bool frame_pending()
    {
        return RawV4L1::frame_pending();
    }
    virtual double frame_rate()
    {
        return RawV4L1::frame_rate();
    }

	virtual ~V4L1Buffer()
	{
	}

private:
    V4L1Buffer( V4L1Buffer& copyof );
    int operator = ( V4L1Buffer& copyof );
};

/// An 8-bit greyscale video buffer from a V4l1 video device.
/// @ingroup gVideoBuffer
typedef V4L1Buffer<byte> V4L1BufferByte;

};
#endif
