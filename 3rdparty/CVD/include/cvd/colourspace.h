#ifndef CVD_COLOURSPACE_H
#define CVD_COLOURSPACE_H

namespace CVD
{
	/// Internal colourspace conversions
	namespace ColourSpace
	{
		/// Convert YUV 411 pixel data to RGB
		/// @param yuv411 The input data
		/// @param npix The number of pixels
		/// @param out The output data
		void yuv411_to_rgb(const unsigned char* yuv411, int npix, unsigned char* out);
		/// Convert YUV 411 pixel data to Y only
		/// @param yuv411 The input data
		/// @param npix The number of pixels
		/// @param out The output data
		void yuv411_to_y(const unsigned char* yuv411, int npix, unsigned char* out);
		/// Convert YUV 411 pixel data to Y and RGB
		/// @param yuv411 The input data
		/// @param npix The number of pixels
		/// @param outc colour output data
		/// @param outy luma output data
		void yuv411_to_rgb_y(const unsigned char* yuv411, int npix, unsigned char* outc, unsigned char* outy);

		
		/// Convert Bayer pattern of the form bggr to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_bggr(const unsigned char* bggr, unsigned char* grey, unsigned int width, unsigned int height);

		/// Convert Bayer pattern of the form gbrg to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_gbrg(const unsigned char* gbrg, unsigned char* grey, unsigned int width, unsigned int height);

		/// Convert Bayer pattern of the form grbg to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_grbg(const unsigned char* grbg, unsigned char* grey, unsigned int width, unsigned int height);
		
		/// Convert Bayer pattern of the form rggb to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_rggb(const unsigned char* rggb, unsigned char* grey, unsigned int width, unsigned int height);
		
		/// Convert Bayer pattern of the form bggr to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_bggr(const unsigned char* bggr, unsigned char* rgb, unsigned int width, unsigned int height);

		/// Convert Bayer pattern of the form gbrg to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_gbrg(const unsigned char* gbrg, unsigned char* rgb, unsigned int width, unsigned int height);

		/// Convert Bayer pattern of the form grbg to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_grbg(const unsigned char* grbg, unsigned char* rgb, unsigned int width, unsigned int height);
		
		/// Convert Bayer pattern of the form rggb to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_rggb(const unsigned char* rggb, unsigned char* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit Bayer pattern of the form bggr to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_bggr(const unsigned short* bggr, unsigned short* grey, unsigned int width, unsigned int height);

		/// Convert 16bit Bayer pattern of the form gbrg to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_gbrg(const unsigned short* gbrg, unsigned short* grey, unsigned int width, unsigned int height);

		/// Convert 16bit Bayer pattern of the form grbg to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_grbg(const unsigned short* grbg, unsigned short* grey, unsigned int width, unsigned int height);
		
		/// Convert 16bit Bayer pattern of the form rggb to greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_rggb(const unsigned short* rggb, unsigned short* grey, unsigned int width, unsigned int height);
		
		/// Convert 16bit Bayer pattern of the form bggr to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_bggr(const unsigned short* bggr, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit Bayer pattern of the form gbrg to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_gbrg(const unsigned short* gbrg, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit Bayer pattern of the form grbg to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_grbg(const unsigned short* grbg, unsigned short* rgb, unsigned int width, unsigned int height);
		
		/// Convert 16bit Bayer pattern of the form rggb to rgb444 data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_rggb(const unsigned short* rggb, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit big endian Bayer pattern of the form bggr to host endian greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_bggr_be(const unsigned short* bggr, unsigned short* grey, unsigned int width, unsigned int height);

		/// Convert 16bit big endian Bayer pattern of the form gbrg to host endian greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_gbrg_be(const unsigned short* gbrg, unsigned short* grey, unsigned int width, unsigned int height);

		/// Convert 16bit big endian Bayer pattern of the form grbg to host endian greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_grbg_be(const unsigned short* grbg, unsigned short* grey, unsigned int width, unsigned int height);
		
		/// Convert 16bit big endian Bayer pattern of the form rggb to host endian greyscale data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_grey_rggb_be(const unsigned short* rggb, unsigned short* grey, unsigned int width, unsigned int height);
		
		/// Convert 16bit big endian Bayer pattern of the form bggr to host endian rgb data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_bggr_be(const unsigned short* bggr, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit big endian Bayer pattern of the form gbrg to host endian rgb data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_gbrg_be(const unsigned short* gbrg, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert 16bit big endian Bayer pattern of the form grbg to host endian rgb data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_grbg_be(const unsigned short* grbg, unsigned short* rgb, unsigned int width, unsigned int height);
		
		/// Convert 16bit big endian Bayer pattern of the form rggb to host endian rgb data
		/// @param bggr The input data
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void bayer_to_rgb_rggb_be(const unsigned short* rggb, unsigned short* rgb, unsigned int width, unsigned int height);

		/// Convert yuv422 to rgb444.
		/// @param yuv The yuv422 data: yuyvyuyv....
		/// @param rgb The output data
		/// @param width The width of the image
		/// @param height The height of the image
		  
		void yuv422_to_rgb(const unsigned char* yuv, unsigned char* rgb, unsigned int width, unsigned int height);
		
		/// Convert yuv422 to grey.
		/// @param yuv The yuv422 data: yuyvyuyv....
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		  void yuv422_to_grey(const unsigned char* yuv, unsigned char* grey, unsigned int width, unsigned int height);
		
		/// Convert yuv420 planar to rgb444
		/// @param y The y plane: one byte per pixel
		/// @param u The u plane: one byte per 2 pixel square 
		/// @param v The v plane: one byte per 2 pixel square
		/// @param rgb The output data
		/// @param width The width of the image
		/// @param rowpairs The number of rows pairs (i.e., height/2) in the image
		void yuv420p_to_rgb(const unsigned char* y, const unsigned char* u, const unsigned char* v, 
				    unsigned char* rgb, unsigned int width, unsigned int rowpairs);
		
		/// Convert yuv420 planar to grey
		/// @param y The y plane: one byte per pixel
		/// @param u The u plane: one byte per 2 pixel square 
		/// @param v The v plane: one byte per 2 pixel square
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void yuv420p_to_grey(const unsigned char* y, const unsigned char* u, const unsigned char* v, 
				     unsigned char* grey, unsigned int width, unsigned int height);
		
		/// Convert vuy422 to rgb444.
		/// @param yuv The vuy422 data: uyvyuyvy....
		/// @param rgb The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void vuy422_to_rgb(const unsigned char* yuv, unsigned char* rgb, unsigned int width, unsigned int height);
		
		/// Convert yuv422 to grey.
		/// @param yuv The vuy422 data: uyvyuyvy....
		/// @param grey The output data
		/// @param width The width of the image
		/// @param height The height of the image
		void vuy422_to_grey(const unsigned char* yuv, unsigned char* grey, unsigned int width, unsigned int height);
	}
}

#endif
