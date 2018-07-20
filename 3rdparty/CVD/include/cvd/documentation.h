///////////////////////////////////////////////////////
// General Doxygen documentation
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// The main title page
/**
@mainpage

\section Information
This C++ library is designed to be easy to use and portable for fast video saving, loading and display.
It supports OpenGL and X Windows.
It is released under the LGPL License.

\section Features
\subsection Language

    - Modern C++ design
	- Optimized assembly code
    - Extensive use of exceptions
    - OpenGL wrappers 

\subsection Imaging

    - Type-safe easy to use images
    - Flexible loading and saving from a variety of types:
		- Native
			- PNM   
			- BMP
			- ASCII text
			- FITS 
			- PS   (saving only)
			- EPS  (saving only)
			- CVD (a custom type which supports fast, lossless compression of greyscale, RGB and Bayer images)
		- External libraries required
			- JPEG
			- TIFF 
			- PNG
		- 1/8/16/32 bit signed and unsigned integer and 32/64 bit floating point images in greyscale, RGB and RGBA.
		- Optimum bit depth and colour depth selected automatically
    - Image grabbing from video sources:
		  - Linux
		  		- Video for Linux devices
          		- Video for Linux 2 devices
         	 	- Firewire cameras
                - Firewire over USB cameras produced by PointGrey - see http://www.ptgrey.com/products/fireflymv/index.asp \n
                  Use DVBuffer3 for that and make sure to set an image resolution
		  - IRIX
          		- DMedia video capture
		  - OSX
		  		- QuickTime video capture
		  - All UNIX platforms
		        - Live capture from HTTP server push JPEG cameras.
		  - All platforms
          		- AVI and MPEG file (all codecs supported by ffmpeg)
          		- list of images
				- Server push multipart JPEG streams.
		  - Convenient run-time selection using a URL-like syntax
    - Colorspace conversions on images and video streams
    - Various image processing tools
	      - FAST corner detection
		  - Harris/Shi-Tomasi corner detection
		  - Connect components analysis
		  - Image interpolation and resampling
		  - Convolutions
		  - Drawing in to images
		  - Flipping, pasting, etc
		  - Interpolation, warping and resampling
		  - Integral images
    - %Camera calibration support: Linear, Cubic, Quintic and Harris
		  - Program to calibrate cameras from video

\subsection Mathematical

    - Lie Group(S03, SE3) algebra
    - Iterative and reweighted least-squares 
	- Random numbers

\section Portability
  LibCVD will compile on Visual Studio and any reasonable unixy environment,
  with a reasonable C++ compiler, with TR1 and GNU Make.  Additionally, libCVD
  supports both normal and cross compilers.  Status:
  
  -Well tested (current):
	- Linux: x86, x86-64
	- Mac OS X: x86
		- Supports the OSX build environment including:
			- Frameworks
			- .dylib libraries
	- iOS
		- XCode project provided
	- Cygwin: x86
	- MinGW: x86 (native and cross compile)
	- MinGW: x86_64 (cross compile)
	- Win32: Visual Studio 2008
	- Win32: Visual Studio 2010 (via import of the VS 2008 project file)

  -Has worked on (current status unknown):
	- Linux: PPC
	- Mac OS X: PPC
	- Solaris: SPARC
	- Linux: ARM LPC3180, XScale (cross compile)
	- uCLinux: Blackfin  (cross compile)
	- FreeBSD: x86
	- OpenBSD: XScale
  
  -No longer supported
	- IRIX SGI O2: MIPS (configuration hacks removed)
	- Win32: Visual Studio 2005 (broken project file now removed)

\section Compiling

The normal system works:
@code
	./configure
	make 
	make install
@endcode

libCVD fully supports parallel builds (<code>make -j48</code> for instance).

\subsection slBugs Library bugs/issues
	There are a few known library bugs which prevent the libraries working with libCVD

	- TooN: If you have gcc >= 4.2.0 you need a version of TooN at least as recent as SNAPSHOT_20080725.


\subsection scBugs Compiler bugs
	There are a few known compiler bugs which affect libCVD on various platforms.

	- ppc-darwin (Mac OS X, on PPC)
		- Does not support gcc 3.3 (20030304) due to possible compiler bug.
		- Workaround: none. Install a more recent compiler.
	- arm-linux (cross compiler running on i686-pc-linux-gnu)
		- Internal compiler error with: gcc version 3.4.0 20040409 (CodeSourcery ARM Q1 2004) on:
			- cvd_src/videosource.cc
			- pnm_src/jpeg.cc
		- Remove <code>cvd_src/videosource.o</code> from <code>Makefile</code> and use <code>--disable-jpeg</code>
		- Compile files with -O2 instead of -O3

\subsection OSX OSX Compilation notes

To build libCVD in 32 bit mode, use the \c configure_osx_32bit script instead of directly calling the configure script.

\subsection iOS iOS

An xcode project is provided in the \c build directory

\subsection win32 Windows

For Win32 systems, the @c build directory contains project files for different versions
of Visual Studio. Currently the vc2008 solutions are supported and should work out of the box. 

libCVD requires the <a href="http://www.microsoft.com/downloads/en/details.aspx?FamilyId=D466226B-8DAB-445F-A7B4-448B326C48E7&displaylang=en">Visual Studio feature pack for TR1</a> support.

There are several projects which can be compiled:

- libcvd-TooN A very basic configuration which only requires the <a href="http://mi.eng.cam.ac.uk/~er258/cvd/toon.html">TooN</a> headers. You must add the TooN
  directory to the include path using: 
    - (Popup Menu) Project Properties -> (Dialog) Configuration -> C/C++ -> Common : Additional Include Directories

- libcvd-TooN-JPEG-pthreads This is a more featureful configuration which has JPEG and threading support. In order to compile
  CVD, you must first compile these libraries:
	- pthreads-win32 at http://sourceware.org/pthreads-win32/
	- jpeg at http://www.ijg.org/

In order to avoid the need to set up a large number of include paths, all libCVD
projects assume the existence of three environment variables describing the location of header, 
library and binary files (for DLLs).
	
	- @c INCLUDEDIR contains the header files. libcvd headers will be copied into @c \%INCLUDEDIR%\\\libcvd
	- @c LIBDIR contains library files. libcvd static libraries (debug and release verions) will be copied into @c \%LIBDIR\%
	- @c BINDIR is not used for libcvd, but would be the default directory for DLLs

To use this feature, make a directory tree containing (for example):

- C:\\local\\include
- C:\\local\\bin
- C:\\local\\lib

Then set up three environment variables (e.g. using <a href="http://www.rapidee.com/en/about">rapidee</a>) to be the following:
- INCLUDEDIR=C:\\local\\include
- BINDIR=C:\\local\\bin
- LIBDIR=C:\\local\\lib

libCVD will then find all headers and libraries in that directory tree. LibCVD also includes the file:
- install.vcproj

This will copy the relevant files into that tree.

libCVD compiles to static libraries for simpler linking and to avoid the _dllexport/_dllimport statements throughout the code.

Configuration of features is manual through a default config file in @c build/vs2008/config-*.h. Edit this file to change your configuration, 
for example to support other image formats such as PNG. Alternatively, new configurations can be generated using the file:

- make/make_vcproj_all.sh

from any unix-like environment (including MINGW and Cygwin).

*/

///////////////////////////////////////////////////////
// Modules classifying classes and functions

/// @defgroup gImage Image storage and manipulation
/// Basic image functionality. The %CVD image classes provide fast and
/// flexible access to images.

/// @defgroup gImageIO Image loading and saving, and format conversion
/// Functions to support saving and loading of BasicImage and Image 
/// to and from streams. Supports a few common file formats (autodetecting on loading).
/// Also functions for perfoming type conversion as necessary.

/// @defgroup gVideo Video devices and video files
/// Classes and functions to manage video streams and present them as images.

/// @defgroup gVideoBuffer Video buffers
/// @ingroup gVideo
/// All classes and functions relating to video buffers (as opposed to video frames)

/// @defgroup gVideoFrame Video frames
/// @ingroup gVideo
/// All classes and functions relating to video frames (as opposed to video buffers)

/// @defgroup gException Exceptions 
/// Exceptions generated and thrown by %CVD classes and functions

/// @defgroup gGraphics Computer graphics
/// Classes and functions to support miscellaneous pixel operations

/// @defgroup gVision Computer Vision
/// Functions and classes to support common computer vision concepts and operations

/// @defgroup gGL GL helper functions and classes.
/// Overloaded versions of GL functions to use %CVD classes and datatypes, and
/// other helpful GL classes and functions.

/// @defgroup gMaths Mathematical operations
/// Useful mathematical classes and functions

/// @defgroup gLinAlg Linear Algebra
/// Classes and functions for common Linear Algebra concepts and operations

/// @defgroup gCPP General C++ and system helper functions
/// Classes and functions for writing better code


/// @namespace CVD
/// All classes and functions are within the CVD namespace

