////////////////////////////////////////////////////////////////////////////
//  Copyright 2017-2018 Computer Vision Group of State Key Lab at CAD&CG, 
//  Zhejiang University. All Rights Reserved.
//
//  For more information see <https://github.com/ZJUCVG/ENFT-SfM>
//  If you use this code, please cite the corresponding publications as 
//  listed on the above website.
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes only.
//  Any modification based on this work must be open source and prohibited
//  for commercial use.
//  You must retain, in the source form of any derivative works that you 
//  distribute, all copyright, patent, trademark, and attribution notices 
//  from the source form of this work.
//   
//
////////////////////////////////////////////////////////////////////////////
#pragma once

#ifndef _WIN32
#define __LINUX__
#define _LINUX_
#define LINUX
#endif

#ifdef __LINUX__
#include <cstring>
#include <climits>
#include <X11/Xlib.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#include <xutility>
#include <io.h>
#include <windows.h>
#endif
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <stdlib.h>
#include <assert.h>
#include <float.h>


#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#ifdef Success
#undef Success
#endif

typedef unsigned char       ubyte;
typedef unsigned short      ushort;
typedef unsigned int        uint;
typedef unsigned long long  ullong;

#ifndef PI
#define PI 3.141592654f
#endif

#ifndef PIx2
#define PIx2 6.283185308f
#endif

#ifndef MAX_NUM_THREADS
#define MAX_NUM_THREADS 8
#endif

#ifndef MAX_LINE_LENGTH
#define MAX_LINE_LENGTH 512
#endif

#ifndef FACTOR_RAD_TO_DEG
#define FACTOR_RAD_TO_DEG 57.295779505601046646705075978956f
#endif

#ifndef FACTOR_DEG_TO_RAD
#define FACTOR_DEG_TO_RAD 0.01745329252222222222222222222222f
#endif

#ifndef EQUAL_TOLERANCE_ABS
#define EQUAL_TOLERANCE_ABS 0.001f
#endif

#ifndef EQUAL_TOLERANCE_REL
#define EQUAL_TOLERANCE_REL 0.01f
#endif

#ifndef EQUAL
#define EQUAL(a, b) (fabs((a) - (b)) <= EQUAL_TOLERANCE_ABS || (fabs((a) - (b)) / std::min(fabs((a)), fabs((b)))) <= EQUAL_TOLERANCE_REL)
#endif

#ifndef ABS_DIF
#define ABS_DIF(a, b) (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))
#endif

#ifndef SWAP
#define SWAP(a, b, t) (t) = (a); (a) = (b); (b) = (t)
#endif

#ifndef CLAMP
#define CLAMP(x, mn, mx) (((x) < (mn)) ? (mn) : (((x) > (mx)) ? (mx) : (x)))
#endif

#ifdef __LINUX__
#define CreateDirectory(filename, tag) mkdir(filename,00777)
#define _aligned_free(p) free(p)
#define _aligned_malloc(size,alignment) aligned_alloc(alignment,size)
#endif
