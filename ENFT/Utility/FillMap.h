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


#ifndef _FILL_MAP_H_
#define _FILL_MAP_H_

#ifndef REPEAT_3_TIMES
#define REPEAT_3_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_5_TIMES
#define REPEAT_5_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_7_TIMES
#define REPEAT_7_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_9_TIMES
#define REPEAT_9_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_11_TIMES
#define REPEAT_11_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_13_TIMES
#define REPEAT_13_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_15_TIMES
#define REPEAT_15_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_16_TIMES
#define REPEAT_16_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_17_TIMES
#define REPEAT_17_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_19_TIMES
#define REPEAT_19_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_21_TIMES
#define REPEAT_21_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_23_TIMES
#define REPEAT_23_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

#ifndef REPEAT_25_TIMES
#define REPEAT_25_TIMES(...)\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}\
{__VA_ARGS__}
#endif

class FillMap
{

public:

	inline FillMap() { memset(this, 0, sizeof(FillMap)); }

	inline void Resize(const ushort &width, const ushort &height)
	{
		if(m_width == width && m_height == height)
			return;
		m_width = width;
		m_height = height;
		//m_stride = m_width + (8 - (width & 7));
		m_stride = (width + 7) & (~7);
		m_strideInv8 = (m_stride >> 3);
		m_strideInv4 = (m_stride >> 2);
		m_strideInv2 = (m_stride >> 1);
		m_nBytes = m_stride * height;
		if(m_nBytes > m_nMaxBytes)
		{
			m_nMaxBytes = m_nBytes;
			if(!m_rows.empty())
				delete m_rows[0];
			m_rows.resize(height);
			m_rows[0] = new ubyte [m_nBytes];
		}
		for(ushort y = 1; y < height; y++)
			m_rows[y] = m_rows[y - 1] + m_stride;
	}
	inline const ushort& GetWidth() const { return m_width; }
	inline const ushort& GetHeight() const { return m_height; }
	inline void GetSize(ushort &width, ushort &height) const { width = m_width; height = m_height; }
	inline const ubyte* operator[] (const ushort &y) const { return m_rows[y]; }
	inline		 ubyte* operator[] (const ushort &y)	   { return m_rows[y]; }
	inline void DownSample(FillMap &fillMap) const
	{
		ushort x, x1, x2, y, y1, y2;
		const ubyte *src1, *src2;
		ubyte *dst;
		for(y = 0, y1 = 0, y2 = 1; y2 < m_height; ++y, y1 += 2, y2 += 2)
		{
			src1 = m_rows[y1];
			src2 = m_rows[y2];
			dst = fillMap.m_rows[y];
			for(x = 0, x1 = 0, x2 = 1; x2 < m_width; ++x, x1 += 2, x2 += 2)
				dst[x] = src1[x1] | src1[x2] | src2[x1] | src2[x2];
		}
	}
	inline void UpSample(FillMap &fillMap) const
	{
		ushort x, x1, x2, y, y1, y2;
		const ubyte *src;
		ubyte *dst1, *dst2;
		for(y = 0, y1 = 0, y2 = 1; y2 < fillMap.m_height; ++y, y1 += 2, y2 += 2)
		{
			dst1 = fillMap.m_rows[y1];
			dst2 = fillMap.m_rows[y2];
			src = m_rows[y];
			for(x = 0, x1 = 0, x2 = 1; x2 < fillMap.m_width; ++x, x1 += 2, x2 += 2)
				dst1[x1] = dst1[x2] = dst2[x1] = dst2[x2] = src[x];
		}
	}
	inline void SetZero() { memset(m_rows[0], 0, m_nBytes); }
	inline ubyte IsFilled(const ushort &x, const ushort &y) const { return m_rows[y][x]; }
	template<ushort MIN_DIST> inline void Fill(const ushort &x, const ushort &y)
	{
		if(IsOutside<MIN_DIST>(x, y))
			FillPartial<MIN_DIST>(x, y);
		else
			FillFast<MIN_DIST>(x, y);
	}

template<ushort MIN_DIST, class Point, typename Score>
	inline void MarkMaximalPoints(const Point *xs, const std::vector<std::pair<Score, uint> > &iPtsScored, const ushort &maxNumPts, std::vector<bool> &ptMarks)
	{
		SetZero();
		const uint nPts = uint(iPtsScored.size());
		ptMarks.assign(nPts, false);

		ushort x, y;
		uint iPt;
		ushort markedCnt = 0;
		for(uint i = 0; i < nPts; ++i)
		{
			iPt = iPtsScored[i].second;
			xs[iPt].Get(x, y);
			if(IsFilled(x, y))
				continue;
			Fill<MIN_DIST>(x, y);
			ptMarks[iPt] = true;
			if(++markedCnt == maxNumPts)
				break;
		}
		if(maxNumPts >= nPts)
			return;
		for(uint i = 0; i < nPts && markedCnt < maxNumPts; ++i)
		{
			iPt = iPtsScored[i].second;
			if(ptMarks[iPt])
				continue;
			ptMarks[iPt] = true;
			++markedCnt;
		}
	}
	template<ushort MIN_DIST, class Point>
	inline void Fill(const Point *xs, const ushort &nPts)
	{
		ushort x, y;
		for(ushort i = 0; i < nPts; ++i)
		{
			xs[i].Get(x, y);
			Fill<MIN_DIST>(x, y);
		}
	}
	template<class Point, typename Score>
	inline void MarkMaximalPoints(const Point *xs, const std::vector<std::pair<Score, uint> > &iPtsScored, const ushort &maxNumPts, std::vector<bool> &ptMarks, 
								  const ushort &minDist)
	{
		switch(minDist)
		{
		case 1 : MarkMaximalPoints<1 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 2 : MarkMaximalPoints<2 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 3 : MarkMaximalPoints<3 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 4 : MarkMaximalPoints<4 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 5 : MarkMaximalPoints<5 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 6 : MarkMaximalPoints<6 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 7 : MarkMaximalPoints<7 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 8 : MarkMaximalPoints<8 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 9 : MarkMaximalPoints<9 >(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 10: MarkMaximalPoints<10>(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 11: MarkMaximalPoints<11>(xs, iPtsScored, maxNumPts, ptMarks); break;
		case 12: MarkMaximalPoints<12>(xs, iPtsScored, maxNumPts, ptMarks); break;
		default: printf("Error!\n");	exit(0);
		}
	}
	template<class Point>
	inline void Fill(const Point *xs, const ushort &nPts, const ushort &minDist)
	{
		switch(minDist)
		{
		case 1 : Fill<1 >(xs, nPts); break;
		case 2 : Fill<2 >(xs, nPts); break;
		case 3 : Fill<3 >(xs, nPts); break;
		case 4 : Fill<4 >(xs, nPts); break;
		case 5 : Fill<5 >(xs, nPts); break;
		case 6 : Fill<6 >(xs, nPts); break;
		case 7 : Fill<7 >(xs, nPts); break;
		case 8 : Fill<8 >(xs, nPts); break;
		case 9 : Fill<9 >(xs, nPts); break;
		case 10: Fill<10>(xs, nPts); break;
		case 11: Fill<11>(xs, nPts); break;
		case 12: Fill<12>(xs, nPts); break;
		default: printf("Error!\n");	exit(0);
		}
	}

private:

	template<ushort MIN_DIST>	inline bool IsOutside(const ushort &x, const ushort &y)
	{
		return x < MIN_DIST || y < MIN_DIST || x + MIN_DIST >= m_width || y + MIN_DIST >= m_height;
	}
	template<ushort MIN_DIST>	inline void FillFast(const ushort &x, const ushort &y);
	template<ushort MIN_DIST>	inline void FillPartial(const ushort &x, const ushort &y)
	{
		const ushort x1 = x < MIN_DIST ? 0 : x - MIN_DIST;
		const ushort x2 = x + MIN_DIST < m_width ? x + MIN_DIST : m_width - 1;
		const ushort y1 = y < MIN_DIST ? 0 : y - MIN_DIST;
		const ushort y2 = y + MIN_DIST < m_height ? y + MIN_DIST : m_height - 1;
		for(ushort y = y1; y <= y2; ++y)
		for(ushort x = x1; x <= x2; ++x)
			m_rows[y][x] = UCHAR_MAX;
	}

private:

	ushort m_width, m_height, m_stride, m_strideInv8, m_strideInv4, m_strideInv2;
	uint m_nBytes, m_nMaxBytes;
	std::vector<ubyte *> m_rows;
};

template<> inline void FillMap::FillFast<0>(const ushort &x, const ushort &y)
{
	m_rows[y][x] = UCHAR_MAX;
}

template<> inline void FillMap::FillFast<1>(const ushort &x, const ushort &y)
{
	ushort *ptr1 = (ushort *) &m_rows[y - 1][x - 1];
	ubyte  *ptr2 = (ubyte  *) ptr1 + 2;
	REPEAT_3_TIMES(*ptr1 = USHRT_MAX;	ptr1 += m_strideInv2;
				   *ptr2 = UCHAR_MAX;	ptr2 += m_stride;)
}

template<> inline void FillMap::FillFast<2>(const ushort &x, const ushort &y)
{
	uint  *ptr1 = (uint  *) &m_rows[y - 2][x - 2];
	ubyte *ptr2 = (ubyte *) ptr1 + 4;
	REPEAT_5_TIMES(*ptr1 = UINT_MAX;	ptr1 += m_strideInv4;
				   *ptr2 = UCHAR_MAX;	ptr2 += m_stride;)
}

template<> inline void FillMap::FillFast<3>(const ushort &x, const ushort &y)
{
	uint   *ptr1 = (uint   *) &m_rows[y - 3][x - 3];
	ushort *ptr2 = (ushort *) ptr1 + 2;
	ubyte  *ptr3 = (ubyte  *) ptr2 + 2;
	REPEAT_7_TIMES(*ptr1 = UINT_MAX;	ptr1 += m_strideInv4;
				   *ptr2 = USHRT_MAX;	ptr2 += m_strideInv2;
				   *ptr3 = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<4>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 4][x - 4];
	ubyte  *ptr2 = (ubyte  *) ptr1 + 8;
	REPEAT_9_TIMES(*ptr1 = ULLONG_MAX;	ptr1 += m_strideInv8;
				   *ptr2 = UCHAR_MAX;	ptr2 += m_stride;)
}

template<> inline void FillMap::FillFast<5>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 5][x - 5];
	ushort *ptr2 = (ushort *) ptr1 + 4;
	ubyte  *ptr3 = (ubyte  *) ptr2 + 2;
	REPEAT_11_TIMES(*ptr1 = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2 = USHRT_MAX;	ptr2 += m_strideInv2;
					*ptr3 = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<6>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 6][x - 6];
	uint   *ptr2 = (uint   *) ptr1 + 2;
	ubyte  *ptr3 = (ubyte  *) ptr2 + 4;
	REPEAT_13_TIMES(*ptr1 = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2 = UINT_MAX;	ptr2 += m_strideInv4;
					*ptr3 = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<7>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 7][x - 7];
	uint   *ptr2 = (uint   *) ptr1 + 2;
	ushort *ptr3 = (ushort *) ptr2 + 2;
	ubyte  *ptr4 = (ubyte  *) ptr3 + 2;
	REPEAT_15_TIMES(*ptr1 = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2 = UINT_MAX;	ptr2 += m_strideInv4;
					*ptr3 = USHRT_MAX;	ptr3 += m_strideInv2;
					*ptr4 = UCHAR_MAX;	ptr4 += m_stride;)
}

template<> inline void FillMap::FillFast<8>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 8][x - 8];
	ubyte  *ptr2 = (ubyte  *) ptr1 + 16;
	REPEAT_17_TIMES(ptr1[0] = ULLONG_MAX;
					ptr1[1] = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2   = UCHAR_MAX;	ptr2 += m_stride;)
}

template<> inline void FillMap::FillFast<9>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 9][x - 9];
	ushort *ptr2 = (ushort *) ptr1 + 8;
	ubyte  *ptr3 = (ubyte  *) ptr2 + 2;
	REPEAT_19_TIMES(ptr1[0] = ULLONG_MAX;
					ptr1[1] = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2   = USHRT_MAX;	ptr2 += m_strideInv2;
					*ptr3   = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<10>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 10][x - 10];
	uint   *ptr2 = (uint   *) ptr1 + 4;
	ubyte  *ptr3 = (ubyte  *) ptr2 + 4;
	REPEAT_21_TIMES(ptr1[0] = ULLONG_MAX;
					ptr1[1] = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2   = UINT_MAX;		ptr2 += m_strideInv4;
					*ptr3   = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<11>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 11][x - 11];
	uint   *ptr2 = (uint   *) ptr1 + 4;
	ushort *ptr3 = (ushort *) ptr2 + 2;
	ubyte  *ptr4 = (ubyte  *) ptr3 + 2;
	REPEAT_23_TIMES(ptr1[0] = ULLONG_MAX;
					ptr1[1] = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2   = UINT_MAX;		ptr2 += m_strideInv4;
					*ptr3   = USHRT_MAX;	ptr3 += m_strideInv2;
					*ptr4   = UCHAR_MAX;	ptr3 += m_stride;)
}

template<> inline void FillMap::FillFast<12>(const ushort &x, const ushort &y)
{
	ullong *ptr1 = (ullong *) &m_rows[y - 12][x - 12];
	ubyte  *ptr2 = (ubyte  *) ptr1 + 24;
	REPEAT_25_TIMES(ptr1[0] = ULLONG_MAX;
					ptr1[1] = ULLONG_MAX;
					ptr1[2] = ULLONG_MAX;	ptr1 += m_strideInv8;
					*ptr2   = UCHAR_MAX;	ptr2 += m_stride;)
}

#endif