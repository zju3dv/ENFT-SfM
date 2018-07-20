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

#ifndef _BUCKET_H_
#define _BUCKET_H_

#include "SfM/Point.h"

template <class TYPE> class Bucket
{

public:

	typedef std::vector<TYPE> Bin;

	Bucket() : m_width(0), m_height(0), m_nBinsX(0), m_nBinsY(0), m_nBins(0) {}

	inline void Initialize(const ushort &width, const ushort &height, const ushort &nBinsX, const ushort &nBinsY)
	{
		if(m_width == width && m_height == height && m_nBinsX == nBinsX && m_nBinsY == nBinsY)
		{
			//m_bins.assign(m_nBins, std::vector<TYPE>());
			for(uint iBin = 0; iBin < m_nBins; ++iBin)
				m_bins[iBin].resize(0);
			return;
		}
		m_width = width;
		m_height = height;
		m_nBinsX = nBinsX;
		m_nBinsY = nBinsY;
		m_nBins = uint(nBinsX) * uint(nBinsY);
		m_bins.assign(m_nBins, std::vector<TYPE>());

		m_iBinsX.resize(width + 1);
		const ushort binWidth = width / nBinsX;
		ushort x = 0;
		for(ushort i = 0; i < nBinsX; ++i)
		for(ushort j = 0; j < binWidth; j++, x++)
			m_iBinsX[x] = i;
		while(x <= width)
			m_iBinsX[x++] = nBinsX - 1;

		m_iBinsY.resize(height + 1);
		const ushort binHeight = height / nBinsY;
		ushort y = 0;
		int iBin = 0;
		for(uint i = 0; i < nBinsY; ++i, iBin += nBinsX)
		for(uint j = 0; j < binHeight; j++, y++)
			m_iBinsY[y] = iBin;
		iBin -= nBinsX;
		while(y <= height)
			m_iBinsY[y++] = iBin;
	}
	inline void ClearBins()
	{
		for(uint iBin = 0; iBin < m_nBins; ++iBin)
			m_bins[iBin].resize(0);
	}

	inline void Push(const ushort &x, const ushort &y, const TYPE &val) { m_bins[m_iBinsY[y] + m_iBinsX[x]].push_back(val); }
	inline void Push(const Point2D &x, const TYPE &val) { Push(ushort(x.x()), ushort(x.y()), val); }
	inline void GetBin(const float x, const float y, const float &range, Bin &bin) const
	{
		bin.resize(0);
		const short x1 = short(x - range), x2 = short(ceil(x + range));
		const short y1 = short(y - range), y2 = short(ceil(y + range));
		if(x1 >= m_width || x2 < 0 || y1 >= m_height || y2 < 0)
			return;
		const uint iBinX1 = m_iBinsX[std::max(x1, short(0))], iBinX2 = m_iBinsX[std::min(x2, short(m_width - 1))];
		const uint iBinY1 = m_iBinsY[std::max(y1, short(0))], iBinY2 = m_iBinsY[std::min(y2, short(m_height - 1))];
		//const int iBinX1 = m_iBinsX[x1], iBinX2 = m_iBinsX[x2];
		//const int iBinY1 = m_iBinsY[y1], iBinY2 = m_iBinsY[y2];
		for(uint iBinY = iBinY1; iBinY <= iBinY2; iBinY += m_nBinsX)
		{
			const uint iBin1 = iBinY + iBinX1, iBin2 = iBinY + iBinX2;
			for(uint iBin = iBin1; iBin <= iBin2; iBin++)
				bin.insert(bin.end(), m_bins[iBin].begin(), m_bins[iBin].end());
		}
	}
	inline void GetBin(const Point2D &x, const float &range, Bin &bin) const { GetBin(x.x(), x.y(), range, bin); }
	inline const uint& GetBinsNumber() const { return m_nBins; }
	inline const ushort& GetBinSizeX() const { return m_nBinsX; }
	inline const ushort& GetBinSizeY() const { return m_nBinsY; }
	inline const Bin& GetBin(const uint &iBin) const { return m_bins[iBin]; }

private:

	ushort m_width, m_height, m_nBinsX, m_nBinsY;
	uint m_nBins;
	std::vector<Bin> m_bins;
	std::vector<uint> m_iBinsX, m_iBinsY;

};

#endif