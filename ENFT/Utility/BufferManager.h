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

#ifndef _BUFFER_H_
#define _BUFFER_H_

#include "Utility/Utility.h"

template<typename DataIndex, typename BufferIndex>
class BufferManager
{

public:

	inline void Initialize(const DataIndex &dataSize, const BufferIndex &bufferSize)
	{
		m_dataSize = dataSize;
		m_bufferSize = bufferSize;
		m_mapBufferToData.assign(bufferSize, dataSize);
		m_mapDataToBuffer.assign(dataSize, bufferSize);
		const DataIndex dataSizePreBuffered = std::min(dataSize, bufferSize);
		for(DataIndex iData = 0; iData < dataSizePreBuffered; ++iData)
		{
			m_mapBufferToData[iData] = iData;
			m_mapDataToBuffer[iData] = iData;
		}
		m_iDataToBeSwappedOut = 0;
	}
	inline const BufferIndex& GetDataBufferIndex(const DataIndex &iData) const { return m_mapDataToBuffer[iData]; }
	inline const DataIndex& GetToBeSwappedOutDataIndex() const { return m_iDataToBeSwappedOut; }
	inline bool IsDataBuffered(const DataIndex &iData) const { return m_mapDataToBuffer[iData] < m_bufferSize; }
	inline void BufferData(const DataIndex &iData)
	{
#if _DEBUG
		assert(!IsDataBuffered(iData));
#endif
		const BufferIndex iBuffer = m_mapDataToBuffer[m_iDataToBeSwappedOut];
		m_mapDataToBuffer[m_iDataToBeSwappedOut] = m_bufferSize;
		m_mapBufferToData[iBuffer] = iData;
		m_mapDataToBuffer[iData] = iBuffer;
		for(++m_iDataToBeSwappedOut; !IsDataBuffered(m_iDataToBeSwappedOut); ++m_iDataToBeSwappedOut);
		if(m_iDataToBeSwappedOut > iData)
			m_iDataToBeSwappedOut = iData;
	}
	inline void SaveB(FILE *fp) const
	{
		fwrite(&m_dataSize, sizeof(DataIndex), 1, fp);
		fwrite(&m_iDataToBeSwappedOut, sizeof(DataIndex), 1, fp);
		fwrite(&m_bufferSize, sizeof(BufferIndex), 1, fp);
		IO::VectorSaveB(m_mapBufferToData, fp);
		IO::VectorSaveB(m_mapDataToBuffer, fp);
	}
	inline void LoadB(FILE *fp)
	{
		fread(&m_dataSize, sizeof(DataIndex), 1, fp);
		fread(&m_iDataToBeSwappedOut, sizeof(DataIndex), 1, fp);
		fread(&m_bufferSize, sizeof(BufferIndex), 1, fp);
		IO::VectorLoadB(m_mapBufferToData, fp);
		IO::VectorLoadB(m_mapDataToBuffer, fp);
	}

private:

	DataIndex m_dataSize, m_iDataToBeSwappedOut;
	BufferIndex m_bufferSize;
	std::vector<DataIndex> m_mapBufferToData;
	std::vector<BufferIndex> m_mapDataToBuffer;
};

#endif