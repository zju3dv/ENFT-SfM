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

#ifndef _SPARSE_MATRIX_H_
#define _SPARSE_MATRIX_H_

#include "SparseVector.h"

template<class TYPE>
class SparseMatrix
{

public:

	SparseMatrix() : m_nRows(0), m_nCols(0) {}
	SparseMatrix(const uint &nRows, const uint &nCols) { Resize(nRows, nCols); }
	inline void Resize(const uint &nRows, const uint &nCols)
	{
		if(nRows != m_nRows)
			m_rows.resize(nRows);
		m_nRows = nRows;
		m_nCols = nCols;
	}
	inline const uint& GetRowsNumber() const { return m_nRows; }
	inline const TYPE* Get(const uint &i, const uint &j) const
	{
#if _DEBUG
		assert(j < m_nCols);
#endif
		return m_rows[i].Get(j);
	}
	inline TYPE* Get(const uint &i, const uint &j)
	{
#if _DEBUG
		assert(j < m_nCols);
#endif
		return m_rows[i].Get(j);
	}
//	inline void Insert(const uint &i, const uint &j, const TYPE &val)
//	{
//#if _DEBUG
//		assert(Get(i, j) == NULL);
//#endif
//		m_rows[i].Insert(j, val);
//	}
	inline TYPE* Insert(const uint &i, const uint &j, const TYPE &val)
	{
#if _DEBUG
		assert(Get(i, j) == NULL);
#endif
		return m_rows[i].Insert(j, val);
	}
	inline const FullIndexValueList<TYPE>& GetRowData(const uint &i) const { return m_rows[i].GetData(); }
	inline		 FullIndexValueList<TYPE>& GetRowData(const uint &i)	   { return m_rows[i].GetData(); }
	inline void Clear()
	{
		for(uint i = 0; i < m_nRows; ++i)
			m_rows[i].Clear();
	}
	inline bool SaveB(const char *fileName) const
	{
		FILE *fp = fopen(fileName, "wb");
		if(!fp)
			return false;
		fwrite(&m_nRows, sizeof(uint), 1, fp);
		fwrite(&m_nCols, sizeof(uint), 1, fp);
		for(uint i = 0; i < m_nRows; ++i)
			m_rows[i].SaveB(fp);
		fclose(fp);
		return true;
	}
	inline bool LoadB(const char *fileName)
	{
		FILE *fp = fopen(fileName, "rb");
		if(!fp)
			return false;
		fread(&m_nRows, sizeof(uint), 1, fp);
		fread(&m_nCols, sizeof(uint), 1, fp);
		m_rows.resize(m_nRows);
		for(uint i = 0; i < m_nRows; ++i)
			m_rows[i].LoadB(fp);
		fclose(fp);
		return true;
	}

private:

	uint m_nRows, m_nCols;
	std::vector<SparseVector<TYPE> > m_rows;

};

#endif