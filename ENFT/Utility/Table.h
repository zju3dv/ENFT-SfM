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

#ifndef _TABLE_H_
#define _TABLE_H_

#include "LinearAlgebra/Vector2.h"

template<class DATA_TYPE, typename INDEX_TYPE>
class Table
{

public:

	Table() : m_nRows(0), m_nCols(0), m_nRowsxnCols(0), m_size(0), m_capacity(0) {}
	Table(const INDEX_TYPE &nRows, const INDEX_TYPE &nCols) : m_nRows(0), m_nCols(0), m_nRowsxnCols(0), m_size(0), m_capacity(0) { Resize(nRows, nCols); }
	~Table() { Clear(); }

	inline void operator = (const Table<DATA_TYPE, INDEX_TYPE> &src)
	{
		Resize(src.m_nRows, src.m_nCols);
		for(uint i = 0; i < m_nRowsxnCols; ++i)
			m_rows[0][i] = src.m_rows[0][i];
	}
	inline void Resize(const INDEX_TYPE nRows, const INDEX_TYPE nCols)
	{
		if(m_nRows == nRows && m_nCols == nCols)
			return;
		m_nRows = nRows;
		m_nCols = nCols;
		m_nRowsxnCols = nRows * nCols;
		m_size = sizeof(DATA_TYPE) * m_nRowsxnCols;
		if(m_size > m_capacity)
		{
			m_capacity = m_size;
			if(!m_rows.empty())
				Deallocate(m_rows[0]);
			m_rows.resize(nRows);
			m_rows[0] = Allocate(m_nRowsxnCols);
		}
		DATA_TYPE *dataBkp = m_rows[0];
		m_rows.resize(nRows);
		m_rows[0] = dataBkp;
		for(INDEX_TYPE row = 1; row < nRows; ++row)
			m_rows[row] = m_rows[row - 1] + nCols;
	}
	inline void Resize(const INDEX_TYPE nRows, const INDEX_TYPE nCols, DATA_TYPE *data)
	{
		if(m_nRows == nRows && m_nCols == nCols && m_rows[0] == data)
			return;
		m_nRows = nRows;
		m_nCols = nCols;
		m_nRowsxnCols = nRows * nCols;
		m_size = sizeof(DATA_TYPE) * m_nRowsxnCols;
		m_capacity = 0;
		m_rows.resize(nRows);
		m_rows[0] = data;
		for(INDEX_TYPE row = 1; row < nRows; ++row)
			m_rows[row] = m_rows[row - 1] + nCols;
	}

	inline void Clear()
	{
		if(m_capacity == 0)
			return;
		Deallocate(m_rows[0]);
		m_rows.resize(0);
		m_nRows = m_nCols = m_nRowsxnCols = m_size = m_capacity = 0;
	}

	inline void SetZero() { memset(m_rows[0], 0, m_size); }
	inline void GetSize(INDEX_TYPE &nRows, INDEX_TYPE &nCols) const { nRows = m_nRows; nCols = m_nCols; }
	inline const INDEX_TYPE& GetRowsNumber() const { return m_nRows; }
	inline const INDEX_TYPE& GetColsNumber() const { return m_nCols; }
	inline const DATA_TYPE* operator[] (const INDEX_TYPE &row) const
	{
#if _DEBUG
		assert(row < m_nRows);
#endif
		return m_rows[row];
	}
	inline DATA_TYPE* operator[] (const INDEX_TYPE &row)
	{
#if _DEBUG
		assert(row < m_nRows);
#endif
		return m_rows[row];
	}
	inline const DATA_TYPE& operator[] (const LA::Vector2<INDEX_TYPE> &x) const
	{
#if _DEBUG
		assert(x.v0() < m_nCols && x.v1() < m_nRows);
#endif
		return m_rows[x.y()][x.x()];
	}
	inline DATA_TYPE& operator[] (const LA::Vector2<INDEX_TYPE> &x)
	{
#if _DEBUG
		assert(x.v0() < m_nCols && x.v1() < m_nRows);
#endif
		return m_rows[x.v1()][x.v0()];
	}

	inline void Swap(Table<DATA_TYPE, INDEX_TYPE> &table)
	{
		m_rows.swap(table.m_rows);

		const INDEX_TYPE nRowsBkp = m_nRows, nColsBkp = m_nCols;
		const uint nRowsxnColsBkp = m_nRowsxnCols, sizeBkp = m_size, capacityBkp = m_capacity;

		m_nRows = table.m_nRows;
		m_nCols = table.m_nCols;
		m_nRowsxnCols = table.m_nRowsxnCols;
		m_size = table.m_size;
		m_capacity = table.m_capacity;

		table.m_nRows = nRowsBkp;
		table.m_nCols = nColsBkp;
		table.m_nRowsxnCols = nRowsxnColsBkp;
		table.m_size = sizeBkp;
		table.m_capacity = capacityBkp;
	}
	virtual void SaveDataB(FILE *fp) const
	{
		if(!m_rows.empty())
			fwrite(m_rows[0], sizeof(DATA_TYPE), m_nRowsxnCols, fp);
	}
	void SaveB(FILE *fp) const
	{
		fwrite(&m_nRows, sizeof(INDEX_TYPE), 1, fp);
		fwrite(&m_nCols, sizeof(INDEX_TYPE), 1, fp);
		SaveDataB(fp);
	}
	bool SaveB(const char *fileName) const
	{
		FILE *fp = fopen(fileName, "wb");
		if(!fp)
			return false;
		SaveB(fp);
		fclose(fp);
		return true;
	}
	virtual void LoadDataB(FILE *fp)
	{
		if(!m_rows.empty())
			fread(m_rows[0], sizeof(DATA_TYPE), m_nRowsxnCols, fp);
	}
	inline void LoadB(FILE *fp)
	{
		INDEX_TYPE nRows, nCols;
		fread(&nRows, sizeof(INDEX_TYPE), 1, fp);
		fread(&nCols, sizeof(INDEX_TYPE), 1, fp);
		Resize(nRows, nCols);
		LoadDataB(fp);
	}
	inline bool LoadB(const char *fileName)
	{
		FILE *fp = fopen(fileName, "rb");
		if(!fp)
			return false;
		LoadB(fp);
		fclose(fp);
		return true;
	}

protected:

	virtual DATA_TYPE* Allocate(const uint &count) const { return new DATA_TYPE [count]; }
	virtual void Deallocate(DATA_TYPE *p) const { delete[] p; }

protected:

	std::vector<DATA_TYPE *> m_rows;
	INDEX_TYPE m_nRows, m_nCols;
	uint m_nRowsxnCols, m_size, m_capacity;

};

#endif