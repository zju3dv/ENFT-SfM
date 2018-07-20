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

#ifndef _ALIGNED_TABLE_H_
#define _ALIGNED_TABLE_H_

#include "Table.h"
#include "SSE.h"

template<class DATA_TYPE, typename INDEX_TYPE>
class AlignedTable : public Table<DATA_TYPE, INDEX_TYPE>
{

public:

	AlignedTable() : Table<DATA_TYPE, INDEX_TYPE>() {}
	AlignedTable(const INDEX_TYPE &nRows, const INDEX_TYPE &nCols) : Table<DATA_TYPE, INDEX_TYPE>(nRows, nCols) {}
	~AlignedTable() { Clear(); }

protected:

	virtual DATA_TYPE* Allocate(const uint &count) const
	{
		DATA_TYPE* p = (DATA_TYPE*) _aligned_malloc(count * sizeof(DATA_TYPE), SSE_ALIGNMENT);
		if(p == NULL) throw std::bad_alloc();
		return p;
	}
	virtual void Deallocate(DATA_TYPE* p) const { _aligned_free(p); }

};

#endif