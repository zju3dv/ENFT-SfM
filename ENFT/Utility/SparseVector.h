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

#ifndef _SPARSE_VECTOR_H_
#define _SPARSE_VECTOR_H_

#include <set>

template<class TYPE>
class FullIndexValue
{
public:
	FullIndexValue() {}
	FullIndexValue(const uint &idx) : m_idx(idx) {}
	FullIndexValue(const uint &idx, const TYPE &val) : m_idx(idx), m_val(val) {}
	inline const uint& GetFullIndex() const { return m_idx; }
	inline const TYPE& GetValue() const { return m_val; }
	inline		 TYPE& GetValue()		{ return m_val; }
	inline bool operator < (const FullIndexValue &idxVal) { return m_idx < idxVal.m_idx; }
private:
	uint m_idx;
	TYPE m_val;
};

template<class TYPE>
class FullIndexValueList : public std::vector<FullIndexValue<TYPE> > {};

template<class TYPE>
class SparseVector
{

private:

	typedef std::pair<uint, uint> FullSparseIndex;
	struct _Pr { bool operator() (const FullSparseIndex &lhs, const FullSparseIndex &rhs) const { return lhs.first < rhs.first; } };
	typedef std::set<FullSparseIndex, _Pr> FullSparseIndexSet;

public:

	inline const TYPE* Get(const uint &i) const
	{
		typename FullSparseIndexSet::const_iterator it = m_idxs.find(std::make_pair(i, 0));
#if _DEBUG
		if(it != m_idxs.end())
			assert(m_data[it->second].GetFullIndex() == i);
#endif
		if(it == m_idxs.end())
			return NULL;
		else
			return &m_data[it->second].GetValue();
	}
	inline TYPE* Get(const uint &i)
	{
		typename FullSparseIndexSet::const_iterator it = m_idxs.find(std::make_pair(i, 0));
#if _DEBUG
		if(it != m_idxs.end())
			assert(m_data[it->second].GetFullIndex() == i);
#endif
		if(it == m_idxs.end())
			return NULL;
		else
			return &m_data[it->second].GetValue();
	}
//	inline void Insert(const uint &i, const TYPE &val)
//	{
//#if _DEBUG
//		assert(Get(i) == NULL);
//#endif
//		m_idxs.insert(FullSparseIndex(i, m_data.size()));
//		m_data.push_back(FullIndexValue<TYPE>(i, val));
//	}
	inline TYPE* Insert(const uint &i, const TYPE &val)
	{
#if _DEBUG
		assert(Get(i) == NULL);
#endif
		m_idxs.insert(FullSparseIndex(i, m_data.size()));
		m_data.push_back(FullIndexValue<TYPE>(i, val));
		return &m_data.back().GetValue();
	}
	inline const FullIndexValueList<TYPE>& GetData() const { return m_data; }
	inline		 FullIndexValueList<TYPE>& GetData()	   { return m_data; }
	//inline void Clear() { m_idxs.clear(); m_data.clear(); }
	inline void Clear() { m_idxs.clear(); m_data.resize(0); }
	inline void SaveB(FILE *fp) const
	{
		const uint N = uint(m_data.size());
		fwrite(&N, sizeof(uint), 1, fp);
		if(N != 0)
			fwrite(&m_data[0], sizeof(FullIndexValue<TYPE>), N, fp);
	}
	inline void LoadB(FILE *fp)
	{
		uint N;
		fread(&N, sizeof(uint), 1, fp);
		m_data.resize(N);
		m_idxs.clear();
		if(N != 0)
		{
			fread(&m_data[0], sizeof(FullIndexValue<TYPE>), N, fp);
			for(uint i = 0; i < N; ++i)
				m_idxs.insert(FullSparseIndex(m_data[i].GetFullIndex(), i));
		}
	}

private:

	FullSparseIndexSet m_idxs;
	FullIndexValueList<TYPE> m_data;

};

#endif