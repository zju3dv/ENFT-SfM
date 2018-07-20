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

#ifndef _ALIGNED_VECTOR_H_
#define _ALIGNED_VECTOR_H_

#include <exception>
#include "SSE.h"

template <class TYPE, uint GROWTH = 0> class AlignedVector
{

public:

	inline AlignedVector()
	{
		m_owner = true;
		m_data = NULL;
		m_size = m_capacity = 0;
	}
	inline AlignedVector(const uint &count) 
	{
		m_data = Allocate(count);
		m_size = m_capacity = count;
		m_owner = true;
	}
	inline AlignedVector(const AlignedVector<TYPE, GROWTH> &src)
	{
		m_data = Allocate(src.Size());
		m_size = m_capacity = src.Size();
		m_owner = true;
		CopyFrom(src.Data());
	}
	inline ~AlignedVector()
	{
		if(m_data && m_owner)
			Deallocate(m_data);
	}
	inline void operator = (const AlignedVector<TYPE, GROWTH> &src)
	{
		Resize(src.Size());
		CopyFrom(src.Data());
	}
	template<uint _GROWTH>
	inline void operator = (const AlignedVector<TYPE, _GROWTH> &src)
	{
		Resize(src.Size());
		CopyFrom(src.Data());
	}
	inline void Resize(const uint &count)
	{
		if(count <= m_capacity)
			m_size = count;
		else
		{
			if(m_data && m_owner)
				Deallocate(m_data);
			else
				m_owner = true;
			m_data = Allocate(count);
			m_size = m_capacity = count;
		}
	}
	inline void Reserve(const uint &capacity)
	{
		Clear();
		m_data = Allocate(capacity);
		m_capacity = capacity;
	}
	inline void Clear()
	{
		if(m_data && m_owner)
			Deallocate(m_data);
		m_owner = true;
		m_data = NULL;
		m_size = m_capacity = 0;
	}
	inline bool Empty() const { return m_size == 0; }
	inline void PushBack(const TYPE &val)
	{
		if(m_size == m_capacity)
		{
#if _DEBUG
			assert(GROWTH != 0);
#endif
			TYPE *dataBkp = m_data;
			m_capacity += GROWTH;
			m_data = Allocate(m_capacity);
			if(dataBkp)
			{
				memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
				if(m_owner)
					Deallocate(dataBkp);
			}
			m_data[m_size++] = val;
			m_owner = true;
		}
		else
			m_data[m_size++] = val;
	}
	inline void PushBack(const TYPE *data, const uint &size)
	{
		if(m_size + size > m_capacity)
		{
			TYPE *dataBkp = m_data;
			m_capacity += size + GROWTH - (GROWTH % size);
			m_data = Allocate(m_capacity);
			if(dataBkp)
			{
				memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
				if(m_owner)
					Deallocate(dataBkp);
			}
			m_owner = true;
		}
		memcpy(m_data + m_size, data, sizeof(TYPE) * size);
		m_size += size;
	}
	inline void EnlargeCapacity(const uint &capacity)
	{
		if(capacity <= m_capacity)
			return;
		m_capacity = capacity;
		TYPE *dataBkp = m_data;
		m_data = Allocate(m_capacity);
		if(dataBkp)
		{
			memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
			if(m_owner)
				Deallocate(dataBkp);
		}
		m_owner = true;
	}
	inline void SetZero() { memset(m_data, 0, sizeof(TYPE) * m_size); }
	inline void CopyFrom(const AlignedVector<TYPE, GROWTH> &src) { memcpy(m_data, src.m_data, sizeof(TYPE) * m_size); }
	inline void CopyFrom(const AlignedVector<TYPE, GROWTH> &src, const uint &iSrcStart, const uint &iDstStart, const uint &cpySize)
	{
		memcpy(m_data + iDstStart, src.m_data + iSrcStart, sizeof(TYPE) * cpySize);
	}
	inline void CopyFrom(const TYPE *src, const uint &size) { memcpy(m_data, src, sizeof(TYPE) * size); }
	inline void CopyFrom(const TYPE *src) { memcpy(m_data, src, sizeof(TYPE) * m_size); }
	inline void CopyTo(TYPE *dst) const { memcpy(dst, m_data, sizeof(TYPE) * m_size); }
	inline void Set(TYPE *data, const uint &count)
	{
		if(m_data && m_owner) Deallocate(m_data);
		m_data = data;
		m_owner = false;
		m_size = count;// * sizeof(float) / sizeof(TYPE);
		m_capacity = m_size;
	}
	inline void Set(const AlignedVector<TYPE, GROWTH> &src, const std::vector<ushort> &idxs)
	{
		const ushort N = ushort(idxs.size());
		Resize(N);
		for(ushort i = 0; i < N; ++i)
			m_data[i] = src[idxs[i]];
	}
	inline void BoundBy(AlignedVector<float>& vec) { vec.Set((float *) m_data, m_size * sizeof(TYPE) / sizeof(float)); }
	inline void Swap(AlignedVector<TYPE, GROWTH>& vec)
	{
		bool ownerBkp = m_owner;
		TYPE* dataBkp = m_data;
		uint sizeBkp = m_size;
		uint capacityBkp = m_capacity;

		m_owner = vec.m_owner;
		m_data = vec.m_data;
		m_size = vec.m_size;
		m_capacity = vec.m_capacity;

		vec.m_owner = ownerBkp;
		vec.m_data = dataBkp;
		vec.m_size = sizeBkp;
		vec.m_capacity = capacityBkp;
	}
	inline void Concatenate(const AlignedVector<TYPE> &vec1, const AlignedVector<TYPE> &vec2)
	{
		Resize(vec1.Size() + vec2.Size());
		memcpy(m_data, vec1.Begin(), sizeof(TYPE) * vec1.Size());
		memcpy(m_data + vec1.Size(), vec2.Begin(), sizeof(TYPE) * vec2.Size());
	}

	inline const TYPE& operator[] (const uint &i) const
	{
#if _DEBUG
		assert(i < m_size);
#endif
		return m_data[i];
	}
	inline TYPE& operator[] (const uint &i)
	{
#if _DEBUG
		assert(i < m_size);
#endif
		return m_data[i];
	}
	inline const TYPE& At(const uint &i) const { return m_data[i]; }
	inline		 TYPE& At(const uint &i)	   { return m_data[i]; }
	inline const TYPE* Data() const { return m_data; }
	inline		 TYPE* Data()	    { return m_data; }
	inline const uint& Size() const { return m_size; }
	inline const uint& Capacity() const { return m_capacity; }
	inline void SaveB(FILE *fp) const
	{
		fwrite(&m_size, sizeof(uint), 1, fp);
		fwrite(m_data, sizeof(TYPE), m_size, fp);
	}
	inline bool LoadB(FILE *fp)
	{
		uint count;
		if(fread(&count, sizeof(uint), 1, fp) == 0)
			return false;
		Resize(count);
		fread(m_data, sizeof(TYPE), m_size, fp);
		return true;
	}
	inline bool SaveB(const char *fileName) const
	{
		FILE *fp = fopen( fileName, "wb");
		if(!fp)
			return false;
		SaveB(fp);
		fclose(fp);
		return true;
	}
	inline bool LoadB(const char *fileName)
	{
		FILE *fp = fopen( fileName, "rb");
		if(!fp)
			return false;
		LoadB(fp);
		fclose(fp);
		return true;
	}

	inline void Save(FILE *fp) const
	{
		const uint nFltsPerElements = sizeof(TYPE) / sizeof(float);
		fprintf(fp, "%d\n", m_size);
		const float *p = (float *) m_data;
		for(uint i = 0; i < m_size; ++i)
		{
			fprintf(fp, "%d", i);
			for(uint j = 0; j < nFltsPerElements; ++j, ++p)
				fprintf(fp, " %f", *p);
			fprintf(fp, "\n");
		}
	}
	inline bool Save(const char *fileName) const
	{
		FILE *fp = fopen( fileName, "w");
		if(!fp)
			return false;
		Save(fp);
		fclose(fp);
		return true;
	}

private:

	inline static TYPE* Allocate(const uint &count) { return (TYPE*) _aligned_malloc(count * sizeof(TYPE), SSE_ALIGNMENT); }
	inline static void Deallocate(TYPE* p) { _aligned_free(p); }

private:

	bool m_owner;
	TYPE* m_data;
	uint m_size, m_capacity;

};

template<class TYPE, uint N> class AlignedVectorN
{

public:

	AlignedVectorN() { m_data = (TYPE*) _aligned_malloc(sizeof(TYPE) * N, SSE_ALIGNMENT); }
	~AlignedVectorN() { _aligned_free(m_data); }

	inline const TYPE& operator[] (const uint &i) const
	{
#if _DEBUG
		assert(i < N);
#endif
		return m_data[i];
	}
	inline TYPE& operator[] (const uint &i)
	{
#if _DEBUG
		assert(i < N);
#endif
		return m_data[i];
	}
	inline const TYPE* Data() const { return m_data; }
	inline		 TYPE* Data()	    { return m_data; }

protected:

	TYPE *m_data;
};

#endif