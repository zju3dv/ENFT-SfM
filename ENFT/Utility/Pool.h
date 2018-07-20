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

#ifndef _POOL_H_
#define _POOL_H_

template<class TYPE>
class Pool
{

public:

	Pool() {}
	~Pool()
	{
		const int N = int(m_data.size());
		for(int i = 0; i < N; ++i)
			delete m_data[i];
	}

	inline void Initialize()
	{
		m_cnt = 0;
	}

	inline TYPE* Create()
	{
		if(m_data.size() < m_cnt + 1)
			m_data.push_back(new TYPE);
		return m_data[m_cnt++];
	}

	inline void Clear()
	{
		const int N = int(m_data.size());
		for(int i = 0; i < N; ++i)
			delete m_data[i];
		m_data.resize(0);
	}

protected:

	std::vector<TYPE *> m_data;
	int m_cnt;

};

#endif