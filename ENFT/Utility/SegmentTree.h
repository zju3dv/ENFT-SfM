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

#ifndef _SEGMENT_TREE_H_
#define _SEGMENT_TREE_H_

#include "Pool.h"

template<typename NumberType>
class Segment
{

public:

	Segment() {}
	Segment(const NumberType iStart, const NumberType iEnd) : m_iStart(iStart), m_iEnd(iEnd) {}
	inline void Set(const NumberType iStart, const NumberType iEnd) { m_iStart = iStart; m_iEnd = iEnd; }
	inline void Get(NumberType &iStart, NumberType &iEnd) const { iStart = m_iStart; iEnd = m_iEnd; }
	inline NumberType GetWidth() const { return m_iEnd - m_iStart + 1; }
	inline bool Include(const NumberType i) const { return i >= m_iStart && i <= m_iEnd; }

protected:

	NumberType m_iStart, m_iEnd;
};

template<typename NumberType, typename SegmentIndex>
class SegmentTree
{

public:

	inline void Initialize(const NumberType iMax)
	{
		m_iMax = iMax;
		m_nSegs = 1;
		m_nodePool.Initialize();
		m_nodes.resize(1);
		m_nodes[0] = m_nodePool.Create();
		m_nodes[0]->Initialize(0, iMax);
	}

	inline const SegmentIndex& GetSegmentsNumber() const { return m_nSegs; }

	inline bool SplitSegment(const NumberType i)
	{
		Node *node = SearchRecursive(m_nodes[0], i);
		if(!node)
			return false;
		NumberType iStart, iEnd;
		node->Get(iStart, iEnd);
		if(iStart == iEnd || i + 1 > iEnd)
			return false;
		
		Node *child = m_nodePool.Create();
		child->Initialize(iStart, i);
		node->SetLeftChild(child);
		m_nodes.push_back(child);

		child = m_nodePool.Create();
		child->Initialize(i + 1, iEnd);
		node->SetRightChild(child);
		m_nodes.push_back(child);

		++m_nSegs;
		return true;
	}

	inline void GetNumberToSegmentMap(std::vector<SegmentIndex> &mapNumToSeg) const
	{
		const Node *node;
		NumberType i, iStart, iEnd;
		SegmentIndex iSeg = 0;

		const int nNodes = int(m_nodes.size());
		for(int iNode = 0; iNode < nNodes; ++iNode)
		{
			node = m_nodes[iNode];
			if(!node->IsLeave())
				continue;
			node->Get(iStart, iEnd);
			for(i = iStart; i <= iEnd; ++i)
				mapNumToSeg[i] = iSeg;
			++iSeg;
		}

		mapNumToSeg.resize(m_iMax + 1);
		SegmentIndex iSegNew = 0;
		for(iStart = 0; iStart <= m_iMax;)
		{
			iSeg = mapNumToSeg[iStart];
			for(i = iStart; i <= m_iMax && mapNumToSeg[i] == iSeg; ++i)
				mapNumToSeg[i] = iSegNew;
			++iSegNew;
			iStart = i;
		}
	}

protected:

	class Node : public Segment<NumberType>
	{
	public:
		inline void Initialize(const NumberType iStart, const NumberType iEnd) { Set(iStart, iEnd); m_childLeft = m_childRight = NULL; }
		inline void SetLeftChild(Node *child) { m_childLeft = child; }
		inline void SetRightChild(Node *child) { m_childRight = child; }
		inline Node* GetLeftChild() { return m_childLeft; }
		inline Node* GetRightChild() { return m_childRight; }
		inline bool IsLeave() const { return !m_childLeft && !m_childRight; }
	protected:
		Node *m_childLeft, *m_childRight;
	};

	inline Node* SearchRecursive(Node *node, const NumberType i)
	{
		if(!node->Include(i))
			return NULL;
		else if(node->IsLeave())
			return node;
		else if(node->GetLeftChild()->Include(i))
			return SearchRecursive(node->GetLeftChild(), i);
		else if(node->GetRightChild()->Include(i))
			return SearchRecursive(node->GetRightChild(), i);
		else
			return NULL;
	}

protected:

	NumberType m_iMax;
	SegmentIndex m_nSegs;
	Pool<Node> m_nodePool;
	std::vector<Node *> m_nodes;

};

#endif