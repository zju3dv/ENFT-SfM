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

#ifndef _KC_TREE_H_
#define _KC_TREE_H_

template<class Point>
class KCTreeCluster
{
public:
	KCTreeCluster() : m_center(*(Point *) _aligned_malloc(sizeof(Point), SSE_ALIGNMENT)), m_sum(*(Point *) _aligned_malloc(sizeof(Point), SSE_ALIGNMENT)) {}
	~KCTreeCluster() { _aligned_free(&m_center); _aligned_free(&m_sum); }
	inline void ClearPoints() { memset(&m_sum, 0, sizeof(Point)); m_dotSum = 0; m_nPts = 0; }
	inline void SetCenter(const Point &center) { m_center = center; }
	inline const Point& GetCenter() const { return m_center; }
	inline const uint& GetPointsNumber() const { return m_nPts; }
	inline const float& GetDistortion() const { return m_distortion; }
	inline void operator = (const KCTreeCluster &cluster)
	{
		m_center = cluster.m_center;
		m_sum = cluster.m_sum;
		m_dotSum = cluster.m_dotSum;
		m_distortion = cluster.m_distortion;
		m_nPts = cluster.m_nPts;
	}
	inline void AddPoints(const Point &sum, const float &dotSum, const uint nPts)
	{
		Point::ApB(sum, m_sum, m_sum); 
		m_dotSum = dotSum + m_dotSum;
		m_nPts += nPts;
	}
	inline void ComputeDistortion()
	{
		const float ccDot = Point::Dot(m_center, m_center);
		const float csDot = Point::Dot(m_center, m_sum);
		m_distortion = m_dotSum - (csDot + csDot) + m_nPts * ccDot;
	}
	inline void MoveCenterToCentroid() { Point::sA(1.0f / m_nPts, m_sum, m_center); }
private:
	Point &m_center, &m_sum;
	float m_dotSum, m_distortion;
	uint m_nPts;
};

template<class Point, ubyte Dimension, typename ElementType, typename ClusterIndex>
class KCTree
{

public:

	~KCTree()
	{
		const uint bufferSizeBranch = uint(m_bufferBranch.size());
		for(uint i = 0; i < bufferSizeBranch; ++i)
			delete m_bufferBranch[i];
		const uint bufferSizeLeaf = uint(m_bufferLeaf.size());
		for(uint i = 0; i < bufferSizeLeaf; ++i)
			delete m_bufferLeaf[i];
	}

	inline void Build(const AlignedVector<Point> &pts)
	{
		m_iBufferBranch = m_iBufferLeaf = 0;
		const uint nPts = pts.Size();
		m_iPts.resize(nPts);
		for(uint iPt = 0; iPt < nPts; ++iPt)
			m_iPts[iPt] = iPt;
		m_work.Resize(3);
		m_root = (BranchNode *) BuildRecursive(pts, nPts, m_iPts.data());
	}

	inline void AssignPointsToClusters(const std::vector<KCTreeCluster<Point> *> &clusters)
	{
		const ClusterIndex nClusters = ClusterIndex(clusters.size());
		m_iClusters.resize(nClusters);
		for(ClusterIndex i = 0; i < nClusters; ++i)
		{
			m_iClusters[i] = i;
			clusters[i]->ClearPoints();
		}
		m_root->AssignPointsToClustersRecursive(clusters, m_iClusters, m_work[0]);
	}

	inline void AssignPointsToClusters(const std::vector<KCTreeCluster<Point> *> &clusters, std::vector<ClusterIndex> &mapPtToCluster)
	{
		const ClusterIndex nClusters = ClusterIndex(clusters.size());
		m_iClusters.resize(nClusters);
		for(ClusterIndex i = 0; i < nClusters; ++i)
			m_iClusters[i] = i;
		mapPtToCluster.resize(m_root->GetPointsNumber());
		m_root->AssignPointsToClustersRecursive(clusters, m_iClusters, mapPtToCluster, m_work[0]);
	}

private:

	class Node
	{
	public:
		Node() : m_sum(*(Point *) _aligned_malloc(sizeof(Point), SSE_ALIGNMENT)) {}
		~Node()	{ _aligned_free(&m_sum); }
		inline const Point& GetSum() const { return m_sum; }
		inline const float& GetDotSum() const { return m_dotSum; }
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, Point &work) const = 0;
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, 
			std::vector<ClusterIndex> &mapPtToCluster, Point &work) const = 0;
	protected:
		Point &m_sum;
		float m_dotSum;
	};
	class BranchNode : public Node
	{
	public:
		BranchNode() : Node(), m_bbLo(*(Point *) _aligned_malloc(sizeof(Point), SSE_ALIGNMENT)), m_bbHi(*(Point *) _aligned_malloc(sizeof(Point), SSE_ALIGNMENT)) {}
		~BranchNode() { _aligned_free(&m_bbLo); _aligned_free(&m_bbHi); }
		inline const uint& GetPointsNumber() const { return m_nPts; }
		inline void SetCell(const Point &bbLo, const Point &bbHi, const uint nPts)
		{
			m_bbLo = bbLo;
			m_bbHi = bbHi;
			m_nPts = nPts;
		}
		inline void SetChildren(const Node* const &childLeft, const Node* const &childRight)
		{
			m_childLeft = childLeft;
			m_childRight = childRight;
			if(m_childLeft && m_childRight)
			{
				Point::ApB(m_childLeft->GetSum(), m_childRight->GetSum(), this->m_sum);
				this->m_dotSum = m_childLeft->GetDotSum() + m_childRight->GetDotSum();
			}
			else if(m_childLeft)
			{
				this->m_sum = m_childLeft->GetSum();
				this->m_dotSum = m_childLeft->GetDotSum();
			}
			else if(m_childRight)
			{
				this->m_sum = m_childRight->GetSum();
				this->m_dotSum = m_childRight->GetDotSum();
			}
			else
			{
				printf("...\n");
				exit(0);
			}
		}
		inline void FilterClusters(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, 
								   std::vector<ClusterIndex> &iClustersRemain, Point &work) const
		{
			// Compute the closest center to the bounding box
			Point &bbMid = work;
			Point::ApB(m_bbLo, m_bbHi, bbMid);
			bbMid.Scale(0.5f);

			ClusterIndex iCluster, iClusterClosest = iClusters[0];
			float distSq, distSqMin = Point::SquaredDistance(clusters[iClusterClosest]->GetCenter(), bbMid);
			const ClusterIndex nClusters = ClusterIndex(iClusters.size());
			for(ClusterIndex i = 1; i < nClusters; ++i)
			{
				iCluster = iClusters[i];
				if((distSq = Point::SquaredDistance(clusters[iCluster]->GetCenter(), bbMid)) < distSqMin)
				{
					distSqMin = distSq;
					iClusterClosest = iCluster;
				}
			}
			const KCTreeCluster<Point> *clusterClosest = clusters[iClusterClosest];

			iClustersRemain.resize(0);
			const ubyte dim = Point::GetDimension();
			for(ClusterIndex i = 0; i < nClusters; ++i)
			{
				iCluster = iClusters[i];
				if(iCluster == iClusterClosest)
					iClustersRemain.push_back(iCluster);
				else
				{
					float ccDot, pcDot = 0;
					Point &ccDif = work;
					Point::AmB(clusters[iCluster]->GetCenter(), clusterClosest->GetCenter(), ccDif);
					ccDot = Point::Dot(ccDif, ccDif);
					for(ubyte d = 0; d < dim; ++d)
					{
						if(ccDif[d] > 0)
							pcDot = (m_bbHi[d] - clusterClosest->GetCenter()[d]) * ccDif[d] + pcDot;
						else
							pcDot = (m_bbLo[d] - clusterClosest->GetCenter()[d]) * ccDif[d] + pcDot;
					}
					if(ccDot < pcDot + pcDot)
						iClustersRemain.push_back(iCluster);
					// else
					// if no part of C is closer to z than it is to z*, we can infer that z is not the nearest center to
					// any data point associated with u and, hence, we can prune, or filter z from the list of candidates
				}
			}
		}
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, Point &work) const
		{
			if(iClusters.size() == 1)
			{
				// If u is associated with a single candidate (which must be z*) then z	is the nearest neighbor of all its data points
				clusters[iClusters[0]]->AddPoints(this->m_sum, this->m_dotSum, m_nPts);
			}
			else
			{
				std::vector<ClusterIndex> iClustersChild;
				FilterClusters(clusters, iClusters, iClustersChild, work);
				if(m_childLeft)
					m_childLeft->AssignPointsToClustersRecursive(clusters, iClustersChild, work);
				if(m_childRight)
					m_childRight->AssignPointsToClustersRecursive(clusters, iClustersChild, work);
			}
		}
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, 
													 std::vector<ClusterIndex> &mapPtToCluster, Point &work) const
		{
			if(iClusters.size() == 1)
			{
				// If u is associated with a single candidate (which must be z*) then z	is the nearest neighbor of all its data points
				if(m_childLeft)
					m_childLeft->AssignPointsToClustersRecursive(clusters, iClusters, mapPtToCluster, work);
				if(m_childRight)
					m_childRight->AssignPointsToClustersRecursive(clusters, iClusters, mapPtToCluster, work);
			}
			else
			{
				std::vector<ClusterIndex> iClustersChild;
				FilterClusters(clusters, iClusters, iClustersChild, work);
				if(m_childLeft)
					m_childLeft->AssignPointsToClustersRecursive(clusters, iClustersChild, mapPtToCluster, work);
				if(m_childRight)
					m_childRight->AssignPointsToClustersRecursive(clusters, iClustersChild, mapPtToCluster, work);
			}
		}
	protected:
		Point &m_bbLo, &m_bbHi;
		const Node *m_childLeft, *m_childRight;
		uint m_nPts;
	};
	class LeafNode : public Node
	{
	public:
		inline void SetPoint(const AlignedVector<Point> &pts, const uint iPt)
		{
			m_iPt = iPt;
			this->m_sum = pts[iPt];
			this->m_dotSum = Point::Dot(this->m_sum, this->m_sum);
		}
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, Point &work) const
		{
			const ClusterIndex nClusters = ClusterIndex(iClusters.size());
			if(nClusters == 1)
			{
				clusters[iClusters[0]]->AddPoints(this->m_sum, this->m_dotSum, 1);
				return;
			}
			const Point &pt = this->m_sum;
			ClusterIndex iCluster, iClusterClosest = iClusters[0];
			float distSq, distSqMin = Point::SquaredDistance(clusters[iClusterClosest]->GetCenter(), pt);
			for(ClusterIndex i = 1; i < nClusters; ++i)
			{
				iCluster = iClusters[i];
				if((distSq = Point::SquaredDistance(clusters[iCluster]->GetCenter(), pt)) < distSqMin)
				{
					distSqMin = distSq;
					iClusterClosest = iCluster;
				}
			}
			clusters[iClusterClosest]->AddPoints(this->m_sum, this->m_dotSum, 1);
		}
		virtual void AssignPointsToClustersRecursive(const std::vector<KCTreeCluster<Point> *> &clusters, const std::vector<ClusterIndex> &iClusters, 
													 std::vector<ClusterIndex> &mapPtToCluster, Point &work) const
		{
			const ClusterIndex nClusters = ClusterIndex(iClusters.size());
			if(nClusters == 1)
			{
				mapPtToCluster[m_iPt] = iClusters[0];
				return;
			}
			const Point &pt = this->m_sum;
			ClusterIndex iCluster, iClusterClosest = iClusters[0];
			float distSq, distSqMin = Point::SquaredDistance(clusters[iClusterClosest]->GetCenter(), pt);
			for(ClusterIndex i = 1; i < nClusters; ++i)
			{
				iCluster = iClusters[i];
				if((distSq = Point::SquaredDistance(clusters[iCluster]->GetCenter(), pt)) < distSqMin)
				{
					distSqMin = distSq;
					iClusterClosest = iCluster;
				}
			}
			mapPtToCluster[m_iPt] = iClusterClosest;
		}
	private:
		uint m_iPt;
	};

	Node* BuildRecursive(const AlignedVector<Point> &pts, const int nPts, uint *iPts)
	{
		if(nPts == 0)
			return NULL;
		else if(nPts == 1)
			return CreateLeafNode(pts, iPts[0]);

		// Sliding midpoint splitting rule
		Point &bbLo = m_work[0], &bbHi = m_work[1], &bbSize = m_work[2];
		Point::BoundingBox(pts, nPts, iPts, bbLo, bbHi);
		Point::AmB(bbHi, bbLo, bbSize);
		const ubyte cutDim = Point::GetMaximalElementDimension(bbSize);
		const ElementType cutVal = (bbLo[cutDim] + bbHi[cutDim]) * 0.5f;

		// Permute points
		//	pts[iPts[0..idx1-1]][cutDim] < cutVal
		//	pts[iPts[idx1..idx2-1]][cutDim] == cutVal
		//	pts[iPts[idx2..nPts-1]][cutDim] > cutVal
		int idx1 = 0, idx2 = nPts - 1;
		uint t;
		while(1)
		{
			while(idx1 < nPts && pts[iPts[idx1]][cutDim] < cutVal) ++idx1;
			while(idx2 >= 0 && pts[iPts[idx2]][cutDim] >= cutVal) --idx2;
			if(idx1 > idx2)
				break;
			SWAP(iPts[idx1], iPts[idx2], t);
			++idx1;
			--idx2;
		}
		const int cutIdx1 = idx1;
		idx2 = nPts - 1;
		while(1)
		{
			while(idx1 < nPts && pts[iPts[idx1]][cutDim] <= cutVal) ++idx1;
			while(idx2 >= cutIdx1 && pts[iPts[idx2]][cutDim] > cutVal) --idx2;
			if (idx1 > idx2) break;
			SWAP(iPts[idx1], iPts[idx2], t);
			++idx1;
			--idx2;
		}
		const int cutIdx2 = idx1;

		int nPtsLo;
		const int nPtsHalf = (nPts >> 1);
		if(cutIdx1 > nPtsHalf)
			nPtsLo = cutIdx1;
		else if(cutIdx2 < nPtsHalf)
			nPtsLo = cutIdx2;
		else
			nPtsLo = nPtsHalf;

		Node *node = CreateBranchNode(bbLo, bbHi, nPts);
		const Node *nodeLeft = BuildRecursive(pts, nPtsLo, iPts);
		const Node *nodeRight = BuildRecursive(pts, nPts - nPtsLo, iPts + nPtsLo);
		((BranchNode *) node)->SetChildren(nodeLeft, nodeRight);
		return node;
	}

	inline Node* CreateBranchNode(const Point &bbLo, const Point &bbHi, const uint nPts)
	{
		if(m_iBufferBranch == m_bufferBranch.size())
			m_bufferBranch.push_back(new BranchNode());
		m_bufferBranch[m_iBufferBranch]->SetCell(bbLo, bbHi, nPts);
		return (Node *) m_bufferBranch[m_iBufferBranch++];
	}
	inline Node* CreateLeafNode(const AlignedVector<Point> &pts, const uint iPt)
	{
		if(m_iBufferLeaf == m_bufferLeaf.size())
			m_bufferLeaf.push_back(new LeafNode());
		m_bufferLeaf[m_iBufferLeaf]->SetPoint(pts, iPt);
		return (Node *) m_bufferLeaf[m_iBufferLeaf++];
	}

private:

	BranchNode *m_root;

	AlignedVector<Point> m_work;
	std::vector<uint> m_iPts;
	std::vector<ClusterIndex> m_iClusters;

	uint m_iBufferBranch, m_iBufferLeaf;
	std::vector<BranchNode *> m_bufferBranch;
	std::vector<LeafNode *> m_bufferLeaf;
};

#endif
