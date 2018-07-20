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

#ifndef _HIERARCHICAL_KMEANS_H_
#define _HIERARCHICAL_KMEANS_H_

#include "KMeans.h"
#include "Utility/Utility.h"

template<class Point, ubyte Dimension, typename ElementType, typename ClusterIndexPerLevel>
class HierarchicalKMeans
{

public:

	HierarchicalKMeans() : m_nClustersPerLevel(2), m_maxNumLevels(20), m_nPtsPerClusterTh(1), m_avgDistortionTh(0.032f), 
						   m_kmMaxNumItersTotal(100), m_kmMaxNumItersCenterMoving(10), m_verbose(false) {}

	inline void Resize(const uint &nPts)
	{
		m_ptsBuffer[0].Resize(nPts);
		m_ptsBuffer[1].Resize(nPts);
		m_iPtsBuffer[0].resize(nPts);
		m_iPtsBuffer[1].resize(nPts);
	}
	inline const Point& operator[] (const uint &iPt) const { return m_ptsBuffer[0][iPt]; }
	inline		 Point& operator[] (const uint &iPt)	   { return m_ptsBuffer[0][iPt]; }

	inline void RunHierarchicalLloyd(std::vector<KMeansCluster> &clusters)
	{
		m_km.m_nClusters = m_nClustersPerLevel;
		m_km.m_maxNumItersTotal = m_kmMaxNumItersTotal;
		m_km.m_maxNumItersCenterMoving = m_kmMaxNumItersCenterMoving;

		const uint nPts = m_ptsBuffer[0].Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
			m_iPtsBuffer[0][iPt] = iPt;

		if(m_verbose)
			printf("Clustering %d points...\n", nPts);
		clusters.resize(0);
		RunHierarchicalLloydRecursive(0, 0, nPts, clusters);
		if(m_verbose)
			printf("\nTotal: %d clusters\n", clusters.size());
	}
public:
	std::vector<bool> clustered;
	ClusterIndexPerLevel m_nClustersPerLevel;
	uint m_maxNumLevels, m_nPtsPerClusterTh, m_kmMaxNumItersTotal, m_kmMaxNumItersCenterMoving;
	float m_avgDistortionTh;
	bool m_verbose;

private:

	inline void RunHierarchicalLloydRecursive(const uint iLevel, const uint iStart, const uint nPts, std::vector<KMeansCluster> &clusters)//0, 0, nPts, clusters
	{
		IO::Assert(iStart + nPts <= m_ptsBuffer[0].Size(), "iStart = %d, nPts = %d, m_ptsBuffer[0].Size() = %d\n", iStart, nPts, m_ptsBuffer[0].Size());

		const ubyte iBuffer = (iLevel & 1);
		AlignedVector<Point> pts;
		pts.Set(m_ptsBuffer[iBuffer].Data() + iStart, nPts);
		const uint *iPts = m_iPtsBuffer[iBuffer].data() + iStart;

		m_km.RunLloyd(pts, m_clusters);

		const ubyte iBufferChild = 1 - iBuffer;
		Descriptor *ptsChild = m_ptsBuffer[iBufferChild].Data() + iStart;
		uint *iPtsChild = m_iPtsBuffer[iBufferChild].data() + iStart;
		uint iPt, iPtChild = 0;
		std::vector<std::pair<uint, uint> > children;
		for(ClusterIndexPerLevel iCluster = 0; iCluster < m_nClustersPerLevel; ++iCluster)
		{
			KMeansCluster &cluster = m_clusters[iCluster];
			const uint nPtsChild = cluster.Size();

			if(nPtsChild >= m_nPtsPerClusterTh && cluster.GetAverageDistortion() < m_avgDistortionTh)
			{
				// Success
				for(uint i = 0; i < nPtsChild; ++i)
					cluster[i] = iPts[cluster[i]];
				clusters.push_back(cluster);
				if(m_verbose)
					printf("\r  Cluster %d: level = %d, #points = %d, average distortion = %f", clusters.size(), iLevel, nPtsChild, cluster.GetAverageDistortion());
			}
			else if(nPtsChild > m_nClustersPerLevel && cluster.GetAverageDistortion() > m_avgDistortionTh)
			{
				// Copy data
				children.push_back(std::make_pair(iStart + iPtChild, nPtsChild));
				for(uint i = 0; i < nPtsChild; ++i, ++iPtChild)
				{
					iPt = cluster[i];
					ptsChild[iPtChild] = pts[iPt];
					iPtsChild[iPtChild] = iPts[iPt];
				}
			}
		}
		const uint iLevelChild = iLevel + 1;
		if(iLevelChild == m_maxNumLevels || children.empty())
			return;

		const ClusterIndexPerLevel nChildren = ClusterIndexPerLevel(children.size());
		for(ClusterIndexPerLevel i = 0; i < nChildren; ++i)
			RunHierarchicalLloydRecursive(iLevelChild, children[i].first, children[i].second, clusters);
	}

private:

	KMeans<Point, Dimension, ElementType, ClusterIndexPerLevel> m_km;
	AlignedVector<Point> m_ptsBuffer[2];
	std::vector<uint> m_iPtsBuffer[2];
	std::vector<KMeansCluster> m_clusters;

};

#endif