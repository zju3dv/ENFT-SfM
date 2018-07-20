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

#ifndef _KMEANS_H_
#define _KMEANS_H_

#include "KCTree.h"
#include "Utility/Random.h"
//#include "Utility/Utility.h"

class KMeansCluster
{
public:
	inline void Reserve(const uint &nPts) { m_iPts.reserve(nPts); }
	inline void Resize(const uint &nPts) { m_iPts.resize(nPts); }
	inline void PushBack(const uint &iPt) { m_iPts.push_back(iPt); }
	inline uint Size() const { return uint(m_iPts.size()); }
	inline void SetAverageDistortion(const float avgDistortion) { m_avgDistortion = avgDistortion; }
	inline const float& GetAverageDistortion() const { return m_avgDistortion; }
	inline const uint& operator[] (const uint &i) const { return m_iPts[i]; }
	inline		 uint& operator[] (const uint &i)		{ return m_iPts[i]; }
private:
	std::vector<uint> m_iPts;
	float m_avgDistortion;
};

template<class Point, ubyte Dimension, typename ElementType, typename ClusterIndex>
class KMeans
{

public:

	KMeans() : m_maxNumItersTotal(100), m_maxNumItersCenterMoving(5), m_stopAccumulatedRDL(0.1f) {}
	~KMeans()
	{
		const ClusterIndex bufferSize = ClusterIndex(m_buffer.size());
		for(ClusterIndex i = 0; i < bufferSize; ++i)
			delete m_buffer[i];
	}

	inline void RunLloyd(const AlignedVector<Point> &pts, std::vector<KMeansCluster> &clusters)
	{
		m_KCTree.Build(pts);

		std::vector<KCTreeCluster<Point> *> clustersCurrent, clustersBest;
		CreateClusters(clustersCurrent, clustersBest);

		SampleCenters(pts, clustersCurrent);
		m_KCTree.AssignPointsToClusters(clustersCurrent);
		//AssertAssignment(pts, clustersCurrent);
		float distortionCurrent = ComputeDistortion(clustersCurrent), distortionBest = distortionCurrent;
		SaveBestClusters(clustersCurrent, clustersBest);

		for(uint iterTotal = 0; iterTotal < m_maxNumItersTotal; ++iterTotal)
		{
			const float distortionInit = distortionCurrent;
			bool centerMovingImprove = false;
			for(uint iterCenterMoving = 0; iterCenterMoving < m_maxNumItersCenterMoving && iterTotal < m_maxNumItersTotal; ++iterCenterMoving, ++iterTotal)
			{
				MoveCentersToCentroids(clustersCurrent);
				m_KCTree.AssignPointsToClusters(clustersCurrent);
				//AssertAssignment(pts, clustersCurrent);
				distortionCurrent = ComputeDistortion(clustersCurrent);
				if(distortionCurrent < distortionBest)
				{
					distortionBest = distortionCurrent;
					SaveBestClusters(clustersCurrent, clustersBest);
				}

				const float accumulatedRDL = (distortionInit - distortionCurrent) / distortionInit;
				if(accumulatedRDL >= m_stopAccumulatedRDL)
				{
					centerMovingImprove = true;
					break;
				}
			}
			if(iterTotal == m_maxNumItersTotal)
				break;
			if(!centerMovingImprove)
			{
				SampleCenters(pts, clustersCurrent);
				m_KCTree.AssignPointsToClusters(clustersCurrent);
				//AssertAssignment(pts, clustersCurrent);
				distortionCurrent = ComputeDistortion(clustersCurrent);
				if(distortionCurrent < distortionBest)
				{
					distortionBest = distortionCurrent;
					SaveBestClusters(clustersCurrent, clustersBest);
				}
			}
		}

		clusters.resize(m_nClusters);
		for(ClusterIndex iCluster = 0; iCluster < m_nClusters; ++iCluster)
		{
			clusters[iCluster].Reserve(clustersBest[iCluster]->GetPointsNumber());
			clusters[iCluster].Resize(0);
			clusters[iCluster].SetAverageDistortion(clustersBest[iCluster]->GetDistortion() / clustersBest[iCluster]->GetPointsNumber());
		}

		m_KCTree.AssignPointsToClusters(clustersBest, m_mapPtToCluster);
		const uint nPts = uint(m_mapPtToCluster.size());
		for(uint iPt = 0; iPt < nPts; ++iPt)
			clusters[m_mapPtToCluster[iPt]].PushBack(iPt);
	}

public:

	ClusterIndex m_nClusters;
	uint m_maxNumItersTotal, m_maxNumItersCenterMoving;
	float m_stopAccumulatedRDL;

private:

	inline void CreateClusters(std::vector<KCTreeCluster<Point> *> &clustersCurrent, std::vector<KCTreeCluster<Point> *> &clustersBest)
	{
		const ClusterIndex bufferSizeCurrent = ClusterIndex(m_buffer.size());
		const ClusterIndex bufferSizeRequired = (m_nClusters << 1);
		if(bufferSizeCurrent < bufferSizeRequired)
		{
			for(ClusterIndex iBuffer = bufferSizeCurrent; iBuffer < bufferSizeRequired; ++iBuffer)
				m_buffer.push_back(new KCTreeCluster<Point>());
		}
		clustersCurrent.resize(m_nClusters);
		for(ClusterIndex i = 0; i < m_nClusters; ++i)
			clustersCurrent[i] = m_buffer[i];
		clustersBest.resize(m_nClusters);
		for(uint i = 0, j = m_nClusters; i < m_nClusters; ++i, ++j)
			clustersBest[i] = m_buffer[j];
	}
	inline void SaveBestClusters(const std::vector<KCTreeCluster<Point> *> &clustersCurrent, const std::vector<KCTreeCluster<Point> *> &clustersBest)
	{
		for(ClusterIndex iCluster = 0; iCluster < m_nClusters; ++iCluster)
			*clustersBest[iCluster] = *clustersCurrent[iCluster];
	}
	inline void SampleCenters(const AlignedVector<Point> &pts, const std::vector<KCTreeCluster<Point> *> &clusters)
	{
		ClusterIndex i, j;
		uint iPt;
		const ClusterIndex nCenters = ClusterIndex(clusters.size());
		const uint nPts = pts.Size();
		m_mapCenterToPt.resize(nCenters);
		for(i = 0; i < nCenters; ++i)
		{
			do
			{
				iPt = Random::GenerateUint(nPts);
				for(j = 0; j < i; ++j)
				{
					if(m_mapCenterToPt[j] == iPt)
						break;
				}
			} while(i != j);
			m_mapCenterToPt[i] = iPt;
			clusters[i]->SetCenter(pts[iPt]);
		}
	}
	inline void MoveCentersToCentroids(const std::vector<KCTreeCluster<Point> *> &clusters)
	{
		const ClusterIndex nClusters = ClusterIndex(clusters.size());
		for(ClusterIndex iCluster = 0; iCluster < nClusters; ++iCluster)
		{
			if(clusters[iCluster]->GetPointsNumber() > 0)
				clusters[iCluster]->MoveCenterToCentroid();
		}
	}
	inline float ComputeDistortion(const std::vector<KCTreeCluster<Point> *> &clusters)
	{
		float distortionTotal = 0;
		const ClusterIndex nClusters = ClusterIndex(clusters.size());
		for(ClusterIndex iCluster = 0; iCluster < nClusters; ++iCluster)
		{
			clusters[iCluster]->ComputeDistortion();
			distortionTotal = clusters[iCluster]->GetDistortion() + distortionTotal;
		}
		return distortionTotal;
	}

	//inline void AssertAssignment(const AlignedVector<Point> &pts, const std::vector<KCTreeCluster<Point> *> &clusters)
	//{
	//	m_KCTree.AssignPointsToClusters(clusters);
	//	m_KCTree.AssignPointsToClusters(clusters, m_mapPtToCluster);
	//	const uint nPts = pts.Size();
	//	IO::Assert(nPts == m_mapPtToCluster.size(), "nPts = %d, m_mapPtToCluster.size() = %d\n", nPts, m_mapPtToCluster.size());

	//	std::vector<float> distSqSum(m_nClusters, 0);
	//	std::vector<uint> ptCnts(m_nClusters, 0);

	//	float distSq, distSqMin;
	//	ClusterIndex iCluster, iClusterClosest;
	//	for(uint iPt = 0; iPt < nPts; ++iPt)
	//	{
	//		const Point &pt = pts[iPt];
	//		distSqMin = FLT_MAX;
	//		for(iCluster = 0; iCluster < m_nClusters; ++iCluster)
	//		{
	//			if((distSq = Point::SquaredDistance(pt, clusters[iCluster]->GetCenter())) < distSqMin)
	//			{
	//				distSqMin = distSq;
	//				iClusterClosest = iCluster;
	//			}
	//		}
	//		IO::Assert(iClusterClosest == m_mapPtToCluster[iPt], "iClusterClosest = %d, m_mapPtToCluster[iPt] = %d\n", iClusterClosest, m_mapPtToCluster[iPt]);

	//		distSqSum[iClusterClosest] += distSqMin;
	//		++ptCnts[iClusterClosest];
	//	}

	//	float avgDistortion1, avgDistortion2;
	//	for(iCluster = 0; iCluster < m_nClusters; ++iCluster)
	//	{
	//		IO::Assert(clusters[iCluster]->GetPointsNumber() == ptCnts[iCluster], "clusters[iCluster]->GetPointsNumber() = %d, ptCnts[iCluster] = %d\n", 
	//			clusters[iCluster]->GetPointsNumber(), ptCnts[iCluster]);
	//		if(ptCnts[iCluster] == 0)
	//			continue;
	//		clusters[iCluster]->ComputeDistortion();
	//		avgDistortion1 = clusters[iCluster]->GetDistortion() / clusters[iCluster]->GetPointsNumber();
	//		avgDistortion2 = distSqSum[iCluster] / ptCnts[iCluster];
	//		IO::Assert(fabs(avgDistortion1 - avgDistortion2) < 0.00001f, "%f - %f = %f\n", avgDistortion1, avgDistortion2, avgDistortion1 - avgDistortion2);
	//	}
	//}

private:

	KCTree<Point, Dimension, ElementType, ClusterIndex> m_KCTree;
	std::vector<KCTreeCluster<Point> *> m_buffer;
	std::vector<uint> m_mapCenterToPt;
	std::vector<ClusterIndex> m_mapPtToCluster;

};

#endif