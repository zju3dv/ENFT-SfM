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

#ifndef _SEQUENCE_TAG_H_
#define _SEQUENCE_TAG_H_

class SequenceTag
{

public:

	SequenceTag() : m_iStart(-1), m_iStep(-1), m_iEnd(-1), m_width(0), m_height(0) {}
	SequenceTag(const std::string &seqDir) : m_seqDir(seqDir), m_iStart(-1), m_iStep(-1), m_iEnd(-1), m_width(0), m_height(0) {}
	inline void Set(const std::string &seqDir, const std::string &seqName, const int &iStart, const int &iStep, const int &iEnd)
	{
		m_seqDir = seqDir;
		m_seqName = seqName;
		m_iStart = iStart;
		m_iStep  = iStep;
		m_iEnd   = iEnd;
		GenerateImageFileNames();
		LoadImageSize();
	}
	inline void Set(const std::string &seqDir, const std::string &seqName, const int &iStart, const int &iStep, const int &iEnd, 
					const ushort &width, const ushort &height)
	{
		m_seqDir = seqDir;
		m_seqName = seqName;
		m_iStart = iStart;
		m_iStep  = iStep;
		m_iEnd   = iEnd;
		m_width = width;
		m_height = height;
		GenerateImageFileNames();
	}
	inline void Set(const std::vector<std::string> &imgFileNames, const std::string seqName = "")
	{
		//m_seqName = "";
		//m_iStart = m_iStep = m_iEnd = -1;
		m_imgFileNames = imgFileNames;
		LoadImageSize();
		m_seqName = seqName;
	}
	inline void Swap(SequenceTag &tag)
	{
		m_seqDir.swap(tag.m_seqDir);
		m_seqName.swap(tag.m_seqName);
		int iTmp;
		SWAP(m_iStart, tag.m_iStart, iTmp);
		SWAP(m_iStep, tag.m_iStep, iTmp);
		SWAP(m_iEnd, tag.m_iEnd, iTmp);
		ushort usTmp;
		SWAP(m_width, tag.m_width, usTmp);
		SWAP(m_height, tag.m_height, usTmp);
		m_imgFileNames.swap(tag.m_imgFileNames);
	}
	inline void GetSubSequence(const std::vector<ushort> &iFrms, SequenceTag &tag) const
	{
		tag.m_seqDir = m_seqDir;
		tag.m_seqName = m_seqName;
		tag.m_iStart = tag.m_iStep = tag.m_iEnd = -1;
		tag.m_width = m_width;
		tag.m_height = m_height;
		const ushort nFrmsSub = ushort(iFrms.size());
		tag.m_imgFileNames.resize(nFrmsSub);
		for(ushort i = 0; i < nFrmsSub; ++i)
			tag.m_imgFileNames[i] = m_imgFileNames[iFrms[i]];
	}
	inline void GetSubSequence(const ushort &iFrm1, const ushort &iFrm2, SequenceTag &tag) const
	{
		tag.m_seqDir = m_seqDir;
		tag.m_seqName = m_seqName;
		//tag.m_iStart = tag.m_iStep = tag.m_iEnd = -1;
		tag.m_iStart = m_iStart + iFrm1 * m_iStep;
		tag.m_iStep = m_iStep;
		tag.m_iEnd = m_iStart + (iFrm2 - 1) * m_iStep;
		tag.m_width = m_width;
		tag.m_height = m_height;
		const ushort nFrms = GetFramesNumber(), nFrmsSub = std::min(iFrm2, nFrms) - iFrm1;
		tag.m_imgFileNames.resize(nFrmsSub);
		for(ushort i = 0, iFrm = iFrm1; i < nFrmsSub; ++i, ++iFrm)
			tag.m_imgFileNames[i] = m_imgFileNames[iFrm];
	}
	inline void Clear() { m_imgFileNames.clear(); }
	inline void SetDirectory(const std::string &seqDir) { m_seqDir = seqDir; GenerateImageFileNames(); }
	inline const std::string& GetDirectory() const { return m_seqDir; }
	inline void SetImageSize(const ushort &width, const ushort &height) { m_width = width; m_height = height; }
	inline const ushort&	  GetImageWidth () const { return m_width ; }
	inline const ushort&	  GetImageHeight() const { return m_height; }
	inline const std::string& GetImageFileName(const ushort &iFrm) const { return m_imgFileNames[iFrm] ; }
	inline const std::vector<std::string>& GetImageFileNames() const { return m_imgFileNames; }
	inline ushort GetFramesNumber() const { return ushort(m_imgFileNames.size()); }
	inline const std::string& GetSequenceName() const { return m_seqName; }
	inline const int& GetStartFrame() const { return m_iStart; }
	inline const int& GetStepFrame() const { return m_iStep; }
	inline const int& GetEndFrame() const { return m_iEnd; }
	inline void SetImageFileName(const ushort &iFrm, const std::string &imgFileName) { m_imgFileNames[iFrm] = imgFileName; }
	inline void PushBackImageFileName(const std::string &imgFileName) { m_imgFileNames.push_back(imgFileName); }
	void GenerateImageFileNames();
	void LoadImageSize();
	void SaveB(FILE *fp) const;
	void LoadB(FILE *fp);
	void SaveActb(FILE *fp) const;
	void LoadActb(FILE *fp);
	void SaveAct(FILE *fp) const;
	void LoadAct(FILE *fp);

private:

	std::string m_seqDir, m_seqName;
	int m_iStart, m_iStep, m_iEnd;
	ushort m_width, m_height;
	std::vector<std::string> m_imgFileNames;

};

#endif