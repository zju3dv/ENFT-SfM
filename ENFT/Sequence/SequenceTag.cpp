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

#include "stdafx.h"
#include "Sequence/SequenceTag.h"
#include "Utility/Utility.h"
#include <cvd/image_io.h>
#include <cvd/draw.h>

void SequenceTag::GenerateImageFileNames()
{
	if(m_iStart == -1 && m_iStep == -1 && m_iEnd == -1)
		return;
	if(m_seqName == "")
	{
		const int nFrms = (m_iEnd - m_iStart + m_iStep) / m_iStep;
		if(m_iStart >= 0 && m_iEnd >= 0)
			m_imgFileNames.assign(nFrms, "");
		else
			m_imgFileNames.resize(0);
	}
	else
	{
		m_imgFileNames.resize(0);
		for(int incr = m_iStart; incr <= m_iEnd; incr += m_iStep)
			m_imgFileNames.push_back(m_seqDir + IO::IncreaseFileNumber(m_seqName, incr));
	}
}

template<typename TYPE>
static inline void LoadImage(CVD::Image<TYPE> &img1, CVD::Image<TYPE> &img2, const std::string &fileName)
{
	CVD::img_load(img1, fileName);
	if(img1.size().x % 4 == 0)
		img2 = img1;
	else
	{
		const int width = (img1.size().x + 3) & (~3), height = img1.size().y;
		img2.resize(CVD::ImageRef(width, height));
		img2.zero();
		for(int y = 0; y < height; ++y)
			memcpy(img2[y], img1[y], sizeof(TYPE) * img1.row_stride());
	}
}

void SequenceTag::LoadImageSize()
{
	if(m_imgFileNames.empty() || m_imgFileNames[0] == "")
		return;
		//m_width = m_height = 0;
	CVD::Image<CVD::Rgb<CVD::byte> > img, imgTmp;
	//CVD::img_load(img, std::string(m_seqDir + m_seqName));
	//CVD::img_load(img, m_imgFileNames[0]);
	::LoadImage(imgTmp, img, m_imgFileNames[0]);
	const CVD::ImageRef size = img.size();
	m_width  = ushort(size.x);
	m_height = ushort(size.y);
}

void SequenceTag::SaveB(FILE *fp) const
{
	char buf[MAX_LINE_LENGTH];
	IO::StringSaveB(m_seqDir, buf, fp);
	IO::StringSaveB(m_seqName, buf, fp);

	fwrite(&m_iStart, sizeof(int), 1, fp);
	fwrite(&m_iStep, sizeof(int), 1, fp);
	fwrite(&m_iEnd, sizeof(int), 1, fp);
	fwrite(&m_width, sizeof(ushort), 1, fp);
	fwrite(&m_height, sizeof(ushort), 1, fp);
	const ushort nFrms = ushort(m_imgFileNames.size());
	fwrite(&nFrms, sizeof(ushort), 1, fp);
	for(ushort iFrm = 0; iFrm < nFrms; ++iFrm)
		IO::StringSaveB(m_imgFileNames[iFrm], buf, fp);
}

void SequenceTag::LoadB(FILE *fp)
{
	char buf[MAX_LINE_LENGTH];
	const std::string seqDirBkp = m_seqDir;
	IO::StringLoadB(m_seqDir, buf, fp);
	IO::StringLoadB(m_seqName, buf, fp);

	fread(&m_iStart, sizeof(int), 1, fp);
	fread(&m_iStep, sizeof(int), 1, fp);
	fread(&m_iEnd, sizeof(int), 1, fp);
	fread(&m_width, sizeof(ushort), 1, fp);
	fread(&m_height, sizeof(ushort), 1, fp);
	ushort nFrms;
	fread(&nFrms, sizeof(ushort), 1, fp);
	m_imgFileNames.resize(nFrms);
	for(ushort iFrm = 0; iFrm < nFrms; ++iFrm)
		IO::StringLoadB(m_imgFileNames[iFrm], buf, fp);

	if(m_seqDir == seqDirBkp || seqDirBkp.empty())
		return;
	for(ushort iFrm = 0; iFrm < nFrms; ++iFrm)
		m_imgFileNames[iFrm] = IO::ReplaceFileDirectory(m_imgFileNames[iFrm], m_seqDir, seqDirBkp);
	m_seqDir = seqDirBkp;
}

void SequenceTag::SaveActb(FILE *fp) const
{
	char buf[MAX_LINE_LENGTH];
	sprintf(buf, ".\\%s\n", m_seqName.c_str());
	fwrite(buf, 1, strlen(buf), fp);
	fwrite(&m_iStart, sizeof(int), 1, fp);
	fwrite(&m_iStep, sizeof(int), 1, fp);
	fwrite(&m_iEnd, sizeof(int), 1, fp);
}

void SequenceTag::LoadActb(FILE *fp)
{
	char buf[MAX_LINE_LENGTH];
	IO::StringLoadB(m_seqName, buf, fp);
	m_seqName = IO::RemovePrefix(m_seqName, GetDirectory());
	fread(&m_iStart, sizeof(int), 1, fp);
	fread(&m_iStep, sizeof(int), 1, fp);
	fread(&m_iEnd, sizeof(int), 1, fp);
	GenerateImageFileNames();
	LoadImageSize();
}

void SequenceTag::SaveAct(FILE *fp) const
{
	fprintf(fp, "Sequence:.\\%s\n", m_seqName.c_str());
	fprintf(fp, "start:%d\n", m_iStart);
	fprintf(fp, "step:%d\n", m_iStep);
	fprintf(fp, "end:%d\n", m_iEnd);
}

void SequenceTag::LoadAct(FILE *fp)
{
	char buf[MAX_LINE_LENGTH];
	int int3[3];
	fscanf(fp, "Sequence:%s\n", buf);		m_seqName = IO::RemovePrefix(buf, GetDirectory());
	fscanf(fp, "start:%d\n", &int3[0]);		m_iStart = int(int3[0]);
	fscanf(fp, "step:%d\n", &int3[1]);		m_iStep = int(int3[1]);
	fscanf(fp, "end:%d\n", &int3[2]);		m_iEnd = int(int3[2]);
	GenerateImageFileNames();
	LoadImageSize();
}