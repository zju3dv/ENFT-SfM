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

#ifndef _VIEWER_SEQUENCE_DEPTH_H_
#define _VIEWER_SEQUENCE_DEPTH_H_

#include "ViewerSequence.h"
#include "Sequence/SequenceDepth.h"
#include "ProgramGL/ProgramGLUtility.h"
#include "ProgramGL/ProgramGLDepth.h"

#define KEY_DEPTH_VIEW              'i'
#define KEY_DEPTH_SCALE_INCREASE    '+'
#define KEY_DEPTH_SCALE_DECREASE    '-'
#define KEY_UPDATE_MODEL            'm'
#define KEY_UPDATE_MODEL_KEY_FRAME  'n'

#define DEPTH_SCALE_INITIAL         0.3f

class ViewerSequenceDepth : public ViewerSequence {

  protected:

    virtual void Initialize(const Sequence &seq, const FrameIndex iFrmActive = 0);
    virtual bool PrepareImageView(const FrameIndex iFrmImg);
    virtual bool PrepareSceneView(const FrameIndex iFrmScn);
    virtual void LoadDepthTexture(const FrameIndex iFrmDep);
    virtual void ScaleDepthTexture();
    virtual void ComputeLocalCoordinate(const FrameIndex iFrmLocalCoord);
    virtual void DrawFrameImageFileName();
    virtual void DrawSceneView();
    virtual void DrawActiveScenePoints();
    virtual bool IsMeasurementOutlier(const MeasurementIndex &iMea);
    virtual bool ShouldMeasurementBeHidden(const MeasurementIndex &iMea);
    virtual bool OnKeyDown(const int key);

  protected:

    bool m_depView;
    FrameIndex m_iFrmDep, m_iFrmDepScaled, m_iFrmLocalCoord;
    TextureGL1 m_texDep, m_texDepScaled;
    TextureGL3 m_texLocalCoord;
    CVD::Image<float> m_imgDep;
    CVD::Image<LA::Vector3f> m_imgLocalCoord;
    float m_depScale;
    ProgramGLScale m_programScale;
    ProgramGLComputeLocalCoordinate m_programComputeLocalCoordinate;

    class Model {
      public:
        Model() {}
        Model(const FrameIndex iFrm, const Camera &C,
              const CVD::Image<CVD::Rgba<ubyte> > &img,
              const CVD::Image<LA::Vector3f> &imgLocalCoord) {
            Set(iFrm, C, img, imgLocalCoord);
        }
        inline void Initialize() {
            m_iFrm = INVALID_FRAME_INDEX;
        }
        inline const FrameIndex &GetFrameIndex() const {
            return m_iFrm;
        }
        inline void Validate() {
            m_valid = true;
        }
        inline void Invalidate() {
            m_valid = false;
        }
        inline const bool &IsValid() const {
            return m_valid;
        }
        template<typename TYPE>
        inline void CopyImage(const CVD::Image<TYPE> &imgSrc,
                              CVD::Image<TYPE> &imgDst) {
            if(imgSrc.size().x==0||imgSrc.size().y == 0)
                return;
            imgDst.resize(imgSrc.size());
            memcpy(imgDst.data(), imgSrc.data(), sizeof(TYPE) * imgSrc.totalsize());
        }
        inline void Set(const FrameIndex iFrm, const Camera &C,
                        const CVD::Image<CVD::Rgba<ubyte> > &img,
                        const CVD::Image<LA::Vector3f> &imgLocalCoord) {
            m_iFrm = iFrm;
            m_valid = true;
            m_R.Set(C);
            C.GetTranslation(m_t);
            CopyImage(img, m_img);
            CopyImage(imgLocalCoord, m_imgLocalCoord);
        }
        inline void Draw() const {
            glPushMatrix();
            glMultMatrixf(m_R);
            glTranslatef(-m_t.v0(), -m_t.v1(), -m_t.v2());

            int x, y;
            glBegin(GL_POINTS);
            //glEnable(GL_TEXTURE_RECTANGLE_ARB);
            const int width = m_imgLocalCoord.size().x, height = m_imgLocalCoord.size().y;
            for(y = 0; y < height; ++y)
                for(x = 0; x < width; ++x) {
                    const LA::Vector3f &v = m_imgLocalCoord[y][x];
                    if(v.v2() == 0.0f)
                        continue;
                    glTexCoord2i(x, y);
                    //const CVD::Rgb<ubyte> &clr = m_img[y][x];
                    const CVD::Rgba<ubyte> &clr = m_img[y][x];
                    glColor3ub(clr.red, clr.green, clr.blue);
                    glVertex3fv(v);
                }
            //glDisable(GL_TEXTURE_RECTANGLE_ARB);
            glEnd();
            glPopMatrix();
        }
      protected:
        FrameIndex m_iFrm;
        bool m_valid;
        CVD::Image<CVD::Rgba<ubyte> > m_img;
        CVD::Image<LA::Vector3f> m_imgLocalCoord;
        LA::Matrix4f m_R;
        LA::Vector3f m_t;
    };
    Model m_modelActive;
    std::vector<Model> m_models;

};

#endif