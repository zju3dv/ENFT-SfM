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

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include "ProgramGL/ProgramGL.h"
#include "Viewing/EventHandler.h"
#include "SfM/Point.h"
#include <cvd/glwindow.h>
#include <cvd/image_io.h>
#include <cvd/vision.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <stdarg.h>

#define STRING_FONT_LARGE GLUT_BITMAP_HELVETICA_18
#define STRING_SIZE_LARGE 18
#define STRING_FONT_SMALL GLUT_BITMAP_HELVETICA_12
#define STRING_SIZE_SMALL 12
#define STRING_BORDER_X 15
#define STRING_BORDER_Y 15
#define STRING_GAP_Y 10
#define STRING_COLOR_R 0
#define STRING_COLOR_G 255
#define STRING_COLOR_B 0

class Viewer {

  public:

    Viewer() : m_pWnd(NULL) {}

    inline void Run() {
        ProgramGL::UnbindFrameBuffer();
        Initialize();
        Resize(m_pWnd->size());
        m_handler.Quit() = false;
        while(!m_handler.Quit()) {
            Draw();
            DrawString();
            m_pWnd->swap_buffers();
            m_pWnd->handle_events(m_handler);
        }
        ProgramGL::BindFrameBuffer();
    }

  protected:

    inline void DrawStringTopLeftLarge(char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        glRasterPos2i(STRING_BORDER_X, STRING_BORDER_Y + STRING_SIZE_LARGE);
        for(char *s = buf; *s; s++)
            glutBitmapCharacter(STRING_FONT_LARGE, *s);
    }
    inline void DrawStringTopLeftLarge(const ushort &row, char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        glRasterPos2i(STRING_BORDER_X, STRING_BORDER_Y + STRING_SIZE_LARGE + STRING_SIZE_LARGE * row);
        for(char *s = buf; *s; s++)
            glutBitmapCharacter(STRING_FONT_LARGE, *s);
    }
    inline void DrawStringBottomLeftLarge(char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        glRasterPos2i(STRING_BORDER_X, m_pWnd->size().y - STRING_BORDER_Y);
        for(char *s = buf; *s; s++)
            glutBitmapCharacter(STRING_FONT_LARGE, *s);
    }
    inline void DrawStringBottomLeftLarge(const ushort &row, char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        glRasterPos2i(STRING_BORDER_X, m_pWnd->size().y - STRING_BORDER_Y - STRING_SIZE_LARGE * row);
        for(char *s = buf; *s; s++)
            glutBitmapCharacter(STRING_FONT_LARGE, *s);
    }
    static inline void DrawStringCurrentLarge(char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        for(char *s = buf; *s; s++)
            glutBitmapCharacter(STRING_FONT_LARGE, *s);
    }
    static inline void DrawString(const int &x, const int &y, void *const &font, char *format, ...) {
        va_list args;
        char buf[MAX_LINE_LENGTH];

        va_start(args, format);
        vsprintf(buf, format, args);
        va_end(args);

        glRasterPos2i(x, y);
        for(char *s = buf; *s; s++)
            glutBitmapCharacter(font, *s);
    }
    static inline void DrawBox(const float &x, const float &y, const float &boxSize) {
        glVertex2f(x - boxSize, y - boxSize);
        glVertex2f(x - boxSize, y + boxSize);
        glVertex2f(x - boxSize, y + boxSize);
        glVertex2f(x + boxSize, y + boxSize);
        glVertex2f(x + boxSize, y + boxSize);
        glVertex2f(x + boxSize, y - boxSize);
        glVertex2f(x + boxSize, y - boxSize);
        glVertex2f(x - boxSize, y - boxSize);
    }
    static inline void DrawBox(const float &x, const float &y, const Point2D &boxSize) {
        glVertex2f(x - boxSize.x(), y - boxSize.y());
        glVertex2f(x - boxSize.x(), y + boxSize.y());
        glVertex2f(x - boxSize.x(), y + boxSize.y());
        glVertex2f(x + boxSize.x(), y + boxSize.y());
        glVertex2f(x + boxSize.x(), y + boxSize.y());
        glVertex2f(x + boxSize.x(), y - boxSize.y());
        glVertex2f(x + boxSize.x(), y - boxSize.y());
        glVertex2f(x - boxSize.x(), y - boxSize.y());
    }
    static inline void DrawBox(const Point2D &start, const Point2D &end) {
        glVertex2f(start.x(), start.y());
        glVertex2f(start.x(), end.y());
        glVertex2f(start.x(), end.y());
        glVertex2f(end.x(), end.y());
        glVertex2f(end.x(), end.y());
        glVertex2f(end.x(), start.y());
        glVertex2f(end.x(), start.y());
        glVertex2f(start.x(), start.y());
    }
    static inline void DrawQuad(const float &x, const float &y, const Point2D &quadSize) {
        glVertex2f(x - quadSize.x(), y - quadSize.y());
        glVertex2f(x - quadSize.x(), y + quadSize.y());
        glVertex2f(x + quadSize.x(), y + quadSize.y());
        glVertex2f(x + quadSize.x(), y - quadSize.y());
    }
    static inline void DrawQuad(const Point2D x1, const Point2D &x2) {
        glVertex2f(x1.x(), x1.y());
        glVertex2f(x1.x(), x2.y());
        glVertex2f(x2.x(), x2.y());
        glVertex2f(x2.x(), x1.y());
    }
    static inline void DrawCross(const float &x, const float &y, const Point2D &crossSize) {
        glVertex2f(x - crossSize.x(), y);
        glVertex2f(x + crossSize.x(), y);
        glVertex2f(x, y - crossSize.y());
        glVertex2f(x, y + crossSize.y());
    }
    inline void DrawTexture(const ushort &width, const ushort &height) const {
        glEnable(GL_TEXTURE_RECTANGLE_ARB);
        glBegin(GL_QUADS);
        //glTexCoord2s(0, height);      glVertex2i(0, 0);
        //glTexCoord2s(0, 0);           glVertex2i(0, m_pWnd->size().y);
        //glTexCoord2s(width, 0);       glVertex2i(m_pWnd->size().x, m_pWnd->size().y);
        //glTexCoord2s(width, height);  glVertex2i(m_pWnd->size().x, 0);
        glTexCoord2i(0, 0);
        glVertex2i(0, 0);
        glTexCoord2i(0, height);
        glVertex2i(0, m_pWnd->size().y);
        glTexCoord2i(width, height);
        glVertex2i(m_pWnd->size().x, m_pWnd->size().y);
        glTexCoord2i(width, 0);
        glVertex2i(m_pWnd->size().x, 0);
        glEnd();
        glDisable(GL_TEXTURE_RECTANGLE_ARB);
        glFlush();
    }
    inline void SaveView(const char *fileName) {
        Draw();
        CVD::Image<CVD::Rgba<ubyte> > img(m_pWnd->size());
        glReadPixels(0, 0, m_pWnd->size().x, m_pWnd->size().y, GL_RGBA, GL_UNSIGNED_BYTE, img.data());
        CVD::flipVertical(img);
        CVD::img_save(img, fileName);
        printf("Saved \'%s\'\n", fileName);
    }

    static inline void ConvertMeanCovarianceToEllipse(const Point2D &mean, const LA::Vector3f &cov, std::vector<Point2D> &ellipse) {
        const float det = cov.v0() * cov.v2() - cov.v1() * cov.v1();
        const float one_over_det = 1 / det;
        const float c0 = one_over_det * cov.v2();
        const float c1 = -one_over_det * cov.v1();
        const float c2 = one_over_det * cov.v0();

        // c0*x^2 + 2*c1*xy + c2*y^2 = 1
        // x = u * cos(theta) - v * sin(theta)
        // y = u * sin(theta) + v * cos(theta)
        // u = x * cos(theta) + y * sin(theta)
        // v = x * sin(theta) - y * cos(theta)
        // A * u^2 + B * v^2 = 1
        const float c3 = c0 + c2;                   // A + B
        const float c4 = c0 - c2;                   // (A - B) * cos(2*theta)
        const float c5 = 4 * c1 * c1 + c4 * c4;     // (A - B)^2
        const float AmB = c1 > 0 ? sqrt(c5) : -sqrt(c5);

        const float theta = asin(2 * c1 / AmB) * 0.5f;
        const float A = (AmB + c3) * 0.5f;
        const float B = c3 - A;

        const float cosTheta = cos(theta);
        const float sinTheta = sin(theta);
        const float a = 1 / sqrt(A);
        const float b = 1 / sqrt(B);

        float u, v, x, y;
        ellipse.resize(0);
        for(float t = 0; t < PIx2; t += FACTOR_DEG_TO_RAD) {
            u = a * cos(t);
            v = b * sin(t);
            x = u * cosTheta - v * sinTheta + mean.x();
            y = u * sinTheta + v * cosTheta + mean.y();
            ellipse.push_back(Point2D(x, y));
        }
    }

    static inline void DrawPoints(const std::vector<Point2D> &xs) {
        const ushort nPts = ushort(xs.size());
        for(ushort i = 0; i < nPts; ++i)
            glVertex2fv(xs[i]);
    }

    inline void Initialize() {
        m_pWnd = ProgramGL::GetGLWindow();
        m_handler.Initialize(this, &Viewer::KeyDown, &Viewer::KeyUp, &Viewer::MouseMove, &Viewer::MouseDown, &Viewer::MouseUp, &Viewer::MouseDraging,
                             &Viewer::MouseDoubleClicked, &Viewer::Resize);
        OnInitialize();
    }
    inline void Draw()                                                                                      {
        OnDraw();
    }
    inline void DrawString()                                                                                {
        OnDrawString();
    }
    inline bool KeyDown             (const int key)                                                         {
        return OnKeyDown(key);
    }
    inline bool KeyUp               (const int key)                                                         {
        return OnKeyUp(key);
    }
    inline void MouseMove           (const CVD::ImageRef &where)                                            {
        OnMouseMove(where);
    }
    inline void MouseDown           (const CVD::ImageRef &where, const int button)                          {
        OnMouseDown(where, button);
    }
    inline void MouseUp             (const CVD::ImageRef &where, const int button)                          {
        OnMouseUp(where, button);
    }
    inline bool MouseDraging        (const CVD::ImageRef &from, const CVD::ImageRef &to, const int button)  {
        return OnMouseDraging(from, to, button);
    }
    inline bool MouseDoubleClicked  (const CVD::ImageRef &where, const int button)                          {
        return OnMouseDoubleClicked(where, button);
    }
    inline void Resize              (const CVD::ImageRef &size)                                             {
        ProgramGL::FitViewportWindows(size.x, size.y);
        OnResize(size);
    }

  protected:

    virtual void OnInitialize ()                                                                                {}
    virtual void OnDraw       ()                                                                                {}
    virtual void OnDrawString ()                                                                                {}
    virtual bool OnKeyDown              (const int key)                                                         {
        return false;
    }
    virtual bool OnKeyUp                (const int key)                                                         {
        return false;
    }
    virtual void OnMouseMove            (const CVD::ImageRef &where)                                            {}
    virtual void OnMouseDown            (const CVD::ImageRef &where, const int button)                          {}
    virtual void OnMouseUp              (const CVD::ImageRef &where, const int button)                          {}
    virtual bool OnMouseDraging         (const CVD::ImageRef &from, const CVD::ImageRef &to, const int button)  {
        return false;
    }
    virtual bool OnMouseDoubleClicked   (const CVD::ImageRef &where, const int button)                          {
        return false;
    }
    virtual void OnResize(const CVD::ImageRef &size)                                                            {}

  protected:

    EventHandler<Viewer> m_handler;
    CVD::GLWindow *m_pWnd;
};

#endif