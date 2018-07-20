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

#ifndef _VIEWER_SEQUENCE_H_
#define _VIEWER_SEQUENCE_H_

#include "Viewer.h"
#include "Sequence/Sequence.h"
#include "CameraVolume.h"
#include "Arcball.h"
#include <cvd/image.h>

#define KEY_SWITCH_VIEW					' '
#define KEY_STEP_ACTIVE_FRAME_FORWARD	'f'
#define KEY_STEP_ACTIVE_FRAME_BACKWARD	'd'
#define KEY_STEP_SELECTED_TRACK_FRAME	9	// Tab
#define KEY_SAVE_VIEW					's'
#define KEY_INPUT_FRAME					6		// Ctrl + f
#define KEY_INPUT_TRACK					20		// Ctrl + t
#define KEY_INPUT_MEASUREMENT			13		// Ctrl + m
#define KEY_INPUT_FEATURE				5		// Ctrl + e
#define KEY_INPUT_DESCRIPTOR			4		// Ctrl + d
#define KEY_INPUT_MIN_TRACK_LENGTH		12		// Ctrl + l
#define KEY_PRINT_STATE					16		// Ctrl + p

#define KEY_STEP_PROJECTION_TYPE		'j'
#define KEY_DRAW_ERRORS					'e'
#define KEY_DRAW_DISTRIBUTION			'l'
#define KEY_STEP_DRAW_TRACK_TYPE		'x'

#define KEY_TRANSLATE_XY				'x'
#define KEY_TRANSLATE_Z					'z'
#define KEY_ROTATION					'r'
#define KEY_DRAW_ARCBALL				'b'
#define KEY_STEP_DRAW_CAMERA_TYPE		'c'
#define KEY_STEP_DRAW_POINT_TYPE		'p'
#define KEY_DRAW_ACTIVE_POINTS			'a'
#define KEY_RESET_TRANSFORMATION		't'
#define KEY_SWITCH_BACKGROUND_COLOR		'k'

#define KEY_CAMERA_VOLUME_SCALE_INCREASE	38	// Up
#define KEY_CAMERA_VOLUME_SCALE_DECREASE	40	// Down
#define KEY_CAMERA_PATH_WIDTH_INCREASE		39	// Right
#define KEY_CAMERA_PATH_WIDTH_DECREASE		37	// Left

#define WINDOW_ZPLANE							0.00001f
#define CAMERA_VOLUME_ZPLANE_DEPTH_RANGE_RATIO	0.5f
#define ARCBALL_RADIUS_WINDOWN_RATIO			0.4f
#define ARCBALL_SLICES							20
#define ARCBALL_STACKS							20
#define TRANSLATE_Z_FACTOR						2.0f

#define FRAME_BAR_COLOR_R					0
#define FRAME_BAR_COLOR_G					255
#define FRAME_BAR_COLOR_B					0
#define FRAME_BAR_POINTER_WIDTH				20
#define FRAME_BAR_POINTER_HEIGHT			20

#define TRACK_BAR_COLOR_R					0
#define TRACK_BAR_COLOR_G					0
#define TRACK_BAR_COLOR_B					255
#define TRACK_BAR_POINTER_HEIGHT			10

#define FEATURE_CROSS_SIZE					5
#define FEATURE_SELECTION_BOX_SIZE			5

#define COLOR_FEATURE_SINGLE_R			127
#define COLOR_FEATURE_SINGLE_G			127
#define COLOR_FEATURE_SINGLE_B			127
#define COLOR_FEATURE_OUTLIER_R			255
#define COLOR_FEATURE_OUTLIER_G			0
#define COLOR_FEATURE_OUTLIER_B			0
#define COLOR_FEATURE_COMMON_TRACK_R	255
#define COLOR_FEATURE_COMMON_TRACK_G	0
#define COLOR_FEATURE_COMMON_TRACK_B	255
#define COLOR_FEATURE_MERGED_TRACK_R	255
#define COLOR_FEATURE_MERGED_TRACK_G	0
#define COLOR_FEATURE_MERGED_TRACK_B	255
#define COLOR_FEATURE_SIFT_R			0
#define COLOR_FEATURE_SIFT_G			255
#define COLOR_FEATURE_SIFT_B			0
#define COLOR_FEATURE_ENFT_R			0
#define COLOR_FEATURE_ENFT_G			255
#define COLOR_FEATURE_ENFT_B			255
#define COLOR_FEATURE_SELECTION_BOX_R	0
#define COLOR_FEATURE_SELECTION_BOX_G	0
#define COLOR_FEATURE_SELECTION_BOX_B	255
#define COLOR_FEATURE_ERROR_LINE_R		0
#define COLOR_FEATURE_ERROR_LINE_G		0
#define COLOR_FEATURE_ERROR_LINE_B		255
#define COLOR_FEATURE_ELLIPSE_TRACKED_R	255
#define COLOR_FEATURE_ELLIPSE_TRACKED_G	255
#define COLOR_FEATURE_ELLIPSE_TRACKED_B	0
#define COLOR_FEATURE_ELLIPSE_INLIER_R	0
#define COLOR_FEATURE_ELLIPSE_INLIER_G	255
#define COLOR_FEATURE_ELLIPSE_INLIER_B	0
#define COLOR_FEATURE_ELLIPSE_OUTLIER_R	255
#define COLOR_FEATURE_ELLIPSE_OUTLIER_G	0
#define COLOR_FEATURE_ELLIPSE_OUTLIER_B	0
#define COLOR_TRACK_TEMPORAL_R			255
#define COLOR_TRACK_TEMPORAL_G			255
#define COLOR_TRACK_TEMPORAL_B			255
#define COLOR_PROJECTION_ACTIVE_R		255
#define COLOR_PROJECTION_ACTIVE_G		255
#define COLOR_PROJECTION_ACTIVE_B		0
#define COLOR_PROJECTION_FRONT_R		0
#define COLOR_PROJECTION_FRONT_G		0
#define COLOR_PROJECTION_FRONT_B		255
#define COLOR_PROJECTION_BACK_R			255
#define COLOR_PROJECTION_BACK_G			0
#define COLOR_PROJECTION_BACK_B			0

#define COLOR_CAMERA_PATH_R				204
#define COLOR_CAMERA_PATH_G				102
#define COLOR_CAMERA_PATH_B				0
#define COLOR_CAMERA_VOLUME_R			135
#define COLOR_CAMERA_VOLUME_G			204
#define COLOR_CAMERA_VOLUME_B			234
#define COLOR_KEY_FRAME_R				0
#define COLOR_KEY_FRAME_G				0
#define COLOR_KEY_FRAME_B				255
#define COLOR_ACTIVE_POINT_INLIER_R		255
#define COLOR_ACTIVE_POINT_INLIER_G		255
#define COLOR_ACTIVE_POINT_INLIER_B		0
#define COLOR_ACTIVE_POINT_OUTLIER_R	255
#define COLOR_ACTIVE_POINT_OUTLIER_G	0
#define COLOR_ACTIVE_POINT_OUTLIER_B	0
#define COLOR_ARCBALL_R					255
#define COLOR_ARCBALL_G					255
#define COLOR_ARCBALL_B					255
#define COLOR_TRACK_DEFAULT_R			127
#define COLOR_TRACK_DEFAULT_G			127
#define COLOR_TRACK_DEFAULT_B			127

class ViewerSequence : public Viewer
{

public:

	ViewerSequence();
	~ViewerSequence();

	virtual void Run(const Sequence &seq, const FrameIndex iFrmActive = 0);

protected:

	virtual void Initialize(const Sequence &seq, const FrameIndex iFrmActive = 0);

	virtual bool PrepareView(const bool imgView);
	virtual bool PrepareActiveFrame(const FrameIndex iFrmActive);
	virtual bool PrepareImageView(const FrameIndex iFrmImg);
	virtual bool PrepareSceneView(const FrameIndex iFrmScn);
	virtual bool PrepareSelectedTrack(const TrackIndex iTrkSelected);
	virtual bool PrepareSelectedFeature(const FeatureIndex iFtrSelected);
	virtual void LoadImageTexture(const FrameIndex iFrmImg);

	virtual void DrawImageView();
	virtual void DrawFeaturePoints();
	virtual void DrawFeatureTracks();
	virtual void DrawFeatureDistribution();
	virtual void DrawSelectedFeature();
	virtual void DrawActiveProjections();
	virtual void DrawAllProjections();
	virtual void DrawSceneView();
	virtual void DrawSceneCameras();
	virtual void DrawScenePoints();
	virtual void DrawSelectedScenePoint();
	virtual void DrawActiveScenePoints();
	virtual void DrawFrameBar();
	virtual void DrawFrameState();
	virtual void DrawFrameImageFileName();
	virtual bool DragFrameBar(const CVD::ImageRef &from, const CVD::ImageRef &to);
	virtual bool IsFrameSolved(const FrameIndex &iFrm);
	virtual bool ShouldTrackBeHidden(const TrackIndex &iTrk);
	virtual bool ShouldMeasurementBeHidden(const MeasurementIndex &iMea);
	virtual bool IsTrackOutlier(const TrackIndex &iTrk);
	virtual bool IsMeasurementOutlier(const MeasurementIndex &iMea);
	void ConvertWindowCoordinate2DTo3D(const CVD::ImageRef &where2D, Point3D &where3D);
	void ConvertWindowCoordinate2DTo3D(const CVD::ImageRef &where2D, Point3D &where3D, const float &zPlane);
	virtual void ApplyImageViewTransformation();
	virtual void ApplySceneViewTransformation();
	virtual void ResetSceneViewProjectionMatrix();
	virtual void ResetSceneViewModelViewMatrix();
	virtual void ResetSceneViewTransformation();
	virtual void ResetImageWindowSizeRatio();
	void UpdateSceneViewTransformation();
	virtual void ComputeDepths(std::vector<float> &depths);
	float ComputeDepthMean();
	void ComputeDepthMeanAndRange(float &depthMean, float &depthRange);
	virtual float ComputeArcballRadius(const float &centerZ);

	virtual void OnDraw();
	virtual void OnDrawString();
	virtual bool OnKeyDown(const int key);
	virtual void OnMouseDown(const CVD::ImageRef &where, const int button);
	virtual void OnMouseUp(const CVD::ImageRef &where, const int button);
	virtual bool OnMouseDraging(const CVD::ImageRef &from, const CVD::ImageRef &to, const int button);
	virtual bool OnMouseDoubleClicked(const CVD::ImageRef &where, const int button);
	virtual void OnResize(const CVD::ImageRef &size);

protected:

	const Sequence *m_pSeq;
	enum DrawProjectionType { DRAW_NO_PROJECTION, DRAW_ALL_PROJECTIONS, DRAW_ACTIVE_PROJECTIONS } m_drawProjType;
	bool m_imgView;
	FrameIndex m_minTrkLen;
	FrameIndex m_iFrmActive, m_iFrmImg, m_iFrmScn;
	TrackIndex m_iTrkSelected;
	FeatureIndex m_iFtrSelected;
	Point2D	m_factorImgToWin, m_factorWinToImg/*, m_factorWinToNorm*/;
	FeatureIndex m_nInliers;
	float m_inlierRatio, m_MSE, m_inlierRatioArea;
	FeatureIndexList m_iFtrs;
	Point2D m_meanFtrsTrked, m_meanFtrsInlier, m_meanFtrsOutlier;
	LA::Vector3f m_covFtrsTrked, m_covFtrsInlier, m_covFtrsOutlier;
	std::vector<Point2D> m_ellipse;
	
	//////////////////////////////////////////////////////////////////////////
	// Image view
	//////////////////////////////////////////////////////////////////////////
	//TextureGL3 m_texImg;
	//CVD::Image<CVD::Rgb<ubyte> > m_img;
	TextureGL4 m_texImg;
	CVD::Image<CVD::Rgba<ubyte> > m_img, m_imgTmp;
	bool m_drawErrs, m_drawDistribution;
	enum DrawPointType { DRAW_NO_POINT, DRAW_ALL_POINTS, DRAW_INLIER_POINTS, DRAW_OUTLIER_POINTS } m_drawPtTypes[2];
	enum DrawTrackType { DRAW_NO_TRACK/*, DRAW_TRACK_ALL_FRAMES*/, DRAW_TRACK_FROM_LAST_FRAME, DRAW_TRACK_FROM_LAST_KEY_FRAME, DRAW_TRACK_TO_NEXT_KEY_FRAME } m_drawTrkType;

	//////////////////////////////////////////////////////////////////////////
	// Scene view
	//////////////////////////////////////////////////////////////////////////
	enum ControlType { CONTROL_TRANSLATE_XY, CONTROL_TRANSLATE_Z, CONTROL_ROTATION } m_controlType;
	CameraVolume &m_camVolume;
	float m_camVolumeScale, m_camPathWidth;
	bool m_drawArcball, m_drawActivePts, m_backgroundDark;
	enum DrawCameraType { DRAW_NO_CAMERA, DRAW_PATH, DRAW_PATH_AND_ACTIVE_FRAME, DRAW_PATH_AND_ALL_KEY_FRAMES } m_drawCamType;
	//const LA::Matrix4d m_I;
	double m_projMatrix[16], m_modelMatrix[16];
	GLint m_viewport[4];
	Camera &m_Cg;
	Point3D &m_translationStart, &m_translation, &m_whereMouseDown3DStart;
	Arcball &m_arcball;
	float m_dZuint;
	std::vector<float> m_depths;
	AlignedVector<Point3D> m_rayDirs;

};

#endif