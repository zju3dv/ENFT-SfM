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

#ifndef _EVENT_HANDLER_H_
#define _EVENT_HANDLER_H_

#include <cvd/glwindow.h>

#define KEY_QUIT					27

template<class HANDLER>
class EventHandler : public CVD::GLWindow::EventHandler
{

public:

	typedef bool (HANDLER::*FunctionPointerKeyDown)			(const int key);
	typedef bool (HANDLER::*FunctionPointerKeyUp)			(const int key);
	typedef void (HANDLER::*FunctionPointerMouseMove)		(const CVD::ImageRef &where);
	typedef void (HANDLER::*FunctionPointerMouseDown)		(const CVD::ImageRef &where, const int button);
	typedef void (HANDLER::*FunctionPointerMouseUp)			(const CVD::ImageRef &where, const int button);
	typedef bool (HANDLER::*FunctionPointerMouseDraging)	(const CVD::ImageRef &from, const CVD::ImageRef &to, const int button);
	typedef bool (HANDLER::*FunctionPointerMouseDoubleClick)(const CVD::ImageRef &where, const int button);
	typedef void (HANDLER::*FunctionPointerResize)			(const CVD::ImageRef &size);

public:

	EventHandler() : m_quit(false), m_keyDown(false), m_mouseDown(false), m_pHandler(NULL), m_pFuncKeyDown(NULL), m_pFuncKeyUp(NULL), 
		m_pFuncMouseMove(NULL), m_pFuncMouseDown(NULL), m_pFuncMouseUp(NULL), m_pFuncMouseDraging(NULL), m_pFuncMouseDoubleClick(NULL), m_pFuncResize(NULL) {}

	/// Called for key press events
	virtual void on_key_down(CVD::GLWindow &win, int key)
	{
		m_keyDown = true;
		m_key = key;
		m_quit = (key == KEY_QUIT);
		if(m_pHandler && m_pFuncKeyDown)
			(m_pHandler->*m_pFuncKeyDown)(key);
	}
	/// Called for key release events
	virtual void on_key_up(CVD::GLWindow &win, int key)
	{
		m_keyDown = false;
		m_key = -1;
		if(m_pHandler && m_pFuncKeyUp)
			(m_pHandler->*m_pFuncKeyUp)(key);
	}
	/// Called for mouse movement events
	virtual void on_mouse_move(CVD::GLWindow &win, CVD::ImageRef where, int state)
	{
		m_whereMouseMove = where;
		if(m_pHandler && m_pFuncMouseMove)
			(m_pHandler->*m_pFuncMouseMove)(where);
		m_mouseDraging = m_mouseDown;
		if(m_mouseDown && m_pHandler && m_pFuncMouseDraging)
			(m_pHandler->*m_pFuncMouseDraging)(m_whereMouseDown, where, m_button);
	}
	/// Called for mouse button press events
	virtual void on_mouse_down(CVD::GLWindow &win, CVD::ImageRef where, int state, int button)
	{
		m_mouseDown = true;
		m_button = button;
		if(m_whereMouseDown == where)	// Double click
		{
			m_whereMouseDown.x = -1;
		}
		else
		{
			m_whereMouseDown = where;
			if(m_pHandler && m_pFuncMouseDown)
				(m_pHandler->*m_pFuncMouseDown)(where, button);
		}
	}
	/// Called for mouse button release events
	virtual void on_mouse_up(CVD::GLWindow &win, CVD::ImageRef where, int state, int button)
	{
		if(m_whereMouseDown.x == -1)	//Double click
		{
			if(m_pHandler && m_pFuncMouseDoubleClick)
				(m_pHandler->*m_pFuncMouseDoubleClick)(where, button);
		}
		else
		{
			if(m_pHandler && m_pFuncMouseUp)
				(m_pHandler->*m_pFuncMouseUp)(where, button);
		}
		m_mouseDown = false;
		m_mouseDraging = false;
	}
	/// Called for window resize events
	virtual void on_resize(CVD::GLWindow &win, CVD::ImageRef size)
	{
		if(m_pHandler && m_pFuncResize)
			(m_pHandler->*m_pFuncResize)(size);
	}
	/// Called for general window events (such as EVENT_CLOSE)
	virtual void on_event(CVD::GLWindow &win, int event)
	{
		m_quit = (event == CVD::GLWindow::EVENT_CLOSE);
	}

	inline void Initialize(HANDLER *pHandler, 
		FunctionPointerKeyDown			pFuncKeyDown, 
		FunctionPointerKeyUp			pFuncKeyUp, 
		FunctionPointerMouseMove		pFuncMouseMove, 
		FunctionPointerMouseDown		pFuncMouseDown, 
		FunctionPointerMouseUp			pFuncMouseUp, 
		FunctionPointerMouseDraging		pFuncMouseDraging, 
		FunctionPointerMouseDoubleClick	pFuncMouseDoubleClick, 
		FunctionPointerResize			pFuncResize)
	{
		m_quit = false;
		m_keyDown = false;
		m_mouseDown = false;
		m_pHandler = pHandler;
		m_pFuncKeyDown			= pFuncKeyDown;
		m_pFuncKeyUp			= pFuncKeyUp;
		m_pFuncMouseMove		= pFuncMouseMove;
		m_pFuncMouseDown		= pFuncMouseDown;
		m_pFuncMouseUp			= pFuncMouseUp;
		m_pFuncMouseDraging		= pFuncMouseDraging;
		m_pFuncMouseDoubleClick	= pFuncMouseDoubleClick;
		m_pFuncResize			= pFuncResize;
	}

	inline const bool& Quit() const { return m_quit; }
	inline		 bool& Quit()		{ return m_quit; }
	inline const bool& IsMouseDown() const { return m_mouseDown; }
	inline const bool& IsMouseDraging() const { return m_mouseDraging; }

protected:

	bool m_quit, m_keyDown, m_mouseDown, m_mouseDraging;
	int m_key, m_button;
	CVD::ImageRef m_whereMouseDown, m_whereMouseMove;
	HANDLER *m_pHandler;
	FunctionPointerKeyDown			m_pFuncKeyDown;
	FunctionPointerKeyUp			m_pFuncKeyUp;
	FunctionPointerMouseMove		m_pFuncMouseMove;
	FunctionPointerMouseDown		m_pFuncMouseDown;
	FunctionPointerMouseUp			m_pFuncMouseUp;
	FunctionPointerMouseDraging		m_pFuncMouseDraging;
	FunctionPointerMouseDoubleClick	m_pFuncMouseDoubleClick;
	FunctionPointerResize			m_pFuncResize;

};

#endif