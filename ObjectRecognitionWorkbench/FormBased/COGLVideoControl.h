#pragma once
#include <afxwin.h>

class COGLVideoControl :
	public CWnd
{
protected:

public:
	/******************/
	/* Public Members */
	/******************/

	bool	 m_bIsMaximized;

private:
	/*******************/
	/* Private Members */
	/*******************/
	// Window information
	CWnd  *hWnd;
	HDC   hdc;
	HGLRC hrc;
	int   m_nPixelFormat;
	CRect m_rect;
	CRect m_oldWindow;
	CRect m_originalRect;
	GLuint  _gl_handle;
	bool m_bInitted;

public:
	COGLVideoControl();
	virtual ~COGLVideoControl();


	void NewBitmap(GLint internal_format, GLsizei image_width, GLsizei image_height, GLenum format, const GLvoid* pixels);
	void oglInitialize(void);

	// Added message classes:
	afx_msg void OnSize(UINT nType, int cx, int cy);


	DECLARE_MESSAGE_MAP()
};

