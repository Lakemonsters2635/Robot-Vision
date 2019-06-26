#include "stdafx.h"
#include "COGLVideoControl.h"

#undef min
#undef max

COGLVideoControl::COGLVideoControl()
{
	m_bIsMaximized = false;
	m_bInitted = false;
	m_hWnd = 0;
	_gl_handle = 0;
}


COGLVideoControl::~COGLVideoControl()
{
}
BEGIN_MESSAGE_MAP(COGLVideoControl, CWnd)
	ON_WM_SIZE()
//	ON_WM_CREATE()
END_MESSAGE_MAP()


void COGLVideoControl::OnSize(UINT nType, int cx, int cy)
{
	CWnd::OnSize(nType, cx, cy);

	if (0 >= cx || 0 >= cy || nType == SIZE_MINIMIZED) return;

	oglInitialize();

	// Map the OpenGL coordinates.
	glViewport(0, 0, cx, cy);

	// Projection view
	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	//// Set our current view perspective
	gluPerspective(35.0f, (float)cx / (float)cy, 0.01f, 2000.0f);

	//// Model view
	glMatrixMode(GL_MODELVIEW);

	switch (nType)
	{
		// If window resize token is "maximize"
	case SIZE_MAXIMIZED:
	{
		// Get the current window rect
		GetWindowRect(m_rect);

		// Move the window accordingly
		MoveWindow(6, 6, cx - 14, cy - 14);

		// Get the new window rect
		GetWindowRect(m_rect);

		// Store our old window as the new rect
		m_oldWindow = m_rect;

		break;
	}

	// If window resize token is "restore"
	case SIZE_RESTORED:
	{
		// If the window is currently maximized
		if (m_bIsMaximized)
		{
			// Get the current window rect
			GetWindowRect(m_rect);

			// Move the window accordingly (to our stored old window)
			MoveWindow(m_oldWindow.left, m_oldWindow.top - 18, m_originalRect.Width() - 4, m_originalRect.Height() - 4);

			// Get the new window rect
			GetWindowRect(m_rect);

			// Store our old window as the new rect
			m_oldWindow = m_rect;
		}

		break;
	}
	}
}

void COGLVideoControl::oglInitialize(void)
{
	if (m_bInitted)
	{
		return;
	}

	m_bInitted = true;

	// Initial Setup:
	//
	static PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32, // bit depth
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		16, // z-buffer depth
		0, 0, 0, 0, 0, 0, 0,
	};

	// Get device context only once.
	hdc = GetDC()->m_hDC;

	// Pixel format.
	m_nPixelFormat = ChoosePixelFormat(hdc, &pfd);
	SetPixelFormat(hdc, m_nPixelFormat, &pfd);

	// Create the OpenGL Rendering Context.
	hrc = wglCreateContext(hdc);
	wglMakeCurrent(hdc, hrc);

	// Basic Setup:
	//
	// Set color to use when clearing the background.
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);

	// Turn on backface culling
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);

	// Turn on depth testing
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

}

void COGLVideoControl::NewBitmap(GLint internal_format, GLsizei image_width, GLsizei image_height, GLenum format, const GLvoid* pixels)
{
	oglInitialize();
	GLenum err;

	wglMakeCurrent(hdc, hrc);

	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!_gl_handle)
		glGenTextures(1, &_gl_handle);
	err = glGetError();

	glBindTexture(GL_TEXTURE_2D, _gl_handle);
	err = glGetError();
	glTexImage2D(GL_TEXTURE_2D, 0, internal_format, image_width, image_height, 0, format, GL_UNSIGNED_BYTE, pixels);
	err = glGetError();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	err = glGetError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	err = glGetError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	err = glGetError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	err = glGetError();
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	err = glGetError();
	glBindTexture(GL_TEXTURE_2D, 0);
	err = glGetError();

	CRect rectClient;
	GetClientRect(&rectClient);
	GLsizei width = rectClient.Width();
	GLsizei height = rectClient.Height();

	glViewport(0, 0, width, height);
	err = glGetError();
	glLoadIdentity();
	err = glGetError();
	glMatrixMode(GL_PROJECTION);
	err = glGetError();
	glOrtho(0, width, height, 0, -1, +1);
	err = glGetError();

	glBindTexture(GL_TEXTURE_2D, _gl_handle);
	err = glGetError();
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	err = glGetError();
	glEnable(GL_TEXTURE_2D);
	err = glGetError();
	glBegin(GL_QUADS);
	err = glGetError();
	glTexCoord2f(0, 0); glVertex2f(0, 0);
	err = glGetError();
	glTexCoord2f(0, 1); glVertex2f(0, 1.0f*height);
	err = glGetError();
	glTexCoord2f(1, 1); glVertex2f(1.0f*width, 1.0f*height);
	err = glGetError();
	glTexCoord2f(1, 0); glVertex2f(1.0f*width, 0);
	err = glGetError();
	glEnd();
	err = glGetError();

	glDisable(GL_TEXTURE_2D);
	err = glGetError();
	glBindTexture(GL_TEXTURE_2D, 0);
	err = glGetError();

//	oglDrawScene();

	// Swap buffers
	SwapBuffers(hdc);

	wglMakeCurrent(0, 0);
	return;

}


