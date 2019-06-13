#include "stdafx.h"
#include "CPointCloudViewer.h"

IMPLEMENT_DYNAMIC(CPointCloudViewer, CStatic)

CPointCloudViewer::CPointCloudViewer()
	: m_bInitted(false)
	, m_bIsMaximized(false)
	//, m_hWnd(0)
	, _gl_handle(0)
{
}


CPointCloudViewer::~CPointCloudViewer()
{
}

BEGIN_MESSAGE_MAP(CPointCloudViewer, CWnd)
	ON_WM_PAINT()
	ON_WM_SIZE()
END_MESSAGE_MAP()



void CPointCloudViewer::OnPaint()
{
	return;
	//CPaintDC dc(this); // device context for painting

	//CRect rectClient;
	//GetClientRect(&rectClient);

	//CBrush brush;
	//brush.CreateSolidBrush(RGB(255, 0, 0));

	//dc.FillRect(&rectClient, &brush);
	oglInitialize();

	glLoadIdentity();

	// Wireframe Mode
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glBegin(GL_QUADS);
	// Front Side
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);

	// Back Side
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);

	// Top Side
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);

	// Bottom Side
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);

	// Right Side
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);

	// Left Side
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glEnd();

}

void CPointCloudViewer::oglInitialize(void)
{
	if (m_bInitted)
	{
		wglMakeCurrent(m_hdc, m_hrc);
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
	m_hdc = GetDC()->m_hDC;

	// Pixel format.
	m_nPixelFormat = ChoosePixelFormat(m_hdc, &pfd);
	SetPixelFormat(m_hdc, m_nPixelFormat, &pfd);

	// Create the OpenGL Rendering Context.
	m_hrc = wglCreateContext(m_hdc);
	wglMakeCurrent(m_hdc, m_hrc);

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



void CPointCloudViewer::OnSize(UINT nType, int cx, int cy)
{
	CStatic::OnSize(nType, cx, cy);

	if (0 >= cx || 0 >= cy || nType == SIZE_MINIMIZED) return;

	return;

	oglInitialize();
	
	// Map the OpenGL coordinates.
	glViewport(0, 0, cx, cy);

	// Projection view
	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	// Set our current view perspective
	gluPerspective(35.0f, (float)cx / (float)cy, 0.01f, 2000.0f);

	// Model view
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

