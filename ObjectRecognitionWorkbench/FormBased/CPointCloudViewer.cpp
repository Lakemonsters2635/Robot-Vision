#include "stdafx.h"
#include "CPointCloudViewer.h"
#include <vector>

#undef max
#undef min

struct byte3
{
	unsigned char r, g, b;
};

static byte3 colorsb[] = {
	{ 255, 255, 255 },
	{ 240, 41, 12 },
	{ 240, 139, 12 },
	{ 240, 233, 12 },
	{ 19, 240, 12 },
	{ 12, 240, 218 },
	{ 12, 157, 240 },
	{ 12, 70, 240 },
	{ 99, 12, 240 },
	{ 193, 12, 240 },
	{ 240, 12, 208 }
};


// Handles all the OpenGL calls needed to display the point cloud
void CPointCloudViewer::draw_pointcloud(/*state& app_state, */const std::vector<feature_ptr>& features)
{
	// OpenGL commands that prep screen for the pointcloud
	//glPopMatrix();
	auto gerr = glGetError();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	gerr = glGetError();

	CRect rectClient;
	GetClientRect(&rectClient);

	float width = rectClient.Width(), height = rectClient.Height();

	//	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClearColor(0.0, 0.0, 0.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	//glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	//glRotated(app_state.pitch, 1, 0, 0);
	//glRotated(app_state.yaw, 0, 1, 0);
	//glTranslatef(0, 0, -0.5f);

	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -m_fZoom);
	glTranslatef(m_fPosX, m_fPosY, 0.0f);
	glRotatef(m_fRotX, 1.0f, 0.0f, 0.0f);
	glRotatef(m_fRotY, 0.0f, 1.0f, 0.0f);

	//glTranslatef(0, 0, +0.5f);
	//glRotated(0, 1, 0, 0);
	//glRotated(0, 0, 1, 0);
	//glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& f : features)
	{
		if (m_nCoefSel != -1 && color == m_nCoefSel)
		{
			if (f->coefficients)
			{
				std::cerr << "Model Coefficients: " << *f->coefficients << std::endl;
				double Nx = f->coefficients->values[0];
				double Nz = f->coefficients->values[2];
				float theta = acos(Nz / sqrt(Nx*Nx + Nz * Nz));
				std::cerr << "Error Angle: " << theta * 180 / 3.1415926 << " degrees" << std::endl;
			}

			m_nCoefSel = -1;
		}

		auto pc = f->cloud;
		if (m_bbMask & (1LL << color))
		{
			//		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
			auto c = colorsb[(color) % (sizeof(colorsb) / sizeof(byte3))];

			glBegin(GL_POINTS);
			glColor3f(c.r / 255.0, c.g / 255.0, c.b / 255.0);

			/* this segment actually prints the pointcloud */
			for (int i = 0; i < pc->points.size(); i++)
			{
				auto&& p = pc->points[i];
				if (p.z)
				{
					// upload the point and texture coordinates only for points we have depth data for
					glVertex3f(p.x, p.y, p.z);
				}
			}

			glEnd();
			gerr = glGetError();
			int xlksld = 0;
		}
		color++;
	}

	gerr = glGetError();
	// OpenGL cleanup
	glPopMatrix();
	gerr = glGetError();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	gerr = glGetError();
	glPopAttrib();
	gerr = glGetError();
	glPushMatrix();
	gerr = glGetError();
}


CPointCloudViewer::CPointCloudViewer(void)
{
	m_fPosX = 0.0f;		// X position of model in camera view
	m_fPosY = 0.0f;		// Y position of model in camera view
	m_fZoom = 10.0f/*-0.5f*/;	// Zoom on model in camera view
	m_fRotX = 0.0f;		// Rotation on model in camera view
	m_fRotY = 0.0f;		// Rotation on model in camera view
	m_bIsMaximized = false;
	m_bInitted = false;
	m_nCoefSel = -1;
	m_bbMask = -1;
	m_pFeatures = NULL;
}

CPointCloudViewer::~CPointCloudViewer(void)
{
}

BEGIN_MESSAGE_MAP(CPointCloudViewer, CWnd)
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_CREATE()
	ON_WM_MOUSEMOVE()
	ON_WM_TIMER()
END_MESSAGE_MAP()

void CPointCloudViewer::OnPaint()
{
	//CPaintDC dc(this); // device context for painting
	ValidateRect(NULL);
}

void CPointCloudViewer::OnSize(UINT nType, int cx, int cy)
{
	CWnd::OnSize(nType, cx, cy);

	if (0 >= cx || 0 >= cy || nType == SIZE_MINIMIZED) return;

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

int CPointCloudViewer::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CWnd::OnCreate(lpCreateStruct) == -1) return -1;

	oglInitialize();

	return 0;
}

void CPointCloudViewer::OnDraw(CDC *pDC)
{
	// If the current view is perspective...
	glLoadIdentity();

	glTranslatef(0.0f, 0.0f, -m_fZoom);
	glTranslatef(m_fPosX, m_fPosY, 0.0f);
	glRotatef(m_fRotX, 1.0f, 0.0f, 0.0f);
	glRotatef(m_fRotY, 0.0f, 1.0f, 0.0f);
}

void CPointCloudViewer::OnTimer(UINT_PTR nIDEvent)
{
	switch (nIDEvent)
	{
	case 1:
	{
		// Clear color and depth buffer bits
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Draw OpenGL scene
		oglDrawScene();

		// Swap buffers
		SwapBuffers(hdc);

		break;
	}

	default:
		break;
	}

	CWnd::OnTimer(nIDEvent);
}

void CPointCloudViewer::OnMouseMove(UINT nFlags, CPoint point)
{
	int diffX = (int)(point.x - m_fLastX);
	int diffY = (int)(point.y - m_fLastY);
	m_fLastX = (float)point.x;
	m_fLastY = (float)point.y;

	// Left mouse button
	if (nFlags & MK_LBUTTON)
	{
		m_fRotX += (float)0.5f * diffY;

		if ((m_fRotX > 360.0f) || (m_fRotX < -360.0f))
		{
			m_fRotX = 0.0f;
		}

		m_fRotY += (float)0.5f * diffX;

		if ((m_fRotY > 360.0f) || (m_fRotY < -360.0f))
		{
			m_fRotY = 0.0f;
		}
	}

	// Right mouse button
	else if (nFlags & MK_RBUTTON)
	{
		m_fZoom -= (float)0.1f * diffY;
	}

	// Middle mouse button
	else if (nFlags & MK_MBUTTON)
	{
		m_fPosX += (float)0.05f * diffX;
		m_fPosY -= (float)0.05f * diffY;
	}

	OnDraw(NULL);

	CWnd::OnMouseMove(nFlags, point);
}

void CPointCloudViewer::oglCreate(CRect rect, CWnd *parent)
{
	CString className = AfxRegisterWndClass(CS_HREDRAW | CS_VREDRAW | CS_OWNDC, NULL, (HBRUSH)GetStockObject(BLACK_BRUSH), NULL);

	CreateEx(0, className, _T("OpenGL"), WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, rect, parent, 0);

	// Set initial variables' values
	m_oldWindow = rect;
	m_originalRect = rect;

	hWnd = parent;
}

void CPointCloudViewer::oglInitialize(void)
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

	// Send draw request
	OnDraw(NULL);
}

void CPointCloudViewer::oglDrawScene(void)
{
	oglInitialize();

	if (m_pFeatures)
		draw_pointcloud(*m_pFeatures);
	return;

	//OnDraw(NULL);

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

