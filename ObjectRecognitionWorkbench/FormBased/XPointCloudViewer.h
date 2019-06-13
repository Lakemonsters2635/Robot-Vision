#pragma once
#include <afxwin.h>
class CPointCloudViewer :
	public CStatic
{
	DECLARE_DYNAMIC(CPointCloudViewer)

public:
	CPointCloudViewer();
	virtual ~CPointCloudViewer();

public:
	bool	 m_bIsMaximized;

private:
	HDC m_hdc;
	HGLRC m_hrc;
	int   m_nPixelFormat;
	CRect m_rect;
	CRect m_oldWindow;
	CRect m_originalRect;
	GLuint  _gl_handle;
	bool m_bInitted;

protected:

	DECLARE_MESSAGE_MAP()
public:
	void oglInitialize(void);

	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
};
