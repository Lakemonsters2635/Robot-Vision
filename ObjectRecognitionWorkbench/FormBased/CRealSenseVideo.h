#pragma once
#include <afxwin.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class CRealSenseVideo :
	public CWnd
{
protected:
	CImage	m_Image;

public:
	CRealSenseVideo();
	virtual ~CRealSenseVideo();

	void CreateImage(int nWidth, int nHeight, int nbPP);
	void DrawFrame(rs2::video_frame& frame);
	DECLARE_MESSAGE_MAP()
	afx_msg void OnPaint();
};

