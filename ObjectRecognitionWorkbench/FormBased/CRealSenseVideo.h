#pragma once
#include <afxwin.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utilities.h"

class CRealSenseVideo :
	public CWnd
{
protected:
	CImage	m_Image;

public:
	CRealSenseVideo();
	virtual ~CRealSenseVideo();

	void NewBitmap(rs2::video_frame& frame, const ColorFilter& filter);
	DECLARE_MESSAGE_MAP()
	afx_msg void OnPaint();
};

