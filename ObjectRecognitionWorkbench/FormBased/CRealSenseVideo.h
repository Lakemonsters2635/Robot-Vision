#pragma once
#include <afxwin.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class ColorFilter
{
public:
	ColorFilter()
		: leftH(0)
		, rightH(255)
		, leftS(0)
		, rightS(255)
		, leftV(0)
		, rightV(255)
		, leftR(0)
		, rightR(255)
		, leftG(0)
		, rightG(255)
		, leftB(0)
		, rightB(255)
	{}
	int	leftH;
	int	rightH;
	int	leftS;
	int	rightS;
	int	leftV;
	int	rightV;
	int	leftR;
	int	rightR;
	int	leftG;
	int	rightG;
	int	leftB;
	int	rightB;
};

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

