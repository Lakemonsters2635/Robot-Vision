#include "stdafx.h"
#include "CRealSenseVideo.h"
#include "Utilities.h"

CRealSenseVideo::CRealSenseVideo()
{
}


CRealSenseVideo::~CRealSenseVideo()
{
}

// return TRUE if pixel should be eliminated.

inline bool Filter(BYTE pixelColor, const int left, const int right)
{
	if (left < right)
	{
		return pixelColor < left || pixelColor > right;
	}
	else
	{
		return pixelColor < left && pixelColor > right;
	}
}

void CRealSenseVideo::NewBitmap(rs2::video_frame& frame, const ColorFilter& filter)
{
	auto format = frame.get_profile().format();
	if (format != RS2_FORMAT_RGB8)
		return;

	auto width = frame.get_width();
	auto height = frame.get_height();
	auto BPP = frame.get_bytes_per_pixel();
	auto stride = frame.get_stride_in_bytes();
	uint8_t* pixels = (uint8_t*)frame.get_data();

	HBITMAP hBitmap = m_Image;
	if (hBitmap == NULL)
	{
		m_Image.Create(width, height, frame.get_bits_per_pixel());
	}

	auto nPitch = m_Image.GetPitch();
	BYTE* pDest = (BYTE*)m_Image.GetBits() + (height - 1)*nPitch;

	BYTE* pSource = pixels + (height - 1)*stride;

	for (int row = 0; row < height; row++)
	{
		BYTE* pDR = pDest + 2;
		BYTE* pDG = pDest + 1;
		BYTE* pDB = pDest;

		BYTE* pSB = pSource + 2;
		BYTE* pSG = pSource + 1;
		BYTE* pSR = pSource;

		for (int col = 0; col < width; col++)
		{
			if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
			{
				uint16_t H, S, V;
				RgbToHsv(*pSR, *pSG, *pSB, H, S, V);

				if (Filter(H, filter.leftH, filter.rightH)
					|| Filter(S, filter.leftS, filter.rightS)
					|| Filter(V, filter.leftV, filter.rightV)
					|| Filter(*pSR, filter.leftR, filter.rightR)
					|| Filter(*pSG, filter.leftG, filter.rightG)
					|| Filter(*pSB, filter.leftB, filter.rightB))
				{
					*pDR = 0;
					*pDG = 0;
					*pDB = 0;
				}
				else
				{
					*pDR = *pSR;
					*pDG = *pSG;
					*pDB = *pSB;
				}
			}
			else
			{
					*pDR = *pSR;
					*pDG = *pSG;
					*pDB = *pSB;
			}

			pDR += 3;
			pDG += 3;
			pDB += 3;
			pSR += 3;
			pSG += 3;
			pSB += 3;
		}
		//memcpy(pDest, pSource, BPP*width);
		pDest = (BYTE*)pDest - nPitch;
		pSource = (BYTE*)pSource - stride;
	}

	Invalidate();
}
BEGIN_MESSAGE_MAP(CRealSenseVideo, CWnd)
	ON_WM_PAINT()
END_MESSAGE_MAP()


void CRealSenseVideo::OnPaint()
{
	CPaintDC dc(this); // device context for painting
					   // TODO: Add your message handler code here
					   // Do not call CWnd::OnPaint() for painting messages
	HBITMAP hBitmap = m_Image;
	if (hBitmap == NULL)
		return;

	CRect rectClient;
	GetClientRect(&rectClient);

	dc.SetStretchBltMode(COLORONCOLOR);

	m_Image.StretchBlt(dc, rectClient);
}
