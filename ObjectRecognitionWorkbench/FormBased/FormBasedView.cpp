
// FormBasedView.cpp : implementation of the CFormBasedView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "FormBased.h"
#endif

#include "FormBasedDoc.h"
#include "FormBasedView.h"
#include "Utilities.h"

#include "example.hpp"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

typedef struct
{
	int	Histogram;
	int Label;
	int Average;
	int LeftSlider;
	int RightSlider;
	int LeftSliderValue;
	int RightSliderValue;
	CHistogram CFormBasedView::*pHistogram;
	int ColorFilter::*pLeftFilter;
	int ColorFilter::*pRightFilter;
} HistogramSet;

#define	SET(s)	{		\
	IDC_##s,				\
	IDC_##s##_LABEL,		\
	IDC_##s##_AVERAGE,	\
	IDC_##s##_LEFT,		\
	IDC_##s##_RIGHT,		\
	IDC_##s##_LEFT_VALUE,	\
	IDC_##s##_RIGHT_VALUE,	\
	&CFormBasedView::m_ctrl##s, \
	&ColorFilter::left##s, \
	&ColorFilter::right##s, \
	}

HistogramSet histogramSet[2][3]
{
	{ SET(H), SET(S), SET(V) },
	{ SET(R), SET(G), SET(B) }
};

struct RANSACControls
{
	int		idcLabel;
	int		idcValues[3];
};

RANSACControls ransacControls[] = {
	{ IDC_SAC_MODEL_LABEL, { IDC_SAC_MODEL, 0, 0 } },
	{ IDC_DISTANCE_THRESHHOLD_LABEL, { IDC_DISTANCE_THRESHHOLD, 0 } },
	{ IDC_RADIUS_LIMITS_LABEL, { IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	{ IDC_AXIS_LABEL, { IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z } },
	{ IDC_EPSILON_LABEL, { IDC_EPSILON, 0, 0 } },
	{ IDC_CONE_ANGLE_LABEL, { IDC_CONE_ANGLE_MIN, IDC_CONE_ANGLE_MAX, 0 } }
};

#define	N_RANSAC_CONTROLS	(sizeof(ransacControls) / sizeof(ransacControls[0]))


void
AdjustAspect(int& nOGLWidth, int& nOGLHeight)
{
	if (nOGLHeight*CAMERA_ASPECT_WIDTH > nOGLWidth*CAMERA_ASPECT_HEIGHT)
	{
		nOGLHeight = (int) (nOGLWidth * CAMERA_ASPECT_HEIGHT / CAMERA_ASPECT_WIDTH + 0.5);
	}
	else if (nOGLHeight*CAMERA_ASPECT_WIDTH < nOGLWidth*CAMERA_ASPECT_HEIGHT)
	{
		nOGLWidth = (int)(nOGLHeight * CAMERA_ASPECT_WIDTH / CAMERA_ASPECT_HEIGHT + 0.5);
	}

}

// CFormBasedView

IMPLEMENT_DYNCREATE(CFormBasedView, CFormView)

BEGIN_MESSAGE_MAP(CFormBasedView, CFormView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CFormView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CFormView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CFormView::OnFilePrintPreview)
	ON_WM_SIZE()
	ON_WM_HSCROLL()
	ON_WM_DESTROY()
	ON_WM_MOUSEMOVE()
END_MESSAGE_MAP()

// CFormBasedView construction/destruction

//	noexcept
CFormBasedView::CFormBasedView()
	: CFormView(IDD_FORMBASED_FORM)
	, m_nDepthMinValue(0)
	, m_nDepthMaxValue(1000)
	, m_pPointCloudWindow(NULL)
	, m_pAppState(NULL)
	, m_nSACModel(0)
	, m_fDistanceThreshhold(0.0)
	, m_fRadiusLimitsMin(0.0)
	, m_fRadiusLimitsMax(0.0)
	, m_fAxisX(0.0)
	, m_fAxisY(0.0)
	, m_fAxisZ(0.0)
{
	// TODO: add construction code here

}

CFormBasedView::~CFormBasedView()
{
	if (m_pPointCloudWindow)
	{
		m_pPointCloudWindow->close();
		delete m_pPointCloudWindow;
	}
	if (m_pAppState)
	{
		delete m_pAppState;
	}
}

void CFormBasedView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_OGL_DEPTH_FEED, m_oglDepth);
	DDX_Control(pDX, IDC_OGL_CAMERA_FEED, m_oglCamera);
	DDX_Control(pDX, IDC_H, m_ctrlH);
	DDX_Control(pDX, IDC_S, m_ctrlS);
	DDX_Control(pDX, IDC_V, m_ctrlV);
	DDX_Control(pDX, IDC_R, m_ctrlR);
	DDX_Control(pDX, IDC_G, m_ctrlG);
	DDX_Control(pDX, IDC_B, m_ctrlB);
	DDX_Control(pDX, IDC_H_LABEL, m_ctrlHLabel);
	DDX_Control(pDX, IDC_S_LABEL, m_ctrlSLabel);
	DDX_Control(pDX, IDC_V_LABEL, m_ctrlVLabel);
	DDX_Control(pDX, IDC_R_LABEL, m_ctrlRLabel);
	DDX_Control(pDX, IDC_G_LABEL, m_ctrlGLabel);
	DDX_Control(pDX, IDC_B_LABEL, m_ctrlBLabel);
	DDX_Control(pDX, IDC_H_AVERAGE, m_ctrlHAverage);
	DDX_Control(pDX, IDC_S_AVERAGE, m_ctrlSAverage);
	DDX_Control(pDX, IDC_V_AVERAGE, m_ctrlVAverage);
	DDX_Control(pDX, IDC_R_AVERAGE, m_ctrlRAverage);
	DDX_Control(pDX, IDC_G_AVERAGE, m_ctrlGAverage);
	DDX_Control(pDX, IDC_B_AVERAGE, m_ctrlBAverage);
	DDX_Control(pDX, IDC_H_LEFT, m_ctrlHLeft);
	DDX_Control(pDX, IDC_S_LEFT, m_ctrlSLeft);
	DDX_Control(pDX, IDC_V_LEFT, m_ctrlVLeft);
	DDX_Control(pDX, IDC_R_LEFT, m_ctrlRLeft);
	DDX_Control(pDX, IDC_G_LEFT, m_ctrlGLeft);
	DDX_Control(pDX, IDC_B_LEFT, m_ctrlBLeft);
	DDX_Control(pDX, IDC_H_RIGHT, m_ctrlHRight);
	DDX_Control(pDX, IDC_S_RIGHT, m_ctrlSRight);
	DDX_Control(pDX, IDC_V_RIGHT, m_ctrlVRight);
	DDX_Control(pDX, IDC_R_RIGHT, m_ctrlRRight);
	DDX_Control(pDX, IDC_G_RIGHT, m_ctrlGRight);
	DDX_Control(pDX, IDC_B_RIGHT, m_ctrlBRight);
	DDX_Control(pDX, IDC_H_LEFT_VALUE, m_ctrlHLeftValue);
	DDX_Control(pDX, IDC_S_LEFT_VALUE, m_ctrlSLeftValue);
	DDX_Control(pDX, IDC_V_LEFT_VALUE, m_ctrlVLeftValue);
	DDX_Control(pDX, IDC_R_LEFT_VALUE, m_ctrlRLeftValue);
	DDX_Control(pDX, IDC_G_LEFT_VALUE, m_ctrlGLeftValue);
	DDX_Control(pDX, IDC_B_LEFT_VALUE, m_ctrlBLeftValue);
	DDX_Control(pDX, IDC_H_RIGHT_VALUE, m_ctrlHRightValue);
	DDX_Control(pDX, IDC_S_RIGHT_VALUE, m_ctrlSRightValue);
	DDX_Control(pDX, IDC_V_RIGHT_VALUE, m_ctrlVRightValue);
	DDX_Control(pDX, IDC_R_RIGHT_VALUE, m_ctrlRRightValue);
	DDX_Control(pDX, IDC_G_RIGHT_VALUE, m_ctrlGRightValue);
	DDX_Control(pDX, IDC_B_RIGHT_VALUE, m_ctrlBRightValue);
	DDX_Text(pDX, IDC_H_LEFT_VALUE, m_colorFilter.leftH);
	DDX_Text(pDX, IDC_S_LEFT_VALUE, m_colorFilter.leftS);
	DDX_Text(pDX, IDC_V_LEFT_VALUE, m_colorFilter.leftV);
	DDX_Text(pDX, IDC_R_LEFT_VALUE, m_colorFilter.leftR);
	DDX_Text(pDX, IDC_G_LEFT_VALUE, m_colorFilter.leftG);
	DDX_Text(pDX, IDC_B_LEFT_VALUE, m_colorFilter.leftB);
	DDX_Text(pDX, IDC_H_RIGHT_VALUE, m_colorFilter.rightH);
	DDX_Text(pDX, IDC_S_RIGHT_VALUE, m_colorFilter.rightS);
	DDX_Text(pDX, IDC_V_RIGHT_VALUE, m_colorFilter.rightV);
	DDX_Text(pDX, IDC_R_RIGHT_VALUE, m_colorFilter.rightR);
	DDX_Text(pDX, IDC_G_RIGHT_VALUE, m_colorFilter.rightG);
	DDX_Text(pDX, IDC_B_RIGHT_VALUE, m_colorFilter.rightB);
	DDX_Control(pDX, IDC_FREEZE, m_ctrlFreeze);
	DDX_Control(pDX, IDC_DEPTH_MIN, m_ctrlDepthMin);
	DDX_Control(pDX, IDC_DEPTH_MAX, m_ctrlDepthMax);
	DDX_Control(pDX, IDC_DEPTH_MIN_VALUE, m_ctrlDepthMinValue);
	DDX_Control(pDX, IDC_DEPTH_MAX_VALUE, m_ctrlDepthMaxValue);
	DDX_Text(pDX, IDC_DEPTH_MIN_VALUE, m_nDepthMinValue);
	DDX_Text(pDX, IDC_DEPTH_MAX_VALUE, m_nDepthMaxValue);
	//	DDX_Control(pDX, IDC_OGL_OBJECTS, m_PointCloudViewer);
	//	DDX_Control(pDX, IDC_OGL_OBJECTS, test);
	DDX_Control(pDX, IDC_SAC_MODEL, m_ctrlSACModel);
	DDX_CBIndex(pDX, IDC_SAC_MODEL, m_nSACModel);
	DDX_Text(pDX, IDC_DISTANCE_THRESHHOLD, m_fDistanceThreshhold);
	DDV_MinMaxFloat(pDX, m_fDistanceThreshhold, 0.0, FLT_MAX);
	DDV_MinMaxFloat(pDX, m_fRadiusLimitsMin, 0.0, FLT_MAX);
	DDV_MinMaxFloat(pDX, m_fRadiusLimitsMax, 0.0, FLT_MAX);
	DDV_MinMaxFloat(pDX, m_fAxisX, 0.0, FLT_MAX);
	DDV_MinMaxFloat(pDX, m_fAxisY, 0.0, FLT_MAX);
	DDV_MinMaxFloat(pDX, m_fAxisZ, 0.0, FLT_MAX);
}

BOOL CFormBasedView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CFormView::PreCreateWindow(cs);
}

void CFormBasedView::OnInitialUpdate()
{
	CSliderCtrl* pSlider;
	CFormView::OnInitialUpdate();
	GetParentFrame()->RecalcLayout();
	ResizeParentToFit();

	for (int y = 0; y < 2; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			pSlider = (CSliderCtrl*)GetDlgItem(histogramSet[y][x].LeftSlider);
			pSlider->SetRange(0, 255);
			pSlider->SetPos(0);
			pSlider = (CSliderCtrl*)GetDlgItem(histogramSet[y][x].RightSlider);
			pSlider->SetRange(0, 255);
			pSlider->SetPos(0);
		}
	}

	pSlider = (CSliderCtrl*)GetDlgItem(IDC_DEPTH_MAX);
	pSlider->SetRange(0, 1000);
	pSlider->SetPos(1000);

	pSlider = (CSliderCtrl*)GetDlgItem(IDC_DEPTH_MIN);
	pSlider->SetRange(0, 1000);
	pSlider->SetPos(0);

	UpdateData(FALSE);

	rs2::pipeline_profile profile = m_rsPipe.start();

	rs2::device dev = profile.get_device();

	rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
	m_fDepthScale = ds.get_depth_scale();


	// Setup the OpenGL Window's timer to render
//	m_PointCloudViewer.m_unpTimer = m_PointCloudViewer.SetTimer(1, 1, 0);
}


// CFormBasedView printing

BOOL CFormBasedView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CFormBasedView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CFormBasedView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}

void CFormBasedView::OnPrint(CDC* pDC, CPrintInfo* /*pInfo*/)
{
	// TODO: add customized printing code here
}


// CFormBasedView diagnostics

#ifdef _DEBUG
void CFormBasedView::AssertValid() const
{
	CFormView::AssertValid();
}

void CFormBasedView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}

CFormBasedDoc* CFormBasedView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CFormBasedDoc)));
	return (CFormBasedDoc*)m_pDocument;
}
#endif //_DEBUG


// CFormBasedView message handlers

void CFormBasedView::OnSize(UINT nType, int cx, int cy)
{
	CFormView::OnSize(nType, cx, cy);

	CRect rectClient;
	GetClientRect(&rectClient);
	TRACE(_T("%d x %d\n"), rectClient.Width(), rectClient.Height());

	CRect rectOGLDepthFeed;
	if (!IsWindow(m_oglDepth.m_hWnd))
		return;

	m_oglDepth.GetWindowRect(&rectOGLDepthFeed);

	int nBorder = BORDER;

	// Resize the Camera Feed

	int nOGLWidth = (rectClient.Width() / 2 - 3 * nBorder) / 2;
	int nOGLHeight = (rectClient.Height() / 3 - 2 * nBorder);
	AdjustAspect(nOGLWidth, nOGLHeight);

	rectOGLDepthFeed.left = nBorder;
	rectOGLDepthFeed.top = nBorder;
	rectOGLDepthFeed.right = rectOGLDepthFeed.left + nOGLWidth;
	rectOGLDepthFeed.bottom = rectOGLDepthFeed.top + nOGLHeight;

	m_oglDepth.MoveWindow(rectOGLDepthFeed, TRUE);

	CRect rectOGLCameraFeed = rectOGLDepthFeed;
	rectOGLCameraFeed.left += nOGLWidth + nBorder;
	rectOGLCameraFeed.right = rectOGLCameraFeed.left + nOGLWidth;
	m_oglCamera.MoveWindow(rectOGLCameraFeed, TRUE);

	nOGLWidth = rectClient.Width() / 2 - 2 * nBorder;
	nOGLHeight = rectClient.Height() / 2 - 2 * nBorder;
	AdjustAspect(nOGLWidth, nOGLHeight);

	int left = rectClient.Width() / 2 + nBorder;

	CRect rectOGLObjects(left, nBorder, left + nOGLWidth, nBorder + nOGLHeight);
	GetDlgItem(IDC_OGL_OBJECTS)->MoveWindow(rectOGLObjects, TRUE);
	m_rectPointCloudViewer = rectOGLObjects;

	//if (m_pPointCloudWindow == NULL)
	//{
	//	m_pPointCloudWindow = new window(rectOGLObjects.Width(), rectOGLObjects.Height(), "");
	//	m_pAppState = new glfw_state;
	//}

	//m_pPointCloudWindow->set_size(rectOGLObjects.Width(), rectOGLObjects.Height());
	//CPoint ptTL = rectOGLObjects.TopLeft();
	//ClientToScreen(&ptTL);
	//m_pPointCloudWindow->move(ptTL.x, ptTL.y);

	CRect rectFreeze;
	m_ctrlFreeze.GetClientRect(&rectFreeze);

	m_ctrlFreeze.MoveWindow(
		CRect(
			CPoint(3*rectClient.Width() / 8 - rectFreeze.Width() / 2, rectClient.Height() / 3 - rectFreeze.Height() / 2 - 2*nBorder),
			rectFreeze.Size()),
		TRUE);

	CRect rectDepth, rectDepthValue;
	m_ctrlDepthMin.GetClientRect(&rectDepth);
	m_ctrlDepthMinValue.GetClientRect(&rectDepthValue);
	auto widthMinMax = (rectOGLDepthFeed.Width() - nBorder) / 2 - SMALL_SPACING - rectDepthValue.Width();
	auto heightMinMax = rectDepth.Height();

	rectDepth = CRect(
		CPoint(rectOGLDepthFeed.left + SMALL_SPACING + rectDepthValue.Width(), rectClient.Height() / 3 - heightMinMax / 2 - 2 * nBorder),
		CSize(widthMinMax, heightMinMax));
	m_ctrlDepthMin.MoveWindow(rectDepth, TRUE);

	rectDepth.MoveToX(rectOGLDepthFeed.right - widthMinMax);
	m_ctrlDepthMax.MoveWindow(rectDepth, TRUE);

	rectDepthValue = CRect(
		CPoint(rectOGLDepthFeed.left, rectClient.Height() / 3 - rectDepthValue.Height() / 2 - 2 * nBorder),
		CSize(rectDepthValue.Size()));
	m_ctrlDepthMinValue.MoveWindow(rectDepthValue, TRUE);

	rectDepthValue.MoveToX(rectOGLDepthFeed.right - widthMinMax - SMALL_SPACING - rectDepthValue.Width());
	m_ctrlDepthMaxValue.MoveWindow(rectDepthValue, TRUE);


	CRect rectText;
	m_ctrlHLabel.GetClientRect(&rectText);

	CRect rectSlider;
	m_ctrlHLeft.GetClientRect(&rectSlider);

	CRect rectAverage;
	m_ctrlHAverage.GetClientRect(&rectAverage);

	CRect rectValue;
	m_ctrlHLeftValue.GetClientRect(&rectValue);

	auto histWidth = (rectClient.Width() / 2 - 4 * nBorder) / 3;
	auto textHeight = rectText.Height();
	auto textWidth = rectText.Width();
	auto sliderHeight = rectSlider.Height();

	auto histHeight = (rectClient.Height() * 2 / 3 - 2 * textHeight - 4 * sliderHeight - 5 * nBorder - 6 * SMALL_SPACING)/2;

	if (histHeight > histWidth)
		histHeight = histWidth;
	if (histWidth > histHeight)
		histWidth = histHeight;

	for (int y = 0; y < 2; y++)
	{
		auto yPos = (y+1) * rectClient.Height() / 3;

		for (int x = 0; x < 3; x++)
		{
			auto xPos = 3*nBorder + x * rectClient.Width() / 6;

			CRect rectHist(CPoint(xPos, yPos), CSize(histWidth, histHeight));
			GetDlgItem(histogramSet[y][x].Histogram)->MoveWindow(rectHist, TRUE);

			CRect rectLabel(CPoint(xPos - SMALL_SPACING - textWidth, yPos + (textHeight + histHeight) / 2), rectText.Size());
			GetDlgItem(histogramSet[y][x].Label)->MoveWindow(rectLabel, TRUE);

			CRect rectAverage(CPoint(xPos, yPos + histHeight + SMALL_SPACING), CSize(histWidth, rectAverage.Height()));
			GetDlgItem(histogramSet[y][x].Average)->MoveWindow(rectAverage, TRUE);

			CRect rectLeftSlider = CRect(CPoint(xPos, rectAverage.bottom + SMALL_SPACING), CSize(histWidth, sliderHeight));
			GetDlgItem(histogramSet[y][x].LeftSlider)->MoveWindow(rectLeftSlider, TRUE);

			CRect rectLeftSliderValue(CPoint(xPos - SMALL_SPACING - rectValue.Width(), rectLeftSlider.top), CSize(rectValue.Width(), rectLeftSlider.Height()));
			GetDlgItem(histogramSet[y][x].LeftSliderValue)->MoveWindow(rectLeftSliderValue, TRUE);

			CRect rectRightSlider = rectLeftSlider;
			rectRightSlider.MoveToY(rectLeftSlider.bottom + SMALL_SPACING);
			GetDlgItem(histogramSet[y][x].RightSlider)->MoveWindow(rectRightSlider, TRUE);

			CRect rectRightSliderValue(CPoint(xPos - SMALL_SPACING - rectValue.Width(), rectRightSlider.top), CSize(rectValue.Width(), rectRightSlider.Height()));
			GetDlgItem(histogramSet[y][x].RightSliderValue)->MoveWindow(rectRightSliderValue, TRUE);

		}
	}

	auto ransacX = rectClient.Width() / 2 + nBorder;
	auto ransacY = rectClient.Height() - nBorder;
	CRect rectValues;
	GetDlgItem(IDC_SAC_MODEL)->GetClientRect(&rectValues);
	auto valueWidth = rectValues.Width();
	auto valueHeight = rectValues.Height();

	int yPos = ransacY - rectValues.Height();

	for (int i = N_RANSAC_CONTROLS - 1; i >= 0; i--)
	{
		CRect rectLabel;
		GetDlgItem(ransacControls[i].idcLabel)->GetClientRect(&rectLabel);

		int xPos = ransacX;

		GetDlgItem(ransacControls[i].idcLabel)->MoveWindow(xPos, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
		xPos += rectLabel.Width() + SMALL_SPACING;

		int nControls = 0;
		for (int j = 0; j < 3; j++)
		{
			if (ransacControls[i].idcValues[j])
				nControls++;
		}
		for (int j = 0; j < nControls; j++)
		{
			GetDlgItem(ransacControls[i].idcValues[j])->MoveWindow(xPos, yPos, (valueWidth - (nControls - 1)*SMALL_SPACING) / nControls, valueHeight);
			xPos += valueWidth / nControls + SMALL_SPACING/2;
		}
		yPos -= valueHeight + SMALL_SPACING;
	}
}


void CFormBasedView::NewFrame(rs2::frameset& frames)
{
	for (auto f : frames)
	{
		if (auto vf = f.as<rs2::video_frame>())
		{
					switch (vf.get_profile().stream_type())
					{
					case RS2_STREAM_COLOR:
						m_oglCamera.NewBitmap(vf, m_colorFilter);
						break;
					case RS2_STREAM_DEPTH:
						m_oglDepth.NewBitmap(vf, m_colorFilter);
						break;
					}
		}
	}

}

void CFormBasedView::DoIdleProcessing()
{
	static int nSkip = 30;

	if (m_rsPipe.poll_for_frames(&m_rsFrames))
	{
		if (m_ctrlFreeze.GetCheck())
			return;

		auto depth = m_rsFrames.get_depth_frame();

		// Filter the depth

		auto depthWidth = depth.get_width();
		auto depthHeight = depth.get_height();
		auto depthPoints = depthWidth * depthHeight;
		uint16_t* depthData = (uint16_t*)(depth.as<rs2::depth_frame>()).get_data();

		for (int x = 0; x < depthPoints; x++)
		{
			float dDepthInCm = depthData[x] * m_fDepthScale * 100;
			if (dDepthInCm < m_nDepthMinValue || dDepthInCm > m_nDepthMaxValue)
				depthData[x] = 0;
		}

		FrameHistogram Hist;
		rs2::frameset data = m_rsFrames.apply_filter(m_rsPrinter).apply_filter(m_rsColorMap);
		NewFrame(data);
		ComputeHistogram(m_rsFrames, Hist);

#define INIT_HIST(c, b)			\
	m_ctrl##c.SetData(Hist.c);			\
	m_ctrl##c.Invalidate(TRUE);		\
	m_ctrl##c##Average.SetWindowText(AverageAndSD(Hist.c, b))

		INIT_HIST(H, 1);
		INIT_HIST(S, 0);
		INIT_HIST(V, 0);
		INIT_HIST(R, 0);
		INIT_HIST(G, 0);
		INIT_HIST(B, 0);

		UpdateData(FALSE);

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;

		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points points;

		auto color = m_rsFrames.get_color_frame();

		pc.map_to(color);

		// Generate the pointcloud and texture mappings
//		points = pc.calculate(depth);

//		auto pcl_points = points_to_pcl(points, color);

		//if (m_pPointCloudWindow)
		//{
		//	if (!m_pAppState)
		//	{
		//		m_pAppState = new glfw_state;
		//	}

		//	m_pPointCloudWindow->show(color);

		//	// register callbacks to allow manipulation of the pointcloud
		//	//register_glfw_callbacks(*m_pPointCloudWindow, *m_pAppState);
		//	//draw_pointcloud(m_rectPointCloudViewer.Width(), m_rectPointCloudViewer.Height(), *m_pAppState, points);
		//}

		//window app(1280, 720, "Point Cloud");
		//m_pPointCloudWindow = new window(1280, 720, "PC");
		//m_pAppState = new glfw_state;

		//glfw_state state;

		//////////if (nSkip-- == 0)
		//////////{
		//////////	while (*m_pPointCloudWindow)
		//////////	//for (int i=0; ; i++)
		//////////	{
		//////////		draw_pointcloud(1280, 720, *m_pAppState, points);
		//////////	}
		//////////}
	

		//delete m_pPointCloudWindow;
		//delete m_pAppState;

		//m_Layers.clear();
		//m_Layers.push_back(new Feature(pcl_points));
		//m_PointCloudViewer.SetFeatures(&m_Layers);
	}
	//m_PointCloudViewer.OnTimer(1);
}


void CFormBasedView::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	int value = ((CSliderCtrl*)pScrollBar)->GetPos();
	auto id = pScrollBar->GetDlgCtrlID();

	switch (id)
	{
	case IDC_DEPTH_MIN:
		m_nDepthMinValue = value;
		break;

	case IDC_DEPTH_MAX:
		m_nDepthMaxValue = value;
		break;

	default:
		for (int y = 0; y < 2; y++)
		{
			for (int x = 0; x < 3; x++)
			{
				HistogramSet*	pHS = &histogramSet[y][x];

				if (pHS->LeftSlider == id)
				{
					((*this).*(pHS->pHistogram)).SetLeftSlider(value);
					m_colorFilter.*(pHS->pLeftFilter) = value;
				}
				if (pHS->RightSlider == id)
				{
					((*this).*(pHS->pHistogram)).SetRightSlider(value);
					m_colorFilter.*(pHS->pRightFilter) = value;
				}

			}
		}
	}
	UpdateData(FALSE);


	//BuildBitmapFromFrame();
	//m_Image.Invalidate(TRUE);

	CFormView::OnHScroll(nSBCode, nPos, pScrollBar);
}





void CFormBasedView::OnDestroy()
{
	CFormView::OnDestroy();

	m_rsPipe.stop();
}


void CFormBasedView::OnMouseMove(UINT nFlags, CPoint point)
{
	if (m_rectPointCloudViewer.PtInRect(point)) {
		//TRACE(_T("Move: %d, %d    %d, %d\n"), point.x, point.y, point.x - m_rectPointCloudViewer.left, point.y - m_rectPointCloudViewer.top);
		//m_PointCloudViewer.OnMouseMove(nFlags, point);
	}

	CFormView::OnMouseMove(nFlags, point);
}
