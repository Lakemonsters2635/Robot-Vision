
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
#include "CChooseCamera.h"
#include "CGaussianSettings.h"
#include "CCannySettings.h"

#include <librealsense2/rs_advanced_mode.hpp>


//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

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
//	{ IDC_CONE_ANGLE_LABEL, { IDC_CONE_ANGLE_MIN, IDC_CONE_ANGLE_MAX, 0 } }
	{ IDC_EPSILON_LABEL, { IDC_EPSILON, 0, 0 } },
	{ IDC_AXIS_LABEL, { IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z } },
	{ IDC_RADIUS_LIMITS_LABEL, { IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	{ IDC_DISTANCE_THRESHHOLD_LABEL, { IDC_DISTANCE_THRESHHOLD, 0 } },
	{ IDC_MAX_ITERATIONS_LABEL, { IDC_MAX_ITERATIONS, 0, 0 } },
	{ IDC_SAC_MODEL_LABEL, { IDC_SAC_MODEL, 0, 0 } },
};

struct RANSACModels
{
	enum pcl::SacModel	model;
	LPCTSTR				model_text;
	int					enables[20];
};

RANSACModels ransacModels[] = {
	{ pcl::SACMODEL_PLANE, _T("Plane"), { 0 } },
	{ pcl::SACMODEL_LINE, _T("Line"), { 0 } },
	{ pcl::SACMODEL_CIRCLE2D, _T("Circle2D"), 
		{ IDC_RADIUS_LIMITS_LABEL, IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	//{ pcl::SACMODEL_CIRCLE3D, _T("Circle3D"), 
	//	{ IDC_RADIUS_LIMITS_LABEL, IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	{ pcl::SACMODEL_SPHERE, _T("Sphere"), 
		{ IDC_RADIUS_LIMITS_LABEL, IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	{ pcl::SACMODEL_CYLINDER, _T("Cylinder"), 
		{ IDC_AXIS_LABEL, IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z, 
		  IDC_EPSILON_LABEL, IDC_EPSILON, 
		  IDC_RADIUS_LIMITS_LABEL, IDC_RADIUS_LIMITS_MIN, IDC_RADIUS_LIMITS_MAX, 0 } },
	//{ pcl::SACMODEL_CONE, _T("Cone"),
	//	{ IDC_AXIS_LABEL, IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z,
	//	  IDC_EPSILON_LABEL, IDC_EPSILON,
	//	  IDC_CONE_ANGLE_LABEL, IDC_CONE_ANGLE_MIN, IDC_CONE_ANGLE_MAX, 0 } },
//	{ pcl::SACMODEL_TORUS, _T("Torus"), { 0 } },
	{ pcl::SACMODEL_PARALLEL_LINE, _T("Parallel Line"),
		{ IDC_AXIS_LABEL, IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z,
		  IDC_EPSILON_LABEL, IDC_EPSILON, 0 } },
	{ pcl::SACMODEL_PERPENDICULAR_PLANE, _T("Perpendicular Plane"),
		{ IDC_AXIS_LABEL, IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z,
		  IDC_EPSILON_LABEL, IDC_EPSILON, 0 } },
//	{ pcl::SACMODEL_PARALLEL_LINES, _T("Parallel Lines"), { 0 } },
//	{ pcl::SACMODEL_NORMAL_PLANE, _T("Normal Plane"), { 0 } },
//	{ pcl::SACMODEL_NORMAL_SPHERE, _T("Normal Sphere"), { 0 } },
//	{ pcl::SACMODEL_REGISTRATION, _T("Registration"), { 0 } },
//	{ pcl::SACMODEL_REGISTRATION_2D, _T("Registration2D"), { 0 } },
	{ pcl::SACMODEL_PARALLEL_PLANE, _T("Parallel Plane"),
		{ IDC_AXIS_LABEL, IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z,
		  IDC_EPSILON_LABEL, IDC_EPSILON, 0 } },
//	{ pcl::SACMODEL_NORMAL_PARALLEL_PLANE, _T("Normal Parallel Plane"), { 0 } },
//	{ pcl::SACMODEL_STICK, _T("Stick"), { 0 } },
};

static ColorFilter noFilter;

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
	ON_CBN_SELCHANGE(IDC_SAC_MODEL, OnSelChangeSacModel)
	ON_BN_CLICKED(IDC_ENABLE_VOXEL_FILTER, &CFormBasedView::OnBnClickedEnableVoxelFilter)
	ON_EN_CHANGE(IDC_VOXEL_X, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_VOXEL_Y, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_VOXEL_Z, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_DISTANCE_THRESHHOLD, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_RADIUS_LIMITS_MIN, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_RADIUS_LIMITS_MAX, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_AXIS_X, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_AXIS_Y, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_AXIS_Z, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_EPSILON, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_CONE_ANGLE_MIN, &CFormBasedView::OnEnChange)
	ON_EN_CHANGE(IDC_CONE_ANGLE_MAX, &CFormBasedView::OnEnChange)
	ON_BN_CLICKED(IDC_GO, &CFormBasedView::OnBnClickedGo)
	ON_BN_CLICKED(IDC_SAVE_PCD, &CFormBasedView::OnBnClickedSavePcd)
	ON_BN_CLICKED(IDC_CLEAR_LOG, &CFormBasedView::OnBnClickedClearLog)
	ON_BN_CLICKED(IDC_GAUSSIAN_SETTINGS, &CFormBasedView::OnBnClickedGaussianSettings)
	ON_BN_CLICKED(IDC_CANNY_SETTINGS, &CFormBasedView::OnBnClickedCannySettings)
	ON_BN_CLICKED(IDC_GAUSSIAN_ENABLE, &CFormBasedView::OnBnClickedGaussianEnable)
	ON_BN_CLICKED(IDC_CANNY_ENABLE, &CFormBasedView::OnBnClickedCannyEnable)
END_MESSAGE_MAP()

	// CFormBasedView construction/destruction

	//	
CFormBasedView::CFormBasedView()
	noexcept
: CFormView(IDD_FORMBASED_FORM)
, m_fDepthScale(0.0)
, m_fDepthMinValue(0.0)
, m_fDepthMaxValue(10.00)
, m_pPointCloudWindow(NULL)
, m_pAppState(NULL)
, m_nSACModel(0)
, m_nMaxIterations(1000)
, m_fDistanceThreshhold(0.0)
, m_fRadiusLimitsMin(0.0)
, m_fRadiusLimitsMax(0.0)
, m_fAxisX(0.0)
, m_fAxisY(0.0)
, m_fAxisZ(0.0)
, m_fEpsilon(0.0)
, m_fConeAngleMin(0.0)
, m_fConeAngleMax(0.0)
, m_fVoxelX(0)
, m_fVoxelY(0)
, m_fVoxelZ(0)
, m_bEnableVoxelFilter(0)
, m_strAlignment(_T("Color"))
, m_bFreeze(TRUE)
, m_dwEdgeDetector(0) 
, m_prsPipe(NULL)
, m_bGaussianEnable(FALSE)
, m_sizeGaussian(CSize(9, 9))
, m_dSigmaXGaussian(2.0)
, m_dSigmaYGaussian(0.0)
, m_nBorderTypeGaussian(4)
, m_bCannyEnable(FALSE)
, m_dThreshhold1Canny(100)
, m_dThreshhold2Canny(100)
, m_nApertureCanny(3)
, m_bL2GradientCanny(false)
{
}

CFormBasedView::~CFormBasedView()
{
	//if (m_pPointCloudWindow)
	//{
	//	m_pPointCloudWindow->close();
	//	delete m_pPointCloudWindow;
	//}
	//if (m_pAppState)
	//{
	//	delete m_pAppState;
	//}
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
	DDX_Control(pDX, IDC_LOG, m_ctrlLog);
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
	DDX_Text(pDX, IDC_DEPTH_MIN_VALUE, m_fDepthMinValue);
	DDX_Text(pDX, IDC_DEPTH_MAX_VALUE, m_fDepthMaxValue);
	//	DDX_Control(pDX, IDC_OGL_OBJECTS, m_PointCloudViewer);
	//	DDX_Control(pDX, IDC_OGL_OBJECTS, test);
	DDX_Control(pDX, IDC_SAC_MODEL, m_ctrlSACModel);
	DDX_Text(pDX, IDC_MAX_ITERATIONS, m_nMaxIterations);
	DDV_MinMaxFloat(pDX, m_nMaxIterations, 1, 10000);
	DDX_Text(pDX, IDC_DISTANCE_THRESHHOLD, m_fDistanceThreshhold);
	DDV_MinMaxFloat(pDX, m_fDistanceThreshhold, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_RADIUS_LIMITS_MIN, m_fRadiusLimitsMin);
	DDV_MinMaxFloat(pDX, m_fRadiusLimitsMin, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_RADIUS_LIMITS_MAX, m_fRadiusLimitsMax);
	DDV_MinMaxFloat(pDX, m_fRadiusLimitsMax, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_AXIS_X, m_fAxisX);
	DDV_MinMaxFloat(pDX, m_fAxisX, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_AXIS_Y, m_fAxisY);
	DDV_MinMaxFloat(pDX, m_fAxisY, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_AXIS_Z, m_fAxisZ);
	DDV_MinMaxFloat(pDX, m_fAxisZ, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_EPSILON, m_fEpsilon);
	DDV_MinMaxFloat(pDX, m_fEpsilon, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_CONE_ANGLE_MIN, m_fConeAngleMin);
	DDV_MinMaxFloat(pDX, m_fConeAngleMin, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_CONE_ANGLE_MAX, m_fConeAngleMax);
	DDV_MinMaxFloat(pDX, m_fConeAngleMax, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_VOXEL_X, m_fVoxelX);
	DDV_MinMaxFloat(pDX, m_fVoxelX, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_VOXEL_Y, m_fVoxelY);
	DDV_MinMaxFloat(pDX, m_fVoxelY, 0.0, FLT_MAX);
	DDX_Text(pDX, IDC_VOXEL_Z, m_fVoxelZ);
	DDV_MinMaxFloat(pDX, m_fVoxelZ, 0.0, FLT_MAX);
	DDX_Check(pDX, IDC_ENABLE_VOXEL_FILTER, m_bEnableVoxelFilter);
	DDX_Check(pDX, IDC_FREEZE, m_bFreeze);
	DDX_CBString(pDX, IDC_ALIGNMENT, m_strAlignment);
	m_ctrlEdgeDetector.DDX(pDX, m_dwEdgeDetector);
	DDX_Control(pDX, IDC_EDGE_DETECTOR, m_ctrlEdgeDetector);
	DDX_Check(pDX, IDC_GAUSSIAN_ENABLE, m_bGaussianEnable);
	DDX_Check(pDX, IDC_CANNY_ENABLE, m_bCannyEnable);
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

// Set the ranges for the color sliders

	for (int y = 0; y < 2; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			HistogramSet*	pHS = &histogramSet[y][x];

			pSlider = (CSliderCtrl*)GetDlgItem(pHS->LeftSlider);
			pSlider->SetRange(0, 255);
			pSlider->SetPos(m_colorFilter.*(pHS->pLeftFilter));
			pSlider = (CSliderCtrl*)GetDlgItem(pHS->RightSlider);
			pSlider->SetRange(0, 255);
			pSlider->SetPos(m_colorFilter.*(pHS->pRightFilter));
			
		}
	}

	// Set the ranges for the depth sliders

	pSlider = (CSliderCtrl*)GetDlgItem(IDC_DEPTH_MAX);
	pSlider->SetRange(0, 1000);
	pSlider->SetPos((int) (m_fDepthMaxValue*100));

	pSlider = (CSliderCtrl*)GetDlgItem(IDC_DEPTH_MIN);
	pSlider->SetRange(0, 1000);
	pSlider->SetPos((int)(m_fDepthMinValue * 100));

	UpdateData(FALSE);

// If we don't have a camera started, yet.

	if (m_fDepthScale == 0)
	{
// Get a list of connected RS devices

		rs2::context ctx;            // Create librealsense context for managing devices
		auto devices = ctx.query_devices();

// If none connected, abort

		if (devices.size() == 0)
		{
			::AfxMessageBox(_T("No RealSense Cameras Found"), MB_ICONEXCLAMATION | MB_OK);
			PostQuitMessage(0);
			return;
		}

// Build a list of depth cameras of D435 series

		std::vector<rs2::device> depthCameras;

		for (auto&& dev : devices)
		{
			CString strCameraName(dev.get_info(RS2_CAMERA_INFO_NAME));
			CString strCameraSerialNumber(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			
			TRACE(_T("%s: %s\n"), strCameraName, strCameraSerialNumber);

			if (strCameraName.Find(_T("D435")) != -1)
				depthCameras.push_back(dev);
		}

// DId we find any?

		if (depthCameras.size() == 0)
		{
			::AfxMessageBox(_T("No RealSense Depth Cameras Found"), MB_ICONEXCLAMATION | MB_OK);
			PostQuitMessage(0);
			return;
		}

// If there's more than 1 depth camera connected, allow user to choose 1.  Default camera is #0

		int nCamera = 0;

		if (depthCameras.size() > 1)
		{
			CChooseCamera dlg(depthCameras);

			for (;;)
			{
				if (dlg.DoModal() != IDOK)
				{
					PostQuitMessage(0);
					return;
				}
				if (dlg.m_nSelectedCamera == -1)
				{
					::AfxMessageBox(_T("Please select a camera"), MB_ICONEXCLAMATION | MB_OK);
				}
				else
				{
					nCamera = dlg.m_nSelectedCamera;
					break;
				}
			}
		}

// start the camera pipeline

		rs2::config cfg;
		cfg.enable_device(depthCameras[nCamera].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

		m_prsPipe = new rs2::pipeline(ctx);
		
		rs2::pipeline_profile profile = m_prsPipe->start();

// For learning: in DEBUG mode, list the camera options

		rs2::device dev = profile.get_device();

		auto s = dev.query_sensors();
		rs2::depth_sensor ds = s.front().as<rs2::depth_sensor>();
		m_fDepthScale = ds.get_depth_scale();

		for (int i = 0; i < RS2_OPTION_COUNT; i++)
		{
			if (ds.supports(rs2_option(i)))
			{
				rs2::option_range range = ds.get_option_range((rs2_option(i)));
				CString strDesc(ds.get_option_description((rs2_option(i))));
				float fOption = ds.get_option(rs2_option(i));
				//CString strValueDesc(ds.get_option_value_description((rs2_option(i)), fOption));

				TRACE(_T("%2d %.1f (%.1f - %.1f x %.1f: %.1f) %s\n"), i, fOption, range.min, range.max, range.step, range.def, strDesc/*, strValueDesc*/);
			}
		}
	}

// Populate the SAC Model combo box

	if (m_RansacIds.GetSize() == 0)
	{
		m_ctrlSACModel.ResetContent();
		int nIdx = m_ctrlSACModel.AddString(_T(""));
		m_ctrlSACModel.SetItemData(nIdx, 0);

		for (auto& ransacModel : ransacModels)
		{
			int nIdx = m_ctrlSACModel.AddString(ransacModel.model_text);
			m_ctrlSACModel.SetItemData(nIdx, ransacModel.model);
			int testID;
//			for (int j = 0; (testID = ransacModel.enables[j]); j++)
			for (auto& testID : ransacModel.enables)
			{
				if (testID == 0)
					break;
				int k;
				for (k = 0; k < m_RansacIds.GetSize(); k++)
				{
					if (m_RansacIds[k] == testID)
						break;
				}
				if (k == m_RansacIds.GetSize())
				{
					m_RansacIds.Add(testID);
				}
			}
		}

		UpdateData(FALSE);
	}

// Set the current selected in the SAC model combo box

	for (int i = 0; i < m_ctrlSACModel.GetCount(); i++)
	{
		if (m_ctrlSACModel.GetItemData(i) == m_nSACModel)
		{
			m_ctrlSACModel.SetCurSel(i);
			break;
		}
	}

// Populate the Edge Detector CheckCombo box.  Also, verify that the enum constants are in the same order as the strings

	if (m_ctrlEdgeDetector.GetCount() == 0)
	{
		assert(EF_NANBOUNDARY == 0); m_ctrlEdgeDetector.AddString(_T("Nan Boundary"));
		assert(EF_OCCLUDING == 1); m_ctrlEdgeDetector.AddString(_T("Occluding"));
		assert(EF_OCCLUDED == 2); m_ctrlEdgeDetector.AddString(_T("Occluded"));
		assert(EF_HIGHCURVATURE == 3); m_ctrlEdgeDetector.AddString(_T("High Curvature"));
		assert(EF_RGB == 4); m_ctrlEdgeDetector.AddString(_T("RGB"));
	}


// Update the UI

	OnSelChangeSacModel();		
	OnEnChange();
	OnBnClickedEnableVoxelFilter();
	OnBnClickedGaussianEnable();
	OnBnClickedCannyEnable();

	GetDocument()->SetModifiedFlag(FALSE);

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

// This code lays out the controls when the dialog box is created or resized.

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
	int xPos;

	CRect rectLabel;
	GetDlgItem(ransacControls[0].idcLabel)->GetClientRect(&rectLabel);


	for (auto& ransacControl : ransacControls)
	{
		xPos = ransacX;

		GetDlgItem(ransacControl.idcLabel)->MoveWindow(xPos, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
		xPos += rectLabel.Width() + SMALL_SPACING;

		int nControls = 0;
		for (int j = 0; j < 3; j++)
		{
			if (ransacControl.idcValues[j])
				nControls++;
		}
		for (int j = 0; j < nControls; j++)
		{
			GetDlgItem(ransacControl.idcValues[j])->MoveWindow(xPos, yPos, (valueWidth - (nControls - 1)*SMALL_SPACING) / nControls, valueHeight);
			xPos += valueWidth / nControls + SMALL_SPACING / 2;
		}
		yPos -= valueHeight + SMALL_SPACING;
	}

	yPos -= 5 * SMALL_SPACING;
	xPos = ransacX;
	GetDlgItem(IDC_ENABLE_VOXEL_FILTER)->MoveWindow(xPos, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
	xPos += rectLabel.Width() + SMALL_SPACING;
	GetDlgItem(IDC_VOXEL_X)->MoveWindow(xPos, yPos, (valueWidth - 2 * SMALL_SPACING) / 3, valueHeight);
	xPos += valueWidth / 3 + SMALL_SPACING / 2;
	GetDlgItem(IDC_VOXEL_Y)->MoveWindow(xPos, yPos, (valueWidth - 2 * SMALL_SPACING) / 3, valueHeight);
	xPos += valueWidth / 3 + SMALL_SPACING / 2;
	GetDlgItem(IDC_VOXEL_Z)->MoveWindow(xPos, yPos, (valueWidth - 2 * SMALL_SPACING) / 3, valueHeight);
	xPos += valueWidth / 3 + SMALL_SPACING / 2;

	yPos -= rectLabel.Height() + 5 * SMALL_SPACING;
	GetDlgItem(IDC_ALIGNMENT_LABEL)->MoveWindow(ransacX, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
	GetDlgItem(IDC_ALIGNMENT)->MoveWindow(ransacX + rectLabel.Width() + SMALL_SPACING, yPos, valueWidth, valueHeight);

	yPos -= rectLabel.Height() + 5 * SMALL_SPACING;
	GetDlgItem(IDC_EDGE_DETECTOR_LABEL)->MoveWindow(ransacX, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
	GetDlgItem(IDC_EDGE_DETECTOR)->MoveWindow(ransacX + rectLabel.Width() + SMALL_SPACING, yPos, valueWidth, valueHeight);

	yPos -= rectLabel.Height() + 10 * SMALL_SPACING;
	GetDlgItem(IDC_CANNY_ENABLE)->MoveWindow(ransacX, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
	GetDlgItem(IDC_CANNY_SETTINGS)->MoveWindow(ransacX + rectLabel.Width() + SMALL_SPACING, yPos, valueWidth, valueHeight);

	yPos -= rectLabel.Height() + 5 * SMALL_SPACING;
	GetDlgItem(IDC_GAUSSIAN_ENABLE)->MoveWindow(ransacX, yPos, rectLabel.Width(), rectLabel.Height(), TRUE);
	GetDlgItem(IDC_GAUSSIAN_SETTINGS)->MoveWindow(ransacX + rectLabel.Width() + SMALL_SPACING, yPos, valueWidth, valueHeight);


	CRect rectLog;
	rectLog.left = xPos + nBorder;
	rectLog.right = rectClient.right - nBorder;
	rectLog.bottom = rectClient.bottom - 4 * nBorder;
	rectLog.top = rectClient.bottom / 2;
	GetDlgItem(IDC_LOG)->MoveWindow(rectLog, TRUE);

	CRect rectGo;
	GetDlgItem(IDC_GO)->GetClientRect(&rectGo);
	GetDlgItem(IDC_GO)->MoveWindow(CRect(CPoint(rectClient.right - nBorder - rectGo.Width(), rectClient.bottom - nBorder - rectGo.Height()), rectGo.Size()));

	CRect rectClearLog;
	GetDlgItem(IDC_CLEAR_LOG)->GetClientRect(&rectClearLog);
	GetDlgItem(IDC_CLEAR_LOG)->MoveWindow(CRect(CPoint(rectLog.left, rectClient.bottom - nBorder - rectClearLog.Height()), rectClearLog.Size()));

	CRect rectSavePCD;
	GetDlgItem(IDC_SAVE_PCD)->GetClientRect(&rectSavePCD);
	GetDlgItem(IDC_SAVE_PCD)->MoveWindow(CRect(CPoint(rectClient.left+nBorder, rectClient.bottom - nBorder - rectSavePCD.Height()), rectSavePCD.Size()));
}

// When we have a new frame, convert it to bitmap for display

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

// Whenever nothing is going on, we grab frames and lightly process them

void CFormBasedView::DoIdleProcessing()
{
	static int nSkip = 30;

	if (m_fDepthScale != 0 && m_prsPipe->poll_for_frames(&m_rsFrames))
	{
		if (m_ctrlFreeze.GetCheck())
			return;

		auto depth = m_rsFrames.get_depth_frame();

// Filter the depth.  This filtering is only applied to the heat map for display.  Actual filtering of the point cloud is handled in main processing code.

		auto depthWidth = depth.get_width();
		auto depthHeight = depth.get_height();
		auto depthPoints = depthWidth * depthHeight;
		uint16_t* depthData = (uint16_t*)(depth.as<rs2::depth_frame>()).get_data();

		for (int x = 0; x < depthPoints; x++)
		{
			float dDepthInM = depthData[x] * m_fDepthScale;
			if (dDepthInM < m_fDepthMinValue || dDepthInM > m_fDepthMaxValue)
				depthData[x] = 0;
		}

// Compute the histograms

		FrameHistogram Hist;
		rs2::frameset data = m_rsFrames.apply_filter(m_rsPrinter).apply_filter(m_rsColorMap);
		NewFrame(data);
		ComputeHistogram(m_rsFrames, Hist);

// This macro sets the data into a histogram control, invalidates it (to cause it to redraw), computes the average and SD and sets the text to them

#define INIT_HIST(c, b)			\
	m_ctrl##c.SetData(Hist.c);			\
	m_ctrl##c.Invalidate(TRUE);		\
	m_ctrl##c##Average.SetWindowText(AverageAndSD(Hist.c, b))

// Do this once for each color channel

		INIT_HIST(H, 1);
		INIT_HIST(S, 0);
		INIT_HIST(V, 0);
		INIT_HIST(R, 0);
		INIT_HIST(G, 0);
		INIT_HIST(B, 0);

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;

		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points points;

		auto color = m_rsFrames.get_color_frame();

		pc.map_to(color);
	}
}

// Handle the sliders for the colors and depth

void CFormBasedView::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	UpdateData(TRUE);

	int value = ((CSliderCtrl*)pScrollBar)->GetPos();
	auto id = pScrollBar->GetDlgCtrlID();

	switch (id)
	{
	case IDC_DEPTH_MIN:
		m_fDepthMinValue = value/100.0;
		break;

	case IDC_DEPTH_MAX:
		m_fDepthMaxValue = value/100.0;
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

	GetDocument()->SetModifiedFlag(TRUE);

	//BuildBitmapFromFrame();
	//m_Image.Invalidate(TRUE);

	CFormView::OnHScroll(nSBCode, nPos, pScrollBar);
}





void CFormBasedView::OnDestroy()
{
	CFormView::OnDestroy();

	m_prsPipe->stop();
}


void CFormBasedView::OnSelChangeSacModel()
{
	UpdateData(TRUE);

	int nIdx = m_ctrlSACModel.GetCurSel();
	if (nIdx == -1)
		return;

	m_nSACModel = m_ctrlSACModel.GetItemData(nIdx);

	GetDocument()->SetModifiedFlag(TRUE);

	// Disable all controls

	for (int i = 0; i < m_RansacIds.GetSize(); i++)
	{
		GetDlgItem(m_RansacIds[i])->EnableWindow(FALSE);
	}

	GetDlgItem(IDC_DISTANCE_THRESHHOLD_LABEL)->EnableWindow(TRUE);
	GetDlgItem(IDC_DISTANCE_THRESHHOLD)->EnableWindow(TRUE);

	// Enable the ones we need

	for (auto& ransacModel : ransacModels)
	{
		if (m_nSACModel == ransacModel.model)
		{
			int enableID;

			for (auto& enableID : ransacModel.enables)
			{
				if (enableID == 0)
					break;
				GetDlgItem(enableID)->EnableWindow(TRUE);
			}
			break;
		}
	}
}



void CFormBasedView::OnBnClickedEnableVoxelFilter()
{
	UpdateData(TRUE);

	GetDlgItem(IDC_VOXEL_X)->EnableWindow(m_bEnableVoxelFilter);
	GetDlgItem(IDC_VOXEL_Y)->EnableWindow(m_bEnableVoxelFilter);
	GetDlgItem(IDC_VOXEL_Z)->EnableWindow(m_bEnableVoxelFilter);

	GetDocument()->SetModifiedFlag(TRUE);
}


void CFormBasedView::OnEnChange()
{
	GetDocument()->SetModifiedFlag(TRUE);
}


void CFormBasedView::OnBnClickedGo()
{
	UpdateData(TRUE);

	m_Layers.clear();

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	rs2::align align(AlignmentMode(m_strAlignment));

	rs2::frameset frameset = align.process(m_rsFrames);

	auto depth = frameset.get_depth_frame();
	auto color = frameset.get_color_frame();

	pc.map_to(color);

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	pcl_ptr pcl_points;

// If the edge detector is enabled

	if (m_dwEdgeDetector != 0)
	{
// Convert RS to PCL, maintaining color and depth for the edge finder

		auto color_points = points_to_pcl(points, color, noFilter, 0.0, 1000.0);

// Find the edges and create a new cloud of them

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edges(new pcl::PointCloud<pcl::PointXYZRGBA>);
		FindEdges(color_points, m_dwEdgeDetector, 0.02, 50, edges);

// Now apply the color filter to the edges

		colorFilter(edges, m_colorFilter);

// Copy the filtered edges into an XYZ (no color) cloud

		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl_points = cloud;
		copyPointCloud(*edges, *pcl_points);
	}
	else
	{
// With no edge filter, just convert RS to PCL while applying the color filter

		pcl_points = points_to_pcl(points, color, m_colorFilter);
	}

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	PrintToScreen(m_ctrlLog, _T("PointCloud before filtering %d data points\n"), pcl_points->width * pcl_points->height);

	// Get starting time

	LARGE_INTEGER liStart;
	::QueryPerformanceCounter(&liStart);

	//========================================
	// Filter PointCloud by Depth
	//========================================
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
	Cloud_Filter.setInputCloud(pcl_points);           // Input generated cloud to filter
	Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
	Cloud_Filter.setFilterLimits(m_fDepthMinValue, m_fDepthMaxValue);      // Set accepted interval values
	Cloud_Filter.filter(*newCloud);              // Filtered Cloud Outputted
	pcl::toPCLPointCloud2(*newCloud, *cloud_blob);

	LARGE_INTEGER liDepthAndColor;
	::QueryPerformanceCounter(&liDepthAndColor);

	PrintToScreen(m_ctrlLog, _T("%f sec.  PointCloud after depth and color filtering %d data points\n"),
		ConvertToSeconds(liDepthAndColor.QuadPart - liStart.QuadPart), newCloud->width * newCloud->height);

	LARGE_INTEGER liVolume;

// If the Voxel FIlter is enabled, downsample the point cloud

	if (m_bEnableVoxelFilter)
	{
		// Create the filtering object: downsample the dataset using a leaf size of VOXEL_DENSITY
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud(cloud_blob);
		sor.setLeafSize(m_fVoxelX, m_fVoxelY, m_fVoxelZ);
		sor.filter(*cloud_filtered_blob);

		// Convert to the templated PointCloud
		pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

		::QueryPerformanceCounter(&liVolume);

		PrintToScreen(m_ctrlLog, _T("%f sec.  PointCloud after volume filtering: %d data points\n"), 
			ConvertToSeconds(liVolume.QuadPart - liDepthAndColor.QuadPart), cloud_filtered->width * cloud_filtered->height);
	}
	else
	{
		pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);
		liVolume = liDepthAndColor;
	}

	m_Layers.push_back(new Feature(cloud_filtered));

// Now run RANSAC using the SAC model and parameters

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(m_nSACModel);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(m_nMaxIterations);
	seg.setDistanceThreshold(m_fDistanceThreshhold);

	if ((GetDlgItem(IDC_RADIUS_LIMITS_LABEL)->GetStyle() & WS_DISABLED) == 0)
		seg.setRadiusLimits(m_fRadiusLimitsMin, m_fRadiusLimitsMax);

	if ((GetDlgItem(IDC_AXIS_LABEL)->GetStyle() & WS_DISABLED) == 0)
	{
		Eigen::Vector3f axis(m_fAxisX, m_fAxisY, m_fAxisZ);
		seg.setAxis(axis);
	}

	if ((GetDlgItem(IDC_EPSILON_LABEL)->GetStyle() & WS_DISABLED) == 0)
	{
		seg.setEpsAngle(m_fEpsilon);
	}

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

// SAC Model Name for printing in log file

	CString strModelName;
	for (auto& ransacModel : ransacModels)
	{
		if (ransacModel.model == m_nSACModel)
		{
			strModelName = ransacModel.model_text;
			break;
		}
	}

	LARGE_INTEGER liStartExtract = liVolume;

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			PrintToScreen(m_ctrlLog, _T("Could not estimate a %s model for the given dataset.\n"), strModelName);
			break;
		}


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);

		LARGE_INTEGER liExtract;
		::QueryPerformanceCounter(&liExtract);

		PrintToScreen(m_ctrlLog, _T("%f sec.  PointCloud representing the %s component: %d\n"),
			ConvertToSeconds(liExtract.QuadPart - liStartExtract.QuadPart), strModelName, cloud_p->width * cloud_p->height);

		PrintToScreen(m_ctrlLog, _T("Model Coefficients:\nheader:\n"));
		PrintModelCoefficients(m_ctrlLog, *coefficients);
		feature_ptr p = new Feature(cloud_p);
		pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients(*coefficients));
		p->coefficients = c;
		m_Layers.push_back(p);

// Depending on the SAC Model, print the offset angle and distance
		float fDistance = 0;

		switch (m_nSACModel)
		{
		case pcl::SACMODEL_PLANE:
		case pcl::SACMODEL_PERPENDICULAR_PLANE:
		case pcl::SACMODEL_NORMAL_PLANE:
		case pcl::SACMODEL_PARALLEL_PLANE:
		case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
			PrintToScreen(m_ctrlLog, _T("Angle = %.2f rad (%.1f deg)   Distance = %.2f m (%.1f in)\n"), 
				Angle(*coefficients), Angle(*coefficients)*DEGREES_PER_RADIAN, coefficients->values[3], coefficients->values[3]*INCHES_PER_METER);
			break;

		case pcl::SACMODEL_LINE:
		case pcl::SACMODEL_CIRCLE3D:
			break;

// The circle only returns the x&y coordinates of the circle center, plus its radius.  Let's try computing the distance
// as the average Z value of all the points in the circle cloud.

		case pcl::SACMODEL_CIRCLE2D:
			for (auto& p : *cloud_p)
			{
				fDistance += p.z;
			}
			fDistance /= cloud_p->size();
			PrintToScreen(m_ctrlLog, _T("Average distance to circle: %.2f (%.1f in)\n"), fDistance, fDistance*INCHES_PER_METER);
			break;

		case pcl::SACMODEL_SPHERE:
			PrintToScreen(m_ctrlLog, _T("Angle = %.2f rad (%.1f deg)   Distance = %.2f m (%.1f in)\n"), 
				Angle(*coefficients), Angle(*coefficients)*DEGREES_PER_RADIAN, DistanceXZ(*coefficients), DistanceXZ(*coefficients)*INCHES_PER_METER);
			break;

		case pcl::SACMODEL_CYLINDER:
		//case pcl::SACMODEL_CONE:
		//case pcl::SACMODEL_TORUS:
		case pcl::SACMODEL_PARALLEL_LINE:
		//case pcl::SACMODEL_PARALLEL_LINES:
		case pcl::SACMODEL_NORMAL_SPHERE:
		case pcl::SACMODEL_REGISTRATION:
		case pcl::SACMODEL_REGISTRATION_2D:
		case pcl::SACMODEL_STICK:
			break;
		}

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		i++;
		if (m_Layers.size() >= (N_COLORS - 1))
			break;
	}

	LARGE_INTEGER liStop;
	::QueryPerformanceCounter(&liStop);
	PrintToScreen(m_ctrlLog, _T("%f sec.  Total Execution Time\n"),
		ConvertToSeconds(liStop.QuadPart - liStart.QuadPart));
	   
	draw_pointcloud(m_Layers);
}


void CFormBasedView::OnBnClickedSavePcd()
{
	CFileDialog dlg(FALSE, _T("pcd"), NULL, OFN_OVERWRITEPROMPT, _T("Point Cloud Files (*.pcd)|*.pcd|All Files (*.*)|*.*||"));
	dlg.AddCheckButton(IDC_APPLY_COLOR_FILTER, _T("Apply Color Filter"), FALSE);
	dlg.AddCheckButton(IDC_APPLY_DEPTH_FILTER, _T("Apply Depth Filter"), FALSE);
	dlg.AddCheckButton(IDC_APPLY_VOXEL_FILTER, _T("Apply Voxel Filter"), FALSE);

	if (dlg.DoModal() != IDOK)
		return;

	CT2A asciiPath(dlg.GetPathName());

// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	rs2::align align(AlignmentMode(m_strAlignment));

	rs2::frameset frameset = align.process(m_rsFrames);

	auto depth = frameset.get_depth_frame();
	auto color = frameset.get_color_frame();

	pc.map_to(color);

// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

// Get the checkbox values from the File Save dialog box

	BOOL bApplyColorFilter, bApplyDepthFilter, bApplyVoxelFilter;
	dlg.GetCheckButtonState(IDC_APPLY_COLOR_FILTER, bApplyColorFilter);
	dlg.GetCheckButtonState(IDC_APPLY_DEPTH_FILTER, bApplyDepthFilter);
	dlg.GetCheckButtonState(IDC_APPLY_VOXEL_FILTER, bApplyVoxelFilter);

// Convert RS to PCL while applying the color filter and depth filter.

	auto pcl_points = points_to_pcl(points, color, bApplyColorFilter?m_colorFilter:noFilter, bApplyDepthFilter? m_fDepthMinValue :0.0, bApplyDepthFilter? m_fDepthMaxValue :1000.0);

// Apply the Voxel Filter if needed

	if (bApplyVoxelFilter)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(pcl_points);
		sor.setLeafSize(m_fVoxelX, m_fVoxelY, m_fVoxelZ);
		sor.filter(*cloud_filtered);
		pcl_points = cloud_filtered;
	}
	pcl::io::savePCDFile<pcl::PointXYZRGBA>(asciiPath.m_psz, *pcl_points, true);
}


void CFormBasedView::OnBnClickedClearLog()
{
	m_ctrlLog.SetWindowText(_T(""));
}


void CFormBasedView::OnBnClickedGaussianSettings()
{
	CGaussianSettings dlg(m_sizeGaussian, m_dSigmaXGaussian, m_dSigmaYGaussian, m_nBorderTypeGaussian);

	if (dlg.DoModal() == IDOK)
	{
		m_sizeGaussian = CSize(dlg.m_nSizeX, dlg.m_nSizeY);
		m_dSigmaXGaussian = dlg.m_dSigmaX;
		m_dSigmaYGaussian = dlg.m_dSigmaY;
		m_nBorderTypeGaussian = dlg.m_nBorderType;

		GetDocument()->SetModifiedFlag(TRUE);
	}
}


void CFormBasedView::OnBnClickedCannySettings()
{
	CCannySettings dlg(m_dThreshhold1Canny, m_dThreshhold2Canny, m_nApertureCanny, m_bCannyEnable);

	if (dlg.DoModal() == IDOK)
	{
		m_dThreshhold1Canny = dlg.m_dThreshhold1;
		m_dThreshhold2Canny = dlg.m_dThreshhold2;
		m_nApertureCanny = dlg.m_nAperture;
		m_bL2GradientCanny = dlg.m_bL2Gradient;

		GetDocument()->SetModifiedFlag(TRUE);
	}

}


void CFormBasedView::OnBnClickedGaussianEnable()
{
	UpdateData(TRUE);

	GetDlgItem(IDC_GAUSSIAN_SETTINGS)->EnableWindow(m_bGaussianEnable);

	GetDocument()->SetModifiedFlag(TRUE);
}


void CFormBasedView::OnBnClickedCannyEnable()
{
	UpdateData(TRUE);

	GetDlgItem(IDC_CANNY_SETTINGS)->EnableWindow(m_bCannyEnable);

	GetDocument()->SetModifiedFlag(TRUE);
}
