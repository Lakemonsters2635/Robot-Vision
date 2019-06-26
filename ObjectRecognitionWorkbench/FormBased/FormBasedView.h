
// FormBasedView.h : interface of the CFormBasedView class
//

#pragma once
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "CRealSenseVideo.h"
#include "CHistogram.h"
#include "CheckComboBox.h"

#include "utilities.h"

#define	CAMERA_ASPECT_WIDTH		16.0
#define	CAMERA_ASPECT_HEIGHT	9.0

#define	BORDER					16
#define	SMALL_SPACING			4

#define	MIN_CLIENET_HEIGHT		920
#define	MIN_CLIENT_WIDTH		1900

class window;
struct glfw_state;

class CFormBasedView : public CFormView
{
protected: // create from serialization only
	CFormBasedView() noexcept;
	DECLARE_DYNCREATE(CFormBasedView)

public:
#ifdef AFX_DESIGN_TIME
	enum{ IDD = IDD_FORMBASED_FORM };
#endif

// Attributes
public:
	CFormBasedDoc* GetDocument() const;

// Operations
public:

	void DoIdleProcessing();

// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnInitialUpdate(); // called first time after construct
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnPrint(CDC* pDC, CPrintInfo* pInfo);

	void NewFrame(rs2::frameset& frames);

// Implementation
public:
	virtual ~CFormBasedView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer m_rsColorMap;
	// Declare rates printer for showing streaming rates of the enabled streams.
	rs2::rates_printer m_rsPrinter;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline m_rsPipe;
	float m_fDepthScale;

	// Frameset returned from pipeline
	rs2::frameset m_rsFrames;

	CRealSenseVideo m_oglDepth;
	CRealSenseVideo m_oglCamera;

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()

public:
	CHistogram m_ctrlH;
	CHistogram m_ctrlS;
	CHistogram m_ctrlV;
	CHistogram m_ctrlR;
	CHistogram m_ctrlG;
	CHistogram m_ctrlB;
	CStatic m_ctrlHLabel;
	CStatic m_ctrlSLabel;
	CStatic m_ctrlVLabel;
	CStatic m_ctrlRLabel;
	CStatic m_ctrlGLabel;
	CStatic m_ctrlBLabel;
	CStatic m_ctrlHAverage;
	CStatic m_ctrlSAverage;
	CStatic m_ctrlVAverage;
	CStatic m_ctrlRAverage;
	CStatic m_ctrlGAverage;
	CStatic m_ctrlBAverage;
	CSliderCtrl m_ctrlHLeft;
	CSliderCtrl m_ctrlSLeft;
	CSliderCtrl m_ctrlVLeft;
	CSliderCtrl m_ctrlRLeft;
	CSliderCtrl m_ctrlGLeft;
	CSliderCtrl m_ctrlBLeft;
	CSliderCtrl m_ctrlHRight;
	CSliderCtrl m_ctrlSRight;
	CSliderCtrl m_ctrlVRight;
	CSliderCtrl m_ctrlRRight;
	CSliderCtrl m_ctrlGRight;
	CSliderCtrl m_ctrlBRight;
	CStatic m_ctrlHLeftValue;
	CStatic m_ctrlSLeftValue;
	CStatic m_ctrlVLeftValue;
	CStatic m_ctrlRLeftValue;
	CStatic m_ctrlGLeftValue;
	CStatic m_ctrlBLeftValue;
	CStatic m_ctrlHRightValue;
	CStatic m_ctrlSRightValue;
	CStatic m_ctrlVRightValue;
	CStatic m_ctrlRRightValue;
	CStatic m_ctrlGRightValue;
	CStatic m_ctrlBRightValue;
	CButton m_ctrlFreeze;
	CSliderCtrl m_ctrlDepthMin;
	CSliderCtrl m_ctrlDepthMax;
	CStatic m_ctrlDepthMinValue;
	CStatic m_ctrlDepthMaxValue;
	CEdit	m_ctrlLog;

protected:

//	CPointCloudViewer m_PointCloudViewer;
	CRect m_rectPointCloudViewer;
	std::vector <feature_ptr> m_Layers;

	window*		m_pPointCloudWindow;
	glfw_state*	m_pAppState;

	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnDestroy();
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnSelChangeSacModel();
	afx_msg void OnBnClickedEnableVoxelFilter();
	afx_msg void OnEnChange();
	afx_msg void OnBnClickedGo();

	CComboBox m_ctrlSACModel;
	CDWordArray m_RansacIds;

// Parameters that get saved/restored
// Doc Serializer needs access

	friend void CFormBasedDoc::Serialize(CArchive& ar);

	ColorFilter	m_colorFilter;
	float m_fDepthMinValue;
	float m_fDepthMaxValue;
	int m_bEnableVoxelFilter;
	float m_fVoxelX;
	float m_fVoxelY;
	float m_fVoxelZ;
	int m_nSACModel;
	int m_nMaxIterations;
	float m_fDistanceThreshhold;
	float m_fRadiusLimitsMin;
	float m_fRadiusLimitsMax;
	float m_fAxisX;
	float m_fAxisY;
	float m_fAxisZ;
	float m_fEpsilon;
	float m_fConeAngleMin;
	float m_fConeAngleMax;
	CString m_strAlignment;
	DWORD m_dwEdgeDetector;


	BOOL m_bFreeze;
public:
	afx_msg void OnBnClickedSavePcd();
	CCheckComboBox m_ctrlEdgeDetector;
};

#ifndef _DEBUG  // debug version in FormBasedView.cpp
inline CFormBasedDoc* CFormBasedView::GetDocument() const
   { return reinterpret_cast<CFormBasedDoc*>(m_pDocument); }
#endif

