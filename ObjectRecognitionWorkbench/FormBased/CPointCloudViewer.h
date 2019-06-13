#pragma once
#include "afxwin.h"

#include <gl/gl.h>
#include <gl/glu.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr; 

struct Feature
{
	Feature(pcl_ptr p) { cloud = p; }
	pcl_ptr	cloud;
	pcl::ModelCoefficients::Ptr coefficients;
};

typedef Feature* feature_ptr;



class CPointCloudViewer : public CWnd
{
public:
	/******************/
	/* Public Members */
	/******************/
	UINT_PTR m_unpTimer;
	// View information variables
	float	 m_fLastX;
	float	 m_fLastY;
	float	 m_fPosX;
	float	 m_fPosY;
	float	 m_fZoom;
	float	 m_fRotX;
	float	 m_fRotY;
	bool	 m_bIsMaximized;

private:
	/*******************/
	/* Private Members */
	/*******************/
	// Window information
	CWnd  *hWnd;
	HDC   hdc;
	HGLRC hrc;
	int   m_nPixelFormat;
	CRect m_rect;
	CRect m_oldWindow;
	CRect m_originalRect;

	bool m_bInitted;

	int		m_nCoefSel;
	unsigned long long m_bbMask;

	const std::vector<feature_ptr>* m_pFeatures;

public:
	CPointCloudViewer(void);
	virtual ~CPointCloudViewer(void);

	void oglCreate(CRect rect, CWnd *parent);
	void oglInitialize(void);
	void oglDrawScene(void);

	// Added message classes:
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg	void OnDraw(CDC *pDC);
	afx_msg int  OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);

	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	void SetFeatures(const std::vector<feature_ptr>* features) { m_pFeatures = features; }

private:
	void CPointCloudViewer::draw_pointcloud(const std::vector<feature_ptr>& features);
};