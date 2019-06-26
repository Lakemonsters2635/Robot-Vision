#pragma once

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API

// CChooseCamera dialog

class CChooseCamera : public CDialogEx
{
	DECLARE_DYNAMIC(CChooseCamera)

public:
	CChooseCamera(std::vector<rs2::device>& depthCameras, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CChooseCamera();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CHOOSE_CAMERA };
#endif

protected:
	CComboBox m_ctrlCameras;
	std::vector<rs2::device>& m_depthCameras;

	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	int m_nSelectedCamera;

	virtual BOOL OnInitDialog();
	afx_msg void OnCbnCloseupCameras();
};
