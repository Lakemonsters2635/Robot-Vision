
// FormBasedDoc.cpp : implementation of the CFormBasedDoc class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "FormBased.h"
#endif

#include "FormBasedDoc.h"
#include "FormBasedView.h"

#include <propkey.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CFormBasedDoc

IMPLEMENT_DYNCREATE(CFormBasedDoc, CDocument)

BEGIN_MESSAGE_MAP(CFormBasedDoc, CDocument)
END_MESSAGE_MAP()


// CFormBasedDoc construction/destruction

CFormBasedDoc::CFormBasedDoc() noexcept
{
	// TODO: add one-time construction code here

}

CFormBasedDoc::~CFormBasedDoc()
{
}

BOOL CFormBasedDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CFormBasedDoc serialization

//#define	CURRENT_REV	1		// initial version
//#define	CURRENT_REV	2		// Added m_nMaxIterations
//#define	CURRENT_REV	3		// Changed DepthMinValue and DepthMaxValue to float
//#define	CURRENT_REV	4		// Added alignment mode
//#define	CURRENT_REV	5		// Added edge detector
#define		CURRENT_REV	6		// Added Gaussian Blur and Canny Edge Detection

void CFormBasedDoc::Serialize(CArchive& ar)
{
	POSITION pos = GetFirstViewPosition();
	CFormBasedView* pView = reinterpret_cast<CFormBasedView*> (GetNextView(pos));

	if (ar.IsStoring())
	{
		ar << CURRENT_REV;
		ar.Write(&pView->m_colorFilter, sizeof(pView->m_colorFilter));
		ar << pView->m_fDepthMinValue;
		ar << pView->m_fDepthMaxValue;
		ar << pView->m_bEnableVoxelFilter;
		ar << pView->m_fVoxelX;
		ar << pView->m_fVoxelY;
		ar << pView->m_fVoxelZ;
		ar << pView->m_nSACModel;
		ar << pView->m_nMaxIterations;
		ar << pView->m_fDistanceThreshhold;
		ar << pView->m_fRadiusLimitsMin;
		ar << pView->m_fRadiusLimitsMax;
		ar << pView->m_fAxisX;
		ar << pView->m_fAxisY;
		ar << pView->m_fAxisZ;
		ar << pView->m_fEpsilon;
		ar << pView->m_fConeAngleMin;
		ar << pView->m_fConeAngleMax;
		ar << AlignmentMode(pView->m_strAlignment);
		ar << pView->m_dwEdgeDetector;
		ar << pView->m_bGaussianEnable;
		ar << pView->m_sizeGaussian;
		ar << pView->m_dSigmaXGaussian;
		ar << pView->m_dSigmaYGaussian;
		ar << pView->m_nBorderTypeGaussian;
		ar << pView->m_bCannyEnable;
		ar << pView->m_dThreshhold1Canny;
		ar << pView->m_dThreshhold2Canny;
		ar << pView->m_nApertureCanny;
		ar << pView->m_bL2GradientCanny;
	}
	else
	{
		int nRev;

		ar >> nRev;

		if (nRev > CURRENT_REV)
		{
			::AfxMessageBox(_T("File contains rev %d data.  This version only supports up to %d\n"), nRev, CURRENT_REV);
			return;
		}

		ar.Read(&pView->m_colorFilter, sizeof(pView->m_colorFilter));
		if (nRev >= 3)
		{
			ar >> pView->m_fDepthMinValue;
			ar >> pView->m_fDepthMaxValue;
		}
		else
		{
			long nDepthMinValue, nDepthMaxValue;
			ar >> nDepthMinValue;
			ar >> nDepthMaxValue;
			pView->m_fDepthMinValue = nDepthMinValue / 100.0;
			pView->m_fDepthMaxValue = nDepthMaxValue / 100.0;
		}
		ar >> pView->m_bEnableVoxelFilter;
		ar >> pView->m_fVoxelX;
		ar >> pView->m_fVoxelY;
		ar >> pView->m_fVoxelZ;
		ar >> pView->m_nSACModel;
		if (nRev >= 2)
			ar >> pView->m_nMaxIterations;
		else
			pView->m_nMaxIterations = 1000;
		ar >> pView->m_fDistanceThreshhold;
		ar >> pView->m_fRadiusLimitsMin;
		ar >> pView->m_fRadiusLimitsMax;
		ar >> pView->m_fAxisX;
		ar >> pView->m_fAxisY;
		ar >> pView->m_fAxisZ;
		ar >> pView->m_fEpsilon;
		ar >> pView->m_fConeAngleMin;
		ar >> pView->m_fConeAngleMax;

		if (nRev >= 4)
		{
			int x;
			ar >> x;
			pView->m_strAlignment = AlignmentMode(x);
		}
		else
		{
			pView->m_strAlignment = _T("Depth");
		}

		if (nRev >= 5)
			ar >> pView->m_dwEdgeDetector;
		else
			pView->m_dwEdgeDetector = 0;

		if (nRev >= 6)
		{
			ar >> pView->m_bGaussianEnable;
			ar >> pView->m_sizeGaussian;
			ar >> pView->m_dSigmaXGaussian;
			ar >> pView->m_dSigmaYGaussian;
			ar >> pView->m_nBorderTypeGaussian;
			ar >> pView->m_bCannyEnable;
			ar >> pView->m_dThreshhold1Canny;
			ar >> pView->m_dThreshhold2Canny;
			ar >> pView->m_nApertureCanny;
			ar >> pView->m_bL2GradientCanny;

		}
		else
		{
			pView->m_bGaussianEnable = false;
			pView->m_sizeGaussian = CSize(9, 9);
			pView->m_dSigmaXGaussian = 2.0;
			pView->m_dSigmaYGaussian = 0.0;
			pView->m_nBorderTypeGaussian = 4;
			pView->m_bCannyEnable = false;
			pView->m_dThreshhold1Canny = 100;
			pView->m_dThreshhold2Canny = 100;
			pView->m_nApertureCanny = 3;
			pView->m_bL2GradientCanny = false;
		}
		pView->UpdateData(FALSE);
	}
}

#ifdef SHARED_HANDLERS

// Support for thumbnails
void CFormBasedDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// Modify this code to draw the document's data
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// Support for Search Handlers
void CFormBasedDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// Set search contents from document's data.
	// The content parts should be separated by ";"

	// For example:  strSearchContent = _T("point;rectangle;circle;ole object;");
	SetSearchContent(strSearchContent);
}

void CFormBasedDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = nullptr;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != nullptr)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CFormBasedDoc diagnostics

#ifdef _DEBUG
void CFormBasedDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CFormBasedDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CFormBasedDoc commands

