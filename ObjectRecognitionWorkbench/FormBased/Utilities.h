#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#undef	min
#undef	max
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <stdarg.h>

//#include "example.hpp"

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

//Return true if outside range

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

#define	N_COLORS	11

struct byte3
{
	unsigned char r, g, b;
};

extern byte3 colorsb[N_COLORS];

struct FrameHistogram
{
	long	R[256];
	long	G[256];
	long	B[256];
	long	H[256];
	long	S[256];
	long	V[256];
};



using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

struct Feature
{
	Feature(pcl_ptr p) { cloud = p; }
	pcl_ptr	cloud;
	pcl::ModelCoefficients::Ptr coefficients;
};

typedef Feature* feature_ptr;



void RgbToHsv(unsigned short R, unsigned short G, unsigned short B, unsigned short& H, unsigned short& S, unsigned short& V);
void ComputeHistogram(rs2::frameset frames, FrameHistogram& Hist);
CString AverageAndSD(long values[256], bool bIsH);
void FilterFrameByDepth(rs2::frameset frames, int nDepthMin, int nDepthMax);



using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color, ColorFilter& colorFilter);
void draw_pointcloud(const std::vector <feature_ptr>& layers);

void PrintToScreen(CEdit& Edit, LPCTSTR pszFormat, ...);
void PrintModelCoefficients(CEdit& Edit, pcl::ModelCoefficients& v);
