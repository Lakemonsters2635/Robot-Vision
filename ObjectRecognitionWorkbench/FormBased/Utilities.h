#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

struct FrameHistogram
{
	long	R[256];
	long	G[256];
	long	B[256];
	long	H[256];
	long	S[256];
	long	V[256];
};

void RgbToHsv(unsigned short R, unsigned short G, unsigned short B, unsigned short& H, unsigned short& S, unsigned short& V);
void ComputeHistogram(rs2::frameset frames, FrameHistogram& Hist);
CString AverageAndSD(long values[256], bool bIsH);
void FilterFrameByDepth(rs2::frameset frames, int nDepthMin, int nDepthMax);

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);