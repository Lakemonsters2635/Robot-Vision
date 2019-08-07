// CubeFinder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <algorithm> 

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#define ONE_SHOT

bool s_bVerbose = true;

#define		MODEL	pcl::SACMODEL_PARALLEL_PLANE
#define	MODEL_TEXT	"planar"
#define	VOXEL_DENSITY	0.005
#define	DISTANCE_THRESHHOLD		0.03
#define	EPSILON	0.0
#define DEPTH_FILTER_MIN	1.33
#define	DEPTH_FILTER_MAX	4.73
#define	SOR_MEAN_K			50
#define	SOR_STD_DEV_MULT	1.0

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
	ColorFilter(int LH, int RH, int LS, int RS, int LV, int RV, int LR, int RR, int LG, int RG, int LB, int RB)
		: leftH(LH)
		, rightH(RH)
		, leftS(LS)
		, rightS(RS)
		, leftV(LV)
		, rightV(RV)
		, leftR(LR)
		, rightR(RR)
		, leftG(LG)
		, rightG(RG)
		, leftB(LB)
		, rightB(RB)
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

ColorFilter colorFilter(21, 48, 63, 255, 103, 255, 0, 255, 0, 255, 0, 255);

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), panx(0.0), pany(0.0), last_x(0.0), last_y(0.0),
		ml(false), mm(false), /*offset_x(0.0f), offset_y(0.0f), */offset_z(0.0f), mask(-1), info(false) {}
	double yaw, pitch, panx, pany, last_x, last_y; bool ml; bool mm; float offset_z;
//	double yaw, pitch, panx, pany, last_x, last_y; bool ml; bool mm; float offset_x, offset_y;
	unsigned long long mask;
	bool info;
	int coef_sel;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_color_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;


struct Feature
{
	Feature(pcl_ptr p) { cloud = p; }
	pcl_ptr	cloud;
	pcl::ModelCoefficients::Ptr coefficients;
};

typedef Feature* feature_ptr;
std::vector <feature_ptr> layers;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<feature_ptr>& features);

void
RgbToHsv(unsigned short R, unsigned short G, unsigned short B, unsigned short& H, unsigned short& S, unsigned short& V)
{
	unsigned short rgbMin, rgbMax;

	rgbMin = R < G ? (R < B ? R : B) : (G < B ? G : B);
	rgbMax = R > G ? (R > B ? R : B) : (G > B ? G : B);

	V = rgbMax;
	if (V == 0)
	{
		H = 0;
		S = 0;
		return;
	}

	S = unsigned short(0.5 + 255.0 * long(rgbMax - rgbMin) / V);
	if (S == 0)
	{
		H = 0;
		return;
	}

	if (rgbMax == R)
		H = unsigned short(0.5 + 43.0 * (G - B) / (rgbMax - rgbMin));
	else if (rgbMax == G)
		H = unsigned short(85.5 + 43.0 * (B - R) / (rgbMax - rgbMin));
	else
		H = unsigned short(171.5 + 43.0 * (R - G) / (rgbMax - rgbMin));

}

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

	// Normals to Texture Coordinates conversion
	int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

inline bool Filter(unsigned char pixelColor, const int left, const int right)
{
	if (left < right)
	{
		return pixelColor < left || pixelColor > right;
	}
	else if (left > right)
	{
		return pixelColor < left && pixelColor > right;
	}
	else
	{
		return pixelColor != left;
	}
}

pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color, ColorFilter& colorFilter)
{
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();
	auto ptr = Vertex;

	for (int i = 0; i < points.size(); i++)
	{
		auto &pt = cloud->points[i];
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		pt.x = Vertex[i].x;
		pt.y = Vertex[i].y;
		pt.z = Vertex[i].z;

		// Obtain color texture for specific point
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		unsigned short R = std::get<0>(RGB_Color); // Reference tuple<2>
		unsigned short G = std::get<1>(RGB_Color); // Reference tuple<1>
		unsigned short B = std::get<2>(RGB_Color); // Reference tuple<0>

		unsigned short H, S, V;

		RgbToHsv(R, G, B, H, S, V);

		if (Filter(H, colorFilter.leftH, colorFilter.rightH)
			|| Filter(S, colorFilter.leftS, colorFilter.rightS)
			|| Filter(V, colorFilter.leftV, colorFilter.rightV)
			|| Filter(R, colorFilter.leftR, colorFilter.rightR)
			|| Filter(G, colorFilter.leftG, colorFilter.rightG)
			|| Filter(B, colorFilter.leftB, colorFilter.rightB))
		{
			pt.z = 0;
		}

	}	//for (auto& p : cloud->points)

	return cloud;
}

float3 colors[] = {
	{ 1.0f, 1.0f, 1.0f },
	{ 0.8f, 0.1f, 0.3f },
	{ 0.1f, 0.9f, 0.5f },
};

struct byte3
{
	unsigned char r, g, b;
};

byte3 colorsb[] = {
	{ 255, 255, 255 },
	{ 240, 41, 12 },
	{ 240, 139, 12 },
	{ 240, 233, 12 },
	{ 19, 240, 12 },
	{ 12, 240, 218 },
	{ 12, 157, 240 },
	{ 12, 70, 240 },
	{ 99, 12, 240 },
	{ 193, 12, 240 },
	{ 240, 12, 208 }
};

#define	N_COLORS	(sizeof(colorsb)/sizeof(colorsb[0]))

int main()
{
	window xyz(1280, 720, "Camera Data");

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	// Declare rates printer for showing streaming rates of the enabled streams.
	rs2::rates_printer printer;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();

	rs2::frameset frames;

	// Wait for the next set of frames from the camera
	while (xyz)
	{
		frames = pipe.wait_for_frames();
		rs2::frameset data = frames.apply_filter(printer).apply_filter(color_map);
		xyz.show(data);
	}
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);
#ifndef ONE_SHOT
	while (app) // Application still alive?
	{
#endif
		frames = pipe.wait_for_frames();

		auto depth = frames.get_depth_frame();
		auto color = frames.get_color_frame();

		pc.map_to(color);

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		auto pcl_points = points_to_pcl(points, color, colorFilter);

		pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
		Cloud_Filter.setInputCloud(pcl_points);           // Input generated cloud to filter
		Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
	//	Cloud_Filter.setFilterLimits(0.0, 1000.0);      // Set accepted interval values
		Cloud_Filter.setFilterLimits(DEPTH_FILTER_MIN, DEPTH_FILTER_MAX);      // Set accepted interval values
		Cloud_Filter.filter(*pcl_points);              // Filtered Cloud Outputted	pcl::PointCloud<pcl::PointXYZ>::Ptr mono_points(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

		if (s_bVerbose)
			std::cerr << "PointCloud before filtering: " << pcl_points->width * pcl_points->height << " data points." << std::endl;

		pcl_points->width *= pcl_points->height;
		pcl_points->height = 1;

		// Statistical Outlier Removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(pcl_points);
		sor.setMeanK(SOR_MEAN_K);
		sor.setStddevMulThresh(SOR_STD_DEV_MULT);
		sor.filter(*pcl_points);

		//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

			////========================================
			//// Filter PointCloud (PassThrough Method)
			////========================================
			//pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);

			//pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
			//Cloud_Filter.setInputCloud(pcl_points);           // Input generated cloud to filter
			//Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
			//Cloud_Filter.setFilterLimits(DEPTH_FILTER_MIN, DEPTH_FILTER_MAX);      // Set accepted interval values
			//Cloud_Filter.filter(*newCloud);              // Filtered Cloud Outputted


		pcl::toPCLPointCloud2(*pcl_points, *cloud_blob);

		if (s_bVerbose)
			std::cerr << "PointCloud after depth filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

		// Create the filtering object: downsample the dataset using a leaf size of VOXEL_DENSITY
		pcl::VoxelGrid<pcl::PCLPointCloud2> vog;
		vog.setInputCloud(cloud_blob);
		vog.setLeafSize(VOXEL_DENSITY, VOXEL_DENSITY, VOXEL_DENSITY);
		vog.filter(*cloud_filtered_blob);

		// Convert to the templated PointCloud
		pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

		if (s_bVerbose)
			std::cerr << "PointCloud after volume filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

#ifndef ONE_SHOT
		layers.clear();
#endif

		layers.push_back(new Feature(cloud_filtered));

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(MODEL);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(DISTANCE_THRESHHOLD);
		Eigen::Vector3f axis(0.0, 1.0, 0.0);
		seg.setAxis(axis);
		seg.setEpsAngle(0.0);


		std::vector<Eigen::Vector4f> Models;

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		int i = 0, nr_points = (int)cloud_filtered->points.size();
		// While 30% of the original cloud is still there
		while (cloud_filtered->points.size() > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud_filtered);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				std::cerr << "Could not estimate a " MODEL_TEXT " model for the given dataset." << std::endl;
				break;
			}


			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

			// Extract the inliers
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_p);

			//std::stringstream ss;
			//ss << "table_scene_lms400_plane_" << i << ".pcd";
			//writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

			{
				Eigen::Vector4f model(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
				Models.insert(Models.end(), model);

				if (s_bVerbose)
				{
					std::cerr << "PointCloud representing the " << MODEL_TEXT << " component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
					std::cerr << "Model Coefficients: " << *coefficients << std::endl;
				}
				feature_ptr p = new Feature(cloud_p);
				pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients(*coefficients));
				p->coefficients = c;
				layers.push_back(p);
			}


			//ShowCloud(cloud_p);

			// Create the filtering object
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			i++;
			if (layers.size() >= (N_COLORS - 2))
				break;
		}

		for (int i = 0; i < Models.size(); i++)
		{
			Eigen::Vector4f& model1 = Models[i];
			Eigen::Vector3f N1(model1[0], model1[1], model1[2]);

			for (auto j = i + 1; j < Models.size(); j++)
			{
				Eigen::Vector4f& model2 = Models[j];
				Eigen::Vector3f N2(model2[0], model2[1], model2[2]);
				Eigen::Vector3f U = N1.cross(N2);
				double dMag = sqrt(U[0] * U[0] + U[1] * U[1] + U[2] * U[2]);
				if (dMag >= 0.01)
				{
					U = U / sqrt(U[0] * U[0] + U[1] * U[1] + U[2] * U[2]);
					double Theta = acos(N1.dot(N2)) * 180 / 3.14159265358979323843383;
					Eigen::Vector3f P;
					P[0] = model1[2] * model2[3] - model2[2] * model1[3];
					P[1] = 0.0;
					P[2] = model1[3] * model2[0] - model2[3] * model1[0];
					double d = model2[0] * model1[2] - model1[0] * model2[2];
					P /= d;

					//P[0] = model1[2] * model2[3] - model2[2] * model1[3];
					//P[1] = 0.0;
					//P[2] = model1[3] * model2[0] - model2[3] * model1[0];
					if (s_bVerbose)
					{
						std::cerr << i + 1 << " x " << j + 1 << " = (" << U[0] << ", " << U[1] << ", " << U[2] << ")  " << Theta <<
							"  (" << P[0] << ", " << P[1] << ", " << P[2] << ")" << std::endl;
						std::cerr << "P(s) = (" << P[0] << ", " << P[1] << ", " << P[2] << ") + s * (" << U[0] << ", " << U[1] << ", " << U[2] << ")" << std::endl;
					}

					// Draw the intersection of 1x2 only

					if (i == 0 && j == 1)
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
						feature_ptr p = new Feature(cloud_line);
						pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients());
						p->coefficients = c;
						layers.push_back(p);

						for (double s = -0.5; s < 0.5; s += 0.01)
						{
							Eigen::Vector3f L;
							L = P + s * U;
							pcl::PointXYZ pt;
							pt.x = -L[0];
							pt.y = L[1];
							pt.z = -L[2];

							cloud_line->insert(cloud_line->end(), 1, pt);
						}
					}
				}
			}
		}

#ifdef ONE_SHOT
		while (app) // Application still alive?
		{
#endif
			draw_pointcloud(app, app_state, layers);
		}

		return EXIT_SUCCESS;
}
	//catch (const rs2::error & e)
	//{
	//	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	//	return EXIT_FAILURE;
	//}
	//catch (const std::exception & e)
	//{
	//	std::cerr << e.what() << std::endl;
	//	return EXIT_FAILURE;
	//}

	double
		Mod180(double d)
	{
		return d > 180.0 ? d - 360 : (d < -180.0 ? d + 360 : d);
	}

	// Registers the state variable and callbacks to allow mouse control of the pointcloud
	void register_glfw_callbacks(window& app, state& app_state)
	{
		app.on_left_mouse = [&](bool pressed)
		{
			app_state.ml = pressed;
		};

		app.on_middle_mouse = [&](bool pressed)
		{
			app_state.mm = pressed;
		};

		app.on_mouse_scroll = [&](double xoffset, double yoffset)
		{
			app_state.offset_z += static_cast<float>(yoffset);
			//app_state.offset_x += static_cast<float>(xoffset);
			//app_state.offset_y += static_cast<float>(yoffset);
		};

		app.on_mouse_move = [&](double x, double y)
		{
			if (app_state.ml)
			{
				app_state.yaw -= (x - app_state.last_x);
				app_state.yaw = Mod180(app_state.yaw);
				app_state.pitch += (y - app_state.last_y);
				app_state.pitch = Mod180(app_state.pitch);
			}
			else if (app_state.mm)
			{
				app_state.panx += (x - app_state.last_x);
				app_state.pany += (y - app_state.last_y);
			}
			app_state.last_x = x;
			app_state.last_y = y;
		};

		app.on_key_release = [&](int key)
		{
			int which = -1;
			if (key > 255 || key < 0)
				return;

			switch (key)
			{
			case ' ':
//				app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
				app_state.yaw = app_state.pitch = 0; app_state.offset_z = 0; app_state.panx = app_state.pany = 0;
				break;

			case '/':
				app_state.info = true;
				break;

			default:
				if (isdigit(key))
				{
					which = (key - '0');
				}
				else if (isupper(key))
				{
					if (key == 'Z')
					{
						if (app_state.mask != 0)
							app_state.mask = 0;
						else
							app_state.mask = -1;
					}
					else
					{
						which = (10 + key - 'A');
					}
				}
			}
			if (which != -1)
			{
				if (app_state.info)
				{
					app_state.coef_sel = which;
					app_state.info = false;
				}
				else
				{
					unsigned long long bit = 1LL << which;
					app_state.mask ^= bit;
				}
			}
		};
	}

	// Handles all the OpenGL calls needed to display the point cloud
	void draw_pointcloud(window& app, state& app_state, const std::vector<feature_ptr>& features)
	{
		// OpenGL commands that prep screen for the pointcloud
		glPopMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		float width = app.width(), height = app.height();

		//	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
		glClearColor(0.0, 0.0, 0.0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(60, width / height, 0.01f, 10.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

		glTranslatef(-0.5f, -0.5f, +0.5f + app_state.offset_z*0.05f);
//		glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
		glRotated(app_state.pitch, 1, 0, 0);
		glRotated(app_state.yaw, 0, 1, 0);
		glTranslatef(+0.5f + app_state.panx*0.05f, +0.5f + app_state.pany*0.05f, -0.5f);
//		glTranslatef(0, 0, -0.5f);

		glPointSize(width / 640);
		glEnable(GL_TEXTURE_2D);

		int color = 0;

		for (auto&& f : features)
		{
			if (app_state.coef_sel != -1 && color == app_state.coef_sel)
			{
				if (f->coefficients)
				{
					std::cerr << "Model Coefficients: " << *f->coefficients << std::endl;
					double Nx = f->coefficients->values[0];
					double Nz = f->coefficients->values[2];
					float theta = acos(Nz / sqrt(Nx*Nx + Nz * Nz));
					std::cerr << "Error Angle: " << theta * 180 / 3.1415926 << " degrees" << std::endl;
				}

				app_state.coef_sel = -1;
			}

			auto pc = f->cloud;
			if (app_state.mask & (1LL << color))
			{
				//		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
				auto c = colorsb[(color) % (sizeof(colors) / sizeof(byte3))];

				glBegin(GL_POINTS);
				glColor3f(c.r / 255.0, c.g / 255.0, c.b / 255.0);

				/* this segment actually prints the pointcloud */
				for (int i = 0; i < pc->points.size(); i++)
				{
					auto&& p = pc->points[i];
					if (p.z)
					{
						// upload the point and texture coordinates only for points we have depth data for
						glVertex3f(p.x, p.y, p.z);
					}
				}

				glEnd();
			}
			color++;
		}

		// OpenGL cleanup
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glPushMatrix();
	}

