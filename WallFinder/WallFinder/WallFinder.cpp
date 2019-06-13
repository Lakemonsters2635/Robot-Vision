// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include "pch.h"
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

// Define exactly one of the following and no more.
#define		PLANE
#undef		SPHERE


#ifdef PLANE
#define		MODEL	pcl::SACMODEL_PLANE
#define	MODEL_TEXT	"planar"
#undef	APPLY_COLOR_FILTER
#define	VOXEL_DENSITY	0.01
#define	WALL_TOLERANCE	0.09
#define	DISTANCE_THRESHHOLD		0.01
#endif
#ifdef SPHERE
#define		MODEL	pcl::SACMODEL_SPHERE
#define	MODEL_TEXT	"spherical"
#define	APPLY_COLOR_FILTER
#define	DEPTH_FILTER_MIN	0.1
#define	DEPTH_FILTER_MAX	2.0
#define	VOXEL_DENSITY	0.0025
#define	MIN_RADIUS		0.1525
#define	MAX_RADIUS		0.1775
#define	DISTANCE_THRESHHOLD		0.005
#endif

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
		ml(false), offset_x(0.0f), offset_y(0.0f), mask(-1), info(false) {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
	unsigned long long mask;
	bool info;
	int coef_sel;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

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

#ifdef APPLY_COLOR_FILTER
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
#endif

pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
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
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

#ifdef APPLY_COLOR_FILTER
		// Obtain color texture for specific point
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		unsigned short R = std::get<0>(RGB_Color); // Reference tuple<2>
		unsigned short G = std::get<1>(RGB_Color); // Reference tuple<1>
		unsigned short B = std::get<2>(RGB_Color); // Reference tuple<0>

		unsigned short H, S, V;

		RgbToHsv(R, G, B, H, S, V);

		if (H > 20)
		{
			cloud->points[i].z = 0;
		}
#endif
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

int main(int argc, char * argv[]) try
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

	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();

	pc.map_to(color);

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	auto pcl_points = points_to_pcl(points, color);

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "PointCloud before filtering: " << pcl_points->width * pcl_points->height << " data points." << std::endl;

#ifdef DEPTH_FILTER_MIN
	//========================================
	// Filter PointCloud (PassThrough Method)
	//========================================
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
	Cloud_Filter.setInputCloud(pcl_points);           // Input generated cloud to filter
	Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
	Cloud_Filter.setFilterLimits(DEPTH_FILTER_MIN, DEPTH_FILTER_MAX);      // Set accepted interval values
	Cloud_Filter.filter(*newCloud);              // Filtered Cloud Outputted
	pcl::toPCLPointCloud2(*newCloud, *cloud_blob);
	std::cerr << "PointCloud after depth filtering: " << newCloud->width * newCloud->height << " data points." << std::endl;
#else
	pcl::toPCLPointCloud2(*pcl_points, *cloud_blob);
#endif // DEPTH_FILTER_MIN

	// Create the filtering object: downsample the dataset using a leaf size of VOXEL_DENSITY
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(VOXEL_DENSITY, VOXEL_DENSITY, VOXEL_DENSITY);
	sor.filter(*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after volume filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;


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
#if MODEL==SACMODEL_PLANE
	seg.setDistanceThreshold(0.01);
#else
	seg.setDistanceThreshold(0.005);
//	seg.setRadiusLimits(0.1, 1.0);
#endif // MODEL==SACMODEL_PLANE


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

#ifdef WALL_TOLERANCE
		if (coefficients->values[1] <= WALL_TOLERANCE)		// look for vertical walls
#endif // WALL_TOLERANCE
#ifdef MIN_RADIUS
		if (coefficients->values[3] < MAX_RADIUS && coefficients->values[3] > MIN_RADIUS)
#endif // MIN_RADIUS
		{
			std::cerr << "PointCloud representing the " << MODEL_TEXT << " component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
			std::cerr << "Model Coefficients: " << *coefficients << std::endl;
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
		if (layers.size() >= (N_COLORS-1))
			break;
	}

	while (app) // Application still alive?
	{
		draw_pointcloud(app, app_state, layers);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

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

	app.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		app_state.offset_x += static_cast<float>(xoffset);
		app_state.offset_y += static_cast<float>(yoffset);
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
			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
			break;

		case '/' :
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

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

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
