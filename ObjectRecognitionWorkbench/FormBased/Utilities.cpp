#include "stdafx.h"

#include "Utilities.h"

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


void ComputeHistogram(rs2::frameset frames, FrameHistogram& Hist)
{
	auto color = frames.get_color_frame();
	auto width = color.get_width();
	auto height = color.get_height();
	auto BPP = color.get_bytes_per_pixel();
	auto bPP = color.get_bits_per_pixel();
	auto stride = color.get_stride_in_bytes();
	uint8_t* pixels = (uint8_t*)color.get_data();

	//Hist.m_pHPixels = new BYTE[width * height];			// Cache the H values so we don't need to recalculate
	ZeroMemory(&Hist, sizeof(Hist));

	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			int bytes = x * BPP;
			int strides = y * stride;
			int Index = (bytes + strides);

			unsigned short R = pixels[Index];
			unsigned short G = pixels[Index + 1];
			unsigned short B = pixels[Index + 2];

			Hist.R[R]++;
			Hist.G[G]++;
			Hist.B[B]++;

			unsigned short H;
			unsigned short S;
			unsigned short V;

			RgbToHsv(R, G, B, H, S, V);
			Hist.H[H & 0xff]++;
			Hist.S[S & 0xff]++;
			Hist.V[V & 0xff]++;

			//Hist.m_pHPixels[x + y * width] = H & 0xff;
		}
	}

	//Hist.m_pFrame = &color;

}

CString
AverageAndSD(long values[256], bool bIsH)		// bIsH means we are computing the H channel which can wrap.
{
	int iMin = bIsH ? 128 : 0;
	int iMax = bIsH ? 384 : 256;

	long nPoints = 0;
	double dSum = 0;
	double dAverage, dSDSum = 0;

	for (int i = iMin; i < iMax; i++)
	{
		nPoints += values[i % 256];
		dSum += i * values[i % 256];
	}

	dAverage = dSum / nPoints;

	for (int i = iMin; i < iMax; i++)
	{
		double dDiff = (i - dAverage);
		dSDSum += values[i % 256] * dDiff * dDiff;
	}

	CString strRes;
	strRes.Format(_T("%.0f ± %.0f"), dAverage - 2 * iMin, sqrt(dSDSum / nPoints));

	if (bIsH)
		strRes = strRes + _T("  ") + AverageAndSD(values, 0);

	return strRes;
}

void FilterFrameByDepth(rs2::frameset frames, int nDepthMin, int nDepthMax)
{
	auto depthFrame = frames.get_depth_frame();

	auto width = depthFrame.get_width();
	auto height = depthFrame.get_height();

	auto data = depthFrame.get_data();



}

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

//// Struct for managing rotation of pointcloud view
//struct state {
//	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
//		ml(false), offset_x(0.0f), offset_y(0.0f), mask(-1), info(false) {}
//	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
//	unsigned long long mask;
//	bool info;
//	int coef_sel;
//};
//
//double
//Mod180(double d)
//{
//	return d > 180.0 ? d - 360 : (d < -180.0 ? d + 360 : d);
//}
//
//// Registers the state variable and callbacks to allow mouse control of the pointcloud
//static void register_glfw_callbacks(window& app, state& app_state)
//{
//	app.on_left_mouse = [&](bool pressed)
//	{
//		app_state.ml = pressed;
//	};
//
//	app.on_mouse_scroll = [&](double xoffset, double yoffset)
//	{
//		app_state.offset_x += static_cast<float>(xoffset);
//		app_state.offset_y += static_cast<float>(yoffset);
//	};
//
//	app.on_mouse_move = [&](double x, double y)
//	{
//		if (app_state.ml)
//		{
//			app_state.yaw -= (x - app_state.last_x);
//			app_state.yaw = Mod180(app_state.yaw);
//			app_state.pitch += (y - app_state.last_y);
//			app_state.pitch = Mod180(app_state.pitch);
//		}
//		app_state.last_x = x;
//		app_state.last_y = y;
//	};
//
//	app.on_key_release = [&](int key)
//	{
//		int which = -1;
//		if (key > 255 || key < 0)
//			return;
//
//		switch (key)
//		{
//		case ' ':
//			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
//			break;
//
//		case '/':
//			app_state.info = true;
//			break;
//
//		default:
//			if (isdigit(key))
//			{
//				which = (key - '0');
//			}
//			else if (isupper(key))
//			{
//				if (key == 'Z')
//				{
//					if (app_state.mask != 0)
//						app_state.mask = 0;
//					else
//						app_state.mask = -1;
//				}
//				else
//				{
//					which = (10 + key - 'A');
//				}
//			}
//		}
//		if (which != -1)
//		{
//			if (app_state.info)
//			{
//				app_state.coef_sel = which;
//				app_state.info = false;
//			}
//			else
//			{
//				unsigned long long bit = 1LL << which;
//				app_state.mask ^= bit;
//			}
//		}
//	};
//}
//
//
//// Handles all the OpenGL calls needed to display the point cloud
//void draw_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	window app(1280, 720, "Point Cloud");
//
//	state app_state;
//	// register callbacks to allow manipulation of the pointcloud
//	register_glfw_callbacks(app, app_state);
//
//	// OpenGL commands that prep screen for the pointcloud
//	glPopMatrix();
//	glPushAttrib(GL_ALL_ATTRIB_BITS);
//
//	float width = app.width(), height = app.height();
//
//	//	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
//	glClearColor(0.0, 0.0, 0.0, 1);
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	gluPerspective(60, width / height, 0.01f, 10.0f);
//
//	glMatrixMode(GL_MODELVIEW);
//	glPushMatrix();
//	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
//
//	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
//	glRotated(app_state.pitch, 1, 0, 0);
//	glRotated(app_state.yaw, 0, 1, 0);
//	glTranslatef(0, 0, -0.5f);
//
//	glPointSize(width / 640);
//	glEnable(GL_TEXTURE_2D);
//
//
//
//		glBegin(GL_POINTS);
//		glColor3f(1.0f, 1.0f, 1.0f);
//
//		/* this segment actually prints the pointcloud */
//		for (int i = 0; i < cloud->points.size(); i++)
//		{
//			auto&& p = cloud->points[i];
//			if (p.z)
//			{
//				// upload the point and texture coordinates only for points we have depth data for
//				glVertex3f(p.x, p.y, p.z);
//			}
//		}
//
//		glEnd();
//
//	// OpenGL cleanup
//	glPopMatrix();
//	glMatrixMode(GL_PROJECTION);
//	glPopMatrix();
//	glPopAttrib();
//	glPushMatrix();
//}
