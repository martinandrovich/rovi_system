#include "rovi_vision/RGBTemplateMatching.h"

#include <algorithm>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/linemod.hpp>
#include <opencv2/core/eigen.hpp>

inline bool is_border(cv::Mat& edge, cv::Vec3b color);
inline cv::Rect autocrop(cv::Mat & src);
inline cv::linemod::Detector create_linemod_detector();

namespace Eigen
{
	static void read(const cv::FileNode& node, Matrix4d& value, const Matrix4d& default_value = Matrix4d()) 
	{ 
		cv::cv2eigen(node.mat(), value);
	};
};

namespace rovi_vision::RGBTemplateMatching
{

static std::map<std::string, int> object_dict;
static std::vector<std::string> template_dirs;
static std::vector<cv::linemod::Detector> linemods;
static std::vector<std::vector<cv::Mat>> templates;
static std::vector<std::vector<cv::Mat>> cropped_templates;
static double tsh = 50.;

void cluster_matches(const std::vector<cv::linemod::Match> & matches, const int & key);

Eigen::Isometry3d
est_pose(const std::string& name, const sensor_msgs::Image& img, const bool vis)
{
	// utilized to use the correct linemod interface
	auto key = object_dict[name];
	// ROS_INFO_STREAM(key);

	// conversion
	auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	ROS_ASSERT_MSG(!cv_ptr->image.empty(), "image data is empty.");
	cv::Mat img_processed = cv_ptr->image.clone();
	
	// insert the images that should be matched
	std::vector<cv::Mat> sources{img_processed};
	
	// matches
	std::vector<cv::linemod::Match> matches;
	linemods[key].match(sources, tsh, matches);
	
	// print the number of responses
	ROS_INFO("number of matches: %04i", (int)matches.size());

	// visualize the result if turned on
	if (vis)
	{
		for(auto i = 0; i < matches.size(); i++)
		{
			// very ineffiecient method
			cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
			cv::Mat orginal_img = cv_ptr->image.clone();
			auto idx = atoi(matches[i].class_id.c_str());
			
			// draw features
			circle(orginal_img, cv::Point(matches[i].x, matches[i].y), 8, cv::Scalar(0, 255, 0) , -1);
			cv::linemod::drawFeatures(orginal_img, linemods[key].getTemplates(matches[i].class_id.c_str(), matches[i].template_id), cv::Point( matches[i].x, matches[i].y), 2);

			// draw matches
			cv::imshow("img_processed", img_processed);
			cv::imshow("img_template", templates[key][idx]);
			cv::imshow("img_original",  orginal_img);
			cv::waitKey(200);
			if(i > 2)
				break;
		}
		
		// cluster the matches
		cluster_matches(matches, key);
	}

	// get the transformation matrix
	Eigen::Matrix4d m;
	auto match_id_as_string = std::string(4 - std::min(4, (int)matches[0].class_id.length()), '0') + matches[0].class_id.c_str();
	auto filename = std::string("/template") + match_id_as_string + std::string("_pose.xml");
	cv::FileStorage fs(template_dirs[key] + filename, cv::FileStorage::READ);
	fs["T"] >> m;

	return Eigen::Isometry3d(m);
}

void
set_template_dir_for_obj(const std::string& name, const std::string& dir)
{
	// add, in reality there should be a check whether the object exists
	object_dict[name] = template_dirs.size();
	template_dirs.push_back(dir);
	linemods.push_back(create_linemod_detector());
	templates.push_back(std::vector<cv::Mat>());
	cropped_templates.push_back(std::vector<cv::Mat>());

	// idx templates
	int cnt = 0;
	int key = object_dict[name];

	while(true)
	{
		// load file
		auto cnt_as_string = std::string(4 - std::min(4, (int)std::to_string(cnt).length()), '0') + std::to_string(cnt);
		auto filename = std::string("template") + cnt_as_string + std::string(".png");
		cv::Mat t = cv::imread(dir + filename, cv::IMREAD_UNCHANGED);

		// if no image is read
		if(t.empty()) 
			break;
		
		// write loaded templates
		if(cnt % 100 == 0)
			ROS_INFO("%04i templates are now added to the template matcher.", cnt);
		
		// otherwise pushback the image
		templates[key].push_back(t.clone());
		
		// crop the image
		cv::Mat out;
		cv::Rect win = autocrop(t);
		win.height = win.height + 4;
		win.width = win.width + 4;
		win.x = win.x - 2;
		win.y = win.y - 2;
		t = t(win);

		cv::inRange(t, cv::Scalar(0,0,244), cv::Scalar(1,1,255), out);
		out = 255 - out;
		
		// template for later visualizations
		cropped_templates[key].push_back(t.clone());

		// insert templates into the detectior
		std::vector<cv::Mat> src{t.clone()};
		auto tmp = linemods[key].addTemplate(src, cnt_as_string, out);
		if(tmp != 0)
			ROS_INFO_STREAM(cnt_as_string << " was not added to the template");

		cnt++;
	}

	ROS_ASSERT_MSG(cnt != 0, "did not load any templates for some reason.");
}

void cluster_matches(const std::vector<cv::linemod::Match> & matches, const int & key)
{
	// insert into template matching
	std::vector<cv::Mat> templates_over_tsh;

	// iterate through matches
	for(auto i = 0; i < matches.size(); i++)
	{
		if (matches[i].similarity > 0.90 && i < 10)
		{
			cv::Mat temp = cropped_templates[key][atoi(matches[i].class_id.c_str())].clone();
			cv::resize(temp, temp, cv::Size(64, 64));
			templates_over_tsh.push_back(temp);
		}
		else
			break;
	}

	if (templates_over_tsh.size() == 0)
		return;

	cv::Mat disp;
	ROS_INFO_STREAM(templates_over_tsh.size());

	// mainly for the purposes of debugging
	cv::hconcat(templates_over_tsh, disp);
	cv::imshow("templates", disp);
	cv::waitKey(0);
}

}

inline bool is_border
(
	cv::Mat& edge, 
	cv::Vec3b color
) 
{
	cv::Mat im = edge.clone().reshape(0,1);

	bool res = true;
	for(int i = 0; i < im.cols; ++i)
		res &= (color == im.at<cv::Vec3b>(0,i));

	return res;
}

inline cv::Rect autocrop
(
	cv::Mat & src
) 
{
	ROS_ASSERT(src.type() == CV_8UC3);
	cv::Rect win(0, 0, src.cols, src.rows);

	std::vector<cv::Rect> edges;
	edges.push_back(cv::Rect(0, 0, src.cols, 1));
	edges.push_back(cv::Rect(src.cols-2, 0, 1, src.rows));
	edges.push_back(cv::Rect(0, src.rows-2, src.cols, 1));
	edges.push_back(cv::Rect(0, 0, 1, src.rows));

	cv::Mat edge;
	int nborder = 0;
	cv::Vec3b color = src.at<cv::Vec3b>(0,0);

	for (size_t i = 0; i < edges.size(); ++i) 
	{
		edge = src(edges[i]);
		nborder += is_border(edge, color);
	}

	if (nborder < 4) return win;

	bool next;

	do 
	{
		edge = src(cv::Rect(win.x, win.height-2, win.width, 1));
		if( (next = is_border(edge, color)) )
			win.height--;
	} while (next && win.height > 0);

	do 
	{
		edge = src(cv::Rect(win.width-2, win.y, 1, win.height));
		if( (next = is_border(edge, color)) )
			win.width--;
	} while (next && win.width > 0);

	do 
	{
		edge = src(cv::Rect(win.x, win.y, win.width, 1));
		if( (next = is_border(edge, color)) )
			win.y++, win.height--;
	} while (next && win.y <= src.rows);

	do 
	{
		edge = src(cv::Rect(win.x, win.y, 1, win.height));
		if( (next = is_border(edge, color)) )
			win.x++, win.width--;
	} while (next && win.x <= src.cols);

	return win;
}

inline cv::linemod::Detector create_linemod_detector()
{
	std::vector<int> pyramid{4, 2};
	std::vector<cv::Ptr<cv::linemod::Modality>> modals;
	modals.push_back(cv::linemod::Modality::create("ColorGradient"));
	cv::linemod::Detector detector(modals, pyramid);
	return detector;
}