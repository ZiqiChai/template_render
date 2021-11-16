#ifndef __RENDER_COMMON__
#define __RENDER_COMMON__

//C++
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <set>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//OpenGL
#include <GL/freeglut.h>
#include <GL/glu.h>
#include <GL/gl.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

int32_t createDirectory(const std::string &directoryPath);
std::string intToString(int number);
IplImage * loadDepth(std::string a_name);
bool WriteDepth(cv::Mat& depth, std::string filename);
bool WriteDepthDPT(std::string filename, cv::Mat& depth);
bool WriteDepthPNG(std::string filename, cv::Mat& depth);
bool WriteColorBMP(std::string filename, cv::Mat& color);
int STLRead(std::string nameStl, std::vector<GLdouble> &vertices, std::vector<GLdouble> &normal_vectors);
void IcospherePointWrite(Eigen::Vector3f vec, cv::FileStorage& fs);
void IcosphereWrite(float& radius, int& num_templates_on_one_sphere, float& angle_gamma_ratation, std::vector<Eigen::Vector3f>& vec_coordinate, cv::FileStorage& fs);
std::vector<cv::Mat> GetCroppedRGBD(cv::Mat color, cv::Mat depth);
std::vector<cv::Mat> GetCroppedRGBD(cv::Mat color, cv::Mat depth, int desired_width, int desired_height);
#endif