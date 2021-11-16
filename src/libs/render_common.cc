// common functions
#include "render_common.h"

#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

#include <stdint.h>
#include <string>
#define MAX_PATH_LEN 256

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

// 从左到右依次判断文件夹是否存在,不存在就创建
// example: /home/root/mkdir/1/2/3/4/
// 注意:最后一个如果是文件夹的话,需要加上 '\' 或者 '/'
int32_t createDirectory(const std::string &directoryPath)
{
	uint32_t dirPathLen = directoryPath.length();
	if (dirPathLen > MAX_PATH_LEN)
	{
		return -1;
	}
	char tmpDirPath[MAX_PATH_LEN] = { 0 };
	for (uint32_t i = 0; i < dirPathLen; ++i)
	{
		tmpDirPath[i] = directoryPath[i];
		if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
		{
			if (ACCESS(tmpDirPath, 0) != 0)
			{
				int32_t ret = MKDIR(tmpDirPath);
				if (ret != 0)
				{
					return ret;
				}
			}
		}
	}
	return 0;
}

//this function has a number input and string output
std::string intToString(int number)
{
	std::stringstream ss;
	ss << int(number);
	return ss.str();
}

IplImage * loadDepth( std::string a_name )
{
	std::ifstream l_file(a_name.c_str(),std::ofstream::in|std::ofstream::binary );

	if( l_file.fail() == true )
	{
		printf("cv_load_depth: could not open file for reading!\n");
		return NULL;
	}
	int l_row;
	int l_col;

	l_file.read((char*)&l_row,sizeof(l_row));
	l_file.read((char*)&l_col,sizeof(l_col));

	IplImage * lp_image = cvCreateImage(cvSize(l_col,l_row),IPL_DEPTH_16U,1);

	for(int l_r=0;l_r<l_row;++l_r)
	{
		for(int l_c=0;l_c<l_col;++l_c)
		{
			l_file.read((char*)&CV_IMAGE_ELEM(lp_image,unsigned short,l_r,l_c),sizeof(unsigned short));
		}
	}
	l_file.close();

	return lp_image;
}

bool WriteDepth(cv::Mat& depth, std::string filename)
{
	CV_Assert(depth.type() == CV_16UC1);
	std::ofstream ofs(filename.c_str(),std::ofstream::out|std::ofstream::binary );
	if( ofs.fail() == true )
	{
		printf("WriteDepth error: could not open file for writing!\n");
		return false;
	}
	int rows = depth.rows;
	int cols = depth.cols;
	ofs.write((char*)&rows,sizeof(rows));
	ofs.write((char*)&cols,sizeof(cols));
	for (int i = 0; i < rows; i++)
	{
		const ushort* data = depth.ptr<ushort>(i);
		for (int j = 0; j < cols; j++)
		{
			ofs.write((char*)&data[j],sizeof(ushort));
		}
	}
	ofs.close();
	return true;
}

bool WriteDepthDPT(std::string filename, cv::Mat& depth)
{
	if (depth.type() != CV_16UC1)
	{
		std::cout << "depth map type is not CV_16UC1!" << std::endl;
		exit(-1);
	}
	std::ofstream ofs((filename+".dpt").c_str(),std::ofstream::out|std::ofstream::binary );
	if( ofs.fail() == true )
	{
		printf("WriteDepthDPT error: could not open file for writing!\n");
		exit(-1);
	}
	int rows = depth.rows;
	int cols = depth.cols;
	ofs.write((char*)&rows,sizeof(rows));
	ofs.write((char*)&cols,sizeof(cols));
	for (int i = 0; i < rows; i++)
	{
		const ushort* data = depth.ptr<ushort>(i);
		for (int j = 0; j < cols; j++)
		{
			ofs.write((char*)&data[j],sizeof(ushort));
		}
	}
	ofs.close();
	return true;
}

bool WriteDepthPNG(std::string filename, cv::Mat& depth)
{
	if (depth.type() == CV_16UC1)
	{
		cv::imwrite(filename + ".png", depth);
		return true;
	}
	else
	{
		std::cout << "depth map type is not CV_16UC1!" << std::endl;
		exit(-1);
	}
}

bool WriteColorBMP(std::string filename, cv::Mat& color)
{
	if (color.type() == CV_8UC3)
	{
		cv::imwrite(filename + ".bmp", color);
		return true;
	}
	else
	{
		std::cout << "color image type is not CV_8UC3!" << std::endl;
		exit(-1);
	}
}

int STLRead(std::string nameStl, std::vector<GLdouble> &vertices, std::vector<GLdouble> &normal_vectors)
{
	clock_t start,finish;
	start=clock();

	std::string line;
	std::vector<std::string>tmp;
	std::ifstream fin;
	fin.open(nameStl.c_str());
	if(!fin)
	{
		std::cout<<"STLRead error!"<<std::endl;
		exit(-1);
	}
	while(std::getline(fin,line))
	{
		char Del[]=" ";
		char buff[100];
		strcpy(buff,line.c_str());
		char* token=strtok(buff,Del);
		while(token){
			tmp.push_back(token);
			token=strtok(NULL,Del);
		}
	}
	fin.close();

	for(int i=0;i<tmp.size();i++)
	{
		if(strcasecmp(tmp[i].c_str(), "normal")==0)
		{
			normal_vectors.push_back(atof(tmp[i+1].c_str()));
			normal_vectors.push_back(atof(tmp[i+2].c_str()));
			normal_vectors.push_back(atof(tmp[i+3].c_str()));
		}
		if(strcasecmp(tmp[i].c_str(), "vertex")==0)
		{
			vertices.push_back(1000*atof(tmp[i+1].c_str()));
			vertices.push_back(1000*atof(tmp[i+2].c_str()));
			vertices.push_back(1000*atof(tmp[i+3].c_str()));
		}
	}
	std::cout<<"stl file read successed"<<std::endl;

	finish=clock();
	std::cout<<"read stl file cost time: "<< ((double)(finish-start)/CLOCKS_PER_SEC) <<"s"<<std::endl;
	return 0;
}

void IcospherePointWrite(Eigen::Vector3f vec, cv::FileStorage& fs)
{
	fs<<"[:" << vec[0] << vec[1] << vec[2] << "]";
}

void IcosphereWrite(float& radius, int& num_templates_on_one_sphere, float& angle_gamma_ratation, std::vector<Eigen::Vector3f>& vec_coordinate, cv::FileStorage& fs)
{
	fs<<"r"<<radius;
	fs<<"num"<<num_templates_on_one_sphere;
	fs<<"angle_gamma_ratation"<<angle_gamma_ratation;
	fs<<"coordinate"<<"[";
	for(int i = 0;i<(int)vec_coordinate.size();i++)
	{
		Eigen::Vector3f tem_vec = vec_coordinate[i];
		IcospherePointWrite(tem_vec,fs);
	}
	fs<<"]";
}

std::vector<cv::Mat> GetCroppedRGBD(cv::Mat color, cv::Mat depth)
{
	cv::Mat imgGray;
	cv::threshold(color, imgGray,80, 255,CV_THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(imgGray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	std::vector<cv::Point> maxContours;
	for (int i = 0; i < contours.size(); i++)
	{
		if (maxContours.size() < contours[i].size())
		{
			maxContours = contours[i];
		}
	}
	if (maxContours.size())
	{
		cv::Rect r0 = cv::boundingRect(cv::Mat(maxContours));
		r0.x -= 3;
		r0.y -= 3;
		r0.width += 6;
		r0.height += 6;
		cv::Mat Cropped_rgbImg = color(r0);
		cv::Mat Cropped_depImg  = depth(r0);
		std::vector<cv::Mat> res;
		res.push_back(Cropped_rgbImg);
		res.push_back(Cropped_depImg);
		return res;
	}
	else
	{
		std::cout << "failed to find contours!" << std::endl;
		exit(-1);
	}
}

std::vector<cv::Mat> GetCroppedRGBD(cv::Mat color, cv::Mat depth, int desired_width, int desired_height)
{
	cv::Mat imgGray;
	cv::threshold(color, imgGray,80, 255,CV_THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(imgGray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	std::vector<cv::Point> maxContours;
	for (int i = 0; i < contours.size(); i++)
	{
		if (maxContours.size() < contours[i].size())
		{
			maxContours = contours[i];
		}
	}
	if (maxContours.size())
	{
		cv::Rect r0 = cv::boundingRect(cv::Mat(maxContours));
		if (r0.width<desired_width)
		{
			r0.x -= int((desired_width-r0.width)/2);
			if (r0.x<0){r0.x=0;}
		}
		if (r0.height<desired_height)
		{
			r0.y -= int((desired_height-r0.height)/2);
			if (r0.y<0){r0.y=0;}
		}
		r0.width = desired_width;
		r0.height = desired_height;
		cv::Mat Cropped_rgbImg = color(r0);
		cv::Mat Cropped_depImg  = depth(r0);
		std::vector<cv::Mat> res;
		res.push_back(Cropped_rgbImg);
		res.push_back(Cropped_depImg);
		return res;
	}
	else
	{
		std::cout << "failed to find contours!" << std::endl;
		exit(-1);
	}
}