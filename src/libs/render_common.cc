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





















































// int screen2world(int x, int y)
// {
// 	GLint viewport[4];
// 	GLdouble modelview[16];
// 	GLdouble projection[16];
// 	GLfloat winX, winY, winZ;
// 	GLdouble posX, posY, posZ;
// 	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
// 	glGetDoublev(GL_PROJECTION_MATRIX, projection);
// 	glGetIntegerv(GL_VIEWPORT, viewport);
// 	for(int i = 0; i < 3; i++)
// 	{
// 		for (int j = 0; j < 3; j++)
// 		{
// 			winX = (float)i;
// 			winY = (float)viewport[3] - (float)j;
// 			glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
// 			gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
// 			std::cout<<"posX"<<posX<<" ";
// 		}
// 	}
// 	std::cout<<std::endl<<"done!";
// 	getchar();
// 	return posZ;
// }

// void RenderScene_XYZG()
// {
// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	glLoadIdentity();
// 	glPushMatrix();

// 	//gluLookAt(r*cos(c*du), r*sin(c*du), h, 0, 0, 0, -1*r*cos(c*du)/h,   -1*r*sin(c*du)/h,1);
// 	gluLookAt(xc, yc, zc, 0, 0, 0, -zc*xc/(xc*xc+yc*yc),  -zc*yc/(xc*xc+yc*yc),1);
// 	/*gluLookAt(r*cos(c*alpha)*cos(c*beta), r*cos(c*alpha)*sin(c*beta), r*sin(c*alpha), 0, 0, 0, -1*tan(c*alpha)*cos(c*beta),  -1*tan(c*alpha)*sin(c*beta),1); */

// 	glMatrixMode(GL_PROJECTION);
// 	glLoadMatrixf(projectionMatrix.data);
// 	glMatrixMode(GL_MODELVIEW);
// 	int normal_size=normal_vectors.size();
// 	for(int i=0;i<normal_size;i=i+3)
// 	{
// 		glBegin(GL_TRIANGLES);
// 		GLdouble VertorArr3[]={normal_vectors[i],normal_vectors[i+1],normal_vectors[i+2]};
// 		glNormal3dv(VertorArr3);
// 		for(int j=3*i;j<(i+3)*3-1;j=j+3)
// 		{
// 			GLdouble VertexArr3[]={vertices[j],vertices[j+1],vertices[j+2]};
// 			glVertex3dv(VertexArr3);
// 		}
// 		glEnd();
// 	}
// 	glFlush();
// 	glPopMatrix();
// 	glutSwapBuffers();
// }