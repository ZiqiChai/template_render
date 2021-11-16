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

//ROS
#include <ros/ros.h>
#include <ros/package.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//OpenGL
#include <GL/freeglut.h>
#include <GL/glu.h>
#include <GL/gl.h>

//Customed
#include "icosphere.h"
#include "config.h"
#include "render_common.h"

#define THREAD_SIZE (2*PAGE_SIZE)
#define pi 3.14159

using namespace std;
using namespace cv;
using namespace Eigen;

std::string pkg_loc = ros::package::getPath("template_render");

// used to load vertices and normals in the *.stl file
std::vector <GLdouble> vertices;
std::vector <GLdouble> normal_vectors;

float c= pi/180.0f;
float r;
float alpha,beta,gama;
bool change_Z_direction;
float zDirection;

int desired_img_size_width=0;
int desired_img_size_height=0;

bool screenshot(string filename);


bool screenshot(string filename)
{
	GLint iViewport[4];
	glGetIntegerv(GL_VIEWPORT, iViewport);
	int Img_width=iViewport[2];
	int Img_height=iViewport[3];
	unsigned long lImageSize = Img_width * Img_height * 3;

	GLbyte* pBits = 0;
	GLfloat* pBits2 = 0;
	GLenum lastBuffer;
	pBits = (GLbyte*)new unsigned char[lImageSize];
	pBits2 = (GLfloat*)new float[lImageSize];
	if (!pBits) return false;
	if (!pBits2) return false;

	glPixelStorei(GL_PACK_ROW_LENGTH, 0);
	glPixelStorei(GL_PACK_SKIP_ROWS, 0);
	glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
	glGetIntegerv(GL_READ_BUFFER, (GLint*)&lastBuffer);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, Img_width, Img_height, GL_BGR_EXT, GL_UNSIGNED_BYTE, pBits);
	glReadPixels(0, 0, Img_width, Img_height, GL_DEPTH_COMPONENT,GL_FLOAT, pBits2);
	glReadBuffer(lastBuffer);

	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	cv::Mat color = cv::Mat(Img_height,Img_width,CV_8UC3);
	for(int i = 0; i < Img_height; i++)
	{
		uchar* data = color.ptr<uchar>(i);
		for (int j = 0; j < Img_width; j++)
		{
			data[3*j+0] = pBits[Img_width*i*3+3*j+0];
			data[3*j+1] = pBits[Img_width*i*3+3*j+1];
			data[3*j+2] = pBits[Img_width*i*3+3*j+2];
		}
	}
	free(pBits);
	// OpenGL origin is Left-Bottom, while OpenCV origin is Left-UP. flip color along X axis.
	cv::flip(color,color,0);

	cv::Mat depth = cv::Mat(Img_height,Img_width,CV_16UC1);
	for(int i = 0; i < Img_height; i++)
	{
		ushort* data = depth.ptr<ushort>(i);
		for (int j = 0; j < Img_width; j++)
		{
			winX = (float)j;
			winY = (float)viewport[3] - (float)i;
			winZ = pBits2[Img_width*i+j];
			gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
			ushort d = (ushort)-posZ;
			d = d > 3500 ? 0 : d;
			data[j] = (ushort)d;
		}
	}
	free(pBits2);
	// OpenGL origin is Left-Bottom, while OpenCV origin is Left-UP. flip depth along X axis.
	cv::flip(depth,depth,0);

	std::vector<cv::Mat> CroppedRGBD = GetCroppedRGBD(color, depth, desired_img_size_width, desired_img_size_height);

	if (WriteColorBMP(filename, CroppedRGBD[0]) && WriteDepthPNG(filename, CroppedRGBD[1])) // && WriteDepthDPT(filename, CroppedRGBD[1]))
		return true;
	else
		return false;
}

struct Matrix44
{
	union
	{
		float data[16];
		float mat[4][4];
	} ;
}projectionMatrix;

void readCameraParameter()
{
	int imagewidth  = Config::get<int>("image_width");
	int imageheight = Config::get<int>("image_height");

	float f_x = Config::get<float>("fx"); // focal length in x axis
	float f_y = Config::get<float>("fy"); // focal length in y axis
	float c_x = Config::get<float>("cx"); // camera primary point x
	float c_y = Config::get<float>("cy"); // camera primary point y

	float near1 = 0.01; // Near clipping distance
	float far1 = 4000;  // Far clipping distance

	projectionMatrix.data[0] = 2.0 * f_x / imagewidth;
	projectionMatrix.data[1] = 0.0;
	projectionMatrix.data[2] = 0.0;
	projectionMatrix.data[3] = 0.0;

	projectionMatrix.data[4] = 0.0;
	projectionMatrix.data[5] = 2.0 * f_y / imageheight;
	projectionMatrix.data[6] = 0.0;
	projectionMatrix.data[7] = 0.0;

	projectionMatrix.data[8] = 2.0 * c_x / imagewidth - 1.0;
	projectionMatrix.data[9] = 2.0 * c_y / imageheight - 1.0;
	projectionMatrix.data[10] = -( far1+near1 ) / ( far1 - near1 );
	projectionMatrix.data[11] = -1.0;

	projectionMatrix.data[12] = 0.0;
	projectionMatrix.data[13] = 0.0;
	projectionMatrix.data[14] = -2.0 * far1 * near1 / ( far1 - near1 );
	projectionMatrix.data[15] = 0.0;

}

void LightInit(void)
{
	GLfloat mat_ambient[]   = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat mat_diffuse[]   = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat mat_specular[]  = {0.00, 0.00, 0.00, 0.00};
	GLfloat mat_shininess[] = {60.0};

	GLfloat light0_position[] = {1.00, 1.00, 1.00, 0.00};
	GLfloat light0_ambient[]  = {0.2f, 0.2f, 0.2f, 1.0f};
	GLfloat light0_diffuse[]  = {0.9f, 0.9f, 0.9f, 1.0f};
	GLfloat light0_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

	GLfloat light1_position[] = {1.00, 1.00, -1.0, 0.00};
	GLfloat light1_ambient[]  = {0.1f, 0.1f, 0.1f, 1.0f};
	GLfloat light1_diffuse[]  = {0.1f, 0.1f, 0.1f, 1.0f};
	GLfloat light1_specular[] = {0.00, 0.00, 0.00, 1.00};

	GLfloat light2_position[] = {-1.0, 1.0, 1.0, 0.00};
	GLfloat light2_ambient[]  = { 0.0, 0.0, 0.0, 0.05};
	GLfloat light2_diffuse[]  = { 0.0, 1.0, 2.0, 0.05};
	GLfloat light2_specular[] = { 0.0, 0.0, 0.0, 0.05};

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);

	glMaterialfv(GL_FRONT_AND_BACK , GL_AMBIENT,mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK , GL_DIFFUSE,mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK , GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK , GL_SHININESS, mat_shininess);

	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);

	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);

	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT2, GL_AMBIENT, light2_ambient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse);
	glLightfv(GL_LIGHT2, GL_SPECULAR, light2_specular);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	// glEnable(GL_LIGHT2);
	glEnable(GL_DEPTH_TEST);
}

void LightInit2(void)
{
	GLfloat ambientLight[] = {0.3f, 0.3f, 0.3f, 1.0f};
	GLfloat diffuseLight[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat lightPos[] = {50.0f, 50.0f, 50.0f, 0.0f};
	GLfloat gray[] = {0.75f, 1.0f, 0.75f, 1.0f};

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	glEnable(GL_LIGHT0);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 128);
	glClearColor(0.0f,0.0f,0.0f,1.0f);
	glColor4f(0.75,0.75,0.75,1.0);
	glShadeModel(GL_SMOOTH);
}

void RenderScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glPushMatrix();

	// keep Z direction always up or not
	if (change_Z_direction)
	{
		if (alpha >= 0)
		{
			zDirection = 1;
		}
		else
		{
			zDirection = -1;
		}
	}
	else
	{
		zDirection = 1;
	}
    //gluLookAt(r*cos(c*du), r*sin(c*du), h, 0, 0, 0, -1*r*cos(c*du)/h,   -1*r*sin(c*du)/h,1);
	//gluLookAt(x, y, z, 0, 0, 0, -z*x/(x*x+y*y),  -z*y/(x*x+y*y),1);
	/*gluLookAt(r*cos(c*alpha)*cos(c*beta), r*cos(c*alpha)*sin(c*beta), r*sin(c*alpha), 0, 0, 0, -1*tan(c*alpha)*cos(c*beta),  -1*tan(c*alpha)*sin(c*beta),1); */
	gluLookAt(r*cos(c*alpha)*cos(c*beta), r*cos(c*alpha)*sin(c*beta), r*sin(c*alpha), 0, 0, 0, zDirection*(sin(c*gama)*sin(c*beta)-cos(c*gama)*sin(c*alpha)*cos(c*beta)),zDirection*(-sin(c*gama)*cos(c*beta)-cos(c*gama)*sin(c*alpha)*sin(c*beta)),zDirection*cos(c*gama)*cos(c*alpha));
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projectionMatrix.data);
	glMatrixMode(GL_MODELVIEW);

	int normal_size=normal_vectors.size();
	for(int i=0;i<normal_size;i=i+3)
	{
		glBegin(GL_TRIANGLES);
		GLdouble VertorArr3[]={normal_vectors[i],normal_vectors[i+1],normal_vectors[i+2]};
		glNormal3dv(VertorArr3);
		for(int j=3*i;j<(i+3)*3-1;j=j+3)
		{
			GLdouble VertexArr3[]={vertices[j],vertices[j+1],vertices[j+2]};
			glVertex3dv(VertexArr3);
		}
		glEnd();
	}
	glFlush();
	glPopMatrix();
	// glutSwapBuffers(hrenderDC);
	glutSwapBuffers();
}


int main(int argc,char *argv[])
{
	if (argc<3)
	{
		std::cout<<"please input desired desired_img_size_width[argv[1]] and desired_img_size_height[argv[2]], optional parameter FileName[argv[3]]"<<std::endl;
	}
	sscanf( argv[1],"%d",&desired_img_size_width);
	sscanf( argv[2],"%d",&desired_img_size_height);

	string parameterFileName = "parameter.yml";
	if (argc == 4)
	{
		parameterFileName = string(argv[3]);
	}
	Config::setParameterFile(pkg_loc + "/" + parameterFileName);
	std::cout << "Using parameters in file: " << (pkg_loc + "/" + parameterFileName) << std::endl;

	STLRead(pkg_loc + "/" + Config::get<string>("stl_folder_name") + "/" + Config::get<string>("stl_file_name"), vertices, normal_vectors);

	string template_folder = pkg_loc + "/" + Config::get<string>("template_folder_name") + "/";
	if(createDirectory(template_folder) == 0)
		std::cout << "output folder create successfully! Go on!" << std::endl;
	change_Z_direction = Config::get<bool>("change_Z_direction");

	readCameraParameter();

	bool flagr,flaga,flagb,flagg;
	float rMin, rMax, dr, alphaMin, alphaMax, da, betaMin, betaMax, db, gMin, gMax, dg;
	flagr = false; flaga= false; flagb= false; flagg= false;

	rMin = Config::get<float>("rMin"); rMax = Config::get<float>("rMax");
	alphaMin = Config::get<float>("alphaMin"); alphaMax = Config::get<float>("alphaMax");
	betaMin = Config::get<float>("betaMin");  betaMax = Config::get<float>("betaMax");
	gMin = Config::get<float>("gammaMin"); gMax = Config::get<float>("gammaMax");
	r = rMin;
	alpha = alphaMin;
	beta = betaMin;
	gama = gMin;
	dr = Config::get<float>("dr");
	da = Config::get<float>("da");
	db = Config::get<float>("db");
	dg = Config::get<float>("dg");

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(640,480);
	glutCreateWindow("RenderScene_ABG_Mode");
	LightInit2();
	glutDisplayFunc(&RenderScene);
	//glutMainLoop();
	glutLeaveMainLoop();

	int count = 0;
	// TXT file used to store "img_Name, label, r, num, gamma, xyz
	std::ofstream fs_txt(pkg_loc + "/" + Config::get<string>("template_folder_name") + "/../" + Config::get<string>("txt_file_name"), std::ios::trunc);

	while (1)
	{
		RenderScene();
		cv::waitKey(10);
		string img_Name = "Radius" + intToString(r) + "Alpha" + intToString(alpha) + "Beta" + intToString(beta) + "Gamma" + intToString(gama);
		cout<<"template : "<< count <<" with Name:"<<img_Name<<endl;
		float xc = r*cos(c*alpha)*cos(c*beta);
		float yc = r*cos(c*alpha)*sin(c*beta);
		float zc = r*sin(c*alpha);

		int label=8;
		if (xc>0 && yc>0 && zc>0)
		{
			label=0;
		}
		else if(xc>0 && yc<0 && zc>0)
		{
			label=1;
		}
		else if(xc<0 && yc<0 && zc>0)
		{
			label=2;
		}
		else if(xc<0 && yc>0 && zc>0)
		{
			label=3;
		}
		else if(xc>0 && yc>0 && zc<0)
		{
			label=4;
		}
		else if(xc>0 && yc<0 && zc<0)
		{
			label=5;
		}
		else if(xc<0 && yc<0 && zc<0)
		{
			label=6;
		}
		else if(xc<0 && yc>0 && zc<0)
		{
			label=7;
		}
		else
		{
			label=8;
		}
		fs_txt<<"img_Name: "<<img_Name<<".bmp"<<"  label: "<<label<<"  r: "<<r<<"  num: "<<count<<"  gamma: "<<gama<<"  xyz: "<<xc<<" "<<yc<<" "<<zc<<"\n";
		string sName = template_folder + img_Name ;
		count++;

		if (!screenshot(sName))
		{
			cout<<"screenshot error when store templates!"<<endl;
			getchar();
		}

		/**\
		 * update render parameters
		\**/
		// add gamma if delta-gamma is none Zero
		if (dg != 0)
		{
			if (gama <= gMax - dg)
			{
				gama += dg;
			}
			else
			{
				gama = gMin;
				flagb = true;
			}
		}
		else
		{
			flagb = true;
		}

		// add beta if delta-beta is none Zero
		if ((db != 0) & flagb)
		{
			// beta from 0 to 360, 0 and 360 is the same position, so use "beta < betaMax - db" instead of "beta <= betaMax - db"
			if (beta < betaMax - db)
			{
				beta += db;
			}
			else
			{
				beta = betaMin;
				flaga = true;
			}
			// add beta once and set flagb false
			flagb = false;
		}
		if ((db == 0) & flagb)
		{
			flaga = true;
		}

		// add alpha if delta-alpha is none Zero
		if ((da != 0) & flaga)
		{
			// for example, alpha from -90 to 90, no same position, so use "alpha <= alphaMax - da" instead of "alpha < alphaMax - da"
			if (alpha <= alphaMax - da)
			{
				alpha += da;
			}
			else
			{
				alpha = alphaMin;
				flagr = true;
			}
			// add alpha once and set flaga false
			flaga = false;
		}
		if ((da == 0) & flaga)
		{
			flagr = true;
		}

		// add radius if delta-radius is none Zero
		if ((dr != 0) & flagr)
		{
			if (r <= rMax - dr)
			{
				r += dr;
			}
			else
			{
				break;
			}
			flagr = false;
		}
		// only render images on one radius
		if ((dr == 0) & flagr)
		{
			break;
		}

	}
	fs_txt.close();
	cout<<"total num of templates is : "<< count <<endl;
	cout<<"generate completed!"<<endl;
	cv::waitKey(3000);
}
