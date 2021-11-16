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
#define random(x) (rand()%x)

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
float xc,yc,zc;
float angle_gamma_ratation=0;

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

	if (WriteColorBMP(filename, color) && WriteDepthPNG(filename, depth)) // && WriteDepthDPT(filename, depth))
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
	glutSwapBuffers();
}

void RenderScene_XYZ()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glPushMatrix();

	float upx=-zc*xc/(xc*xc+yc*yc);
	float upy=-zc*yc/(xc*xc+yc*yc);
	float upz=1.0f;
	float up = sqrt(pow(upx,2)+pow(upy,2)+pow(upz,2));
	upx = upx/up;
	upy = upy/up;
	upz = upz/up;
	// std::cout << "up Vector3d before rotation is: " << std::endl << upx <<" "<< upy <<" "<< upz << std::endl;

	float n = sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2));
	float nx= xc/n;
	float ny= yc/n;
	float nz= zc/n;
	// std::cout << "original rotation axis is: " << std::endl << xc <<" "<< yc <<" "<< zc << std::endl;
	// std::cout << "normaled rotation axis is: " << std::endl << nx <<" "<< ny <<" "<< nz << std::endl;

	Eigen::Vector3d v(upx, upy, upz);
	Eigen::AngleAxisd angle_axis( angle_gamma_ratation*c, Eigen::Vector3d(nx, ny, nz));
	Eigen::Vector3d rotated_v = angle_axis.matrix().inverse()*v;
	upx=rotated_v[0];
	upy=rotated_v[1];
	upz=rotated_v[2];
	// std::cout << "rotated Vector3d after rotation / up Vector3d after rotation is: [" << std::endl << upx <<" "<< upy <<" "<< upz << "]" << std::endl;

	// keep Z direction always up or not
	if (change_Z_direction)
	{
		if (zc >= 0)
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
	gluLookAt(xc, yc, zc, 0, 0, 0, zDirection*upx, zDirection*upy, zDirection*upz);
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
	glutSwapBuffers();
}


int main(int argc,char *argv[])
{
	string parameterFileName = "parameter.yml";
	if (argc == 2)
	{
		parameterFileName = string(argv[1]);
	}
	Config::setParameterFile(pkg_loc + "/" + parameterFileName);
	std::cout << "Using parameters in file: " << (pkg_loc + "/" + parameterFileName) << std::endl;

	STLRead(pkg_loc + "/" + Config::get<string>("stl_folder_name") + "/" + Config::get<string>("stl_file_name"), vertices, normal_vectors);

	string random_scean_folder = pkg_loc + "/" + Config::get<string>("random_scene_folder_name") + "/";
	if(createDirectory(random_scean_folder) == 0)
		std::cout << "output folder create successfully! Go on!" << std::endl;

	//generate_model have choice: 0, 1, 2, 3.
	int generate_model = Config::get<int>("int_generate_model");
	int total_num_random_templates = Config::get<int>("total_num_random_templates");
	change_Z_direction = Config::get<bool>("change_Z_direction");
	int count = 0;

	readCameraParameter();

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(640,480);

	if (generate_model == 3)
	{
		glutCreateWindow("RenderScene_XYZ_Mode");
		LightInit2();
		glutDisplayFunc(&RenderScene_XYZ);
	}
	else
	{
		glutCreateWindow("RenderScene_ABG_Mode");
		LightInit2();
		glutDisplayFunc(&RenderScene);
	}
	glutLeaveMainLoop();

	srand((int)time(0));

	// XML used to store coordinates of icosohedron
	cv::FileStorage fs(pkg_loc + "/" + Config::get<string>("random_scene_folder_name") + "/../" + Config::get<string>("random_scene_xml_file_name"), cv::FileStorage::WRITE);
	fs<<"parameterData"<<"[";

	// TXT file used to store "img_Name, label, r, num, gamma, xyz
	std::ofstream fs_txt(pkg_loc + "/" + Config::get<string>("random_scene_folder_name") + "/../" + Config::get<string>("random_scene_txt_file_name"), std::ios::trunc);


	/****************generate use UV Layer 6D parameter****************************/
	if(generate_model == 0)
	{
		std::cout<<"generate_model == 0, generate use same UV Layer 6D parameter!"<<std::endl;
		float rMin, rMax, dr, alphaMin, alphaMax, da, betaMin, betaMax, db, gammaMin, gammaMax, dg;
		rMin = Config::get<float>("rMin"); rMax = Config::get<float>("rMax");
		alphaMin = Config::get<float>("alphaMin"); alphaMax = Config::get<float>("alphaMax");
		betaMin = Config::get<float>("betaMin");  betaMax = Config::get<float>("betaMax");
		gammaMin = Config::get<float>("gammaMin"); gammaMax = Config::get<float>("gammaMax");
		dr = Config::get<float>("dr");
		da = Config::get<float>("da");
		db = Config::get<float>("db");
		dg = Config::get<float>("dg");
		r = rMin;
		alpha = alphaMin;
		beta = betaMin;
		gama = gammaMin;

		for (float current_r=rMin; ( (current_r<rMax) || (abs(rMax-current_r)<= 0.1) ); current_r += dr)
		{
			for (float angle_alpha = alphaMin; ( (angle_alpha<alphaMax) || (abs(alphaMax-angle_alpha)<= 0.1) ); angle_alpha += da)
			{
				for (float angle_beta = betaMin; angle_beta < betaMax; angle_beta += db)
				{
					for (float angle_gamma = gammaMin; ( (angle_gamma<gammaMax) || (abs(gammaMax-angle_gamma)<= 0.1) ); angle_gamma += dg)
					{
						// rename alpha beta gamma r in this repeat block to avoid replacing the global var alpha beta gamma and r
						// otherwise nothing would be rendered.
						r     = current_r;
						alpha = angle_alpha;
						beta  = angle_beta;
						gama  = angle_gamma;
						std::cout << "r: " << r << " alpha: " << alpha << " beta: " << beta << " gamma: " << gama << std::endl;

						xc=r*cos(c*alpha)*cos(c*beta);
						yc=r*cos(c*alpha)*sin(c*beta);
						zc=r*sin(c*alpha);
						std::cout<<"xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<< gama <<std::endl;

						//xc,yc == 0时 Lookat函数里面分母会变为0,所以需要调整
						if(xc == 0 && yc == 0)
						{
							xc = yc = 1;
							if(zc > 0)
								zc = sqrt(zc*zc - xc*xc - yc*yc);
							else
								zc = -sqrt(zc*zc - xc*xc - yc*yc);
							std::cout<<"adjusted -- xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<<gama<<std::endl;
						}

						fs << "[" << r << xc << yc << zc << gama << "]";

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
						fs_txt << "img_Name: " << "test" + intToString(count) << ".bmp" << "  label: "<< label
						<< "  r: " << r << "  alpha: " << alpha << "  beta: " << beta << "  gamma: " << gama
						<< "  xyz: " << xc << " " << yc << " " << zc << "\n";

						// render use r, alpha, beta, gama.
						RenderScene();
						cv::waitKey(10);
						string sName = random_scean_folder + "test" + intToString(count) ;
						std::cout<<"count:"<<(count+1)<<"  sName:"<<sName<<std::endl;

						count++;
						if (!screenshot(sName))
						{
							std::cout<<"screenshot error when store templates!"<<std::endl;
							cv::waitKey(5000);
						}
					}
				}
			}
		}
		std::cout<<"Total Number of RandomTemplates is :"<< count <<std::endl;
		fs<<"]";
		fs<<"Total_Number_of_Random_scenes"<<"["<<count<<"]";
		fs.release();
	}
	/********************end generate use UV Layer 6D parameter************************/


	/******************random sampleing in UV Layer 6D generate**************************/
	if(generate_model == 1)
	{
		std::cout<<"generate_model == 1, random sampleing in UV Layer 6D generate!"<<std::endl;
		float rMin, rMax, dr, alphaMin, alphaMax, da, betaMin, betaMax, db, gammaMin, gammaMax, dg;
		rMin = Config::get<float>("rMin"); rMax = Config::get<float>("rMax");
		alphaMin = Config::get<float>("alphaMin"); alphaMax = Config::get<float>("alphaMax");
		betaMin = Config::get<float>("betaMin");  betaMax = Config::get<float>("betaMax");
		gammaMin = Config::get<float>("gammaMin"); gammaMax = Config::get<float>("gammaMax");
		dr = Config::get<float>("dr");
		da = Config::get<float>("da");
		db = Config::get<float>("db");
		dg = Config::get<float>("dg");
		r = rMin;
		alpha = alphaMin;
		beta = betaMin;
		gama = gammaMin;

		for (int i = 0;i < total_num_random_templates;i++)
		{
			if ( abs(dr-0.0) > 0.00001 )
				r     = int(random(int(rMax-rMin))/dr)*dr + rMin;
			if ( abs(da-0.0) > 0.00001 )
				alpha = int(random(int(alphaMax-alphaMin))/da)*da + alphaMin;
			if ( abs(db-0.0) > 0.00001 )
				beta  = int(random(int(betaMax-betaMin))/db)*db + betaMin;
			if ( abs(dg-0.0) > 0.00001 )
				gama  = int(random(int(gammaMax-gammaMin))/dg)*dg + gammaMin;

			std::cout << "r: " << r << " alpha: " << alpha << " beta: " << beta << " gamma: " << gama << std::endl;

			xc=r*cos(c*alpha)*cos(c*beta);
			yc=r*cos(c*alpha)*sin(c*beta);
			zc=r*sin(c*alpha);
			std::cout<<"xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<< gama <<std::endl;

			//xc,yc == 0时 Lookat函数里面分母会变为0,所以需要调整
			if(xc == 0 && yc == 0)
			{
				xc = yc = 1;
				if(zc > 0)
					zc = sqrt(zc*zc - xc*xc - yc*yc);
				else
					zc = -sqrt(zc*zc - xc*xc - yc*yc);
				std::cout<<"adjusted -- xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<<gama<<std::endl;
			}

			fs << "[" << r << xc << yc << zc << gama << "]";

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
			fs_txt << "img_Name: " << "test" + intToString(count) << ".bmp" << "  label: "<< label
			<< "  r: " << r << "  alpha: " << alpha << "  beta: " << beta << "  gamma: " << gama
			<< "  xyz: " << xc << " " << yc << " " << zc << "\n";

			// render use r, alpha, beta, gama.
			RenderScene();
			cv::waitKey(10);
			string sName = random_scean_folder + "test" + intToString(count) ;
			std::cout<<"count:"<<(count+1)<<"  sName:"<<sName<<std::endl;

			count++;
			if (!screenshot(sName))
			{
				std::cout<<"screenshot error when store templates!"<<std::endl;
				cv::waitKey(5000);
			}
		}
		std::cout<<"Total Number of RandomTemplates is :"<< count <<std::endl;
		fs<<"]";
		fs<<"Total_Number_of_Random_scenes"<<"["<<count<<"]";
		fs.release();
	}
	/*****************end random sampleing in UV Layer 6D generate***************************/


	/******************really random generate using parameter of int valve*******************/
	if(generate_model == 2)
	{
		std::cout<<"generate_model == 2, really random generate using parameter of int valve!"<<std::endl;
		float rMin, rMax, dr, alphaMin, alphaMax, da, betaMin, betaMax, db, gammaMin, gammaMax, dg;
		rMin = Config::get<float>("rMin"); rMax = Config::get<float>("rMax");
		alphaMin = Config::get<float>("alphaMin"); alphaMax = Config::get<float>("alphaMax");
		betaMin = Config::get<float>("betaMin");  betaMax = Config::get<float>("betaMax");
		gammaMin = Config::get<float>("gammaMin"); gammaMax = Config::get<float>("gammaMax");
		dr = Config::get<float>("dr");
		da = Config::get<float>("da");
		db = Config::get<float>("db");
		dg = Config::get<float>("dg");
		r = rMin;
		alpha = alphaMin;
		beta = betaMin;
		gama = gammaMin;

		for (int i = 0;i < total_num_random_templates;i++)
		{
			if ( abs(dr-0.0) > 0.00001 )
				r     = int(random(int(rMax-rMin))) + rMin;
			if ( abs(da-0.0) > 0.00001 )
				alpha = int(random(int(alphaMax-alphaMin))) + alphaMin;
			if ( abs(db-0.0) > 0.00001 )
				beta  = int(random(int(betaMax-betaMin))) + betaMin;
			if ( abs(dg-0.0) > 0.00001 )
				gama  = int(random(int(gammaMax-gammaMin))) + gammaMin;

			std::cout << "r: " << r << " alpha: " << alpha << " beta: " << beta << " gamma: " << gama << std::endl;

			xc=r*cos(c*alpha)*cos(c*beta);
			yc=r*cos(c*alpha)*sin(c*beta);
			zc=r*sin(c*alpha);
			std::cout<<"xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<< gama <<std::endl;

			//xc,yc == 0时 Lookat函数里面分母会变为0,所以需要调整
			if(xc == 0 && yc == 0)
			{
				xc = yc = 1;
				if(zc > 0)
					zc = sqrt(zc*zc - xc*xc - yc*yc);
				else
					zc = -sqrt(zc*zc - xc*xc - yc*yc);
				std::cout<<"adjusted -- xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<<gama<<std::endl;
			}

			fs << "[" << r << xc << yc << zc << gama << "]";

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
			fs_txt << "img_Name: " << "test" + intToString(count) << ".bmp" << "  label: "<< label
			<< "  r: " << r << "  alpha: " << alpha << "  beta: " << beta << "  gamma: " << gama
			<< "  xyz: " << xc << " " << yc << " " << zc << "\n";

			// render use r, alpha, beta, gama.
			RenderScene();
			cv::waitKey(10);
			string sName = random_scean_folder + "test" + intToString(count) ;
			std::cout<<"count:"<<(count+1)<<"  sName:"<<sName<<std::endl;

			count++;
			if (!screenshot(sName))
			{
				std::cout<<"screenshot error when store templates!"<<std::endl;
				cv::waitKey(5000);
			}
		}
		std::cout<<"Total Number of RandomTemplates is :"<< count <<std::endl;
		fs<<"]";
		fs<<"Total_Number_of_Random_scenes"<<"["<<count<<"]";
		fs.release();
	}
	/****************end really random generate using parameter of int valve******************/


	/**************read coordinates from XML file and render from XYZ coordinates*************/
	if ( generate_model == 3)
	{
		std::cout<<"generate_model == 3, read coordinates from XML file and render from XYZ coordinates!"<<std::endl;
		cv::FileStorage fs_read(pkg_loc + "/" + Config::get<string>("input_xml_file_name_for_random_generater"), cv::FileStorage::READ);

		float Scale2millimeter = 0;
		Scale2millimeter = fs_read["Scale2millimeter"];
		if ( Scale2millimeter == 0)
		{
			std::cout << "ERROR: Scale2millimeter is 0. STOP!" << std::endl;
			exit(-1);
		}

		cv::FileNode fn_read = fs_read["data"];
		for (cv::FileNodeIterator i = fn_read.begin(), iend = fn_read.end(); i != iend; ++i)
		{
			int num_tem=0;
			// r is global var, while not used in this code block.
			(*i)["r"]>>r;
			(*i)["num"]>>num_tem;
			(*i)["angle_gamma_ratation"]>>angle_gamma_ratation;
			cv::FileNode coordinate_fn = (*i)["coordinate"];
			cv::FileNodeIterator it = coordinate_fn.begin(), it_end = coordinate_fn.end();
			for ( ; it != it_end; ++it)
			{
				Eigen::Vector3f vertex_coordinate;
				cv::FileNodeIterator fni = (*it).begin();
				fni >> vertex_coordinate[0] >> vertex_coordinate[1] >> vertex_coordinate[2];
				xc = vertex_coordinate[0]*Scale2millimeter;  //convert m to mm
				yc = vertex_coordinate[1]*Scale2millimeter;  //convert m to mm
				zc = vertex_coordinate[2]*Scale2millimeter;  //convert m to mm
				std::cout<<"xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<< angle_gamma_ratation <<std::endl;

				//xc,yc == 0时 Lookat函数里面分母会变为0,所以需要调整
				if(xc == 0 && yc == 0)
				{
					xc = yc = 1;
					if(zc > 0)
						zc = sqrt(zc*zc - xc*xc - yc*yc);
					else
						zc = -sqrt(zc*zc - xc*xc - yc*yc);
					std::cout<<"adjusted -- xc: "<< xc <<"; yc: "<< yc <<"; zc: "<< zc <<"; gamma: "<< angle_gamma_ratation <<std::endl;
				}

				fs << "[" << r << xc << yc << zc << angle_gamma_ratation << "]";

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
				fs_txt << "img_Name: " << "test" + intToString(count) << ".bmp" << "  label: "<< label
				<< "  r: " << r << "  xyz: " << xc << " " << yc << " " << zc << "  gamma: " << angle_gamma_ratation << "\n";

				RenderScene_XYZ();
				string sName = random_scean_folder + "test" + intToString(count) ;
				std::cout<<"count:"<<(count+1)<<"  sName:"<<sName<<std::endl;

				count++;
				if (!screenshot(sName))
				{
					std::cout<<"screenshot error when store templates!"<<std::endl;
					cv::waitKey(5000);
				}
			}
		}

		int totalnum;
		fs_read["total_num_of_all_templates"] >> totalnum;
		std::cout<<"Total Number of coordinates in XML file is :"<< totalnum <<std::endl;
		std::cout<<"Total Number of Random Templates is :"<< count <<std::endl;
		fs<<"]";
		fs<<"Total_Number_of_Random_scenes"<<"["<<count<<"]";
		fs.release();
	}
	/********end of read coordinates from XML file and render from XYZ coordinates************/

	std::cout<<"generate completed!"<<std::endl;
	cv::waitKey(5000);
}
