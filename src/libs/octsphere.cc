#include "octsphere.h"
#include <GL/gl.h>
#include <map>
#include <iostream>

using namespace Eigen;
using namespace std;

//--------------------------------------------------------------------------------
// octahedron data
//--------------------------------------------------------------------------------
float X,Y,Z;

//--------------------------------------------------------------------------------

IcoSphere::IcoSphere(float pr,unsigned int levels)
{
	// init with an octahedron
	r = pr;
	X = r;
	Y = r;
	Z = r;

	GLfloat vdata[6][3] =
	{
		{X, 0.0, 0.0}, {0.0, Y, 0.0}, {-X, 0.0, 0.0}, {0.0, -Y, 0.0}, {0.0, 0.0, Z}, {0.0, 0.0, -Z}
	};
	
	GLint tindices[8][3] = 
	{
		{0, 4, 1},
		{1, 4, 2},
		{2, 4, 3},
		{3, 4, 0},
		{0, 1, 5}, 
		{1, 2, 5}, 
		{2, 3, 5},
		{3, 0, 5} 

	};

	for (int i = 0; i < 6; i++)
		mVertices.push_back(Map<Vector3f>(vdata[i]));
	mIndices.push_back(new std::vector<int>);
	std::vector<int>& indices = *mIndices.back();
	for (int i = 0; i < 8; i++)
	{
		for (int k = 0; k < 3; k++)
			indices.push_back(tindices[i][k]);
	}
	mListIds.push_back(0);

	while(mIndices.size()<levels)
		_subdivide();
}

const std::vector<int>& IcoSphere::indices(int level) const
{
	while (level>=int(mIndices.size()))
		const_cast<IcoSphere*>(this)->_subdivide();
	return *mIndices[level];
}

void IcoSphere::_subdivide(void)
{
	cout<<"subdivide once"<<endl;
	typedef unsigned long long Key;
	std::map<Key,int> edgeMap;
	const std::vector<int>& indices = *mIndices.back();
	mIndices.push_back(new std::vector<int>);
	std::vector<int>& refinedIndices = *mIndices.back();
	int end = indices.size();
	for (int i=0; i<end; i+=3)
	{
    int ids0[3],  // indices of outer vertices
        ids1[3];  // indices of edge vertices
        for (int k=0; k<3; ++k)
        {
        	int k1 = (k+1)%3;
        	int e0 = indices[i+k];
        	int e1 = indices[i+k1];
        	ids0[k] = e0;
        	if (e1>e0)
        		std::swap(e0,e1);
        	Key edgeKey = Key(e0) | (Key(e1)<<32);
        	std::map<Key,int>::iterator it = edgeMap.find(edgeKey);
        	if (it==edgeMap.end())
        	{
        		ids1[k] = mVertices.size();
        		edgeMap[edgeKey] = ids1[k];
        		mVertices.push_back( (mVertices[e0]+mVertices[e1]).normalized()*r );
        	}
        	else
        		ids1[k] = it->second;
        }
        refinedIndices.push_back(ids0[0]); refinedIndices.push_back(ids1[0]); refinedIndices.push_back(ids1[2]);
        refinedIndices.push_back(ids0[1]); refinedIndices.push_back(ids1[1]); refinedIndices.push_back(ids1[0]);
        refinedIndices.push_back(ids0[2]); refinedIndices.push_back(ids1[2]); refinedIndices.push_back(ids1[1]);
        refinedIndices.push_back(ids1[0]); refinedIndices.push_back(ids1[1]); refinedIndices.push_back(ids1[2]);
    }
    mListIds.push_back(0);
}

IcoSphere::~IcoSphere()
{
	cout<<"delete icosphere"<<endl;
}
