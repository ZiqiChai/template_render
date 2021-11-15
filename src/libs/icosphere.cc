#include "icosphere.h"
#include <GL/gl.h>
#include <map>
#include <iostream>

using namespace Eigen;
using namespace std;

//--------------------------------------------------------------------------------
// icosahedron data
//--------------------------------------------------------------------------------
#define OX .525731112119133606
#define OZ .850650808352039932
float X,Z;

//--------------------------------------------------------------------------------

IcoSphere::IcoSphere(float pr,unsigned int levels)
{
	// init with an icosahedron
	r = pr;
	X = r*OX;
	Z = r*OZ;
	GLfloat vdata[12][3] = {
		{-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z},
		{0.0, Z, X}, {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X},
		{Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0}
	};

	GLint tindices[20][3] = {
		{0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},
		{8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},
		{7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6},
		{6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };

		for (int i = 0; i < 12; i++)
			mVertices.push_back(Map<Vector3f>(vdata[i]));
		mIndices.push_back(new std::vector<int>);
		std::vector<int>& indices = *mIndices.back();
		for (int i = 0; i < 20; i++)
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
        		//mVertices.push_back( (mVertices[e0]+mVertices[e1]).normalized() );
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
