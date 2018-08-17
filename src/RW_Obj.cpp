
#include "stdafx.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>

#include <opencv/cv.h>

#include "RW_Obj.h"

using namespace std;
using namespace cv;

void RW_Obj::CreateColor_PointCloud( Mat& _mat_Depth, Mat& _mat_Color)
{
	const double max_z = 1.0e4;
	Vertex_Color _vertex_Color;
	for(int y = 0; y < _mat_Depth.rows; y++)
	{
		for(int x = 0; x < _mat_Depth.cols; x++)
		{
			Vec3f _point = _mat_Depth.at<Vec3f>(y, x);
			Vec3b _color = _mat_Color.at<Vec3b>(y, x);

			if(fabs(_point[2] - max_z) < FLT_EPSILON || fabs(_point[2]) > max_z) continue;
			
			_vertex_Color.position.x=_point[0];
			_vertex_Color.position.y=_point[1];
			_vertex_Color.position.z=_point[2];

			_vertex_Color.color.R=_color[2];
			_vertex_Color.color.G=_color[1];
			_vertex_Color.color.B=_color[0];

			vertexList.push_back(_vertex_Color);
		}
	}

}

vector< double >   RW_Obj::processVertex(std::string line)
{
	map< int,std::string > vertexString;
	int x = 0;

	for (uint i = 2; i <= line.size(); i++)
	{
		if ( line[i] == ' ')
		{
			x++;
		}
		else
		{
			// add to same cord
			vertexString[x]+=line[i];
		}
	}

	vector< double > _vertex_Color;
	if( (double)atof(vertexString.at(2).c_str())>0)
	{
		for(uint i=0;i<vertexString.size();i++){
			_vertex_Color.push_back( (double)atof(vertexString.at(i).c_str()) );
		}
	}

	return _vertex_Color;
}


int RW_Obj::Read_Obj_File(string File_Path_Name,vector< vector< double > > &_vertexList)
{
	ifstream inFile (File_Path_Name.c_str());
	string currentLine;

	if ( inFile.is_open())
	{
		// While the file is open, we read the info and construct triangles
		while ( getline(inFile, currentLine) )
		{
			//       	Read vertices into "vertexList" w/wo color
			if (currentLine[0] == 'v' && currentLine[1] == ' ' )
			{
				_vertexList.push_back(processVertex(currentLine));//read the position of the point
			}
		
		}
		inFile.close();
	}else
	{
			return 0;
	}

	return 1;
}


int RW_Obj::Write_Obj_File(string File_Path_Name)
{
    ofstream outfile (File_Path_Name.c_str());
    if (outfile.is_open())
    {

   	 for(uint i=0;i<vertexList.size();i++)
   	 {
   		outfile << "v";

   		outfile <<" "<<vertexList.at(i).position.x<<" "<<vertexList.at(i).position.y<<" "<<vertexList.at(i).position.z;

		outfile <<" "<<vertexList.at(i).color.R<<" "<<vertexList.at(i).color.G<<" "<<vertexList.at(i).color.B;

   		outfile << endl;
   	 }

   	 outfile.close();
    }
}



