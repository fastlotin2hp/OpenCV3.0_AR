#ifndef _RW_OBJ_H_
#define _RW_OBJ_H_


#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <opencv/cv.h>


using namespace std;
using namespace cv;

class Disparity_Map_OpenGL;

class RW_Obj
{
	friend Disparity_Map_OpenGL;
	private:
			typedef  struct _vertex
			{
				double x;
				double y;
				double z;
			} Vertex;//Position
			typedef  struct _color
			{
				double R;
				double G;
				double B;
			} Color;//Position

			typedef  struct _vertex_Color
			{
				Vertex			  	 position;
				Color                color;
			} Vertex_Color;

			vector< Vertex_Color >         vertexList;//vertices
	public:

			void 				CreateColor_PointCloud(Mat& _mat_Depth, Mat& _mat_Color);
			vector< double >    processVertex(std::string line);
			int					Read_Obj_File (string File_Path_Name,vector< vector< double > >  &_vertexList);
			int 				Write_Obj_File(string File_Path_Name);

};


#endif // _RW_OBJ_H_
