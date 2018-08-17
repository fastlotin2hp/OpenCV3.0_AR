// SLAM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

//STL
#include <iostream> 
#include <string>   
#include <stdio.h>
//OpenCV
#include "opencv2\opencv.hpp"
//Own_Header
#include "RW_Obj.h"

using namespace std;
using namespace cv;

//---------------------------------- Global ---------------------------------
int camera_NB=0;

string file_Name         = "out_camera_data.xml";
string input_PointCloud  = "out.obj";
string known_Image       = "im6.png";


bool done = false;

//default image size
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;


const double akaze_thresh = 0.0008f; // AKAZE detection threshold
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const float nn_match_ratio = 0.60f;   // Nearest neighbor matching ratio
const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
vector<vector<Point2f> > akaze_points;
vector<vector<Point2f> > AKAZE_8_Points(vector<KeyPoint> kpts1,Mat desc1 ,Mat img1,Mat img2,Mat &akaze_Mask );

//---------------------------------- Main ---------------------------------
int main()
{
	//1-1 Initiation---Camera params
	FileStorage fs;
	fs.open(file_Name, FileStorage::READ);
	// read camera matrix and distortion coefficients from file
	Mat intrinsics, distortion;
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distortion;
	// close the input file
	fs.release();

	//1-2 Initiation---to read Point clouds
	RW_Obj obj;
	vector< vector< double > > pointcloud;

	cout<<" reading pointcloud "<<endl;
	obj.Read_Obj_File(input_PointCloud,pointcloud);
	cout<<" pointcloud loaded  "<<pointcloud.size() <<endl;

	double pointcloud_Scale=200.0;

	//1-3 Initiation---Known Image
	Mat img1 = imread(known_Image, IMREAD_GRAYSCALE);

	//1-4 Initiation---live scene (camera stream)
	VideoCapture capture;
	capture.open(camera_NB);
	//set the capture frame size
	capture.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

	//VideoWriter  video_out("out.avi",1,4,Size(FRAME_WIDTH,FRAME_HEIGHT));

	//1-5 Initiation---Preparing other variables 
	Mat webcamImage, gray, one;
	Mat rvec = Mat(Size(3,1), CV_64F);
	Mat tvec = Mat(Size(3,1), CV_64F);

	vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
	

	//1-6 Initiation---Feature Point preparation before get in to the loop
	vector<KeyPoint> kpts1;
	Mat desc1;

	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->setThreshold(akaze_thresh);
	akaze->detectAndCompute(img1, noArray(), kpts1, desc1);

	while(!done)
	{
		//2-1 Loop---Read current scene
		capture.read(webcamImage);
		cvtColor(webcamImage,gray,COLOR_BGR2GRAY);

		//2-2 Loop---to get matche pairs for solvePnP()
		Mat akaze_Mask;
		akaze_points=AKAZE_8_Points( kpts1, desc1 ,img1,gray,akaze_Mask );

		if (!akaze_Mask.empty())
		{
			imshow("akaze_Mask",akaze_Mask);
		}else
		{
			imshow("OpenCV Webcam", webcamImage);
			waitKey(10);
			continue;
		}

		if (akaze_points.empty())
		{
			imshow("OpenCV Webcam", webcamImage);
			waitKey(10);
			continue;
		}

		//2-3 Loop---Creating a Point Cloud for known image
		vector<Point3d> AKAZE_boardPoints;
		for (uint j=0; j<akaze_points.at(0).size(); j++)
		{
			AKAZE_boardPoints.push_back( Point3d( double(akaze_points.at(0).at(j).y), 
												  double(akaze_points.at(0).at(j).x), 0.0) );
		}

		//2-4 Loop---Creating a Point Cloud for known image
		//find the camera extrinsic parameters
		solvePnP( Mat(AKAZE_boardPoints), Mat(akaze_points.at(1)), intrinsics, distortion, rvec, tvec, false );
		//solvePnPRansac( Mat(AKAZE_boardPoints), Mat(akaze_points.at(1)), intrinsics, distortion, rvec, tvec, false );


		//2-5 Loop---project the known image onto the live scene 
		//           magic numbers again,do not have to look deep into
		vector<Point3d> framePoints_akaze;
		vector<Point3d> pointcloud_color_temp;
		for(uint i=0;i<pointcloud.size();i++)
		{
			double _k_Rows=akaze_Mask.rows/4;
			double _k_Cols=akaze_Mask.cols/4;
			double _pt_y=pointcloud.at(i).at(1);
			double _pt_x=pointcloud.at(i).at(0);
			int _y= (int) ((_pt_y+1.5)*_k_Rows);
			int _x= (int) ((_pt_x+1.5)*_k_Cols);
			
			//remove out liner
			if (_y>akaze_Mask.rows){continue;}
			if (_y<0){continue;}
			if (_x>akaze_Mask.cols){continue;}
			if (_x<0){continue;}

			uint _mask=akaze_Mask.at<uchar>(_y, _x);

			if( _mask< 10  )//CV_8U
			{
				framePoints_akaze.push_back( Point3d( pointcloud.at(i).at(1)*pointcloud_Scale+img1.rows/2+100,
													  pointcloud.at(i).at(0)*pointcloud_Scale+img1.cols/2,
													  pointcloud.at(i).at(2)*-pointcloud_Scale+250 ) );

				pointcloud_color_temp.push_back( Point3d( pointcloud.at(i).at(5),
														  pointcloud.at(i).at(4),
														  pointcloud.at(i).at(3) ) );
			}

		}

		if (framePoints_akaze.size()<8)
		{
			imshow("OpenCV Webcam", webcamImage);
			waitKey(10);
			continue;
		}

		projectPoints(framePoints_akaze, rvec, tvec, intrinsics, distortion, imageFramePoints );


		//2-6 Loop---draw the projected point cloud onto the live scene 

		for(uint i=6;i<imageFramePoints.size();i++){
			line(webcamImage, imageFramePoints.at(i),    imageFramePoints.at(i), 
											Scalar( pointcloud_color_temp.at(i).x,//blue 
													pointcloud_color_temp.at(i).y,//green
													pointcloud_color_temp.at(i).z), 1 );//red
		}

		//2-7 Loop---to update live scene 
		namedWindow("OpenCV Webcam", 0);
		imshow("OpenCV Webcam", webcamImage);

		//	video_out<<webcamImage;
		//2-7 Loop---press 'Esc' to live the loop 
		if( 27 ==waitKey(10)){done=true;}
	}
	return 0;
}










//To generate a match pairs for solvePnP() 
vector<vector<Point2f> > AKAZE_8_Points(vector<KeyPoint> _kpts1,Mat _desc1 ,Mat _img1,Mat _img2,Mat &_akaze_Mask )
{
	//1-1 Matches---to get match pairs
	vector<vector<Point2f> > _return_Points;//OutPut
	vector<KeyPoint>		 kpts2;
	Mat						 desc2;

	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->setThreshold(akaze_thresh);
	akaze->detectAndCompute(_img2, noArray(), kpts2, desc2);

	BFMatcher matcher(NORM_HAMMING2);
	vector< vector<DMatch> > nn_matches;
	matcher.knnMatch(_desc1, desc2, nn_matches, 2);

	vector<KeyPoint> matched1, matched2, inliers1, inliers2;
	vector<DMatch> good_matches;
	for(size_t i = 0; i < nn_matches.size(); i++) {
		DMatch first = nn_matches[i][0];
		float  dist1 = nn_matches[i][0].distance;
		float  dist2 = nn_matches[i][1].distance;

		if(dist1 < nn_match_ratio * dist2) {
			matched1.push_back(_kpts1[first.queryIdx]);
			matched2.push_back(kpts2[first.trainIdx]);
		}
	}

	for(unsigned i = 0; i < matched1.size(); i++) {

		int new_i = static_cast<int>(inliers1.size());
		inliers1.push_back(matched1[i]);
		inliers2.push_back(matched2[i]);
		good_matches.push_back(DMatch(new_i, new_i, 0));

	}

	//2-1 Homography---Preparing Raw matches for Homography func

	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for( uint i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( matched1[ good_matches[i].queryIdx ].pt );
		scene.push_back( matched2[ good_matches[i].trainIdx ].pt );
	}


	//2-2 Homography---to get Homography

	Mat H;

	if(obj.size() >= 4 && scene.size()>=4)
	{
		H = findHomography( obj, scene, RANSAC,ransac_thresh );
	}else{
		return _return_Points;
	}

	if (0==H.rows|| 0==H.cols)
	{
		return _return_Points;
	}


	//2-3 Homography---Preparing predictable Points for output to use PnP
	vector<Point2f> obj_corners(8);
	obj_corners[0] = cvPoint(0,0);                        obj_corners[1] = cvPoint( _img1.cols, 0 );
	obj_corners[2] = cvPoint( _img1.cols, _img1.rows );   obj_corners[3] = cvPoint( 0, _img1.rows );
	obj_corners[4] = cvPoint( _img1.cols/2, _img1.rows ); obj_corners[5] = cvPoint( _img1.cols/2, 0  );
	obj_corners[6] = cvPoint( 0, _img1.rows/2 );          obj_corners[7] = cvPoint(  0, _img1.rows/2  );

	//2-4 Homography---to get predictable Points on the other side
	vector<Point2f> scene_corners(8);
		
	try {
		perspectiveTransform( obj_corners, scene_corners, H);
	} catch ( std::exception& e) {
		return _return_Points;
	}

	//2-5 Homography---Averaging filter to aliviate the jitter
	float avg_k  = 0.6; 
	float avg_1_k= 1-avg_k;
	static vector<Point2f> scene_corners_avg(8);
	for (uint i=0;i<scene_corners_avg.size();i++)
	{
		scene_corners_avg.at(i).x = avg_1_k*scene_corners.at(i).x+avg_k*scene_corners_avg.at(i).x;
		scene_corners_avg.at(i).y = avg_1_k*scene_corners.at(i).y+avg_k*scene_corners_avg.at(i).y;
	}



	//3-1 Occlusion--To detect the position of the obstacle,we have a known image and its homograph
	//             --in the live scene,re-project it back to the known image and compare them
	//             --to find the area of the obstacle covered.(currently ,only white material works) 
	Mat H_hat = findHomography( scene_corners_avg, obj_corners, RANSAC,ransac_thresh );

	if (0==H_hat.rows|| 0==H_hat.cols)
	{
		return _return_Points;
	}

	Mat warpPerspective_img1=_img1;
	Mat warpPerspective_img2;

	warpPerspective(_img2, warpPerspective_img2,H_hat,_img1.size());

	resize(_img1,                warpPerspective_img1, Size(), 0.5, 0.5, INTER_LINEAR);
	resize(warpPerspective_img2, warpPerspective_img2, Size(), 0.5, 0.5, INTER_LINEAR);

	GaussianBlur(warpPerspective_img1,warpPerspective_img1,Size(15,15),10,10);
	GaussianBlur(warpPerspective_img2,warpPerspective_img2,Size(15,15),10,10);

	//a bit magic numbers here and it's not a generalized algorithm.
	Mat blur_img;
	blur_img=(warpPerspective_img2-warpPerspective_img1)-30;
	GaussianBlur(blur_img,blur_img,Size(15,15),10,10);

	_akaze_Mask=blur_img*10;

	_return_Points.push_back(obj_corners);
	_return_Points.push_back(scene_corners_avg);

	return _return_Points;
};