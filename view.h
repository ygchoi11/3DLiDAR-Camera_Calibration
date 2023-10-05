#pragma once
#include "pch.h"

void sortingPoints(Mat src_, Mat& dst_, Mat view_rotation_, Mat view_translation_)
{
	Mat P;
	calcRt(src_, P, view_rotation_, view_translation_);

	Mat indice;
	sortIdx(P.col(2), indice, SORT_EVERY_COLUMN + SORT_DESCENDING);

	dst_.release();
	for (int i = 0; i < indice.rows; i++) dst_.push_back(src_.row(indice.at<int>(i)));
}

void filterOutOfCamera(Mat src_, Mat& dst_, vector<Scalar>& colorV_, Mat img_, Mat camera_intrinsic_, Mat l2c_rotation_, Mat l2c_translation_)
{
	colorV_.clear();

	Mat pointCloudInCamera = src_ * l2c_rotation_.t();
	pointCloudInCamera += Mat::ones(Size(1, src_.rows), l2c_translation_.type()) * l2c_translation_;

	Mat uvPoints = (camera_intrinsic_ * pointCloudInCamera.t()).t();
	uvPoints.col(0) /= uvPoints.col(2);
	uvPoints.col(1) /= uvPoints.col(2);

	Mat filtered_pointClouds;
	for (int i = 0; i < uvPoints.rows; i++)
	{
		double u = uvPoints.row(i).at<double>(0);
		double v = uvPoints.row(i).at<double>(1);
		double z = uvPoints.row(i).at<double>(2);
		if (u >= 0 && u < img_.cols - 1 && v >= 0 && v < img_.rows - 1 && z > 0)
		{
			filtered_pointClouds.push_back(src_.row(i));

			double f_u = floor(u), c_u = ceil(u), f_v = floor(v), c_v = ceil(v);
			double w[4] = { c_v - v, v - f_v, u - f_u, c_u - u };
			Point2i idx[4] = { Point2i(f_u, c_v), Point2i(f_u, f_v), Point2i(c_u, f_v), Point2i(c_u, c_v) };
			Scalar colors[4] = {
				Scalar(img_.at<Vec3b>(idx[0].y, idx[0].x)[0], img_.at<Vec3b>(idx[0].y, idx[0].x)[1], img_.at<Vec3b>(idx[0].y, idx[0].x)[2]),
				Scalar(img_.at<Vec3b>(idx[1].y, idx[1].x)[0], img_.at<Vec3b>(idx[1].y, idx[1].x)[1], img_.at<Vec3b>(idx[1].y, idx[1].x)[2]),
				Scalar(img_.at<Vec3b>(idx[2].y, idx[2].x)[0], img_.at<Vec3b>(idx[2].y, idx[2].x)[1], img_.at<Vec3b>(idx[2].y, idx[2].x)[2]),
				Scalar(img_.at<Vec3b>(idx[3].y, idx[3].x)[0], img_.at<Vec3b>(idx[3].y, idx[3].x)[1], img_.at<Vec3b>(idx[3].y, idx[3].x)[2]) };

			Scalar color = (w[3] * w[1] * colors[0] + w[3] * w[0] * colors[1] + w[2] * w[0] * colors[2] + w[2] * w[1] * colors[3]);
			colorV_.push_back(color);
		}
	}

	filtered_pointClouds.copyTo(dst_);
}

void on_mouseEvent(int event_, int x_, int y_, int flags_, void* param_)
{
	VertualCameraInf* vcInf = (VertualCameraInf*)param_;

	if (event_ == EVENT_LBUTTONDOWN)
	{
		vcInf->MouseStartPointL.x = x_;
		vcInf->MouseStartPointL.y = y_;

		vcInf->rotation.copyTo(vcInf->MouseStartRotation);

	}
	else if (event_ == EVENT_MBUTTONDOWN)
	{
		vcInf->MouseStartPointR.x = x_;
		vcInf->MouseStartPointR.y = y_;

		vcInf->translation.copyTo(vcInf->MouseStartTranslation);
	}
	else if (event_ == EVENT_MOUSEWHEEL)
	{
		if (flags_ < 0)
		{
			vcInf->translation.at<double>(2) += vcInf->visualSensitivityZoom;
		}
		else
		{
			vcInf->translation.at<double>(2) -= vcInf->visualSensitivityZoom;
		}
	}

	if (flags_ == EVENT_LBUTTONDOWN)
	{
		double degreeX = -((double)x_ - (double)vcInf->MouseStartPointL.x) * vcInf->visualSensitivityAngle;
		double degreeY = ((double)y_ - (double)vcInf->MouseStartPointL.y) * vcInf->visualSensitivityAngle;
		double radianX = degreeX * ((double)CV_PI / (double)180);
		double radianY = degreeY * ((double)CV_PI / (double)180);

		Mat RodriguesR1 = (Mat_<double>(3, 1) << radianY, radianX, 0);
		Mat r1;
		Rodrigues(RodriguesR1, r1);
		vcInf->rotation = r1 * vcInf->MouseStartRotation;
	}
	else if (flags_ == EVENT_MBUTTONDOWN + 1)
	{
		double movementX = ((double)x_ - (double)vcInf->MouseStartPointR.x) * vcInf->visualSensitivityTranslation;
		double movementY = ((double)y_ - (double)vcInf->MouseStartPointR.y) * vcInf->visualSensitivityTranslation;

		vcInf->translation.at<double>(0) = vcInf->MouseStartTranslation.at<double>(0) + movementX;
		vcInf->translation.at<double>(1) = vcInf->MouseStartTranslation.at<double>(1) + movementY;
	}
}

void checkPointCloudIn3D(Calibration calb_, Gt gt_, double axis_len_ = 1)
{
	if (calb_.pointCloud.empty()) return;

	//initialize
	int axis_thickness = 3;
	int point_size = 1;
	Size dstSize = Size((int)(calb_.lidar_intrinsic.at<double>(2) * 2), (int)(calb_.lidar_intrinsic.at<double>(5) * 2));

	VertualCameraInf vcInf;
	string windowName = "check_lidar_gt";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, dstSize);
	moveWindow(windowName, 0, 0);
	bool pointFlag = true;
	bool gtFlag = true;

	Mat dst;
	Mat init_dst = Mat(dstSize, CV_8UC3);
	init_dst.setTo(57);
	string inf_text[7] = { "mouse left button: rotation", "mouse wheel button: translation", "mouse wheel scroll: zoom in/out",
		"key 'g': gt axes on/off", "key 'p': point cloud on/off", "key 'd': get back to initial position", "key 'Esc': exit" };
	for (int i = 0; i < 7; i++) putText(init_dst, inf_text[i], Point(10, 15 + 20 * i), 2, 0.6, Scalar(255, 255, 255), 1);

	while (1)
	{
		dst = init_dst.clone();

		if (pointFlag == true)
		{
			Mat points;
			filterNegativeDepth(calb_.pointCloud, points, vcInf.rotation, vcInf.translation);
			if (!points.empty())
			{
				calcRt(points, points, vcInf.rotation, vcInf.translation);

				Mat uv = (calb_.lidar_intrinsic * points.t()).t();
				Mat uv_z;
				for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
				uv_z = uv_z.t();
				uv /= uv_z;

				for (int i = 0; i < uv.rows; i++)
				{
					Point point((int)uv.at<double>(i, 0), (int)uv.at<double>(i, 1));

					circle(dst, point, point_size, Scalar(0, 255, 255), -1);
				}
			}
		}

		if (gtFlag == true && !gt_.lidar_tvec.empty() && !gt_.lidar_Rmat.empty())
		{
			Mat gtAxes;
			gtAxes.push_back(gt_.lidar_tvec);
			for (int i = 0; i < 3; i++) gtAxes.push_back(gt_.lidar_tvec + axis_len_ * gt_.lidar_Rmat.t().row(i));

			filterNegativeDepth(gtAxes, gtAxes, vcInf.rotation, vcInf.translation);
			if (!gtAxes.empty())
			{
				calcRt(gtAxes, gtAxes, vcInf.rotation, vcInf.translation);

				Mat uv = (calb_.lidar_intrinsic * gtAxes.t()).t();
				Mat uv_z;
				for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
				uv_z = uv_z.t();
				uv /= uv_z;

				for (int i = 1; i < uv.rows; i++)
				{
					Point origin((int)uv.at<double>(0, 0), (int)uv.at<double>(0, 1));
					Point point((int)uv.at<double>(i, 0), (int)uv.at<double>(i, 1));

					line(dst, origin, point, Scalar(0, 0, 255), axis_thickness);

					string axis_text;
					if (i == 1) axis_text = "x";
					else if (i == 2) axis_text = "y";
					else axis_text = "z";
					putText(dst, axis_text, point, 2, 1.2, Scalar(0, 255, 0), 2);
				}
			}
		}

		setMouseCallback(windowName, on_mouseEvent, (void*)&vcInf);

		imshow(windowName, dst);
		char key = waitKey(1);
		if (key == 27) break; //esc
		else if (key == 'g' || key == 'G') gtFlag = gtFlag == false ? true : false;
		else if (key == 'p' || key == 'P') pointFlag = pointFlag == false ? true : false;
		else if (key == 'd' || key == 'D')
		{
			vcInf.rotation = Mat::eye(Size(3, 3), CV_64FC1);
			vcInf.translation.setTo(0);
		}
	}
}

void checkCameraGt(Args args_, Blensor bls_, Calibration calb_, Gt gt_)
{
	//initialize
	int axis_thickness = 2;
	int point_size = 3;
	string windowName = "check_camera_gt";

	Mat dst;
	calb_.cameraImg.copyTo(dst);

	Point3i chessAxes[3];
	chessAxes[0] = Point3i(0, 1, 2);
	chessAxes[1] = Point3i(1, 2, 0);
	chessAxes[2] = Point3i(2, 0, 1);

	Mat corners3D;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < args_.chessSize.height; j++)
		{
			for (int k = 0; k < args_.chessSize.width; k++)
			{
				Mat p3d = Mat_<double>(1, 4);
				p3d.at<double>(0, chessAxes[i].x) = (double)k * (double)args_.chessGridLen + args_.chessOffset.width;
				p3d.at<double>(0, chessAxes[i].y) = (double)j * (double)args_.chessGridLen + args_.chessOffset.height;
				p3d.at<double>(0, chessAxes[i].z) = 0;
				p3d.at<double>(0, 3) = 1;

				corners3D.push_back(p3d);
			}
		}
	}

	Mat M;
	M.push_back(gt_.camera_Rmat.t());
	M.push_back(gt_.camera_tvec);
	M = M.t();

	Mat corners2D = (bls_.camera_intrinsic * M * corners3D.t()).t();;
	corners2D /= corners2D.col(2) * Mat::ones(Size(3, 1), corners2D.type());

	for (int i = 0; i < corners2D.rows; i++)
	{
		Point point((int)corners2D.at<double>(i, 0), (int)corners2D.at<double>(i, 1));

		circle(dst, point, point_size, Scalar(0, 0, 255), -1);

		char text[10];
		sprintf_s(text, "%d", i);

		putText(dst, text, point, 2, 0.3, Scalar(0, 255, 0));
	}

	for (int i = 1; i < gt_.axis_vectors2D.cols; i++)
	{
		Point origin((int)gt_.axis_vectors2D.at<Vec2d>(0)[0], (int)gt_.axis_vectors2D.at<Vec2d>(0)[1]);
		Point vector((int)gt_.axis_vectors2D.at<Vec2d>(i)[0], (int)gt_.axis_vectors2D.at<Vec2d>(i)[1]);

		line(dst, origin, vector, Scalar(0, 0, 255), axis_thickness);

		string axis_text;
		if (i == 1) axis_text = "x";
		else if (i == 2) axis_text = "y";
		else axis_text = "z";

		putText(dst, axis_text, vector, 2, 1.2, Scalar(0, 255, 0), 2);
	}

	imshow(windowName, dst);
	moveWindow(windowName, 0, 0);
	waitKey(0);
}

void checkChessboardCorners(Args args_, Calibration& calb_)
{
	//initialize
	int axis_thickness = 2;
	Mat dst = calb_.cameraImg.clone();
	string windowName = "check_chess_corners";

	//draw all ordered chess corners
	for (int i = 0; i < 3; i++)
	{
		int n_corners_per_plane = args_.chessSize.width * args_.chessSize.height;

		vector<Point2f> corners;
		corners.assign((Point2f*)calb_.chessCorners2D.datastart + i * n_corners_per_plane, (Point2f*)calb_.chessCorners2D.datastart + (i + 1) * n_corners_per_plane);

		drawChessboardCorners(dst, args_.chessSize, corners, true);
	}

	//draw all axes of 3D chess corners
	for (int i = 0; i < 3; i++)
	{
		int n_corners_per_plane = args_.chessSize.width * args_.chessSize.height;

		vector<Point2f> corners;
		corners.assign((Point2f*)calb_.chessCorners2D.datastart + i * n_corners_per_plane, (Point2f*)calb_.chessCorners2D.datastart + (i + 1) * n_corners_per_plane);

		Point2f v1, v2;
		v1 = (float)2 * (corners[args_.chessSize.width - 1] - corners[0]) + corners[0];
		v2 = (float)2 * (corners[args_.chessSize.width * (args_.chessSize.height - 1)] - corners[0]) + corners[0];

		line(dst, corners[0], v1, Scalar(0, 0, 255), axis_thickness);
		line(dst, corners[0], v2, Scalar(255, 0, 0), axis_thickness);

		string axis_text;
		if (i == 0) axis_text = "x";
		else if (i == 1) axis_text = "y";
		else axis_text = "z";

		putText(dst, axis_text, v1, 2, 1.2, Scalar(0, 255, 0), 2);
	}

	for (int i = 0; i < calb_.chessCorners2D.rows; i++)
	{
		char text[128];
		sprintf_s(text, "(%.2f, %.2f, %.2f)", calb_.chessCorners3D.at<Vec3f>(i)[0], calb_.chessCorners3D.at<Vec3f>(i)[1], calb_.chessCorners3D.at<Vec3f>(i)[2]);

		Point point = (Point)calb_.chessCorners2D.at<Vec2f>(i);
		putText(dst, text, point, 2, 0.2, Scalar(0, 0, 255));
	}

	imshow(windowName, dst);
	moveWindow(windowName, 0, 0);
	waitKey(0);
}

void checkCameraPose(Args args_, Calibration calb_)
{
	//initialize
	int axis_thickness = 2;
	int point_size = 5;
	Scalar color[3] = { Scalar(255, 0, 0), Scalar(0, 0, 255), Scalar(0, 255, 0) };
	string windowName = "check_camera_pose";

	Mat dst;
	calb_.cameraImg.copyTo(dst);

	Point3i chessAxes[3];
	chessAxes[0] = Point3i(0, 1, 2);
	chessAxes[1] = Point3i(1, 2, 0);
	chessAxes[2] = Point3i(2, 0, 1);

	Mat corners3D;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < args_.chessSize.height; j++)
		{
			for (int k = 0; k < args_.chessSize.width; k++)
			{
				Mat p3d = Mat_<double>(1, 4);
				p3d.at<double>(chessAxes[i].x) = (double)k * (double)args_.chessGridLen + args_.chessOffset.width;
				p3d.at<double>(chessAxes[i].y) = (double)j * (double)args_.chessGridLen + args_.chessOffset.height;
				p3d.at<double>(chessAxes[i].z) = 0;
				p3d.at<double>(3) = 1;

				corners3D.push_back(p3d);
			}
		}
	}

	Mat M;
	M.push_back(calb_.camera_Rmat.t());
	M.push_back(calb_.camera_tvec);
	M = M.t();

	Mat corners2D = (calb_.camera_intrinsic * M * corners3D.t()).t();
	for (int i = 0; i < 3; i++) corners2D.col(i) /= corners2D.col(2);

	for (int i = 0; i < 3; i++)
	{
		int n_corners_per_plane = args_.chessSize.width * args_.chessSize.height;

		Point2d v0(corners2D.row(i * n_corners_per_plane).at<double>(0), corners2D.row(i * n_corners_per_plane).at<double>(1));
		Point2d v1(corners2D.row(i * n_corners_per_plane + args_.chessSize.width - 1).at<double>(0),
			corners2D.row(i * n_corners_per_plane + args_.chessSize.width - 1).at<double>(1));
		Point2d v2(corners2D.row((i + 1) * n_corners_per_plane - args_.chessSize.width).at<double>(0),
			corners2D.row((i + 1) * n_corners_per_plane - args_.chessSize.width).at<double>(1));
		v1 = (double)2 * (v1 - v0) + v0;
		v2 = (double)2 * (v2 - v0) + v0;

		line(dst, v0, v1, Scalar(0, 0, 255), axis_thickness);
		line(dst, v0, v2, Scalar(255, 0, 0), axis_thickness);

		string axis_text;
		if (i == 0) axis_text = "x";
		else if (i == 1) axis_text = "y";
		else axis_text = "z";

		putText(dst, axis_text, v1, 2, 1.2, Scalar(0, 255, 0), 2);
	}

	for (int i = 0; i < corners2D.rows; i++)
	{
		Point point((int)corners2D.at<double>(i, 0), (int)corners2D.at<double>(i, 1));

		int n_corners_per_plane = args_.chessSize.width * args_.chessSize.height;
		int color_idx = i / n_corners_per_plane;
		circle(dst, point, point_size, color[color_idx], -1);

		char text[10];
		sprintf_s(text, "%d", i);

		putText(dst, text, point, 2, 0.4, Scalar(0, 255, 0));
	}

	imshow(windowName, dst);
	moveWindow(windowName, 0, 0);
	waitKey(0);
}

void checkLidarPose(Args args_, Calibration calb_, double axis_len_ = 1)
{
	//initialize
	int axis_thickness = 3;
	int point_size = 1;
	Scalar color[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };
	string windowName = "check_lidar_pose";
	Size dstSize = Size((int)(calb_.lidar_intrinsic.at<double>(2) * 2), (int)(calb_.lidar_intrinsic.at<double>(5) * 2));
	Mat dst = Mat(dstSize, CV_8UC3).setTo(57);

	Mat pL[3];
	for (int i = 0; i < 3; i++)
	{
		pL[i] = calb_.inlier_pointClouds[i].clone();
		if (!pL[i].empty())
		{
			Mat uv = (calb_.lidar_intrinsic * pL[i].t()).t();
			Mat uv_z;
			for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
			uv_z = uv_z.t();
			uv /= uv_z;

			for (int j = 0; j < uv.rows; j++)
			{
				Point point((int)uv.at<double>(j, 0), (int)uv.at<double>(j, 1));

				circle(dst, point, point_size, color[i], -1);
			}
		}
	}

	Mat M;
	M.push_back(calb_.lidar_Rmat.t());
	M.push_back(calb_.lidar_tvec);
	M = M.t();

	Mat corners3D = (Mat_<double>(4, 4) <<
		0, 0, 0, 1,
		axis_len_, 0, 0, 1,
		0, axis_len_, 0, 1,
		0, 0, axis_len_, 1);

	Mat corners2D = (calb_.lidar_intrinsic * M * corners3D.t()).t();
	for (int i = 0; i < 3; i++) corners2D.col(i) /= corners2D.col(2);

	for (int i = 1; i < corners2D.rows; i++)
	{
		Point origin((int)corners2D.at<double>(0, 0), (int)corners2D.at<double>(0, 1));
		Point point((int)corners2D.at<double>(i, 0), (int)corners2D.at<double>(i, 1));

		line(dst, origin, point, Scalar(0, 0, 255), axis_thickness);

		string axis_text;
		if (i == 1) axis_text = "x";
		else if (i == 2) axis_text = "y";
		else axis_text = "z";

		putText(dst, axis_text, point, 2, 1.2, Scalar(0, 255, 0), 2);
	}

	imshow(windowName, dst);
	moveWindow(windowName, 0, 0);
	waitKey(0);
}

void checkRelativePoseL2C(Calibration calb_, double dist_range_ = 20, bool calibrate_ = true)
{
	//initialize
	int point_size = 3;
	Mat pointCloudInCamera = calb_.pointCloud.clone();
	if (calibrate_)
	{
		Size dstSize = calb_.cameraImg.size();
		string windowName = "check_relative_pose_l2c (after)";
		namedWindow(windowName, WINDOW_NORMAL);
		resizeWindow(windowName, dstSize);
		moveWindow(windowName, 0, 0);
		string inf_text = "key '1', '2', '3': manual relative pose selection";
		string selection_text = "pose selection: automatic";

		while (1)
		{
			pointCloudInCamera = calb_.pointCloud * calb_.l2c_Rmat.t();
			pointCloudInCamera += Mat::ones(Size(1, calb_.pointCloud.rows), calb_.l2c_tvec.type()) * calb_.l2c_tvec;

			Mat dists, dist_colors;
			for (int i = 0; i < pointCloudInCamera.rows; i++) dists.push_back(norm(pointCloudInCamera.row(i)));
			dists *= 255 / dist_range_;
			threshold(dists, dists, 0, 0, THRESH_TOZERO);
			threshold(dists, dists, 255, 255, THRESH_TRUNC);
			dists.convertTo(dists, CV_8UC1);
			applyColorMap(dists, dist_colors, COLORMAP_RAINBOW);

			Mat uvPoints = (calb_.camera_intrinsic * pointCloudInCamera.t()).t();
			Mat uv_z;
			for (int i = 0; i < 3; i++) uv_z.push_back(uvPoints.col(2).t());
			uv_z = uv_z.t();
			uvPoints /= abs(uv_z);

			Mat dst = calb_.cameraImg.clone();
			for (int i = 0; i < uvPoints.rows; i++)
			{
				int axis_x = (int)uvPoints.at<double>(i, 0);
				int axis_y = (int)uvPoints.at<double>(i, 1);

				if (uvPoints.at<double>(i, 2) > 0) circle(dst, Point(axis_x, axis_y), point_size, Scalar(dist_colors.at<uchar>(i, 0), dist_colors.at<uchar>(i, 1), dist_colors.at<uchar>(i, 2)), -1);
			}

			putText(dst, inf_text, Point(10, 20), 2, 0.6, Scalar(255, 255, 255), 1);
			putText(dst, selection_text, Point(10, 40), 2, 0.6, Scalar(255, 255, 255), 1);
			imshow(windowName, dst);
			char key = waitKey(1);
			if (key == 27) //esc
			{
				calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
				calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
				break;
			}
			else if (key == '1')
			{
				selection_text = "pose selection: relative pose 1";
				calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
				calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
			}
			else if (key == '2')
			{
				selection_text = "pose selection: relative pose 2";
				calb_.l2c_three_Rmats[1].copyTo(calb_.l2c_Rmat);
				calb_.l2c_three_tvecs[1].copyTo(calb_.l2c_tvec);
			}
			else if (key == '3')
			{
				selection_text = "pose selection: relative pose 3";
				calb_.l2c_three_Rmats[2].copyTo(calb_.l2c_Rmat);
				calb_.l2c_three_tvecs[2].copyTo(calb_.l2c_tvec);
			}
		}
	}
	else
	{
		string windowName = "check_relative_pose_l2c (before)";

		Mat pointCloudInCamera = calb_.pointCloud.clone();

		Mat dists, dist_colors;
		for (int i = 0; i < pointCloudInCamera.rows; i++) dists.push_back(norm(pointCloudInCamera.row(i)));
		dists *= 255 / dist_range_;
		threshold(dists, dists, 0, 0, THRESH_TOZERO);
		threshold(dists, dists, 255, 255, THRESH_TRUNC);
		dists.convertTo(dists, CV_8UC1);
		applyColorMap(dists, dist_colors, COLORMAP_RAINBOW);

		Mat uvPoints = (calb_.camera_intrinsic * pointCloudInCamera.t()).t();
		Mat uv_z;
		for (int i = 0; i < 3; i++) uv_z.push_back(uvPoints.col(2).t());
		uv_z = uv_z.t();
		uvPoints /= abs(uv_z);

		Mat dst = calb_.cameraImg.clone();
		for (int i = 0; i < uvPoints.rows; i++)
		{
			int axis_x = (int)uvPoints.at<double>(i, 0);
			int axis_y = (int)uvPoints.at<double>(i, 1);

			if (uvPoints.at<double>(i, 2) > 0) circle(dst, Point(axis_x, axis_y), point_size, Scalar(dist_colors.at<uchar>(i, 0), dist_colors.at<uchar>(i, 1), dist_colors.at<uchar>(i, 2)), -1);
		}

		imshow(windowName, dst);
		moveWindow(windowName, 0, 0);
		waitKey(0);
	}
}

void checkRelativePoseC2L(Calibration calb_)
{
	//initialize
	int point_size = 3;
	Size dstSize = Size((int)(calb_.lidar_intrinsic.at<double>(2) * 2), (int)(calb_.lidar_intrinsic.at<double>(5) * 2));

	VertualCameraInf vcInf;
	string windowName = "check_relative_pose_c2l";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, dstSize);
	moveWindow(windowName, 0, 0);

	Mat dst;
	Mat init_dst = Mat(dstSize, CV_8UC3);
	init_dst.setTo(57);
	string inf_text[6] = { "mouse left button: rotation", "mouse wheel button: translation", "mouse wheel scroll: zoom in/out", "key 'd': get back to initial position", "kes 'Esc': exit", "key '1', '2', '3': manual relative pose selection" };
	for (int i = 0; i < 6; i++) putText(init_dst, inf_text[i], Point(10, 15 + 20 * i), 2, 0.6, Scalar(255, 255, 255), 1);
	string selection_text = "pose selection: automatic";

	while (1)
	{
		dst = init_dst.clone();

		Mat points = calb_.pointCloud.clone();
		filterNegativeDepth(points, points, vcInf.rotation, vcInf.translation);

		if (!points.empty())
		{
			sortingPoints(points, points, vcInf.rotation, vcInf.translation);

			Mat points_in_camera;
			vector<Scalar> point_color;
			filterOutOfCamera(points, points_in_camera, point_color, calb_.cameraImg, calb_.camera_intrinsic, calb_.l2c_Rmat, calb_.l2c_tvec);

			calcRt(points_in_camera, points_in_camera, vcInf.rotation, vcInf.translation);

			Mat uv = (calb_.lidar_intrinsic * points_in_camera.t()).t();
			Mat uv_z;
			for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
			uv_z = uv_z.t();
			uv /= uv_z;

			for (int j = 0; j < uv.rows; j++)
			{
				Point point((int)uv.at<double>(j, 0), (int)uv.at<double>(j, 1));

				circle(dst, point, point_size, point_color[j], -1);
			}
		}

		setMouseCallback(windowName, on_mouseEvent, (void*)&vcInf);

		putText(dst, selection_text, Point(10, 135), 2, 0.6, Scalar(255, 255, 255), 1);
		imshow(windowName, dst);
		char key = waitKey(1);
		if (key == 27) //esc
		{
			calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
			break;
		}
		else if (key == '1')
		{
			selection_text = "pose selection: relative pose 1";
			calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
		}
		else if (key == '2')
		{
			selection_text = "pose selection: relative pose 2";
			calb_.l2c_three_Rmats[1].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[1].copyTo(calb_.l2c_tvec);
		}
		else if (key == '3')
		{
			selection_text = "pose selection: relative pose 3";
			calb_.l2c_three_Rmats[2].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[2].copyTo(calb_.l2c_tvec);
		}
		else if (key == 'd' || key == 'D')
		{
			vcInf.rotation = Mat::eye(Size(3, 3), CV_64FC1);
			vcInf.translation.setTo(0);
		}
	}
}

void checkRelativePoseWithTarget(Args args_, Calibration calb_, double axis_len_ = 1)
{
	//initialize
	int axis_thickness = 2;
	int point_size = 3;
	Scalar color[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };

	Size dstSize = calb_.cameraImg.size();
	string windowName = "check_relative_pose_with_target";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, dstSize);
	moveWindow(windowName, 0, 0);
	string inf_text = "key '1', '2', '3': manual relative pose selection";
	string selection_text = "pose selection: automatic";

	while (1)
	{
		Mat dst = calb_.cameraImg.clone();

		for (int i = 0; i < 3; i++)
		{
			Mat pointCloudInCamera = calb_.inlier_pointClouds[i] * calb_.l2c_Rmat.t();
			pointCloudInCamera += Mat::ones(Size(1, calb_.inlier_pointClouds[i].rows), calb_.l2c_tvec.type()) * calb_.l2c_tvec;

			Mat uvPoints = (calb_.camera_intrinsic * pointCloudInCamera.t()).t();
			uvPoints.col(0) /= uvPoints.col(2);
			uvPoints.col(1) /= uvPoints.col(2);

			for (int j = 0; j < uvPoints.rows; j++)
			{
				int axis_x = (int)uvPoints.at<double>(j, 0);
				int axis_y = (int)uvPoints.at<double>(j, 1);

				if (uvPoints.at<double>(j, 2) > 0) circle(dst, Point(axis_x, axis_y), point_size, color[i], -1);
			}
		}

		Mat M;
		M.push_back(calb_.l2c_Rmat.t());
		M.push_back(calb_.l2c_tvec);
		M = M.t();

		Mat corners3D;
		corners3D.push_back(calb_.lidar_tvec);
		corners3D.push_back(calb_.lidar_tvec + axis_len_ * calb_.lidar_Rmat.col(0).t());
		corners3D.push_back(calb_.lidar_tvec + axis_len_ * calb_.lidar_Rmat.col(1).t());
		corners3D.push_back(calb_.lidar_tvec + axis_len_ * calb_.lidar_Rmat.col(2).t());
		corners3D = corners3D.t();
		corners3D.push_back(Mat::ones(Size(4, 1), corners3D.type()));
		corners3D = corners3D.t();

		Mat corners2D = (calb_.camera_intrinsic * M * corners3D.t()).t();
		for (int i = 0; i < 3; i++) corners2D.col(i) /= corners2D.col(2);

		for (int i = 1; i < corners2D.rows; i++)
		{
			Point origin((int)corners2D.at<double>(0, 0), (int)corners2D.at<double>(0, 1));
			Point point((int)corners2D.at<double>(i, 0), (int)corners2D.at<double>(i, 1));

			line(dst, origin, point, Scalar(0, 0, 255), axis_thickness);

			string axis_text;
			if (i == 1) axis_text = "x";
			else if (i == 2) axis_text = "y";
			else axis_text = "z";

			putText(dst, axis_text, point, 2, 1.2, Scalar(0, 255, 0), 2);
		}

		Point3i chessAxes[3];
		chessAxes[0] = Point3i(0, 1, 2);
		chessAxes[1] = Point3i(1, 2, 0);
		chessAxes[2] = Point3i(2, 0, 1);

		corners3D.release();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < args_.chessSize.height; j++)
			{
				for (int k = 0; k < args_.chessSize.width; k++)
				{
					Mat p3d = Mat_<double>(1, 4);
					p3d.at<double>(chessAxes[i].x) = (double)k * (double)args_.chessGridLen + args_.chessOffset.width;
					p3d.at<double>(chessAxes[i].y) = (double)j * (double)args_.chessGridLen + args_.chessOffset.height;
					p3d.at<double>(chessAxes[i].z) = 0;
					p3d.at<double>(3) = 1;

					corners3D.push_back(p3d);
				}
			}
		}

		M.release();
		M.push_back(calb_.camera_Rmat.t());
		M.push_back(calb_.camera_tvec);
		M = M.t();

		corners2D = (calb_.camera_intrinsic * M * corners3D.t()).t();
		for (int i = 0; i < 3; i++) corners2D.col(i) /= corners2D.col(2);

		for (int i = 0; i < 3; i++)
		{
			int n_corners_per_plane = args_.chessSize.width * args_.chessSize.height;

			Point2d v0(corners2D.row(i * n_corners_per_plane).at<double>(0), corners2D.row(i * n_corners_per_plane).at<double>(1));
			Point2d v1(corners2D.row(i * n_corners_per_plane + args_.chessSize.width - 1).at<double>(0),
				corners2D.row(i * n_corners_per_plane + args_.chessSize.width - 1).at<double>(1));
			Point2d v2(corners2D.row((i + 1) * n_corners_per_plane - args_.chessSize.width).at<double>(0),
				corners2D.row((i + 1) * n_corners_per_plane - args_.chessSize.width).at<double>(1));
			v1 = (double)2 * (v1 - v0) + v0;
			v2 = (double)2 * (v2 - v0) + v0;

			line(dst, v0, v1, Scalar(0, 0, 255), 2);
			line(dst, v0, v2, Scalar(255, 0, 0), 2);

			string axis_text;
			if (i == 0) axis_text = "x";
			else if (i == 1) axis_text = "y";
			else axis_text = "z";

			putText(dst, axis_text, v1, 2, 1.2, Scalar(0, 255, 0), 2);
		}

		putText(dst, inf_text, Point(10, 20), 2, 0.6, Scalar(255, 255, 255), 1);
		putText(dst, selection_text, Point(10, 40), 2, 0.6, Scalar(255, 255, 255), 1);
		imshow(windowName, dst);
		char key = waitKey(1);
		if (key == 27) //esc
		{
			calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
			break;
		}
		else if (key == '1')
		{
			selection_text = "pose selection: relative pose 1";
			calb_.l2c_three_Rmats[0].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[0].copyTo(calb_.l2c_tvec);
		}
		else if (key == '2')
		{
			selection_text = "pose selection: relative pose 2";
			calb_.l2c_three_Rmats[1].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[1].copyTo(calb_.l2c_tvec);
		}
		else if (key == '3')
		{
			selection_text = "pose selection: relative pose 3";
			calb_.l2c_three_Rmats[2].copyTo(calb_.l2c_Rmat);
			calb_.l2c_three_tvecs[2].copyTo(calb_.l2c_tvec);
		}
	}
}

void checkResultIn3D(Calibration calb_, Gt gt_, double gt_axis_len_ = 2, double pred_axis_len_ = 1)
{
	for (int i = 0; i < 3; i++) if (calb_.inlier_pointClouds[i].rows == 0) return;

	//initialize
	int gt_axis_thickness = 3;
	int pred_axis_thickness = 3;
	int point_size = 1;
	Scalar color[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };
	string windowName = "check_result_in_3D";
	Size dstSize = Size((int)(calb_.lidar_intrinsic.at<double>(2) * 2), (int)(calb_.lidar_intrinsic.at<double>(5) * 2));

	VertualCameraInf vcInf;
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, dstSize);
	moveWindow(windowName, 0, 0);
	bool pointFlag = true;
	bool axisFlag = true;
	bool gtFlag = true;

	Mat dst;
	Mat init_dst = Mat(dstSize, CV_8UC3);
	init_dst.setTo(57);
	string inf_text[8] = { "mouse left button: rotation", "mouse wheel button: translation", "mouse wheel scroll: zoom in/out", "key 'a': estimated axes (blue) on/off",
		"key 'g': gt axes (red) on/off", "key 'p': point cloud on/off", "key 'd': get back to initial position", "kes 'Esc': exit" };
	for (int i = 0; i < 8; i++) putText(init_dst, inf_text[i], Point(10, 15 + 20 * i), 2, 0.6, Scalar(255, 255, 255), 1);

	while (1)
	{
		dst = init_dst.clone();

		Mat pL[3];
		if (pointFlag == true)
		{
			for (int i = 0; i < 3; i++)
			{
				pL[i] = calb_.inlier_pointClouds[i].clone();
				filterNegativeDepth(pL[i], pL[i], vcInf.rotation, vcInf.translation);
				if (!pL[i].empty())
				{
					calcRt(pL[i], pL[i], vcInf.rotation, vcInf.translation);

					Mat uv = (calb_.lidar_intrinsic * pL[i].t()).t();
					Mat uv_z;
					for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
					uv_z = uv_z.t();
					uv /= uv_z;

					for (int j = 0; j < uv.rows; j++)
					{
						Point point((int)uv.at<double>(j, 0), (int)uv.at<double>(j, 1));

						circle(dst, point, 1, color[i], -1);
					}
				}
			}
		}

		if (gtFlag == true && !gt_.lidar_tvec.empty() && !gt_.lidar_Rmat.empty())
		{
			Mat gtAxes;
			gtAxes.push_back(gt_.lidar_tvec);
			for (int i = 0; i < 3; i++) gtAxes.push_back(gt_.lidar_tvec + gt_axis_len_ * gt_.lidar_Rmat.t().row(i));

			filterNegativeDepth(gtAxes, gtAxes, vcInf.rotation, vcInf.translation);
			if (!gtAxes.empty())
			{
				calcRt(gtAxes, gtAxes, vcInf.rotation, vcInf.translation);

				Mat uv = (calb_.lidar_intrinsic * gtAxes.t()).t();
				Mat uv_z;
				for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
				uv_z = uv_z.t();
				uv /= uv_z;

				for (int i = 1; i < uv.rows; i++)
				{
					Point origin((int)uv.at<double>(0, 0), (int)uv.at<double>(0, 1));
					Point point((int)uv.at<double>(i, 0), (int)uv.at<double>(i, 1));

					line(dst, origin, point, Scalar(0, 0, 255), gt_axis_thickness);
				}
			}
		}

		if (axisFlag == true)
		{
			Mat axes;
			axes.push_back(calb_.lidar_tvec);
			for (int i = 0; i < 3; i++) axes.push_back(calb_.lidar_tvec + pred_axis_len_ * calb_.lidar_Rmat.col(i).t());

			filterNegativeDepth(axes, axes, vcInf.rotation, vcInf.translation);
			if (!axes.empty())
			{
				calcRt(axes, axes, vcInf.rotation, vcInf.translation);

				Mat uv = (calb_.lidar_intrinsic * axes.t()).t();
				Mat uv_z;
				for (int i = 0; i < 3; i++) uv_z.push_back(uv.col(2).t());
				uv_z = uv_z.t();
				uv /= uv_z;

				for (int i = 1; i < uv.rows; i++)
				{
					Point origin((int)uv.at<double>(0, 0), (int)uv.at<double>(0, 1));
					Point point((int)uv.at<double>(i, 0), (int)uv.at<double>(i, 1));

					line(dst, origin, point, Scalar(255, 0, 0), pred_axis_thickness);
				}
			}
		}

		setMouseCallback(windowName, on_mouseEvent, (void*)&vcInf);

		imshow(windowName, dst);
		char key = waitKey(1);
		if (key == 27) break; //esc
		else if (key == 'a' || key == 'A') axisFlag = axisFlag == false ? true : false;
		else if (key == 'g' || key == 'G') gtFlag = gtFlag == false ? true : false;
		else if (key == 'p' || key == 'P') pointFlag = pointFlag == false ? true : false;
		else if (key == 'd' || key == 'D')
		{
			vcInf.rotation = Mat::eye(Size(3, 3), CV_64FC1);
			vcInf.translation.setTo(0);
		}
	}
}
