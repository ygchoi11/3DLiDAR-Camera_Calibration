#pragma once
#include "pch.h"
#include "calib.h"
#include "view.h"

void testBlensor()
{
	//parameter settings
	Args args;
	args.path_camera_img = "data/blensor/cameraImage.png";
	args.path_lidar_pc = "data/blensor/pointcloud_noisy_positional_0.07.pcd";
	args.path_blensor = "data/blensor/blensor.txt";
	args.inlier_distance = 0.1;
	args.score_method = MSC;
	args.min_plane_score = 30.;
	args.n_point_plane_subsample = 3;
	args.n_max_plane = 5;
	args.n_loop_plane_hypo = 100;
	args.n_loop_init_pose = 10;
	args.n_aug_sample = 10;
	args.n_max_refine_iteration = 100;
	args.refine_gamma = 5.;
	args.refine_init_lambda = 1e-3;
	args.refine_lambda_up_fac = 11.;
	args.refine_lambda_dn_fac = 9.;
	args.refine_min_gradient_norm_r = 1e-4;
	args.refine_min_gradient_norm_t = 1e-4;
	args.chessSize = Size2i(4, 4);
	args.chessOffset = Size2d(0.66666666, 0.66666666);
	args.chessGridLen = 0.33333333;

	//read blensor files
	Calibration calb;
	Blensor bls;
	Gt gt;
	readBlensorFiles(args, bls, calb);
	assignBlensorGt(args, bls, gt);

	//assign camera intrinsic parameter
	bls.camera_intrinsic.copyTo(calb.camera_intrinsic);

	//assign lidar intrinsic parameter for uv image
	bls.lidar_intrinsic.copyTo(calb.lidar_intrinsic);

	//estimate lidar initial pose
	findInitCoordinate(args, calb, false);

	//refine lidar pose
	refinement(args, calb);

	//assign chess board corners based on world coordinates of lidar pose estimation
	assignChessboardCornersIn3D(args, calb);

	//find chess board corners in camera
	findChessboardCorners(args, calb);

	//calculate camera pose & relative pose; pnp_method: { SOLVEPNP_ITERATIVE, SOLVEPNP_EPNP, SOLVEPNP_SQPNP }
	calcRelativePose(args, calb, SOLVEPNP_SQPNP);

	//calculate errors
	printError(calb, gt);

	//check gt in viewer
	checkPointCloudIn3D(calb, gt, 2);
	checkCameraGt(args, bls, calb, gt);

	//check algorithm in viewer
	checkChessboardCorners(args, calb);
	checkCameraPose(args, calb);
	checkLidarPose(args, calb, 2);
	checkRelativePoseL2C(calb, 20, false);
	checkRelativePoseL2C(calb, 20);
	checkRelativePoseC2L(calb);
	checkRelativePoseWithTarget(args, calb, 1);

	//compare algorithm with gt in viewer
	checkResultIn3D(calb, gt, 2, 1);
}

void findCameraIntrinsicInReal()
{
	//parameters
	Size chessSize(24, 12);
	float gridSize = 0.01;

	//read images
	vector<Mat> chessImg;
	for (int i = 0; i < 100; i++)
	{
		char path[256];
		sprintf_s(path, "data/realworld/camera_calibration/calib (%d).png", i);
		Mat img = imread(path);
		if (!img.empty()) chessImg.push_back(img);
		else break;
	}

	//find corners
	vector<vector<Point2f>> imgPoints;
	vector<vector<Point3f>> objPoints;
	for (int i = 0; i < chessImg.size(); i++)
	{
		Mat grayImg;
		cvtColor(chessImg[i], grayImg, COLOR_BGR2GRAY);

		vector<Point2f> corners;
		bool patternfound = findChessboardCorners(grayImg, chessSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << i << "-th image can not find chess corners" << endl;
			if (i == 0) return;
			else continue;
		}
		cornerSubPix(grayImg, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));

		vector<Point3f> w_corners;
		for (int iy = 0; iy < chessSize.height; iy++)
		{
			for (int ix = 0; ix < chessSize.width; ix++)
			{
				float x = ix * gridSize;
				float y = iy * gridSize;
				float z = 0;

				w_corners.push_back(Point3f(x, y, z));
			}
		}

		imgPoints.push_back(corners);
		objPoints.push_back(w_corners);

		Mat temp = chessImg[i].clone();
		drawChessboardCorners(temp, chessSize, Mat(corners), patternfound);
		imshow("check_corners", temp);
		waitKey(1);
	}

	//calibrate camera
	Mat Kmat, dvec;
	vector<Mat> rv, tv;
	calibrateCamera(objPoints, imgPoints, chessImg[0].size(), Kmat, dvec, rv, tv);

	//print output
	cout << "intrinsic: " << endl << Kmat << endl;
	cout << "distCoeffs: " << endl << dvec << endl;
}

void calibrateInReal()
{
	//parameter settings
	Args args;
	args.path_camera_img = "data/realworld/cameraImage.png";
	args.path_lidar_pc = "data/realworld/pointcloud.pcd";
	args.inlier_distance = 0.02;
	args.score_method = MSC;
	args.min_plane_score = 30.;
	args.n_point_plane_subsample = 3;
	args.n_max_plane = 5;
	args.n_loop_plane_hypo = 100;
	args.n_loop_init_pose = 10;
	args.n_aug_sample = 10;
	args.n_max_refine_iteration = 100;
	args.refine_gamma = 5.;
	args.refine_init_lambda = 1e-3;
	args.refine_lambda_up_fac = 11.;
	args.refine_lambda_dn_fac = 9.;
	args.refine_min_gradient_norm_r = 1e-4;
	args.refine_min_gradient_norm_t = 1e-4;
	args.chessSize = Size2i(4, 4);
	args.chessOffset = Size2d(0.073, 0.073);
	args.chessGridLen = 0.05;

	//read image and pointcloud
	Calibration calb;
	readData(args, calb);

	//assign camera intrinsic parameters
	calb.camera_intrinsic = (Mat_<double>(3, 3) << 1395.920847556212, 0, 960.3837076945695,	0, 1394.075193341523, 567.5264921622173, 0, 0, 1);
	Mat distCoeffs = (Mat_<double>(1, 5) << 0.1974509030043082, -0.6598610042798466, -0.000203963024743637, -0.001275173259955003, 0.5936230776713313);

	//assign lidar intrinsic parameter for uv image
	calb.lidar_intrinsic = calb.camera_intrinsic.clone();

	//undistort image
	Mat und_img;
	undistort(calb.cameraImg, und_img, calb.camera_intrinsic, distCoeffs);
	und_img.copyTo(calb.cameraImg);

	//estimate lidar initial pose
	findInitCoordinate(args, calb, true, 3.4);

	//refine lidar pose
	refinement(args, calb);

	//extra-step
	removeOutlierPointCloud2(args, calb);
	refinement(args, calb);

	//assign chess board corners based on world coordinates of lidar pose estimation
	assignChessboardCornersIn3D(args, calb);

	//find chess board corners in camera
	findChessboardCorners(args, calb);

	//calculate camera pose & relative pose; pnp_method: { SOLVEPNP_ITERATIVE, SOLVEPNP_EPNP, SOLVEPNP_SQPNP }
	calcRelativePose(args, calb, SOLVEPNP_SQPNP);

	//print output
	cout << "[l2c_rotation]" << endl << calb.l2c_Rmat << endl;
	cout << "[l2c_translation] (in meters)" << endl << calb.l2c_tvec << endl;

	//check pointcloud in viewer
	checkPointCloudIn3D(calb, Gt(), 2);

	//check algorithm in viewer
	checkChessboardCorners(args, calb);
	checkCameraPose(args, calb);
	checkLidarPose(args, calb, 0.5);
	checkRelativePoseL2C(calb, 3.4, false);
	checkRelativePoseL2C(calb, 3.4);
	checkRelativePoseC2L(calb);
	checkRelativePoseWithTarget(args, calb, 0.5);

	//compare algorithm with gt in viewer
	checkResultIn3D(calb, Gt(), 0, 0.5);
}

void calibrateInReal2()
{
	//parameter settings
	Args args;
	args.path_camera_img = "data/realworld/cameraImage_ud.png";
	args.path_lidar_pc = "data/realworld/pointcloud.pcd";
	args.inlier_distance = 0.02;
	args.score_method = MSC;
	args.min_plane_score = 30.;
	args.n_point_plane_subsample = 3;
	args.n_max_plane = 5;
	args.n_loop_plane_hypo = 100;
	args.n_loop_init_pose = 10;
	args.n_aug_sample = 10;
	args.n_max_refine_iteration = 100;
	args.refine_gamma = 5.;
	args.refine_init_lambda = 1e-3;
	args.refine_lambda_up_fac = 11.;
	args.refine_lambda_dn_fac = 9.;
	args.refine_min_gradient_norm_r = 1e-4;
	args.refine_min_gradient_norm_t = 1e-4;
	args.chessSize = Size2i(4, 4);
	args.chessOffset = Size2d(0.073, 0.073);
	args.chessGridLen = 0.05;

	//read image and pointcloud
	Calibration calb;
	readData(args, calb);

	//assign camera intrinsic parameters
	calb.camera_intrinsic = (Mat_<double>(3, 3) << 1395.920847556212, 0, 960.3837076945695, 0, 1394.075193341523, 567.5264921622173, 0, 0, 1);
	Mat distCoeffs = (Mat_<double>(1, 5) << 0.1974509030043082, -0.6598610042798466, -0.000203963024743637, -0.001275173259955003, 0.5936230776713313);

	//assign lidar intrinsic parameter for uv image
	calb.lidar_intrinsic = calb.camera_intrinsic.clone();

	//undistort image
	Mat und_img;
	undistort(calb.cameraImg, und_img, calb.camera_intrinsic, distCoeffs);
	und_img.copyTo(calb.cameraImg);

	//estimate lidar initial pose
	findInitCoordinate(args, calb, true, 3.4);

	//refine lidar pose
	refinement(args, calb);

	//extra-step
	removeOutlierPointCloud2(args, calb);
	refinement(args, calb);

	//assign chess board corners based on world coordinates of lidar pose estimation
	assignChessboardCornersIn3D(args, calb);

	//find chess board corners in camera
	findChessboardCorners(args, calb);

	//calculate camera pose & relative pose; pnp_method: { SOLVEPNP_ITERATIVE, SOLVEPNP_EPNP, SOLVEPNP_SQPNP }
	calcRelativePose(args, calb, SOLVEPNP_SQPNP);

	//print output
	cout << "[l2c_rotation]" << endl << calb.l2c_Rmat << endl;
	cout << "[l2c_translation] (in meters)" << endl << calb.l2c_tvec << endl;

	//check pointcloud in viewer
	checkPointCloudIn3D(calb, Gt(), 2);

	//check algorithm in viewer
	checkChessboardCorners(args, calb);
	checkCameraPose(args, calb);
	checkLidarPose(args, calb, 0.5);
	checkRelativePoseL2C(calb, 3.4, false);
	checkRelativePoseL2C(calb, 3.4);
	checkRelativePoseC2L(calb);
	checkRelativePoseWithTarget(args, calb, 0.5);

	//compare algorithm with gt in viewer
	checkResultIn3D(calb, Gt(), 0, 0.5);
}
