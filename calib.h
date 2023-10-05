#pragma once
#include "pch.h"
#include <fstream>

vector<int> generateRandVal(int start_, int end_, int n_)
{
	int size = end_ - start_ + 1;
	vector<int> dst;
	if (n_ > size)
	{
		cout << "input n must be less than or equal to range between start to end" << endl;
		return dst;
	}

	if (0.2 * size > n_)
	{
		for (int i = 0; i < n_; i++)
		{
			int rand_idx = rand() % size;
			for (int j = i - 1; j >= 0; j--) if (dst[j] == rand_idx) rand_idx = -1;
			if (rand_idx != -1) dst.push_back(rand_idx + start_);
			else i -= 1;
		}
	}
	else
	{
		vector<int> list;
		for (int i = 0; i < size; i++) list.push_back(i + start_);
		random_shuffle(list.begin(), list.end());
		for (int i = 0; i < n_; i++) dst.push_back(list[i]);
	}

	return dst;
}

double degreeToRadian(double angle_)
{
	return angle_ * ((double)CV_PI / (double)180);
}

double radianToDegree(double radian_)
{
	return radian_ * ((double)180 / (double)CV_PI);
}

double getRotationError(Mat const& r1_, Mat const& r2_)
{
	double traceVal = trace(r1_.t() * r2_)[0];
	traceVal = MAX(-1., MIN(3., traceVal));

	return acos((traceVal - 1.) / 2.);
}

void printError(Calibration const& calb_, Gt const& gt_)
{
	Mat error_tvec = gt_.l2c_tvec - calb_.l2c_tvec;
	Mat error_Rmat = gt_.l2c_Rmat.t() * calb_.l2c_Rmat;
	double error_distance = norm(error_tvec);
	double error_rotation_radian = getRotationError(gt_.l2c_Rmat, calb_.l2c_Rmat);
	double error_rotation_degree = radianToDegree(error_rotation_radian);

	cout << "<<Results>>" << endl;
	cout << "[gt_translation] (in meters)" << endl << gt_.l2c_tvec << endl;
	cout << "[pred_translation] (in meters)" << endl << calb_.l2c_tvec << endl;
	cout << "[gt_rotation]" << endl << gt_.l2c_Rmat << endl;
	cout << "[pred_rotation]" << endl << calb_.l2c_Rmat << endl;
	cout << endl;
	cout << "<<Errors>>" << endl;
	cout << "[error_translation] (in meters)" << endl << error_tvec << endl;
	cout << "[error_rotation]" << endl << error_Rmat << endl;
	cout << "[error_distance] (in meters)" << endl << error_distance << endl;
	cout << "[error_angle] (in degrees/radians)" << endl << error_rotation_degree << ", " << error_rotation_radian << endl;
	cout << endl;
}

void rvec2quat(Mat const& r_, Mat& q_)
{
	Mat dst;
	double theta = norm(r_);

	dst = sin(theta / 2.) * (r_ / theta);
	dst.push_back(cos(theta / 2.));
	dst /= norm(dst);
	dst.copyTo(q_);
}

void quat2rvec(Mat const& q_, Mat& r_)
{
	double q0 = q_.at<double>(0);
	double q1 = q_.at<double>(1);
	double q2 = q_.at<double>(2);
	double w = q_.at<double>(3);

	Mat rvec = (Mat_<double>(3, 1) << q0, q1, q2);
	double r_norm = norm(rvec);

	Mat dst = (2. / r_norm) * atan(r_norm / w) * rvec;
	dst.copyTo(r_);
}

Mat wMean(Mat const& src_, Mat const& w_)
{
	int paramDim = src_.cols;
	Mat w = w_ / sum(w_)[0];

	Mat dst;
	for (int i = 0; i < paramDim; i++) dst.push_back(sum(src_.col(i).mul(w))[0]);

	return dst;
}

void zyxEulerToRotation(Mat const& src_, Mat& dst_)
{
	Mat src_x, src_y, src_z;

	src_x = (Mat_<double>(1, 3) << src_.at<double>(0), 0, 0);
	src_y = (Mat_<double>(1, 3) << 0, src_.at<double>(1), 0);
	src_z = (Mat_<double>(1, 3) << 0, 0, src_.at<double>(2));

	Rodrigues(src_x, src_x);
	Rodrigues(src_y, src_y);
	Rodrigues(src_z, src_z);

	dst_ = src_z * src_y * src_x;
}

void rotationToZyxEuler(Mat const& src_, Mat& dst_)
{
	if (abs(src_.at<double>(2, 0)) != 1)
	{
		Mat xyz[2];
		for (int i = 0; i < 2; i++) xyz[i] = Mat_<double>(1, 3).setTo(0);

		xyz[0].at<double>(1) = -asin(src_.at<double>(2, 0));
		xyz[0].at<double>(0) = atan2(src_.at<double>(2, 1) / cos(xyz[0].at<double>(1)), src_.at<double>(2, 2) / cos(xyz[0].at<double>(1)));
		xyz[0].at<double>(2) = atan2(src_.at<double>(1, 0) / cos(xyz[0].at<double>(1)), src_.at<double>(0, 0) / cos(xyz[0].at<double>(1)));
		xyz[1].at<double>(1) = CV_PI - xyz[0].at<double>(1);
		xyz[1].at<double>(0) = atan2(src_.at<double>(2, 1) / cos(xyz[1].at<double>(1)), src_.at<double>(2, 2) / cos(xyz[1].at<double>(1)));
		xyz[1].at<double>(2) = atan2(src_.at<double>(1, 0) / cos(xyz[1].at<double>(1)), src_.at<double>(0, 0) / cos(xyz[1].at<double>(1)));

		double xyz_scale[2];
		for (int i = 0; i < 2; i++) xyz_scale[i] = pow(xyz[i].at<double>(0), 2) + pow(xyz[i].at<double>(1), 2) + pow(xyz[i].at<double>(2), 2);

		if (xyz_scale[0] < xyz_scale[1]) dst_ = xyz[0].clone();
		else dst_ = xyz[1].clone();
	}
	else
	{
		Mat xyz = Mat_<double>(1, 3).setTo(0);
		xyz.at<double>(2) = 0;
		if (src_.at<double>(2, 0) == -1)
		{
			xyz.at<double>(1) = CV_PI / 2;
			xyz.at<double>(0) = xyz.at<double>(2) + atan2(src_.at<double>(0, 1), src_.at<double>(0, 2));
		}
		else
		{
			xyz.at<double>(1) = -CV_PI / 2;
			xyz.at<double>(0) = -xyz.at<double>(2) + atan2(-src_.at<double>(0, 1), -src_.at<double>(0, 2));
		}

		dst_ = xyz.clone();
	}
}

bool checkPointInROI(Rect const& r_, Point const& p_)
{
	return (r_.tl().x <= p_.x) && (r_.br().x >= p_.x) && (r_.tl().y <= p_.y) && (r_.br().y >= p_.y);
}

void calcRt(Mat const& src_, Mat& dst_, Mat const& rotation_, Mat const& translation_)
{
	if (src_.empty()) return;

	Mat P;
	src_.copyTo(P);
	P(Rect(0, 0, 3, P.rows)) = (rotation_ * P(Rect(0, 0, 3, P.rows)).t() + translation_ * Mat::ones(Size(P.rows, 1), translation_.type())).t();

	P.copyTo(dst_);
}

void filterNegativeDepth(Mat const& src_, Mat& dst_, Mat const& view_rotation_, Mat const& view_translation_)
{
	Mat P;
	calcRt(src_, P, view_rotation_, view_translation_);

	Mat filtered_pointClouds;
	for (int i = 0; i < src_.rows; i++)
	{
		float z = P.at<double>(i, 2);
		if (z > 0)
		{
			filtered_pointClouds.push_back(src_.row(i));
		}
	}

	filtered_pointClouds.copyTo(dst_);
}

void project3DTo2D(Calibration& calb_, Mat& points_, Mat& dists_)
{
	dists_.release();
	filterNegativeDepth(calb_.pointCloud, points_, Mat::eye(Size(3, 3), calb_.pointCloud.type()), Mat::zeros(Size(1, 3), calb_.pointCloud.type()));

	for (int i = 0; i < points_.rows; i++)
	{
		Mat d = Mat_<double>(1, 1) << norm(points_.row(i));
		dists_.push_back(d);
	}

	calb_.uvPoints = points_ * calb_.lidar_intrinsic.t();
	for (int i = 0; i < 3; i++) calb_.uvPoints.col(i) /= calb_.uvPoints.col(2);
}

void readBlensorFiles(Args const& args_, Blensor& bls_, Calibration& calb_)
{
	//read images
	calb_.cameraImg = imread(args_.path_camera_img);

	//read blender data
	FILE* fblensor;
	if (!fopen_s(&fblensor, args_.path_blensor.c_str(), "r"))
	{
		bls_.object_scale = Mat_<double>(1, 3);
		bls_.object_location = Mat_<double>(1, 3);
		bls_.object_rotation = Mat_<double>(1, 3);
		bls_.lidar_intrinsic = Mat_<double>(3, 3);
		bls_.lidar_location = Mat_<double>(1, 3);
		bls_.lidar_default_rotation = Mat_<double>(1, 3);
		bls_.lidar_rotation = Mat_<double>(1, 3);
		bls_.camera_intrinsic = Mat_<double>(3, 3);
		bls_.camera_location = Mat_<double>(1, 3);
		bls_.camera_default_rotation = Mat_<double>(1, 3);
		bls_.camera_rotation = Mat_<double>(1, 3);

		fscanf_s(fblensor, "object_scale: %lf, %lf, %lf \n", &bls_.object_scale.at<double>(0, 0), &bls_.object_scale.at<double>(0, 1), &bls_.object_scale.at<double>(0, 2));
		fscanf_s(fblensor, "object_location: %lf, %lf, %lf \n", &bls_.object_location.at<double>(0, 0), &bls_.object_location.at<double>(0, 1), &bls_.object_location.at<double>(0, 2));
		fscanf_s(fblensor, "object_rotation: %lf, %lf, %lf \n", &bls_.object_rotation.at<double>(0, 0), &bls_.object_rotation.at<double>(0, 1), &bls_.object_rotation.at<double>(0, 2));

		fscanf_s(fblensor, "camera_intrinsic: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf \n",
			&bls_.camera_intrinsic.at<double>(0, 0), &bls_.camera_intrinsic.at<double>(0, 1), &bls_.camera_intrinsic.at<double>(0, 2),
			&bls_.camera_intrinsic.at<double>(1, 0), &bls_.camera_intrinsic.at<double>(1, 1), &bls_.camera_intrinsic.at<double>(1, 2),
			&bls_.camera_intrinsic.at<double>(2, 0), &bls_.camera_intrinsic.at<double>(2, 1), &bls_.camera_intrinsic.at<double>(2, 2));
		fscanf_s(fblensor, "camera_location: %lf, %lf, %lf \n", &bls_.camera_location.at<double>(0, 0), &bls_.camera_location.at<double>(0, 1), &bls_.camera_location.at<double>(0, 2));
		fscanf_s(fblensor, "camera_default_rotation: %lf, %lf, %lf \n", &bls_.camera_default_rotation.at<double>(0, 0), &bls_.camera_default_rotation.at<double>(0, 1), &bls_.camera_default_rotation.at<double>(0, 2));
		fscanf_s(fblensor, "camera_rotation: %lf, %lf, %lf \n", &bls_.camera_rotation.at<double>(0, 0), &bls_.camera_rotation.at<double>(0, 1), &bls_.camera_rotation.at<double>(0, 2));

		fscanf_s(fblensor, "lidar_intrinsic: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf \n",
			&bls_.lidar_intrinsic.at<double>(0, 0), &bls_.lidar_intrinsic.at<double>(0, 1), &bls_.lidar_intrinsic.at<double>(0, 2),
			&bls_.lidar_intrinsic.at<double>(1, 0), &bls_.lidar_intrinsic.at<double>(1, 1), &bls_.lidar_intrinsic.at<double>(1, 2),
			&bls_.lidar_intrinsic.at<double>(2, 0), &bls_.lidar_intrinsic.at<double>(2, 1), &bls_.lidar_intrinsic.at<double>(2, 2));
		fscanf_s(fblensor, "lidar_location: %lf, %lf, %lf \n", &bls_.lidar_location.at<double>(0, 0), &bls_.lidar_location.at<double>(0, 1), &bls_.lidar_location.at<double>(0, 2));
		fscanf_s(fblensor, "lidar_default_rotation: %lf, %lf, %lf \n", &bls_.lidar_default_rotation.at<double>(0, 0), &bls_.lidar_default_rotation.at<double>(0, 1), &bls_.lidar_default_rotation.at<double>(0, 2));
		fscanf_s(fblensor, "lidar_rotation: %lf, %lf, %lf \n", &bls_.lidar_rotation.at<double>(0, 0), &bls_.lidar_rotation.at<double>(0, 1), &bls_.lidar_rotation.at<double>(0, 2));
		fclose(fblensor);
	}

	//read point cloud data
	calb_.pointCloud.release();

	fstream fs_lidar;
	fs_lidar.open(args_.path_lidar_pc, ios::in);

	string str_buf;
	for (int i = 0; i < 11; i++) getline(fs_lidar, str_buf); //skip 11-lines
	Mat point(Size(3, 1), CV_64FC1);
	while (1)
	{
		getline(fs_lidar, str_buf);
		if (fs_lidar.eof()) break;

		istringstream sb(str_buf);
		string sub_str_buf;
		for (int i = 0; i < 3; i++)
		{
			getline(sb, sub_str_buf, ' ');
			point.at<double>(i) = stof(sub_str_buf);
		}
		calb_.pointCloud.push_back(point);
	}

	//align point cloud data
	Mat lidar_default_Rmat;
	zyxEulerToRotation(bls_.lidar_default_rotation, lidar_default_Rmat);
	calb_.pointCloud = calb_.pointCloud * lidar_default_Rmat.t();

	//close file
	fs_lidar.close();
}

void readData(Args const& args_, Calibration& calb_)
{
	//read images
	calb_.cameraImg = imread(args_.path_camera_img);

	//read point cloud data
	calb_.pointCloud.release();

	fstream fs_lidar;
	fs_lidar.open(args_.path_lidar_pc, ios::in);

	string str_buf;
	for (int i = 0; i < 11; i++) getline(fs_lidar, str_buf); //skip 11-lines
	Mat point(Size(3, 1), CV_64FC1);
	while (1)
	{
		getline(fs_lidar, str_buf);
		if (fs_lidar.eof()) break;

		istringstream sb(str_buf);
		string sub_str_buf;
		for (int i = 0; i < 3; i++)
		{
			getline(sb, sub_str_buf, ' ');
			point.at<double>(i) = stof(sub_str_buf);
		}
		calb_.pointCloud.push_back(point);
	}

	//close file
	fs_lidar.close();
}

void assignBlensorGt(Args const& args_, Blensor const& bls_, Gt& gt_)
{
	//set new world coordinate in a corner of object
	//gt location and rotation are based on new world coordinate

	//find new world coordinate origin point
	//    _________
	//   /        /
	//  /        /|
	// |--------| |
	// |    o   | |
	// |        | |
	// ---------*/

	Mat object_origin_in_object = Mat(bls_.object_scale.size(), bls_.object_scale.type());
	object_origin_in_object.at<double>(0, 0) = -bls_.object_scale.at<double>(0, 0) / (double)2;
	object_origin_in_object.at<double>(0, 1) = -bls_.object_scale.at<double>(0, 1) / (double)2;
	object_origin_in_object.at<double>(0, 2) = -bls_.object_scale.at<double>(0, 2) / (double)2;

	Mat object_rotation_3x3, camera_default_rotation_3x3, camera_rotation_3x3, lidar_default_rotation_3x3, lidar_rotation_3x3;
	zyxEulerToRotation(bls_.object_rotation, object_rotation_3x3);
	zyxEulerToRotation(bls_.camera_default_rotation, camera_default_rotation_3x3);
	zyxEulerToRotation(bls_.camera_rotation, camera_rotation_3x3);
	zyxEulerToRotation(bls_.lidar_default_rotation, lidar_default_rotation_3x3);
	zyxEulerToRotation(bls_.lidar_rotation, lidar_rotation_3x3);

	Mat object_origin_in_world = bls_.object_location + (object_rotation_3x3 * object_origin_in_object.t()).t();
	Mat camera_origin_in_world = bls_.camera_location;
	Mat lidar_origin_in_world = bls_.lidar_location;

	Mat world_origin_in_camera = (camera_default_rotation_3x3 * camera_rotation_3x3.t() * camera_origin_in_world.t()).t();
	Mat world_origin_in_lidar = (lidar_default_rotation_3x3 * lidar_rotation_3x3.t() * lidar_origin_in_world.t()).t();

	Mat object_origin_in_camera = (camera_default_rotation_3x3 * camera_rotation_3x3.t() * (object_origin_in_world - camera_origin_in_world).t()).t();
	Mat object_origin_in_lidar = (lidar_default_rotation_3x3 * lidar_rotation_3x3.t() * (object_origin_in_world - lidar_origin_in_world).t()).t();

	gt_.camera_tvec = object_origin_in_camera;
	gt_.lidar_tvec = object_origin_in_lidar;

	gt_.camera_Rmat = camera_default_rotation_3x3 * camera_rotation_3x3.t() * object_rotation_3x3;
	gt_.lidar_Rmat = lidar_default_rotation_3x3 * lidar_rotation_3x3.t() * object_rotation_3x3;

	gt_.l2c_tvec = (-gt_.camera_Rmat * gt_.lidar_Rmat.t() * gt_.lidar_tvec.t() + gt_.camera_tvec.t()).t();
	gt_.l2c_Rmat = gt_.camera_Rmat * gt_.lidar_Rmat.t();

	//calculate axis-vectors of gt
	Mat corners3D = (Mat_<double>(4, 4) <<
		0, 0, 0, 1,
		1, 0, 0, 1,
		0, 1, 0, 1,
		0, 0, 1, 1);

	Mat M;
	M.push_back(gt_.camera_Rmat.t());
	M.push_back(gt_.camera_tvec);
	M = M.t();

	Mat corners2D = (bls_.camera_intrinsic * M * corners3D.t()).t();
	for (int i = 0; i < 3; i++) corners2D.col(i) /= corners2D.col(2);

	gt_.axis_vectors2D = Mat_<Vec2d>(1, 4);
	for (int i = 0; i < 4; i++) gt_.axis_vectors2D.at<Vec2d>(i) = Point2d(corners2D.at<double>(i, 0), corners2D.at<double>(i, 1));
}

Mat getPointCloudInROI(Calibration& calb_, double dist_range_)
{
	//calculate uv points & distance color
	Mat points, dists, dist_colors;
	project3DTo2D(calb_, points, dists);
	dists *= 255 / dist_range_;
	threshold(dists, dists, 0, 0, THRESH_TOZERO);
	threshold(dists, dists, 255, 255, THRESH_TRUNC);
	dists.convertTo(dists, CV_8UC1);
	applyColorMap(dists, dist_colors, COLORMAP_RAINBOW);

	//get uv image
	calb_.uvImg = Mat::zeros(Size((int)(calb_.lidar_intrinsic.at<double>(2) * 2), (int)(calb_.lidar_intrinsic.at<double>(5) * 2)), CV_8UC3);
	for (int i = 0; i < calb_.uvPoints.rows; i++)
	{
		Point point((int)calb_.uvPoints.at<double>(i, 0), (int)calb_.uvPoints.at<double>(i, 1));

		if (dists.at<uchar>(i) != 0) circle(calb_.uvImg, point, 3, Scalar(dist_colors.at<uchar>(i, 0), dist_colors.at<uchar>(i, 1), dist_colors.at<uchar>(i, 2)), -1);
	}

	//select ROI
	Rect roi = selectROI(calb_.uvImg);

	//select points in RoI
	Mat pcInROI;
	for (int i = 0; i < calb_.uvPoints.rows; i++)
	{
		Point p(calb_.uvPoints.at<double>(i, 0), calb_.uvPoints.at<double>(i, 1));
		if (checkPointInROI(roi, p) && dists.at<uchar>(i) != 0) pcInROI.push_back(points.row(i));
	}

	return pcInROI;
}

void fitPlanes(Mat const& src_, Mat& dst_, Mat const& pointClouds_, double inlier_distance_)
{
	Mat d[3];
	for (int i = 0; i < 3; i++)
	{
		Mat x = src_.row(i).t();
		d[i] = abs((pointClouds_ * x - 1) / norm(x));
	}

	Mat p[3];
	for (int i = 0; i < d[0].rows; i++)
	{
		int inlier_cnt = 0;
		for (int j = 0; j < 3; j++) if (d[j].at<double>(i) < inlier_distance_) inlier_cnt++;
		if (inlier_cnt != 1) continue;

		double min_d = MIN(d[0].at<double>(i), MIN(d[1].at<double>(i), d[2].at<double>(i)));
		if (min_d == d[0].at<double>(i)) p[0].push_back(pointClouds_.row(i));
		else if (min_d == d[1].at<double>(i)) p[1].push_back(pointClouds_.row(i));
		else p[2].push_back(pointClouds_.row(i));
	}
	for (int i = 0; i < 3; i++) if (p[i].rows == 0) return;

	//SVD
	Mat dst;
	for (int i = 0; i < 3; i++)
	{
		Mat ones = Mat::ones(Size(p[i].rows, 1), p[i].type());
		p[i] = p[i].t();
		p[i].push_back(-ones);
		p[i] = p[i].t();

		Mat w, u, vt;
		SVD::compute(p[i], w, u, vt);

		Mat y = (Mat_<double>(1, 3) << vt.row(3).at<double>(0), vt.row(3).at<double>(1), vt.row(3).at<double>(2));
		y /= vt.row(3).at<double>(3);

		dst.push_back(y);
	}

	dst_ = dst.clone();
}

void findRotationSamples(Mat const& src_, vector<Mat>& dst_)
{
	Mat a[3], A, dst[4], w, u, vt;

	A = src_.clone();
	SVD::compute(A, w, u, vt);
	dst[0] = u * vt;
	A.release();

	a[0] = src_.row(0);
	a[1] = src_.row(1);
	a[2] = a[0].cross(a[1]);
	for (int i = 0; i < 3; i++) A.push_back(a[i]);
	SVD::compute(A, w, u, vt);
	dst[1] = u * vt;
	A.release();

	a[0] = src_.row(0);
	a[2] = src_.row(2);
	a[1] = a[0].cross(a[2]);
	for (int i = 0; i < 3; i++) A.push_back(a[i]);
	SVD::compute(A, w, u, vt);
	dst[2] = u * vt;
	A.release();

	a[1] = src_.row(1);
	a[2] = src_.row(2);
	a[0] = a[1].cross(a[2]);
	for (int i = 0; i < 3; i++) A.push_back(a[i]);
	SVD::compute(A, w, u, vt);
	dst[3] = u * vt;
	A.release();

	for (int i = 0; i < 4; i++) dst_.push_back(dst[i]);
}

void findOriginSamples(vector<Mat> const& src_, vector<Mat>& dst_, Mat const& init_three_planes_, Mat const& pointClouds_, double inlier_distance_)
{
	Mat d[3];
	for (int i = 0; i < 3; i++)
	{
		Mat x = init_three_planes_.row(i).t();
		d[i] = abs((pointClouds_ * x - 1) / norm(x));
	}

	Mat p[3];
	for (int i = 0; i < d[0].rows; i++)
	{
		int inlier_cnt = 0;
		for (int j = 0; j < 3; j++) if (d[j].at<double>(i) < inlier_distance_) inlier_cnt++;
		if (inlier_cnt != 1) continue;

		double min_d = MIN(d[0].at<double>(i), MIN(d[1].at<double>(i), d[2].at<double>(i)));
		if (min_d == d[0].at<double>(i)) p[0].push_back(pointClouds_.row(i));
		else if (min_d == d[1].at<double>(i)) p[1].push_back(pointClouds_.row(i));
		else p[2].push_back(pointClouds_.row(i));
	}
	for (int i = 0; i < 3; i++) if (p[i].rows == 0) return;

	//SVD
	Mat A, w, u, vt;
	for (int i = 0; i < src_.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Mat n = src_[i].row(j);

			for (int k = 0; k < p[j].rows; k++)
			{
				Mat x = p[j].row(k);
				Mat a = (Mat_<double>(1, 4) << -n.at<double>(0), -n.at<double>(1), -n.at<double>(2), x.dot(n));
				A.push_back(a);
			}
		}

		SVD::compute(A, w, u, vt);
		Mat t = vt.row(3) / vt.row(3).at<double>(3);

		dst_.push_back(t(Rect(0, 0, 3, 1)));

		A.release();
	}
}

void checkAxesDirection(Mat const& src_, Mat& dst_, Mat const& axis_origin_, Mat const& pointClouds_, double inlier_distance_)
{
	src_.copyTo(dst_);

	Mat planes;
	for (int i = 0; i < src_.rows; i++) planes.push_back(src_.row(i) / (src_.row(i) * axis_origin_.t()));

	Mat d[3];
	for (int i = 0; i < 3; i++)
	{
		Mat x = planes.row(i).t();
		d[i] = abs((pointClouds_ * x - 1) / norm(x));
	}

	Mat p[3];
	for (int i = 0; i < d[0].rows; i++)
	{
		int inlier_cnt = 0;
		for (int j = 0; j < 3; j++) if (d[j].at<double>(i) < inlier_distance_) inlier_cnt++;
		if (inlier_cnt != 1) continue;

		double min_d = MIN(d[0].at<double>(i), MIN(d[1].at<double>(i), d[2].at<double>(i)));
		if (min_d == d[0].at<double>(i)) p[0].push_back(pointClouds_.row(i));
		else if (min_d == d[1].at<double>(i)) p[1].push_back(pointClouds_.row(i));
		else p[2].push_back(pointClouds_.row(i));
	}
	for (int i = 0; i < 3; i++) if (p[i].rows == 0) return;

	//check axis direction
	for (int i = 0; i < 3; i++)
	{
		int idx_k[2];
		if (i == 0) idx_k[0] = 1, idx_k[1] = 2;
		else if (i == 1) idx_k[0] = 0, idx_k[1] = 2;
		else if (i == 2) idx_k[0] = 0, idx_k[1] = 1;

		Mat x;
		for (int k = 0; k < 2; k++) x.push_back(p[idx_k[k]](Rect(0, 0, 3, p[idx_k[k]].rows)));
		Mat n = dst_.row(i) / norm(dst_.row(i));

		x.col(0) -= axis_origin_.at<double>(0);
		x.col(1) -= axis_origin_.at<double>(1);
		x.col(2) -= axis_origin_.at<double>(2);

		Mat checker = x * n.t();
		if (sum(checker)[0] < 0) dst_.row(i) = -dst_.row(i);
	}
}

void getRvecs(vector<Mat> const& src_, Mat& dst_)
{
	//align rotation Mat
	for (int i = 1; i < src_.size(); i++)
	{
		vector<Mat>Rs = { Mat(), Mat(), Mat() };

		Rs[0] = src_[i];
		Rs[1].push_back(Rs[0].row(1));
		Rs[1].push_back(Rs[0].row(2));
		Rs[1].push_back(Rs[0].row(0));
		Rs[2].push_back(Rs[0].row(2));
		Rs[2].push_back(Rs[0].row(0));
		Rs[2].push_back(Rs[0].row(1));

		Mat errors;
		errors.push_back(getRotationError(src_[0], Rs[0]));
		errors.push_back(getRotationError(src_[0], Rs[1]));
		errors.push_back(getRotationError(src_[0], Rs[2]));

		int minIdx[2];
		double minVal, maxVal;
		minMaxIdx(errors, &minVal, &maxVal, minIdx);
		Rs[minIdx[0]].copyTo(src_[i]);
	}

	//convert Rmat to rvec
	for (int i = 0; i < src_.size(); i++)
	{
		Mat rvec;
		Rodrigues(src_[i], rvec);
		dst_.push_back(rvec.t());
	}
}

Mat scoreParamVec(Mat const& samples_, Mat const& pointClouds_, double threshold_, int score_method_ = MSC)
{
	double squared_thr = threshold_ * threshold_;

	Mat scores;
	for (int i = 0; i < samples_.rows; i++)
	{
		Mat R, t;
		Rodrigues(samples_(Rect(0, i, 3, 1)), R);
		t = samples_(Rect(3, i, 3, 1)).t();

		double sum_scores = 0;
		for (int j = 0; j < pointClouds_.rows; j++)
		{
			Mat p = pointClouds_.row(j).t();
			Mat e = R * (p - t);
			Mat squared_e = e.mul(e);
			double minVal = MIN(MIN(squared_e.at<double>(0), squared_e.at<double>(1)), squared_e.at<double>(2));

			double score;
			if (minVal > squared_thr) score = 0.;
			else
			{
				if (score_method_ == RSC) score = 1.;
				else if (score_method_ == MSC) score = 1. - (minVal / squared_thr);
			}

			sum_scores += score;
		}
		double normalizedScore = sum_scores / (double)pointClouds_.rows;

		scores.push_back(normalizedScore);
	}

	return scores;
}

Mat findBestScoredParamVec(Mat const& samples_, Mat const& scores_)
{
	double minVal, maxVal;
	int minIdx[2], maxIdx[2];
	minMaxIdx(scores_, &minVal, &maxVal, minIdx, maxIdx);

	return samples_.row(maxIdx[0]).t();
}

Mat findDominantParamVec(Mat const& samples_, Mat const& pointClouds_, int proposed_augmentation_max_, double threshold_, int score_method_ = MSC)
{
	if (samples_.rows == 0) return Mat();
	else if (samples_.rows == 1) return samples_;

	//initialization: convert rvec to quaternion
	int samplesDim = samples_.cols;
	int samplesSize = samples_.rows;
	Mat samples = samples_.clone();
	Mat rvecs = samples_(Rect(0, 0, 3, samplesSize)).clone();
	Mat tvecs = samples_(Rect(3, 0, 3, samplesSize)).clone();
	Mat quats;
	for (int i = 0; i < samplesSize; i++)
	{
		Mat q_i;
		rvec2quat(rvecs.row(i).t(), q_i);
		quats.push_back(q_i.t());
	}

	//measure scores for samples
	Mat scores = scoreParamVec(samples, pointClouds_, threshold_, score_method_);

	//get sorted indices
	Mat sortedIndices;
	sortIdx(scores, sortedIndices, SORT_EVERY_COLUMN);
	flip(sortedIndices, sortedIndices, 0);

	//sample augmentation
	Mat quat_seq, tvec_seq, score_seq, aug_samples;
	for (int i = 0; i < MIN(sortedIndices.rows, proposed_augmentation_max_); i++)
	{
		int idx = sortedIndices.at<int>(i);
		Mat quat_i = quats.row(idx);
		Mat tvec_i = tvecs.row(idx);
		Mat score_i = scores.row(idx);
		quat_seq.push_back(quat_i);
		tvec_seq.push_back(tvec_i);
		score_seq.push_back(score_i);

		if (i > 0)
		{
			Mat aug_quat_i = wMean(quat_seq, score_seq);
			Mat aug_rvec_i;
			quat2rvec(aug_quat_i, aug_rvec_i);
			Mat aug_tvec_i = wMean(tvec_seq, score_seq);
			Mat sample_i;
			sample_i.push_back(aug_rvec_i);
			sample_i.push_back(aug_tvec_i);
			aug_samples.push_back(sample_i.t());
		}
	}

	//measure scores for the augmented samples
	Mat aug_scores = scoreParamVec(aug_samples, pointClouds_, threshold_, score_method_);

	//concatenate the augmented samples and original samples
	aug_samples.push_back(samples);
	aug_scores.push_back(scores);

	//find the best-scored sample
	Mat dst = findBestScoredParamVec(aug_samples, aug_scores);

	return dst;
}

void findInlierPointCloud(Args const& args_, Calibration& calb_, Mat const& pointClouds_)
{
	if (pointClouds_.rows == 0) return;
	for (int i = 0; i < 3; i++) calb_.inlier_pointClouds[i].release();

	Mat d[3];
	for (int i = 0; i < 3; i++)
	{
		Mat x = calb_.planes().row(i).t();
		d[i] = abs((pointClouds_ * x - 1) / norm(x));
	}

	for (int i = 0; i < d[0].rows; i++)
	{
		double min_d = MIN(d[0].at<double>(i), MIN(d[1].at<double>(i), d[2].at<double>(i)));
		if (min_d > args_.inlier_distance) continue;

		if (min_d == d[0].at<double>(i)) calb_.inlier_pointClouds[0].push_back(pointClouds_.row(i));
		else if (min_d == d[1].at<double>(i)) calb_.inlier_pointClouds[1].push_back(pointClouds_.row(i));
		else calb_.inlier_pointClouds[2].push_back(pointClouds_.row(i));
	}
}

void findInitCoordinate(Args const& args_, Calibration& calb_, bool use_roi_ = false, double dist_range_ = 20)
{
	//crop target region
	Mat pcInROI;
	if (use_roi_ == true) pcInROI = getPointCloudInROI(calb_, dist_range_);
	else pcInROI = calb_.pointCloud.clone();

	//sampling initial LiDAR poses
	vector<Mat> sample_axis_vectors;
	Mat sample_origins;
	vector<int> init_indices, indices, rand_indices;
	for (int i = 0; i < pcInROI.rows; i++) init_indices.push_back(i);
	for (int itr = 0; itr < args_.n_loop_init_pose; itr++)
	{
		Mat planes;
		Mat threePlanes = Mat(Size(3, 3), pcInROI.type());

		while (1)
		{
			for (int i = 0; i < args_.n_max_plane; i++)
			{
				if (i == 0)
				{
					indices.clear();
					indices.assign(init_indices.begin(), init_indices.end());
				}
				else
				{
					for (int j = indices.size() - 1; j >= 0; j--)
					{
						Mat P = pcInROI.row(indices[j]);
						Mat x = planes.row(planes.rows - 1).t();
						Mat d = abs((P * x - 1) / norm(x));
						if (d.at<double>(0) < args_.inlier_distance) indices.erase(indices.begin() + j, indices.begin() + j + 1);
					}
				}
				if (indices.size() < args_.n_point_plane_subsample + 1) break;

				Mat P, bestPlane;
				for (int j = 0; j < indices.size(); j++) P.push_back(pcInROI.row(indices[j]));
				double max_score = args_.min_plane_score;
				for (int j = 0; j < args_.n_loop_plane_hypo; j++)
				{
					rand_indices.clear();
					rand_indices = generateRandVal(0, indices.size() - 1, args_.n_point_plane_subsample);

					Mat A, piA, b, x, d;
					double score = 0;
					for (int k = 0; k < args_.n_point_plane_subsample; k++) A.push_back(pcInROI.row(indices[rand_indices[k]]));
					piA = (A.t() * A).inv() * A.t();
					b = Mat::ones(Size(1, args_.n_point_plane_subsample), CV_64FC1);
					x = piA * b;
					d = abs((P * x - 1) / norm(x)); //Euclidean distance
					if (args_.score_method == RSC)
					{
						for (int k = 0; k < d.rows; k++) if (d.at<double>(k) < args_.inlier_distance) score++;
					}
					else if (args_.score_method == MSC)
					{
						for (int k = 0; k < d.rows; k++) if (d.at<double>(k) < args_.inlier_distance) score += 1. - pow(d.at<double>(k) / args_.inlier_distance, 2);
					}

					if (score > max_score)
					{
						max_score = score;
						x = x.t();
						x.copyTo(bestPlane);
					}
				}

				if (!bestPlane.empty()) planes.push_back(bestPlane);
			}

			if (planes.rows < 3) continue;

			//calculate normals
			Mat normals;
			for (int i = 0; i < planes.rows; i++) normals.push_back(planes.row(i) / norm(planes.row(i)));

			//find the best perpendicular 3-planes
			double minError = 1;
			for (int i = 0; i < planes.rows; i++)
			{
				for (int j = i + 1; j < planes.rows; j++)
				{
					for (int k = j + 1; k < planes.rows; k++)
					{
						double error = MAX(abs(normals.row(i).dot(normals.row(j))), MAX(abs(normals.row(i).dot(normals.row(k))), abs(normals.row(j).dot(normals.row(k)))));

						if (error < minError)
						{
							minError = error;
							planes.row(i).copyTo(threePlanes.row(0));
							planes.row(j).copyTo(threePlanes.row(1));
							planes.row(k).copyTo(threePlanes.row(2));
						}
					}
				}
			}

			if (minError > 0.5)
			{
				planes.release();
				threePlanes.setTo(0);
			}
			else break;
		}

		//fit three planes
		fitPlanes(threePlanes, threePlanes, pcInROI, args_.inlier_distance);

		//normalize axes
		Mat normalizedAxis;
		for (int i = 0; i < 3; i++) normalizedAxis.push_back(threePlanes.row(i) / norm(threePlanes.row(i)));

		//find rotation samples
		vector<Mat> perpendicular_axis_vectors;
		findRotationSamples(normalizedAxis, perpendicular_axis_vectors);

		//find origin samples
		vector<Mat> axis_origins;
		findOriginSamples(perpendicular_axis_vectors, axis_origins, threePlanes, pcInROI, args_.inlier_distance);

		//4-sampled poses
		for (int i = 0; i < perpendicular_axis_vectors.size(); i++)
		{
			//check axes direction
			checkAxesDirection(perpendicular_axis_vectors[i], perpendicular_axis_vectors[i], axis_origins[i], pcInROI, args_.inlier_distance);

			//follow right-hand-rule
			double checkRhr = (perpendicular_axis_vectors[i].row(0).cross(perpendicular_axis_vectors[i].row(1))).dot(perpendicular_axis_vectors[i].row(2));
			if (checkRhr < 0)
			{
				Mat temp_threePlanes;
				temp_threePlanes.push_back(perpendicular_axis_vectors[i].row(0));
				temp_threePlanes.push_back(perpendicular_axis_vectors[i].row(2));
				temp_threePlanes.push_back(perpendicular_axis_vectors[i].row(1));
				temp_threePlanes.copyTo(perpendicular_axis_vectors[i]);
			}

			//assign planes, axis-origin and axis-vectors
			sample_axis_vectors.push_back(perpendicular_axis_vectors[i]);
			sample_origins.push_back(axis_origins[i]);
		}
	}

	//set samples into equal ordered coordinate system
	Mat rvecs;
	getRvecs(sample_axis_vectors, rvecs);

	//arrange sample params as 6-components
	Mat sample_params;
	sample_params.push_back(rvecs.t());
	sample_params.push_back(sample_origins.t());
	sample_params = sample_params.t();

	//find dominant parameter vector
	Mat dominantParam = findDominantParamVec(sample_params, pcInROI, args_.n_aug_sample, args_.inlier_distance, args_.score_method);
	Mat axis_vectors;
	Rodrigues(dominantParam(Rect(0, 0, 1, 3)), axis_vectors);
	calb_.lidar_Rmat = axis_vectors.t();
	calb_.lidar_tvec = dominantParam(Rect(0, 3, 1, 3)).t();

	//update inliers
	findInlierPointCloud(args_, calb_, pcInROI);
}

void removeOutlierPointCloud(Args const& args_, Calibration& calb_)
{
	//based on distance with plane
	for (int i = 0; i < 3; i++) if (calb_.inlier_pointClouds[i].rows == 0) return;

	for (int i = 0; i < 3; i++)
	{
		if (i == 0)
		{
			Mat p = calb_.inlier_pointClouds[i].clone();
			Mat x1 = calb_.planes().row(1).t();
			Mat x2 = calb_.planes().row(2).t();
			Mat d1 = p * x1 - 1;
			Mat d2 = p * x2 - 1;

			calb_.inlier_pointClouds[i].release();
			for (int k = 0; k < p.rows; k++) if (d1.at<double>(k) > 0 && d2.at<double>(k) > 0) calb_.inlier_pointClouds[i].push_back(p.row(k));
		}
		else if (i == 1)
		{
			Mat p = calb_.inlier_pointClouds[i].clone();
			Mat x1 = calb_.planes().row(0).t();
			Mat x2 = calb_.planes().row(2).t();
			Mat d1 = p * x1 - 1;
			Mat d2 = p * x2 - 1;

			calb_.inlier_pointClouds[i].release();
			for (int k = 0; k < p.rows; k++) if (d1.at<double>(k) > 0 && d2.at<double>(k) > 0) calb_.inlier_pointClouds[i].push_back(p.row(k));
		}
		else
		{
			Mat p = calb_.inlier_pointClouds[i].clone();
			Mat x1 = calb_.planes().row(0).t();
			Mat x2 = calb_.planes().row(1).t();
			Mat d1 = p * x1 - 1;
			Mat d2 = p * x2 - 1;

			calb_.inlier_pointClouds[i].release();
			for (int k = 0; k < p.rows; k++) if (d1.at<double>(k) > 0 && d2.at<double>(k) > 0) calb_.inlier_pointClouds[i].push_back(p.row(k));
		}
	}
}

void removeOutlierPointCloud2(Args const& args_, Calibration& calb_)
{
	//based on distance with the target region
	for (int i = 0; i < 3; i++) if (calb_.inlier_pointClouds[i].rows == 0) return;

	// initialize
	Mat wpc;
	for (int i = 0; i < 3; i++) wpc.push_back(calb_.inlier_pointClouds[i]);
	Mat Rmat_lw = calb_.lidar_Rmat.t();
	Mat tvec_lw = -(calb_.lidar_tvec * Rmat_lw.t());
	wpc = (Rmat_lw * wpc.t() + tvec_lw.t() * Mat::ones(Size(wpc.rows, 1), tvec_lw.type())).t();
	
	double maxDists[3] = { 0, };
	for (int i = 0; i < 3; i++)
	{
		if (i == 0)
		{
			Mat ppc = wpc.clone();
			ppc.col(1).setTo(0);
			ppc.col(2).setTo(0);
			for (int k = 0; k < wpc.rows; k++) 
			{
				double dk = norm(wpc.row(k) - ppc.row(k));
				if (dk < args_.inlier_distance && ppc.at<double>(k, 0) > maxDists[i]) maxDists[i] = ppc.at<double>(k, 0);
			}
		}
		else if (i == 1)
		{
			Mat ppc = wpc.clone();
			ppc.col(0).setTo(0);
			ppc.col(2).setTo(0);
			for (int k = 0; k < wpc.rows; k++)
			{
				double dk = norm(wpc.row(k) - ppc.row(k));
				if (dk < args_.inlier_distance && ppc.at<double>(k, 1) > maxDists[i]) maxDists[i] = ppc.at<double>(k, 1);
			}
		}
		else
		{
			Mat ppc = wpc.clone();
			ppc.col(0).setTo(0);
			ppc.col(1).setTo(0);
			for (int k = 0; k < wpc.rows; k++)
			{
				double dk = norm(wpc.row(k) - ppc.row(k));
				if (dk < args_.inlier_distance && ppc.at<double>(k, 2) > maxDists[i]) maxDists[i] = ppc.at<double>(k, 2);
			}
		}
	}

	int n_points[3] = { calb_.inlier_pointClouds[0].rows, calb_.inlier_pointClouds[1].rows, calb_.inlier_pointClouds[2].rows };
	Mat temp_inlier_pointClouds[3];
	for (int i = 0; i < wpc.rows; i++)
	{
		if (wpc.at<double>(i, 0) < maxDists[0] && wpc.at<double>(i, 1) < maxDists[1] && wpc.at<double>(i, 2) < maxDists[2])
		{
			if (i < n_points[0]) temp_inlier_pointClouds[0].push_back(calb_.inlier_pointClouds[0].row(i));
			else if (i >= n_points[0] && i < n_points[0] + n_points[1]) temp_inlier_pointClouds[1].push_back(calb_.inlier_pointClouds[1].row(i - n_points[0]));
			else if (i >= n_points[0] + n_points[1] && i < n_points[0] + n_points[1] + n_points[2]) temp_inlier_pointClouds[2].push_back(calb_.inlier_pointClouds[2].row(i - n_points[0] - n_points[1]));
		}
	}

	for (int i = 0; i < 3; i++) temp_inlier_pointClouds[i].copyTo(calb_.inlier_pointClouds[i]);
}

void refinement(Args const& args_, Calibration& calb_)
{
	for (int i = 0; i < 3; i++) if (calb_.inlier_pointClouds[i].rows == 0) return;

	// initialize
	Mat x;
	for (int i = 0; i < 3; i++) x.push_back(calb_.inlier_pointClouds[i]);
	Mat thetas;
	rotationToZyxEuler(calb_.lidar_Rmat.t(), thetas);
	Mat curr_p = (Mat_<double>(6, 1) << thetas.at<double>(0), thetas.at<double>(1), thetas.at<double>(2), calb_.lidar_tvec.at<double>(0), calb_.lidar_tvec.at<double>(1), calb_.lidar_tvec.at<double>(2));
	double lambda = args_.refine_init_lambda;

	// iteration
	Mat t, vs;
	Mat prev_p = curr_p.clone();
	Mat diff_v_thetaX[3], diff_v_thetaY[3], diff_v_thetaZ[3];
	for (int i = 0; i < args_.n_max_refine_iteration; i++)
	{
		zyxEulerToRotation(curr_p(Rect(0, 0, 1, 3)), vs);
		t = curr_p(Rect(0, 3, 1, 3)).t();

		double thetaX = curr_p.at<double>(0);
		double thetaY = curr_p.at<double>(1);
		double thetaZ = curr_p.at<double>(2);

		diff_v_thetaX[0] = (Mat_<double>(1, 3) << 0, sin(thetaX) * sin(thetaZ) + cos(thetaX) * sin(thetaY) * cos(thetaZ), cos(thetaX) * sin(thetaZ) - sin(thetaX) * sin(thetaY) * cos(thetaZ));
		diff_v_thetaX[1] = (Mat_<double>(1, 3) << 0, -sin(thetaX) * cos(thetaZ) + cos(thetaX) * sin(thetaY) * sin(thetaZ), -cos(thetaX) * cos(thetaZ) - sin(thetaX) * sin(thetaY) * sin(thetaZ));
		diff_v_thetaX[2] = (Mat_<double>(1, 3) << 0, cos(thetaX) * cos(thetaY), -sin(thetaX) * cos(thetaY));
		diff_v_thetaY[0] = (Mat_<double>(1, 3) << -sin(thetaY) * cos(thetaZ), sin(thetaX) * cos(thetaY) * cos(thetaZ), cos(thetaX) * cos(thetaY) * cos(thetaZ));
		diff_v_thetaY[1] = (Mat_<double>(1, 3) << -sin(thetaY) * sin(thetaZ), sin(thetaX) * cos(thetaY) * sin(thetaZ), cos(thetaX) * cos(thetaY) * sin(thetaZ));
		diff_v_thetaY[2] = (Mat_<double>(1, 3) << -cos(thetaY), -sin(thetaX) * sin(thetaY), -cos(thetaX) * sin(thetaY));
		diff_v_thetaZ[0] = (Mat_<double>(1, 3) << -cos(thetaY) * sin(thetaZ), -cos(thetaX) * cos(thetaZ) - sin(thetaX) * sin(thetaY) * sin(thetaZ), sin(thetaX) * cos(thetaZ) - cos(thetaX) * sin(thetaY) * sin(thetaZ));
		diff_v_thetaZ[1] = (Mat_<double>(1, 3) << cos(thetaY) * cos(thetaZ), -cos(thetaX) * sin(thetaZ) + sin(thetaX) * sin(thetaY) * cos(thetaZ), sin(thetaX) * sin(thetaZ) + cos(thetaX) * sin(thetaY) * cos(thetaZ));
		diff_v_thetaZ[2] = (Mat_<double>(1, 3) << 0, 0, 0);

		Mat J, r, wJ, wr;
		for (int j = 0; j < x.rows; j++)
		{
			Mat xt = x.row(j) - t;

			Mat d;
			for (int k = 0; k < 3; k++) d.push_back(abs(vs.row(k) * xt.t()));

			int min_k[2];
			double w, sum_d, min_d, max_d;
			sum_d = sum(d)[0];
			minMaxIdx(d, &min_d, &max_d, min_k);
			w = 1. - 3. * (min_d / sum_d);
			w = pow(w, args_.refine_gamma);

			Mat rk = vs.row(min_k[0]) * xt.t();
			r.push_back(rk);
			wr.push_back(w * rk);

			Mat j0 = diff_v_thetaX[min_k[0]] * xt.t();
			Mat j1 = diff_v_thetaY[min_k[0]] * xt.t();
			Mat j2 = diff_v_thetaZ[min_k[0]] * xt.t();
			double j3 = -vs.row(min_k[0]).at<double>(0);
			double j4 = -vs.row(min_k[0]).at<double>(1);
			double j5 = -vs.row(min_k[0]).at<double>(2);

			Mat Jk = (Mat_<double>(1, curr_p.rows) << j0.at<double>(0), j1.at<double>(0), j2.at<double>(0), j3, j4, j5);
			J.push_back(Jk);
			wJ.push_back(w * Jk);
		}

		Mat JtwJ = J.t() * wJ;
		Mat diag_JtwJ = Mat::zeros(JtwJ.size(), JtwJ.type());
		for (int j = 0; j < diag_JtwJ.rows; j++) diag_JtwJ.at<double>(j, j) = JtwJ.at<double>(j, j);

		Mat gradient;
		double prev_r = r.dot(wr);
		while (i < args_.n_max_refine_iteration)
		{
			//gradient
			gradient = (JtwJ + lambda * diag_JtwJ).inv() * J.t() * wr;

			//check gradient norm
			if (norm(gradient(Rect(0, 0, 1, 3))) < args_.refine_min_gradient_norm_r && norm(gradient(Rect(0, 3, 1, 3))) < args_.refine_min_gradient_norm_t) break;

			curr_p = prev_p - gradient;

			zyxEulerToRotation(curr_p(Rect(0, 0, 1, 3)), vs);
			t = curr_p(Rect(0, 3, 1, 3)).t();
			double curr_r = 0;
			for (int j = 0; j < x.rows; j++)
			{
				Mat xt = x.row(j) - t;

				Mat d;
				for (int k = 0; k < 3; k++) d.push_back(abs(vs.row(k) * xt.t()));

				int min_k[2];
				double w, sum_d, min_d, max_d;
				sum_d = sum(d)[0];
				minMaxIdx(d, &min_d, &max_d, min_k);
				w = 1 - 3 * (min_d / sum_d);
				w = pow(w, args_.refine_gamma);

				Mat rk = vs.row(min_k[0]) * xt.t();
				curr_r += w * pow(rk.at<double>(0), 2);
			}

			if (prev_r > curr_r)
			{
				lambda /= args_.refine_lambda_dn_fac;
				prev_p = curr_p.clone();
				break;
			}
			else
			{
				lambda *= args_.refine_lambda_up_fac;
				i++;
			}
		}

		//check gradient norm
		if (norm(gradient(Rect(0, 0, 1, 3))) < args_.refine_min_gradient_norm_r && norm(gradient(Rect(0, 3, 1, 3))) < args_.refine_min_gradient_norm_t) break;
	}

	calb_.lidar_tvec = curr_p(Rect(0, 3, 1, 3)).t();
	Mat axis_vectors;
	zyxEulerToRotation(curr_p(Rect(0, 0, 1, 3)), axis_vectors);
	calb_.lidar_Rmat = axis_vectors.t();

	//update inliers
	findInlierPointCloud(args_, calb_, x);

	//remove outliers
	removeOutlierPointCloud(args_, calb_);
}

void assignChessboardCornersIn3D(Args const& args_, Calibration& calb_)
{
	Point3i chessAxes[3];
	chessAxes[0] = Point3i(0, 1, 2);
	chessAxes[1] = Point3i(1, 2, 0);
	chessAxes[2] = Point3i(2, 0, 1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < args_.chessSize.height; j++)
		{
			for (int k = 0; k < args_.chessSize.width; k++)
			{
				Mat p3d = Mat_<Vec3f>(1, 1);
				p3d.at<Vec3f>(0)[chessAxes[i].x] = (float)k * (float)args_.chessGridLen + (float)args_.chessOffset.width;
				p3d.at<Vec3f>(0)[chessAxes[i].y] = (float)j * (float)args_.chessGridLen + (float)args_.chessOffset.height;
				p3d.at<Vec3f>(0)[chessAxes[i].z] = 0;

				calb_.chessCorners3D.push_back(p3d);
			}
		}
	}
}

void findChessboardCorners(Args const& args_, Calibration& calb_)
{
	Mat grayImg;
	cvtColor(calb_.cameraImg, grayImg, COLOR_BGR2GRAY);

	//find chess corners on 3-planes
	vector<Point2f> corners[3];
	vector<Point2i> fourCorners[3];
	for (int i = 0; i < 3; i++)
	{
		bool patternfound = findChessboardCorners(grayImg, args_.chessSize, corners[i]);
		if (!patternfound) patternfound = findChessboardCorners(grayImg, args_.chessSize, corners[i], CALIB_CB_ADAPTIVE_THRESH);
		if (!patternfound)
		{
			cout << "can not find three chessboards in this image" << endl << endl;
			return;
		}

		cornerSubPix(grayImg, corners[i], Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));

		fourCorners[i].push_back((Point2i)corners[i][0]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width - 1]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width * args_.chessSize.height - 1]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width * (args_.chessSize.height - 1)]);

		Mat mask = Mat::zeros(grayImg.size(), grayImg.type());
		fillPoly(mask, fourCorners[i], Scalar(1));
		grayImg.setTo(Scalar(0), mask);
	}

	//find origin indices of chess corners
	int indices[3] = { 0, };
	double minDist = calb_.cameraImg.size().width * calb_.cameraImg.size().height;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				double dist = sqrt((double)pow(fourCorners[0][i].x - fourCorners[1][j].x, 2) + (double)pow(fourCorners[0][i].y - fourCorners[1][j].y, 2)) +
					sqrt((double)pow(fourCorners[0][i].x - fourCorners[2][k].x, 2) + (double)pow(fourCorners[0][i].y - fourCorners[2][k].y, 2)) +
					sqrt((double)pow(fourCorners[1][j].x - fourCorners[2][k].x, 2) + (double)pow(fourCorners[1][j].y - fourCorners[2][k].y, 2)) / 3;

				if (dist < minDist)
				{
					minDist = dist;
					indices[0] = i;
					indices[1] = j;
					indices[2] = k;
				}
			}
		}
	}

	//order chess corners on each planes
	for (int i = 0; i < 3; i++)
	{
		Mat cornersMat = Mat(args_.chessSize, CV_32FC2, &corners[i][0]);
		for (int j = 0; j < indices[i]; j++) rotate(cornersMat, cornersMat, ROTATE_90_COUNTERCLOCKWISE);

		corners[i].clear();
		corners[i].assign((Point2f*)cornersMat.datastart, (Point2f*)cornersMat.dataend);

		fourCorners[i].clear();
		fourCorners[i].push_back((Point2i)corners[i][0]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width - 1]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width * args_.chessSize.height - 1]);
		fourCorners[i].push_back((Point2i)corners[i][args_.chessSize.width * (args_.chessSize.height - 1)]);
	}

	//align chess corner sets as following right-hand-rule
	Mat cameraNorAxis_x[3], cameraNorAxis_y[3];
	for (int i = 0; i < 3; i++)
	{
		cameraNorAxis_x[i] = (Mat_<double>(2, 1) << fourCorners[i][1].x - fourCorners[i][0].x, fourCorners[i][1].y - fourCorners[i][0].y);
		cameraNorAxis_y[i] = (Mat_<double>(2, 1) << fourCorners[i][3].x - fourCorners[i][0].x, fourCorners[i][3].y - fourCorners[i][0].y);
		cameraNorAxis_x[i] /= norm(cameraNorAxis_x[i]);
		cameraNorAxis_y[i] /= norm(cameraNorAxis_y[i]);
	}

	if (cameraNorAxis_y[0].dot(cameraNorAxis_x[1]) > cameraNorAxis_y[0].dot(cameraNorAxis_x[2]))
	{
		for (int i = 0; i < 3; i++) calb_.chessCorners2D.push_back(Mat(Size(1, args_.chessSize.width * args_.chessSize.height), CV_32FC2, &corners[i][0]));
	}
	else
	{
		calb_.chessCorners2D.push_back(Mat(Size(1, args_.chessSize.width * args_.chessSize.height), CV_32FC2, &corners[0][0]));
		calb_.chessCorners2D.push_back(Mat(Size(1, args_.chessSize.width * args_.chessSize.height), CV_32FC2, &corners[2][0]));
		calb_.chessCorners2D.push_back(Mat(Size(1, args_.chessSize.width * args_.chessSize.height), CV_32FC2, &corners[1][0]));
	}
}

void calcRelativePose(Args const& args_, Calibration& calb_, int pnp_method_)
{
	//assign 3D points in lidar coordinates
	Mat points3D;
	for (int i = 0; i < calb_.chessCorners3D.rows; i++)
	{
		Mat x = (Mat_<double>(1, 3) << calb_.chessCorners3D.at<Vec3f>(i)[0], calb_.chessCorners3D.at<Vec3f>(i)[1], calb_.chessCorners3D.at<Vec3f>(i)[2]);
		x = x.at<double>(0) * calb_.lidar_Rmat.col(0).t() + x.at<double>(1) * calb_.lidar_Rmat.col(1).t() + x.at<double>(2) * calb_.lidar_Rmat.col(2).t() + calb_.lidar_tvec;

		Mat p3D(1, 1, CV_32FC3);

		p3D.at<Vec3f>(0)[0] = x.at<double>(0);
		p3D.at<Vec3f>(0)[1] = x.at<double>(1);
		p3D.at<Vec3f>(0)[2] = x.at<double>(2);

		points3D.push_back(p3D);
	}

	//assign three possible 2D points in camera coordinates
	Mat chessCorners2D_temp[3];
	int corners_num = calb_.chessCorners2D.rows / 3;
	for (int i = 0; i < 3; i++)
	{
		if (i == 0)
		{
			chessCorners2D_temp[i] = calb_.chessCorners2D.clone();
		}
		else if (i == 1)
		{
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, corners_num, calb_.chessCorners2D.cols, corners_num)));
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, 2 * corners_num, calb_.chessCorners2D.cols, corners_num)));
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, 0, calb_.chessCorners2D.cols, corners_num)));
		}
		else
		{
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, 2 * corners_num, calb_.chessCorners2D.cols, corners_num)));
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, 0, calb_.chessCorners2D.cols, corners_num)));
			chessCorners2D_temp[i].push_back(calb_.chessCorners2D(Rect(0, corners_num, calb_.chessCorners2D.cols, corners_num)));
		}
	}

	//estimate three possible relative pose cases
	Mat camera_tvec[3], camera_rvec[3], camera_R[3];
	Mat relative_tvec[3], relative_rvec[3], relative_R[3];
	for (int i = 0; i < 3; i++)
	{
		solvePnP(calb_.chessCorners3D, chessCorners2D_temp[i], calb_.camera_intrinsic, Mat(), camera_rvec[i], camera_tvec[i], false, pnp_method_);
		solvePnP(points3D, chessCorners2D_temp[i], calb_.camera_intrinsic, Mat(), relative_rvec[i], relative_tvec[i], false, pnp_method_);
		Rodrigues(camera_rvec[i], camera_R[i]);
		Rodrigues(relative_rvec[i], relative_R[i]);
	}

	//find reasonable pose: minimum rotation scale
	int minRidx = -1;
	double minRscale = CV_2PI;
	for (int i = 0; i < 3; i++)
	{
		double Rscale = getRotationError(Mat::eye(Size(3, 3), relative_R[i].type()), relative_R[i]);
		if (Rscale < minRscale)
		{
			minRidx = i;
			minRscale = Rscale;
		}
	}

	chessCorners2D_temp[minRidx].copyTo(calb_.chessCorners2D);
	calb_.camera_tvec = camera_tvec[minRidx].t();
	calb_.camera_Rmat = camera_R[minRidx].clone();
	calb_.l2c_tvec = relative_tvec[minRidx].t();
	calb_.l2c_Rmat = relative_R[minRidx].clone();

	int three_idx = 1;
	for (int i = 0; i < 3; i++)
	{
		if (minRidx == i)
		{
			calb_.l2c_three_Rmats[0] = relative_R[i].clone();
			calb_.l2c_three_tvecs[0] = relative_tvec[i].t();
		}
		else
		{
			calb_.l2c_three_Rmats[three_idx] = relative_R[i].clone();
			calb_.l2c_three_tvecs[three_idx] = relative_tvec[i].t();
			three_idx++;
		}
	}
}