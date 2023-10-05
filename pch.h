#ifndef PCH_H
#define PCH_H
#include <opencv2/opencv.hpp>

//name spaces
using namespace std;
using namespace cv;

//evaluation types
enum scoreType { RSC, MSC };

class Args {
public:
	string path_camera_img; // camera image file path
	string path_lidar_pc; // LiDAR pointcloud file path
	string path_blensor; // blensor file path

	double inlier_distance; // inlier threshold for plane and pose evaluation
	int score_method; // score method: RSC(RANSAC; hard-voting) and MSC(MSAC; truncated L2 loss)
	double min_plane_score; //minimum score for a plane to be detected
	int n_point_plane_subsample; // the number of subsamples for plane detection (>= 3)
	int n_max_plane; // Np: maximum number of plane detections
	int n_loop_plane_hypo; // the number of hypothesis generation-and-evaluation loops for plane detection
	int n_loop_init_pose; // Ns: the number of initial pose estimation loops, 4Ns initial poses are estimated
	int n_aug_sample; // Na: the number of augmented sample poses
	int n_max_refine_iteration; // max iteration for refinement
	double refine_gamma; // focusing parameter for the adaptive-weight
	double refine_init_lambda; // initial damping parameter for Levenberg-Marquardt
	double refine_lambda_up_fac; // increasing factor for Levenberg-Marquardt
	double refine_lambda_dn_fac; // decreasing factor for Levenberg-Marquardt
	double refine_min_gradient_norm_r; // early stop condition for rotation
	double refine_min_gradient_norm_t; // early stop condition for translation

	Size2i chessSize; // size of checkered pattern (in rows and columns)
	Size2d chessOffset; // offset for the origin of the checkered pattern
	double chessGridLen; // grid length of the checkered pattern
};

class Calibration {
public:
	Mat lidar_intrinsic;
	Mat uvImg;
	Mat pointCloud;
	Mat inlier_pointClouds[3];
	Mat uvPoints;

	Mat camera_intrinsic;
	Mat cameraImg;
	Mat chessCorners2D;
	Mat chessCorners3D;

	Mat lidar_tvec;
	Mat lidar_Rmat;

	Mat camera_tvec;
	Mat camera_Rmat;

	Mat l2c_tvec;
	Mat l2c_Rmat;

	Mat l2c_three_tvecs[3];
	Mat l2c_three_Rmats[3];

	Mat planes()
	{
		Mat planes;
		for (int i = 0; i < lidar_Rmat.cols; i++) planes.push_back(lidar_Rmat.col(i).t() / (lidar_tvec * lidar_Rmat.col(i)));

		return planes;
	}
};

class Blensor {
public:
	Mat object_scale;
	Mat object_location;
	Mat object_rotation;

	Mat lidar_intrinsic;
	Mat lidar_location;
	Mat lidar_default_rotation;
	Mat lidar_rotation;

	Mat camera_intrinsic;
	Mat camera_location;
	Mat camera_default_rotation;
	Mat camera_rotation;
};

class Gt {
public:
	Mat lidar_tvec;
	Mat lidar_Rmat;
	Mat axis_vectors2D;

	Mat camera_tvec;
	Mat camera_Rmat;

	Mat l2c_tvec;
	Mat l2c_Rmat;
};

class VertualCameraInf {
public:
	Mat rotation = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	Mat translation = (Mat_<double>(3, 1) << 0, 0, 0);

	const double visualSensitivityZoom = 0.5; //(meter)/(frame)
	const double visualSensitivityAngle = 0.2; //(degree)/(pixel)
	const double visualSensitivityTranslation = 0.01; //(meter)/(pixel)

	Mat MouseStartRotation = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	Mat MouseStartTranslation = (Mat_<double>(3, 1) << 0, 0, 0);
	Point2i MouseStartPointL = Point2i(0, 0);
	Point2i MouseStartPointR = Point2i(0, 0);
};

#endif //PCH_H
