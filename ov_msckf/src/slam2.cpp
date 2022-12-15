#include <functional>
#include <filesystem>
#include <fstream>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
#include "state/State.h"
#include "../../ov_core/src/feat/Feature.h"

#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/relative_clock.hpp"
#include "common/pose_prediction.hpp"

using namespace ILLIXR;
using namespace ov_msckf;

// Comment in if using ZED instead of offline_imu_cam
// TODO: Pull from config YAML file
// #define ZED

VioManagerOptions create_params()
{
	VioManagerOptions params;

	// Camera #0
	Eigen::Matrix<double, 8, 1> intrinsics_0;
#ifdef ZED
  // ZED calibration tool; fx, fy, cx, cy, k1, k2, p1, p2
  // https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
  intrinsics_0 << 349.686, 349.686, 332.778, 192.423, -0.175708, 0.0284421, 0, 0;
#else
  // EuRoC
	intrinsics_0 << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
#endif

#ifdef ZED
  // Camera extrinsics from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  std::vector<double> matrix_TCtoI_0 = {-0.01080233, 0.00183858, 0.99993996, 0.01220425,
            -0.99993288, -0.00420947, -0.01079452, 0.0146056,
            0.00418937, -0.99998945, 0.00188393, -0.00113692,
            0.0, 0.0, 0.0, 1.0};
#else
	std::vector<double> matrix_TCtoI_0 = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0};
#endif

  Eigen::Matrix4d T_CtoI_0;
	T_CtoI_0 << matrix_TCtoI_0.at(0), matrix_TCtoI_0.at(1), matrix_TCtoI_0.at(2), matrix_TCtoI_0.at(3),
		matrix_TCtoI_0.at(4), matrix_TCtoI_0.at(5), matrix_TCtoI_0.at(6), matrix_TCtoI_0.at(7),
		matrix_TCtoI_0.at(8), matrix_TCtoI_0.at(9), matrix_TCtoI_0.at(10), matrix_TCtoI_0.at(11),
		matrix_TCtoI_0.at(12), matrix_TCtoI_0.at(13), matrix_TCtoI_0.at(14), matrix_TCtoI_0.at(15);

	// Load these into our state
	Eigen::Matrix<double, 7, 1> extrinsics_0;
	extrinsics_0.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_0.block(0, 0, 3, 3).transpose());
	extrinsics_0.block(4, 0, 3, 1) = -T_CtoI_0.block(0, 0, 3, 3).transpose() * T_CtoI_0.block(0, 3, 3, 1);

	params.camera_fisheye.insert({0, false});
	params.camera_intrinsics.insert({0, intrinsics_0});
	params.camera_extrinsics.insert({0, extrinsics_0});

#ifdef ZED
  params.camera_wh.insert({0, {672, 376}});
#else
	params.camera_wh.insert({0, {752, 480}});
#endif

	// Camera #1
	Eigen::Matrix<double, 8, 1> intrinsics_1;
#ifdef ZED
  // ZED calibration tool; fx, fy, cx, cy, k1, k2, p1, p2
  intrinsics_1 << 350.01, 350.01, 343.729, 185.405, -0.174559, 0.0277521, 0, 0;
#else
  // EuRoC
	intrinsics_1 << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
#endif

#ifdef ZED
  // Camera extrinsics from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  std::vector<double> matrix_TCtoI_1 = {-0.01043535, -0.00191061, 0.99994372, 0.01190459,
            -0.99993668, -0.00419281, -0.01044329, -0.04732387,
            0.00421252, -0.99998938, -0.00186674, -0.00098799,
            0.0, 0.0, 0.0, 1.0};
#else
	std::vector<double> matrix_TCtoI_1 = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0};
#endif

  Eigen::Matrix4d T_CtoI_1;
	T_CtoI_1 << matrix_TCtoI_1.at(0), matrix_TCtoI_1.at(1), matrix_TCtoI_1.at(2), matrix_TCtoI_1.at(3),
		matrix_TCtoI_1.at(4), matrix_TCtoI_1.at(5), matrix_TCtoI_1.at(6), matrix_TCtoI_1.at(7),
		matrix_TCtoI_1.at(8), matrix_TCtoI_1.at(9), matrix_TCtoI_1.at(10), matrix_TCtoI_1.at(11),
		matrix_TCtoI_1.at(12), matrix_TCtoI_1.at(13), matrix_TCtoI_1.at(14), matrix_TCtoI_1.at(15);

	// Load these into our state
	Eigen::Matrix<double, 7, 1> extrinsics_1;
	extrinsics_1.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_1.block(0, 0, 3, 3).transpose());
	extrinsics_1.block(4, 0, 3, 1) = -T_CtoI_1.block(0, 0, 3, 3).transpose() * T_CtoI_1.block(0, 3, 3, 1);

	params.camera_fisheye.insert({1, false});
	params.camera_intrinsics.insert({1, intrinsics_1});
	params.camera_extrinsics.insert({1, extrinsics_1});

#ifdef ZED
  params.camera_wh.insert({1, {672, 376}});
#else
	params.camera_wh.insert({1, {752, 480}});
#endif

	// params.state_options.max_slam_features = 0;
	params.state_options.num_cameras = 2;
	params.init_window_time = 0.75;
#ifdef ZED
  // Hand tuned
  params.init_imu_thresh = 0.5;
#else
  // EuRoC
	params.init_imu_thresh = 1.5;
#endif
	params.fast_threshold = 15;
	params.grid_x = 5;
	params.grid_y = 3;
#ifdef ZED
  // Hand tuned
  params.num_pts = 200;
#else
  params.num_pts = 150;
#endif
	params.msckf_options.chi2_multipler = 1;
	params.knn_ratio = .7;

	params.state_options.imu_avg = true;
	params.state_options.do_fej = true;
	params.state_options.use_rk4_integration = true;
	params.use_stereo = true;
	params.state_options.do_calib_camera_pose = true;
	params.state_options.do_calib_camera_intrinsics = true;
	params.state_options.do_calib_camera_timeoffset = true;

	params.dt_slam_delay = 3.0;
	params.state_options.max_slam_features = 50;
	params.state_options.max_slam_in_update = 25;
	params.state_options.max_msckf_in_update = 999;

#ifdef ZED
  // Pixel noise; ZED works with defaults values but these may better account for rolling shutter
  params.msckf_options.chi2_multipler = 2;
  params.msckf_options.sigma_pix = 5;
	params.slam_options.chi2_multipler = 2;
	params.slam_options.sigma_pix = 5;

  // IMU biases from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  params.imu_noises.sigma_a = 0.00395942;  // Accelerometer noise
  params.imu_noises.sigma_ab = 0.00072014; // Accelerometer random walk
  params.imu_noises.sigma_w = 0.00024213;  // Gyroscope noise
  params.imu_noises.sigma_wb = 1.9393e-05; // Gyroscope random walk
#else
	params.slam_options.chi2_multipler = 1;
	params.slam_options.sigma_pix = 1;
#endif

	params.use_aruco = false;

	params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
  params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");

	return params;
}

duration from_seconds(double seconds) {
	return duration{long(seconds * 1e9L)};
}

class slam2 : public plugin {
public:
	/* Provide handles to slam2 */
	slam2(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		// , _m_feats_MSCKF{sb->get_writer<features>("feats_MSCKF")}
		// , _m_feats_slam_UPDATE{sb->get_writer<features>("feats_slam_UPDATE")}
		// , _m_feats_slam_DELAYED{sb->get_writer<features>("feats_slam_DELAYED")}
		, _m_left_pts{sb->get_writer<key_points>("left_pts")}
		, _m_right_pts{sb->get_writer<key_points>("right_pts")}
		, _m_imus{sb->get_writer<imu_buffer>("imu_buffer")}
		, open_vins_estimator{manager_params}
		, imu_cam_buffer{nullptr}
	{

        // Disabling OpenCV threading is faster on x86 desktop but slower on
        // jetson. Keeping this here for manual disabling.
        // cv::setNumThreads(0);

#ifdef CV_HAS_METRICS
		std::cerr << "OPEN CV HAS METRICS IS SET OLAJSDKLFJAKSDJFKASJDFKAJSDFKJASDKFJASKDFJASK" << std::endl;
		cv::metrics::setAccount(new std::string{"-1"});
#endif
		if (!std::filesystem::create_directory(data_path)) {
            std::cerr << "Failed to create data directory.";
		}
		vio_time.open(data_path + "/vio_time.csv");

        slam_csv.open(boost::filesystem::current_path().string() + "/recorded_data/slam.csv");
	}


	virtual void start() override {
		plugin::start();
		sb->schedule<imu_cam_type_prof>(id, "imu_cam", [&](switchboard::ptr<const imu_cam_type_prof> datum, std::size_t iteration_no) {
			this->feed_imu_cam(datum, iteration_no);
		});
	}


	void feed_imu_cam(switchboard::ptr<const imu_cam_type_prof> datum, std::size_t iteration_no) {
		unsigned long long curr_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Ensures that slam doesnt start before valid IMU readings come in
		if (datum == NULL) {
			return;
		}

		// Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
		assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
		open_vins_estimator.feed_measurement_imu(duration2double(datum->time.time_since_epoch()), datum->angular_v.cast<double>(), datum->linear_a.cast<double>());
		imus.push_back(imu_type{datum->time, datum->angular_v.cast<double>(), datum->linear_a.cast<double>()});

		// If there is not cam data this func call, break early
		if (!datum->img0.has_value() && !datum->img1.has_value()) {
			return;
		} else if (imu_cam_buffer == NULL) {
			imu_cam_buffer = datum;
			return;
		}

// #ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{std::to_string(counter_2)});
		// if (iteration_no % 20 == 0) {
		cv::metrics::dump();
		counter_2++;
		// }
// #else
// #warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
// #endif

		cv::Mat img0{imu_cam_buffer->img0.value()};
		cv::Mat img1{imu_cam_buffer->img1.value()};
		// cv::imshow("img0", img0);
		// cv::waitKey(1);
		// cv::imshow("img1", img1);
		// cv::waitKey(1);
		open_vins_estimator.feed_measurement_stereo(duration2double(imu_cam_buffer->time.time_since_epoch()), img0, img1, 0, 1);

		// Get features
		// feats_MSCKF = open_vins_estimator.get_feats_MSCKF();
		// feats_slam_UPDATE = open_vins_estimator.get_feats_slam_UPDATE();
		// feats_slam_DELAYED = open_vins_estimator.get_feats_slam_DELAYED();

		good_ids_left = open_vins_estimator.get_good_ids_left();
		good_ids_right = open_vins_estimator.get_good_ids_right();
		good_left = open_vins_estimator.get_good_left();
		good_right = open_vins_estimator.get_good_right();

		unsigned long long updated_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		// vio_timestamps.push_back(datum->time.time_since_epoch().count());
		// vio_durations.push_back((updated_time - curr_time) / 1e6);
		slam_csv << (updated_time - curr_time) / 1e6 << std::endl;

		std::vector<key_point> left_pts;
		std::vector<key_point> right_pts;
		for (size_t i = 0; i < good_ids_left.size(); i++) {
			left_pts.emplace_back(key_point{good_ids_left.at(i), good_left.at(i)});
		}
		for (size_t i = 0; i < good_ids_right.size(); i++) {
			right_pts.emplace_back(key_point{good_ids_right.at(i), good_right.at(i)});
		}
		_m_left_pts.put(_m_left_pts.allocate<key_points>(
			key_points {
				imu_cam_buffer->time, 
				left_pts
			}
		));
		_m_right_pts.put(_m_right_pts.allocate<key_points>(
			key_points {
				imu_cam_buffer->time, 
				right_pts
			}
		));
		std::cout << "size of left_pts " << left_pts.size() << "\n";
		std::cout << "size of right_pts " << right_pts.size() << "\n";
		_m_imus.put(_m_imus.allocate<imu_buffer>(
				imu_buffer{imus}
			));
		if (imus.empty()){
			std::cout << "imus is empty after move\n";
		} else {
			imus.clear();
		}

		/* offload all features every time -> too large data size 
		std::vector<feature> feats_MSCKF_offload;
		std::vector<feature> feats_slam_UPDATE_offload;
		std::vector<feature> feats_slam_DELAYED_offload;
		if (feats_MSCKF.size() != 0) {
			// std::cout << "INITIALIZED\n";
			for (Feature *f : feats_MSCKF) {
				// std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_cp(f->uvs);
				// std::cout << "COPY CONSTRUCTOR WORKS\n";
				feature fo = {
					f->featid,
					f->to_delete,
					f->uvs,
					f->uvs_norm,
					f->timestamps,
					f->anchor_cam_id,
					f->anchor_clone_timestamp,
					f->p_FinA,
					f->p_FinG
				};
				feats_MSCKF_offload.emplace_back(fo);
			}
			for (Feature *f : feats_slam_UPDATE) {
				feature fo = {
					f->featid,
					f->to_delete,
					f->uvs,
					f->uvs_norm,
					f->timestamps,
					f->anchor_cam_id,
					f->anchor_clone_timestamp,
					f->p_FinA,
					f->p_FinG
				};
				feats_slam_UPDATE_offload.emplace_back(fo);
			}
			for (Feature *f : feats_slam_DELAYED) {
				feature fo = {
					f->featid,
					f->to_delete,
					f->uvs,
					f->uvs_norm,
					f->timestamps,
					f->anchor_cam_id,
					f->anchor_clone_timestamp,
					f->p_FinA,
					f->p_FinG
				};
				feats_slam_DELAYED_offload.emplace_back(fo);
			}
			_m_feats_MSCKF.put(_m_feats_MSCKF.allocate<features>(
                features {
                    feats_MSCKF.size(),
					imu_cam_buffer->time,
					feats_MSCKF_offload
                }
       		));
			_m_feats_slam_UPDATE.put(_m_feats_slam_UPDATE.allocate<features>(
				features {
					feats_slam_UPDATE.size(),
					imu_cam_buffer->time,
					feats_slam_UPDATE_offload
				}
       		));
			_m_feats_slam_DELAYED.put(_m_feats_slam_DELAYED.allocate<features>(
				features {
					feats_slam_DELAYED.size(),
					imu_cam_buffer->time,
					feats_slam_DELAYED_offload
				}
       		));
			
			_m_imus.put(_m_imus.allocate<imu_buffer>(
				imu_buffer{imus}
			));
			if (imus.empty()){
				std::cout << "imus is empty after move\n";
			} else {
				imus.clear();
			}
		
		} else {

		}
		*/

		// I know, a priori, nobody other plugins subscribe to this topic
		// Therefore, I can const the cast away, and delete stuff
		// This fixes a memory leak.
		// -- Sam at time t1
		// Turns out, this is no longer correct. debbugview uses it
		// const_cast<imu_cam_type*>(imu_cam_buffer)->img0.reset();
		// const_cast<imu_cam_type*>(imu_cam_buffer)->img1.reset();
		imu_cam_buffer = datum;
	}

private:
	const std::string data_path = std::filesystem::current_path().string() + "/recorded_data";

    std::ofstream vio_time;
	std::ofstream slam_csv;

	const std::shared_ptr<switchboard> sb;
	// switchboard::writer<features> _m_feats_MSCKF;
	// switchboard::writer<features> _m_feats_slam_UPDATE;
	// switchboard::writer<features> _m_feats_slam_DELAYED;
	switchboard::writer<key_points> _m_left_pts;
	switchboard::writer<key_points> _m_right_pts;
	switchboard::writer<imu_buffer> _m_imus;

	// int counter = 0;
	int counter_2 = 0;

	VioManagerOptions manager_params = create_params();
	VioManager open_vins_estimator;

	switchboard::ptr<const imu_cam_type_prof> imu_cam_buffer;

	std::vector<pose_type> poses;

	// std::vector<long> vio_timestamps;
	// std::vector<double> vio_durations;

	// std::vector<Feature*> feats_MSCKF;
	// std::vector<Feature*> feats_slam_UPDATE;
	// std::vector<Feature*> feats_slam_DELAYED;

	std::vector<size_t> good_ids_left, good_ids_right;
	std::vector<cv::KeyPoint> good_left, good_right;

	std::vector<imu_type> imus;
};

PLUGIN_MAIN(slam2)
