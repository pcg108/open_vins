#include <functional>

#include <opencv2/core.hpp>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <utility>

#include "core/VioManager.h"
#include "state/State.h"

#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format.hpp"
#include "illixr/opencv_data_types.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/relative_clock.hpp"
#include "illixr/read_hpm.h"

#include "illixr/offline_cam.hpp"

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
		: plugin{std::move(name_), pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_rtc{pb->lookup_impl<RelativeClock>()}
		, _m_pose{sb->get_writer<pose_type>("slow_pose")}
		, _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
		, _m_cam{pb->lookup_impl<offline_cam>()}
		, open_vins_estimator{manager_params}
	{

        // Disabling OpenCV threading is faster on x86 desktop but slower on
        // jetson. Keeping this here for manual disabling.
        // cv::setNumThreads(0);

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{"-1"});
#endif

	}


	void start() override {
		plugin::start();
		sb->schedule<imu_type>(id, "imu", [&](const switchboard::ptr<const imu_type>& datum, std::size_t iteration_no) {
			this->feed_imu_cam(datum, iteration_no);
		});
	}


	void feed_imu_cam(const switchboard::ptr<const imu_type>& datum, [[maybe_unused]]std::size_t iteration_no) {

		read_counters(counters_before);

		// Ensures that slam doesnt start before valid IMU readings come in
		if (datum == nullptr) {
			return;
		}

		// Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
		open_vins_estimator.feed_measurement_imu(duration2double(datum->time.time_since_epoch()), datum->angular_v, datum->linear_a);

		auto cam = _m_cam->get_cam_reading(datum->imu_real_time);
		// If there is not cam data this func call, break early
		if (!cam) {
			return;
		}

		if (!cam_buffer) {
			cam_buffer = cam;
			return;
		}


#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
		if (iteration_no % 20 == 0) {
			cv::metrics::dump();
		}
#else
#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

		cv::Mat img0{cam_buffer->img0};
		cv::Mat img1{cam_buffer->img1};

		auto duration = cam_buffer->time.time_since_epoch();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

		// std::cout << "[open-vins] the time of this camera sample is: " << ms.count() << std::endl;
		open_vins_estimator.feed_measurement_stereo(duration2double(cam_buffer->time.time_since_epoch()), img0, img1, 0, 1);

		// Get the pose returned from SLAM
		state = open_vins_estimator.get_state();
		Eigen::Vector4d quat = state->_imu->quat();
		Eigen::Vector3d vel = state->_imu->vel();
		Eigen::Vector3d pose = state->_imu->pos();

		Eigen::Vector3f swapped_pos = Eigen::Vector3f{float(pose(0)), float(pose(1)), float(pose(2))};
		Eigen::Quaternionf swapped_rot = Eigen::Quaternionf{float(quat(3)), float(quat(0)), float(quat(1)), float(quat(2))};
		Eigen::Quaterniond swapped_rot2 = Eigen::Quaterniond{(quat(3)), (quat(0)), (quat(1)), (quat(2))};

       	assert(isfinite(swapped_rot.w()));
        assert(isfinite(swapped_rot.x()));
        assert(isfinite(swapped_rot.y()));
        assert(isfinite(swapped_rot.z()));
        assert(isfinite(swapped_pos[0]));
        assert(isfinite(swapped_pos[1]));
        assert(isfinite(swapped_pos[2]));

		if (open_vins_estimator.initialized()) {
			_m_pose.put(_m_pose.allocate(
				cam_buffer->time,
				swapped_pos,
				swapped_rot
			));

			_m_imu_integrator_input.put(_m_imu_integrator_input.allocate(
				cam_buffer->time,
				from_seconds(state->_calib_dt_CAMtoIMU->value()(0)),
				imu_params{
					.gyro_noise = 0.00016968,
					.acc_noise = 0.002,
					.gyro_walk = 1.9393e-05,
					.acc_walk = 0.003,
					.n_gravity = Eigen::Matrix<double,3,1>(0.0, 0.0, -9.81),
					.imu_integration_sigma = 1.0,
					.nominal_rate = 200.0,
				},
				state->_imu->bias_a(),
				state->_imu->bias_g(),
				pose,
				vel,
				swapped_rot2
			));
		}
		cam_buffer = cam;

		read_counters(counters_after);
        std::cout << "openvins: " << diff_to_string(counters_after, counters_before) << std::endl;
	}

	~slam2() override = default;

private:
	const std::shared_ptr<switchboard> sb;
	std::shared_ptr<RelativeClock> _m_rtc; 
	switchboard::writer<pose_type> _m_pose;
	switchboard::writer<imu_integrator_input> _m_imu_integrator_input;
	State *state{};

	std::optional<cam_type> cam_buffer;
	std::shared_ptr<offline_cam>  _m_cam;

	VioManagerOptions manager_params = create_params();
	VioManager open_vins_estimator;

	double counters_before[29] = {0.0};
    double counters_after[29] = {0.0};
};

PLUGIN_MAIN(slam2)
