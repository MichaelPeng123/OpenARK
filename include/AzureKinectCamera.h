#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include "concurrency.h"
#include <atomic>

// OpenARK Libraries
#include "CameraSetup.h"

namespace ark {
    /**
    * Class defining the behavior of an Azure Kinect (K4A) Camera.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class AzureKinectCamera : public CameraSetup
    {
	public:

		/**
		* Public constructor initializing the Azure Kinect Camera.
		* @param device_id camera device id. 0 is default.
		* @param wide_fov_mode if true, starts Azure Kinect in wide FOV depth mode
		* @param use_1080p if true, records in 1080p rather than 720p
		* @param scale amount to scale down final image by
		*/
		explicit AzureKinectCamera(uint32_t device_id = 0,
			bool wide_fov_mode = false,
			bool use_1080p = false,
			double scale = 0.5);

		/**
		* Destructor for the Azure Kinect Camera.
		*/
		~AzureKinectCamera() override;
		/**
		* Get the camera's model name.
		*/
		const std::string getModelName() const override;


		/**
		* Returns the size of image
		*/
		cv::Size getImageSize() const override;

		/**
		* Sets the external hardware sync ans starts the camera
		*/
		void start() override;

		void update(MultiCameraFrame & frame) override;

		std::vector<float> getColorIntrinsics() override;

		void project(cv::Mat & xyz_map);

		bool getImuToTime(double timestamp, std::vector<ImuPair>& data_out);

		void imuReader();

		/**Beyond this point are methods that are not part of CameraSetup
		but and were originally part of the AzureKinectCamera class

		Function: Returns default detection parameters for this depth camera class:
		const DetectionParams::Ptr & getDefaultParams() const override;


		Function: Preferred frame height
		const int PREFERRED_FRAME_H = 480;

		Function: Shared pointer to Azure Kinect camera instance
		typedef std::shared_ptr<AzureKinectCamera> Ptr;

		Function: Get the timestamp of the last image in nanoseconds
		uint64_t getTimestamp();

		Function: Get the basic calibration intrinsics @return (fx, cx, fy, cy)
		cv::Vec4d getCalibIntrinsics();
		*/
	protected:
		/**
		* Gets the new frame from the sensor (implements functionality).
		* Updates xyzMap and ir_map.
		*/

		/**
		* Initialize the camera, opening channels and resetting to initial configurations
		*/
		//void initCamera();

		// internal storage

		double scale;
		int width, height, scaled_width, scaled_height;
		double last_ts_g;

		// Kinect Azure device (k4a_device_t)
		void * k4a_device = NULL;

		// Kinect Azure depth/color transformation (k4a_transformation_t)
		void * k4a_transformation = NULL;

		// Cached XY position multiplier: multiply by current depth to get XY position (k4a_image_t *)
		void * xy_table_cache = NULL;

		const int32_t TIMEOUT_IN_MS = 1000;

		mutable bool defaultParamsSet = false;
		mutable DetectionParams::Ptr defaultParams;

		int64_t timestamp;
		/* (fx, cx, fy, cy) */
		cv::Vec4d calib_intrin;
	};
}
