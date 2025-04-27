#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <camera_driver_srvs/srv/configure_camera.hpp>
#include <camera_driver_srvs/srv/reconfigure_defaults.hpp>
#include "../include/camera_driver/cameramanager/CameraManager.hpp"
#include <camera_driver/camera_driver_parameters.hpp>
#include "../include/camera_driver/helpers/util.hpp"

class CameraDriverNode : public rclcpp::Node {

    public:
        CameraDriverNode() : Node("camera_driver_node"){}

        void initialize(std::shared_ptr<Node> node){
            param_listener = std::make_shared<camera_driver::ParamListener>(node);
            params = param_listener->get_params();

            if(params.camera_path == "unused"){
                RCLCPP_INFO(this->get_logger(), std::string("Opening Camera on: " + std::to_string(params.camera_index)).c_str());
                camera = CameraManager(params.camera_index, cv::CAP_V4L2);
            }
            else{
                RCLCPP_INFO(this->get_logger(), std::string("Opening Camera on: " + params.camera_path).c_str());
                camera = CameraManager(params.camera_path, cv::CAP_V4L2);
            }

            D = params.distortion_coeffs;

            K = util::Util::getK(D);

            R = util::Util::getR(D);

            P = util::Util::getP(D);

            if(params.resolution[0] != -1 && params.resolution[1] != -1 && params.image_fps != -1)
                camera.config(params.resolution[0], params.resolution[1], params.image_fps);

            image_raw_publisher = this->create_publisher<sensor_msgs::msg::Image>(params.camera_name + "/image_raw", 10);
            image_compressed_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(params.camera_name + "/image_raw/compressed", 10);
            camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(params.camera_name + "/camera_info", 10);

            timer = this->create_wall_timer(
                std::chrono::milliseconds(int((1.0 / params.publish_rate) * 1000)),
                std::bind(&CameraDriverNode::publish_data, this)
            );

            config_service = this->create_service<camera_driver_srvs::srv::ConfigureCamera>(
                params.camera_name + "/configure_camera",
                std::bind(&CameraDriverNode::handle_config, this, std::placeholders::_1, std::placeholders::_2)
            );

            reconfig_service = this->create_service<camera_driver_srvs::srv::ReconfigureDefaults>(
                params.camera_name + "/reconfigure_defaults",
                std::bind(&CameraDriverNode::handle_reconfig, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
    /**
     * Sets property of camera. False means fail.
     */
        bool set_camera_property_ack(cv::VideoCaptureProperties property, double value, int repeat){
            
            bool status = camera.setPropertyAck(property, value, repeat);

            std::string message = status ? "Successfully Configured: " : "Failed to Configure: ";

            switch(property){
                case cv::CAP_PROP_FRAME_WIDTH:
                    message += "WIDTH";
                    break;

                case cv::CAP_PROP_FRAME_HEIGHT:
                    message += "HEIGHT";
                    break;

                case cv::CAP_PROP_FPS:
                    message += "FPS";
                    break;

                case cv::CAP_PROP_FOURCC:
                    message += "FOURCC";
                    break;
            }

            RCLCPP_INFO(this->get_logger(), message.c_str());

            return status;        
        }

        bool config_camera(double width, double height, double fourcc, double fps, int repeat){

            RCLCPP_INFO(this->get_logger(), "Configuring with Values: width=%d, height=%d, fourcc=%d fps=%d",
                (int)width, (int)height, (int)fourcc, (int)fps
            );

            bool all_successful = true;

            if(!set_camera_property_ack(cv::CAP_PROP_FRAME_WIDTH, width, repeat))
                all_successful = false;

            if(!set_camera_property_ack(cv::CAP_PROP_FRAME_HEIGHT, height, repeat))
                all_successful = false;
            
            if(!set_camera_property_ack(cv::CAP_PROP_FOURCC, fourcc, repeat))
                all_successful = false;

            if(!set_camera_property_ack(cv::CAP_PROP_FPS, fps, repeat))
                all_successful = false;

            RCLCPP_INFO(this->get_logger(), "Configuration Status: %s", all_successful ? "success" : "failed");

            return all_successful;
        }

        void handle_reconfig(const std::shared_ptr<camera_driver_srvs::srv::ReconfigureDefaults::Request> request, 
            std::shared_ptr<camera_driver_srvs::srv::ReconfigureDefaults::Response> response){

            response->status = config_camera(
                params.resolution[0],
                params.resolution[1],
                cv::VideoWriter::fourcc('M','J','P','G'),
                params.image_fps,
                20
            );
        }

        void handle_config(const std::shared_ptr<camera_driver_srvs::srv::ConfigureCamera::Request> request, 
            std::shared_ptr<camera_driver_srvs::srv::ConfigureCamera::Response> response){

            response->status = config_camera(
                request->width,
                request->height,
                cv::VideoWriter::fourcc('M','J','P','G'),
                request->fps,
                20
            );
        }

        void publish_data(){
            cv::Mat frame = params.publish_as_gray ? camera.getFrameGray() : camera.getFrame();

            if(frame.empty())
                return;

            std_msgs::msg::Header header = std_msgs::msg::Header();
                header.stamp = this->now();
                header.frame_id = params.camera_frame_id; 

            cv_bridge::CvImage message = cv_bridge::CvImage(
                header, 
                params.publish_as_gray ? "mono8" : "bgr8", 
                frame
            );

            image_raw_publisher->publish(*message.toImageMsg());

            if(params.publish_compressed)
                image_compressed_publisher->publish(*message.toCompressedImageMsg(cv_bridge::Format::JPEG));

            if(params.publish_camera_info){
                sensor_msgs::msg::CameraInfo info = sensor_msgs::msg::CameraInfo();

                info.header = header;
                info.width = frame.cols;
                info.height = frame.rows;

                info.distortion_model = "plumb_bob";

                info.k = K;
                info.d = D;
                info.r = R;
                info.p = P;

                camera_info_publisher->publish(info);
            }
        }

        CameraManager camera;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Service<camera_driver_srvs::srv::ConfigureCamera>::SharedPtr config_service;
        rclcpp::Service<camera_driver_srvs::srv::ReconfigureDefaults>::SharedPtr reconfig_service;

        std::shared_ptr<camera_driver::ParamListener> param_listener;
        camera_driver::Params params;
        
        std::vector<double> D;
        std::array<double, 9> K;
        std::array<double, 9> R;
        std::array<double, 12> P;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    std::shared_ptr node = std::make_shared<CameraDriverNode>();

    node->initialize(node);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}