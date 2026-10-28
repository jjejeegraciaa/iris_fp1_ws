#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        // namedWindow("Control", WINDOW_AUTOSIZE);

        // // Trackbar menggunakan variabel anggota kelas
        // createTrackbar("Hmin", "Control", &h_min_, 179);
        // createTrackbar("Hmax", "Control", &h_max_, 179);

        // createTrackbar("Smin", "Control", &s_min_, 255);
        // createTrackbar("Smax", "Control", &s_max_, 255);

        // createTrackbar("Vmin", "Control", &v_min_, 255);
        // createTrackbar("Vmax", "Control", &v_max_, 255);

        // Perbaikan: "create_subcription" diubah menjadi "create_subscription" 
        // dan variabel pointer subscription disamakan dengan deklarasi di bawah (`image_sub_`)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front",
            10,
            std::bind(
                &VisionNode::imageCallback,
                this,
                std::placeholders::_1
            )
        );

        RCLCPP_INFO(get_logger(), "Node BEV berjalan");
    }    

private:
    int h_min_ = 19;
    int h_max_ = 92;

    int s_min_ = 25;
    int s_max_ = 98;

    int v_min_ = 118;
    int v_max_ = 246;

    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const cv::Mat& frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Menerima frame kosong!");
                return;
            }

            cv::Mat frame_hsv, frame_threshold, bev ;
            
            // Konversi BGR ke HSV
            cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

            int bev_width_ = frame.cols; 
            int bev_height_ = frame.rows;

            std::vector<cv::Point2f> src_pts = {
                cv::Point2f(360, 452),
                cv::Point2f(633, 455),
                cv::Point2f(785, 621),
                cv::Point2f(2, 541)
            };

            std::vector<cv::Point2f> dst_pst = {
                cv::Point2f(0.0f),
                cv::Point2f(static_cast<float>(bev_width_ - 1), 0.0f),           // Kanan-Atas
                cv::Point2f(static_cast<float>(bev_width_ - 1), static_cast<float>(bev_height_ - 1)), // Kanan-Bawah
                cv::Point2f(0.0f, static_cast<float>(bev_height_ - 1))

            };

            cv::Mat matrix = cv::getPerspectiveTransform(src_pts, dst_pst);
            cv::warpPerspective(frame, bev, matrix, cv::Size(bev_width_ , bev_height_));
            
            cv::Mat bev_hsv, bev_threshold;
            cv::cvtColor(bev , bev_hsv , cv::COLOR_BGR2HSV);

            // Menggunakan nilai dari trackbar kelas untuk inRange
            cv::inRange(
                bev_hsv, 
                cv::Scalar(h_min_, s_min_, v_min_), 
                cv::Scalar(h_max_, s_max_, v_max_), 
                bev_threshold
            );

            // cv::Mat contours;

            // std:vector<std::vector<cvPoint>> contours;
            // cv::findContours(bev_threshold, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // cv::Mat result = frame.clone();

            // for (size_t i = 0 ; i < contours.size() ; i++){
            //     if (cv::contourArea(contours[i]) > 1000) {
            //         cv::Rect boundingBox = cv::boundingRect(contours[i])
            //         cv::rectangle(result, boundingBox, cv::Scalar(0, 255, 0), 2);
            //     }
            // }

            // Tampilkan gambar asli dan hasil threshold
            cv::imshow("RAW", frame);
            cv::imshow("BEV", bev);
            cv::imshow("Threshold", bev_threshold);
            //cv::imshow("Countours", result);

            cv::waitKey(1);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Gagal memproses gambar: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
