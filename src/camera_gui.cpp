#include "camera_gui.hpp"
#include <QVBoxLayout>
#include <QApplication>
#include <QMainWindow>
#include <QSplitter>
#include <QShortcut>
#include <QCommandLineParser>
#include <QToolBar>
#include <QStatusBar>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include <opencv2/opencv.hpp>

#include <fstream>

#include <QDebug>

cv::Mat colorize(const cv::Mat &depth) {
    double min;
    double max;
    cv::minMaxIdx(depth, &min, &max);
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    depth.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);

    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
  return falseColorsMap;
}

using namespace std ;

class CameraGrabber: public rclcpp::Node {
public:
    CameraGrabber(const std::string &rgb_topic, const std::string &depth_topic, rclcpp::NodeOptions nh = rclcpp::NodeOptions()):
        rclcpp::Node("camera_grabber_node", nh) {
        subscribe(rgb_topic, depth_topic) ;
    }

    void setViewer(CameraViewer *viewer) {
        viewer_ = viewer ;
    }

private:

    void subscribe(const std::string &rgb_topic, const std::string &depth_topic) {

        rgb_sub_.subscribe(this, rgb_topic) ;
        depth_sub_.subscribe(this, depth_topic) ;

        sync_.reset(new Synchronizer( SyncPolicy(10), rgb_sub_, depth_sub_ ));
        sync_->registerCallback(std::bind(&CameraGrabber::frameCallback, this,std::placeholders::_1, std::placeholders::_2));

    }


    void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr colorMsg, const sensor_msgs::msg::Image::ConstSharedPtr depthMsg) {

        try
        {
            auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);

            rgb_ = colorPtr->image;
            frame_number_ ++ ;
            if ( viewer_->getChannel() == CameraViewer::CHANNEL_RGB ) {
                QMetaObject::invokeMethod(viewer_, [=]() {
                    viewer_->setImage(rgb_);
                }, Qt::ConnectionType::QueuedConnection);

            }

        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                                  __FUNCTION__, __FILE__);
        }

        try
        {
            auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

            if (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depthMsg->encoding == sensor_msgs::image_encodings::MONO16) {
                depth_ = depthPtr->image; // no conversion needed

                double min_val, max_val ;
                cv::minMaxLoc(depth_, &min_val, &max_val) ;
                cv_bridge::CvtColorForDisplayOptions options ;
                //options.bg_label = 0 ;
                options.min_image_value = min_val ;
                options.max_image_value = max_val ;
                options.colormap = cv::COLORMAP_JET ;
                auto depthViz = cv_bridge::cvtColorForDisplay(depthPtr, sensor_msgs::image_encodings::BGR8, options) ;
                if ( viewer_->getChannel() == CameraViewer::CHANNEL_DEPTH ) {
                    QMetaObject::invokeMethod(viewer_, [=]() {
                        viewer_->setImage(depthViz->image);
                    }, Qt::ConnectionType::QueuedConnection);

                }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                                  __FUNCTION__, __FILE__);
        }



    }

    cv::Mat rgb_, depth_ ;

     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
     message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_ ;
     message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_ ;

     using SyncPolicy =  typename message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ;
     using Synchronizer = message_filters::Synchronizer<SyncPolicy> ;
     std::unique_ptr<Synchronizer> sync_ ;
     size_t frame_number_ = 0 ;
     CameraViewer *viewer_ ;
};



CameraViewer::CameraViewer(QWidget *parent):
    QMainWindow(parent) {

    image_widget_ = new QImageWidget(this) ;

    setCentralWidget(image_widget_) ;

    QToolBar *tb = new QToolBar() ;

    tb->setAllowedAreas(Qt::TopToolBarArea) ;
    tb->setFloatable(false) ;
    tb->setMovable(false) ;

    channel_combo_ = new QComboBox(this) ;
    channel_combo_->addItems(QStringList() << "RGB" << "Depth") ;
    tb->addWidget(channel_combo_) ;
    addToolBar(tb);

    statusBar()->showMessage("Ready");
}


void CameraViewer::setImage(const cv::Mat &im) {
    image_widget_->setImage(im) ;
}

CameraViewer::Channel CameraViewer::getChannel() const {
    if ( channel_combo_->currentIndex() == 0 ) return CHANNEL_RGB ;
    else return CHANNEL_DEPTH ;
}


int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    QCommandLineParser parser;
    parser.setApplicationDescription("Camera viewer");
    parser.addHelpOption();
    parser.addPositionalArgument("rgb_topic", QCoreApplication::translate("main", "Topic for RGB image."));
    parser.addPositionalArgument("depth_topic", QCoreApplication::translate("main", "Topic for Depth topic."));
    parser.process(app);

    const QStringList args = parser.positionalArguments();

    if ( args.size() < 2 ) {
        parser.showHelp(0);
    }

    rclcpp::init(argc, argv);

    std::shared_ptr<CameraGrabber> grabber = std::make_shared<CameraGrabber>(args.at(0).toStdString(), args.at(1).toStdString()) ;

    CameraViewer window ;
    window.resize(512, 512) ;
    window.show() ;

    grabber->setViewer(&window) ;

    std::thread t = std::thread([&]{
        rclcpp::spin(grabber) ;
        rclcpp::shutdown();
    });


    app.exec();

    t.join();

    return 1;
}
