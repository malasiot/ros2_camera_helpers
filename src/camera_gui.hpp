#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QListView>
#include <QStringListModel>
#include <QComboBox>
#include <QAction>

#include <opencv2/opencv.hpp>

#include "image_widget.hpp"

class CameraGrabber ;

class CameraViewer: public QMainWindow
{
    Q_OBJECT

public:
    enum Channel { CHANNEL_RGB, CHANNEL_DEPTH } ;

    CameraViewer(std::shared_ptr<CameraGrabber> grabber, QWidget *parent = 0);

    void setImage(const cv::Mat &im) ;

    Channel getChannel() const ;

    void setStatus(const std::string &msg) ;

private Q_SLOTS:
    void onRecordingStateChanged() ;

private:
    void enableButtons(bool enable) ;

    QComboBox *channel_combo_, *rgb_topics_, *depth_topics_, *camera_topics_ ;
    QImageWidget *image_widget_ ;
    QAction *record_action_ ;
    std::shared_ptr<CameraGrabber> grabber_ ;

 };

