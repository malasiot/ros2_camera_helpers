#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QListView>
#include <QStringListModel>
#include <QComboBox>

#include <opencv2/opencv.hpp>

#include "image_widget.hpp"


class CameraViewer: public QMainWindow
{
    Q_OBJECT

public:
    enum Channel { CHANNEL_RGB, CHANNEL_DEPTH } ;

    CameraViewer(QWidget *parent = 0);

    void setImage(const cv::Mat &im) ;

    Channel getChannel() const ;

private Q_SLOTS:

private:
    void enableButtons(bool enable) ;

    QComboBox *channel_combo_, *rgb_topics_, *depth_topics_, *camera_topics_ ;
    QImageWidget *image_widget_ ;

 };

