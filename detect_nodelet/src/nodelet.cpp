#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace vision_detection
{
    class yolo_plug_in : public nodelet::Nodelet
    {
    private:
        virtual void onInit()
        {
            NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
        }

    public:
        yolo_plug_in(/* args */);
        ~yolo_plug_in();
    };
}

PLUGINLIB_EXPORT_CLASS(vision_detection::yolo_plug_in, nodelet::Nodelet)