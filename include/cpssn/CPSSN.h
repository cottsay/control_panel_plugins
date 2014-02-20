#ifndef CPSSN_H
#define CPSSN_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/function.hpp>

namespace control_panel
{

class CPSSN : public nodelet::Nodelet
{
public:
    template<class M>
    void activate(const std::string topic, const boost::function<void(const boost::shared_ptr<const M>&)> cb, bool passive = false)
    {
        if(sub)
        {
            sub.shutdown();
            ros::NodeHandle nh = getNodeHandle();
            sub = nh.subscribe(topic, 1, cb);
        }
        else if(!passive)
        {
            ros::NodeHandle nh = getNodeHandle();
            sub = nh.subscribe(topic, 1, cb);
        }
    }

    void onInit()
    {
    }

    void deactivate()
    {
        sub.shutdown();
    }

    bool isActive()
    {
        return (sub);
    }

private:
    ros::Subscriber sub;
};
}

#endif // CPSSN_H
