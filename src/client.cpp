//
// Created by zxj on 2024/4/23.
//

/*
    需求:
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    服务器实现:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建 客户端 对象
        5.请求服务，接收响应

*/
// 1.包含头文件
#include "ros/ros.h"
#include "../../../devel/include/laser_perception/status.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");



    // 2.初始化 ROS 节点
    ros::init(argc,argv,"status_Client");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 客户端 对象
    ros::ServiceClient client = nh.serviceClient<laser_perception::status>("detect_status");
    //等待服务启动成功
    //方式1
    ros::service::waitForService("detect_status");
    //方式2
    // client.waitForExistence();
    // 5.组织请求数据
    laser_perception::status ai;
    ai.request.request_status=std::atoi(argv[1]);
    // 6.发送请求,返回 bool 值，标记是否成功
    bool flag = client.call(ai);
//    // 7.处理响应
//    if (flag)
//    {
//        ROS_INFO("请求正常处理,响应结果:%d",ai.response.do_request_status);
//    }
//    else
//    {
//        ROS_ERROR("请求处理失败....");
//        return 1;
//    }

    return 0;
}
