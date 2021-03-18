/*!
  \example tutorial-ros-pioneer-visual-servo.cpp

  Example that shows how to control the Pioneer mobile robot by IBVS visual servoing with respect to a blob.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point corresponding to the blob center of gravity measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is proportional to the depth Z.

  */

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visual_servo/vpSimulatorPioneer.h>
#include <visp_ros/vpROSGrabber.h>
// #include <visp_ros/vpROSRobotPioneer.h>

#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
#  define TEST_COULD_BE_ACHIEVED
#endif

#ifdef TEST_COULD_BE_ACHIEVED
void stop( );
//Sort rules 
bool op(vpDot2&a,vpDot2&b){
  
	return (a.getCog().get_i()+a.getCog().get_j())< (b.getCog().get_i()+b.getCog().get_j());
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "velocity_publisher");

    // 创建节点句柄
   ros::NodeHandle n;

  // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
   ros::Publisher pionner_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   geometry_msgs::Twist vel_msg;

  try {
    vpImage<unsigned char> I, D; // Create a gray level image container
    double depth = 1.;
    double lambda = 0.6;
    double coef = 0.0358;// Scale parameter used to estimate the depth Z of the blob from its surface
    unsigned int j =0, k=0, l=0;
    vpSimulatorPioneer robot;
    // robot.setSamplingTime(0.040);//设置采样时间
    // Camera parameters. In this experiment we don't need a precise calibration of the camera
    vpCameraParameters cam;

    // Create a grabber based on libdc1394-2.x third party lib (for firewire cameras under Linux)
    vpROSGrabber g;
    g.setCameraInfoTopic("/camera_rgb/camera_info");
    g.setImageTopic("/camera_rgb/image_raw");
    g.setRectify(true);

    // Set camera parameters 
    cam.initPersProjWithoutDistortion(600,600,I.getWidth()/2, I.getHeight()/2);
    g.open(I);
    g.acquire(I);

    // Create an image viewer
    vpDisplayX c(I, 10, 10, "Current frame");
    vpDisplay::display(I);
    vpDisplay::flush(I);
    std::string filename = "../desired_pose.png"; 
    // vpImageIo::write(I, filename); 
    vpImageIo::read(D, filename); 

    vpDisplayX d(D, 0, 0, "Target  frame");
    vpDisplay::display(D);
    vpDisplay::flush(D);

    vpDot2 blob_d;
    blob_d.setGraphics(true);
    blob_d.setGraphicsThickness(1);
    blob_d.initTracking(D);
    blob_d.track(D);

    std::list<vpDot2> blob_d_list;
    blob_d.searchDotsInArea(D, 0, 0, D.getWidth(), D.getHeight(), blob_d_list);
    blob_d_list.sort(op);
    vpDisplay::flush(D);
    
    // Create a blob tracker
   vpDot2 blob;
   std::list<vpDot2>  blob_list;
    blob.setGraphics(true);
    blob.setGraphicsThickness(1);
    blob.initTracking(I);
    blob.track(I);
    blob.searchDotsInArea(I, 0, 0,I.getWidth(), I.getHeight(), blob_list);
    blob_list.sort(op);
    vpDisplay::flush(I);
    std::vector<vpDot2> dot(4);

     for (std::list<vpDot2>::iterator it = blob_list.begin(); it != blob_list.end(); ++it) {
        dot[l]=(*it);
         l+= 1;  

      }

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;
    task.setLambda(lambda) ;
    vpVelocityTwistMatrix cVe ;
    cVe = robot.get_cVe() ;
    task.set_cVe(cVe) ;

    std::cout << "cVe: \n" << cVe << std::endl;

    vpMatrix eJe;
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;
    std::cout << "eJe: \n" << eJe << std::endl;

    std::cout <<"dot_size:"<<dot[0].m00<<std::endl ;

    // Current and desired visual feature associated to the x coordinate of the point
    vpFeaturePoint s_x[4], s_xd[4];
    
    double surface_d[4],  Zd[4];
    // Create the desired x* visual feature
     for (std::list<vpDot2>::iterator it = blob_d_list.begin(); it != blob_d_list.end(); ++it) {
        vpFeatureBuilder::create(s_xd[j], cam,(*it));
        surface_d[j]= 1./sqrt((*it).m00/(cam.get_px()*cam.get_py()));
        Zd[j] = coef * surface_d[j] ;
        s_xd[j].set_Z(Zd[j]);
        std::cout <<"s_xd_x:"<<s_xd[j].get_x()<<std::endl ;
        std::cout <<"s_xd_y:"<<s_xd[j].get_y()<<std::endl ;
        std::cout <<"s_xd_Z:"<<s_xd[j].get_Z()<<std::endl ;
        j += 1;  
      }

    // Create the current log(Z/Z*) visual feature
    vpFeatureDepth s_Z[4], s_Zd[4];
    // Surface of the blob estimated from the image moment m00 and converted in meters
    double surface[4], Z[4];
    for(unsigned int i =0; i<4; i++){
      surface[i] =   1./sqrt(dot[i].m00/(cam.get_px()*cam.get_py()));
      Z [i]= coef * surface[i] ;
    }
    
    // Initial depth of the blob in from of the camera
    
    // Desired depth Z* of the blob. This depth is learned and equal to the initial depth

    for (unsigned int i =0; i<4; i++){
    // Create the current x visual feature
    vpFeatureBuilder::create(s_x[i], cam, dot[i]);

    // Add the feature
    task.addFeature(s_x[i], s_xd[i]) ;

    s_Z[i].buildFrom(s_x[i].get_x(), s_x[i].get_y(), Z[i], 0); // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Zd[i].buildFrom(s_x[i].get_x(), s_x[i].get_y(), Zd[i] ,0); // log(Z/Z*) = 0 that's why the last parameter is 0

    // // Add the feature
    task.addFeature(s_Z[i], s_Zd[i]) ;
    }
    vpPlot graph(3, 800, 1000, 400, 10, "Curves...");//创建图像用于显示曲线
    
    // Init the curve plotter
    graph.initGraph(0, 2);
    graph.initGraph(1, 3);
    graph.initGraph(2, 2);
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "Depth");
    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    graph.setLegend(1, 0, "x[0]");
    graph.setLegend(1, 1, "y[0]");
    graph.setLegend(1, 2, "log(Z[0]/Z*)");
    // graph.setLegend(1, 3, "x[1]");
    // graph.setLegend(1, 4, "y[1]");
    // graph.setLegend(1, 5, "log(Z[1]/Z*)");
    // graph.setLegend(1, 6, "x[2]");
    // graph.setLegend(1, 7, "y[2]");
    // graph.setLegend(1, 8, "log(Z[2]/Z*)");
    // graph.setLegend(1, 9, "x[3]");
    // graph.setLegend(1, 10, "y[3]");
    // graph.setLegend(1, 11, "log(Z[3]/Z*)");
    graph.setLegend(2, 0, "Z[0]");
    graph.setLegend(2, 1, "Z[0]*");
    // graph.setLegend(2, 2, "Z[1]");
    // graph.setLegend(2, 3, "Z[1]*");
    // graph.setLegend(2, 4, "Z[2]");
    // graph.setLegend(2, 5, "Z[2]*");
    // graph.setLegend(2, 6, "Z[3]");
    // graph.setLegend(2, 7, "Z[3]*");
    int iter = 0;


    vpColVector v; // vz, wx

    while(1)
    {
      // Acquire a new image
      g.acquire(I);
      vpDisplay::display(I);
      unsigned int i =0;

      for (std::list<vpDot2>::iterator it = blob_list.begin(); it != blob_list.end(); ++it) {
        (*it).setGraphics(true);
        (*it).setGraphicsThickness(3);
        (*it).track(I);
        dot[i]=(*it);
              // Update the current x feature
      vpFeatureBuilder::create(s_x[i] ,cam, dot[i]);
      
     // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
      surface[i] = 1./sqrt(dot[i].m00/(cam.get_px()*cam.get_py()));
      Z[i] = coef * surface[i] ;
      s_Z[i].buildFrom(s_x[i].get_x(), s_x[i].get_y(), Z[i], log(Z[i]/Zd[i])) ;
      s_x[i].set_Z(Z[i]);
      std::cout <<"s_x_"<<i<<":"<<s_x[i].get_x()<<" s_xd_"<<i<<":"<<s_xd[i].get_x()<<std::endl;
      std::cout <<"s_y_"<<i<<":"<<s_x[i].get_y()<<" s_yd_"<<i<<":"<<s_xd[i].get_y()<<std::endl ;
      std::cout <<"s_Z_"<<i<<":"<<s_x[i].get_Z()<<" s_Zd_"<<i<<":"<<s_xd[i].get_Z()<<std::endl ;
      std::cout << "dot_cog_x_ "<<i<<":" <<  dot[i].getCog().get_u()
                        << " dot_cog_y_ " <<i<<":" << dot[i].getCog().get_v()
                        << std::endl;
      i += 1;
      }

      robot.get_cVe(cVe) ;
      task.set_cVe(cVe) ;

      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      // Compute the control law. Velocities are computed in the mobile robot reference frame
      v = task.computeControlLaw() ;

      std::cout <<"Error:"<<task.getError()<<std::endl;
      std::cout <<"Error_sum:"<<task.getError().sumSquare()<<std::endl;

      // Send the velocity to the robot
      vel_msg.linear.x =v[0];
      vel_msg.angular.z =vpMath::deg(v[1]);

        // 发布消息
      pionner_vel_pub.publish(vel_msg);
      ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
          vel_msg.linear.x, vel_msg.angular.z);

      // // Draw a vertical line which corresponds to the desired x coordinate of the dot cog
      //  vpDisplay::displayLine(I,0, 300, 600, 300 , vpColor::green);
      vpDisplay::displayCross(I, s_xd[0].get_y()*600,s_xd[0].get_x()*600, 10, vpColor::green);
      vpDisplay::displayCross(I, s_xd[1].get_y()*600,s_xd[1].get_x()*600, 10, vpColor::green);
      vpDisplay::displayCross(I, s_xd[2].get_y()*600,s_xd[2].get_x()*600, 10, vpColor::green);
      vpDisplay::displayCross(I, s_xd[3].get_y()*600,s_xd[3].get_x()*600, 10,  vpColor::green);
    
      vpDisplay::flush(I);
      vpDisplay::flush(D);
      graph.plot(0, iter, v);               // plot velocities applied to the robot
      // graph.plot(1, iter, task.getError()); // plot error vector
      graph.plot(1, 0, iter, task.getError()[0]); // plot error vector
      graph.plot(1, 1, iter, task.getError()[1]); // plot error vector
      graph.plot(1, 2, iter, task.getError()[2]); // plot error vector
      graph.plot(2, 0, iter, Z[0]);            // plot the depth
      graph.plot(2, 1, iter, Zd[0]);            // plot the depth
      // graph.plot(2, 2, iter, Z[1]);            // plot the depth
      // graph.plot(2, 3, iter, Zd[1]); 
      // graph.plot(2, 4, iter, Z[2]);            // plot the depth
      // graph.plot(2, 5, iter, Zd[2]); 
      // graph.plot(2, 6, iter, Z[3]);            // plot the depth
      // graph.plot(2, 7, iter, Zd[3]); 
      
      iter++;
      vpTime::wait(0.004* 1000);
      // A click in the viewer to exit
      if ( vpDisplay::getClick(I, false) )
        break;
      if (sqrt(task.getError().sumSquare() )< 0.01) {
        std::cout << "Reached a small error. We stop the loop... " << std::endl;
        std::cout << "Ending robot thread..." << std::endl;
        vel_msg.linear.x =0;
        vel_msg.angular.z = 0;
        // 发布消息
        pionner_vel_pub.publish(vel_msg);
        std::cout << "Robot  is stopped" << std::endl;
        break;
      }
    };
    std::cout << "Ending robot thread..." << std::endl;
      vel_msg.linear.x =0;
      vel_msg.angular.z = 0;

        // 发布消息
      pionner_vel_pub.publish(vel_msg);
      std::cout << "Robot  is stopped" << std::endl;
    // Kill the servo task
    graph.saveData(0, "./v2.dat");
    graph.saveData(1, "./error2.dat");
    const char *legend = "Click to quit...";
    vpDisplay::displayText(graph.I, (int)graph.I.getHeight() - 60, (int)graph.I.getWidth() - 150, legend, vpColor::red);
    vpDisplay::flush(graph.I);
    vpDisplay::getClick(graph.I);

    task.print() ;
    task.kill();
  }

  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    vel_msg.linear.x =0;
     vel_msg.angular.z = 0;

        // 发布消息
    pionner_vel_pub.publish(vel_msg);
    std::cout << "Robot  is stopped" << std::endl;
    
    return 1;
  }
}

#else
int main()
{
  std::cout << "You don't have the right 3rd party libraries to run this example..." << std::endl;
}
#endif
