#include <mrpt/math/interp_fit.hpp>
#include <mrpt/math/interp_fit.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
//#include <mrpt/comms/CSerialPort.h>
//#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/math/ransac.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/gui.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // This class is in mrpt-maps
//#include <OpenNI.h>
//#include <PS1080.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

//#include "mrpt_rawlog/RawLogRecordConfig.h"
//#include "mrpt_rawlog_record/rawlog_record.h"

//#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <stdexcept>
#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/CArray.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/math/ransac_applications.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"


//CPose3D=geometry_msgs::Pose

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::slam;
using namespace mrpt::opengl;
//using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;
using namespace Eigen;


const float vert_FOV = DEG2RAD( 4.0 );
#define DEBUG 1
#define useLIDAR 3
#define SHOW_SEGMENTATION 1
#define SHOW_CALIBRATED_SCANS 1

//#define ETH_LIDAR 0
//#define ACM_LIDAR 0
#define OPENNI_LIDAR 1

sensor_msgs::LaserScan laserscan1;
sensor_msgs::LaserScan laserscan2;
CObservation2DRangeScan obs_LIDARACM;
CObservation2DRangeScan obs_LIDARETH;
const string  out_rawlog_fil =  string("/home/snake/catkin_ws/src/CALIBRATION/currentoutput.rawlog");
mrpt::utils::CFileGZOutputStream  f_out(out_rawlog_fil);


bool convertrostomrpt(
        const sensor_msgs::LaserScan& _msg, const mrpt::poses::CPose3D& _pose,
        CObservation2DRangeScan& _obj)
{
    _obj.timestamp=mrpt::system::time_tToTimestamp(laserscan1.header.stamp.sec + laserscan1.header.stamp.nsec * 1e-9);
    //mrpt_bridge::convert(_msg.header.stamp, _obj.timestamp);
    _obj.rightToLeft = true;
    _obj.sensorLabel = _msg.header.frame_id;
    _obj.aperture = _msg.angle_max - _msg.angle_min;
    _obj.maxRange = _msg.range_max;
    _obj.sensorPose = _pose;

    ASSERT_(_msg.ranges.size() > 1);

    const size_t N = _msg.ranges.size();
    const double ang_step = _obj.aperture / (N - 1);
    const double fov05 = 0.5 * _obj.aperture;
    const double inv_ang_step = (N - 1) / _obj.aperture;

    _obj.resizeScan(N);
    for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
    {
        // ROS indices go from _msg.angle_min to _msg.angle_max, while
        // in MRPT they go from -FOV/2 to +FOV/2.
        int i_ros =
                inv_ang_step * (-fov05 - _msg.angle_min + ang_step * i_mrpt);
        if (i_ros < 0)
            i_ros += N;
        else if (i_ros >= (int)N)
            i_ros -= N;  // wrap around 2PI...

        // set the scan
        const float r = _msg.ranges[i_ros];
        _obj.setScanRange(i_mrpt, r);

        // set the validity of the scan
        const bool r_valid =
                ((_obj.scan[i_mrpt] < (_msg.range_max * 0.95)) &&
                 (_obj.scan[i_mrpt] > _msg.range_min));
        _obj.setScanRangeValidity(i_mrpt, r_valid);
    }

    return true;
}

void scan_1_sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laserscan1.angle_increment=scan->angle_increment;
    laserscan1.angle_max=scan->angle_max;
    laserscan1.angle_min=scan->angle_min;
    laserscan1.header=scan->header;
    laserscan1.header.frame_id="HOKUYO1";
    laserscan1.intensities=scan->intensities;
    laserscan1.range_max=scan->range_max;
    laserscan1.range_min=scan->range_min;
    laserscan1.ranges=scan->ranges;
    laserscan1.time_increment=scan->time_increment;
    laserscan1.scan_time=scan->scan_time;

    //obs_LIDARACM.timestamp=mrpt::system::time_tToTimestamp(laserscan1.header.stamp.sec + laserscan1.header.stamp.nsec * 1e-9);
    convertrostomrpt(laserscan1,CPose3D(0, 0, 0),obs_LIDARACM);

    f_out << obs_LIDARACM;





}
void scan_2_sub_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laserscan2.angle_increment=scan->angle_increment;
    laserscan2.angle_max=scan->angle_max;
    laserscan2.angle_min=scan->angle_min;
    laserscan2.header=scan->header;
    laserscan2.header.frame_id="HOKUYO2";
    laserscan2.intensities=scan->intensities;
    laserscan2.range_max=scan->range_max;
    laserscan2.range_min=scan->range_min;
    laserscan2.ranges=scan->ranges;
    laserscan2.time_increment=scan->time_increment;
    laserscan2.scan_time=scan->scan_time;
    convertrostomrpt(laserscan2,CPose3D(0, 0, 0),obs_LIDARETH);
    f_out << obs_LIDARETH;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multisensorcalibtor");
    ros::NodeHandle n;

    ros::Subscriber Subscriber_scan_1 = n.subscribe("\scan_1", 1, scan_1_sub_callback);
    ros::Subscriber Subscriber_scan_2 = n.subscribe("\scan_2", 1, scan_2_sub_callback);

    ros::Rate r(60);
    double LIDARfov=2;


#ifdef OPENNI_LIDAR
    COpenNI2Sensor rgbd_sensor;
    unsigned sensor_id_or_serial = 0;
    sensor_id_or_serial = atoi("/home/snake/library/mrpt-mrpt-1.5/config_file.ini");
    if (sensor_id_or_serial > 10)
    {rgbd_sensor.setSerialToOpen(sensor_id_or_serial);}
    else
    {rgbd_sensor.setSensorIDToOpen(sensor_id_or_serial);}

    rgbd_sensor.initialize();

    if(rgbd_sensor.getNumDevices() == 0)
        return 0;

    cout << "OK " << rgbd_sensor.getNumDevices() << " available devices."  << endl;
    cout << "\nUse device " << sensor_id_or_serial << endl << endl;
#endif
    mrpt::gui::CDisplayWindowPlots Laserwin_ETH("Laser scans ETH");
    mrpt::gui::CDisplayWindowPlots Laserwin_ACM("Laser scans ACM");
    //creating saving method
        // Create output directory for images ------------------------------
        const string  out_img_dir = string("/home/snake/catkin_ws/src/CALIBRATION/recordimg");
        cout << "Creating images directory: " << out_img_dir << endl;
        mrpt::system::createDirectory(out_img_dir);
        //const string  out_rawlog_fil =  string("/home/snake/catkin_ws/src/CALIBRATION/currentoutput.rawlog");
       // cout << "Creating rawlog: " << out_rawlog_fil << endl;
       // mrpt::utils::CFileGZOutputStream  f_out(out_rawlog_fil);










        // Create window and prepare OpenGL object in the scene:
        // --------------------------------------------------------
        mrpt::gui::CDisplayWindow3D  win3D("OpenNI2 3D view",800,600);
        win3D.setCameraAzimuthDeg(140);
        win3D.setCameraElevationDeg(20);
        win3D.setCameraZoom(8.0);
        win3D.setFOV(90);
        win3D.setCameraPointingToPoint(2.5,0,0);
        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->setPointSize(2.5);
        // The 2D "laser scan" OpenGL object:
        mrpt::opengl::CPlanarLaserScanPtr gl_2d_scan = mrpt::opengl::CPlanarLaserScan::Create();
        gl_2d_scan->enablePoints(true);
        gl_2d_scan->enableLine(true);
        gl_2d_scan->enableSurface(true);
        gl_2d_scan->setSurfaceColor(0,0,1, 0.3);  // RGBA
        /// create 2D view points
        opengl::COpenGLViewportPtr viewInt; // Extra viewports for the RGB images.
        {
            mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

            // Create the Opengl object for the point cloud:
            scene->insert( gl_points );
            scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
            scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
            scene->insert( gl_2d_scan );

            const double aspect_ratio =  480.0 / 640.0;
            const int VW_WIDTH = 400;	// Size of the viewport into the window, in pixel units.
            const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

            // Create an extra opengl viewport for the RGB image:
            viewInt = scene->createViewport("view2d_int");
            viewInt->setViewportPosition(5, 30, VW_WIDTH,VW_HEIGHT );
            win3D.addTextMessage(10, 30+VW_HEIGHT+10,"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

            win3D.addTextMessage(5,5,
                                 format("'o'/'i'-zoom out/in, ESC: quit"),
                                 TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );

            win3D.unlockAccess3DScene();
            win3D.repaint();
        }

        //							Grab frames continuously and show
        //========================================================================================

        bool bObs = false, bError = true;
        mrpt::system::TTimeStamp  last_obs_tim = INVALID_TIMESTAMP;

        while (!win3D.keyHit()&&!mrpt::system::os::kbhit()&&ros::ok())	//Push any key to exit // win3D.isOpen()
        {

            ros::spinOnce();
            r.sleep();
            //    cout << "Get new observation\n";
            CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();
            newObs->sensorLabel = "OpenNI2";
            newObs->hasConfidenceImage = false;
            newObs->range_is_depth = true;
            newObs->cameraParams.nrows = 640;
            newObs->cameraParams.ncols = 488;
            newObs->cameraParams.fx(572.882768);
            newObs->cameraParams.fy(542.739980);
            newObs->cameraParams.cx(( 640-1)*0.5);
            newObs->cameraParams.cy((488-1)*0.5);
            newObs->cameraParamsIntensity = newObs->cameraParams;
            newObs->relativePoseIntensityWRTDepth = mrpt::poses::CPose3D(0,0,0,0,0,0);
            newObs->hasPoints3D = false;
            newObs->timestamp = mrpt::system::getCurrentTime();
            newObs->hasRangeImage = true;
            newObs->rangeImage_forceResetExternalStorage();
            newObs->rangeImage_setSize(488,640);
            newObs->hasIntensityImage = true;
            newObs->intensityImageChannel = mrpt::obs::CObservation3DRangeScan::CH_VISIBLE;


            //float hFov = depth.getHorizontalFieldOfView();
            float fx = 572.882768;
            //float vFov = depth.getVerticalFieldOfView();
            float fy = 542.739980;



            rgbd_sensor.getNextObservation(* newObs, bObs, bError);

            CObservation2DRangeScanPtr obs_2d; // The equivalent 2D scan




    //  f_out << obs_LIDARACM;


//Pose3D(0, 0, 0);
   //             f_out << obs_LIDARETH;




           // f_out << newObs;
            if (bObs && !bError && newObs && newObs->timestamp!=INVALID_TIMESTAMP && newObs->timestamp!=last_obs_tim )
            {

                // It IS a new observation:
                last_obs_tim = newObs->timestamp;

                // Convert ranges to an equivalent 2D "fake laser" scan:
                if (newObs->hasRangeImage )
                {
                    // Convert to scan:
                    obs_2d = CObservation2DRangeScan::Create();

                    T3DPointsTo2DScanParams p2s;
                    p2s.angle_sup = .5f*vert_FOV;
                    p2s.angle_inf = .5f*vert_FOV;
                    p2s.sensorLabel = "KINECT_2D_SCAN";
                    newObs->convertTo2DScan(*obs_2d, p2s);
                }
                obs_2d->sensorLabel = "HOKUYO3";
                //obs_2d->timestamp = mrpt::system::getCurrentTime();
                //obs_2d->rightToLeft=true;
                //        obs_2d->maxRange = 5;
                //        obs_2d->aperture =1.01229;           //horzitonal field of view
                //        obs_2d->beamAperture =0.00001;       //sensor vertical fov
                //        obs_2d->sensorPose=mrpt::poses::CPose3D(0,0,0,0,0,0);;
               //         obs_2d->stdError = 0.01;

                //-----------------------------------------------------------
                //record
                //-----------------------------------------------------------
               // f_out << newObs;
                  f_out << obs_2d;
                //-----------------------------------------------------------
                //end record
                //-----------------------------------------------------------

                // Update visualization ---------------------------------------

                win3D.get3DSceneAndLock();

                // Estimated grabbing rate:
                win3D.addTextMessage(-350,-13, format("Timestamp: %s", mrpt::system::dateTimeLocalToString(last_obs_tim).c_str()), TColorf(0.6,0.6,0.6),"mono",10,mrpt::opengl::FILL, 100);

                // Show intensity image:
                if (newObs->hasIntensityImage )
                {
                    viewInt->setImageView(newObs->intensityImage); // This is not "_fast" since the intensity image may be needed later on.
                }
                win3D.unlockAccess3DScene();

                // -------------------------------------------------------
                //           Create 3D points from RGB+D data
                //
                // There are several methods to do this.
                //  Switch the #if's to select among the options:
                // See also: http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
                // -------------------------------------------------------
                if (newObs->hasRangeImage)
                {
                    // Pathway: RGB+D --> XYZ+RGB opengl
                    win3D.get3DSceneAndLock();
                    newObs->project3DPointsFromDepthImageInto(*gl_points, false /* without obs.sensorPose */);
                    win3D.unlockAccess3DScene();
                }

                // And load scan in the OpenGL object:
                gl_2d_scan->setScan(*obs_2d);


                win3D.repaint();


                mrpt::maps::CSimplePointsMap the_ETH_Map;
                the_ETH_Map.insertionOptions.minDistBetweenLaserPoints = 0;
                the_ETH_Map.insertObservation(&obs_LIDARETH);
                std::vector<float> xse_ETH, yse_ETH, zse_ETH;
                the_ETH_Map.getAllPoints(xse_ETH, yse_ETH, zse_ETH);
                Laserwin_ETH.plot(xse_ETH, yse_ETH, ".b3");
                Laserwin_ETH.axis_equal();
                mrpt::maps::CSimplePointsMap the_ACM_Map;
                the_ACM_Map.insertionOptions.minDistBetweenLaserPoints = 0;
                the_ACM_Map.insertObservation(&obs_LIDARACM);
                std::vector<float> xs_ACM, ys_ACM, zs_ACM;
                the_ACM_Map.getAllPoints(xs_ACM, ys_ACM, zs_ACM);
                Laserwin_ACM.plot(xs_ACM, ys_ACM, ".b3");
                Laserwin_ACM.axis_equal();


            } // end update visualization:


        }

        cout << "\nClosing RGBD sensor...\n";

        return 0;


   // laser_ETH.turnOff();
   // laser_acm.turnOff();
    ros::spin();

    return 0;
}
