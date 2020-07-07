// STD
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <tuple>

// Eigen
#include <Eigen/Eigen>

// Open3d
#include <Open3D/Open3D.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/Visualization/Visualizer/Visualizer.h>

// CV
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/rgbd.hpp>
//#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;
using namespace cv::rgbd;
using namespace open3d;
using namespace Eigen;

#define RESIZE_WIDTH 1280
#define RESIZE_HEIGHT 720

#define ODOMETRY 1 // 0 - OPENCV, 1 - OPEN3D

class Timer
{
private:
	using clock_t = std::chrono::high_resolution_clock;
	using second_t = std::chrono::duration<double, std::ratio<1> >;
 
	std::chrono::time_point<clock_t> m_beg;
 
public:
	Timer() : m_beg(clock_t::now())	{ }
 
	void reset()
	{
		m_beg = clock_t::now();
	}
 
	double elapsed() const
	{
		return std::chrono::duration_cast<second_t>(clock_t::now() - m_beg).count();
	}
};

void PaintGrid( visualization::Visualizer &vis, double Gsize, double Gstep )
{
    double R = 0.005, H = Gsize;
    int resolution = 3, split = 2;
    for ( double i = 0; i <= Gsize; i += Gstep )
    {
            // XoZ || oX
        auto plane1 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane1->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Matrix4d Rt;
        Rt << 0.0, 0.0, 1.0, 0.0,
              0.0, 1.0, 0.0, 0.0,
              1.0, 0.0, 0.0,  i,
              0.0, 0.0, 0.0, 1.0;
        plane1->Transform( Rt );
        plane1->ComputeVertexNormals();
        vis.AddGeometry( plane1 );
        
            // XoZ || oZ
        auto plane2 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane2->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0, i-Gsize/2,
              0.0, 1.0, 0.0,    0.0,
              0.0, 0.0, 1.0,    H/2,
              0.0, 0.0, 0.0,    1.0;
        plane2->Transform( Rt );
        plane2->ComputeVertexNormals();
        vis.AddGeometry( plane2 );
        
            // YoZ || oZ
        auto plane3 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane3->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0,    0.0,
              0.0, 1.0, 0.0, i-Gsize/2,
              0.0, 0.0, 1.0,    H/2,
              0.0, 0.0, 0.0,    1.0;
        plane3->Transform( Rt );
        plane3->ComputeVertexNormals();
        vis.AddGeometry( plane3 );
        
            // YoZ || oY
        auto plane4 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane4->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, 1.0, 0.0,  i,
              0.0, 0.0, 0.0, 1.0;
        plane4->Transform( Rt );
        plane4->ComputeVertexNormals();
        vis.AddGeometry( plane4 );
    }
}

int main( int argc, char *argv[] )    //int argc, char *argv[]
{
    //std::cout << cv::getBuildInformation() << std::endl;
    
    // --config=/home/roman/work/RGBD_data/cfg/113/head_1/SL_config.yaml --img-path=/home/roman/work/RGBD_data/test_samples/test_men/
    /*** READ CONFGURATION ***/
    const cv::String keys =
        "{help h usage ? |                  | print this message     }"
        "{config         |SL_config.yaml    | path to calib config   }"
        "{img-path       |path to L & R img | path to image files    }"
//        "{save-conf      |              | Sava config file       }"
//        "{print-conf     |              | Print config file      }"   
        ;

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("RGBD");
    if ( (parser.has("help")) || (argc < 3) )
    {
        parser.printMessage();
        return 0;
    }
    
    // Open config file
    std::string confPath = parser.get< cv::String >("config");
    std::cout << " --- Configuration file is " << confPath << "\n";
    FileStorage confFile( confPath, FileStorage::READ );
    if (!confFile.isOpened())
    {
        cerr << "Failed to open " << confPath << endl;
        return 1;
    }
    
    // --- Read config
    int width = confFile["width"];
    int height = confFile["height"];
    Mat mtxL, mtxR; 
    Mat distL, distR;
    confFile["mtxL"] >> mtxL;
    confFile["mtxR"] >> mtxR;
    confFile["distL"] >> distL;
    confFile["distR"] >> distR;
    Mat rectifyL, rectifyR;
    Mat projectL, projectR;
    confFile["rectifyL"] >> rectifyL;
    confFile["rectifyR"] >> rectifyR;
    confFile["projectL"] >> projectL;
    confFile["projectR"] >> projectR;
    Mat Q;
    confFile["Q"] >> Q;
        
    cout << "width: " << width << endl;
    cout << "height: " << height << endl;
    cout << "mtxL: \n" << mtxL << endl;
    cout << "mtxR: \n" << mtxR << endl;
    cout << "distL: \n" << distL << endl;
    cout << "distR: \n" << distR << endl;
    cout << "rectifyL: \n" << rectifyL << endl;
    cout << "rectifyR: \n" << rectifyR << endl;
    cout << "projectL: \n" << projectL << endl;
    cout << "projectR: \n" << projectR << endl;
    cout << "Q: \n" << Q << "\n\n";
    
    cout << " --- StereoSGBM config: \n";
    int maxDisp = confFile["general_stereo_parametrs"]["maxDisp"];
    int blockSize = confFile["general_stereo_parametrs"]["blockSize"];
    int preFilterCap = confFile["general_stereo_parametrs"]["preFilterCap"];
    int P1 = confFile["general_stereo_parametrs"]["P1"];
    int P2 = confFile["general_stereo_parametrs"]["P2"];
    int speckleRange = confFile["general_stereo_parametrs"]["speckleRange"];
    int speckleWindowSize = confFile["general_stereo_parametrs"]["speckleWindowSize"];
    cout << "maxDisp: " << maxDisp << endl;
    cout << "blockSize: " << blockSize << endl;
    cout << "preFilterCap: " << preFilterCap << endl;
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;
    cout << "speckleRange: " << speckleRange << endl;
    cout << "speckleWindowSize: " << speckleWindowSize << "\n\n";
    confFile.release();
    
    // Read image path
    std::string imgFolder = parser.get< cv::String >("img-path");
    if(*(imgFolder.end()-1) != '/') imgFolder += '/';
    size_t pos = confPath.rfind('/');
    std::string imgFolderL = imgFolder + confPath[pos+1] + "L/";
    std::string imgFolderR = imgFolder + confPath[pos+1] + "R/";
    std::cout << " --- Image Left files is " << imgFolderL << "\n";
    std::cout << " --- Image Right files is " << imgFolderR << "\n\n\n";
    
    vector< string > imgPathL, imgPathR;
    cv::glob( imgFolderL + "*LZcmCameraBaslerJpegFrame*.png", imgPathL );
    cv::glob( imgFolderR + "*RZcmCameraBaslerJpegFrame*.png", imgPathR );
    
    
    // Computes the undistortion and rectification transformation map
    Mat rmapL[2], rmapR[2];
    initUndistortRectifyMap( mtxL, distL, rectifyL, projectL, Size( width, height ), 
                             CV_32FC1, rmapL[0], rmapL[1] );
    initUndistortRectifyMap( mtxR, distR, rectifyR, projectR, Size( width, height ), 
                             CV_32FC1, rmapR[0], rmapR[1] );
    
    // Params of disparity
    Ptr < StereoSGBM > sbm = StereoSGBM::create( 0,                             // minDisparity                         0
                                                 maxDisp,                       // numDisparities must be divisible by  16
                                                 blockSize,                     // blockSize                            3
                                                 P1,                            // P1                                   0
                                                 P2,                            // P2                                   0
                                                 0,                             // disp12MaxDiff                        0
                                                 preFilterCap,                  // prefilterCap                         0
                                                 0,                             // uniquenessRatio                      0
                                                 speckleWindowSize,             // speckleWindowSize                    0
                                                 speckleRange,                  // speckleRange                         0
                                                 StereoSGBM::MODE_SGBM_3WAY );  // mode MODE_SGBM
    
    size_t numFrames = (imgPathL.size() < imgPathR.size()) ? imgPathL.size() : imgPathR.size();
    
    
    // --- Visualization
    visualization::Visualizer vis;
    vis.CreateVisualizerWindow( "RGBD_odometry", 1600, 900, 50, 50 );
    // Add Coordinate
    auto coord = geometry::TriangleMesh::CreateCoordinateFrame( 1.0, Vector3d( 0.0, 0.0, 0.0 ) );
    coord->ComputeVertexNormals();
    vis.AddGeometry( coord );
    // Grid
    PaintGrid( vis, 600.0, 1.0 );
    // Add streo 3d point
    shared_ptr< geometry::PointCloud > cloudStereo ( new geometry::PointCloud );  // Global cloud 3D points
    vis.AddGeometry( cloudStereo );
    // Visualisation trajectory
    shared_ptr< geometry::PointCloud > cloudTpoint ( new geometry::PointCloud );
    shared_ptr< geometry::LineSet > cloudTline ( new geometry::LineSet );
    vis.AddGeometry( cloudTpoint );
    vis.AddGeometry( cloudTline );
    Vector3d colorTpoint = Vector3d(1.0, 0.0, 0.0);
    Vector3d colorTline = Vector3d(1.0, 1.0, 0.0);
    Eigen::Matrix4d Rt;
    // Parameters for odometry
    double minDepth = 0.0;
    double maxDepth = 400.0;
    double maxDepthDiff = 0.15;
    std::vector< int > iterCounts = {20,10,5};
    
    // --- Create rgbd odometry
#if (ODOMETRY == 0)     // opencv
    vector< Mat > trajectory;
    RgbdOdometry rgbdOdom( mtxL,                        // cameraMatrix
                           minDepth,                        // minDepth = 0
                           maxDepth,                      // maxDepth = 4
                           maxDepthDiff,                       // maxDepthDiff = 0.07
                           iterCounts,             // iterCounts
                           vector< float >(),           // minGradientMagnitudes
                           0.2f,                       // maxPointsPart = 0.07 to 1.0
                           Odometry::RIGID_BODY_MOTION  // transformType = RIGID_BODY_MOTION
                           );
    OdometryFrame preOFrame;
#elif (ODOMETRY == 1)  // open3d
    vector< Matrix4d > trajectory;
//    Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d trans_odo = Eigen::Matrix4d::Identity();
    Eigen::Matrix6d info_odo = Eigen::Matrix6d::Zero();
    bool is_success;
    camera::PinholeCameraIntrinsic intrinsic( width, height, 
                                              mtxL.at<double>(0,0), mtxL.at<double>(1,1),
                                              mtxL.at<double>(0,2), mtxL.at<double>(1,2));
    odometry::RGBDOdometryJacobianFromColorTerm jacobian_method;
    odometry::OdometryOption option( iterCounts,
                                     maxDepthDiff,
                                     minDepth,
                                     maxDepth );
    std::shared_ptr< open3d::geometry::RGBDImage > preOFrame;
#endif
    
    // Timer
    double sumTime = 0;
    
    /*** START ***/ 
    for ( size_t i = 0; i < numFrames; i++ )
    {
        // load images
        Mat imgL = imread( imgPathL[i], IMREAD_COLOR );
        Mat imgR = imread( imgPathR[i], IMREAD_COLOR );
        
        // Undistort & rectify
        Mat imgRemapL, imgRemapR;
        remap( imgL, imgRemapL, rmapL[0], rmapL[1], INTER_LINEAR );
        remap( imgR, imgRemapR, rmapR[0], rmapR[1], INTER_LINEAR );
        
        // Convert to gray
        Mat imgLgray, imgRgray;
        cvtColor( imgRemapL, imgLgray, COLOR_BGR2GRAY );
        cvtColor( imgRemapR, imgRgray, COLOR_BGR2GRAY );
        
        // Calculate disparity        
        Mat imgDisp;
        sbm->compute( imgRemapL, imgRemapR, imgDisp );
        
        // Scaled to float
        Mat imgDisp32;
        imgDisp.convertTo( imgDisp32, CV_32FC1, 1.0/16 );
//        imgDisp32 /= 16;
        
        double minVal; double maxVal;
        minMaxLoc( imgDisp32, &minVal, &maxVal );
        Mat imgDispNorm;
        imgDisp32.convertTo( imgDispNorm, CV_8UC1, 255/(maxVal - minVal) );
        Mat imgDisp_color;
        applyColorMap( imgDispNorm, imgDisp_color, COLORMAP_RAINBOW );   // COLORMAP_HOT
        resize( imgDisp_color, imgDisp_color, Size(RESIZE_WIDTH, RESIZE_HEIGHT), 0, 0, cv::INTER_LINEAR );
//        imshow( "Disparity", imgDisp_color );
        
        // Make 3D points
        Mat points3D;
        reprojectImageTo3D( imgDisp32, points3D, Q, false, CV_32F ); // -1 CV_16S CV_32S CV_32F
        Mat depth[3];
        split( points3D, depth );
        
        cout << i << ": \n";
#if (ODOMETRY == 0)     // --- RGBD odometry with opencv
        if (i > 0)
        {
            // Create next odometry frame (img + depth)
            OdometryFrame nextOFrame( imgLgray, depth[2], Mat(), Mat(), int(i) );
            
            // Next camera coordinates
            Mat Rtn;
            Timer time;
            rgbdOdom.compute( preOFrame.image, preOFrame.depth, preOFrame.mask, 
                              nextOFrame.image, nextOFrame.depth, nextOFrame.mask, 
                              Rtn, trajectory[i-1] );
            double nowTime = time.elapsed();
            sumTime += nowTime;
            cout << "Time taken: " << nowTime << " c\n";
            trajectory.push_back( Rtn );
            // Save next odometry frame
            preOFrame = nextOFrame;
            
            // Convert opencv Mat to eigen Matrix
            Eigen::Matrix4d nextRt;
            cv::cv2eigen( Rtn, nextRt );
            Rt *= nextRt;
            // Add trajectory point & line
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTline->lines_.push_back( Vector2i( i-1, i ) );
            cloudTline->colors_.push_back( colorTline );
        }
        else    // First pass
        {
            // Create basic odometry frame (img + depth)
            preOFrame = OdometryFrame( imgLgray, depth[2], Mat(), Mat(), int(i) );
            
            // Basic camera coordinates
            Mat Rt0 = Mat::eye( 4, 4, CV_64FC1 );
            trajectory.push_back( Rt0 );
            
            // Convert opencv Mat to eigen Matrix
            Eigen::Matrix4d firstRt;
            cv::cv2eigen( Rt0, firstRt );
            Rt = firstRt;
            // Add trajectory point & line
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
        }
#elif (ODOMETRY == 1)       // --- RGBD odometry with open3d
        if (i > 0)
        {
            // Create next odometry frame (img + depth)
            geometry::Image nextImg;
            nextImg.Prepare( width, height, 3, sizeof(uint8_t) );
            std::copy( imgRemapL.datastart, imgRemapL.dataend, nextImg.data_.data() );
            geometry::Image nextDepth;
            nextDepth.Prepare( width, height, 1, sizeof(uint32_t) );
            std::copy( depth[2].datastart, depth[2].dataend, nextDepth.data_.data() );
            std::shared_ptr< open3d::geometry::RGBDImage > nextOFrame = open3d::geometry::RGBDImage::CreateFromColorAndDepth( nextImg, nextDepth );
            
            // Next camera coordinates
            Eigen::Matrix4d nextRt;
            Timer time;
            std::tie( is_success, nextRt, info_odo ) = odometry::ComputeRGBDOdometry( *preOFrame, *nextOFrame, intrinsic,
                                                                                      trajectory[i-1], jacobian_method,
                                                                                      option );
            double nowTime = time.elapsed();
            sumTime += nowTime;
            cout << "Time taken: " << nowTime << " c\n";
            trajectory.push_back( nextRt );
            Rt *= nextRt;
            
            // Save next odometry frame
            preOFrame = nextOFrame;
            
            // Add trajectory point & line
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTline->lines_.push_back( Vector2i( i-1, i ) );
            cloudTline->colors_.push_back( colorTline );
        }
        else    // First pass
        {
            // Create basic odometry frame (img + depth)
            geometry::Image preImg;
            preImg.Prepare( width, height, 3, sizeof(uint8_t) );
            std::copy( imgRemapL.datastart, imgRemapL.dataend, preImg.data_.data() );
            geometry::Image preDepth;
            preDepth.Prepare( width, height, 1, sizeof(uint32_t) );
            std::copy( depth[2].datastart, depth[2].dataend, preDepth.data_.data() );   // ----ERROR
            preOFrame = open3d::geometry::RGBDImage::CreateFromColorAndDepth( preImg, preDepth );
            
            // Basic camera coordinates
            Eigen::Matrix4d firstRt = Eigen::Matrix4d::Identity();
            trajectory.push_back( firstRt );
            Rt = firstRt;
            
            // Add trajectory point & line
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
        }
#endif
        
        
//        cout << trajectory[i] << "\n\n";
//        cout << Rt << "\n";
        cout << Rt(0,3) << " | " << Rt(1,3) << " | " << Rt(2,3) << "\n\n";   
        
        // Try add 3d point of stereo )))
        geometry::PointCloud cloudTemp;
        int step = 15;
        for ( int y = 0; y < points3D.rows; y += step  )
        {
            for ( int x = 0; x < points3D.cols; x += step  )
            {
                if ( points3D.at< Vec3f >( y, x ).val[2] > 0 )
                {
                    Vector3d temPoint, tempColor;
                    temPoint << double( points3D.at< Vec3f >( y, x ).val[0] ),
                                double( points3D.at< Vec3f >( y, x ).val[1] ),
                                double( points3D.at< Vec3f >( y, x ).val[2] );
                    tempColor.x() = double( imgRemapL.at< Vec3b >( y, x ).val[2] / 255.0 );
                    tempColor.y() = double( imgRemapL.at< Vec3b >( y, x ).val[1] / 255.0 );
                    tempColor.z() = double( imgRemapL.at< Vec3b >( y, x ).val[0] / 255.0 );
                    cloudTemp.points_.push_back( temPoint );
                    cloudTemp.colors_.push_back( tempColor );
                }
            }
        }
        // Transform new 3d stereo points
        cloudTemp.Transform( Rt );
        // Add new 3d stereo points to global cloud
        cloudStereo->operator+=( cloudTemp );
        
        // 3D point
//        cloudStereo->Clear();
//        for ( size_t j = 0; j < points3D.total(); j += 4 )
//        {
//            Vector3d temPoint, tempColor;
//            temPoint << double( points3D.at< Vec3f >(int(j))[0] ),
//                        double( points3D.at< Vec3f >(int(j))[1] ),
//                        double( points3D.at< Vec3f >(int(j))[2] );
//            tempColor.x() = double( imgRemapL.at< Vec3b >(int(j)).val[2] / 255.0 );
//            tempColor.y() = double( imgRemapL.at< Vec3b >(int(j)).val[1] / 255.0 );
//            tempColor.z() = double( imgRemapL.at< Vec3b >(int(j)).val[0] / 255.0 );
//            cloudStereo->points_.push_back( temPoint );
//            cloudStereo->colors_.push_back( tempColor );
//        }
        
//        char key = char(waitKey(30));
////        break;
//        if ( (key == 'q') || (key == 'Q') || (key == 27) ) break; // Interrupt the cycle, press "ESC"
//        else if ( key == 'p' || key == 'P' ) 
//        {
//            cout << "Pause" << endl;
//            vis.UpdateGeometry();
//            //vis.PollEvents();
//            vis.Run();
//            cout << "END Pause" << endl;
//        }
//        vis.UpdateGeometry();
//        vis.PollEvents();
//        //vis.Run();
    }
    cout << "Mean time: " << sumTime / (numFrames - 1) << "\n\n";
    vis.UpdateGeometry();
    vis.Run();
    
    return 0;
}



















//#include <opencv2/core/utility.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>
//#include <iostream>
//#include <cstring>
//using namespace std;
//using namespace cv;
//int main( int argc, char** argv ){
//  // show help
//  if(argc<2){
//    cout<<
//      " Usage: tracker <video_name>\n"
//      " examples:\n"
//      " example_tracking_kcf Bolt/img/%04d.jpg\n"
//      " example_tracking_kcf faceocc2.webm\n"
//      << endl;
//    return 0;
//  }
//  // declares all required variables
//  Rect2d roi;
//  Mat frame;
//  // create a tracker object
//  Ptr<Tracker> tracker = TrackerKCF::create();
//  // set input video
//  std::string video = argv[1];
//  VideoCapture cap;
//  cap.open(video);
//  // check if we succeeded
//  if (!cap.isOpened()) {
//      cerr << "ERROR! Unable to open camera\n";
//      return -1;
//  }
//  // get bounding box
//  cap >> frame;
//  roi=selectROI("tracker",frame);
//  //quit if ROI was not selected
//  if(roi.width==0 || roi.height==0)
//    return 0;
//  // initialize the tracker
//  tracker->init(frame,roi);
//  // perform the tracking process
//  printf("Start the tracking process, press ESC to quit.\n");
//  for ( ;; ){
//    // get frame from the video
//    cap >> frame;
//    // stop the program if no more images
//    if(frame.rows==0 || frame.cols==0)
//      break;
//    // update the tracking result
//    tracker->update(frame,roi);
//    // draw the tracked object
//    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
//    // show image with the tracked object
//    imshow("tracker",frame);
//    //quit on ESC button
//    if(waitKey(1)==27)break;
//  }
//  return 0;
//}





/*    for ( size_t i = 0; i < trajectory.size(); i++ )
    {
        if (i > 0)
        {
            Eigen::Matrix4d nextRt;
            cv::cv2eigen( trajectory[i], nextRt );
            Rt *= nextRt.inverse();
            
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTline->lines_.push_back( Vector2i( i-1, i ) );
            cloudTline->colors_.push_back( colorTline );
            
//            geometry::PointCloud cloudTemp;
//            for ( size_t j = 0; j < skyImg.size(); j++ )
//            {
//                for ( int y = 0; y < height; y++ )
//                {
//                    for ( int x = 0; x < width; x++ )
//                    {
//                        cloudTemp.points_.push_back( Vector3d( double(skyDepth[j].at<Vec3f>(y,x).val[0]),
//                                                               double(skyDepth[j].at<Vec3f>(y,x).val[1]),
//                                                               double(skyDepth[j].at<Vec3f>(y,x).val[2]) ) );
//                        cloudTemp.colors_.push_back( Vector3d( skyImg[j].at<Vec3b>(y,x).val[2],
//                                                               skyImg[j].at<Vec3b>(y,x).val[1],
//                                                               skyImg[j].at<Vec3b>(y,x).val[0] ) );
//                    }
//                }
//            }
//            cloudTemp.Transform( Rt );
//            cloudStereo->operator+=( cloudTemp );
        }
        else
        {
            Eigen::Matrix4d firstRt;
            cv::cv2eigen( trajectory[i], firstRt );
            Rt = firstRt.inverse();
            
//            cloudTpoint->points_.push_back( Vector3d(0,0,0) );
            cloudTpoint->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            cloudTpoint->colors_.push_back( colorTpoint );
            cloudTline->points_.push_back( Vector3d( Rt(0,3), Rt(1,3), Rt(2,3) ) );
            
//            geometry::PointCloud cloudTemp0;
//            for ( size_t j = 0; j < skyImg.size(); j++ )
//            {
//                for ( int y = 0; y < height; y++ )
//                {
//                    for ( int x = 0; x < width; x++ )
//                    {
//                        cloudTemp0.points_.push_back( Vector3d( double(skyDepth[j].at<Vec3f>(y,x).val[0]),
//                                                                double(skyDepth[j].at<Vec3f>(y,x).val[1]),
//                                                                double(skyDepth[j].at<Vec3f>(y,x).val[2]) ) );
//                        cloudTemp0.colors_.push_back( Vector3d( skyImg[j].at<Vec3b>(y,x).val[2],
//                                                                skyImg[j].at<Vec3b>(y,x).val[1],
//                                                                skyImg[j].at<Vec3b>(y,x).val[0] ) );
                        
//                    }
//                }
//            }
//            cloudTemp0.Transform( Rt );
//            cloudStereo->operator+=( cloudTemp0 );
        }
    }
    vis.UpdateGeometry();
    vis.Run();*/






//    float progress = 0.0;
//    while (progress < 1.0) {
//        int barWidth = 70;
    
//        std::cout << "[";
//        int pos = barWidth * progress;
//        for (int i = 0; i < barWidth; ++i) {
//            if (i < pos) std::cout << "=";
//            else if (i == pos) std::cout << ">";
//            else std::cout << " ";
//        }
//        std::cout << "] " << int(progress * 100.0) << " %\r";
//        std::cout.flush();
    
//        progress += 0.16; // for demonstration only
//    }
//    std::cout << std::endl;
    
/*    std::shared_ptr< geometry::RGBDImage > (*CreateRGBDImage)( const geometry::Image&, const geometry::Image&, bool );
    CreateRGBDImage = &geometry::RGBDImage::CreateFromRedwoodFormat;
        // Redwood format
    auto color = io::CreateImageFromFile( "color_00000.jpg" );
    auto depth = io::CreateImageFromFile( "depth_00000.png" );
    auto RGBD0 = CreateRGBDImage( *color, *depth, true );
    color = io::CreateImageFromFile( "color_00001.jpg" );
    depth = io::CreateImageFromFile( "depth_00001.png" );
    auto RGBD1 = CreateRGBDImage( *color, *depth, true );
    color = io::CreateImageFromFile( "color_00002.jpg" );
    depth = io::CreateImageFromFile( "depth_00002.png" );
    auto RGBD2 = CreateRGBDImage( *color, *depth, true );
    color = io::CreateImageFromFile( "color_00003.jpg" );
    depth = io::CreateImageFromFile( "depth_00003.png" );
    auto RGBD3 = CreateRGBDImage( *color, *depth, true );
    color = io::CreateImageFromFile( "color_00004.jpg" );
    depth = io::CreateImageFromFile( "depth_00004.png" );
    auto RGBD4 = CreateRGBDImage( *color, *depth, true );
        // SUN format
    auto color_SUN = io::CreateImageFromFile( "SUN_color.jpg" );
    auto depth_SUN = io::CreateImageFromFile( "SUN_depth.png" );
        // NYU format
//    auto color_NYU = io::CreateImageFromFile( "NYU_color.ppm" );
//    auto depth_NYU = io::CreateImageFromFile( "NYU_depth.pgm" );
        // NYU format
    auto color_TUM = io::CreateImageFromFile( "TUM_color.png" );
    auto depth_TUM = io::CreateImageFromFile( "TUM_depth.png" );
    
    vector< uint8_t > color_data = RGBD0->color_.data_;
    int H = RGBD0->color_.height_;
    int W = RGBD0->color_.width_;
//    int numC = RGBD0->color_.num_of_channels_;
//    int bytC = RGBD0->color_.bytes_per_channel_;
    vector< uint8_t > depth_data = RGBD0->depth_.data_;
    Mat test = Mat( H, W, CV_8UC4, &RGBD0->color_.data_, 4 );
    cvtColor( test, test, COLOR_RGBA2BGR ); // COLOR_RGBA2BGR
    imshow( "test", test );
    //waitKey(0);
    if( waitKey(0) == 27 ) exit(0);
    
    //CreateRGBDImage = &geometry::RGBDImage::CreateFromSUNFormat;
    //CreateRGBDImage = &geometry::RGBDImage::CreateFromNYUFormat;
    //CreateRGBDImage = &geometry::RGBDImage::CreateFromTUMFormat;
    
    CreateRGBDImage = &geometry::RGBDImage::CreateFromSUNFormat;
    auto RGBD_SUN = CreateRGBDImage( *color_SUN, *depth_SUN, true );
    
//    CreateRGBDImage = &geometry::RGBDImage::CreateFromNYUFormat;
//    auto RGBD_NYU = CreateRGBDImage( *color_NYU, *depth_NYU, true );
    
    CreateRGBDImage = &geometry::RGBDImage::CreateFromTUMFormat;
    auto RGBD_TUM = CreateRGBDImage( *color_TUM, *depth_TUM, true );
    
    odometry::OdometryOption option;
    Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d trans_odo = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
    Eigen::Matrix6d info_odo = Eigen::Matrix6d::Zero();
    bool is_success;
    camera::PinholeCameraIntrinsic intrinsic;
    intrinsic = camera::PinholeCameraIntrinsic( camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault );
    
    odometry::RGBDOdometryJacobianFromColorTerm jacobian_method;
    
    shared_ptr< geometry::PointCloud > cloud_RGBD ( new geometry::PointCloud );
    //auto cloud_RGBD = geometry::PointCloud::CreateFromRGBDImage( *RGBD1, intrinsic );
    //auto cloud_RGBD = make_shared< geometry::PointCloud >();
    shared_ptr< geometry::PointCloud > cloud_RGBD0;
    cloud_RGBD0 = geometry::PointCloud::CreateFromRGBDImage( *RGBD0, intrinsic );
    cloud_RGBD0->Transform( Rt );
    auto coord0 = geometry::TriangleMesh::CreateCoordinateFrame( 0.2, Vector3d( 0.0, 0.0, 0.0 ) );
    coord0->Transform( Rt );
    *cloud_RGBD += *cloud_RGBD0;
            
    std::tie( is_success, trans_odo, info_odo ) = odometry::ComputeRGBDOdometry( *RGBD1, *RGBD0, intrinsic,
                                                                                 odo_init, jacobian_method,
                                                                                 option );
    Rt *= trans_odo;
    shared_ptr< geometry::PointCloud > cloud_RGBD1;
    cloud_RGBD1 = geometry::PointCloud::CreateFromRGBDImage( *RGBD1, intrinsic );
    cloud_RGBD1->Transform( Rt );
    auto coord1 = geometry::TriangleMesh::CreateCoordinateFrame( 0.2, Vector3d( 0.0, 0.0, 0.0 ) );
    coord1->Transform( Rt );
    *cloud_RGBD += *cloud_RGBD1;
    
    std::tie( is_success, trans_odo, info_odo ) = odometry::ComputeRGBDOdometry( *RGBD2, *RGBD1, intrinsic,
                                                                                 odo_init, jacobian_method,
                                                                                 option );
    Rt *= trans_odo;
    shared_ptr< geometry::PointCloud > cloud_RGBD2;
    cloud_RGBD2 = geometry::PointCloud::CreateFromRGBDImage( *RGBD2, intrinsic );
    cloud_RGBD2->Transform( Rt );
    auto coord2 = geometry::TriangleMesh::CreateCoordinateFrame( 0.2, Vector3d( 0.0, 0.0, 0.0 ) );
    coord2->Transform( Rt );
    *cloud_RGBD += *cloud_RGBD2;
    
    std::tie( is_success, trans_odo, info_odo ) = odometry::ComputeRGBDOdometry( *RGBD3, *RGBD2, intrinsic,
                                                                                 odo_init, jacobian_method,
                                                                                 option );
    Rt *= trans_odo;
    shared_ptr< geometry::PointCloud > cloud_RGBD3;
    cloud_RGBD3 = geometry::PointCloud::CreateFromRGBDImage( *RGBD3, intrinsic );
    cloud_RGBD3->Transform( Rt );
    auto coord3 = geometry::TriangleMesh::CreateCoordinateFrame( 0.2, Vector3d( 0.0, 0.0, 0.0 ) );
    coord3->Transform( Rt );
    *cloud_RGBD += *cloud_RGBD3;
    
    std::tie( is_success, trans_odo, info_odo ) = odometry::ComputeRGBDOdometry( *RGBD4, *RGBD3, intrinsic,
                                                                                 odo_init, jacobian_method,
                                                                                 option );
    Rt *= trans_odo;
    shared_ptr< geometry::PointCloud > cloud_RGBD4;
    cloud_RGBD4 = geometry::PointCloud::CreateFromRGBDImage( *RGBD4, intrinsic );
    cloud_RGBD4->Transform( Rt );
    auto coord4 = geometry::TriangleMesh::CreateCoordinateFrame( 0.2, Vector3d( 0.0, 0.0, 0.0 ) );
    coord4->Transform( Rt );
    *cloud_RGBD += *cloud_RGBD4;
    
    
    
    shared_ptr< geometry::PointCloud > cloud_RGBD_SUN;
    cloud_RGBD_SUN = geometry::PointCloud::CreateFromRGBDImage( *RGBD_SUN, intrinsic );
    
//    shared_ptr< geometry::PointCloud > cloud_RGBD_NYU;
//    cloud_RGBD_NYU = geometry::PointCloud::CreateFromRGBDImage( *RGBD_NYU, intrinsic );
    
    shared_ptr< geometry::PointCloud > cloud_RGBD_TUM;
    cloud_RGBD_TUM = geometry::PointCloud::CreateFromRGBDImage( *RGBD_TUM, intrinsic );
    
    
    visualization::Visualizer vis_RGBD;
    vis_RGBD.CreateVisualizerWindow( "Open3D_Lidar", 1600, 900, 50, 50 );
    coord0->ComputeVertexNormals();
    coord1->ComputeVertexNormals();
    coord2->ComputeVertexNormals();
    coord3->ComputeVertexNormals();
    coord4->ComputeVertexNormals();
    vis_RGBD.AddGeometry( coord0 );
    vis_RGBD.AddGeometry( coord1 );
    vis_RGBD.AddGeometry( coord2 );
    vis_RGBD.AddGeometry( coord3 );
    vis_RGBD.AddGeometry( coord4 );
    PaintGrid( vis_RGBD, 5, 0.5 );
    vis_RGBD.AddGeometry( cloud_RGBD );
//    vis_RGBD.AddGeometry( cloud_RGBD0 );
//    vis_RGBD.AddGeometry( cloud_RGBD1 );
//    vis_RGBD.AddGeometry( cloud_RGBD2 );
//    vis_RGBD.AddGeometry( cloud_RGBD3 );
//    vis_RGBD.AddGeometry( cloud_RGBD4 );
//    vis_RGBD.AddGeometry( cloud_RGBD_SUN );
//    vis_RGBD.AddGeometry( cloud_RGBD_NYU );
//    vis_RGBD.AddGeometry( cloud_RGBD_TUM );
    
    vis_RGBD.Run();*/
