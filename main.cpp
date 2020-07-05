// STD
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>

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
    vis.CreateVisualizerWindow( "Open3D_odometry", 1600, 900, 50, 50 );
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
    vector< Mat > trajectory;
    Eigen::Matrix4d Rt;
    
    // --- Create rgbd odometry by camera matrix
    RgbdOdometry rgbdOdom( mtxL,                        // cameraMatrix
                           10.0f,                        // minDepth = 0
                           400.0f,                      // maxDepth = 4
                           0.15f,                       // maxDepthDiff = 0.07
                           vector< int >(),             // iterCounts
                           vector< float >(),           // minGradientMagnitudes
                           0.2f,                       // maxPointsPart = 0.07 to 1.0
                           Odometry::RIGID_BODY_MOTION  // transformType = RIGID_BODY_MOTION
                           );
    OdometryFrame preOFrame;
    
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
        imgDisp.convertTo( imgDisp32, CV_32FC1 );
        imgDisp32 /= 16;
        
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
        
        // --- Odometry
        cout << i << ": \n";
        if (i > 0)
        {
            // Create next odometry frame (img + depth)
            OdometryFrame nextOFrame( imgLgray, depth[2], Mat(), Mat(), int(i) );
            
            // Next camera coordinates
            Mat Rtn;
            Timer time;
            rgbdOdom.compute( preOFrame.image, preOFrame.depth, preOFrame.mask, nextOFrame.image, nextOFrame.depth, nextOFrame.mask, Rtn, trajectory[i-1] );
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
