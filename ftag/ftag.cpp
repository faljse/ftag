/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/


#ifdef __unix__         
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#elif defined(_WIN32) || defined(WIN32) 
#define OS_Windows
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <tchar.h>

#define popen _popen
#define pclose _pclose
#define inet_pton InetPton
#endif

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <sys/types.h>

#include <mutex>
#include <condition_variable>
#include <iomanip>
#include <unordered_map>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "../src/aruco.h"
#include "../src/cvdrawingutils.h"
#include "params.hpp"
#include "blockingqueue.hpp"

using namespace cv;
using namespace aruco;

class  DetectedTag {
public:
    bool cam=false;
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
    int id;
};

MarkerDetector MDetector;
std::vector< Marker > TheMarkers;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
Params p;
BlockingQueue<DetectedTag> sendq;

void cvTackBarEvents(int pos, void *);
void print(Mat mat, int prec);
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType );
void readConfig();
Vec3d rotationMatrixToEulerAngles(Mat &R);
bool isRotationMatrix(Mat &R);


// http://stackoverflow.com/questions/32978066/why-is-there-no-wait-function-for-condition-variable-which-does-not-relock-the-m/32978267#32978267
// http://en.cppreference.com/w/cpp/thread/condition_variable

std::pair< double, double > AvrgTime(0, 0); // determines the average time required for detection
int iThresParam1, iThresParam2;
int waitTime = 1;
class CmdLineParser {
    int argc;
    char **argv;
public:
    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv) {}  bool operator[] ( std::string param ) {
        int idx=-1;
        for ( int i=0; i<argc
                &&
                idx==-1; i++ ) if ( std::string ( argv[i] ) ==param ) idx=i;
        return ( idx!=-1 ) ;
    } std::string operator()(std::string param,std::string defvalue="-1") {
        int idx=-1;
        for ( int i=0;
                i<argc
                &&
                idx==-1;
                i++ ) if ( std::string ( argv[i] ) ==param ) idx=i;
        if ( idx==-1 ) return defvalue;
        else  return ( argv[  idx+1] );
    }
};
bool _running=true;


cv::Mat resize(const cv::Mat &in,int width) {
    // if (in.size().width<=width) return in;
    float yf=float(  width)/float(in.size().width);
    cv::Mat im2;
    cv::resize(in,im2,cv::Size(width, float(in.size().height)*yf));
    return im2;

}

void tcpClient(std::string hostName)
{
    size_t colonPos = hostName.find(':');
    if(colonPos == std::string::npos) {
        std::cout << "hostName must be in host:port format" << std::endl;
    }

    std::string hostPart = hostName.substr(0, colonPos);
    std::string portPart = hostName.substr(colonPos + 1);
    std::stringstream parser(portPart);
    int port = 0;
    if(! (parser >> port) ) {
        std::cout << "invalid portnumber" << std::endl;
        return;
    }

    std::cout << "tcpClient start" << std::endl;
    while(_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); //throtle reconnects
        std::cout << "tcpClient connecting to \"" << hostPart << ":" << port << "\"" << std::endl;
        int sockfd = 0;
        struct sockaddr_in serv_addr;
        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("\n Error : Could not create socket \n");
            continue;
        }
        memset(&serv_addr, '0', sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
		
        if(inet_pton(AF_INET, hostPart.c_str(), &serv_addr.sin_addr)<=0) {
            printf("\n inet_pton error occured\n");
            continue;
        }

        if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("\n Error : Connect Failed \n");
            continue;
        }

        while(1) {
            DetectedTag m;
            bool s=sendq.tryWaitAndPop(m, 1500);

            if(s) {
                char c[255]= {0};
                sprintf(c+strlen(c), "%d %d %f %f %f %f %f %f", m.cam, m.id, m.x, m.y, m.z, m.rx, m.ry, m.rz);
                sprintf(c+strlen(c), "\n");
                if( send(sockfd , c, strlen(c) , 0) < 0) {
                    puts("send failed\n");
                    break;
                }
            }
            else {
                if( send(sockfd , "info\n", strlen("info\n") , 0) < 0) {
                    puts("send failed\n");
                    break;
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    FILE *fp=NULL;
    try {
        if(argc<2)
            p.readConfig("ftag.yml");
        else
            p.readConfig(argv[1]);
        std::cout << "read cam parameters from \"" << p.camParamsFile << "\"" << std::endl;
        TheCameraParameters.readFromXMLFile(p.camParamsFile);

        std::unordered_map<int, int> pointsMap;

        ///////////  OPEN VIDEO
        std::cout << "Opening camera \"" << p.videoDevice << "\"" << std::endl;
        cv::VideoCapture cap(p.videoDevice, cv::CAP_V4L2);
        if (!cap.isOpened())  // check if succeeded to connect to the camera
            CV_Assert("Cam open failed");
        cap.set(CV_CAP_PROP_FRAME_WIDTH,  p.xRes);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, p.yRes);

        ///// CONFIGURE DATA
        // read first image to get the dimensions
        cap >> TheInputImage;
        std::cout << "rows: " << TheInputImage.rows << std::endl;
        std::cout << "cols: " << TheInputImage.cols << std::endl;

        if (TheCameraParameters.isValid())
            TheCameraParameters.resize(TheInputImage.size());
        fp = popen(p.ffmpegCmd.c_str(), "w");

        MDetector.setDictionary("ARUCO_MIP_36h12");//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)

        std::thread t1(tcpClient, p.posServer);
        char key = 0;
        int index = 0;
        // capture until press ESC or until the end of the video
        do {
            // std::lock_guard<std::mutex> lk(m);
            cap.retrieve(TheInputImage);             // copy image
            int rows = TheInputImage.rows;      //
            int cols = TheInputImage.cols;      //

            double tick = (double)getTickCount(); // for checking the speed
            // Detection of markers in the image passed
            TheMarkers= MDetector.detect(TheInputImage, TheCameraParameters, p.markerSize);

            // chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
            AvrgTime.second++;
            // cout << "\rTime detection=" << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds nmarkers=" << TheMarkers.size() << std::endl;

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);

            for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), -1, true);
            }

            //send

            std::vector<cv::Point2f> points2D;
            std::vector<cv::Point3f> points3D;


            for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                Marker m = TheMarkers[i];
                std::map<int, Point3f>::iterator it = p.points.find(m.id);
                if(it == p.points.end()) {
                    continue;
                }
                Point3f point = it->second;
                points3D.push_back(point);
                points2D.push_back(m.getCenter());
                DetectedTag t;
                t.cam=true;
                t.id=m.id;
                t.x=m.getCenter().x;
                t.y=m.getCenter().y;
                sendq.push(t);
            }

            for (unsigned int i = 0; i < points3D.size(); i++) {
                points3D[i] *= (p.pointsScale);
            }
            

            if(points3D.size() >= 4) {
                cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);
                cv::setIdentity(cameraMatrix);
                cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
                cv::Mat rvec(3, 1, cv::DataType<double>::type); //in eulers
                cv::Mat tvec(3, 1, cv::DataType<double>::type);
                cv::solvePnP(points3D, points2D, TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, rvec, tvec, false, CV_ITERATIVE);
                cv::Mat rtm=getRTMatrix(rvec, tvec, 6);
                cv::Mat irtm = rtm.inv();
                
                DetectedTag camTag;
                camTag.id=0;
                camTag.x = irtm.at<double>(0,3);
                camTag.y = irtm.at<double>(1,3);
                camTag.z = irtm.at<double>(2,3);
                
                Vec3d rot = rotationMatrixToEulerAngles(irtm);
                camTag.rx = rot.val[0];
                camTag.ry = rot.val[1];
                camTag.rz = rot.val[2];
                
                // camTag.x = rtm.at<double>(3,0);
                // camTag.y = rtm.at<double>(3,1);
                // camTag.z = rtm.at<double>(3,2);
                sendq.push(camTag);

                for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                    Marker m = TheMarkers[i];
                    cv::Mat tvecFloat(4, 1, irtm.type());

                    tvecFloat.at<double>(0,0) = m.Tvec.at<float>(0,0);
                    tvecFloat.at<double>(1,0) = m.Tvec.at<float>(1,0);
                    tvecFloat.at<double>(2,0) = m.Tvec.at<float>(2,0);
                    tvecFloat.rows = 4;
                    tvecFloat.cols = 1;
                    tvecFloat.at<double>(3,0) = 1;
                    cv::Mat x = irtm * tvecFloat;

                    DetectedTag t;
                    t.id=m.id;
                    t.x=x.at<double>(0);
                    t.y=x.at<double>(1);
                    t.z=x.at<double>(2);

                    cv::Mat n(4, 1, irtm.type());;
                    cv::Mat mRvec(4, 1, irtm.type());
                    mRvec.at<double>(0,0) = m.Rvec.at<float>(0,0);
                    mRvec.at<double>(1,0) = m.Rvec.at<float>(1,0);
                    mRvec.at<double>(2,0) = m.Rvec.at<float>(2,0);
                    mRvec.at<double>(3,0) = 0;
                    normalize(mRvec, n);
                    cv::Mat xr = irtm * mRvec;
                    t.rx = xr.at<double>(0);
                    t.ry = xr.at<double>(1);
                    t.rz = xr.at<double>(2);
                    sendq.push(t);
                }
            }

            // cout << "\rTheCameraParameters.isValid()" << TheCameraParameters.isValid() << std::endl;
            // draw a 3d cube in each marker if there is 3d info
            if (TheCameraParameters.isValid() && p.markerSize > 0)
                for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                }
            // DONE! Easy, right?
            if(p.headless==false) {
                // show input with augmented information and  the thresholded image
                cv::imshow("in", resize(TheInputImageCopy, 1360));
                cv::imshow("thres", resize(MDetector.getThresholdedImage(),640));
            }

            Mat streamImage = resize(TheInputImageCopy, p.xResOut);
            // cout << "stream rows: "<< streamImage.rows <<", cols: "<< streamImage.cols << std::endl;
            uchar* buffer = streamImage.data; // 8-bit pixel
            fwrite(buffer, 3, streamImage.total(), fp);

            key = cv::waitKey(waitTime); // wait for key to be pressed
            if(key=='s')  waitTime= waitTime==0?10:0;
            index++; // number of images captured

        } while (key != 27 && (cap.grab() ));

    } catch (std::exception &ex)

    {
        std::cout << "Exception :" << ex.what() << std::endl;
    }
    int status = pclose(fp);
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt(3,3,6);
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3d rotationMatrixToEulerAngles(Mat &R)
{
    // assert(isRotationMatrix(R));    
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3d(x, y, z);
}


/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos, void *) {
    (void)(pos);
    if (iThresParam1 < 3)  iThresParam1 = 3;
    if (iThresParam1 % 2 != 1)  iThresParam1++;
    if (iThresParam1 < 1)  iThresParam1 = 1;
    // MDetector.setThresholdParams(iThresParam1, iThresParam2);
    // recompute
    MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i = 0; i < TheMarkers.size(); i++)
        TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255));

    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < TheMarkers.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);

    cv::imshow("in", resize(TheInputImageCopy,1280));
    cv::imshow("thres", resize(MDetector.getThresholdedImage(),1280));
}

void print(Mat mat, int prec)
{
    for(int i=0; i<mat.size().height; i++)
    {
        std::cout << "[";
        for(int j=0; j<mat.size().width; j++)
        {
            std::cout <<  std::fixed << std::setprecision(prec) << std::setw(10) << mat.at<double>(i,j);
            if(j != mat.size().width-1)
                std::cout << ", ";
            else
                std::cout << "]" << std::endl;
        }
    }
}

/**
   * Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
   */
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
    cv::Mat M;
    cv::Mat R,T;
    R_.copyTo ( R );
    T_.copyTo ( T );
    if ( R.type() ==CV_64F ) {
        assert ( T.type() ==CV_64F );
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );
        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R64;
            R.convertTo ( R64,CV_64F );
            R.copyTo ( R33 );
        }
        for ( int i=0; i<3; i++ )
            Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
        M=Matrix;
    } else if ( R.depth() ==CV_32F ) {
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R32;
            R.convertTo ( R32,CV_32F );
            R.copyTo ( R33 );
        }

        for ( int i=0; i<3; i++ )
            Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
        M=Matrix;
    }

    if ( forceType==-1 ) return M;
    else {
        cv::Mat MTyped;
        M.convertTo ( MTyped,forceType );
        return MTyped;
    }
}

