#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <math.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#define _USE_MATH_DEFINES
#define ARROW 0
#define QR_CODE 1
#define STOP 2
#define SIZE_IMAGES 3

using namespace cv;
using namespace std;
void getComponents(const Mat normalised_homography, float &theta );
void drawInclination(float theta);
void sock_connect(void);
void send_speed(int);
void send_angle(int);

int cl=0;
int angle = 20, speed = 200;
int main(int argc, char* argv[])
{
   sock_connect();
    Mat object[SIZE_IMAGES];
    object[0] = imread("3.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    object[1] = imread("1.jpg", CV_LOAD_IMAGE_GRAYSCALE );
    object[2] = imread("3.jpg", CV_LOAD_IMAGE_GRAYSCALE );

    if( !object[0].data )
    {
        std::cout<< "Error reading object " << std::endl;
        return -1;
    }

    SurfDescriptorExtractor extractor[SIZE_IMAGES];
    Mat des_object[SIZE_IMAGES];
    SurfFeatureDetector detector[SIZE_IMAGES];
    vector<vector<KeyPoint>> kp_object(16, vector<KeyPoint>(SIZE_IMAGES));
    FlannBasedMatcher matcher;
    vector<vector<Point2f>> obj_corners(SIZE_IMAGES, vector<Point2f>(4));

   // VideoCapture cap("tcp:10.42.0.254:25101");
    VideoCapture cap(0);
	namedWindow("Good Matches");
    int minHessian = 1000;
    for(int i=0; i<2; i++) {

        detector[i] = SurfFeatureDetector(minHessian);

        detector[i].detect(object[i], kp_object[i]);

        //Calculate descriptors (feature vectors)
        extractor[i].compute(object[i], kp_object[i], des_object[i]);
        //Get the corners from the object
        obj_corners[i][0] = cvPoint(0,0);
        obj_corners[i][1] = cvPoint(object[0].cols, 0);
        obj_corners[i][2] = cvPoint(object[0].cols, object[0].rows);
        obj_corners[i][3] = cvPoint(0, object[0].rows);
    }
    char key = 'a';
    int framecount = 0;
    while (key != 27) {
    Mat frame;
    Mat des_image, img_matches;
	int detectet_obj=0;


      cap >> frame;

        if (framecount < 6) {
            framecount++;
			//cout << framecount << endl;
            continue;
        }
		framecount = 0;
		
		img_matches=frame;

    for(int images_counter=0; images_counter<2;images_counter++) {
	
      std::vector<KeyPoint> kp_image;
      std::vector<vector<DMatch > > matches;
      std::vector<DMatch > good_matches;
      std::vector<Point2f> obj;
      std::vector<Point2f> scene;
      std::vector<Point2f> scene_corners(4);
      Mat H;
	  Mat image;
	  cvtColor(frame, image, CV_RGB2GRAY);

        detector[images_counter].detect( image, kp_image );
        extractor[images_counter].compute( image, kp_image, des_image );
		
		if(kp_image.size() > 10) {
        matcher.knnMatch(des_object[images_counter], des_image, matches, 2);
        for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
        {
            if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
            {
                good_matches.push_back(matches[i][0]);
            }
        }
        if (good_matches.size() >= 10)
        {
			detectet_obj=1;

            for( int i = 0; i < good_matches.size(); i++ )
            {
                //Get the keypoints from the good matches
                obj.push_back( kp_object[images_counter][good_matches[i].queryIdx ].pt );
                scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
            }

            H = findHomography( obj, scene, CV_RANSAC );

            perspectiveTransform( obj_corners[images_counter], scene_corners, H);
            //Draw lines between the corners (the mapped object in the scene image )

            line( img_matches, scene_corners[0],  scene_corners[1], Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1],  scene_corners[2], Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2],  scene_corners[3]/* Point2f( object[images_counter].cols, 0)*/, Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3],  scene_corners[0] /*Point2f( object[images_counter].cols, 0)*/, Scalar( 0, 255, 0), 4 );

			cout << images_counter  << endl;
			if(images_counter == 0) {
			angle = 12;
			speed = 100;
			}
			
			if(images_counter == 1) {
				speed = 100;
	            float theta;
		        getComponents(H,theta);
			    drawInclination(theta);/* Magic */
				theta = abs(theta)/7.5; 
				angle = rint(theta);
				cout << angle << endl;
			}

        }
		}
		}
		if(detectet_obj == 0) {
			angle = 12;
			speed = 255;
		}
		send_angle(angle);
		send_speed(speed);
        //Show detected matchesnn
        imshow( "Good Matches", img_matches );
        key = waitKey(1);
    }
    return 0;

}

void getComponents(Mat const normalised_homography, float &theta )
{
    float a = normalised_homography.at<double>(0,0);
    float b = normalised_homography.at<double>(0,1);

    theta = atan2(b,a)*(180/M_PI);
}

String ToString( float value)
{
    ostringstream ss;
    ss << value;
    return ss.str();
}

void drawInclination(float theta)
{
    String Stheta = ToString(theta);
    int tam = 200;
    Mat canvas = Mat::zeros(tam,tam,CV_8UC3);

    theta = -theta * M_PI / 180;

    line(canvas, Point(0,tam/2),Point(tam,tam/2), Scalar(255,255,255) );
    line(canvas,Point(tam/2,tam/2), Point(tam/2 + tam*cos(theta),tam/2 + tam*sin(theta)),Scalar(0,255,0),2 );

    cv::putText(canvas, Stheta, Point(tam-100,tam), FONT_HERSHEY_PLAIN,1, Scalar(0,0,255)  );
    imshow("Inclunacion",canvas);

}



void sock_connect(void)
{

    struct sockaddr_in address;
    int addrlen = sizeof(address);
    int opt = 1;
    int fd=0;

    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                             &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

		memset(&address, 0, sizeof(address));
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = INADDR_ANY;
		address.sin_port = htons(50000);

    if (bind(fd, (struct sockaddr *)&address, 
                             sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(fd, 1) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
     }

    if ((cl = accept(fd, (struct sockaddr *)&address,
                            (socklen_t*)&addrlen))<0) {
		perror("accept");
		exit(EXIT_FAILURE);
    }



}
void send_speed(int speed1) {
	int command_speed = 100;

	send(cl, &command_speed, 4, 0);
	send(cl, &speed1, 4, 0);
//	recv(cl, &speed1, 4, 0);
}

void send_angle(int angle1){
	int command_angle = 15;
	
	send(cl, &command_angle, 4, 0);
	send(cl, &angle1, 4, 0);


}
