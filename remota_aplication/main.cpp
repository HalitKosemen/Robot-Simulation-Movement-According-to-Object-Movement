#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include "extApi.h"
}


double calculate_distance(double focalLength, double realHeight, double pixelHeight) {
    return (focalLength * realHeight) / pixelHeight;
}


using namespace std;
#define PI 3.14
int main()
{
    bool VERBOSE = true;
    int clientID = 0;
    int leftmotorHandle = 0;
    int rightmotorHandle = 0;

    int lbrJoint1 = 0;
    int lbrJoint2 = 0;
    int lbrJoint3 = 0;
    int lbrJoint4 = 0;
    int lbrJoint5 = 0;
    int lbrJoint6 = 0;
    int lbrJoint7 = 0;

    int xdjoint = 0;


    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
   

    //! Todo Naresh: check to run this in parallel with real robot driver. May need to integrate my planner
    bool WORK = true;
    simxFinish(-1);                                                     //! Close any previously unfinished business
    clientID = simxStart((simxChar*)"127.0.0.1", 19000, true, true, 5000, 5);  //!< Main connection to V-REP
    Sleep(1);
    if (clientID != -1)
    {
        cout << " Connection status to VREP: SUCCESS" << endl;
        simxInt syncho = simxSynchronous(clientID, 1);
        int start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
        int TEST1 = simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
        int TEST2 = simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);


        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint1", &lbrJoint1, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint2", &lbrJoint2, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint3", &lbrJoint3, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint4", &lbrJoint4, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint5", &lbrJoint5, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint6", &lbrJoint6, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint7", &lbrJoint7, simx_opmode_oneshot_wait);


        if (VERBOSE)
        {
            cout << "Computed object handle: " << TEST1 << "  " << leftmotorHandle << endl;
            cout << "Computed object handle: " << TEST2 << "  " << rightmotorHandle << endl;
        }

        //        simxPauseCommunication(clientID,true);
        simxSetJointTargetPosition(clientID, lbrJoint1, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint2, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint3, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint4, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint5, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint6, 0.0, simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint7, 0.0, simx_opmode_oneshot_wait);

        //        simxPauseCommunication(clientID,false);

        cout << "At Second Block..." << endl;

        //simxPauseCommunication(clientID, 1);


        simxSetJointTargetPosition(clientID, lbrJoint1, 60.0 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint2, 45.0 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint3, 60 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint4, 80 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint5, 60 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint6, 45 * (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint7, 50 * (PI / 180), simx_opmode_oneshot_wait);



        Sleep(1000);
        simxPauseCommunication(clientID, 0);

        float joint2 = 1;
        simxSetJointTargetVelocity(clientID, lbrJoint2, 0.1, simx_opmode_oneshot_wait);

        cv::Scalar lower_blue = cv::Scalar(100, 150, 50);
        cv::Scalar upper_blue = cv::Scalar(130, 255, 255);


        cout << "geldin..." << endl;

        cv::VideoCapture video_cap(0);


        int width = video_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = video_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        std::cout << "WIDTH : " << width << std::endl;
        std::cout << "HEIGHT : " << height << std::endl;


        int centerX = width / 2;
        int centerY = height / 2;

        double focalLength = 910.0;
        double realObjectHeight = 2.1;

        double x_distance = 0;
        double y_distance = 0;

        double pixel_size_mm = 0.176421749;


        double distance = 0;


        while (simxGetConnectionId(clientID) != -1 && WORK) {

            cv::Mat frame, frame_hsv, frame_mask;
            video_cap >> frame; 
            if (frame.empty()) continue;


            cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

            cv::inRange(frame_hsv, lower_blue, upper_blue, frame_mask);
            cv::GaussianBlur(frame_mask, frame_mask, cv::Size(3, 3), cv::BORDER_DEFAULT);
            cv::threshold(frame_mask, frame_mask, 128, 255, cv::THRESH_BINARY); 

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(frame_mask, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

            std::vector<std::vector<cv::Point>> filtered_contours;
            double min_contour_area = 30.0;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > min_contour_area) {
                    filtered_contours.push_back(contour);
                }
            }


            if (!filtered_contours.empty()) {
                cv::Moments M = cv::moments(filtered_contours[0]);
                if (M.m00 != 0) {
                    int object_centerX = static_cast<int>(M.m10 / M.m00);
                    int object_centerY = static_cast<int>(M.m01 / M.m00);


                    centerX = object_centerX;
                    centerY = object_centerY;

                    //std::cout << "Nesne Orta Nokta : (" << object_centerX << "," << object_centerY << ")" << std::endl;
                }
            }


            //calculate distance
            double pixel_height;
            for (const auto& contour : filtered_contours) {
                int y_min = std::numeric_limits<int>::max();
                int y_max = std::numeric_limits<int>::min();
                for (const auto& point : contour) {
                    if (point.y < y_min) y_min = point.y;
                    if (point.y > y_max) y_max = point.y;
                }
                pixel_height = y_max - y_min;
                distance = calculate_distance(focalLength, realObjectHeight, pixel_height);
                //std::cout << "Nesnenin mesafesi: " << distance << " cm" << std::endl;
                //std::cout << "ymax ve ymin" << y_max << y_min << std::endl;
            }


            x_distance = (centerX - 320.0);
            x_distance = x_distance * pixel_size_mm;
            double aci_x_eksen = atan(x_distance / distance);
            aci_x_eksen = aci_x_eksen * 180 / 3.14;
            


            y_distance = (centerY - 240);
            y_distance = y_distance * pixel_size_mm;
            double aci_y_eksen = atan(y_distance / distance);
            aci_y_eksen = aci_y_eksen * 180 / 3.14;
            std::cout << "aci_y_ekeseni :" << aci_y_eksen << std::endl;

            

            cv::line(frame, cv::Point(0, centerY), cv::Point(width, centerY), cv::Scalar(120, 120, 120), 1);
            cv::line(frame, cv::Point(centerX, 0), cv::Point(centerX, height), cv::Scalar(120, 120, 120), 1);

            cv::line(frame, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(0, 0, 255), 1);
            cv::line(frame, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(0, 0, 255), 1);

            cv::circle(frame, cv::Point(centerX, centerY), 5, cv::Scalar(0, 0, 255), -1);  // Kýrmýzý merkez noktasý


            cv::drawContours(frame, filtered_contours, -1, cv::Scalar(0, 255, 0), 2);


            cv::imshow("Nesne Takibi ile Hareketli Cizgi", frame);


            simxSetJointTargetPosition(clientID, lbrJoint1, aci_x_eksen * (PI / 180), simx_opmode_oneshot_wait); //simxSetJointTargetPosition(clientID, lbrJoint1, aci_x_eksen * (PI / 180), simx_opmode_oneshot_wait);
            simxSetJointTargetPosition(clientID, lbrJoint6, aci_y_eksen * (PI / 180), simx_opmode_oneshot_wait);





            int key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }


        } 

        video_cap.release();
        simxFinish(clientID);
        return clientID;
   
    }
}