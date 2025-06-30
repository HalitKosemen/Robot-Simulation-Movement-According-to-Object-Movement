#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


extern "C" {
#include "extApi.h"
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

    int counter = 0;

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

        //        simxPauseCommunication(clientID,1);
        simxSetJointTargetPosition(clientID, lbrJoint1, 90.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint2, 90.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint3, 170.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint4, -90.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint5, 90.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint6, 90.0* (PI / 180), simx_opmode_oneshot_wait);
        simxSetJointTargetPosition(clientID, lbrJoint7, 0.0* (PI / 180), simx_opmode_oneshot_wait);
        //        simxPauseCommunication(clientID,0);

        //        float joint2 = 1;
        //        //simxSetJointTargetVelocity(clientID, lbrJoint2, 0.1, simx_opmode_oneshot_wait);
        //        while (simxGetConnectionId(clientID)!=-1  && WORK)              ///**<  while we are connected to the server.. */
        //        {
        ////            simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.2, simx_opmode_oneshot_wait);
        ////            simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.2, simx_opmode_oneshot_wait);
        //                TEST3 = simxSetJointTargetVelocity(clientID, lbrJoint2, -0.1, simx_opmode_oneshot);
        ////            TEST3 = simxSetJointTargetPosition(clientID, lbrJoint2, joint2 * (PI/180), simx_opmode_oneshot);
        //
        //            if(counter>1000)
        //            {
        //                simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.0, simx_opmode_oneshot_wait);
        //                simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.0, simx_opmode_oneshot_wait);
        //                simxSetJointTargetVelocity(clientID, lbrJoint2, 0, simx_opmode_oneshot_wait);
        //                break;
        //            }
        //            cout<<counter<< "  "<< TEST3<<endl;
        //            counter++;
        //            joint2 = joint2 + 0.08;
        //        }
    }
    else
    {
        cout << " Connection status to VREP: FAILED" << endl;
    }
    simxFinish(clientID);
    return clientID;
}
