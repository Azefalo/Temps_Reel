/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    Camera  camera;
    Img img = Img(ImageMat());
    Arena arena;
    int arena_ok = 0;
    int Counter =0;
    int camOpen = 0;
    int robotStarted = 0;
    bool cameraPaused = false;
    int move = MESSAGE_ROBOT_STOP;
    int battery = 0;
    int robot_pos = 0;
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_startRobotWD;
    RT_TASK th_move;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openCam;
    RT_TASK th_closeCam;
    RT_TASK th_cam;
    RT_TASK th_position;
    RT_TASK th_Arena;
    RT_TASK th_battery;
    RT_TASK th_monitorError;
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_camera;

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_serverOk;
    RT_SEM sem_openComRobot;
    RT_SEM sem_startRobot;
    RT_SEM sem_startRobotWD;
    RT_SEM sem_cam;
    RT_SEM sem_Arena;
    RT_SEM sem_ArenaOK;
    RT_SEM sem_readBattery;

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot without WD.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread starting the communication with the robot with WD.
     */
    void StartRobotWDTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /** 
     * @brief Thread handling the opening of the camera.
     */
    void OpenCameraTask(void *arg);

    /**
     * @brief Thread handling the closing of the camera.
     */
    void CloseCameraTask(void *arg);
    
    /**
     * @brief Thread handling the camera.
     */
    void CameraTask(void *arg);
    
    /**
     * @brief Thread handling the arena.
     */
    void ArenaTask(void *arg);
    
    /**
     * @brief Thread handling the robot's position.
     */
    void CalculPositionTask (Img * img);
    
    /**
     * @brief Thread handling the robot's battery.
     */
    void BatteryTask(void *arg);
    
    /**
     * @brief Thread handling the loss of communication between monitor and supervisor.
     */
    void MonitorError (Message * msgReceived);
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);

};

#endif // __TASKS_H__ 
