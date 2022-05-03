/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "globals.h"

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#else
#include "MPU6050_6Axis_MotionApps20.h"
#endif

#include "mpu6050sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"
#include "GlobalVars.h"

#include "deepSleep.cpp"

int16_t rXt, rYt, rZt;

int16_t rXtPose1, rXtPose2, rXtPose3, rXtPose4, rXtPose5, rXtPose6, rXtPose7, rXtPose8, rXtPose9, rXtPose10;
int16_t rYtPose1, rYtPose2, rYtPose3, rYtPose4, rYtPose5, rYtPose6, rYtPose7, rYtPose8, rYtPose9, rYtPose10;
int16_t rZtPose1, rZtPose2, rZtPose3, rZtPose4, rZtPose5, rZtPose6, rZtPose7, rZtPose8, rZtPose9, rZtPose10;

unsigned long timeCount = 0;
bool timeDelayFlag = 0;

void MPU6050Sensor::motionSetup()
{
    imu.initialize(addr);
    if (!imu.testConnection())
    {
        m_Logger.fatal("Can't connect to MPU6050 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("Connected to MPU6050 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

#ifndef IMU_MPU6050_RUNTIME_CALIBRATION
    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU6050:
            m_Calibration = sensorCalibration.data.mpu6050;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }
#endif

    devStatus = imu.dmpInitialize();

    if (devStatus == 0)
    {
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
        // We don't have to manually calibrate if we are using the dmp's automatic calibration
#else  // IMU_MPU6050_RUNTIME_CALIBRATION

        m_Logger.debug("Performing startup calibration of accel and gyro...");
        // Do a quick and dirty calibration. As the imu warms up the offsets will change a bit, but this will be good-enough
        delay(1000); // A small sleep to give the users a chance to stop it from moving

        imu.CalibrateGyro(6);
        imu.CalibrateAccel(6);
        imu.PrintActiveOffsets();
#endif // IMU_MPU6050_RUNTIME_CALIBRATION

        ledManager.pattern(50, 50, 5);

        // turn on the DMP, now that it's ready
        m_Logger.debug("Enabling DMP...");
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        m_Logger.debug("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();

        working = true;
        configured = true;
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        m_Logger.error("DMP Initialization failed (code %d)", devStatus);
    }
}

void MPU6050Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#endif

    if (!dmpReady)
        return;

    if (imu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
        quaternion.set(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);
        quaternion *= sensorOffset;

#if ENABLE_INSPECTION
        {
            Network::sendInspectionFusedIMUData(sensorId, quaternion);
        }
#endif

        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
    }

    //imu.getRotation(&rXt, &rYt, &rZt);
    //timeCount = millis();
    
    //rXtPose1 = rXt;
    //if (millis() % 5000 == 0) rXtPose2 = rXt;
    
    //if (rXtPose1 == rXtPose2) ESP.deepSleep(10e6);
    //хуйню понаписал

    
    
    if (timeDelayFlag == 0)
    {
        timeCount = millis();
        Serial.println(timeCount);
        timeDelayFlag = 1;
    }
    

        //запись значений MPU каждую секунду

    if (millis() - timeCount == 1000) {
        //poseQuat1 = quaternion;

        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose1 = rXt;
        rYtPose1 = rYt;
        rZtPose1 = rZt;

        Serial.println(1);
    }
    else if (millis() - timeCount == 2000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose2 = rXt;
        rYtPose2 = rYt;
        rZtPose2 = rZt;
        //poseQuat2 = quaternion;
        Serial.println(2);
    }
    else if (millis() - timeCount == 3000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose3 = rXt;
        rYtPose3 = rYt;
        rZtPose3 = rZt;
        //poseQuat3 = quaternion;
        Serial.println(3);
    }
    else if (millis() - timeCount == 4000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose4 = rXt;
        rYtPose4 = rYt;
        rZtPose4 = rZt;
        //poseQuat4 = quaternion;
        Serial.println(4);
    }
    else if (millis() - timeCount == 5000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose5 = rXt;
        rYtPose5 = rYt;
        rZtPose5 = rZt;
        //poseQuat5 = quaternion;
        Serial.println(5);
    }
    else if (millis() - timeCount == 6000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose6 = rXt;
        rYtPose6 = rYt;
        rZtPose6 = rZt;
        //poseQuat6 = quaternion;
        Serial.println(6);
    }
    else if (millis() - timeCount == 7000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose7 = rXt;
        rYtPose7 = rYt;
        rZtPose7 = rZt;
        //poseQuat7 = quaternion;
        Serial.println(7);
    }
    else if (millis() - timeCount == 8000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose8 = rXt;
        rYtPose8 = rYt;
        rZtPose8 = rZt;
        //poseQuat8 = quaternion;
        Serial.println(8);
    }
    else if (millis() - timeCount == 9000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose9 = rXt;
        rYtPose9 = rYt;
        rZtPose9 = rZt;
        //poseQuat9 = quaternion;
        Serial.println(9);
    }
    else if (millis() - timeCount == 10000)
    {
        imu.getRotation(&rXt, &rYt, &rZt);
        rXtPose10 = rXt;
        rYtPose10 = rYt;
        rZtPose10 = rZt;
        //poseQuat10 = quaternion;

        Serial.println(10);

        

        Serial.println("Начинаю сравнивать");
        Serial.println(rXtPose1, rXtPose2);
        Serial.println(rXtPose3, rXtPose4);
        Serial.println(rXtPose5, rXtPose6);
        Serial.println(rXtPose7, rXtPose8);
        Serial.println(rXtPose9, rXtPose10);

        Serial.println(rYtPose1, rYtPose2);
        Serial.println(rYtPose3, rYtPose4);
        Serial.println(rYtPose5, rYtPose6);
        Serial.println(rYtPose7, rYtPose8);
        Serial.println(rYtPose9, rYtPose10);

        Serial.println(rZtPose1, rZtPose2);
        Serial.println(rZtPose3, rZtPose4);
        Serial.println(rZtPose5, rZtPose6);
        Serial.println(rZtPose7, rZtPose8);
        Serial.println(rZtPose9, rZtPose10);
        

        if (rXtPose1 == rXtPose2 && rXtPose2 == rXtPose3 && rXtPose3 == rXtPose4 && rXtPose4 == rXtPose5 && rXtPose5 == rXtPose6 && rXtPose6 == rXtPose7 && rXtPose7 == rXtPose8 && rXtPose8 == rXtPose9 && rXtPose9 == rXtPose10)
        {
            if (rYtPose1 == rYtPose2 && rYtPose2 == rYtPose3 && rYtPose3 == rYtPose4 && rYtPose4 == rYtPose5 && rYtPose5 == rYtPose6 && rYtPose6 == rYtPose7 && rYtPose7 == rYtPose8 && rYtPose8 == rYtPose9 && rYtPose9 == rYtPose10)
            {
                if (rZtPose1 == rZtPose2 && rZtPose2 == rZtPose3 && rZtPose3 == rZtPose4 && rZtPose4 == rZtPose5 && rZtPose5 == rZtPose6 && rZtPose6 == rZtPose7 && rZtPose7 == rZtPose8 && rZtPose8 == rZtPose9 && rZtPose9 == rZtPose10)
                {

                    Serial.println("!!!DeepSleep!!!");
                    ESP.deepSleep(ESP.deepSleepMax());
                }
                
            }

            

        }
        else
        {
            Serial.println("!!!Awake!!!");
        }

        timeDelayFlag = 0;
        

    }

}

void MPU6050Sensor::startCalibration(int calibrationType) {
    ledManager.on();

#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
    m_Logger.info("MPU is using automatic runtime calibration. Place down the device and it should automatically calibrate after a few seconds");

    // Lie to the server and say we've calibrated
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_GYRO, 0);//was CALIBRATION_TYPE_INTERNAL_GYRO for some reason? there wasn't a point to this switch
        break;
    }
#else //!IMU_MPU6050_RUNTIME_CALIBRATION
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    imu.setDMPEnabled(false);
    imu.CalibrateGyro(6);
    imu.CalibrateAccel(6);
    imu.setDMPEnabled(true);

    m_Logger.debug("Gathered baseline gyro reading");
    m_Logger.debug("Starting offset finder");
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        imu.CalibrateAccel(10);
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);//doesn't send calibration data anymore, has that been depricated in server?
        m_Calibration.A_B[0] = imu.getXAccelOffset();
        m_Calibration.A_B[1] = imu.getYAccelOffset();
        m_Calibration.A_B[2] = imu.getZAccelOffset();
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        imu.CalibrateGyro(10);
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_GYRO, 0);//doesn't send calibration data anymore
        m_Calibration.G_off[0] = imu.getXGyroOffset();
        m_Calibration.G_off[1] = imu.getYGyroOffset();
        m_Calibration.G_off[2] = imu.getZGyroOffset();
        break;
    }

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
    calibration.data.mpu6050 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    m_Logger.info("Calibration finished");
#endif // !IMU_MPU6050_RUNTIME_CALIBRATION

    ledManager.off();
}
