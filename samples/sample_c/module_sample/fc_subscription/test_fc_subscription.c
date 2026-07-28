/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "widget_interaction_test/test_widget_interaction.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (5)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (1024)
#define HYFLY_CALLBACK_SCRIPT "/home/sorad/dji-psdk-builds/hyfly-callback.sh"

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

static void HyFly_StatusReader();

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat_quaternion;
    T_DjiReturnCode djiStat_gpsposition;
    T_DjiReturnCode djiStat_gpsdetail;
    T_DjiReturnCode djiStat_gpstime;
    T_DjiReturnCode djiStat_gpsdate;
    T_DjiReturnCode djiStat_rtkpos;
    T_DjiReturnCode djiStat_rtkinfo;
    T_DjiReturnCode djiStat_althome;
    T_DjiReturnCode djiStat_altbar;
    T_DjiReturnCode djiStat_compass;
    T_DjiReturnCode djiStat_velocity;
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDate gpsData = {0};
    T_DjiFcSubscriptionGpsTime gpsTime = {0};
    T_DjiFcSubscriptionGpsDetails GpsDetail = {0};
    T_DjiFcSubscriptionRtkPosition PositionData = {0};
    T_DjiFcSubscriptionRtkPositionInfo rtkSolution = {0};
    T_DjiFcSubscriptionRTKConnectStatus RTKConnectStatus = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint AltHomepoint = {0};
    T_DjiFcSubscriptionCompass Mag = {0};
    T_DjiFcSubscriptionPositionFused PositionFused = {0};
    T_DjiFcSubscriptionAltitudeFused AltitudeFused = {0};
    T_DjiFcSubscriptionAltitudeBarometer Altbar = {0};

    s_userFcSubscriptionDataShow = true;

    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps detail error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gpstimes error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic gpstimes success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gpsdata error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic gpsdata success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk_position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic rtk_position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk_position_info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic rtk_position_info success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude_homepoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude_homepoint success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude_barometer error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude_barometer success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic compass error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic compass success.");
    }


    int count = 0;

    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);


        count++;

        if (count % FC_SUBSCRIPTION_TASK_FREQ == 0) {
            HyFly_StatusReader();
        }


        djiStat_althome = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                         (uint8_t *) &AltHomepoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);

        djiStat_altbar = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER,
                                                          (uint8_t *) &Altbar,
                                                          sizeof(T_DjiFcSubscriptionAltitudeBarometer),
                                                          &timestamp);

        djiStat_compass = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
                                                          (uint8_t *) &Mag,
                                                          sizeof(T_DjiFcSubscriptionCompass),
                                                          &timestamp);

        djiStat_gpsdate = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                                          (uint8_t *) &gpsData,
                                                          sizeof(T_DjiFcSubscriptionGpsDate),
                                                          &timestamp);

        djiStat_gpstime = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                          (uint8_t *) &gpsTime,
                                                          sizeof(T_DjiFcSubscriptionGpsTime),
                                                          &timestamp);

        djiStat_gpsdetail = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &GpsDetail,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);

        djiStat_rtkinfo = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkSolution,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);

        djiStat_gpsposition = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);

        djiStat_rtkpos = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                          (uint8_t *) &PositionData,
                                                          sizeof(T_DjiFcSubscriptionRtkPosition),
                                                          &timestamp);

        djiStat_velocity = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &PositionData,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);

        if ((djiStat_althome != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_altbar != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_compass != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_gpsdate != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_gpstime != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_gpsdetail != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_rtkinfo != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_gpsposition != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_velocity != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ||
           (djiStat_rtkpos != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)) {

            USER_LOG_ERROR("Error getting value from subscribed topic(s)");
        } else {
            USER_LOG_INFO(
                "{\"home_altitude\":%f, \"altitude_barometer\":%f, \"compass_x\":%d, \"compass_y\":%d, \"compass_z\":%d, \"gps_date\":%d, \"gps_time\":%d, \"timestamp_ms\":%u, \"timestamp_us\":%u}",
                Altbar, AltHomepoint, Mag.x, Mag.y, Mag.z, gpsData, gpsTime, timestamp.millisecond, timestamp.microsecond);

            USER_LOG_INFO(
               "{\"velocity_x\":%f,\"velocity_y\":%f,\"velocity_z\":%f,\"healthFlag\":%d}",
               velocity.data.x, velocity.data.y, velocity.data.z, velocity.health);

            USER_LOG_INFO(
                "{\"rtk_solution\":%d, \"gps_x\":%d,\"gps_y\":%d,\"gps_z\":%d, \"nsat\":%d, \"gps_fix\":%f, \"rtk_lon\":%lf, \"rtk_lat\":%lf, \"rtk_hfsl\":%f}",
                rtkSolution, gpsPosition.x, gpsPosition.y, gpsPosition.z, GpsDetail.totalSatelliteNumberUsed, GpsDetail.fixState, PositionData.longitude, PositionData.latitude, PositionData.hfsl);
        }

    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps_position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gpstimes error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gpstimes error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic rtk position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic rtk position info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic altitude of homepoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe altitude_barometer error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic compass error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


void HyFly_StatusReader(void)
{
    FILE *fp;
    char buffer[1024];
    char output[4096] = "";

    fp = popen(HYFLY_CALLBACK_SCRIPT, "r");
    if (fp == NULL) {
        perror("Failed to open HyFly_StatusReader input script file");
        return;
    } else {

        while (fgets(buffer, sizeof(buffer), fp) != NULL) {
            strcat(output, buffer);
        }

        pclose(fp);

        // printf("Script output:\n%s", output);
        // report back to floating widget log on controller app
        DjiTest_WidgetLogAppend("Hy-Fly: %s\n", output);
    }
}


T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    dji_f64_t pitch, yaw, roll;

    USER_UTIL_UNUSED(dataSize);

    pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                             -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                             -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) * 57.3;

    USER_LOG_INFO(
        "{\"quaternion_0\":%f, \"quaternion_1\":%f, \"quaternion_2\":%f, \"quaternion_3\":%f, \"pitch\":%.2f, \"roll\":%.2f, \"yaw\":%.2f}",
        quaternion->q0, quaternion->q1, quaternion->q2, quaternion->q3, pitch, roll, yaw);

    // report back to floating widget log on controller app
    //DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);


    //if (s_userFcSubscriptionDataShow == true) {
    //    if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
            //USER_LOG_INFO("receive quaternion data.");
    //        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
    //                      timestamp->microsecond);
    //       USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2,
    //                      quaternion->q3);

    //        USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);
    //        DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
    //    }
    //}

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
