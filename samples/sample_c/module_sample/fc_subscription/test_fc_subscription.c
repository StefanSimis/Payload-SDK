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

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (1024)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};
    T_DjiFcSubscriptionGpsDate gpsData = {0};
    T_DjiFcSubscriptionGpsTime gpsTime = {0};
    T_DjiFcSubscriptionGpsDetails GpsDetail = {0};
    T_DjiFcSubscriptionRtkPosition PositionData = {0};
    T_DjiFcSubscriptionRtkPositionInfo rtkSolution = {0};
    T_DjiFcSubscriptionRTKConnectStatus RTKConnectStatus = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint AltHomepoint = {0};

    s_userFcSubscriptionDataShow = true;

    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps detail error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
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

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk_position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic rtk_position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk_position_info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_INFO("Subscribe topic rtk_position_info success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk connect status error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk_connect_status success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude_homepoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude_homepoint success.");
    }


    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        } else {
            USER_LOG_INFO(
                "{\"velocity_x\":%f,\"velocity_y\":%f,\"velocity_z\":%f,\"healthFlag\":%d,\"timestamp_ms\":%d,\"timestamp_us\":%d}",
                velocity.data.x,
                velocity.data.y,
                velocity.data.z, velocity.health, timestamp.millisecond, timestamp.microsecond);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        } else {
            USER_LOG_INFO(
                "{\"gps_x\":%d,\"gps_y\":%d,\"gps_z\":%d}",
                gpsPosition.x,gpsPosition.y,gpsPosition.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &GpsDetail,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps details error.");
        } else {
            USER_LOG_INFO("{\"gps_fix\":%.0f,\"nsat\":%d}",
            GpsDetail.fixState, GpsDetail.totalSatelliteNumberUsed);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index1 error.");
        } else {
            USER_LOG_INFO(
                "{\"battery1_capacity\":%ld,\"battery1_voltage\":%ld,\"battery1_temperature\":%.2f}",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index2 error.");
        } else {
            USER_LOG_INFO(
                "{\"battery2_capacity\":%ld,\"battery2_voltage\":%ld,\"battery2_temperature\":%.2f}",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                          (uint8_t *) &gpsTime,
                                                          sizeof(T_DjiFcSubscriptionGpsTime),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps times error.");
        } else {
            USER_LOG_INFO(
                "{\"gps_time\":%d}",
                gpsTime);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkSolution,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk position error.");
        } else {
            USER_LOG_INFO(
                "{\"rtk_solution\":%d}",
                rtkSolution);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                         (uint8_t *) &AltHomepoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude_homepoint error.");
        } else {
            USER_LOG_INFO(
                "{\"home_altitude\":%f}",
                AltHomepoint);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,
                                                          (uint8_t *) &RTKConnectStatus,
                                                          sizeof(T_DjiFcSubscriptionRTKConnectStatus),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk connect status error.");
        } else {
            USER_LOG_INFO(
                "{\"rtk_connection\":%hu}",
                RTKConnectStatus.rtkConnected);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                                          (uint8_t *) &gpsData,
                                                          sizeof(T_DjiFcSubscriptionGpsDate),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps date error.");
         } else {
            USER_LOG_INFO(
                "{\"gps_date\":%d}",
                gpsData);
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
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
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

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
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
        "{\"timestamp_ms\":%u, \"timestamp_us\":%u}",
        timestamp->millisecond, timestamp->microsecond);
    USER_LOG_INFO(
        "{\"quaternion_0\":%f, \"quaternion_1\":%f, \"quaternion_2\":%f, \"quaternion_3\":%f}",
        quaternion->q0, quaternion->q1, quaternion->q2, quaternion->q3);
    USER_LOG_INFO(
        "{\"pitch\":%.2f, \"roll\":%.2f, \"yaw\":%.2f}",
        pitch, roll, yaw);
    // report back to floating widget log on controller app
    DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);


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
