// //#include "globals.h"
// #include "Arduino.h"
// //#include "deepSleep.h"

// int16_t rXtPose1, rXtPose2, rXtPose3, rXtPose4, rXtPose5, rXtPose6, rXtPose7, rXtPose8, rXtPose9, rXtPose10;
// int16_t rYtPose1, rYtPose2, rYtPose3, rYtPose4, rYtPose5, rYtPose6, rYtPose7, rYtPose8, rYtPose9, rYtPose10;
// int16_t rZtPose1, rZtPose2, rZtPose3, rZtPose4, rZtPose5, rZtPose6, rZtPose7, rZtPose8, rZtPose9, rZtPose10;

// unsigned long timeCount = 0;
// bool timeDelayFlag = 0;

// //таймер отсчёта по 10 сек
// // namespace GlueVr
// // {
//     void timer10secs()
//     {
//         if (timeDelayFlag == 0)
//         {
//             timeCount = millis();
//             Serial.println(timeCount);
//             timeDelayFlag = 1;
//         }
//     }

//     //проверка ухода в сон

//     void checkSleep()
//     {

//         if (millis() - timeCount == 10000)
//         {

//             Serial.println("Начинаю сравнивать");

//             if (rXtPose1 == rXtPose2 && rXtPose2 == rXtPose3 && rXtPose3 == rXtPose4 && rXtPose4 == rXtPose5 && rXtPose5 == rXtPose6 && rXtPose6 == rXtPose7 && rXtPose7 == rXtPose8 && rXtPose8 == rXtPose9 && rXtPose9 == rXtPose10)
//             {
//                 if (rYtPose1 == rYtPose2 && rYtPose2 == rYtPose3 && rYtPose3 == rYtPose4 && rYtPose4 == rYtPose5 && rYtPose5 == rYtPose6 && rYtPose6 == rYtPose7 && rYtPose7 == rYtPose8 && rYtPose8 == rYtPose9 && rYtPose9 == rYtPose10)
//                 {
//                     if (rZtPose1 == rZtPose2 && rZtPose2 == rZtPose3 && rZtPose3 == rZtPose4 && rZtPose4 == rZtPose5 && rZtPose5 == rZtPose6 && rZtPose6 == rZtPose7 && rZtPose7 == rZtPose8 && rZtPose8 == rZtPose9 && rZtPose9 == rZtPose10)
//                     {

//                         Serial.println("!!!DeepSleep!!!");
//                         ESP.deepSleep(ESP.deepSleepMax());
//                     }
//                     else
//                     {
//                         Serial.println("!!!Awake!!!");
//                     }

//                     timeDelayFlag = 0;
//                 }
//             }
//         }
//     }
// //}
