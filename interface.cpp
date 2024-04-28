#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"

//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    // BEGIN_SUB=1,
    // END_SUB=2,
    START = 1,
    STOP = 2,
    // LED = 3,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};
          
const char IMUPath[]="/Meas/IMU6/52";
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex
size_t sample_counter_sip = 0;
size_t sip_num = 0;
size_t sample_counter_eat = 0;
size_t eat_num = 0;
size_t sample_counter_type = 0;

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = { 'Y','i','c','h','u','a', 'n' }; // {'H','e','l','l','o','!'};
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
        }
        break;
        case Commands::START:
        {
            //unsubscribes to prevent duplicate subscriptions
            unsubscribe(DEFAULT_REFERENCE);
            //subscribes to the path given above, in this case the Gyro at 52hz
            subscribe(IMUPath, sizeof(IMUPath), DEFAULT_REFERENCE);
            sip_num = 0;
            eat_num = 0;
        }
        break;
        case Commands::STOP:
        {
            //unsubscribes only from default service
            unsubscribe(DEFAULT_REFERENCE);
            sip_num = 0;
            eat_num = 0;
        }
        break;
        //case Commands::LED:
        //{
        //    uint16_t onDuration = 500;
        //    uint16_t offDuration = 2000;
        //    size_t repetitions = 4;
        //    ledSetPattern(onDuration, offDuration, repetitions, false);
        //}
        //break;
    }
}


void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    //sip count
    //this is inspired by jumpmeter_app in the lib
    const WB_RES::IMU6Data& Value = value.convertTo<const WB_RES::IMU6Data&>();
    if (Value.arrayAcc.size() <= 0)
    {
        // No value, do nothing...
        return;
    }
    const wb::Array<wb::FloatVector3D>& arrayData = Value.arrayAcc;
    const wb::Array<wb::FloatVector3D>& arrayData1 = Value.arrayGyro;

    for (size_t i = 0; i < arrayData.size(); i++)
    {
            wb::FloatVector3D accValue = arrayData[i];
            wb::FloatVector3D gyroValue = arrayData1[i];

            const float Xupper_sip = -4.0f;
            const float Xlower_sip = -8.0f;
            
            const size_t min_sample_sip = 40;

            if (accValue.mX < Xupper_sip and accValue.mX > Xlower_sip and accValue.mY > accValue.mX)
            {
                sample_counter_sip++;
            }
            else
            {
                // See if we have been long enough to be a sip
                if (sample_counter_sip >= min_sample_sip)
                {
                    sip_num++;
                    if (sip_num == 100){
                        sip_num = 0;
                    }
                    char sip_str[3]; 
                    sprintf(sip_str, "%02d", sip_num);
                    uint8_t helloMsg[] = { 'D','r','i','n','k'};
                    uint8_t tag = 1;
                    sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
                    uint16_t onDuration = 100;
                    uint16_t offDuration = 100;
                    size_t repetitions = 3;
                    ledSetPattern(onDuration, offDuration, repetitions, false);
                }
                // reset counter
                sample_counter_sip = 0;
            }


            const float Xupper_eat = -4.0f;
            const float Xlower_eat = -9.0f;
            // const float Yupper_eat = -5.0f;
            // const float Ylower_eat = -8.0f;
            const float Zupper_eat = 7.0f;
            const float Zlower_eat = 2.0f;

            const float Zlower1_eat = 150.0f;

            const size_t min_sample_eat = 5;

            if (accValue.mX < Xupper_eat and accValue.mX > Xlower_eat and 
                //accValue.mY < Yupper_eat and accValue.mY > Ylower_eat and
                accValue.mZ < Zupper_eat and accValue.mZ > Zlower_eat and
                gyroValue.mZ > Zlower1_eat)
            {
                sample_counter_eat++;
            }
            else
            {
                // See if we have been long enough to be eating
                if (sample_counter_eat >= min_sample_eat)
                {
                    eat_num++;
                    if (eat_num == 100) {
                        eat_num = 0;
                    }
                    char eat_str[3];
                    sprintf(eat_str, "%02d", eat_num);
                    uint8_t helloMsg[] = { 'S','n','a','c','k'};
                    uint8_t tag = 1;
                    sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
                    uint16_t onDuration = 100;
                    uint16_t offDuration = 100;
                    size_t repetitions = 3;
                    ledSetPattern(onDuration, offDuration, repetitions, false);
                }
                // reset counter
                sample_counter_eat = 0;
            }
    }
}
