#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

class myApp FINAL : private wb::ResourceClient, public wb::LaunchableModule
{
public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;
    myApp();
    ~myApp();

private:
    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;

    /** @see whiteboard::ResourceClient::onPostResult */
    virtual void onPostResult(wb::RequestId requestId,
                              wb::ResourceId resourceId,
                              wb::Result resultCode,
                              const wb::Value& rResultData) OVERRIDE;

    /** @see whiteboard::ResourceClient::onGetResult */
    virtual void onGetResult(wb::RequestId requestId,
                             wb::ResourceId resourceId,
                             wb::Result resultCode,
                             const wb::Value& rResultData);

    /** @see whiteboard::ResourceClient::onGetResult */
    virtual void onSubscribeResult(wb::RequestId requestId,
                                   wb::ResourceId resourceId,
                                   wb::Result resultCode,
                                   const wb::Value& rResultData);

    /** @see whiteboard::ResourceClient::onNotify */
    virtual void onNotify(wb::ResourceId resourceId,
                          const wb::Value& rValue,
                          const wb::ParameterList& rParameters);
    virtual void onTimer(wb::TimerId timerId) OVERRIDE;

private:
    void configGattSvc();
    void unsubscribeAllStreams();

    wb::ResourceId mCommandCharResource;
    wb::ResourceId mDataCharResource;
    wb::TimerId mMeasurementTimer;

    int32_t mSensorSvcHandle;
    int32_t mCommandCharHandle;
    int32_t mDataCharHandle;

    uint16_t* mLedInstructionPtr;
    uint16_t* mLedRepeatPtr;
    size_t mLedPatternRemaining;
    wb::TimerId mLedTimer=wb::ID_INVALID_TIMER;
    uint16_t mRepeatingPattern[3];

    bool mNotificationsEnabled;

    // Data subscriptions

    struct DataSub {
        wb::ResourceId resourceId;
        uint8_t clientReference;
        bool subStarted;
        bool subCompleted;
    };
    static constexpr size_t MAX_DATASUB_COUNT = 4;
    DataSub mDataSubs[MAX_DATASUB_COUNT];

    DataSub *getFreeDataSubSlot();

    // Buffer for outgoing data messages (MTU -3)
    uint8_t mDataMsgBuffer[158];

    DataSub* findDataSub(const wb::ResourceId resourceId);
    DataSub* findDataSubByRef(const uint8_t clientReference);
  
    void handleIncomingCommand(const wb::Array<uint8> &commandData);

    //returns basic status; reference CANNOT be 0
    //currently confirms null termination and non-zero reference
    bool subscribe(const char path[], size_t pathlen, uint8_t reference);
    void unsubscribe(uint8_t reference);
    //send information over GATT connection, arbitrary maximum of 64 bits
    void sendPacket(const uint8_t data[], size_t len, uint8_t tag=0, uint8_t type=2);
    //serialize sensor data to data buffer
    uint8_t mSerializedData[256];
    size_t serializeData(wb::ResourceId resourceId, const wb::Value &value);

    //current status of the LED
    bool ledStatus=false;
    //controls for hardware LED, true is on, false is off
    void ledSet(bool on);
    //repeatedly flash LED multiple by setting a duration for the LED to be on and off
    void ledSetPattern(uint16_t onDuration, uint16_t offDuration, size_t repetitions, bool startOn=true);
    //flash the LED in a custom pattern given by an array of millisecond durations 
    //should be null terminated
    void ledSetPattern(uint16_t* pattern, bool startOn=true);
    //same as previous function with a max length to avoid overflow. (think strncpy)
    void ledSetPattern_n(uint16_t* pattern, size_t length, bool startOn=true);

    //called with any incomming data from subscribed sensors
    void processData(wb::ResourceId resourceId, const wb::Value &value);
    //called with raw serial input from GATT connection
    void handleCommand(uint8_t cmd,const uint8_t values[], size_t len);
};
