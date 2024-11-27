/************************ STM32NUCLEO IOT Contest ******************************
 *
 *                   Green Building IoT Solution for
 *                Plant Life Monitoring And Maintenance
 *
 *                           Authored by
 *                        Dien Hoa Truong
 *                 Muhammad Haziq Bin Kamarul Azman
 *
 *  main.cpp | Program main
 *
 ******************************************************************************/

/** Includes **/
#include "mbed.h"                                                               // ARM mbed               library
#include "x_nucleo_iks01a1.h"                                                   // STM32NUCLEO board      library
#include "ble/BLE.h"                                                            // Bluetooth LE           library
#include "GreenBuildingService.h"                                               // Green Building service library



/** Defines **/
#define GB_SOIL_MOISTURE_MAX 70                                                 // Soil moisture threshold value
#define GB_USERS_CONNECTED_MAX 4                                                // Maximum connected users
#define GB_PUMP_CYCLE_MAX 40                                                    // Maximum pump cycle allowed



/** Declarations **/

// Board-specific
PwmOut     pumpPWM(PC_8);                                                       // PWM motor control out pin
DigitalOut led1(LED1, 1);                                                       // Debug pin instance
AnalogIn   moisture_sensor(PB_1);                                               // Moisture sensor
static X_NUCLEO_IKS01A1 *mems_expansion_board = X_NUCLEO_IKS01A1::Instance(D14, D15); // Expansion board instance
static HumiditySensor   *humidity_sensor      = mems_expansion_board->ht_sensor;      // Expansion board humidity sensor instance
static TempSensor       *temp_sensor          = mems_expansion_board->ht_sensor;      // Expansion board temperature sensor instance

// BLE-specific
BLE&                  ble = BLE::Instance(BLE::DEFAULT_INSTANCE);               // BLE device instance
const static char     DEVICE_NAME[] = "GB-Sensor";                              // Device name
static const uint16_t uuid16_list[] = {GreenBuildingService::UUID_GREEN_BUILDING_SERVICE}; // UUID service list
GreenBuildingService *gbServicePtr;                                             // Service pointer
//Gap::Handle_t         bleConnectionHandles[GB_USERS_CONNECTED_MAX];             // Connection handle tables

// Program-specific
float getMoistureValue();
float getHumidityValue();
float getTemperatureValue();
void  errorLoop(void);
void  activateFastSensorPoll();
void  deactivateFastSensorPoll();
void  pumpActivateCallback(void);
void  pumpDeactivateCallback(void);

Ticker  sanityTicker;
Ticker  sensorPollTicker;
Ticker  fastSensorPollTicker;
Timeout pumpWaitTimeout;

uint8_t pollWaitCount = 0;
uint8_t pumpWaitTime = 3;                                                       // Pump waiting time
uint8_t usersConnected;
bool    sensorPolling;
bool    fastSensorPolling;
bool    bleActive;
bool    pumpActive;



/** Callbacks **/

// BLE-specific callback
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)    // Callback for everytime the connection gets disconnected
{
    ble.gap().startAdvertising();                                               // Restart advertising
    if((!pumpActive)||(!usersConnected))
        deactivateFastSensorPoll();
    bleActive = false;
    --usersConnected;
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)          // Callback for everytime the connection is established
{
    if(usersConnected>GB_USERS_CONNECTED_MAX)
        ble.disconnect(params->handle, Gap::REMOTE_USER_TERMINATED_CONNECTION); // Disconnect automatically due to connection constraint
    else
    {
        activateFastSensorPoll();
        bleActive = true;
        ++usersConnected;
    }
}

void onBleInitError(BLE &ble, ble_error_t error)
{
    errorLoop();
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {                                              // Check to see init errors
        onBleInitError(ble, error);
        errorLoop();
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {                         // If this is not default instance (double instanciation?)
        errorLoop();
    }
    
    ble.gap().onDisconnection(disconnectionCallback);                           // Register disconnection callback
    ble.gap().onConnection(connectionCallback);                                 // Register connection callback

    gbServicePtr = new GreenBuildingService(ble);                               // Init service with initial value
    
    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,(uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();
}

void sanityCallback(void)
{
    led1 = !led1;                                                               // Blink LED1 to indicate system sanity
}

void sensorPollCallback(void)
{
    if(++pollWaitCount==2)
    {
        pollWaitCount = 0;
        sensorPolling = true;
    }
}

void fastSensorPollCallback(void)
{
    fastSensorPolling = true;
}

void pumpActivateCallback(void)
{
    pumpPWM.write(0.7);
}

void pumpDeactivateCallback(void)
{
    pumpPWM.write(1);
}



/** Functions **/

// Helper functions for retrieving data from sensors
float getMoistureValue()
{
    float moisture = 0;
    for (int i=1;i<=10;i++) {
        moisture += moisture_sensor.read();                                     // Get ten samples
    }
    moisture = moisture / 10;
    moisture = moisture * 3300;                                                 // Change the value to be in the 0 to 3300 range
    moisture = moisture / 33;                                                   // Convert to percentage
    return moisture;
}

float getHumidityValue()
{
    float humidity = 0;
    humidity_sensor->GetHumidity(&humidity);
    return humidity;
}

float getTemperatureValue()
{
    float temperature = 0;
    temp_sensor->GetTemperature(&temperature);
    return temperature;
}


// Miscellaneous functions
void activateFastSensorPoll(void)
{
    fastSensorPolling = true;
    fastSensorPollTicker.attach(&fastSensorPollCallback, 1.9f);
}

void deactivateFastSensorPoll(void)
{
    fastSensorPolling = false;
    fastSensorPollTicker.detach();
}

void pumpSetup(void)
{
    pumpPWM.write(1);
    pumpPWM.period(2.0f);
}

void errorLoop(void)
{
    sanityTicker.detach();
    sensorPollTicker.detach();
    ble.shutdown();

    while(true)
    {
        led1 != led1;
        printf("\n\n\n\n\n\n\n\n");
    }
}



/** Pre-main inits **/



/** Main loop **/
int main(void)
{
    pumpSetup();                                                                // Setup pump
        
    sensorPolling     = false;
    fastSensorPolling = false;
    bleActive         = false;
    pumpActive        = false;
    
    sanityTicker.attach(sanityCallback, 1.1f);                                  // LED sanity checker
    sensorPollTicker.attach(sensorPollCallback, 1799.9f);                       // Sensor poll ticker (30 mins)
 
    volatile GreenBuildingService::PlantEnvironmentType_t peVal;                // Plant environment var
    uint8_t pumpLimitCount = 0;                                                 // Pump limit counter
 
    ble.init(bleInitComplete);                                                  // Pass BLE init complete function upon init
    
//    while(ble.hasInitialized() == false);                                       // Buggy loop
    
    // Infinite loop
    while (true) {
        
        if(sensorPolling || fastSensorPolling)
        {
            sensorPolling     = false;                                          // Deassert polling bit
            fastSensorPolling = false;
            
            peVal.soilMoisture   = (uint8_t) getMoistureValue();                // Update all measurements
            peVal.airHumidity    = (uint8_t) getHumidityValue();
            peVal.airTemperature = (int8_t)  getTemperatureValue();
            
            if(ble.getGapState().connected)                                     // Update characteristic if connected
                gbServicePtr->updatePlantEnvironment(peVal);
            
            printf("%d\t%d\t%d\r\n", peVal.airTemperature, peVal.airHumidity, peVal.soilMoisture);
            
            // If moisture is below 50% of max when user is present
            //    or if less than 30% of max
            //    and pump is not active
            if( ( ((peVal.soilMoisture < 0.5*GB_SOIL_MOISTURE_MAX) &&  ble.getGapState().connected)   ||
                  ((peVal.soilMoisture < 0.3*GB_SOIL_MOISTURE_MAX) && !ble.getGapState().connected)   &&
                  !pumpActive
                )
            )
            {
                activateFastSensorPoll();
                pumpActive = true;
                // TODO: calculate pumpWaitTime ( pumpWaitTime = f(peVal.airHumidity, peVal.airTemperature) )
                pumpWaitTimeout.attach(&pumpActivateCallback, pumpWaitTime);
            }
            
            // Stop condition: when soil moisture is at 60% of max or after few pump cycles
            if(pumpActive)
            {
                ++pumpLimitCount;
                
                if( (peVal.soilMoisture >= 0.6*GB_SOIL_MOISTURE_MAX) ||
                    (pumpLimitCount>GB_PUMP_CYCLE_MAX)
                )
                {                                                               // Generally here, we should notify user when there is no water
                    pumpWaitTimeout.detach();
                    pumpDeactivateCallback();
                    pumpActive = false;
                    if(!bleActive)
                        deactivateFastSensorPoll();
                    pumpLimitCount = 0;
                }
            }
            
        }
        else
            ble.waitForEvent();                                                 //Low power wait for event   
        
    }
}
