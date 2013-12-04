#define THISFIRMWARE "ArduTarget V3.1-rc7"



////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_Notify.h>          // Notify library
#include <AP_Scheduler.h>       // main loop scheduler


// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration



////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;



////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;
AP_GPS_UBLOX    g_gps_driver;




static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_GPS,            2,     900 },
    { userhook_SlowLoop,     1000,    100 },
};

void setup() {

  init_ardutarget();


  //This sends at 115200
  hal.console->println("\nReady to TRACK");

  // initialise the main loop scheduler
  scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}


void loop() {

    uint32_t timer = micros();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 10000) - micros();
    scheduler.run(time_available - 300);




}


static void init_ardutarget(void) {
  // Do GPS init
  g_gps = &g_gps_driver;
  // GPS Initialization
  g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);
}

// called at 50hz
static void update_GPS(void)
{
    static uint32_t last_gps_reading;           // time of last gps message
    static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location

    g_gps->update();

    // logging and glitch protection run after every gps message
    if ((g_gps->last_message_time_ms() != last_gps_reading)) {
        last_gps_reading = g_gps->last_message_time_ms();
    }

}

static void userhook_SlowLoop(void) {
  hal.console->printf("GPS is at: %d\n", g_gps->time_week_ms);

}

AP_HAL_MAIN();