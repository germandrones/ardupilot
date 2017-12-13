#include <stdio.h>
#include "AP_MissionCheck_Default.h"


MissionCheck_Default::MissionCheck_Default(AP_Mission &mission, DataFlash_Class &dataflash, GCS& gcs): MissionCheck(mission,dataflash,gcs)
{
    default_mission_usable = true;
}

void MissionCheck_Default::init_mission()
{
    asprintf(&msg,"INIT Default Mission");
    logInfo(msg);

    // Nothing to do here for the moment
}

void MissionCheck_Default::notify_user()
{
    // The idea of this function is to drive the LED to notify the USER about possible errors
}

bool MissionCheck_Default::check()
{

    // To be defined
  
    if(!is_takeoff_wp_present())
    {
		default_mission_usable = false;
		asprintf(&msg,"D: TAKEOFF WP NOT PRESENT");
		logInfo(msg);
    }
       
    if(!is_landing_wp_present())
    {
		default_mission_usable = false;
		asprintf(&msg,"D: LANDING WP NOT PRESENT");
		logInfo(msg);
    }
    
    if(get_num_nav_wayponts() < 2)
    {
		default_mission_usable = false;
		asprintf(&msg,"D: AT LEAST 2 NAV WPS REQUIRED");
		logInfo(msg);
    }
    
    return default_mission_usable;
  
}
