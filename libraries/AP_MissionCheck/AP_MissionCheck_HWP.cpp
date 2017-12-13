#include <stdio.h>

#include "AP_MissionCheck_HWP.h"

MissionCheck_HWP::MissionCheck_HWP(AP_Mission &mission, DataFlash_Class &dataflash, AP_HeadWindLanding &headwind_wp, GCS& gcs) :
	MissionCheck(mission,dataflash,gcs), _headwind_wp(headwind_wp)
{
    hwp_feature_usable = true;
}

void MissionCheck_HWP::init_mission()
{
    asprintf(&msg,"INIT VWP Mission");
    logInfo(msg);
    _headwind_wp.init_HWP();
}

void MissionCheck_HWP::notify_user()
{
    // The idea of this function is to drive the LED to notify the USER about possible errors
}

bool MissionCheck_HWP::check()
{
    
    // Here I check the basic requirements for using the VWP feature
  
    if(!is_takeoff_wp_present())
    {
		hwp_feature_usable = false;
		asprintf(&msg,"TAKEOFF WP NOT PRESENT");
		logInfo(msg);
    }
       
    if(!is_landing_wp_present())
    {
		hwp_feature_usable = false;
		asprintf(&msg,"LANDING WP NOT PRESENT");
		logInfo(msg);
    }
    
    asprintf(&msg,"Num Nav WP: %d",get_num_nav_wayponts());
    logInfo(msg);
    
    if(get_num_nav_wayponts() < 3)
    {
		hwp_feature_usable = false;
		asprintf(&msg,"AT LEAST 3 NAV WPS REQUIRED");
		logInfo(msg);
    }
    
    return hwp_feature_usable;
  
}
