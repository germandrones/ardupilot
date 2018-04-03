/*
 * AP_MissionCheck_VWP.cpp
 *
 * Created on: December 12, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#include <stdio.h>

#include "AP_MissionCheck_HWP.h"
#include <AP_Mission/AP_Mission.h>

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

bool MissionCheck_HWP::check(Location currentLoc)
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
		asprintf(&msg,"NO LANDING WP");
		if(is_takeoff_wp_present())
		{
			AP_Mission::Mission_Command cmdLand;
			AP_Mission::Mission_Command cmdTakeOff;
		    if(_mission.get_next_nav_cmd(index_takeoff_waypoint, cmdTakeOff))
		    {
		    	cmdLand.id = MAV_CMD_NAV_LAND;
				{
		    		cmdLand.content.location = currentLoc;
					cmdLand.content.location.alt = cmdTakeOff.content.location.alt;
					cmdLand.content.location.flags = cmdTakeOff.content.location.flags;

					_mission.add_cmd(cmdLand);
					inspect_stored_mission();
					hwp_feature_usable = true;
					asprintf(&msg,"LANDING AT CURRENT POS");
				}
		    }
		}
		logInfo(msg);
    }

    if(get_num_nav_wayponts() < 3)
    {
		hwp_feature_usable = false;
		asprintf(&msg,"AT LEAST 3 NAV WPS REQUIRED");
		logInfo(msg);
    }

    return hwp_feature_usable;

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
    
    if(get_num_nav_wayponts() < 3)
    {
		hwp_feature_usable = false;
		asprintf(&msg,"AT LEAST 3 NAV WPS REQUIRED");
		logInfo(msg);
    }
    
    return hwp_feature_usable;
}

bool MissionCheck_HWP::is_landing_sequence_present()
{
	// Placeholder
	return true;
}
