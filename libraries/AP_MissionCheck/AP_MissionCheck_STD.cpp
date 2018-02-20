#include "AP_MissionCheck_STD.h"

#include <stdio.h>


MissionCheck_STD::MissionCheck_STD(AP_Mission &mission, DataFlash_Class &dataflash, GCS& gcs): MissionCheck(mission,dataflash,gcs)
{
    std_mission_usable = true;
}

void MissionCheck_STD::init_mission()
{
    asprintf(&msg,"INIT Default Mission");
    logInfo(msg);

    // Nothing to do here for the moment
}

void MissionCheck_STD::notify_user()
{
    // The idea of this function is to drive the LED to notify the USER about possible errors
}

bool MissionCheck_STD::check()
{

    // To be defined
  
    if(!is_takeoff_wp_present())
    {
    	std_mission_usable = false;
		asprintf(&msg,"D: TAKEOFF WP NOT PRESENT");
		logInfo(msg);
    }
       
    if(!is_landing_sequence_present())
    {
    	std_mission_usable = false;
		asprintf(&msg,"D: LANDING SEQUENCE NOT PRESENT");
		logInfo(msg);
    }
    
    if(get_num_nav_wayponts() < 2)
    {
    	std_mission_usable = false;
		asprintf(&msg,"D: AT LEAST 2 NAV WPS REQUIRED");
		logInfo(msg);
    }
    
    return std_mission_usable;
  
}

bool MissionCheck_STD::is_landing_sequence_present()
{
	AP_Mission::Mission_Command cmd;

	bool landing_point_found = false;
	bool wp_for_transition_found = false;
	bool loiter_to_altitude_found = false;

	// The above three commands must be consecutive (excluding the do commands)

	uint16_t loiter_to_altitude_index = 0;

	uint16_t num_items = _mission.num_commands();

	if(is_landing_wp_present())
	{
		landing_point_found = true;
		asprintf(&msg,"STD: LANDING POINT FOUND");
		logInfo(msg);
	}

	if(!landing_point_found)
		return false;

	// Search for loiter to altitude waypoint starting from the end of the mission
	for(uint16_t i = num_items-1; i > 0; i--)
	{
		_mission.get_next_nav_cmd(i, cmd);

		if(cmd.id == MAV_CMD_NAV_LOITER_TO_ALT)
		{
			loiter_to_altitude_found = true;
			loiter_to_altitude_index = cmd.index;
			asprintf(&msg,"STD: LTA CMD FOUND");
			logInfo(msg);
		}
	}

	if(!loiter_to_altitude_found)
		return false;

	// Count how many navigation waypoints between loter to altitude and land
	int num_nav_commands_found = 0;
	for(uint16_t i = loiter_to_altitude_index; i < get_index_landing_wp(); i++)
	{
		_mission.get_next_nav_cmd(i, cmd);

		if(cmd.id == MAV_CMD_NAV_WAYPOINT)
		{
			num_nav_commands_found++;
			asprintf(&msg,"STD: FOUND WP BETWEEN LTA AND LAND WPs");
			logInfo(msg);
		}
	}

	// For the moment we need to have at least one nav waypoint between the LTA and the landing
	// loiter to altitude and the landing waypoint.
	if(num_nav_commands_found == 0)
		return false;

	return true;

}
