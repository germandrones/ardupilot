/*
 * AP_MissionCheck.cpp
 *
 * Created on: December 12, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#include <stdio.h>
#include "AP_MissionCheck.h"

MissionCheck::MissionCheck(AP_Mission& mission, DataFlash_Class &dataflash, GCS& gcs): _mission(mission), _dataflash(dataflash), _gcs(gcs)
{
    takeoff_wp_present = false;
    landing_wp_present = false;
    num_nav_wayponts = 0;

    index_takeoff_waypoint = -1;
    index_landing_waypoint = -1;
    index_return_waypoint = -1;

    inspect_stored_mission();
}

bool MissionCheck::update_land_waypoint(Location currentPosition)
{
	if(is_landing_wp_present() && !is_return_wp_present())
		return true; // normal landing at position in mission
	if(is_takeoff_wp_present()) // we need take-off for transition altitude
	{
		AP_Mission::Mission_Command cmdLand;
		AP_Mission::Mission_Command cmdTakeOff;
		AP_Mission::Mission_Command cmdReturn;
		bool result = true;
		if(_mission.get_next_nav_cmd(index_takeoff_waypoint, cmdTakeOff))
		{
			cmdLand.id = MAV_CMD_NAV_LAND;
			if(check_latlng(currentPosition)){
				cmdLand.content.location = currentPosition;// use current position for landing
				cmdLand.content.location.alt = cmdTakeOff.content.location.alt; // use takeoff alt for transition
				cmdLand.content.location.flags = cmdTakeOff.content.location.flags;

				cmdReturn = cmdLand; // copy pos and alt
				cmdReturn.id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // change id

				if(is_landing_wp_present())
					result &= _mission.replace_cmd(index_landing_waypoint, cmdLand);
				else
					result &= _mission.add_cmd(cmdLand);
				if(is_return_wp_present())
					result &= _mission.replace_cmd(index_return_waypoint, cmdReturn);
				else
					result &= _mission.add_cmd(cmdReturn);
				inspect_stored_mission();
				if(result)
					return true;
			}
		}
		// if we come here something went wrong so delete landing point
		if(is_landing_wp_present())
			_mission.truncate(index_landing_waypoint-1);
		inspect_stored_mission();
	}
	return false;
}
void MissionCheck::inspect_stored_mission()
{
  
  AP_Mission::Mission_Command cmd;
  
  index_takeoff_waypoint = -1;
  index_return_waypoint = -1;
  index_landing_waypoint = -1;
  uint16_t num_items = _mission.num_commands();
  
  for(uint16_t i = 0; i < num_items; i++)
  {
      _mission.get_next_nav_cmd(i, cmd);
      
      if(cmd.id == MAV_CMD_NAV_TAKEOFF && index_takeoff_waypoint==-1) // only first in mission
      {
    	  index_takeoff_waypoint = cmd.index;
      }

      if(cmd.id == MAV_CMD_NAV_RETURN_TO_LAUNCH)
      {
    	  index_return_waypoint = cmd.index;
      }

      if(cmd.id == MAV_CMD_NAV_LAND && index_landing_waypoint==-1) // only first in mission
      {
    	  index_landing_waypoint = cmd.index;
      }
      
      // I check if index is greater than 0 to avoid counting the home waypoint
      if(cmd.id == MAV_CMD_NAV_WAYPOINT && cmd.index > 0)
    	  ++num_nav_wayponts;
     
  }

}

int16_t MissionCheck::get_index_last_nav_WP()
{
    // Get the number of commands for the current mission
    int16_t num_cmd = _mission.num_commands();

    // Stores the current cmd item
    AP_Mission::Mission_Command current_cmd;

    // Start iterating from the end of the mission, looking for the n-th last DO_NAV waypoint.
    for(int16_t i=num_cmd-1; i>=0; i--)
    {
		_mission.get_next_nav_cmd(i, current_cmd);
		// If the current command is a NAV command
		if(current_cmd.id == MAV_CMD_NAV_WAYPOINT)
			return current_cmd.index;
    }

    // If I ended the for loop and the return value is -1, it means that the current mission has no
    // navigation waypoints.
    return -1;
}

void MissionCheck::logInfo(char* _msg)
{
    _gcs.send_text(MAV_SEVERITY_CRITICAL,_msg);
    _dataflash.Log_Write_Message(_msg);
}
