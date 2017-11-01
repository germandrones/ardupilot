/*
 * AP_HeadWindLanding.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Alessandro Benini
 */

#include "AP_HeadWindLanding.h"
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_HeadWindLanding::var_info[] = {

    // @Param: VWP_ENABLE
    // @DisplayName: Enabling virtual waypoint feature
    // @Description: This parameter allows to enable/disable the virtual waypoint feature. By default this feature is enabled.
    // @User: Standard
    // @Units: Boolean
    // @Range: 0 1
    // @Increment: 1
    AP_GROUPINFO("ENABLED", 1, AP_HeadWindLanding, hwp_enabled, 1),

    // @Param: DIST_VWP1
    // @DisplayName: Radius of the extra waypoint area
    // @Description: Radius of the circle area centered in the landing waypoint where the extra waypoint will be generated
    // @User: Standard
    // @Units: m
    // @Range: 20 200
    // @Increment: 1
    AP_GROUPINFO("RADIUS", 2, AP_HeadWindLanding, hwp_radius, 200.0f),

    AP_GROUPEND

};


AP_HeadWindLanding::AP_HeadWindLanding(AP_Mission& mission, AP_AHRS_NavEKF &ahrs):
	hwp_status(HWP_NOT_GENERATED),
	hwp_error(HWP_NO_ERROR),
	_mission(mission),
	_ahrs(ahrs),
	hwp_cfg{2,4}
{}

void AP_HeadWindLanding::init_HWP(void)
{
    num_cmd = _mission.num_commands();

    // I initialize all the indexes with default values
    idx_landing_wp = -1;
    idx_last_mission_wp = -1;
    idx_hwp = 0;

    // I calculate all the indexes
    calc_index_landing_waypoint();
    calc_index_last_mission_waypoint();
    calc_index_hw_waypoints();

    hwp1 = {};
    hwp2 = {};
    hwp3 = {};
    // reduce_speed = {};

}

void AP_HeadWindLanding::calc_index_landing_waypoint(void)
{

    // Command item used for iterating through the mission
    AP_Mission::Mission_Command current_cmd;

    // Start iterating from the end of the mission
    for(int16_t i=num_cmd-1; i>=0; i--)
    {
	_mission.get_next_nav_cmd(i, current_cmd);

	if(current_cmd.id == MAV_CMD_NAV_LAND)
	{
	    idx_landing_wp = current_cmd.index;
	    break;
	}
    }

    // If I ended the for loop and the return value is -1, it means that the current mission has no
    // landing waypoints.
    if(idx_landing_wp < 0)
    {
	//GCS_SEND_MSG("Landing WP not found. VWP generation aborted.");
	hwp_error = HWP_LANDING_WP_NOT_FOUND;
    }
}

void AP_HeadWindLanding::calc_index_last_mission_waypoint(void)
{

    // Command item used for iterating through the mission
    AP_Mission::Mission_Command current_cmd;

    // Start iterating from the end of the mission, looking for the n-th last DO_NAV waypoint.
    for(int16_t i=num_cmd-1; i>=0; i--)
    {
	_mission.get_next_nav_cmd(i, current_cmd);

	if(current_cmd.id == MAV_CMD_NAV_WAYPOINT)
	{
	    idx_last_mission_wp = current_cmd.index;
	    break;
	}
    }

    if(idx_last_mission_wp < 0)
    {
	//GCS_SEND_MSG("Last Mission WP not found. VWP generation aborted.");
	hwp_error = HWP_LAST_MISSION_WP_NOT_FOUND;
    }
}

void AP_HeadWindLanding::calc_index_hw_waypoints()
{

    // Command item used for iterating through the mission
    AP_Mission::Mission_Command current_cmd;

    int16_t n = hwp_cfg.dist_lwp_idx;

    // Current number of NAV commands found
    int16_t curr_num_nav_cmd_idx = 0;

    // Start iterating from the end of the mission, looking for the n-th last DO_NAV waypoint.
    for(int16_t i=num_cmd-1; i>=0; i--)
    {
	_mission.get_next_nav_cmd(i, current_cmd);

	if(current_cmd.id == MAV_CMD_NAV_WAYPOINT)
	    ++curr_num_nav_cmd_idx;

	// When I reach the n-th DO_NAV command, I get out the loop
	if(curr_num_nav_cmd_idx == (n+1))
	{
	    idx_hwp = current_cmd.index;
	    break;
	}
    }

    // If I ended the for loop and the return value is zero, it means that
    // the current mission has up to 2 mission waypoints. It's not common but
    // it can happen.
    if(idx_hwp == 0)
	hwp_error = HWP_INDEX_NOT_FOUND;
}

bool AP_HeadWindLanding::is_change_speed_cmd_issued(const AP_Mission::Mission_Command& cmd)
{
    if(cmd.index > idx_last_mission_wp && cmd.index < idx_landing_wp && cmd.id == MAV_CMD_DO_CHANGE_SPEED)
    	return true;

    return false;
}

// This function generates the virtual waypoints to attach at the end of mission, right before the landing waypoint
void AP_HeadWindLanding::generate_hw_waypoints(const AP_Mission::Mission_Command& cmd)
{

    // Check if the current cmd is where I should generate the virtual waypoints and there are no errors
    if(cmd.index == idx_hwp && hwp_error == HWP_NO_ERROR)
    {
	//GCS_SEND_MSG("Generating VWP: %d",cmd.index);

	// If I reached the point, I attach the virtual waypoint to the end of the mission,
	// just before the landing waypoint

	// Retrieve information about the wind --------------------------------------------
	// I assume that at this point of the mission I have a good estimation of wind
	// speed and direction
	Vector3f wind;

	_ahrs.get_NavEKF2().getWind(0,wind);

	float windX = wind.x;
	float windY = wind.y;
	float modWind = sqrt(wind.x*wind.x+wind.y*wind.y);
	gcs().send_text(MAV_SEVERITY_NOTICE, "WIND_SPD:%5.2f",modWind);
	// -------------------------------------------------------------------------------

	// Retrieve the landing waypoint from the mission
	AP_Mission::Mission_Command wp;
	// The index of the mission starts from 0.
	_mission.get_next_nav_cmd(idx_landing_wp, wp);

	// Conversion of latitude and longitude from degrees to meters -------------------
	// (The reference WP for the conversion is the landing WP)
	// More information at: https://knowledge.safe.com/articles/725/calculating-accurate-length-in-meters-for-lat-long.html
	float lat = wp.content.location.lat*TO_DEG_FORMAT;
	float lng = wp.content.location.lng*TO_DEG_FORMAT;

	float mdlat = METERS_PER_DEG_LAT(lat);
	float mdlng = METERS_PER_DEG_LNG(lat);

	// GCS_SEND_MSG("LLr:%12.4f,%12.4f",lat,lng);
	// GCS_SEND_MSG("LLm:%12.4f,%12.4f",mdlat,mdlng);

	// Coordinates of the virtual waypoints
	Location loc_hwp1, loc_hwp2, loc_hwp3;
	Location land_wp = wp.content.location;

	// Retrieve the last mission waypoint from the mission
	AP_Mission::Mission_Command last_mwp;
	_mission.get_next_nav_cmd(idx_last_mission_wp, last_mwp);

	float last_mwp_lat = last_mwp.content.location.lat*TO_DEG_FORMAT;
	float last_mwp_lng = last_mwp.content.location.lng*TO_DEG_FORMAT;
	float last_mwp_alt = last_mwp.content.location.alt/100.0f;

	float land_wp_lat = land_wp.lat*TO_DEG_FORMAT;
	float land_wp_lng = land_wp.lng*TO_DEG_FORMAT;
	float land_wp_alt = land_wp.alt/100.0;

	// -------------------------------------------------------------------------------
	// The following check prevents to have more than 15 degrees of drop between the
	// last mission waypoint and the landing waypoint. The purpose is to have low speed
	// after the transition and avoid the high pitch.

	// Minimum distance between the landing waypoint and the last mission waypoint

	// sin(max_slope_deg*DEG_TO_RAD) cannot be zero since it is set as a constant value for the moment
	// minimum distance is set as max_altitude_drop * cotg(max_slope_angle).
	// In this case, we have 20*cotg(7degrees) = 20*8.1443 = 162.886.
	// This value has to be divided by 3, since we have 3 virtual waypoints.
	// Therefore minimum_distance is 54.29 --> 55 meters.

	float min_distance_last_phase = 100.0f;

	dist_hwpl_1 = hwp_radius / 2.0f;
	dist_hwp1_2 = hwp_radius / 4.0f;
	dist_hwp2_3 = dist_hwp1_2;

	// Now I check the landing waypoint altitude. If the difference is greater than 20 meters, I set it to 20.
	float max_altitude_difference = 20.0f;

	// Altitude difference is expressed in centimeters
	float altitude_difference = last_mwp.content.location.alt - wp.content.location.alt;

	if(altitude_difference > 2000.0)
	  wp.content.location.alt = last_mwp.content.location.alt - 2000.0;

	if(altitude_difference < -2000.0)
	  wp.content.location.alt = last_mwp.content.location.alt + 2000.0;

	// -------------------------------------------------------------------------------

	// GCS_SEND_MSG("LAST M_WP:%10.6f,%10.6f,%8.3f",last_mwp_lat,last_mwp_lng,last_mwp_alt);

	// Default distance of VWP when there is no wind
	// Direction of the Wind (rad)
	float thetaWind = 0.0f;
	float new_theta_hwp = 0.0f;

	float dist_hwpl_2 = dist_hwp1_2 + dist_hwpl_1;
	float dist_hwpl_3 = dist_hwp2_3 + dist_hwpl_2;

	// WindX is the component of the wind along North Axis. WindY is the component of the wind along East Axis.
	thetaWind = atan2(windY,windX);
	// New theta is the wind direction
	new_theta_hwp = thetaWind + heading_wind*DEG_TO_RAD;
	// GCS_SEND_MSG("WIND_DIR:%f",thetaWind*180.0f/3.1415f);


	// GCS_SEND_MSG("OLD L_WP:%10.6f,%10.6f,%8.3f",land_wp_lat,land_wp_lng,land_wp_alt);
	// GCS_SEND_MSG("VWPS_DIRd:%f",new_theta_vwp*180.0f/3.1415f);

	// --------------------------------------------------------------------------------
	// Difference of altitude between the landing waypoint and the last mission waypoint (in cm)
	float altitude_diff = wp.content.location.alt - last_mwp.content.location.alt;
	float step = 0.0f;

	// GCS_SEND_MSG("ALT DIFF, STEP:%f,%f",altitude_diff,step);

	// Making sure we are not dividing by zero
	if( (hwp_cfg.num_hwp-1) > 0)
	    step = altitude_diff / (hwp_cfg.num_hwp-1);

	// Calculate the coordinates of the first virtual waypoint -----------------------
	loc_hwp1.lat = land_wp.lat + dist_hwpl_1*cos(new_theta_hwp) / mdlat * 10000000.0f;
	loc_hwp1.lng = land_wp.lng + dist_hwpl_1*sin(new_theta_hwp) / mdlng * 10000000.0f;
	// The altitude is the same as the altitude of the last waypoint mission
	loc_hwp1.alt = last_mwp.content.location.alt + 3*step;
	loc_hwp1.options = 1<<0;

	// Print and save info about vwp1
	// float hwp1_lat = loc_hwp1.lat*TO_DEG_FORMAT;
	// float hwp1_lng = loc_hwp1.lng*TO_DEG_FORMAT;
	// float hwp1_alt = loc_hwp1.alt/100.0f;
	// GCS_SEND_MSG("VWP1:%10.6f,%10.6f,%8.3f",vwp1_lat,vwp1_lng,vwp1_alt);
	// -------------------------------------------------------------------------------

	// Calculate the coordinates of the second virtual waypoint ----------------------
	loc_hwp2.lat = land_wp.lat + dist_hwpl_2*cos(new_theta_hwp) / mdlat * 10000000.0f;
	loc_hwp2.lng = land_wp.lng + dist_hwpl_2*sin(new_theta_hwp) / mdlng * 10000000.0f;
	// The altitude is the same as the altitude of the last waypoint mission
	loc_hwp2.alt = last_mwp.content.location.alt + 2*step;
	loc_hwp2.options = 1<<0;

	// Print and save info about vwp2
	// float hwp2_lat = loc_hwp2.lat*TO_DEG_FORMAT;
	// float hwp2_lng = loc_hwp2.lng*TO_DEG_FORMAT;
	// float hwp2_alt = loc_hwp2.alt/100.0f;
	// GCS_SEND_MSG("VWP2:%10.6f,%10.6f,%8.3f",vwp2_lat,vwp2_lng,vwp2_alt);

	// AP_Mission::Change_Speed_Command vwp2_spd;
	// vwp2_spd.speed_type = 0;
	// The target speed is set as 80% of the current speed
	// float current_speed = ahrs.getLastGNDSpeed();
	float reduced_speed = hwp_spd; //current_speed*80.0f/100.0f;

	//if(reduced_speed < MINIMUM_SPEED_DURING_VWP)
	// reduced_speed = MINIMUM_SPEED_DURING_VWP;
	//vwp2_spd.target_ms = reduced_speed;

	// -------------------------------------------------------------------------------

	// Calculate the coordinates of the third virtual waypoint -----------------------
	loc_hwp3.lat = land_wp.lat + dist_hwpl_3*cos(new_theta_hwp) / mdlat * 10000000.0f;
	loc_hwp3.lng = land_wp.lng + dist_hwpl_3*sin(new_theta_hwp) / mdlng * 10000000.0f;
	// The altitude is the same as the altitude of the last waypoint mission
	loc_hwp3.alt = last_mwp.content.location.alt + step;
	loc_hwp3.options = 1<<0;

	// float hwp3_lat = loc_hwp3.lat*TO_DEG_FORMAT;
	// float hwp3_lng = loc_hwp3.lng*TO_DEG_FORMAT;
	// float hwp3_alt = loc_hwp3.alt/100.0f;
	// GCS_SEND_MSG("VWP3:%10.6f,%10.6f,%8.3f",vwp3_lat,vwp3_lng,vwp3_alt);
	// -------------------------------------------------------------------------------

	// The last virtual waypoint and the landinig waypoint should have the same altitude.

	// Update the mission
	// GCS_SEND_MSG("REWRITE_M");

	// Remove the old landing waypoint
	// GCS_SEND_MSG("OLD L_WP IDX:%d",wp.index);
	// GCS_SEND_MSG("Before truncate:%d",mission.num_commands());

	// GCS_SEND_MSG("After truncate:%d",mission.num_commands());

	// Adding the extra-commands (they will have the parameter p1 set to AUTOGENERATED_ITEM)

	// AP_Mission::Mission_Command vwp3 = {};
	// Copy all the properties of the last mission waypoint
	hwp3 = last_mwp;
	// Overwrite command id and waypoint coordinates
	hwp3.id = MAV_CMD_NAV_WAYPOINT;
	hwp3.content.location = loc_hwp3;
	// Add the new command to the mission

	// AP_Mission::Mission_Command vwp2 = {};
	hwp2 = last_mwp;
	hwp2.id = MAV_CMD_NAV_WAYPOINT;
	hwp2.content.location = loc_hwp2;

	// AP_Mission::Mission_Command reduce_speed = {};
	// reduce_speed.id = MAV_CMD_DO_CHANGE_SPEED;
	// reduce_speed.content.speed = vwp2_spd;

	// AP_Mission::Mission_Command vwp1 = {};
	hwp1 = last_mwp;
	hwp1.id = MAV_CMD_NAV_WAYPOINT;
	hwp1.content.location = loc_hwp1;

	if(hwp_enabled)
	{
	    _mission.truncate(idx_landing_wp);
	    _mission.add_cmd(hwp3);
	    _mission.add_cmd(hwp2);
	    // _mission.add_cmd(reduce_speed);
	    _mission.add_cmd(hwp1);
	    // For the moment the UAV will still land at the original landing waypoint
	    _mission.add_cmd(wp);
	}

	hwp_status = HWP_GENERATED;

	update_num_commands();

    }

}

void AP_HeadWindLanding::update_num_commands()
{
    // num_cmd is updated with the total number of commands after adding the virtual waypoints
    if(hwp_status > HWP_NOT_GENERATED)
	num_cmd = _mission.num_commands();
}

void AP_HeadWindLanding::restore_mission()
{
    // If I generate the virtual waypoints, I can remove them and restore the mission to its original state.
    if(hwp_enabled && hwp_status == HWP_GENERATED)
    {
	// Here I restore the original version of the mission (in case it should be reloaded)
	// GCS_SEND_MSG("Restoring original mission");
	AP_Mission::Mission_Command wp;

	// GCS_SEND_MSG("Number of commands: %d",num_commands);
	// The variable wp will contain the landinig waypoint that I need to restore
	_mission.get_next_nav_cmd(num_cmd-1, wp);

	// uint16_t landing_wp_index = wp.index;
	// GCS_SEND_MSG("Landing WP index: %d",landing_wp_index);

	// If the last command is the landing (safety check)
	if(wp.id==MAV_CMD_NAV_LAND)
	{
	    // I remove the mission items starting from the index of the original landing waypoint
	    // GCS_SEND_MSG("Truncate mission at index: %d",idx_landing_wp);
	    // GCS_SEND_MSG("Number of commands before truncate: %d",mission.num_commands());
	    _mission.truncate(idx_landing_wp);
	    // GCS_SEND_MSG("Number of commands after truncate: %d",mission.num_commands());
	    // GCS_SEND_MSG("Re-adding landing WP");
	    _mission.add_cmd(wp);
	    // GCS_SEND_MSG("Number of commands after re-adding landing WP: %d",mission.num_commands());
	}

	// Change the status
	hwp_status = HWP_REMOVED;

	update_num_commands();

    }
}

bool AP_HeadWindLanding::is_current_cmd_hwp(const AP_Mission::Mission_Command& cmd)
{
    // The virtual waypoints are supposed to be added between the last mission waypoint and the landing waypoint
    if(cmd.index > idx_last_mission_wp && cmd.index < idx_landing_wp && cmd.id == MAV_CMD_NAV_WAYPOINT)
    	return true;

    return false;
}

