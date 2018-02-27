/*
 * AP_HeadWindLanding.cpp
 *
 * Created on: Nov 1, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#include "AP_HeadWindLanding.h"
#include <GCS_MAVLink/GCS.h>

#define MIN_RADIUS_DURING_LOITER 30
#define MAX_RADIUS_DURING_LOITER 150

#define MIN_RADIUS_HEADINGWIND_WAYPOINT 100
#define MAX_RADIUS_HEADINGWIND_WAYPOINT 400

const AP_Param::GroupInfo AP_HeadWindLanding::var_info[] = {

    // @Param: HWP_ENABLE
    // @DisplayName: Enabling heading waypoint feature
    // @Description: This parameter allows to enable/disable the heading waypoint feature. By default this feature is enabled.
    // @User: Standard
    // @Units: Boolean
    // @Range: 0 1
    // @Increment: 1
    AP_GROUPINFO("ENABLED", 1, AP_HeadWindLanding, hwp_enabled, 0),

    // @Param: RADIUS
    // @DisplayName: Radius of the heading waypoint area
    // @Description: Radius of the area where the Heading Wind waypoints will be generated
    // @User: Standard
    // @Units: m
    // @Range: 100 400
    // @Increment: 1
    AP_GROUPINFO("RADIUS", 2, AP_HeadWindLanding, hwp_radius, 200),

    // @Param: LOITER_RADIUS
    // @DisplayName: Radius of the loiter waypoint
    // @Description: Radius of the loiter circle for reaching the desired altitude during the landing phase
    // @User: Standard
    // @Units: m
    // @Range: 30 150
    // @Increment: 1
    AP_GROUPINFO("LRADIUS", 3, AP_HeadWindLanding, loiter_radius, 60),

    AP_GROUPEND

};

AP_HeadWindLanding::AP_HeadWindLanding(AP_Mission &mission, AP_AHRS_NavEKF &ahrs):
	hwp_status(HWP_NOT_INITIALIZED),
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

    begin_no_landing_area = 0.0f;
    end_no_landing_area = 0.0f;

    is_no_landing_area_set = false;

    // I calculate all the indexes
    calc_index_landing_waypoint();
    calc_index_last_mission_waypoint();
    calc_index_hw_waypoints();

    check_no_landing_area_defined();

    hwp1 = {};
    hwp2 = {};
    hwp3 = {};
    // reduce_speed = {};

    hwp_status = HWP_INITIALIZED;
}

bool AP_HeadWindLanding::is_disable_HWP_command_present()
{
	  AP_Mission::Mission_Command cmd;

	  uint16_t num_items = _mission.num_commands();

	  for(uint16_t i = 0; i < num_items; i++)
	  {
	      _mission.get_next_nav_cmd(i, cmd);

	      if(cmd.id == MAV_CMD_DO_DISABLE_HWP)
	      {
	    	  return false;
	      }
	  }
	  return true;
}

void AP_HeadWindLanding::check_no_landing_area_defined(void)
{
    // Command item used for iterating through the mission
    AP_Mission::Mission_Command current_cmd;

    // Start iterating from the end of the mission
    for(int16_t i=num_cmd-1; i>=0; i--)
    {
		_mission.get_next_nav_cmd(i, current_cmd);

		if(current_cmd.id == MAV_CMD_SET_FORBIDDEN_ZONE)
		{
			is_no_landing_area_set = true;
			begin_no_landing_area = current_cmd.content.forbidden_zone.begin_area_sector;
			end_no_landing_area = begin_no_landing_area + current_cmd.content.forbidden_zone.offset;

			if(end_no_landing_area < 360.0)
				end_no_landing_area += 360.0;
			if(end_no_landing_area > 360.0)
				end_no_landing_area -= 360.0;
			return;
		}
    }

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

bool AP_HeadWindLanding::all_conditions_satisfied()
{
	return (is_hwp_enabled() && hwp_status == HWP_INITIALIZED && hwp_error == HWP_NO_ERROR);
}

// This function generates the virtual waypoints to attach at the end of mission, right before the landing waypoint
void AP_HeadWindLanding::generate_hw_waypoints(const AP_Mission::Mission_Command& cmd)
{

    // Check if the current cmd is where I should generate the virtual waypoints and there are no errors
    if(cmd.index == idx_hwp && all_conditions_satisfied())
    {
		//GCS_SEND_MSG("Generating VWP: %d",cmd.index);

		// If I reached the point, I attach the virtual waypoint to the end of the mission,
		// just before the landing waypoint

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

		// Coordinates of the virtual waypoints
		Location loc_hwp1, loc_hwp2, loc_hwp3;
		Location land_wp = wp.content.location;

		// Retrieve the last mission waypoint from the mission
		AP_Mission::Mission_Command last_mwp;
		_mission.get_next_nav_cmd(idx_last_mission_wp, last_mwp);

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

		if(hwp_radius < MIN_RADIUS_HEADINGWIND_WAYPOINT)
			hwp_radius = MIN_RADIUS_HEADINGWIND_WAYPOINT;

		if(hwp_radius > MAX_RADIUS_HEADINGWIND_WAYPOINT)
			hwp_radius = MAX_RADIUS_HEADINGWIND_WAYPOINT;

		dist_hwpl_1 = hwp_radius / 2.0f;
		dist_hwpl_2 = hwp_radius * (3.0f/4.0f);
		dist_hwpl_3 = hwp_radius;

		// Retrieve information about the wind --------------------------------------------
		// I assume that at this point of the mission I have a good estimation of wind
		// speed and direction
		Vector3f wind;

		_ahrs.get_NavEKF2().getWind(0,wind);

		float modWind = sqrt(wind.x*wind.x+wind.y*wind.y);
		// gcs().send_text(MAV_SEVERITY_NOTICE, "WIND_SPD:%f",modWind);
		// -------------------------------------------------------------------------------

		// Default distance of VWP when there is no wind
		// Direction of the Wind (rad)
		float thetaWind = 0.0f;
		float theta_hwp = 0.0f;

		// WindX is the component of the wind along North Axis. WindY is the component of the wind along East Axis.
		thetaWind = atan2(wind.y,wind.x)*180.0/M_PI;

		// New theta is the angle from north along which the HWP will be generated

		// TODO: rename wp to land_wp
		theta_hwp = calc_theta_hwp(thetaWind,last_mwp,wp);

		float theta_hwp_rad = theta_hwp*M_PI/180.0f;

		// GCS_SEND_MSG("WIND_DIR:%f",thetaWind*180.0f/3.1415f);

		// Calculate the coordinates of the first heading wind waypoint -----------------------
		loc_hwp1.lat = land_wp.lat + dist_hwpl_1*cos(theta_hwp_rad) / mdlat * 10000000.0f;
		loc_hwp1.lng = land_wp.lng + dist_hwpl_1*sin(theta_hwp_rad) / mdlng * 10000000.0f;
		// The altitude is the same as the altitude of the last waypoint mission
		loc_hwp1.alt = land_wp.alt;
		loc_hwp1.options = 1<<0; // Relative altitude

		// Calculate the coordinates of the second heading waypoint ----------------------
		loc_hwp2.lat = land_wp.lat + dist_hwpl_2*cos(theta_hwp_rad) / mdlat * 10000000.0f;
		loc_hwp2.lng = land_wp.lng + dist_hwpl_2*sin(theta_hwp_rad) / mdlng * 10000000.0f;
		// The altitude is the same as the altitude of the last waypoint mission
		loc_hwp2.alt = land_wp.alt;
		loc_hwp2.options = 1<<0;

		// AP_Mission::Change_Speed_Command vwp2_spd;
		// vwp2_spd.speed_type = 0;
		// The target speed is set as 80% of the current speed
		// float current_speed = ahrs.getLastGNDSpeed();
		float reduced_speed = hwp_spd; //current_speed*80.0f/100.0f;

		// Calculate the coordinates of the third virtual waypoint -----------------------
		loc_hwp3.lat = land_wp.lat + dist_hwpl_3*cos(theta_hwp_rad) / mdlat * 10000000.0f;
		loc_hwp3.lng = land_wp.lng + dist_hwpl_3*sin(theta_hwp_rad) / mdlng * 10000000.0f;
		// The altitude is the same as the altitude of the last waypoint mission
		loc_hwp3.alt = land_wp.alt;
		loc_hwp3.flags.relative_alt = 1;
		loc_hwp3.flags.loiter_xtrack = 1;

		// Adding the extra-commands (they will have the parameter p1 set to AUTOGENERATED_ITEM)

		// Copy all the properties of the last mission waypoint
		hwp3 = last_mwp;
		// Overwrite command id and waypoint coordinates
		hwp3.id = MAV_CMD_NAV_LOITER_TO_ALT;
		hwp3.content.location = loc_hwp3;

		if(loiter_radius < MIN_RADIUS_DURING_LOITER)
			loiter_radius = MIN_RADIUS_DURING_LOITER;

		if(loiter_radius > MAX_RADIUS_DURING_LOITER)
			loiter_radius = MAX_RADIUS_DURING_LOITER;

		hwp3.p1 = (uint16_t)loiter_radius;

		// Add the new command to the mission

		hwp2 = last_mwp;
		hwp2.id = MAV_CMD_NAV_WAYPOINT;
		hwp2.content.location = loc_hwp2;

		// AP_Mission::Mission_Command reduce_speed = {};
		// reduce_speed.id = MAV_CMD_DO_CHANGE_SPEED;
		// reduce_speed.content.speed = vwp2_spd;

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
    if(hwp_status > HWP_INITIALIZED)
	num_cmd = _mission.num_commands();
}

float AP_HeadWindLanding::calc_theta_hwp(float theta_wind, AP_Mission::Mission_Command &last_mwp, AP_Mission::Mission_Command &land_wp)
{
	// If we didn't set-up a no landing area around the landing point
	// I will generate the HWP 180 degrees against the wind direction.
	if(!is_no_landing_area_set)
	{
		return theta_wind;
	}
	// The no landing area is set. Check if the wind blows towards the no landing area.
	else
	{

		// The actual no landing area is bigger than the one configured on mission planner
		// since we need to consider that also the radius of the loitering to altitude cannot cross this area

		// gcs().send_text(MAV_SEVERITY_NOTICE, "3. begin/end initial %f %f",begin_no_landing_area,end_no_landing_area);

		float extra_area = sector_dimension_from_chord(hwp_radius,loiter_radius)*180.0/M_PI;

		float begin_ext_no_landing_area = begin_no_landing_area - extra_area;
		float end_ext_no_landing_area = end_no_landing_area + extra_area;

		if(is_angle_between(begin_ext_no_landing_area,end_ext_no_landing_area,theta_wind))
		{

			// TO DO: Implement this: https://math.stackexchange.com/questions/1766285/is-there-a-formula-that-finds-middle-between-two-angles
			float theta_between = begin_ext_no_landing_area + (end_ext_no_landing_area-begin_ext_no_landing_area)/2.0; //    atan2(x2-x1,y2-y1)*180.0/M_PI;  // # atan2(y, x) or atan2(sin, cos)
			float divider = theta_between + 180.0f;

			// Is the wind vector closer to the begin or the end of the non landing zone?

			float diff_begin = fabs(difference_between_angles(theta_wind,begin_ext_no_landing_area));
			float diff_end = fabs(difference_between_angles(theta_wind,end_ext_no_landing_area));

			// gcs().send_text(MAV_SEVERITY_CRITICAL, "diff: %f, %f, %f, %f",theta_between,divider,diff_begin,diff_end);

			return diff_begin < diff_end ? begin_ext_no_landing_area : end_ext_no_landing_area;

		}
		// If the no landing area is set but we are outside, then everything is fine
		else
		{
			// gcs().send_text(MAV_SEVERITY_NOTICE, "20. Outside the no landing area");
			return theta_wind;
		}
	}
}

// https://www.codeproject.com/Articles/59789/Calculate-the-real-difference-between-two-angles-k
float AP_HeadWindLanding::difference_between_angles(float first, float second)
{
    float difference = second - first;
    if (difference < -180) difference += 360;
    if (difference > 180) difference -= 360;
    return difference;
}

// https://math.stackexchange.com/questions/1044905/simple-angle-between-two-angles-of-circle
bool AP_HeadWindLanding::is_angle_between(float start, float end, float mid)
{
    end = (end - start) < 0.0f ? end - start + 360.0f : end - start;
    mid = (mid - start) < 0.0f ? mid - start + 360.0f : mid - start;
    return (mid < end);
}

float AP_HeadWindLanding::sector_dimension_from_chord(float radius, float chord)
{
	// The HWP radius cannot be less than 50 meter: the dynamic and the expected velocity during transition
	// doesn't allow to have such short distance.
	if(radius < MIN_RADIUS_HEADINGWIND_WAYPOINT)
		radius = MIN_RADIUS_HEADINGWIND_WAYPOINT;

	return 2.0*asin(chord/(4.0*radius));
}

void AP_HeadWindLanding::restore_mission()
{
    // If I generate the virtual waypoints, I can remove them and restore the mission to its original state.
    if(hwp_enabled && hwp_status == HWP_GENERATED)
    {
		// Here I restore the original version of the mission (in case it should be reloaded)
		AP_Mission::Mission_Command wp;

		// The variable wp will contain the landinig waypoint that I need to restore
		_mission.get_next_nav_cmd(num_cmd-1, wp);

		// If the last command is the landing (safety check)
		if(wp.id==MAV_CMD_NAV_LAND)
		{
			// I remove the mission items starting from the index of the original landing waypoint
			_mission.truncate(idx_landing_wp);
			// I add the current waypoint (landing waypoint) as the last item
			_mission.add_cmd(wp);
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

