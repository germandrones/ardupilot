/*
 * AP_HeadWindLanding.h
 *
 *  Created on: Nov 1, 2017
 *      Author: Alessandro Benini
 */

#ifndef _APHEADWINDLANDING_H
#define _APHEADWINDLANDING_H

#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>

// Macro for converting the latitude and longitude back to decimal representation
#define TO_DEG_FORMAT 1.0e-7f
// Macro for calculating how many meters for 1 degree of latitude, given the current latitude in degrees.
#define METERS_PER_DEG_LAT(lat) 111132.954-559.822*cos(2.0*lat*DEG_TO_RAD)+1.175*cos(4.0*lat*DEG_TO_RAD)
// Macro for calculating how many meters for 1 degree of longitude, given the current latitude in degrees.
#define METERS_PER_DEG_LNG(lat) 111132.954*cos(lat*DEG_TO_RAD);

// This the maximum variation of altitude between two consecutive virtual waypoints (value expressed in meters)
#define MAX_STEP 20.0f
#define MAX_STEP_CM MAX_STEP*100.0f

#define MINIMUM_SPEED_DURING_VWP 10.0f

// List of states concerning the generation of the virtual waypoints
typedef enum vwp_generation_states {
	HWP_NOT_GENERATED = 0,
	HWP_GENERATED,
	HWP_REMOVED
} hwp_status_t;

// List of the possible error during the generation of the virtual waypoints
typedef enum vwp_error_states {
	HWP_NO_ERROR = 0,
	HWP_LANDING_WP_NOT_FOUND,
	HWP_LAST_MISSION_WP_NOT_FOUND,
	HWP_INDEX_NOT_FOUND
} hwp_error_status_t;

class AP_HeadWindLanding
{
public:

    // Constructor
	AP_HeadWindLanding(AP_Mission &mission, AP_AHRS_NavEKF &ahrs);

    // Destructor
    ~AP_HeadWindLanding() {}

    // Structure for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // Initialize the indexes for calculating the virtual waypoints
    void init_HWP(void);

    /// calc_index_landing_waypoint - returns the index of the landing waypoint. The landing waypoint
    /// should always be the last item. But this function is implemented in order to contemplate
    /// the case where further actions are programmed after the landing and/or to make sure that
    /// a landing waypoint is set.
    void calc_index_landing_waypoint(void);

    /// calc_index_last_mission_waypoint - returns the index of the last mission waypoint.
    void calc_index_last_mission_waypoint(void);

    /// calc_index_hw_waypoints - returns the index of the nav cmd item after which the vwp will be generated (referred to as CMD_VWP)
    /// returns 0 if the mission is shorter than the distance between landing waypoint and CMD_VWP
    void calc_index_hw_waypoints();

    bool is_change_speed_cmd_issued(const AP_Mission::Mission_Command& cmd);

    /// generate_virtual_waypoints - generates the virtual waypoints based on the current settings and the wind direction.
    void generate_hw_waypoints(const AP_Mission::Mission_Command& cmd);

    /// update_num_commands - updates the variable containing the number of commands in the mission. This function is called after
    /// the addition and removal of the virtual waypoints.
    void update_num_commands();

    /// is_current_cmd_vwp - returns true if the current command is a generated command (head wind waypoint)
    bool is_current_cmd_hwp(const AP_Mission::Mission_Command& cmd);

    void restore_mission();

    // Get methods
    bool		is_hwp_enabled()		{ return hwp_enabled; }
    AP_Float 	get_heading_wind()		{ return heading_wind; }
    AP_Float 	get_hwp_spd()			{ return hwp_spd; }

    AP_Float	get_dist_hwpl_1()		{ return dist_hwpl_1; }
    AP_Float	get_dist_hwp1_2()		{ return dist_hwp1_2; }
    AP_Float	get_dist_hwp2_3()		{ return dist_hwp2_3; }

    int16_t 	get_idx_last_mission_wp()	{ return idx_last_mission_wp; }
    int16_t 	get_idx_landing_wp()		{ return idx_landing_wp; }
    int16_t 	get_idx_hwp()			{ return idx_hwp; }

    int16_t	get_num_commands()		{ return num_cmd; }

    // Set methods
    void 	set_dist_hwpl_1(AP_Float _val)	{ dist_hwpl_1 = _val; }
    void 	set_dist_hwp1_2(AP_Float _val)	{ dist_hwp1_2 = _val; }
    void 	set_dist_hwp2_3(AP_Float _val)	{ dist_hwp2_3 = _val; }

    AP_Mission::Mission_Command get_hwp1() { return hwp1; }
    AP_Mission::Mission_Command get_hwp2() { return hwp2; }
    AP_Mission::Mission_Command get_hwp3() { return hwp3; }
    // AP_Mission::Mission_Command get_reduce_speed() { return reduce_speed; }

    // Status variables
    hwp_status_t hwp_status;
    hwp_error_status_t hwp_error;

protected:

    AP_Int8  hwp_enabled;
    AP_Float hwp_radius;
    AP_Float heading_wind;
    AP_Float hwp_spd;

    AP_Float dist_hwpl_1;
    AP_Float dist_hwp1_2;
    AP_Float dist_hwp2_3;

private:

    typedef struct {
      // The following variable set the point in the mission where the virtual waypoints are generated.
      // The point in the mission is calculated as the number of nav_commands from the landing waypoint.
      int16_t dist_lwp_idx;
      // Number of virtual waypoints to be generated. For the moment it will be fixed at 3.
      int16_t num_hwp;
    } hwp_config_t;

    hwp_config_t hwp_cfg;

    // Index of the item after which calculate the virtual waypoints
    int16_t idx_hwp;

    // Index of the last mission waypoint
    int16_t idx_last_mission_wp;

    // Index of the original landing waypoint
    int16_t idx_landing_wp;

    // Number of commands contained in the mission
    int16_t num_cmd;

    AP_Mission::Mission_Command hwp1;
    AP_Mission::Mission_Command hwp2;
    AP_Mission::Mission_Command hwp3;
    // AP_Mission::Mission_Command reduce_speed;

    AP_Mission&		_mission;
    AP_AHRS_NavEKF&	_ahrs;

};

#endif /* _APHEADWINDLANDING_H */
