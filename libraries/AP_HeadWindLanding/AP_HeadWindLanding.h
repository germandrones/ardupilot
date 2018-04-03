/*
 * AP_HeadWindLanding.h
 *
 * Created on: Nov 1, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#ifndef _APHEADWINDLANDING_H
#define _APHEADWINDLANDING_H

#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>

#include <AP_Math/vector2.h>

// Macro for converting the latitude and longitude back to decimal representation
#define TO_DEG_FORMAT 1.0e-7f
// Macro for calculating how many meters for 1 degree of latitude, given the current latitude in degrees.
#define METERS_PER_DEG_LAT(lat) 111132.954-559.822*cos(2.0*lat*DEG_TO_RAD)+1.175*cos(4.0*lat*DEG_TO_RAD)
// Macro for calculating how many meters for 1 degree of longitude, given the current latitude in degrees.
#define METERS_PER_DEG_LNG(lat) 111132.954*cos(lat*DEG_TO_RAD);

// List of states concerning the generation of the virtual waypoints
typedef enum vwp_generation_states {
	HWP_NOT_INITIALIZED = 0,
	HWP_INITIALIZED,
	HWP_GENERATED,
	HWP_REMOVED
} hwp_status_t;

// List of the possible errors during the generation of the virtual waypoints
typedef enum vwp_error_states {
	HWP_NO_ERROR = 0,
	HWP_LANDING_WP_NOT_FOUND,
	HWP_LAST_MISSION_WP_NOT_FOUND,
	HWP_INDEX_NOT_FOUND
} hwp_error_status_t;

// typedef just to avoid the long name
typedef AP_Mission::Mission_Command MC;

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

    /// generate_hw_waypoints - generates the virtual waypoints based on the current settings and the wind direction.
    void generate_hw_waypoints(const MC& cmd);

    /// update_num_commands - updates the variable containing the number of commands in the mission. This function is called after
    /// the addition and removal of the virtual waypoints.
    void update_num_commands();

    /// is_current_cmd_vwp - returns true if the current command is a generated command (head wind waypoint)
    bool is_current_cmd_hwp(const AP_Mission::Mission_Command& cmd);

    void restore_mission();

    // Get methods
    bool		is_hwp_enabled()			{ return hwp_enabled && !hwp_mav_cmd_present; }
    AP_Float 	get_heading_wind()			{ return heading_wind; }
    AP_Float 	get_hwp_spd()				{ return hwp_spd; }

    AP_Float	get_dist_hwpl_1()			{ return dist_hwpl_1; }
    AP_Float	get_dist_hwpl_2()			{ return dist_hwpl_2; }
    AP_Float	get_dist_hwpl_3()			{ return dist_hwpl_3; }

    int16_t 	get_idx_last_mission_wp()	{ return idx_last_mission_wp; }
    int16_t 	get_idx_landing_wp()		{ return idx_landing_wp; }
    int16_t 	get_idx_hwp()				{ return idx_hwp; }

    int16_t		get_num_commands()			{ return num_cmd; }

    // Set methods
    void 	set_dist_hwpl_1(AP_Float _val)	{ dist_hwpl_1 = _val; }
    void 	set_dist_hwpl_2(AP_Float _val)	{ dist_hwpl_2 = _val; }
    void 	set_dist_hwpl_3(AP_Float _val)	{ dist_hwpl_3 = _val; }

    AP_Mission::Mission_Command get_hwp1() 	{ return hwp1; }
    AP_Mission::Mission_Command get_hwp2()	{ return hwp2; }
    AP_Mission::Mission_Command get_hwp3() 	{ return hwp3; }
    AP_Mission::Mission_Command get_hwp4() 	{ return hwp4; }

    // AP_Mission::Mission_Command get_reduce_speed() { return reduce_speed; }

    void		enable()					{ hwp_enabled = 1; }
    void		disable()					{ hwp_enabled = 0; }
    void		temporarily_disable()		{ hwp_mav_cmd_present = 1; }
    void 		temporarily_enable()        { hwp_mav_cmd_present = 0; }

    // Status variables
    hwp_status_t hwp_status;
    hwp_error_status_t hwp_error;
    
    bool is_hwp_received = false;
    void ack_echo_received(){ is_hwp_received = true; }

protected:

    AP_Int8  hwp_enabled;
    AP_Int8	 hwp_mav_cmd_present;
    AP_Int16 hwp_radius;
    AP_Float heading_wind;
    AP_Float hwp_spd;

    AP_Int16 loiter_radius;
    AP_Int16 waypoint_radius; // waypoint radius during landing sequence

    AP_Float dist_hwpl_1;
    AP_Float dist_hwpl_2;
    AP_Float dist_hwpl_3;
    AP_Float dist_hwpl_4;

    // The following variables describes the beginning and the end of the forbidden area where the HWP
    // cannot be generated. The areas is described as the sector of the circle centered in the landing point.
    // The begin_forbidden_area and end_forbidden_area variables represent the angles from the North axis
    // using the NED frame.
    bool is_no_landing_area_set;
    float begin_no_landing_area;
    float offset_no_landing_area;
    float end_no_landing_area;

private:

    // Returns true if all the conditions for generating the HeadWind waypoints are met.
    // all_conditions_satisfied() - returns true if all the conditions for generating the HeadWind waypoints are met.
    bool all_conditions_satisfied();

    // is_disable_HWP_command_present() - returns true if the mission doesn't contains the MAV_CMD_DO_DISABLE_HWP
    // for temporarily disable the mission
    bool is_disable_HWP_command_present();

    // check_forbidden_area - checks if the user set a no landing area where the HWP should not be generated
    void check_no_landing_area_defined(void);

    // sector_dimension_from_chord() - returns the size of the circular sector specifying the radius of the circle
    // and the lenght of the chord. This functions is used to see how far from the forbidden area we must stay considering
    // the space required by the UAV for performing the loiter to altitude.
    float sector_dimension_from_chord(float radius, float chord);

    // is_angle_between() - checks if an angle is between two angles
    bool is_angle_between(float start, float end, float mid);

    // difference_between_angles() - calculates the difference between two angles
    float difference_between_angles(float first, float second);

    // calc_theta_hwp() - calculates the direction of the HWP based on the forbidden zone
    float calc_theta_hwp(float theta_wind, MC &last_mwp, MC &land_wp);
    
    // some helper methods
    int getBearing(Location p1, Location p2);
    Location newPos(Location inLocation, float bearing, float distance);
    bool DoLineSegmentsIntersect(float x1, float y1, float x2, float y2, float x1s, float y1s, float x2s, float y2s);



    bool check_crossing_no_landing_zone(MC &last_mwp, MC &land_wp, MC &lta_wp, float begin_area, float end_area);
    bool IsOnSegment(double xi, double yi, double xj, double yj,double xk, double yk);
    char ComputeDirection(double xi, double yi, double xj, double yj, double xk, double yk);

    // does_segments_intersect() - checks if two segments intersect. The segments are passed specifying the Point
    bool does_segments_intersects(Vector2l &P1, Vector2l &P2, Vector2l &P3, Vector2l &P4);

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
    AP_Mission::Mission_Command hwp4;

    // AP_Mission::Mission_Command reduce_speed;

    AP_Mission&		_mission;
    AP_AHRS_NavEKF&	_ahrs;

};

#endif /* _APHEADWINDLANDING_H */
