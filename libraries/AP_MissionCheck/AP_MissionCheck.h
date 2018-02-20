/*
 * AP_MissionCheck.h
 *
 *  Created on: December 12, 2017
 *      Author: Alessandro Benini
 */

#ifndef AP_MISSIONCHECK_H_
#define AP_MISSIONCHECK_H_

#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS.h>

class MissionCheck
{
  
public:

    // Constructor
    MissionCheck(AP_Mission &mission, DataFlash_Class &dataflash, GCS& gcs);

    // Destructor
    ~MissionCheck() {}
    
    // Inspect if a mission is loaded at startup. Returns true if we have a takeoff waypoint, a landing waypoint and at least two mission waypoints.
    void inspect_stored_mission();
    
    bool is_takeoff_wp_present()    { return index_takeoff_waypoint > 0 ? true : false; }
    bool is_landing_wp_present()    { return index_landing_waypoint > 0 ? true : false; }

    uint16_t get_index_takeoff_wp()	{ return index_takeoff_waypoint; }
    uint16_t get_index_landing_wp()	{ return index_landing_waypoint; }

    uint16_t get_num_nav_wayponts() { return num_nav_wayponts; }
    
    virtual bool check()             = 0;
    virtual void init_mission()      = 0;
    virtual void notify_user()       = 0;

protected:

    bool	 takeoff_wp_present;
    bool 	 landing_wp_present;
    uint16_t num_nav_wayponts;
    
    AP_Mission&	        _mission;
    DataFlash_Class&	_dataflash;
    GCS&				_gcs;
    
    void logInfo(char* _msg);

    // Generic message container for logging
    char* msg;

private:

    int16_t index_takeoff_waypoint;
    int16_t index_landing_waypoint;
  
    /// get_index_last_nav_WP - returns the index of the last mission waypoint.
    int16_t get_index_last_nav_WP();

    // Check the landing sequence according to which type mission is used
    virtual bool is_landing_sequence_present() = 0;

};

#endif
