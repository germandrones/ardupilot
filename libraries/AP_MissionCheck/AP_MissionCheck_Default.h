/*
 * AP_MissionCheck_Default.h
 *
 *  Created on: December 12, 2017
 *      Author: Alessandro Benini
 */

#ifndef AP_MISSIONCHECK_DEFAULT_H_
#define AP_MISSIONCHECK_DEFAULT_H_

#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#include "AP_MissionCheck.h"

class MissionCheck_Default : public MissionCheck
{
  
public:
  
    MissionCheck_Default(AP_Mission &mission, DataFlash_Class &dataflash, GCS& gcs);
    
    void init_mission();
    bool check();
    void notify_user();  
    char* get_mission_type();
     
private:
    
    bool default_mission_usable;
  
};

#endif
