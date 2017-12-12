/*
 * AP_MissionCheck_VWP.h
 *
 *  Created on: December 12, 2017
 *      Author: Alessandro Benini
 */

#ifndef AP_MISSIONCHECK_VWP_H_
#define AP_MISSIONCHECK_VWP_H_

#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>
#include <AP_HeadWindLanding/AP_HeadWindLanding.h>

#include "AP_MissionCheck.h"

class MissionCheck_HWP : public MissionCheck
{
  
public:
  
	MissionCheck_HWP(AP_Mission &mission, DataFlash_Class &dataflash, AP_HeadWindLanding &headwind_wp, GCS& gcs);
    
    void init_mission();
    bool check();
    void notify_user();
    char* get_mission_type();
     
private:
    
    AP_HeadWindLanding&	_headwind_wp;
    bool                hwp_feature_usable;
  
};

#endif
