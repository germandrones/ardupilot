/*
 * AP_MissionCheck_STD.h
 *
 * Created on: December 12, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#ifndef AP_MISSIONCHECK_STD_H_
#define AP_MISSIONCHECK_STD_H_

#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#include "AP_MissionCheck.h"

class MissionCheck_STD : public MissionCheck
{
  
public:
  
    MissionCheck_STD(AP_Mission &mission, DataFlash_Class &dataflash, GCS& gcs); // to do after upload
    
    void init_mission();
    bool check(Location currentPosition);
    bool check();
    void notify_user();
     
private:
    
    bool std_mission_usable;
    bool is_landing_sequence_present();
  
};

#endif
