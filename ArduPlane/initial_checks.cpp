/*
 * initial_checks.cpp
 *
 * Created on: December 15, 2017
 *     Author: Alessandro Benini
 *    Company: Germandrones GmbH
 */

#include "Plane.h"

// This function performs the initial checks when the ArmingRequired parameter is set to YES_GDPILOT
// Only the GDPilot is allowed to arm the UAV. This is very critical.
// !!!WE MUST ABSOLUTELY SURE THAT THE UAV IS NOT FLYING!!!
void Plane::initial_checks()
{

	// We have a new message from GDPilot that has not been processed
	if(!plane.gd_status.msg_processed)
	{
		// Special cases for DISARM cmd message and UAV ARMED notification message
		if(plane.gd_status.err_num == 127)
		{
			gcs().send_text(MAV_SEVERITY_CRITICAL, "UAV DISARMED");

			AP_Notify::flags.gd_sd_not_logging = false;
			AP_Notify::flags.gd_fmode_wrong = false;
			AP_Notify::flags.gd_disarmed = false;
			AP_Notify::flags.px_not_ready = false;

			arming.disarm();
			AP_Notify::flags.armed = false;

			// Since the GDPilot takes care of the landing phase, after issuing the landing command,
			// PixHawk would just need to re-set its internal state for a new mission.
			mission.start();
		}
		// This is sent by GDPilot after the PixHawk sends the acknowledge
		// to let the user know that everything is fine (the main led becomes solid blue)
		else if(plane.gd_status.err_num == 100)
		{
			gcs().send_text(MAV_SEVERITY_CRITICAL, "UAV ARMED");
			AP_Notify::flags.armed = true;
		}
		// For every other message we need to be in disarmed state and not flying
		else
		{
			if(!arming.is_armed() && plane.is_flying() == false)
			{
				switch(plane.gd_status.err_num)
				{
					case 0:
					{
						// GD Pilot is ready and I force all the gd flags to false
						AP_Notify::flags.gd_sd_not_logging = false;
						AP_Notify::flags.gd_fmode_wrong = false;
						AP_Notify::flags.gd_disarmed = false;


						// When everything is fine on the GDPilot, I start checking the PixHawk.
						// For the moment we only check that the mission is set properly
						// (The PixHawk already have internal check procedures for the sensors)
						bool mission_checked = check_mission();

						if(mission_checked)
						{
							// I allow the UAV to arm
							bool success = arming.arm(AP_Arming::GDPILOT);

							if(success)
							{
								// Here we send the acknowledge to the GDPIlot board.
								ack_to_gdpilot_must_be_sent = true;
							}
							else
							{
								gcs().send_text(MAV_SEVERITY_CRITICAL, "ERROR: ARMING FAILED");
							}

						}
						else
						{
							AP_Notify::flags.px_not_ready = true;
							gcs().send_text(MAV_SEVERITY_CRITICAL, "ERROR: MISSION CHECK FAILED");
						}

						break;
					}

					// For all the errors, I simply shows the message and keep the UAV disarmed
					case 1:
					{
						gcs().send_text(MAV_SEVERITY_CRITICAL, "ERROR: %s",plane.gd_status.err_msg);
						// Something is not ok on the GDPilot side. I keep the UAV disarmed.
						AP_Notify::flags.gd_fmode_wrong = true;

						break;
					}

					case 2:
					{
						gcs().send_text(MAV_SEVERITY_CRITICAL, "ERROR: %s",plane.gd_status.err_msg);
						// Something is not ok on the GDPilot side. I keep the UAV disarmed.
						AP_Notify::flags.gd_sd_not_logging = true;

						break;
					}

					default:
						// Initial value is -1. We don't do anything
						break;
				}

			}
		}

		// Set the status of message to processed
		plane.gd_status.msg_processed = true;
	}
}

// Entry point for check mission procedure
bool Plane::check_mission()
{
    //  gcs().send_text(MAV_SEVERITY_INFO, "ARMING METHOD, REQUIRED: %d, %d",arming.get_arming_method(),arming.arming_required());

	if(headwind_wp.is_hwp_enabled())
		mission_checker = new MissionCheck_HWP{mission,DataFlash,headwind_wp,_gcs};
	else
		mission_checker = new MissionCheck_STD{mission,DataFlash,_gcs};

	bool successfull = mission_checker->check();

	if(successfull)
	{
		mission_checker->init_mission();
		return true;
	}
	else
	{
		// The stored mission is not ok. We build a default mission and then we notify the user
		// The default mission contains
		return false;
	}

}
