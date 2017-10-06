#ifndef AP_BATTMONITOR_SERIAL_UNILOG_H
#define AP_BATTMONITOR_SERIAL_UNILOG_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>

#define UNISENS_MAX_MESSAGE_LEN 140
#define UNISENS_CHECKSUM_LEN 2
#define AP_BATTMONITOR_SERIAL_UNILOG_TIMEOUT_MICROS 4000000    // sensor becomes unhealthy if no successful readings for 2 seconds

enum UNISENS_STATE {
	UNISENS_INITIALIZE_CONNECTION, //!< While we wait for the unisens to be ready
	UNISENS_WAITING_FOR_INITIALIZATION, //!< If we have sent a initialzition req to the UniSens and are waiting for a echo of '2'
	UNISENS_WAITING_FOR_ANSWER, //!< If we have sent a request to the UniSens and are waiting for a complete answer
	UNISENS_WAITING_FOR_CHECKSUM
};

class AP_BattMonitor_Serial_UniLog : public AP_BattMonitor_Backend
{
public:
 /// Constructor
    AP_BattMonitor_Serial_UniLog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state, const AP_SerialManager* serial_manager);

    // virtual destructor to reduce compiler warnings
    virtual ~AP_BattMonitor_Serial_UniLog() {}

    void init();
    void read();

    bool has_current() const;

private:

    void initializeComm() const;
    int8_t waitForInitialization() const;
    void requestLiveBatteryStatus() const;
    int8_t recvLiveBatteryStatus();
    uint8_t recvLiveBatteryChecksum();
    int8_t checkDataOk();
    int16_t uniSensDehex(uint8_t c);
    void uniSensExtractValues();
    float uniSensConvertFloat(const char* s);
    uint32_t last_time_initialized;

    AP_HAL::UARTDriver *_port;

    //! An array to hold the current sentence.
    uint8_t m_auiUniSensSequence[UNISENS_MAX_MESSAGE_LEN];
    uint8_t m_auiUniSensChecksum[UNISENS_CHECKSUM_LEN];
    UNISENS_STATE unisens_state;

    //! The number of the char in the current sequence.
    uint8_t m_uiUniSensSeqNo = 0;
    //! The number of checksum bytes we received.
    uint8_t m_uiUniSensChecksumNo = 0;
};
#endif /* AP_BATTMONITOR_SERIAL_UNILOG_H */
