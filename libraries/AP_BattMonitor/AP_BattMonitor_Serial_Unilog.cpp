#include "AP_BattMonitor_Serial_Unilog.h"
#include <AP_HAL/AP_HAL.h>

// extern const AP_HAL::HAL& hal;


AP_BattMonitor_Serial_UniLog::AP_BattMonitor_Serial_UniLog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state, const AP_SerialManager* serial_manager) :
	AP_BattMonitor_Backend(mon, mon_state)
{
	if (serial_manager != NULL)
		_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_UniLog2, 0);
	else
		_port = NULL;
}

void AP_BattMonitor_Serial_UniLog::init()
{
	if (_port != NULL){
		unisens_state = UNISENS_INITIALIZE_CONNECTION;
		//_state.last_time_micros = hal.scheduler->micros();
		_state.last_time_micros = AP_HAL::micros();
		_state.healthy = false;
	}
	else{
		_state.healthy = false;
	}
}

void AP_BattMonitor_Serial_UniLog::read()
{
	if (_port == NULL)
	{
		_state.healthy = false;
		return;
	}

	switch(unisens_state)
	{
		case UNISENS_INITIALIZE_CONNECTION:
			initializeComm();
			unisens_state = UNISENS_WAITING_FOR_INITIALIZATION;
			last_time_initialized = AP_HAL::micros();
			break;
		case UNISENS_WAITING_FOR_INITIALIZATION:
			if (waitForInitialization() == 1)
			{
				requestLiveBatteryStatus();
				unisens_state = UNISENS_WAITING_FOR_ANSWER;
			}

			break;
		case UNISENS_WAITING_FOR_ANSWER:
			if (recvLiveBatteryStatus() > 0)
			{
				unisens_state = UNISENS_WAITING_FOR_CHECKSUM;
			}
			break;
		case UNISENS_WAITING_FOR_CHECKSUM:
			if (recvLiveBatteryChecksum() > 0)
			{
				if(checkDataOk())
				{
					uniSensExtractValues();
				}
				_port->flush();
				// no matter chechsum correct or not, do re-request
				requestLiveBatteryStatus();
				unisens_state = UNISENS_WAITING_FOR_ANSWER;
			}
			break;
		default:
			break;

	}


	uint32_t tnow = AP_HAL::micros();
	if (uint32_t(tnow - _state.last_time_micros) > AP_BATTMONITOR_SERIAL_UNILOG_TIMEOUT_MICROS &&
			uint32_t(tnow - last_time_initialized) > AP_BATTMONITOR_SERIAL_UNILOG_TIMEOUT_MICROS) {
		_state.healthy = false;
		unisens_state = UNISENS_INITIALIZE_CONNECTION;
	}

}

void AP_BattMonitor_Serial_UniLog::initializeComm() const
{
	_port->println("g"); // Todo: Check this. Before it was port->println_P("g");
}

int8_t AP_BattMonitor_Serial_UniLog::waitForInitialization() const
{
	if (_port->available())
	{
		uint8_t c = _port->read();
		if (c == '2')
			return 1;
		else
			return -1;
	}
	return 0; //nothing yet on the port
}

void AP_BattMonitor_Serial_UniLog::requestLiveBatteryStatus() const
{
	// _port->println_P("v");
	_port->println("v");
}

int8_t AP_BattMonitor_Serial_UniLog::recvLiveBatteryStatus()
{
	uint32_t tnow = AP_HAL::micros();
	while(_port->available())
	{
		uint8_t c = _port->read();
		if (c=='$') //started
		{
			m_uiUniSensSeqNo = 0;
		}
		else if(c=='*') //finished
		{
			m_uiUniSensChecksumNo = 0;
			return m_uiUniSensSeqNo;

		}else if (c == 10 || c== 13) { //on LF or CR, something wrong
			return -1;

		}else{ //real data
			if (m_uiUniSensSeqNo < UNISENS_MAX_MESSAGE_LEN)
			{
				m_auiUniSensSequence[m_uiUniSensSeqNo++] = c;
			}
			else {
				return -1; //exceeding buffer
			}
		}
		if ((AP_HAL::micros() - tnow) > 400) //prevent it to run for too long
			return 0;
	}
	return 0; //nothing yet
}

uint8_t AP_BattMonitor_Serial_UniLog::recvLiveBatteryChecksum()
{
	while(_port->available())
	{
		uint8_t c = _port->read();
		m_auiUniSensChecksum[m_uiUniSensChecksumNo++] = c;

		if (m_uiUniSensChecksumNo == 2)
			return m_uiUniSensChecksumNo;
	}
	return 0;
}

int8_t AP_BattMonitor_Serial_UniLog::checkDataOk()
{
	int8_t iXor = 0;
	//calculate checksum by xor'ing over from $ to *
	for (int8_t i = 0; i < m_uiUniSensSeqNo; i++)
	{
		iXor = iXor ^ m_auiUniSensSequence[i];
	}

	int16_t checksum =	uniSensDehex(m_auiUniSensChecksum[0]) * 16 +
					uniSensDehex(m_auiUniSensChecksum[1]);

	return checksum == iXor;
}


int16_t AP_BattMonitor_Serial_UniLog::uniSensDehex(uint8_t c)
{
	if (int16_t(c) >= 65)
	{
		return int16_t(c) - 55;
	}
	else
	{
		return int16_t(c) - 48;
	}
}



void AP_BattMonitor_Serial_UniLog::uniSensExtractValues()
{
	uint8_t cVoltage[7];
	uint8_t cCurrent[7];
	uint8_t cUsed[7];

	for (int i = 0; i < 7; i++)
	{
		cVoltage[i] = 0;
		cCurrent[i] = 0;
		cUsed[i] = 0;
	}

	//this function goes through the whole sentence and extracts the state of the battery
	int iValueNo = 0;
	int iStart = 0;

	/*
	for (int i = 0; i < m_uiUniSensSeqNo; i++)
		hal.uartA->printf("%c", (char)m_auiUniSensSequence[i]);
	hal.uartA->println();
	*/

	for (int i = 0; i < m_uiUniSensSeqNo; i++)
	{
		if (m_auiUniSensSequence[i] == ',')
		{
			iValueNo++;
			iStart = 0;
		}
		else
		{
			if (iValueNo == 4)
			{
				cVoltage[iStart++] = m_auiUniSensSequence[i];
			}
			else if (iValueNo == 5)
			{
				cCurrent[iStart++] = m_auiUniSensSequence[i];
			}
			else if (iValueNo == 11)
			{
				cUsed[iStart++] = m_auiUniSensSequence[i];
			}
		}
	}

	//convert the arrays to numbers
	_state.voltage = uniSensConvertFloat((const char *)cVoltage);
	_state.current_amps = uniSensConvertFloat((const char *)cCurrent);
	_state.current_total_mah = uniSensConvertFloat((const char *)cUsed);
	_state.healthy = true;
	_state.last_time_micros = AP_HAL::micros();
	//hal.uartA->printf("vol %f cur %f mah %f\n", _state.voltage, _state.current_amps, _state.current_total_mah);

}

float AP_BattMonitor_Serial_UniLog::uniSensConvertFloat(const char* s)
{
	long rl = 0;
	float rr = 0.0;
	float rb = 0.1;
	bool dec = false;
	int i = 0;

	if ((s[i] == '-') || (s[i] == '+'))
		i++;

	while (s[i] != 0)
	{
		if (s[i] == '.')
		{
			dec = true;
		}
		else
		{
			if (!dec)
			{
				rl = (10 * rl) + (s[i] - 48);
			}
			else
			{
				rr += rb * (float)(s[i] - 48);
				rb /= 10.0;
			}
		}
		i++;
	}

	rr += (float)rl;
	if (s[0] == '-')
	{
		rr = 0.0 - rr;
	}

	return rr;
}

// Todo: This method must be implemented, otherwise this class will remain abstract
bool AP_BattMonitor_Serial_UniLog::has_current() const
{
	return true;
}
