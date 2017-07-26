/*
 * States.cpp
 *
 *  Created on: March 17, 2017
 *      Author: Kent Askenmalm
 */

#include "am_driver_safe/automower_safe_states.h"

namespace Husqvarna { 

using namespace decision_making;

static AutomowerSafePtr m_driver;

void ConnectDriverAndStates(AutomowerSafePtr driver)
{
	m_driver = driver;
}



FSM(ManualStates)
{
	FSM_STATES
	{
		Stopped,
		InOperation,
		Paused
	}
	FSM_START(Stopped);
	FSM_BGN
	{
		FSM_STATE(Stopped)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
			}
		}
		FSM_STATE(InOperation)
		{
			
			// ON ENTRY
			
			m_driver->pauseMower(); // We should never stay in "InOperation" when in Manual state

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
			}
		}

		FSM_STATE(Paused)
		{
			// ON ENTRY
			m_driver->stopWheels();
			m_driver->m_regulatingActive = true;
			
			FSM_ON_STATE_EXIT_BGN
			{
				m_driver->m_regulatingActive = false;
				m_driver->stopWheels();
			}
			FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
			}
		}
	}
    FSM_END
}

FSM(RandomStates)
{
	FSM_STATES
	{
		Stopped,
		InOperation,
		Paused
	}
	FSM_START(Stopped);
	FSM_BGN
	{
		FSM_STATE(Stopped)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
			}
		}
		FSM_STATE(InOperation)
		{
			// ON ENTRY 
			m_driver->cutDiscHandling();   // Make sure to turn off if necessary, since mower automatically activates cutting

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
				FSM_ON_EVENT("/AM_IN_OPERATION",m_driver->cutDiscHandling())
			}
		}

		FSM_STATE(Paused)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_PAUSED",m_driver->startMower())  // Move on on to In Operation as soon as possible
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
			}
		}
	}
    FSM_END
}

FSM(ParkStates)
{
	FSM_STATES
	{
		Stopped,
		InOperation,
		Paused
	}
	FSM_START(Stopped);
	FSM_BGN
	{
		FSM_STATE(Stopped)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
			}
		}
		FSM_STATE(InOperation)
		{
			// ON ENTRY 
			m_driver->cutDiscHandling();   // Make sure to turn off if necessary, since mower automatically activates cutting
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_PAUSED",FSM_NEXT(Paused))
			}
		}

		FSM_STATE(Paused)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/AM_PAUSED",m_driver->startMower())  // Move on on to In Operation as soon as possible
				FSM_ON_EVENT("/AM_STOPPED",FSM_NEXT(Stopped))
				FSM_ON_EVENT("/AM_IN_OPERATION",FSM_NEXT(InOperation))
			}
		}
	}
    FSM_END
}



FSM(AutoMowerSafeStates)
{

	FSM_STATES
	{
		Idle,
		Init,
		Manual,
		Random,
		Park,
	}
	FSM_START(Idle);
	FSM_BGN
	{

		FSM_STATE(Idle)
		{
			// ON ENTRY
			m_driver->newControlMainState(AM_STATE_IDLE);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/MANUAL",FSM_NEXT(Manual))
				FSM_ON_EVENT("/RANDOM",FSM_NEXT(Random))
				FSM_ON_EVENT("/PARKING",FSM_NEXT(Park))
			}
		}

		FSM_STATE(Init)
		{
			// ON ENTRY
			m_driver->newControlMainState(AM_STATE_INIT);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/MANUAL",FSM_NEXT(Idle))
				FSM_ON_EVENT("/RANDOM",FSM_NEXT(Idle))
			}
		}
		FSM_STATE(Manual)
		{

			// ON ENTRY
			m_driver->newControlMainState(AM_STATE_MANUAL);
			m_driver->pauseMower();
			m_driver->cutDiscOff();
			m_driver->setAutoMode();

			FSM_ON_STATE_EXIT_BGN
			{
				m_driver->stopWheels();
				m_driver->cutDiscOff();
			}
			FSM_ON_STATE_EXIT_END
			

			FSM_CALL_FSM(ManualStates);
			
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOOPDETECTION_CHANGED",m_driver->loopDetectionHandling())
				FSM_ON_EVENT("/CUTDISC_CHANGED",m_driver->cutDiscHandling())
				FSM_ON_EVENT("/CUTTINGHEIGHT_CHANGED",m_driver->cuttingHeightHandling())
				FSM_ON_EVENT("/RANDOM",FSM_NEXT(Random))
				FSM_ON_EVENT("/PARKING",FSM_NEXT(Park))
			}
		}
		FSM_STATE(Random)
		{

			// ON ENTRY
			m_driver->newControlMainState(AM_STATE_RANDOM);
			m_driver->setAutoMode();
			m_driver->startMower();
			m_driver->cutDiscHandling();


 			FSM_ON_STATE_EXIT_BGN
			{
				m_driver->pauseMower();
				m_driver->cutDiscOff();
			}
			FSM_ON_STATE_EXIT_END

			FSM_CALL_FSM(RandomStates);

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOOPDETECTION_CHANGED",m_driver->loopDetectionHandling())
				FSM_ON_EVENT("/CUTDISC_CHANGED",m_driver->cutDiscHandling())
				FSM_ON_EVENT("/CUTTINGHEIGHT_CHANGED",m_driver->cuttingHeightHandling())
				FSM_ON_EVENT("/MANUAL",FSM_NEXT(Manual))
				FSM_ON_EVENT("/PARKING",FSM_NEXT(Park))
			}
		}

		FSM_STATE(Park)
		{

			// ON ENTRY
			m_driver->newControlMainState(AM_STATE_PARK);
			m_driver->setParkMode();
			m_driver->startMower();
			m_driver->cutDiscHandling();
			
 			FSM_ON_STATE_EXIT_BGN
			{
				m_driver->pauseMower();
				m_driver->cutDiscOff();
			}
			FSM_ON_STATE_EXIT_END

			FSM_CALL_FSM(ParkStates);

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOOPDETECTION_CHANGED",m_driver->loopDetectionHandling())
				FSM_ON_EVENT("/CUTDISC_CHANGED",m_driver->cutDiscHandling())
				FSM_ON_EVENT("/CUTTINGHEIGHT_CHANGED",m_driver->cuttingHeightHandling())
				FSM_ON_EVENT("/MANUAL",FSM_NEXT(Manual))
				FSM_ON_EVENT("/RANDOM",FSM_NEXT(Random))
			}
		}

	}
FSM_END
}



}

