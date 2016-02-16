/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "RobotBase.h"
#include "RobotState.h"
#include "Utility.h"

#include <cstring>

RobotBase* RobotBase::m_instance = nullptr;

void RobotBase::setInstance(RobotBase* robot)
{
	wpi_assert(m_instance == nullptr);
	m_instance = robot;
}

RobotBase &RobotBase::getInstance()
{
	return *m_instance;
}

/**
 * Constructor for a generic robot program.
 * User code should be placed in the constuctor that runs before the Autonomous or Operator
 * Control period starts. The constructor will run to completion before Autonomous is entered.
 *
 * This must be used to ensure that the communications code starts. In the future it would be
 * nice to put this code into it's own task that loads on boot so ensure that it runs.
 */
RobotBase::RobotBase() : m_ds(DriverStation::GetInstance())
{
	RobotState::SetImplementation(DriverStation::GetInstance());
	transport::SubscriberPtr time_pub = MainNode::Subscribe("time", &wpilib::internal::time_callback);
}

/**
 * Determine if the Robot is currently enabled.
 * @return True if the Robot is currently enabled by the field controls.
 */
bool RobotBase::IsEnabled() const
{
	return m_ds.IsEnabled();
}

/**
 * Determine if the Robot is currently disabled.
 * @return True if the Robot is currently disabled by the field controls.
 */
bool RobotBase::IsDisabled() const
{
	return m_ds.IsDisabled();
}

/**
 * Determine if the robot is currently in Autnomous mode.
 * @return True if the robot is currently operating Autonomously as determined by the field controls.
 */
bool RobotBase::IsAutonomous() const
{
	return m_ds.IsAutonomous();
}

/**
 * Determine if the robot is currently in Operator Control mode.
 * @return True if the robot is currently operating in Tele-Op mode as determined by the field controls.
 */
bool RobotBase::IsOperatorControl() const
{
	return m_ds.IsOperatorControl();
}

/**
 * Determine if the robot is currently in Test mode.
 * @return True if the robot is currently running tests as determined by the field controls.
 */
bool RobotBase::IsTest() const
{
    return m_ds.IsTest();
}

/**
 * This class exists for the sole purpose of getting its destructor called when the module unloads.
 * Before the module is done unloading, we need to delete the RobotBase derived singleton.  This should delete
 * the other remaining singletons that were registered.  This should also stop all tasks that are using
 * the Task class.
 */
class RobotDeleter
{
public:
	~RobotDeleter()
	{
		delete &RobotBase::getInstance();
	}
};
static RobotDeleter g_robotDeleter;
