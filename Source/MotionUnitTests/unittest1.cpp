#include "stdafx.h"
#include "CppUnitTest.h"
#include "dwt_stm32_delay.h"

#include "..\RotaryController2\Src\motion.h"

void __disable_irq() {}
void __enable_irq() {}

static bool timerInterruptEnabled = false;

void enableTimerInterupt(bool state)
{
	timerInterruptEnabled = state;
}


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

TEST_CLASS(MotionTests)
{
public:
	uint32_t measuredMaxVelocity;

	// Common motion routine.
	void DoMotion(Motion &motion, uint32_t maxMotorPosition, void (* extraTests)(Motion &motion))
	{
		measuredMaxVelocity = 0;
		uint32_t lastPos = motion.MotorPosition();
		while (timerInterruptEnabled)
		{
			motion.TimerInterruptHandler();
			motion.TimerInterruptHandler();
			if (motion.nVelocity > measuredMaxVelocity)
				measuredMaxVelocity = motion.nVelocity;
			if (motion.eState == Motion::eStopped)
				break;
			if (!timerInterruptEnabled)
				break;
			Assert::IsTrue(motion.MotorPosition() <= maxMotorPosition);
			Assert::IsTrue(motion.MotorPosition() != lastPos);

			extraTests(motion);

			lastPos = motion.MotorPosition();
		}
	}

	TEST_METHOD(TestStartStopToMaxV)
	{
		uint32_t backlash=0;
		uint32_t stepsPerRevolution=2000;
		uint32_t gearRatio=90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2000;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		DoMotion(motion, stepsToMove, [](Motion &motion) {
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove, L"Final position wrong");
		uint32_t actualMaxV = (motion.nMaxVelocity / motion.nAcceleration) * motion.nAcceleration;
		Assert::IsTrue(actualMaxV == measuredMaxVelocity, L"Didn't hit MaxV");

	}

};
