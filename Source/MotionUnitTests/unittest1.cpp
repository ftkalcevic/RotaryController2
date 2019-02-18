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
	uint32_t accelerationPosition;
	uint32_t runPosition;
	uint32_t decelerationPosition;

	// Common motion routine.
	//void DoMotion(Motion &motion, uint32_t maxMotorPosition, void(*extraTests)(Motion &motion))
	template<typename F>
	void DoMotion(Motion &motion, uint32_t maxMotorPosition, F extraTests )
	{
		measuredMaxVelocity = 0;
		accelerationPosition = 0;
		runPosition = 0;
		decelerationPosition = 0;

		int32_t lastPos = motion.MotorPosition();
		Motion::ESTATE lastState = motion.eState;
		while (timerInterruptEnabled)
		{
			motion.TimerInterruptHandler();
			motion.TimerInterruptHandler();

			if (lastState != motion.eState)
			{
				switch (lastState)
				{
					case Motion::eAccelerating: accelerationPosition = motion.MotorPosition(); break;
					case Motion::eRunning: runPosition = motion.MotorPosition(); break;
					case Motion::eDecelerating: decelerationPosition = motion.MotorPosition(); break;
				};
			}
			lastState = motion.eState;

			if (motion.nVelocity > measuredMaxVelocity)
				measuredMaxVelocity = motion.nVelocity;
			if (motion.eState == Motion::eStopped)
				break;
			if (!timerInterruptEnabled)
				break;
			//Assert::IsTrue(motion.MotorPosition() <= (int32_t)maxMotorPosition*2, L"Well exceeded target position");
			Assert::IsTrue(motion.MotorPosition() != lastPos, L"No movement between interrupts");

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

		uint32_t stepsToMove = 2200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		DoMotion(motion, stepsToMove, [](Motion &motion) {
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove, L"Final position wrong");
		uint32_t actualMaxV = (motion.nMaxVelocity / motion.nAcceleration) * motion.nAcceleration;
		Assert::IsTrue(actualMaxV == measuredMaxVelocity, L"Didn't hit MaxV");

		uint32_t accelerationDistance = accelerationPosition;
		uint32_t runDistance = runPosition - accelerationPosition;
		uint32_t decelerationDistance = decelerationPosition - runPosition;

	}

	TEST_METHOD(TestStartStopToLessThanMaxV)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		DoMotion(motion, stepsToMove, [](Motion &motion) {
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove, L"Final position wrong");
		
		uint32_t accelerationDistance = accelerationPosition;
		uint32_t runDistance = runPosition - accelerationPosition;
		uint32_t decelerationDistance = decelerationPosition - runPosition;
	}

	TEST_METHOD(Test2ndMoveAtRunState)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove+ additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 1100)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove+additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtRunStateShort)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 1;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 1100)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}


	TEST_METHOD(Test2ndMoveAtAccState)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 850)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtAccStateShort)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 1;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 850)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtDecStateToVmax)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 2200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 2000)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtDecStateShort)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 1;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 2000)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtDecStateShortIncludesRun)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 25;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 2000)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveAtDecStateShortIncludesAcc)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 2200;
		uint32_t additionalSteps = 200;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 2000)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

	TEST_METHOD(Test2ndMoveShortThenLong)
	{
		uint32_t backlash = 0;
		uint32_t stepsPerRevolution = 2000;
		uint32_t gearRatio = 90;
		uint32_t maxVelocity = 2000;
		uint32_t maxAcceleration = 2000;

		Motion motion(NULL);
		motion.SetMotorConfig(backlash, stepsPerRevolution, gearRatio, maxVelocity, maxAcceleration);

		uint32_t stepsToMove = 10;
		uint32_t additionalSteps = 100;
		motion.MoveSteps(stepsToMove);
		motion.UpdateMotorTravel();

		bool bExtraStep = false;
		DoMotion(motion, stepsToMove + additionalSteps, [additionalSteps](Motion &motion) {
			if (motion.MotorPosition() == 1)
			{
				motion.MoveSteps(additionalSteps);
				motion.UpdateMotorTravel();
			}
		});

		Assert::IsTrue(motion.MotorPosition() == stepsToMove + additionalSteps, L"Final position wrong");
	}

};
