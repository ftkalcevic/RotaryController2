#pragma  once

#include "main.h"
#include "dwt_stm32_delay.h"


class Motion
{
public:
	enum ESTATE
	{
		eAccelerating,
		eRunning,
		eDecelerating,
		eStopped
	};
	ESTATE eState;

private:
	uint32_t nVelocity;
	uint32_t nAccTime;
	uint32_t nPathAcceleration;
	uint32_t nRunTimeFraction;
	uint32_t nRunTime;
	uint32_t nDecTime;
	bool bContinuousMode;
	bool bSynchronousMotion;
	uint32_t nVelocityTimer;	
	uint32_t nVelocityCounter;
	uint8_t interrupt_tick;
	bool bMotorClockwise;
	bool bPositiveMove;
	volatile int32_t nMotorPosition;
	bool bPendingDestination;
	int32_t nPendingDestination;
	int32_t nPendingMoveDistance;
	int32_t nMotorDestination;
	TIM_HandleTypeDef *htim;
	uint32_t targetContinuousSpeed;
	uint16_t nLastEncoderPosition;
	uint32_t synchronousRatio;
	int32_t synchronousPosition;
	
	// Config items
	uint32_t nBacklash;
	uint32_t nTicksPerRotation;
	uint32_t nMaxVelocity;
	uint32_t nAcceleration;
	
	uint32_t isqrt32(uint32_t n) 
	{ 
		uint32_t  c = 0x8000; 
		uint32_t  g = 0x8000; 

		for(;;) 
		{ 
			if(g*g > n) 
			{
				g ^= c; 
			}
     
			c >>= 1; 

			if(c == 0) 
			{
				return g; 
			}

			g |= c; 
		 } 
	}
	
public:
	Motion(TIM_HandleTypeDef *htimer) : htim(htimer)
	{
		bContinuousMode = false;
		bSynchronousMotion = false;
		bMotorClockwise = true;
		nPendingMoveDistance = 0;
		nPendingDestination = 0;
		bPendingDestination = false;
		eState = eStopped;
		SetMotorEnable(false);
	}

	void SetMotorConfig(uint32_t backlash, uint32_t stepsPerRevolution, uint32_t gearRatio, uint32_t maxVelocity, uint32_t acceleration)
	{
		nBacklash = backlash;
		nTicksPerRotation = stepsPerRevolution * gearRatio;
		nMaxVelocity = MakeRawSpeed(maxVelocity);
		nAcceleration = MakeRawAcceleration(acceleration);
	}

	void ResetMotorCounters(void)
	{
		__disable_irq();
		nMotorPosition = 0;
		nMotorDestination = 0;
		nPendingMoveDistance = 0;
		interrupt_tick = 0;
		__enable_irq();
	}

	// timer interrupt.  It takes 2 interrupts to complete 1 cycle.
	void TimerInterruptHandler()
	{
		SetTimerCount(nVelocityTimer);

		interrupt_tick++;
		if (interrupt_tick & 1)
		{
			SetMotorStep(true);
			return;
		}

		SetMotorStep(false);

		// Move to the next step
		if (bMotorClockwise)
			SetMotorDirection(true);
		else
			SetMotorDirection(false);


		if (bMotorClockwise)
		{
			nMotorPosition++;
			if (nMotorPosition >= (long)nTicksPerRotation)
				nMotorPosition -= (long)nTicksPerRotation;
		}
		else
		{
			nMotorPosition--;
			if (nMotorPosition < 0)
				nMotorPosition += (long)nTicksPerRotation;
		}

		// Update AB output
		OutputQuadrature(nMotorPosition);

		if (bSynchronousMotion)
		{
			if (nMotorPosition == nMotorDestination)
			{
				__HAL_TIM_DISABLE(htim);
			}
		}
		else
		{
			if (nVelocityCounter > 0)
				nVelocityCounter--;
			if (nVelocityCounter == 0)
				MotionUpdate();
		}
	}

	// Degrees in 1000ths - Destination will always be +
	void MoveToDegrees(int32_t nDegrees)
	{
		unsigned long nSteps = AbsDegreesToSteps(nDegrees);

		nPendingDestination = (int32_t)nSteps;
		bPendingDestination = true;
	}

	// Degrees in 1000ths 
	void MoveToDegreesCW(int32_t nDegrees)
	{
		unsigned long nStepPosition = AbsDegreesToSteps(nDegrees);
		long nDiff = nStepPosition - nMotorPosition;
		if (nDiff < 0)
			nDiff += (long)nTicksPerRotation;

		nPendingMoveDistance += (int32_t)nDiff;
	}

	// Degrees in 1000ths 
	void MoveToDegreesCCW(int32_t nDegrees)
	{
		unsigned long nStepPosition = AbsDegreesToSteps(nDegrees);
		long nDiff = nStepPosition - nMotorPosition;
		if (nDiff > 0)
			nDiff -= (long)nTicksPerRotation;

		nPendingMoveDistance += (int32_t)nDiff;
	}

	void MoveToTicks(int32_t nTicks)
	{
		nPendingDestination = nTicks;
		bPendingDestination = true;
	}

	void MoveDegrees(int32_t nDegrees)
	{
		if (nDegrees == 0)
			return;

		int32_t n = DegreesToSteps(nDegrees);

		if (n == 0)
			n = sign(nDegrees) * 1;

		__disable_irq();
		nPendingMoveDistance += n;
		__enable_irq();
	}


	void MoveSteps(int32_t nSteps)
	{
		__disable_irq();
		nPendingMoveDistance += nSteps;
		__enable_irq();
	}

	void UpdateMotorTravel(void)
	{
		SetMotorEnable(true);

		// Need to move.
		if (nPendingMoveDistance != 0 || bPendingDestination)
		{
#ifdef MOVINGSUPPORT
			if (nMotorC != 0)		// Are we moving?
			{
				// Don't interrupt while we calculate this.
				__disable_irq();

				if (bMotorAccelerating)
				{
					// Still accelerating - just update the mid point, which is the decelerate position
					nMotorDeceleratePosition = (nMotorDestination + nMotorStartPosition) / 2;
				}
				else if (bMotorDecelerating)
				{
					nMotorDeceleratePosition = (nMotorDestination + nMotorStartPosition) / 2;

					if ((bMotorClockwise && nMotorPosition < nMotorDeceleratePosition) ||
						(!bMotorClockwise && nMotorPosition > nMotorDeceleratePosition))
						bMotorAccelerating = true;
				}
				else
				{
					// Not Acc or Dec, just continue to coast
					nMotorDeceleratePosition = nMotorDestination - (nMotorMaxSpeedPosition - nMotorStartPosition);
				}

				__enable_irq();
			}
			else
#endif
				if (eState == eStopped)
				{
					long nDistance;
					interrupt_tick = 0;

					if (bPendingDestination)
					{
						long nDiff = nPendingDestination - nMotorPosition;
						long nAbsDiff = nDiff < 0 ? -nDiff : nDiff;
						if (nAbsDiff < 4*nBacklash || nAbsDiff - 4 * nBacklash < (long)nTicksPerRotation / 2)		// find the shortest route, CW or CCW
						{
							if (nDiff < 0)
								bMotorClockwise = false;
							else
								bMotorClockwise = true;
							nDistance = nAbsDiff;
						}
						else
						{
							// wrap 360
							if (nDiff < 0)
							{
								nDistance = nDiff + (long)nTicksPerRotation;
								bMotorClockwise = true;
							}
							else
							{
								nDistance = nDiff - (long)nTicksPerRotation;
								bMotorClockwise = false;
							}
							nDistance = nDistance < 0 ? -nDistance : nDistance;
						}
						nMotorDestination = nPendingDestination;

						// Add a pending move.  This may change direction of the goto.
						if (nPendingMoveDistance != 0)
						{
							if (bMotorClockwise)
								nDistance += nPendingMoveDistance;
							else
								nDistance -= nPendingMoveDistance;
							if (nDistance < 0)
							{
								nDistance = -nDistance;
								bMotorClockwise = !bMotorClockwise;
							}
							nMotorDestination += nPendingMoveDistance;
						}
					}
					else
					{
						if (nPendingMoveDistance < 0)
						{
							bMotorClockwise = false;
							nDistance = -nPendingMoveDistance;
						}
						else
						{
							bMotorClockwise = true;
							nDistance = nPendingMoveDistance;
						}
						nMotorDestination += nPendingMoveDistance;
					}

					while (nMotorDestination >= (long)nTicksPerRotation)
						nMotorDestination -= (long)nTicksPerRotation;

					while (nMotorDestination < 0)
						nMotorDestination += (long)nTicksPerRotation;

					nPendingMoveDistance = 0;
					bPendingDestination = false;

					if (nDistance == 0)
					{
						return;
					}
					else if (nDistance == 1)
					{
						// fudge.

						// Move to the next step
						if (bMotorClockwise)
							SetMotorDirection(true);
						else
							SetMotorDirection(false);

						// TODO - use on pulse timer feature
						SetMotorStep(true);
						DWT_Delay_us(2);
						SetMotorStep(false);

						// We always step 1 unit at a time.
						if (bMotorClockwise)
							nMotorPosition++;
						else
							nMotorPosition--;

						// Update AB output
						OutputQuadrature(nMotorPosition);
						return;
					}


					// Precompute the path
					nPathAcceleration = nAcceleration;
					int nMaxAccelerationTime = nMaxVelocity / nPathAcceleration;
					long nMaxAccelerationDistanceX2 = (long)nPathAcceleration * (long)nMaxAccelerationTime * (long)nMaxAccelerationTime;

					nRunTimeFraction = 0;
					nAccTime = 0;
					nRunTime = 0;
					nDecTime = 0;

					if (nDistance > 2 * nMaxAccelerationDistanceX2)	// *2 for acc + dec
					{
						// Accelerate to max, run, then decelerate.
						// We fiddle with the run time to get the correct distance.
						nAccTime = nMaxAccelerationTime;
						int nPeakVelocity = nAccTime * nPathAcceleration;
						nRunTime = (nDistance - nMaxAccelerationDistanceX2) / nPeakVelocity;
						nDecTime = nMaxAccelerationTime;

						// Calculate the run time fraction - extra steps of const vel required to complete the trip
						nRunTimeFraction = (nDistance - nMaxAccelerationDistanceX2) % nPeakVelocity;
						if (nRunTimeFraction != 0)
							nRunTime++;
					}
					else
					{
						// We won't get to max without overruning the end.
						// Accelerate, short run, decelerate.
						if (nPathAcceleration * 2 > nDistance)
						{
							// Problem.  Acceleration is too fast.
							nPathAcceleration = nDistance / 4;
						}
						//nAccTime = isqrt32((2 * (nDistance / 2) / nPathAcceleration));
						nAccTime = isqrt32(nDistance / nPathAcceleration);
						int32_t nAccelerationDistanceX2 = nPathAcceleration * nAccTime * nAccTime;
						int32_t nPeakVelocity = nAccTime * nPathAcceleration;
						nRunTime = (nDistance - nAccelerationDistanceX2) / nPeakVelocity;
						nDecTime = nAccTime;

						// Calculate the run time fraction.
						nRunTimeFraction = (nDistance - nAccelerationDistanceX2) % nPeakVelocity;
						if (nRunTimeFraction != 0)
							nRunTime++;
					}


					// Need to move

					// Power up motor.
					nVelocity = 0;
					eState = eAccelerating;
					MotionUpdate();

					EnableTimerInterrupt(true);
				}
		}
	}
	
	int32_t MotorPosition()
	{
		__disable_irq();
		uint32_t n = nMotorPosition;
		__enable_irq();
		return n;
	}

	int32_t MotorDesitination()
	{
		return nMotorDestination;
	}

	void MotorStop()
	{
		if (bSynchronousMotion)
		{
			eState = Motion::eStopped;
		}
		if (eState != Motion::eStopped)
		{
			if (eState == Motion::eDecelerating)
			{
				// already stopping.
			}
			else
			{
				__disable_irq();
				eState = Motion::eDecelerating;
				nPathAcceleration = nAcceleration;
				nDecTime = nVelocity / nPathAcceleration;
				if (nDecTime == 0)
					nDecTime = 1;
				__enable_irq();
			}
		}
		bContinuousMode = false;
		bSynchronousMotion = false;
	}

	void SetContinuousSpeed(int32_t nContinuousSpeed)
	{
		bool bDir = nContinuousSpeed >= 0;
		targetContinuousSpeed = MakeRawSpeed(nContinuousSpeed);

		if (eState == eStopped)
		{
			bMotorClockwise = bDir;	// only change direction if we are stopped
			bContinuousMode = true;
			// calculate time to accelerate to requested speed.
			nPathAcceleration = nAcceleration;
			if (nPathAcceleration > targetContinuousSpeed)
				nPathAcceleration = targetContinuousSpeed;
			if (nPathAcceleration == 0)
				nPathAcceleration = 1;
			nAccTime = targetContinuousSpeed / nPathAcceleration;
			nDecTime = nAccTime;
			nRunTime = 1;
			nRunTimeFraction = 0;

			nVelocity = 0;
			eState = eAccelerating;
			MotionUpdate();

			EnableTimerInterrupt(true);
		}
		else if (eState == eRunning)
		{
			if (targetContinuousSpeed > nVelocity)
			{
				// calculate time to accelerate to the new velocity.
				nPathAcceleration = nAcceleration;
				if (nPathAcceleration > (targetContinuousSpeed - nVelocity))
					nPathAcceleration = (targetContinuousSpeed - nVelocity);
				if (nPathAcceleration == 0)
					nPathAcceleration = 1;
				nAccTime = (targetContinuousSpeed - nVelocity) / nPathAcceleration;
				eState = eAccelerating;
			}
			else
			{
				// calculate time to decelerate to the new velocity.
				nPathAcceleration = nAcceleration;
				if (nPathAcceleration > (nVelocity - targetContinuousSpeed))
					nPathAcceleration = (nVelocity - targetContinuousSpeed);
				if (nPathAcceleration == 0)
					nPathAcceleration = 1;
				nDecTime = (nVelocity - targetContinuousSpeed) / nAcceleration;
				eState = eDecelerating;
			}
		}
	}
	
	void MSInterrupt()
	{
		if (bSynchronousMotion)
		{
			uint16_t encoderPosition = __HAL_TIM_GET_COUNTER(&htim2);
			int16_t encoderDelta = encoderPosition - nLastEncoderPosition;
			if (encoderDelta != 0)
			{
				nLastEncoderPosition = encoderPosition;
			
				synchronousPosition += encoderDelta;
				nMotorDestination = synchronousPosition * nTicksPerRotation / synchronousRatio;
				int32_t positionDelta = nMotorDestination - nMotorPosition;
			
				bMotorClockwise = positionDelta > 0 ? true : false;
				// calculate the step timer - we need 2 * positionDelta interrupts in 1ms.
				nVelocityTimer = 0xFFFF - (1000 / abs(positionDelta));
				if (nVelocityTimer == 0xFFFF)
					nVelocityTimer = 0xFFFE;
				SetTimerCount(nVelocityTimer);
				EnableTimerInterrupt(true);
				__HAL_TIM_ENABLE(htim);
			}
		}
	}

	void StartSynchronised(uint32_t SynchronousStepsPerRev, uint32_t SynchronousRatio)
	{
		synchronousRatio = SynchronousRatio * SynchronousStepsPerRev;
		bSynchronousMotion = true;
		synchronousPosition = 0;
		nMotorPosition = 0;
		nMotorDestination = 0;
		nLastEncoderPosition = 0;
		bMotorClockwise = true;
		eState = eRunning;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_ENABLE(htim);
	}
	
	uint32_t AbsDegreesToSteps( int32_t nDegrees )
	{
		while ( nDegrees >= 360000 )
			nDegrees -= 360000;
		while ( nDegrees < 0 )
			nDegrees += 360000;

		// Convert absolute position in degrees to absolute position in steps.
		unsigned long long nSteps = nDegrees;
		nSteps *= nTicksPerRotation;
		nSteps /= 360000UL;

		return nSteps;
	}

	int32_t DegreesToSteps( int32_t nDegrees )
	{
		long long nSteps = nDegrees;
		nSteps *= nTicksPerRotation;
		nSteps /= 360000UL;

		return nSteps;
	}
	
private:
	void SetMotorStep(bool state)
	{
		HAL_GPIO_WritePin(GPIO_STEPPER_STEP_GPIO_Port, GPIO_STEPPER_STEP_Pin, state ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
	}
	
	void SetMotorDirection(bool state)
	{
		if (!bPositiveMove)
			state = !state;
		HAL_GPIO_WritePin(GPIO_STEPPER_DIR_GPIO_Port, GPIO_STEPPER_DIR_Pin, state ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
	}
	
	void SetMotorEnable(bool state)
	{
		HAL_GPIO_WritePin(GPIO_STEPPER_DIR_GPIO_Port, GPIO_STEPPER_ENABLE_Pin, state ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
	}
	
	void OutputQuadrature(uint32_t nMotorPosition)
	{
		//static uint8_t Quadrature[] = { 0, 0x20, 0x30, 0x10 };
		//MOTOR_PORT = (MOTOR_PORT & ~0x30) | Quadrature[nMotorPosition & 0x3];
	}
	
	void EnableTimerInterrupt(bool enable)
	{
		if (enable)
		{
			__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
			__HAL_TIM_ENABLE(htim);
		}
		else
		{
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
			__HAL_TIM_DISABLE(htim);
		}
	}

	void SetTimerCount(uint16_t count)
	{
		__HAL_TIM_SET_COUNTER(htim, count);
	}

	void MotionUpdate( void )
	{
		unsigned int nNewVelocity = nVelocity;

		switch ( eState )
		{
			case eAccelerating:
				nAccTime--;
				nNewVelocity += nPathAcceleration;
				if ( nAccTime == 0 )
				{
					if ( nRunTime == 0 && nRunTimeFraction == 0 )
						eState = eDecelerating;
					else
						eState = eRunning;
				}
				break;

			case eRunning:
				if (bContinuousMode)
				{
					// Truncating errors can stop us reaching target velocity - fudge here.
					if (targetContinuousSpeed != nNewVelocity)
						nNewVelocity += sign(targetContinuousSpeed - nNewVelocity);
				}
				else if ( !bContinuousMode )
					nRunTime--;
				if ( nRunTime == 0 )
					eState = eDecelerating;
				break;

			case eDecelerating:
				nDecTime--;
				if ( nPathAcceleration > nNewVelocity )	// It is possible to decelerate too far, if we are asked to stop.
					nDecTime = 0;
				else
					nNewVelocity -= nPathAcceleration;
				if ( nDecTime == 0 )
				{
					if ( bContinuousMode )	// Continuous mode just changes speed.
					{
						eState = eRunning;
					}
					else
					{
						EnableTimerInterrupt(false);
						eState = eStopped;
					}
				}
				break;

			case eStopped:
				break;
		}

		// set timer counter to -COUNT, so we overflow at next time step
		if ( nNewVelocity != nVelocity )
		{
			// Change in velocity
			nVelocity = nNewVelocity;
			// Change timer
			nVelocityTimer = 0xFFFF;
			nVelocityTimer /= nVelocity;
			nVelocityTimer = -nVelocityTimer;
			SetTimerCount(nVelocityTimer);
		}
		if ( nRunTime == 0 && nRunTimeFraction != 0 )
		{
			nVelocityCounter = nRunTimeFraction;
			nRunTimeFraction = 0;
		}
		else
			nVelocityCounter = nVelocity;
	}

	uint32_t MakeRawSpeed(int32_t nSpeed)	// Convert steps per second to ticks per interupt unit
	{
		// Timer 2MHz.  Max 65535us - 65.535 ms step period (x2 cycles) 
		// Each tick is 1us
		// we are not in /s.  We are in /65.535ms, or, per .065535 s

		uint64_t nTemp = abs(nSpeed);
		nTemp *= 65535l;
		nTemp /= 1000000l;

		return (uint32_t)nTemp;
	}

	uint32_t MakeRawAcceleration(int32_t nAcc)	// Convert steps per second per second to ticks per interupt unit per interrupt unit
	{
		uint64_t nTemp = abs(nAcc);
		nTemp *= 65535l;
		nTemp *= 65535l;
		nTemp /= 1000000l;
		nTemp /= 1000000l;

		return (uint32_t)nTemp;
	}

};

