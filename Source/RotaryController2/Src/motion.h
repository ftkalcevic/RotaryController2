#pragma  once

#include "main.h"
#include "dwt_stm32_delay.h"

#define STEP_PULSE_HIGH_US  4
#define MISSED_STEP_COUNT	20	// 200 step motor, 10 microsteps.  1 missed step 10 +/- 5.  

#define DEGREES_SCALE		100UL
#define DEGREES_SCALE_DIGITS 2
class MotionTests;

class Motion
{
public:
	friend MotionTests;

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
	volatile int32_t nMotorPosition;
	bool bPendingDestination;
	int32_t nPendingDestination;
	int32_t nPendingMoveDistance;
	int32_t nMotorDestination;
	uint16_t last_encoder_position;
	volatile int32_t nEncoderPosition;
	TIM_HandleTypeDef *htim;
	TIM_HandleTypeDef *hEnc;
	uint32_t targetContinuousSpeed;
	uint16_t nLastSpindleEncoderPosition;
	uint32_t synchronousRatio;
	int32_t synchronousPosition;
	bool timerInterruptEnabled;
	uint8_t encoderOutLast;
	volatile bool bEncoderError;
	
	// Config items
	uint32_t nBacklash;
	int32_t nTicksPerRotation;
	uint32_t nMaxVelocity;
	uint32_t nAcceleration;
	int32_t nEncoderTicksPerRevolution;
	bool bEncoderReverse;
	bool bMotorReverse;

	
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

	bool UpdateEncoderPosition()
	{
		if(nEncoderTicksPerRevolution)
		{
			uint16_t current_encoder_position = __HAL_TIM_GET_COUNTER(hEnc);
			int16_t diff = last_encoder_position - current_encoder_position;
			if (diff != 0)
			{
				if ( bEncoderReverse )
					nEncoderPosition -= diff;
				else
					nEncoderPosition += diff;
			
				if (nEncoderPosition > nEncoderTicksPerRevolution)
					nEncoderPosition -= nEncoderTicksPerRevolution;
				else if (nEncoderPosition < 0)
					nEncoderPosition += nEncoderTicksPerRevolution;
			
				if (!bEncoderError)
				{
					// todo - divide will be slow 
					int32_t encoder_steps = nEncoderPosition * nTicksPerRotation / nEncoderTicksPerRevolution;
					int32_t error = abs(encoder_steps - nMotorPosition);
					if (error > nTicksPerRotation / 2)
						error -= nTicksPerRotation;
					if (abs(error) > MISSED_STEP_COUNT)
						bEncoderError = true;
				}
				last_encoder_position = current_encoder_position;
				return true;
			}
		}
		return false;
	}
	
public:
	Motion(TIM_HandleTypeDef *htimer, TIM_HandleTypeDef *hencoder) : htim(htimer), hEnc(hencoder)
	{
		SetMotorEnable(false);
		bContinuousMode = false;
		bSynchronousMotion = false;
		bMotorClockwise = true;
		nPendingMoveDistance = 0;
		nPendingDestination = 0;
		bPendingDestination = false;
		eState = eStopped;
		timerInterruptEnabled = false;
		encoderOutLast = 0;
		ResetMotorCounters();
	}

	void SetMotorConfig(uint32_t backlash, uint32_t stepsPerRevolution, uint32_t gearRatio, bool motorReverse, uint32_t encoderStepsPerRevolution, bool encoderReverse, uint32_t maxVelocity, uint32_t acceleration)
	{
		nBacklash = backlash;
		nTicksPerRotation = stepsPerRevolution * gearRatio;
		bMotorReverse = motorReverse;
		nMaxVelocity = MakeRawSpeed(maxVelocity);
		nAcceleration = MakeRawAcceleration(acceleration);
		nEncoderTicksPerRevolution = encoderStepsPerRevolution * gearRatio;
		bEncoderReverse = encoderReverse;
	}

	void ResetMotorCounters(void)
	{
		__disable_irq();
		nMotorPosition = 0;
		nMotorDestination = 0;
		nPendingMoveDistance = 0;
		interrupt_tick = 0;
		nEncoderPosition = 0;
		ClearEncoderError();
		__enable_irq();
	}
	
	void ClearEncoderError()
	{
		bEncoderError = false;
	}
	
	bool EncoderError()
	{
		return bEncoderError;
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

		// Check actual position
		UpdateEncoderPosition();
			
		// Update AB output
		OutputQuadrature(bMotorClockwise);

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
			if ( eState != Motion::eStopped ) // Are we moving?
			{
				// this is still too buggy.
				nPendingMoveDistance=0;
				bPendingDestination = false;
				return;
				
				// Don't interrupt while we calculate this.
				__disable_irq();
				
				// if we are updating half way through a cycle, force complete the cycle
				//ESTATE currentState = eState;
				if (interrupt_tick & 1)
				{
					TimerInterruptHandler();
				}

				// New "to go" distance - "still to go" + "change"
				uint32_t distanceToMove = nMotorDestination - nMotorPosition + nPendingMoveDistance;

				// x = 1/2 a t2 + u t
				// v = a t + u

				// x = 1/2 a t2 + u t
				uint32_t timeDecelerate = nVelocity / nPathAcceleration;
				uint32_t decelerateDistanceFromV0 = nVelocity * timeDecelerate / 2 + (timeDecelerate * nPathAcceleration / 2);	// don't divide distances
				if (decelerateDistanceFromV0 >= distanceToMove)
				{
					// decelerating now, will get us to the new position.
					// Compute new deceleration settings
					if (nPendingMoveDistance > nVelocityCounter)
					{
						// too hard for now.
					}
					else
					{
						nVelocityCounter += nPendingMoveDistance;
						nMotorDestination += nPendingMoveDistance;
					}
				}
				//else if (currentState == Motion::eRunning)
				//{
				//	// We are at Vmax.  Coast, then decelerate
				//	uint32_t additionalRunDistance = distanceToMove - decelerateDistanceFromV0;
				//	// Update runtime/runfraction
				//	
				//	// x = vt
				//	nRunTime = additionalRunDistance / nVelocity+1;
				//	nVelocityCounter = additionalRunDistance % nVelocity;
				//	nRunTimeFraction = 0;
				//	eState = Motion::eRunning;
				//	
				//	// use existing deceleration settings
				//	
				//	nMotorDestination += nPendingMoveDistance;
				//}
				else
				{
					// Else, accelerate from V0, optional coast, decelerate.  Similar to below, but not symetrical V0 != Vf
					int32_t timeDecelerate = nMaxVelocity / nAcceleration;
					uint32_t distanceDecelerate = nMaxVelocity * timeDecelerate / 2 + (timeDecelerate * nAcceleration / 2);	// don't divide distances
					int32_t timeAccelerate = (nMaxVelocity - nVelocity) / nAcceleration;
					uint32_t distanceAccelerateFromV0 = nVelocity * timeAccelerate + (nMaxVelocity - nVelocity) * timeAccelerate / 2 - (timeAccelerate * nAcceleration / 2);	// don't divide distances
					if (distanceAccelerateFromV0 + distanceDecelerate <= distanceToMove)
					{
						// we can accelerate to Vmax
//						nPathAcceleration = nAcceleration;
						int nPeakVelocity = nVelocity + timeAccelerate * nAcceleration;
						
						distanceAccelerateFromV0 = nVelocity * timeAccelerate + (nPeakVelocity - nVelocity) * timeAccelerate / 2 - (timeAccelerate * nAcceleration / 2);	// don't divide distances
						
						timeDecelerate = nPeakVelocity / nPathAcceleration;
						distanceDecelerate = nPeakVelocity * timeDecelerate / 2 + (timeDecelerate * nAcceleration / 2);	// don't divide distances

						nAccTime = timeAccelerate;
						nDecTime = timeDecelerate;
						uint32_t runDistance = distanceToMove - distanceAccelerateFromV0 - distanceDecelerate;
						nRunTime = runDistance / nPeakVelocity;
						nRunTimeFraction = runDistance % nPeakVelocity;
						if (nRunTimeFraction != 0)
							nRunTime++;
						if (nAccTime > 0)
						{
							eState = Motion::eAccelerating;
							nVelocityCounter = nVelocity;
						}
						else
						{
							eState = Motion::eRunning;
							nVelocityCounter = nVelocity;
						}
						nMotorDestination += nPendingMoveDistance;
					}
					else
					{
						// Can't accelerate to Vmax
						// Find v
						uint32_t n = (2*nAcceleration*distanceToMove + nVelocity * nVelocity) / 2;
						uint32_t targetVelocity = isqrt32(n);
						
//						nPathAcceleration = nAcceleration;
						timeAccelerate = (targetVelocity - nVelocity) / nAcceleration;
						if (timeAccelerate < 0)
							timeAccelerate = 0;
						
						int nPeakVelocity = nVelocity + timeAccelerate * nPathAcceleration;
						distanceAccelerateFromV0 = nVelocity * timeAccelerate + (nPeakVelocity - nVelocity) * timeAccelerate / 2 - (timeAccelerate * nPathAcceleration / 2);	// don't divide distances
						
						timeDecelerate = nPeakVelocity / nPathAcceleration;
						distanceDecelerate = nPeakVelocity * timeDecelerate / 2 + (timeDecelerate * nPathAcceleration / 2);	// don't divide distances
						
						nAccTime = timeAccelerate;
						nDecTime = timeDecelerate;
						uint32_t runDistance = distanceToMove - distanceAccelerateFromV0 - distanceDecelerate;
						nRunTime = runDistance / nPeakVelocity;
						nRunTimeFraction = runDistance % nPeakVelocity;
						if (nRunTimeFraction != 0)
							nRunTime++;
						if (nAccTime > 0)
						{
							eState = Motion::eAccelerating;
							nVelocityCounter = nVelocity;
						}
						else if (nRunTime > 0)
						{
							eState = Motion::eRunning;
							if (nRunTime == 1)
							{
								nVelocityCounter = nRunTimeFraction;
								nRunTimeFraction = 0;
							}
							else
							{
								nVelocityCounter = nVelocity;
							}
						}
						else
						{
							eState = Motion::eDecelerating;
							nVelocityCounter = nVelocity;
						}
						nMotorDestination += nPendingMoveDistance;
					}
				}

				__enable_irq();
				EnableTimerInterrupt(true);
				nPendingMoveDistance=0;
				bPendingDestination = false;
			}
			else
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
				else if (nDistance <= 2)
				{
					// fudge.

					// Move to the next step
					if (bMotorClockwise)
						SetMotorDirection(true);
					else
						SetMotorDirection(false);

					// TODO - use one pulse timer feature.  Delays are bad.
					while(nDistance > 0)
					{
						SetMotorStep(true);
						DWT_Delay_us(STEP_PULSE_HIGH_US);
						SetMotorStep(false);

						// We always step 1 unit at a time.
						if (bMotorClockwise)
							nMotorPosition++;
						else
							nMotorPosition--;

						UpdateEncoderPosition();
					
						// Update AB output
						OutputQuadrature(bMotorClockwise);
						nDistance--;
					}
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
					// todo - use div_t t = div()
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
					// todo - use div_t t = div()
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

	int32_t EncoderPosition()
	{
		__disable_irq();
		uint32_t n = nEncoderPosition;
		__enable_irq();
		return n;
	}
	
	bool Update()
	{
		if (eState == Motion::eStopped)
			if (UpdateEncoderPosition())
				return true;
		return false;
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
		SetMotorEnable(true);
		
		bool bDir = nContinuousSpeed >= 0;
		targetContinuousSpeed = MakeRawSpeed(abs(nContinuousSpeed));
		if (targetContinuousSpeed > nMaxVelocity)
			targetContinuousSpeed = nMaxVelocity;

		if (targetContinuousSpeed == nVelocity)
			return;
		
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
				nDecTime = (nVelocity - targetContinuousSpeed) / nPathAcceleration;
				eState = eDecelerating;
			}
		}
	}
	
	void MSInterrupt()
	{
		if (bSynchronousMotion)
		{
			uint16_t encoderPosition = __HAL_TIM_GET_COUNTER(&htim2);
			int16_t encoderDelta = encoderPosition - nLastSpindleEncoderPosition;
			if (encoderDelta != 0)
			{
				nLastSpindleEncoderPosition = encoderPosition;
			
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
		nLastSpindleEncoderPosition = 0;
		bMotorClockwise = true;
		eState = eRunning;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_ENABLE(htim);
	}
	
	uint32_t AbsDegreesToSteps( int32_t nDegrees )
	{
		while ( nDegrees >= 360*DEGREES_SCALE )
			nDegrees -= 360*DEGREES_SCALE;
		while ( nDegrees < 0 )
			nDegrees += 360*DEGREES_SCALE;

		// Convert absolute position in degrees to absolute position in steps.
		unsigned long long nSteps = nDegrees;
		nSteps *= nTicksPerRotation;
		nSteps /= 360UL*DEGREES_SCALE;

		return (uint32_t)nSteps;
	}

	int32_t DegreesToSteps( int32_t nDegrees )
	{
		long long nSteps = nDegrees;
		nSteps *= nTicksPerRotation;
		nSteps /= 360UL*DEGREES_SCALE;

		return nSteps;
	}
	
	void Recover()
	{
		int32_t actualPosition = nEncoderPosition * nTicksPerRotation / nEncoderTicksPerRevolution;
		nMotorPosition = actualPosition;
		ClearEncoderError();
	}
	
	void SetMotorEnable(bool enabled)
	{
		// if we enable for the first time, reset the encoder.
		bool bFirst = enabled && HAL_GPIO_ReadPin(GPIO_STEPPER_DIR_GPIO_Port, GPIO_STEPPER_ENABLE_Pin) == GPIO_PinState::GPIO_PIN_RESET;
		HAL_GPIO_WritePin(GPIO_STEPPER_DIR_GPIO_Port, GPIO_STEPPER_ENABLE_Pin, enabled ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
		if (bFirst)
		{
			// small delay
			HAL_Delay(2); // 2ms
			last_encoder_position = 0;
			__HAL_TIM_SET_COUNTER(hEnc, 0);
		}
	}
	
private:
	void SetMotorStep(bool state)
	{
		HAL_GPIO_WritePin(GPIO_STEPPER_STEP_GPIO_Port, GPIO_STEPPER_STEP_Pin, state ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
	}
	
	void SetMotorDirection(bool state)
	{
		if (bMotorReverse)
			state = !state;
		HAL_GPIO_WritePin(GPIO_STEPPER_DIR_GPIO_Port, GPIO_STEPPER_DIR_Pin, state ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
	}
	
	void OutputQuadrature(bool bClockwise)
	{
		// Quadrature only ever changes one pin at a time, so we can toggle the pin so we don't accidently update the other pins that update during an interrupt
		
		//	#	A B
		//	0	0 0
		//	1	1 0
		//	2	1 1
		//	3	0 1
		
		switch (encoderOutLast & 0x1)
		{
			case 0: HAL_GPIO_TogglePin(GPIO_ENCODER_OUT_A_GPIO_Port, bClockwise ? GPIO_ENCODER_OUT_A_Pin : GPIO_ENCODER_OUT_B_Pin); break;
			case 1: HAL_GPIO_TogglePin(GPIO_ENCODER_OUT_A_GPIO_Port, bClockwise ? GPIO_ENCODER_OUT_B_Pin : GPIO_ENCODER_OUT_A_Pin); break;
			//case 2: HAL_GPIO_TogglePin(GPIO_ENCODER_OUT_A_GPIO_Port, bClockwise ? GPIO_ENCODER_OUT_A_Pin : GPIO_ENCODER_OUT_B_Pin); break;
			//case 3: HAL_GPIO_TogglePin(GPIO_ENCODER_OUT_A_GPIO_Port, bClockwise ? GPIO_ENCODER_OUT_B_Pin : GPIO_ENCODER_OUT_A_Pin); break;
		}
		encoderOutLast += bClockwise ? 1 : 1;
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
		timerInterruptEnabled = enable;
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
						//nNewVelocity += sign(targetContinuousSpeed - nNewVelocity);
						nNewVelocity = targetContinuousSpeed;
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
		if ( nNewVelocity != nVelocity && nNewVelocity != 0 )
		{
			// Change in velocity
			nVelocity = nNewVelocity;
			// Change timer
			nVelocityTimer = 0xFFFF;
			nVelocityTimer /= nVelocity;
			nVelocityTimer = 0-nVelocityTimer;
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

