#pragma  once

class RotaryController;

enum Units : uint8_t
{
	Degrees = 0,
	Steps = 1
};

enum MenuItemType
{
	ENUM,
	UINT32,
	BOOL
};

struct MenuItem
{
	const char *name;
	void *memAddr;
	MenuItemType type;
	uint32_t min;
	uint32_t max;
	const char * const *enums;
	void(RotaryController::* OnChangeEvent)();
};

class RotaryController
{
	void WriteEEPROM();
	void ReadEEPROM();
	bool eepromHasBeenReset;

	uint32_t nTicksPerRotation;
	uint32_t nStepperDivisionsPerRotation;			// Steps per rotation on stepper
	uint32_t nGearRatio;							// Worm to table turn ratio
	uint32_t nBacklash;
	Units movementUnits;
	int32_t division, divisions;
	bool isSlowSpeed;
	int32_t continuousSpeed;
	
public:
	enum EBacklight
	{
		Off,
		Red,
		Green
	};
	
	RotaryController();
	void Init();
	void Run();
	void SetBacklight(EBacklight bl);
	void SetRunButton(bool on);
	uint32_t DoKeyScanEtc();
	void MakeDegrees(char *buffer, uint32_t buflen, int32_t nValue);
	void SetParameters();
	void ToggleUnits();
	void ToggleSpeed();
	
	void DoSplash();
	void DoJog();
	void DoSetup();
	void DoDivisions();
	void DoContinuous();
	void DoSynchronised();
	void DoSegment();
		
	bool DrawSegment(uint32_t sequence, uint32_t sequences, uint32_t sequenceRepeats, bool block);
	bool DrawSynchronised(bool block);
	void DoGoto(uint8_t row);
	bool DrawDivisions(bool block);
	bool DrawContinous(bool block);
	void ShowRotateSpeed();
		
	bool DrawJog(bool block=true);
	void Jog(int32_t distance);
	void JogTo(int32_t position);
		
	void DisplayCoordinates();
	void DoSetupChangeDevice();
	void DrawSetup(MenuItem &item);
	void DisplaySetupValue(uint8_t col, uint8_t row, MenuItem &item);
	bool GetDigit(int nKey, uint8_t &nValue);
	bool EditItem(MenuItem &item);
	bool EditInt32(int32_t *n, uint8_t col, uint8_t row, int32_t min, uint32_t max);
	bool EditEnum(uint32_t *n, const int row, uint32_t min, uint32_t max, const char *const *enums);
	bool EditUInt32Frac(uint8_t col, uint8_t row, uint32_t *n);
};