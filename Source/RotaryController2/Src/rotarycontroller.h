#pragma  once

class RotaryController
{
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
	
	void DoSplash();
	void DoJog();
	void DoSetup();
	
};