// #include "daisy_seed.h"

/** Our hardware board class handles the interface to the actual DaisyPatchSM
 * hardware. */
#include "daisy_patch_sm.h"
#include "daisysp.h"
// #include "onepole.h"

#define BUTTON_SAMPLE_RATE 100
#define TIME_LED_BRIGHTNESS 0.6

// Set max delay time to 0.75 of samplerate.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)

using namespace daisy;
// using namespace daisy::seed;
using namespace patch_sm;

using namespace daisysp;

// DaisySeed hw;
DaisyPatchSM hw;


static OnePole AmpLowPass;
static Svf BassLowPass;
static Svf    AmpBandPass;
static Svf    TrebleBandPass;
static Svf    MidBandPass;
static Overdrive Drive;

// static ReverbSc rev;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dlyl;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dlyr;

float knob_gain, knob_volume, knob_treble, knob_mid_gain, knob_bass, knob_color, knob_fx_dry_wet, knob_fx_feedback, knob_fx_time, knob_fx_tone, ctrl_blend;
int16_t  knob_mid_freq;

Switch button_link, button_blend, button_fx; // button_stereo, button_cv;

GPIO led_link, led_blend, led_delay, led_reverb, led_split, led_stereo, led_time; //TODO reverse orientation of split and stereo LEDs, rotate time led to match others

Led led_time_pwm;


GateIn button_stereo, button_cv;

Oscillator time_osc;

bool state_link, state_blend, state_fx, state_stereo, state_cv, state_stereo_button, state_cv_button;

float maxDelay, currentDelay, feedback, delayTarget, cutoff, drive_compensation;


void GetReverbSample(float &inl, float &inr);
void GetDelaySample(float &inl, float &inr);
void process_eq_mid(float &f1, float &f2);
void process_eq_bass(float &f1, float &f2);
void process_eq_treble(float &f1, float &f2);
void process_eq_color(float &f1, float &f2);
void process_drive(float &f1, float &f2);
void process_amp_sim(float &f1, float &f2);
void process_fx(float &f1, float &f2);


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		float f1, f2, f3, f4;

		f1 = in[0][i];
		f2 = in[1][i];

		process_eq_mid(f1, f2);

		f3 = f1;
		f4 = f2;

		process_drive(f1, f2);

		process_amp_sim(f1, f2);

		f1 = ctrl_blend * (f1 * drive_compensation) + ((1.0-ctrl_blend) * f3); //blend in clean signal
		if (state_stereo)
			f2 = ctrl_blend * (f2 * drive_compensation) + ((1.0-ctrl_blend) * f4);

		process_eq_treble(f1, f2);

		process_eq_bass(f1, f2);

		process_eq_color(f1, f2);
		

		// f1 = f1 * knob_volume * 1.2;
		// if (!state_stereo) 
		// 	f2 = f2 * knob_volume * 1.2;



		// uncomment for audio passthrough test
		// out[0][i] = in[0][i];
		// out[1][i] = in[1][i];

		process_fx(f1, f2);


		out[0][i] = f1;
		out[1][i] = f2;
	}
}

void process_eq_mid(float &f1, float &f2){
	MidBandPass.SetFreq(knob_mid_freq);
	MidBandPass.SetRes(abs(knob_mid_gain) * 0.5);
	MidBandPass.Process(f1);

	f1 = MidBandPass.Band() * knob_mid_gain * 2.0 + f1; //add bandpass to dry signal in +/- to act as peak or dip
	if (state_stereo) {
		MidBandPass.Process(f2);
		f2 = MidBandPass.Band() * knob_mid_gain * 2.0 + f2; //add bandpass to dry signal in +/- to act as peak or dip
	}
}

void process_eq_bass(float &f1, float &f2){
	BassLowPass.SetRes(0.2);
	BassLowPass.SetFreq(300); //(bassValue + 0.5) * 10000);
	BassLowPass.Process(f1);
	f1 = BassLowPass.Low() * knob_bass * 2.0 + f1;
	if (state_stereo) {
		BassLowPass.Process(f2);
		f2 = BassLowPass.Low() * knob_bass * 2.0 + f2;
	}
	// f2 = (f2 * bassValue);// - f1;
}

void process_eq_treble(float &f1, float &f2){
	TrebleBandPass.SetRes(0.2);
	TrebleBandPass.SetFreq(3500);
	TrebleBandPass.Process(f1);
	f1 = (TrebleBandPass.High() * knob_treble * 2.0) + f1;
	if (state_stereo) {
		TrebleBandPass.Process(f2);
		f2 = (TrebleBandPass.High() * knob_treble * 2.0) + f2;
	}
}


void process_amp_sim(float &f1, float &f2){
	// AmpLowPass.SetRes(0.2);
	AmpLowPass.SetFrequency(4100); //(bassValue + 0.5) * 10000);
	f1 = AmpLowPass.Process(f1);
	if (state_stereo) 
		f2 = AmpLowPass.Process(f2);
	

	AmpBandPass.SetRes(0.05);
	AmpBandPass.SetDrive(0.2);
	AmpBandPass.SetFreq(680);
	AmpBandPass.Process(f1);
	f1 = AmpBandPass.Band() * -0.8 + f1;
	if (state_stereo) {
		AmpBandPass.Process(f2);
		f2 = AmpBandPass.Band() * -0.8 + f2;
	}
	// AmpBandPass.Process(f2);
		// f1 = AmpBandPass.Band() * -1.0 + f2;
		// f2 = AmpLowPass.Process(f1);
}

void process_eq_color(float &f1, float &f2){

}

void process_drive(float &f1, float &f2){
	Drive.SetDrive(knob_gain*0.95 + 0.05);
	drive_compensation = (2.0-knob_gain) * 0.5;
	f1 = Drive.Process(f1);
	if (state_stereo) 
		f2 = Drive.Process(f2);
}

void process_fx(float &f1, float &f2) {
	float verbl = f1 * knob_fx_dry_wet;
	float verbr = f2 * knob_fx_dry_wet;
	
	if (!state_stereo)
		f1 = f1 * (1.0 - knob_fx_dry_wet*knob_fx_dry_wet);
	f2 = f2 * (1.0 - knob_fx_dry_wet*knob_fx_dry_wet);
	
	if (state_fx) {		
		GetReverbSample(verbl, verbr);
	} else { 
		GetDelaySample(verbl, verbr);
	}

	if (!state_stereo)
		f1 += verbl;
	f2 += verbr;
}

void setFilterConstants(float sample_rate);

void readKnobs();
void readButtons();

int main(void)
{
	hw.Init();

	//initialize buttons
	button_link.Init(hw.A8, BUTTON_SAMPLE_RATE);
	button_blend.Init(hw.D1, BUTTON_SAMPLE_RATE);
	button_fx.Init(hw.D2, BUTTON_SAMPLE_RATE);
	button_stereo.Init(hw.B10, BUTTON_SAMPLE_RATE);  //gate_in_1
	button_cv.Init(hw.B9, BUTTON_SAMPLE_RATE); //gate_in_2

	//initialize LEDs
	led_link.Init(hw.A9, GPIO::Mode::OUTPUT);
	led_blend.Init(hw.D3, GPIO::Mode::OUTPUT);
	led_delay.Init(hw.D4, GPIO::Mode::OUTPUT);
	led_reverb.Init(hw.D5, GPIO::Mode::OUTPUT);
	led_split.Init(hw.B5, GPIO::Mode::OUTPUT);
	led_stereo.Init(hw.B6, GPIO::Mode::OUTPUT);
	// led_time.Init(hw.B8, GPIO::Mode::OUTPUT);
	led_time_pwm.Init(hw.B8, false, 1000.0f);
	led_time_pwm.Set(127.0);

	//set initial fx/split state
	led_delay.Write(true);
	led_stereo.Write(true);
	state_fx = false;
	state_stereo = true;
	ctrl_blend = 1.0;

	// Set up the LED PWM oscillator
	time_osc.Init(hw.AudioSampleRate());
	time_osc.SetWaveform(Oscillator::WAVE_SIN);
	time_osc.SetFreq(1.0);

	// Start the ADC
	hw.adc.Start();

	// Initialize audio params
	setFilterConstants(hw.AudioSampleRate());
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
	


	while(1) {
		hw.ProcessAllControls();
		readKnobs();
		readButtons();		
		led_time_pwm.Update();
	}
}

float led_brightness = 0;

void readButtons(){
	button_blend.Debounce();
	if (button_blend.RisingEdge()) {
		state_blend = !state_blend;
		led_blend.Write(state_blend);
	}

	button_link.Debounce();
	if (button_link.RisingEdge()) {
		state_link = !state_link;
		led_link.Write(state_link);
	}

	button_fx.Debounce();
	if (button_fx.RisingEdge()) {
		state_fx = !state_fx; //True = delay active
		led_delay.Write(!state_fx);
		led_reverb.Write(state_fx);
		// led_time.Write(state_stereo);// todo convert to pwm pulse
	}	

	//button_stereo and button_cv use gate_in object and need special handling for toggle and state
	if ((button_stereo.State() && state_stereo_button == false)) {
		state_stereo_button = button_stereo.State();
		if (state_stereo_button == true) {
			state_stereo = !state_stereo;
			led_stereo.Write(state_stereo);
			led_split.Write(!state_stereo);
		}
	}
	state_stereo_button = button_stereo.State();



	if (button_cv.State() != state_cv_button) {
		state_cv_button = button_cv.State();
		state_cv = state_cv_button;
		led_stereo.Write(state_cv);
	}
}


void readKnobs(){
	hw.ProcessAnalogControls();
	hw.ProcessDigitalControls();

	knob_gain     = hw.GetAdcValue(ADC_9); // gainValue    = hw.adc.GetFloat(gainKnob);

	if (state_blend) {
		ctrl_blend  = hw.GetAdcValue(CV_2); //.5; //hw.adc.GetFloat(volumeKnob);
	} else {
		knob_volume = hw.GetAdcValue(CV_2); //.5; //hw.adc.GetFloat(volumeKnob);
	}
	
	knob_treble   = hw.GetAdcValue(ADC_10) - 0.25; //hw.adc.GetFloat(trebleKnob) - 0.25;
	knob_mid_freq = hw.GetAdcValue(CV_4) * 4000 + 170; //hw.adc.GetFloat(midFreqKnob) * 4000.0 + 170;
	knob_mid_gain = hw.GetAdcValue(CV_5) - 0.25; //hw.adc.GetFloat(midGainKnob) - 0.25;
	knob_bass     = hw.GetAdcValue(CV_6) - 0.25; //hw.adc.GetFloat(bassKnob) - 0.25;
	knob_color    = hw.GetAdcValue(CV_3);

	knob_fx_dry_wet  = hw.GetAdcValue(ADC_11);
	knob_fx_time     = hw.GetAdcValue(ADC_12);
	knob_fx_feedback = hw.GetAdcValue(CV_7);
	knob_fx_tone     = hw.GetAdcValue(CV_8);

	currentDelay = maxDelay * knob_fx_time;

	time_osc.SetFreq(3.0 * (1.0 - knob_fx_time) + 0.1);
	
	led_time_pwm.Set((time_osc.Process() + 0.6) * TIME_LED_BRIGHTNESS);
}

void setFilterConstants(float sample_rate){
	AmpLowPass.SetFrequency(2800);
	// AmpLowPass.SetRes(0.1);
	AmpLowPass.Init(); //sample_rate);
	// AmpLowPass.SetFilterMode(FilterMode);

	AmpBandPass.SetDrive(0.1);
	AmpBandPass.SetFreq(680);
	AmpBandPass.SetRes(0.1);
	AmpBandPass.Init(sample_rate);

	TrebleBandPass.SetFreq(3500);
	TrebleBandPass.SetDrive(0.2);
	TrebleBandPass.SetRes(0.2);
	TrebleBandPass.Init(sample_rate);

	BassLowPass.Init(sample_rate);
	BassLowPass.SetFreq(300);
	BassLowPass.SetRes(0.1);


	MidBandPass.SetFreq(1000);
	MidBandPass.SetDrive(0.2);
	MidBandPass.SetRes(0.3);
	MidBandPass.Init(sample_rate);

	Drive.SetDrive(0.2);
	Drive.Init();

	// rev.Init(sample_rate);
    dlyl.Init();
    dlyr.Init();

    //reverb parameters
    // rev.SetLpFreq(18000.0f);
    // rev.SetFeedback(0.85f);

    //delay parameters
    maxDelay = delayTarget = sample_rate * 0.75f;
	currentDelay = maxDelay;
    dlyl.SetDelay(currentDelay+ 10);
    dlyr.SetDelay(currentDelay+ 10);
}

void GetReverbSample(float &inl, float &inr)
{
    // rev.Process(inl, inr, &inl, &inr);
}

void GetDelaySample(float &inl, float &inr)
{
    fonepole(currentDelay, delayTarget, .00007f);
    dlyl.SetDelay(currentDelay);
    dlyr.SetDelay(currentDelay);
    float outl = dlyl.Read();
    float outr = dlyr.Read();
	

    dlyl.Write((knob_fx_feedback * outl) + inl);
    inl = outl + inl; //(knob_fx_feedback * outl) + ((1.0f - knob_fx_feedback) * inl);

    dlyr.Write((knob_fx_feedback * outr) + inr);
    inr = outr + inr; //(knob_fx_feedback * outr) + ((1.0f - knob_fx_feedback) * inr);
}