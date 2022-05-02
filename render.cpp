/*
Teresa Pelinski's Music and Audio Programming project.
This is a real-time delay with vibrato and a tremolo synced to the beat of the track playing.
You can copy the folder containing this code into /root/Bela/projects/
I use the aubio (https://github.com/aubio/aubio) library to detect the beat of the track playing, so you will need to compile aubio in Bela in order
to run this script. You can do so by cloning the repo and running the following command in the terminal: make. Once the compilation is finished, and
assuming you did that in /root/ (otherwise use the appropriate path), you can run this project writing the following command in the
terminal: make -C /root/Bela PROJECT=map-final-project CPPFLAGS=-I/root/aubio/src CFLAGS=-I/root/aubio/src LDFLAGS=-L/root/aubio/build/src
LDLIBS=-laubio run

I used code from various sources:
+ For the aubio wrapper: https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c  and
https://github.com/giuliomoro/bela-game/blob/master/libpd-render.cpp
+ The Music and Audio Programming companion materials from Lecture 11: Circular buffers (circular-buffer), Lecture 14: ADSR (adsr-class) and Lecture
18: Phase vocoder part 1 (fft-overlap-add-threads).
+ The Bela example code in Audio/Delay
+ The code for implementing a delay with interpolation and feedback in Reiss, J. D., & McPherson, A. P. (2015). Chapter 2: Delay Line Effects. In
Audio effects: Theory, implementation, and application. CRC Press, Taylor & Francis Group.

*/

#include "ADSR.h"
#include "Ramp.h"
#include <Bela.h>
#include <aubio.h>
#include <cmath> // floorf, fmodf
#include <libraries/Fft/Fft.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/math_neon/math_neon.h>
#include <vector>

// Aubio wrapper in aubioBeatTracking.c
extern "C" {
int aubio_beat_tracker_setup(float sampleRate);
fvec_t *aubio_tempo_tracking_render_bg(float *gBtAnalysisBuffer, int gBtAnalysisBufferSize);
};

const unsigned int gNumChannels = 2;

//__Delay
// Buffer
float gMaxDelayTime = 0.5;	    // in seconds
unsigned int gDelayBufferSize;	    // in samples (depends on the sample rate so it is defined in void setup())
std::vector<float> gDelayBuffer[2]; // buffer to hold samples for the delay
// Pointers for the circular delay buffer
unsigned int gDelayWritePointer = 0; // stores the current signal (to be read later)
float gDelayReadPointer = 0;	     // float since it will be used for interpolation. read pointer retrieves the delayed signal
// Delay parameters
float gPhase = 0;	    // LFO phase
float pDelayTime;	    // Delay time. This is also the width of the vibrato LFO
float pDelayDryMix;	    // The delay output is mixed with the original signal by this amount
float pDelayFeedbackLevel;  // Level of feedback (if 0, the delay is only one echo)
float pVibratoLFOFrequency; // LFO vibrato frequency

//__Browser-based GUI to adjust parameters (parameters controlled from  GUI start by p)
Gui gGui;
GuiController gGuiController;
int pByPass = 0; // Bypass switch to turn off the delay and tremolo

//__Beat tracking (BT)
AuxiliaryTask gBtTask;		   // Beat tracking task runs in an auxiliary thread
float pBtOnsetThreshold = -20.0;   // Onset threshold parameter for BT algorithm, controlled in GUI
float pBtSilenceThreshold = -30.0; //  Silence threshold parameter for BT algorithm, controlled in GUI s
// Beat tracker analysis buffer variables (used in aubioBeatTracking.c)
unsigned int gBtAnalysisBufferSize = 1024;
unsigned int gBtAnalysisHopSize = 1024;
// Buffers
const unsigned int gBtBufferSize = 16384;
std::vector<float> gBtInputBuffer[2];  // Buffer to pass to process_bt_background
std::vector<float> gBtOutputBuffer[2]; // Buffer with the output with added latency so that there is time to process the beattracking
std::vector<float>
    gBtBeatsBuffer; // Buffer with equal size to the output buffer (but only one channel) that indicates at each sample if there is a beat occurring
// Pointers and counters
int gBtInputBuffer_Pointer = 0;
int gBtInputBuffer_CachedPointer = 0;
int gBtOutputBuffer_WritePointer = 2 * gBtAnalysisHopSize; // Need one extra hop of latency to run in second thread
int gBtHopCounter = 0;
int gBtOutputBuffer_ReadPointer = 0;

//__ADSR envelope
ADSR gAmplitudeADSR;
// ADSR parameters from GUI
float pTremoloMix;
float pAmpAttackTime;
float pAmpDecayTime;
float pAmpSustainLevel;
float pAmpReleaseTime;

//__Helpers
float gInverseSampleRate;
unsigned int gAudioSampleRate; // save context->audioSampleRate here to pass it to the aubio thread

//__Function prototyping
std::vector<float> process_delay(std::vector<float> in);
std::vector<float> process_tremolo(std::vector<float> in);
void process_BT_background(void *);

bool setup(BelaContext *context, void *userData) {

	//__1. Initialise GUI sliders
	gGui.setup(context->projectName);
	gGuiController.setup(&gGui, "Controls");
	// Delay
	gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
	gGuiController.addSlider("Dry mix", 0.75, 0, 1, 0);
	gGuiController.addSlider("Feedback Level", 0.2, 0, 1, 0);
	gGuiController.addSlider("Vibrato LFO frequency in Hz", 0.7, 0, 20, 0);
	// Beat Tracking
	gGuiController.addSlider("Onset threshold", -50.0, -90.0, -0.0, 0);
	gGuiController.addSlider("Silence threshold", -40.0, -90.0, -0.0, 0);
	// ADSR
	gGuiController.addSlider("TremoloMix", 0.8, 0, 1, 0);
	gGuiController.addSlider("Amplitude Attack time", 0.01, 0.001, 0.1, 0);
	gGuiController.addSlider("Amplitude Decay time", 0.05, 0.01, 0.3, 0);
	gGuiController.addSlider("Amplitude Sustain level", 0.3, 0, 1, 0);
	gGuiController.addSlider("Amplitude Release time", 0.2, 0.001, 2, 0);
	// Bypass
	gGuiController.addSlider("Delay in s", 0, 0, 1, 1);

	//__2. Precalculate helpers
	gInverseSampleRate = 1.0 / context->audioSampleRate; // Precalc inverse sample rate
	gAudioSampleRate = context->audioSampleRate;	     // save audio sample rate to global varible
							     // so that it can be used in auxiliary task

	//__3. Allocate memory for the buffers
	gBtBeatsBuffer.resize(gBtBufferSize);

	// Resize buffers in each channel
	gDelayBufferSize = gMaxDelayTime * context->audioSampleRate;
	for (unsigned int i = 0; i < gNumChannels; i++) {
		gDelayBuffer[i].resize(gDelayBufferSize); // two channels so that it can be scaled to eg ping pong delays
		gBtInputBuffer[i].resize(gBtBufferSize);
		gBtOutputBuffer[i].resize(gBtBufferSize);
	}

	//__4. Initialise the ADSR object
	gAmplitudeADSR.setSampleRate(context->audioSampleRate);

	//__5. Set up the thread for the beat tracker
	gBtTask = Bela_createAuxiliaryTask(process_BT_background, 50, "bela-process-bt");

	//__6. Set up the beat tracking object
	int errorCode = aubio_beat_tracker_setup(context->audioSampleRate);
	if (errorCode)
		return false;

	return true;
}

void render(BelaContext *context, void *userData) {

	//__1. Get parameters from GUI
	// Delay parameters from GUI
	pDelayTime = gGuiController.getSliderValue(0);		 // Delay time. This is also the width of the vibrato LFO
	pDelayDryMix = gGuiController.getSliderValue(1);	 // The delay output is mixed with the original signal by this amount
	pDelayFeedbackLevel = gGuiController.getSliderValue(2);	 // Level of feedback (if 0, the delay is only one echo)
	pVibratoLFOFrequency = gGuiController.getSliderValue(3); // LFO vibrato frequency
	// Beat Tracking parameters from GUI
	pBtOnsetThreshold = gGuiController.getSliderValue(4);
	pBtSilenceThreshold = gGuiController.getSliderValue(5);
	// ADSR parameters from GUI
	pTremoloMix = gGuiController.getSliderValue(6);
	pAmpAttackTime = gGuiController.getSliderValue(7);
	pAmpDecayTime = gGuiController.getSliderValue(8);
	pAmpSustainLevel = gGuiController.getSliderValue(9);
	pAmpReleaseTime = gGuiController.getSliderValue(10);
	// Bypass
	pByPass = gGuiController.getSliderValue(10);

	//__2. Update ADSR envelope parameters
	gAmplitudeADSR.setAttackTime(pAmpAttackTime);
	gAmplitudeADSR.setDecayTime(pAmpDecayTime);
	gAmplitudeADSR.setSustainLevel(pAmpSustainLevel);
	gAmplitudeADSR.setReleaseTime(pAmpReleaseTime);

	//__3. Define input and output vectors (size: (2), one sample per channel)
	std::vector<float> in(gNumChannels);	   // input signal
	std::vector<float> outDelay(gNumChannels); // input signal after delay in
	std::vector<float> out(gNumChannels);	   // input signal after tremolo in

	// (in --[delay with vibrato and feedback]--> outDelay --[tremolo]--> outTremolo)

	for (unsigned int n = 0; n < context->audioFrames; n++) {
		//__4. Read input audio sample
		for (unsigned int i = 0; i < gNumChannels; i++) {
			in[i] = audioRead(context, n, i);
		}
		//__5. If bypass is off, pass first through delay block and then through tremolo block.
		if (!pByPass) {
			outDelay = process_delay(in);
			out = process_tremolo(outDelay);
		} else {
			out = in;
		}

		//__6. Write the output sample into the audio output .......why does it click in the left channel?
		for (unsigned int i = 0; i < gNumChannels; i++) {
			audioWrite(context, n, i, out[i]);
		};
	}
}

std::vector<float> process_delay(std::vector<float> in) {
	// Receives the 'in' vector with the current (i.e. at the current
	// timeframe in the render for loop) input samples (one per channel)
	// and returns the samples processed with the delay (one per channel)

	std::vector<float> out(2);

	// __1. Calculate the delay time in samples.
	// The delay time is pDelayTime (retrieved from GUI) multiplied by an LFO that creates the vibrato effect
	float delayInSamples =
	    (pDelayTime * gAudioSampleRate) * (0.5f + 0.1f * sinf_neon(gPhase * 2.0 * M_PI)); // scaled so that vibrato (sinf) is less intense

	// __2. Calculate the fractionary Delay in samples. Subtract 3 samples to the delay pointer to make sure we have enough previous
	// sampels to interpolate with. We need interpolation for very small delay times and variabtions
	gDelayReadPointer = fmodf(((float)gDelayWritePointer - (float)delayInSamples + (float)gDelayBufferSize - 3.0),
				  (float)gDelayBufferSize); // mod operator % only defined for integers, so I use fmodf instead

	float interpolatedReadSample = 0.0;
	for (unsigned int i = 0; i < in.size(); i++) {

		// __2.  Write sample that gets stored in the delay buffer: input sample (current input sample, in[i]) and the current delayed
		// sample (interpolatedReadSample) multiplied by the feedback level if the feedback level is 0, that is, if the written sample
		// into the buffer is equal to in, the delay line produces a single echo (pDelayFeedbackLevel).
		gDelayBuffer[i][gDelayWritePointer] = in[i] + pDelayFeedbackLevel * interpolatedReadSample;

		// __3. Read delayed sample from the delay buffer Lnear interpolation in case we have a varying delay over time (e.g.
		// vibrato/chorus/flanger) Use linear interpolation to read a fractional index into the buffer. (necessary in case the delay
		// is very small and for vibratto) Find the fraction by which the read pointer sits between two samples and use this to adjust
		// weights of the samples
		float fraction = gDelayReadPointer - floorf(gDelayReadPointer);
		int previousSample = (int)floorf(gDelayReadPointer);
		int nextSample = (previousSample + 1) % gDelayBufferSize;
		interpolatedReadSample =
		    fraction * gDelayBuffer[i][nextSample] + (1.0 - fraction) * gDelayBuffer[i][previousSample]; // delayed sample

		// __4. Mix input signal with output signal
		// The output is the input plus the contents of the delay buffer (read with the readcursor) and weighted by the mix levels
		out[i] = pDelayDryMix * in[i] + (1 - pDelayDryMix) * interpolatedReadSample;
	}

	// __5. Increase and wrap pointers
	gDelayWritePointer = (gDelayWritePointer + 1) % gDelayBufferSize; // is an integer so it will always wrap around
									  // strictly to 0
	gDelayReadPointer = fmodf(((float)gDelayReadPointer + 1.0), (float)gDelayBufferSize);

	//__6. Update the vibrato LFO phase, keeping it in the range 0-1
	gPhase += pVibratoLFOFrequency * gInverseSampleRate;
	if (gPhase > 1.0) {
		gPhase -= 1.0;
	}

	return out;
}
std::vector<float> process_tremolo(std::vector<float> in) {
	// Receives a vector with the current input samples (one per channel)
	// and returns the samples processed with the tremolo (one per channel)

	float readOut[2]; // delayed input signal by two times the hop size so that there is time to get the beat
	for (unsigned int i = 0; i < gNumChannels; i++) {
		//__1. Write to beat tracking input buffer
		// gInputBuffer holds the buffer that stores the input samples. Once filled, it gets passed to the process_BT_background
		// function to extract the beat.
		gBtInputBuffer[i][gBtInputBuffer_Pointer] = in[i];

		//__2. Get the current (at the read pointer) output sample from the output buffer.
		// We add extra latency (the output is delayed two times the hop size) to allow the beat to be detected in the second thread.
		// If the hop size is less than the FFT size, there will be overlapping windows in every sample (overlap-add), so we need to
		// scale the output to preserve the amplitude.
		readOut[i] = gBtOutputBuffer[i][gBtOutputBuffer_ReadPointer] * (float)gBtAnalysisHopSize / (float)gBtAnalysisBufferSize;

		//__3. Then clear the output sample in the buffer so it is ready for the next overlap-add
		gBtOutputBuffer[i][gBtOutputBuffer_ReadPointer] = 0;
	}

	//__4. Increment pointers and wrap them
	gBtInputBuffer_Pointer = (gBtInputBuffer_Pointer + 1) % gBtBufferSize; // Note this is not a condition for starting a new FFT.
	gBtOutputBuffer_ReadPointer = (gBtOutputBuffer_ReadPointer + 1) % gBtBufferSize;

	//__5. Increment the hop counter and start the BT thread and pass it the appropriate data (via global variables)
	gBtHopCounter = (gBtHopCounter + 1) % gBtAnalysisHopSize; // Increment the hop counter and start a new FFT if we've reached
	if (gBtHopCounter == 0) {
		gBtInputBuffer_CachedPointer =
		    gBtInputBuffer_Pointer;	     // Store the current input buffer pointer to keep track of the indexes in process_BT_background
		Bela_scheduleAuxiliaryTask(gBtTask); // runs process_BT_background in a different thread
	}

	//__6. Trigger ADSR envelope on Beat.
	// The BT task fills a gBtBeatsBuffer whose positions are equivalent to those in gOutputBuffer but contain the information of the
	// detected beats instead. If the beat is detected, trigger the ADSR envelope.
	if (gBtBeatsBuffer[gBtOutputBuffer_ReadPointer]) {
		gAmplitudeADSR.trigger();
	}

	//__7. Output signal is the dry delayed signal (readOut, see step 2) mixed with the same signal modulated with the tremolo ADSR
	// envelope, that
	// is triggered when there is a beat.
	std::vector<float> out(2);
	for (unsigned int i = 0; i < gNumChannels; i++) {
		out[i] = (1 - pTremoloMix) * readOut[i] + pTremoloMix * readOut[i] * gAmplitudeADSR.process();
	};

	return out;
}
void process_BT_background(void *) {
	// This function runs the beat tracking in an auxiliary task on Bela.

	// __1. Unwrap the gBtInputBuffer.
	//  Since we are using a circular buffer, earlier samples in the
	// buffer might not be the earlier samples in terms of time, so we unwrap the gBtInputBuffer so that
	// the position of the samples in the buffer unwrappedBuffer also corresponds to their order in time.
	static float unwrappedBuffer[2][1024];
	for (int n = 0; n < gBtAnalysisBufferSize; n++) {
		// Use modulo arithmetic to calculate the circular buffer index
		int circularBufferIndex = (gBtInputBuffer_CachedPointer + n - gBtAnalysisBufferSize + gBtBufferSize) % gBtBufferSize;
		for (unsigned int i = 0; i < gNumChannels; i++) {
			unwrappedBuffer[i][n] = gBtInputBuffer[i][circularBufferIndex];
		};
	}
	// __2. Do beat tracking in the unwrapped buffer (only on the left channel since the beats should be the same)
	// The gBeatsBuffer array is has the same size as gBtOutputBuffer but contains the result of the beat tracking
	// algorithm instead. This _gBeatsBuffer is an intermediate variable to parse the output of the function
	// aubio_tempo_tracking_render_bg, which is written in C and returns a pointer to a variable of type f_vec
	fvec_t *_gBeatsBuffer;
	_gBeatsBuffer = aubio_tempo_tracking_render_bg(unwrappedBuffer[0], gBtAnalysisBufferSize);

	// __3. Copy delayed samples to the output buffer
	for (int n = 0; n < gBtAnalysisBufferSize; n++) {
		int circularBufferIndex = (gBtOutputBuffer_WritePointer + n) % gBtBufferSize;
		for (unsigned int i = 0; i < gNumChannels; i++) {
			gBtOutputBuffer[i][circularBufferIndex] += unwrappedBuffer[i][n];
		};

		//__4. If there is a beat detected, set the beat flag in the gBeatsBuffer. !! Currently only doing it at the first sample of each
		//unwrappedBuffer, so the tremolo is not synced, TOFIX
		if (n == 0 && _gBeatsBuffer->data[0] != 0) {
			gBtBeatsBuffer[circularBufferIndex] = 1;
		} else {
			gBtBeatsBuffer[circularBufferIndex] = 0;
		}
	}

	// __5. Update the output buffer write pointer to start at the next hop
	gBtOutputBuffer_WritePointer = (gBtOutputBuffer_WritePointer + gBtAnalysisHopSize) % gBtBufferSize;
}

void cleanup(BelaContext *context, void *userData) {}