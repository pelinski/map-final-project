/*
 ____  _____ _        _
| __ )| ____| |      / \
|  _ \|  _| | |     / _ \
| |_) | |___| |___ / ___ \
|____/|_____|_____/_/   \_\

http://bela.io

Using:
C++ Real-Time Audio Programming with Bela - Lecture 11: Circular buffers
circular-buffer: template code for implementing delays
+ Code from the delay Bela Example
+ Audio Effects book
*/

#include <Bela.h>
#include <cmath> // floorf, fmodf
#include <libraries/Fft/Fft.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/math_neon/math_neon.h>
#include <vector>

#include <aubio.h>

// Delay Buffer
unsigned int gDelayBufferSize; // in samples
const unsigned int gNumChannels = 2;
float gMaxDelayTime = 0.5; // in seconds
std::vector<float> gDelayBuffer[2];

// Pointers for the circular Buffer
unsigned int gDelayWritePointer = 0; // stores the current signal (to be read later)
float gDelayReadPointer = 0;	     // float since it will be used for interpolation. read pointer retrieves the delayed signal

// Browser-based GUI to adjust parameters (parameters controlled from GUI start by p)
Gui gGui;
GuiController gGuiController;

// Delay parameters from GUI
float pDelayTime; // This is also sweepwidth
float pDelayDryMix;
float pDelayFeedbackLevel; // Level of feedback
// float pDelayLevelPre;	    // Level of pre-delay input (removed because it doesn't do much)
float pVibratoLFOFrequency; // LFO vibrato frequency

// Helpers
float gPhase = 0;
float gInverseSampleRate;
int gAudioSampleRate; // save context->audioSampleRate here to pass it to the aubio thread

// Beat tracking
std::vector<float> gBeatTrackerBuffer;
unsigned int gBeatTrackerBufferSize = 2048;
int gBeatTrackerBufferPointer = 0;
AuxiliaryTask gBeatTrackerTask;
int gCachedInputBufferPointer = 0;

/// fft sample code
// const int gFftSize = 1024; // FFT window size in samples
// int gHopSize = 256;	   // How often we calculate a window
// int gHopCounter = 0;
// // Circular buffer and pointer for assembling a window of samples
// const int gFFTBufferSize = 16384;
// int gFFTInputBufferPointer = 0;
// // Circular buffer for collecting the output of the overlap-add process
// int gFFTOutputBufferWritePointer = gFftSize + gHopSize;
// int gFFTOutputBufferReadPointer = 0;
// // Buffer to hold the windows for FFT analysis and synthesis
// std::vector<float> gAnalysisWindowBuffer;
// std::vector<float> gSynthesisWindowBuffer;
// Thread for FFT processing

std::vector<float> process_delay(std::vector<float> in);
void process_bt_background(void *);
void process_bt(std::vector<float> gBeatTrackerBuffer);

bool setup(BelaContext *context, void *userData) {

  // Initialise GUI sliders
  gGui.setup(context->projectName);
  gGuiController.setup(&gGui, "Controls");
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
  gGuiController.addSlider("Dry mix", 0.75, 0, 1, 0);
  gGuiController.addSlider("Feedback Level", 0.2, 0, 1, 0);
  gGuiController.addSlider("Vibrato LFO frequency in Hz", 10, 0, 20, 0);

  gInverseSampleRate = 1.0 / context->audioSampleRate; // Precalc inverse sample rate
  gAudioSampleRate = context->audioSampleRate;	       // ave audio sample rate to global varible so that it can be used in auxiliary task

  // Allocate memory for the circular buffers
  gDelayBufferSize = gMaxDelayTime * context->audioSampleRate; // Initialised here since it depends on the sample rate
  for (unsigned int i = 0; i < gNumChannels; i++) {
    gDelayBuffer[i].resize(gDelayBufferSize); // two channels so that it can be scaled to eg ping pong delays
  }
  gBeatTrackerBuffer.resize(gBeatTrackerBufferSize);

  // Calculate the window
  // gAnalysisWindowBuffer.resize(gFftSize);
  // for (int n = 0; n < gFftSize; n++) {
  //   // Hann window
  //   gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
  // }
  // recalculate_window(gFftSize);

  // Set up the thread for the beat tracker
  gBeatTrackerTask = Bela_createAuxiliaryTask(process_bt_background, 50, "bela-process-bt");

  return true;
}

void render(BelaContext *context, void *userData) {

  // Delay parameters from GUI
  pDelayTime = gGuiController.getSliderValue(0); // This is also sweepwidth
  pDelayDryMix = gGuiController.getSliderValue(1);
  pDelayFeedbackLevel = gGuiController.getSliderValue(2);  // Level of feedback
  pVibratoLFOFrequency = gGuiController.getSliderValue(3); // LFO vibrato frequency

  // Fractionary Delay in samples. Subtract 3 samples to the delay pointer to make sure we have enough previous sampels to interpolate with
  float delayInSamples =
      (pDelayTime * context->audioSampleRate) * (0.5f + 0.1f * sinf_neon(gPhase * 2.0 * M_PI)); // scaled so that vibrato (sinf) is less intense
  gDelayReadPointer = fmodf(((float)gDelayWritePointer - (float)delayInSamples + (float)gDelayBufferSize - 3.0),
			    (float)gDelayBufferSize); // mod operator % only defined for integeers, so I use fmodf instead

  // (in --[delay with vibrato and feedback]--> outDelay --[tremolo]--> out)
  std::vector<float> in(gNumChannels);	     // input signal
  std::vector<float> outDelay(gNumChannels); // input signal after delay in
  std::vector<float> out(gNumChannels);	     // final signal after delay and tremolo

  for (unsigned int n = 0; n < context->audioFrames; n++) {
    // 1. Read input audio sample
    for (unsigned int i = 0; i < gNumChannels; i++) {
      in[i] = audioRead(context, n, i);
    }
    // 2. Pass through delay block
    outDelay = process_delay(in);

    // 3. Store in gBeatTrackerBuffer
    gBeatTrackerBuffer[gBeatTrackerBufferPointer] = outDelay[0]; // we only do BeatTracker on the left channel
    // 4. Increase and wrap the pointer
    gBeatTrackerBufferPointer = (gBeatTrackerBufferPointer + 1) % gBeatTrackerBufferSize;
    // 5. If we filled the BTBuffer, run BeatTracker
    if (gBeatTrackerBufferPointer == 0) {
      // Run the beat tracking task
      Bela_scheduleAuxiliaryTask(gBeatTrackerTask);
    }

    // // TODO beat tracking gives back a signal that changes the frequency of the LFO?

    //// fft sample code
    // // Store the sample ("in") in a buffer for the FFT
    // // Increment the pointer and when full window has been
    // // assembled, call process_bt()
    // gBeatTrackerBuffer[gFFTInputBufferPointer] = outDelay[i];

    // // Get the output sample from the output buffer
    // float out = gFFTOutputBuffer[i][gFFTOutputBufferReadPointer];

    // // Then clear the output sample in the buffer so it is ready for the next overlap-add
    // gFFTOutputBuffer[i][gFFTOutputBufferReadPointer] = 0;

    // // Scale the output down by the overlap factor (e.g. how many windows overlap per sample?)
    // out *= (float)gHopSize / (float)gFftSize;

    // // Increment the hop counter and start a new FFT if we've reached the hop size
    // // ????? this happens twice with the channels?
    // if (gHopCounter + 1 >= gHopSize) {
    //   gCachedInputBufferPointer = gFFTInputBufferPointer;
    //   Bela_scheduleAuxiliaryTask(gBeatTrackerTask);
    // }

    for (unsigned int i = 0; i < gNumChannels; i++) {
      // 5. Write the output sample into the audio output
      audioWrite(context, n, i, outDelay[i]);
    };

    // fft sample code
    // Increase and wrap pointers
    // gHopCounter = (gHopCounter + 1) % gHopSize;
    // gFFTOutputBufferReadPointer = (gFFTOutputBufferReadPointer + 1) % gFFTBufferSize;
    // gFFTInputBufferPointer = (gFFTInputBufferPointer + 1) % gFFTBufferSize;
  }
}

void cleanup(BelaContext *context, void *userData) {}

std::vector<float> process_delay(std::vector<float> in) {
  // receives the 'in' vector with the current (i.e. at the current timeframe in the render for loop) input samples (one per channel) and returns the
  // samples processed with the delay (one per channel)
  std::vector<float> out(2);
  float interpolatedReadSample = 0.0;

  for (unsigned int i = 0; i < in.size(); i++) {

    // 1.  Write sample that gets stored in the delay buffer: input sample (current input sample, in[i]) and the current delayed sample
    // (interpolatedReadSample) multiplied by the feedback level
    // if the feedback level is 0, that is, if the written sample into the buffer is equal to in, the delay line produces a single echo
    // (pDelayFeedbackLevel).
    gDelayBuffer[i][gDelayWritePointer] = in[i] + pDelayFeedbackLevel * interpolatedReadSample;

    // 2. Read delayed sample from the delay buffer
    // Lnear interpolation in case we have a varying delay over time (e.g. vibrato/chorus/flanger)
    // Use linear interpolation to read a fractional index into the buffer. (necessary in case the delay is very small and for vibratto) Find the
    // fraction by which the read pointer sits between two samples and use this to adjust weights of the samples
    float fraction = gDelayReadPointer - floorf(gDelayReadPointer);
    int previousSample = (int)floorf(gDelayReadPointer);
    int nextSample = (previousSample + 1) % gDelayBufferSize;
    interpolatedReadSample = fraction * gDelayBuffer[i][nextSample] + (1.0 - fraction) * gDelayBuffer[i][previousSample]; // delayed sample

    // 3. Mix input signal with output signal
    // The output is the input plus the contents of the delay buffer (read with the readcursor) and weighted by the mix levels
    out[i] = pDelayDryMix * in[i] + (1 - pDelayDryMix) * interpolatedReadSample;
  }

  // Increase and wrap pointers
  gDelayWritePointer = (gDelayWritePointer + 1) % gDelayBufferSize; // is an integer so it will always wrap around strictly to 0
  gDelayReadPointer = fmodf(((float)gDelayReadPointer + 1.0), (float)gDelayBufferSize);

  // Update the vibrato LFO phase, keeping it in the range 0-1
  gPhase += pVibratoLFOFrequency * gInverseSampleRate;
  if (gPhase > 1.0) {
    gPhase -= 1.0;
  }

  return out;
}

void process_bt(std::vector<float> const gBeatTrackerBuffer) {

  // get the beat from the buffer

  // do the tremolo effect on the beat

  //////// fft sample code
  // static std::vector<float> unwrappedBuffer(gFftSize); // Container to hold the unwrapped values

  // // Copy buffer into FFT input
  // for (int n = 0; n < gFftSize; n++) {
  //   // Use modulo arithmetic to calculate the circular buffer index
  //   int circularBufferIndex = (inPointer + n - gFftSize + gFFTBufferSize) % gFFTBufferSize;
  //   unwrappedBuffer[n] = inBuffer[circularBufferIndex] * gAnalysisWindowBuffer[n];
  // }

  // // Process the FFT based on the time domain input
  // gFft.fft(unwrappedBuffer);

  // // Robotise the output
  // for (int n = 0; n < gFftSize; n++) {
  //   float amplitude = gFft.fda(n);
  //   gFft.fdr(n) = amplitude;
  //   gFft.fdi(n) = 0;
  // }

  // // Run the inverse FFT
  // gFft.ifft();

  // // Add timeDomainOut into the output buffer
  // for (int n = 0; n < gFftSize; n++) {
  //   int circularBufferIndex = (outPointer + n - gFftSize + gFFTBufferSize) % gFFTBufferSize;
  //   outBuffer[circularBufferIndex] += gFft.td(n) * gSynthesisWindowBuffer[n];
  // }
}

// This function runs in an auxiliary task on Bela, calling process_bt
void process_bt_background(void *) {
  process_bt(gBeatTrackerBuffer);

  // Update the output buffer write pointer to start at the next hop
  // gFFTOutputBufferWritePointer = (gFFTOutputBufferWritePointer + gHopSize) % gFFTBufferSize;
}
