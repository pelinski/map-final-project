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
extern float gTTLastMs;
extern float gTTLastS;
extern unsigned int gTTLast;
extern float gTTbpm;
extern float gTTconfidence;

extern "C" {
int aubio_beat_tracker_setup(float sampleRate);
void process_block(fvec_t *ibuf, fvec_t *obuf);
void aubio_tempo_tracking_render_bg(float *gBT_AnalysisBuffer, int gBT_AnalysisBufferSize);
};

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
float pDelayFeedbackLevel;  // Level of feedback
float pVibratoLFOFrequency; // LFO vibrato frequency

// Helpers
float gPhase = 0;
float gInverseSampleRate;
unsigned int gAudioSampleRate; // save context->audioSampleRate here to pass it to the aubio thread

// Beat tracking
AuxiliaryTask gBT_Task;
// Beat tracker analysis buffer variables (used in aubioBeatTracking.c)
unsigned int gBT_AnalysisBufferSize = 2048;
unsigned int gBT_AnalysisHopSize = 512;
// float gBT_AnalysisBuffer[1024]; // not a std::vector so that we can pass it to aubio_tempo_tracking_render (C function)
int gBT_HopCounter = 0;

std::vector<float> gBT_Window;

const unsigned int gBT_BufferSize = 16384;
std::vector<float> gBT_InputBuffer(gBT_BufferSize);
std::vector<float> gBT_OutputBuffer(gBT_BufferSize);

int gBT_InputBuffer_Pointer = 0;
int gBT_InputBuffer_CachedPointer = 0;
int gBT_OutputBuffer_WritePointer = 2 * gBT_AnalysisHopSize; // Need one extra hop of latency to run in second thread
int gBT_OutputBuffer_ReadPointer = 0;

std::vector<float> process_delay(std::vector<float> in);
void process_BT_background(void *);

bool setup(BelaContext *context, void *userData) {

  // Initialise GUI sliders
  gGui.setup(context->projectName);
  gGuiController.setup(&gGui, "Controls");
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
  gGuiController.addSlider("Dry mix", 0.75, 0, 1, 0);
  gGuiController.addSlider("Feedback Level", 0.2, 0, 1, 0);
  gGuiController.addSlider("Vibrato LFO frequency in Hz", 10, 0, 20, 0);

  // Helpers
  gInverseSampleRate = 1.0 / context->audioSampleRate; // Precalc inverse sample rate
  gAudioSampleRate = context->audioSampleRate;	       // save audio sample rate to global varible so that it can be used in auxiliary task

  // Allocate memory for the circular buffers
  gDelayBufferSize = gMaxDelayTime * context->audioSampleRate; // Initialised here since it depends on the sample rate
  for (unsigned int i = 0; i < gNumChannels; i++) {
    gDelayBuffer[i].resize(gDelayBufferSize); // two channels so that it can be scaled to eg ping pong delays
  }

  // gBT_AnalysisBuffer.resize(gBT_AnalysisBufferSize);
  // Set up the thread for the beat tracker
  gBT_Task = Bela_createAuxiliaryTask(process_BT_background, 50, "bela-process-bt");
  // setup audio_beat_tracker
  int errorCode = aubio_beat_tracker_setup(context->audioSampleRate);
  if (errorCode)
    return false;

  // // Calculate the window for BT
  // gBT_Window.resize(gBT_AnalysisBufferSize);
  // for (int n = 0; n < gBT_AnalysisBufferSize; n++) {
  //   // Hann window
  //   gBT_Window[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gBT_AnalysisBufferSize - 1)));
  // }

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

    gBT_InputBuffer[gBT_InputBuffer_Pointer] = outDelay[0]; // write to input buffer
    gBT_InputBuffer_Pointer =
	(gBT_InputBuffer_Pointer + 1) % gBT_BufferSize; // Wrap input buffer pointer. Note this is not a condition for starting a new FFT.

    // Get the output sample from the output buffer
    float out = gBT_OutputBuffer[gBT_OutputBuffer_ReadPointer];

    // Then clear the output sample in the buffer so it is ready for the next overlap-add
    gBT_OutputBuffer[gBT_OutputBuffer_ReadPointer] = 0;

    // Scale the output down by the overlap factor (e.g. how many windows overlap per sample?)
    out *= (float)gBT_AnalysisHopSize / (float)gBT_AnalysisBufferSize;

    // Increment the read pointer in the output cicular buffer
    gBT_OutputBuffer_ReadPointer = (gBT_OutputBuffer_ReadPointer + 1) % gBT_BufferSize;

    // Increment the hop counter and start a new FFT if we've reached the hop size
    if (++gBT_HopCounter >= gBT_AnalysisHopSize) {
      gBT_HopCounter = 0;

      // TODO: start the FFT thread and pass it the appropriate
      //       data (via global variables)
      gBT_InputBuffer_CachedPointer = gBT_InputBuffer_Pointer;
      Bela_scheduleAuxiliaryTask(gBT_Task);
    }
    //
    for (unsigned int i = 0; i < gNumChannels; i++) {
      // 5. Write the output sample into the audio output
      audioWrite(context, n, i, out);
    };

    // fft sample code
    // Increase and wrap pointers
    // gBT_HopCounter = (gBT_HopCounter + 1) % gBT_AnalysisHopSize;
    // gBT_OutputBuffer_ReadPointer = (gBT_OutputBuffer_ReadPointer + 1) % gFFTBufferSize;
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

void process_BT_background(void *) {
  // This function runs in an auxiliary task on Bela

  // unwrap beat tracker buffer
  static float unwrappedBuffer[2048]; // Container to hold the unwrapped values

  // Copy buffer into FFT input, starting one window ago
  for (int n = 0; n < gBT_AnalysisBufferSize; n++) {
    // Use modulo arithmetic to calculate the circular buffer index
    int circularBufferIndex = (gBT_InputBuffer_CachedPointer + n - gBT_AnalysisBufferSize + gBT_BufferSize) % gBT_BufferSize;
    unwrappedBuffer[n] = gBT_InputBuffer[circularBufferIndex];
  }
  // do something
  aubio_tempo_tracking_render_bg(unwrappedBuffer, gBT_AnalysisBufferSize);

  for (int n = 0; n < gBT_AnalysisBufferSize; n++) {
    int circularBufferIndex = (gBT_OutputBuffer_WritePointer + n) % gBT_BufferSize;
    gBT_OutputBuffer[circularBufferIndex] += unwrappedBuffer[n];
  }

  // Update the output buffer write pointer to start at the next hop
  gBT_OutputBuffer_WritePointer = (gBT_OutputBuffer_WritePointer + gBT_AnalysisHopSize) % gBT_BufferSize;
}
