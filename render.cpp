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
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/math_neon/math_neon.h>
#include <vector>

// Delay Buffer
unsigned int gDelayBufferSize; // in samples
unsigned int gNumChannels = 2;
float gMaxDelayTime = 0.5; // in seconds
std::vector<float> gDelayBuffer[2];

// Pointers for the circular Buffer
unsigned int gWritePointer = 0; // stores the current signal (to be read later)
float gReadPointer = 0;		// float since it will be used for interpolation. read pointer retrieves the delayed signal

// Browser-based GUI to adjust parameters (parameters controlled from GUI start by p)
Gui gGui;
GuiController gGuiController;

// Delay parameters from GUI
float pDelayTime; // This is also sweepwidth
float pDelayDryMix;
float pDelayFeedbackLevel; // Level of feedback
// float pDelayLevelPre;	    // Level of pre-delay input (removed because it doesn't do much)
float pVibratoLFOFrequency; // LFO vibrato frequency

// Doppler effect parameters
// float gDelta; // Growth parameter, derivative of the time-varying delay
// float pSoundSpeed;
// float pSourceSpeed;   // Velocity with which the source moves towards the listener
// float pListenerSpeed; // Velocity with which the listener moves towards the source

// Helpers
float gPhase;
float gInverseSampleRate;

bool setup(BelaContext *context, void *userData) {

  // Initialise GUI sliders
  gGui.setup(context->projectName);
  gGuiController.setup(&gGui, "Controls");
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
  gGuiController.addSlider("Dry mix", 0.75, 0, 1, 0);
  gGuiController.addSlider("Feedback Level", 0.2, 0, 1, 0);
  // gGuiController.addSlider("Pre-delay level", 0.75, 0, 1, 0);
  gGuiController.addSlider("Vibrato LFO frequency in Hz", 10, 0, 20, 0);
  // gGuiController.addSlider("Sound speed in m/s", 340.0, 0, 500, 0);
  // gGuiController.addSlider("Source speed in m/s", 20, 0, 200, 0);
  // gGuiController.addSlider("Listener speed in m/s", 10, 0, 200, 0);

  // Precalc inverse sample rate
  gInverseSampleRate = 1.0 / context->audioSampleRate;

  // Allocate memory for the circular buffer (0.5s)
  gDelayBufferSize = gMaxDelayTime * context->audioSampleRate; // Initialised here since it depends on the sample rate
  for (unsigned int i = 0; i < gNumChannels; i++) {
    gDelayBuffer[i].resize(gDelayBufferSize);
  }

  return true;
}

void render(BelaContext *context, void *userData) {

  // Delay parameters from GUI
  pDelayTime = gGuiController.getSliderValue(0); // This is also sweepwidth
  pDelayDryMix = gGuiController.getSliderValue(1);
  pDelayFeedbackLevel = gGuiController.getSliderValue(2); // Level of feedback
  // pDelayLevelPre = gGuiController.getSliderValue(3);	   // Level of pre-delay input
  pVibratoLFOFrequency = gGuiController.getSliderValue(3); // LFO vibrato frequency
  // pSoundSpeed = gGuiController.getSliderValue(4);
  // pSourceSpeed = gGuiController.getSliderValue(5);
  // pListenerSpeed = gGuiController.getSliderValue(6);

  // Calculate the delay growth rate (derivative of the time-varying delay)
  // gDelta = -(pListenerSpeed + pSourceSpeed) / (pSourceSpeed - pSourceSpeed);

  // Fractionary Delay in samples. Subtract 3 samples to the delay pointer to make sure we have enough previous sampels to interpolate with
  float delayInSamples =
      (pDelayTime * context->audioSampleRate) * (0.5f + 0.1f * sinf_neon(gPhase * 2.0 * M_PI)); // scaled so that vibrato (sinf) is less intense
  gReadPointer = fmodf(((float)gWritePointer - (float)delayInSamples + (float)gDelayBufferSize - 3.0),
		       (float)gDelayBufferSize); // mod operator % only defined for integeers, so I use fmodf instead

  // ?? why allocate memory here instead of in setup???
  float in[gNumChannels];
  float out[gNumChannels];
  float interpolatedReadSample = 0.0;

  for (unsigned int n = 0; n < context->audioFrames; n++) {

    for (unsigned int i = 0; i < gNumChannels; i++) {

      // 1. Read input audio sample
      in[i] = audioRead(context, n, i);

      // 2.  Write sample that gets stored in the delay buffer: input sample (current input sample, in[i]) and the current delayed sample
      // (interpolatedReadSample) multiplied by the feedback level
      // if the feedback level is 0, that is, if the written sample into the buffer is equal to in, the delay line produces a single echo
      // (pDelayFeedbackLevel).
      // ???? IS THIS FLANGER???
      // gDelayBuffer[i][gWritePointer] = pDelayLevelPre * in[i] + pDelayFeedbackLevel * interpolatedReadSample;
      gDelayBuffer[i][gWritePointer] = in[i] + pDelayFeedbackLevel * interpolatedReadSample;

      // 3. Read delayed sample from the delay buffer
      // Lnear interpolation in case we have a varying delay over time (e.g. vibrato/chorus/flanger)
      // Use linear interpolation to read a fractional index into the buffer. (necessary in case the delay is very small and for vibratto) Find the
      // fraction by which the read pointer sits between two samples and use this to adjust weights of the samples
      float fraction = gReadPointer - floorf(gReadPointer);
      int previousSample = (int)floorf(gReadPointer);
      int nextSample = (previousSample + 1) % gDelayBufferSize;
      interpolatedReadSample = fraction * gDelayBuffer[i][nextSample] + (1.0 - fraction) * gDelayBuffer[i][previousSample]; // delayed sample

      // 4. Mix input signal with output signal
      // The output is the input plus the contents of the delay buffer (read with the readcursor) and weighted by the mix levels
      out[i] = pDelayDryMix * in[i] + (1 - pDelayDryMix) * interpolatedReadSample;
      // test bypass
      // out[i] = in[i];

      // 5. Write the output sample into the audio output
      audioWrite(context, n, i, out[i]);
    }

    // Increase pointer value. If the pointer is at the end of the buffer, wrap
    // it to 0 (circular buffer)
    gWritePointer = (gWritePointer + 1) % gDelayBufferSize; // is an integer so it will always wrap around strictly to 0
							    // gReadPointer =
    // fmodf(((float)gReadPointer + 1.0 - gDelta), (float)gDelayBufferSize); // read pointer is fractional so may wrap around a fractional value
    gReadPointer = fmodf(((float)gReadPointer + 1.0), (float)gDelayBufferSize);

    // Update the LFO phase, keeping it in the range 0-1
    gPhase += pVibratoLFOFrequency * gInverseSampleRate;
    if (gPhase > 1.0) {
      gPhase -= 1.0;
    }
  }
}

void cleanup(BelaContext *context, void *userData) {}

// TODO
// x github
// o file reading and store to file : not necessary since the project is rt
// x filter parameters
// x interpolation
// x vibratto
// _ doppler effect: not working
// _ multiple sources where each source produces its own doppler effect
