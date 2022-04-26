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
unsigned int gWritePointer = 0;
float gReadPointer = 0; // float since it will be used for interpolation

// Browser-based GUI to adjust parameters
Gui gGui;
GuiController gGuiController;

// Vibrato LFO parameters

// Helpers
float gPhase;
float gInverseSampleRate;

bool setup(BelaContext *context, void *userData) {

  // Initialise GUI sliders
  gGui.setup(context->projectName);
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
  gGuiController.addSlider("Wet mix (amount of undelayed singal in input)", 0.1, 0, 1, 0);
  gGuiController.addSlider("Feedback Level", 0.999, 0, 1, 0);
  gGuiController.addSlider("Pre-delay level", 0.75, 0, 1, 0);
  gGuiController.addSlider("Vibrato LFO frequency in Hz", 10, 0, 20, 0);

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
  float pDelayTime = gGuiController.getSliderValue(0); // This is also sweepwidth
  float pDelayWetMix = gGuiController.getSliderValue(1);
  float pDelayFeedbackLevel = gGuiController.getSliderValue(2);	 // Level of feedback
  float pDelayLevelPre = gGuiController.getSliderValue(3);	 // Level of pre-delay input
  float pVibratoLFOFrequency = gGuiController.getSliderValue(4); // LFO vibrato frequency

  // fractionary Delay in samples. Subtract 3 samples to the delay pointer to make sure we have enough previous sampels to interpolate with
  // TODO add LFO here
  float delayInSamples = (pDelayTime * context->audioSampleRate) * (0.5f + 0.5f * sinf_neon(gPhase * 2.0 * M_PI));
  gReadPointer =
      fmodf(((float)gWritePointer - (float)delayInSamples + (float)gDelayBufferSize - 3.0), (float)gDelayBufferSize); // % only defined for integeers

  float in[gNumChannels];
  float out[gNumChannels];
  float interpolatedReadPointer = 0.0;

  for (unsigned int n = 0; n < context->audioFrames; n++) {

    //  Now we can write it into the delay buffer
    for (unsigned int i = 0; i < gNumChannels; i++) {

      in[i] = audioRead(context, n, i);

      // Use linear interpolation to read a fractional index into the buffer. (necessary in case the delay is very small and for vibratto) Find the
      // fraction by which the read pointer sits between two samples and use this to adjust weights of the samples
      float fraction = gReadPointer - floorf(gReadPointer);
      int previousSample = (int)floorf(gReadPointer);
      int nextSample = (previousSample + 1) % gDelayBufferSize;
      interpolatedReadPointer = fraction * gDelayBuffer[i][nextSample] + (1.0 - fraction) * gDelayBuffer[i][previousSample];

      // The output is the input plus the contents of the delay buffer (weighted by the mix levels)
      out[i] = (1 - pDelayWetMix) * in[i] + pDelayWetMix * gDelayBuffer[i][interpolatedReadPointer];

      // Calculate the sample that gets written into the delay buffer (sum 1. and 2.)
      // 1. Multiply the current (dry) sample (in) by the pre-delay level parameter (pDelayLevelPre).
      // 2. Multiply the previously delayed sample from the buffer (gDelayBuffer[interpolatedReadPointer]) by the feedback level parameter
      // (pDelayFeedbackLevel).
      gDelayBuffer[i][gWritePointer] = pDelayLevelPre * in[i] + pDelayFeedbackLevel * gDelayBuffer[i][interpolatedReadPointer];

      // Write the current sample in the audio output
      audioWrite(context, n, 0, out[i]);
    }

    // Increase pointer value. If the pointer is at the end of the buffer, wrap
    // it to 0 (circular buffer)
    gWritePointer = (gWritePointer + 1) % gDelayBufferSize;
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
// _ doppler effect
