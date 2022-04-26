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
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <vector>

// Delay Buffer
unsigned int gDelayBufferSize; // in samples
unsigned int gNumChannels = 2;
float gMaxDelayTime = 0.5; // in seconds
std::vector<float> gDelayBuffer[2];

// Pointers for the circular Buffer
unsigned int gWritePointer = 0;
unsigned int gReadPointer = 0;

// Browser-based GUI to adjust parameters
Gui gGui;
GuiController gGuiController;

// Parameters
float gDelayAmount = 1.0;     // Amount of delay
float gFB = 0.999;            // gDelayFeedbackAmount // Amount of feedback
float gDelayAmountPre = 0.75; // Level of pre-delay input
float gDryMix = 0.2;

bool setup(BelaContext *context, void *userData) {
  // guiController Initialise
  gGui.setup(context->projectName);
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);

  // Allocate memory for the circular buffer (0.5s)
  gDelayBufferSize = gMaxDelayTime * context->audioSampleRate; // Initialised here since it depends on the sample rate
  for (unsigned int i = 0; i < gNumChannels; i++) {
    gDelayBuffer[i].resize(gDelayBufferSize);
  }

  return true;
}

void render(BelaContext *context, void *userData) {
  // now delay is controlled in gui, read at every render iteration
  float delayInS = gGuiController.getSliderValue(0);
  int delayInSamples = delayInS * context->audioSampleRate;
  gReadPointer = (gWritePointer - delayInSamples + gDelayBufferSize) % gDelayBufferSize;

  for (unsigned int n = 0; n < context->audioFrames; n++) {
    float in[gNumChannels];
    float out[gNumChannels];

    //  Now we can write it into the delay buffer
    for (unsigned int i = 0; i < gNumChannels; i++) {
      in[i] = audioRead(context, n, i);

      // Calculate the sample that will be written into the delay buffer...
      // 1. Multiply the current (dry) sample by the pre-delay gain level (set
      // above)
      // 2. Get the previously delayed sample from the buffer, multiply it by
      // the feedback gain and add it to the current sample
      gDelayBuffer[i][gWritePointer] = gDelayAmountPre * in[i] + gDelayBuffer[i][gReadPointer] * gFB;
      // Get the delayed sample (by reading `gDelayInSamples` many samples
      // behind our current write pointer) and add it to our output sample
      out[i] = gDryMix * in[i] + gDelayBuffer[i][gReadPointer] * gDelayAmount;
      // Write to output channels
      audioWrite(context, n, 0, out[i]);
    }

    // Increase pointer value. If the pointer is at the end of the buffer, wrap
    // it to 0 (circular buffer)
    gWritePointer = (gWritePointer + 1) % gDelayBufferSize;
    gReadPointer = (gReadPointer + 1) % gDelayBufferSize;
  }
}

void cleanup(BelaContext *context, void *userData) {}

// TODO
// x github
// o file reading and store to file : not necessary since the project is rt
// _ filter parameters
// _ interpolation
// _ doppler effect
