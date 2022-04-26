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

bool setup(BelaContext *context, void *userData) {

  // Initialise GUI sliders
  gGui.setup(context->projectName);
  gGuiController.addSlider("Delay in s", 0.1, 0, gMaxDelayTime, 0);
  gGuiController.addSlider("Wet mix (amount of undelayed singal in input)", 0.1, 0, 1, 0);
  gGuiController.addSlider("Feedback Level", 0.999, 0, 1, 0);
  gGuiController.addSlider("Pre-delay level", 0.75, 0, 1, 0);

  // Allocate memory for the circular buffer (0.5s)
  gDelayBufferSize = gMaxDelayTime * context->audioSampleRate; // Initialised here since it depends on the sample rate
  for (unsigned int i = 0; i < gNumChannels; i++) {
    gDelayBuffer[i].resize(gDelayBufferSize);
  }

  return true;
}

void render(BelaContext *context, void *userData) {

  // Delay parameters from GUI
  float pDelayTime = gGuiController.getSliderValue(0);
  float pDelayWetMix = gGuiController.getSliderValue(1);
  float pDelayFeedbackLevel = gGuiController.getSliderValue(2); // Level of feedback
  float pDelayLevelPre = gGuiController.getSliderValue(3);	// Level of pre-delay input

  int delayInSamples = pDelayTime * context->audioSampleRate;
  gReadPointer = (gWritePointer - delayInSamples + gDelayBufferSize) % gDelayBufferSize;

  for (unsigned int n = 0; n < context->audioFrames; n++) {
    float in[gNumChannels];
    float out[gNumChannels];

    //  Now we can write it into the delay buffer
    for (unsigned int i = 0; i < gNumChannels; i++) {

      in[i] = audioRead(context, n, i);

      // The output is the input plus the contents of the delay buffer (weighted by the mix levels)
      out[i] = (1 - pDelayWetMix) * in[i] + pDelayWetMix * gDelayBuffer[i][gReadPointer];

      // Calculate the sample that gets written into the delay buffer (sum 1. and 2.)
      // 1. Multiply the current (dry) sample (in) by the pre-delay level parameter (pDelayLevelPre). 
      // 2. Multiply the previously delayed sample from the buffer (gDelayBuffer[gReadPointer]) by the feedback level parameter (pDelayFeedbackLevel).
      gDelayBuffer[i][gWritePointer] = pDelayLevelPre * in[i] + pDelayFeedbackLevel * gDelayBuffer[i][gReadPointer];

      // Write the current sample in the audio output
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
// x filter parameters
// _ interpolation
// _ doppler effect
