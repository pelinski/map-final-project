/*
 ____  _____ _        _
| __ )| ____| |      / \
|  _ \|  _| | |     / _ \
| |_) | |___| |___ / ___ \
|____/|_____|_____/_/   \_\

http://bela.io

C++ Real-Time Audio Programming with Bela - Lecture 11: Circular buffers
circular-buffer: template code for implementing delays
*/

#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <vector>
#include "lib/MonoFilePlayer.h"

// Name of the sound file (in project folder)
std::string gFilename = "sample.wav";

// Object that handles playing sound from a buffer
MonoFilePlayer gPlayer;

// Browser-based GUI to adjust parameters
Gui gGui;
GuiController gGuiController;
// add sketch js

// TODO: declare variables for circular buffer
unsigned int gWritePointer = 0;
unsigned int gReadPointer = 0;

// assuming number of channels is 2
unsigned int gNumChannels = 2;
std::vector<float> gDelayBuffer[2]; 


// unsigned int gOffset = 0;

// Amount of delay
float gDelayAmount = 1.0;
// Amount of feedback
float gFB = 0.999; //gDelayFeedbackAmount
// Level of pre-delay input
float gDelayAmountPre = 0.75;
float gDryMix =0.2;

bool setup(BelaContext *context, void *userData)
{
	// Load the audio file
	if (!gPlayer.setup(gFilename))
	{
		rt_printf("Error loading audio file '%s'\n", gFilename.c_str());
		return false;
	}

	// Print some useful info
	rt_printf("Loaded the audio file '%s' with %d frames (%.1f seconds)\n",
			  gFilename.c_str(), gPlayer.size(),
			  gPlayer.size() / context->audioSampleRate);

	// guiController
	gGui.setup(context->projectName);
	gGuiController.addSlider("Delay in s", 0.1, 0, 0.49, 0);

	// Allocate memory for the circular buffer (0.5s)
	for (unsigned int i=0; i<gNumChannels; i++){
		gDelayBuffer[i].resize(0.5 * context->audioSampleRate);
	}

	return true;
}

void render(BelaContext *context, void *userData)
{
	unsigned int delayBufferSize = gDelayBuffer[0].size();

	// now delay is controlled in gui, read at every render iteration
	float delayInS = gGuiController.getSliderValue(0);
	int delayInSamples = delayInS * context->audioSampleRate;
	// gOffset = delayInS * context->audioSampleRate; // crop to int?
	gReadPointer = (gWritePointer - delayInSamples + delayBufferSize) % delayBufferSize;


	for (unsigned int n = 0; n < context->audioFrames; n++)
	{
		float in[gNumChannels];
		float out[gNumChannels];

        // Calculate the sample that will be written into the delay buffer...
        // 1. Multiply the current (dry) sample by the pre-delay gain level (set above)
        // 2. Get the previously delayed sample from the buffer, multiply it by the feedback gain and add it to the current sample


        //  Now we can write it into the delay buffer
		for (unsigned int i=0; i<gNumChannels; i++){
			in[i] = audioRead(context,n,i);

			gDelayBuffer[i][gWritePointer] = gDelayAmountPre * in[i] + gDelayBuffer[i][gReadPointer] * gFB;
			// Get the delayed sample (by reading `gDelayInSamples` many samples behind our current write pointer) and add it to our output sample
			out[i] = gDryMix*in[i] + gDelayBuffer[i][gReadPointer] * gDelayAmount;
			// Write the input and output to different channels
			audioWrite(context, n, 0, out[i]);
		}

		gWritePointer++;
		if (gWritePointer >= delayBufferSize)
		{
			gWritePointer = 0;
		}
		gReadPointer++;
		if (gReadPointer >= delayBufferSize)
		{
			gReadPointer = 0;
		}
	}
}

void cleanup(BelaContext *context, void *userData)
{
}

// TODO
// _ github
// _ file reading and store to file
// _ filter parameters
// _ interpolation
