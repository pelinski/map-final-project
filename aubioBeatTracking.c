// Using https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c as template, with the help of @adanlbenito

#include <Bela.h>
#include <aubio.h>
#include <mathutils.h>
#include <stdio.h>

aubio_tempo_t *oTempo; // Tempo tracking object
fvec_t *tempo_out;     // Tempo output
smpl_t isBeat = 0.;
uint_t isSilence = 0;

extern unsigned int gAudioSampleRate;
extern unsigned int gBeatTrackerBufferSize;
extern unsigned int gBeatTrackerHopSize;

float gOnsetThreshold = -20.0;
float gSilenceThreshold = -40.0;

double gLastBeatInS;
unsigned int gLastBeat;
float gBpm;
float gConfidence;

int aubio_beat_tracker_setup() {
  // from: https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1

  printf("buffer_size: %d, ", gBeatTrackerBufferSize);
  printf("hop_size: %d, ", gBeatTrackerHopSize);
  printf("onset_threshold: %f\n", gOnsetThreshold);

  oTempo = new_aubio_tempo("default", gBeatTrackerBufferSize, gBeatTrackerHopSize, gAudioSampleRate);

  if (oTempo == NULL)
    return 1;
  if (gOnsetThreshold != 0.0)
    aubio_tempo_set_threshold(oTempo, gOnsetThreshold);

  tempo_out = new_fvec(1); // Maybe 2??

  return 0;
}

void aubio_beat_tracker_cleanup() {
  del_aubio_tempo(oTempo);
  del_fvec(tempo_out);
}

// void process_block(fvec_t *ibuf, fvec_t *obuf) {
void process_block(fvec_t *ibuf) {
  // from:
  // 	https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1
  //	https://aubio.org/doc/latest/tempo_2test-tempo_8c-example.html#a6
  aubio_tempo_do(oTempo, ibuf, tempo_out);
  isBeat = fvec_get_sample(tempo_out, 0); // same as: isBeat = tempo_out->data[0]; ??
					  //   if (gSilenceThreshold != -90.0)
  isSilence = aubio_silence_detection(ibuf, gSilenceThreshold);
  //   fvec_zeros(obuf);
  //   if (isBeat && !isSilence) {

  //   } else {
  //   }

  // Features
  if (!isSilence && isBeat) {
    gLastBeatInS = aubio_tempo_get_last_s(oTempo);    // time of the latest beat detected in seconds
    gLastBeat = aubio_tempo_get_last(oTempo);	      // time of the latest beat detected in samples
    gBpm = aubio_tempo_get_bpm(oTempo);		      // current tempo
    gConfidence = aubio_tempo_get_confidence(oTempo); // current tempo confidence, 0 if no consistent value is foound, the higher the more confident

    printf("last s: %f\n", gLastBeatInS);
    printf("last: %i\n", gLastBeat);
    printf("bpm: %f\n", gBpm);
    printf("confidence: %f\n\n", gConfidence);
  }
}

void aubio_tempo_tracking_render(float *gBeatTrackerBuffer, int gBeatTrackerBufferSize) {
  // parse gBeatTrackerBuffer (array) into fvec_t (if gBeatTrackerBuffer is passed directly it results in Segmentation Fault)
  fvec_t ibuf = {.length = gBeatTrackerBufferSize, .data = gBeatTrackerBuffer};
  //   fvec_t obuf = {.length = gBeatTrackerBuffer.size(), , .data = context->audioOut};
  process_block(&ibuf);
}
