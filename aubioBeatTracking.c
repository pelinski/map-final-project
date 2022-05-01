// Using https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c as template, with the help of @adanlbenito

#include <Bela.h>
#include <aubio.h>
#include <mathutils.h>
#include <stdio.h>

aubio_tempo_t *oTempo; // Tempo tracking object
fvec_t *out;	       // Tempo output
smpl_t isBeat = 0.;
uint_t isSilence = 0;

extern unsigned int gAudioSampleRate;
unsigned int _gBT_BufferSize = 1024;
extern unsigned int gBT_AnalysisHopSize;

extern float pBT_OnsetThreshold;
extern float pBT_SilenceThreshold;

double gLastBeatInS;
unsigned int gLastBeat;
float gBpm;
float gConfidence;

int aubio_beat_tracker_setup() {
  // from: https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1

  printf("buffer_size: %d, ", _gBT_BufferSize);
  printf("hop_size: %d, ", gBT_AnalysisHopSize);
  // printf("onset_threshold: %f\n", pBT_OnsetThreshold);

  oTempo = new_aubio_tempo("default", _gBT_BufferSize, gBT_AnalysisHopSize, gAudioSampleRate);

  if (oTempo == NULL)
    return 1;

  aubio_tempo_set_threshold(oTempo, pBT_OnsetThreshold);
  aubio_tempo_set_silence(oTempo, pBT_SilenceThreshold);

  out = new_fvec(1); // Maybe 2??

  return 0;
}

void aubio_beat_tracker_cleanup() {
  del_aubio_tempo(oTempo);
  del_fvec(out);
}

// void process_block(fvec_t *in, fvec_t *obuf) {
void process_block(fvec_t *in, fvec_t *out) {
  // from:
  // 	https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1
  //	https://aubio.org/doc/latest/tempo_2test-tempo_8c-example.html#a6
  aubio_tempo_do(oTempo, in, out);
  // isBeat = fvec_get_sample(out, 0); // same as: isBeat = out->data[0]; ??
  // 			    //   if (pBT_SilenceThreshold != -90.0)
  // isSilence = aubio_silence_detection(in, pBT_SilenceThreshold);
  //   fvec_zeros(obuf);
  //   if (isBeat && !isSilence) {

  //   } else {
  //   }

  // // Features
  // if (!isSilence && isBeat) {
  //   gLastBeatInS = aubio_tempo_get_last_s(oTempo);    // time of the latest beat detected in seconds
  //   gLastBeat = aubio_tempo_get_last(oTempo);	      // time of the latest beat detected in samples
  //   gBpm = aubio_tempo_get_bpm(oTempo);		      // current tempo
  //   gConfidence = aubio_tempo_get_confidence(oTempo); // current tempo confidence, 0 if no consistent value is foound, the higher the more
  //   confident

  //   printf("last s: %f\n", gLastBeatInS);
  //   printf("last: %i\n", gLastBeat);
  //   printf("bpm: %f\n", gBpm);
  //   printf("confidence: %f\n\n", gConfidence);
  // }
  if (out->data[0] != 0) {
    printf("beat at %.3fs, frame %d, %.2f bpm "
	   "with confidence %.2f, onset threshold %.1f\n",
	   aubio_tempo_get_last_s(oTempo), aubio_tempo_get_last(oTempo), aubio_tempo_get_bpm(oTempo), aubio_tempo_get_confidence(oTempo),
	   pBT_OnsetThreshold);
  }
}

// run as background task
fvec_t aubio_tempo_tracking_render_bg(float *gBT_Buffer, int _gBT_BufferSize) {

  aubio_tempo_set_threshold(oTempo, pBT_OnsetThreshold);
  aubio_tempo_set_silence(oTempo, pBT_SilenceThreshold);

  // parse gBT_Buffer (array) into fvec_t (if gBT_Buffer is passed directly it results in Segmentation Fault)
  fvec_t in = {.length = _gBT_BufferSize, .data = gBT_Buffer};
  fvec_t out = {.length = _gBT_BufferSize, .data = gBT_Buffer};

  process_block(&in, &out);

  return out;
}

// run in render using only the audioframes in current audio buffer
// void aubio_tempo_tracking_render(BelaContext *context) {
//   smpl_t audioIn[context->audioFrames];
//   for (unsigned int n = 0; n < context->audioFrames; ++n) {
//     audioIn[n] = audioRead(context, n, 0);
//   }
//   fvec_t in = {.length = context->audioFrames, .data = audioIn};
//   process_block(&in);
// }
