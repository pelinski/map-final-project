// Using https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c as template, with the help of @adanlbenito

#include <Bela.h>
#include <aubio.h>
#include <mathutils.h>
#include <stdio.h>

aubio_tempo_t *tt; // Tempo tracking object
fvec_t *tempo_out; // Tempo output
smpl_t is_beat = 0.;
uint_t is_silence = 0;

char_t *tempo_method = "default";
uint_t buffer_size = 1024;
uint_t hop_size = 512;
smpl_t onset_threshold = -1000.0;
smpl_t silence_threshold = -90.0;


float samplerate;

float gTTLastMs;
float gTTLastS;
unsigned int gTTLast;
float gTTbpm;
float gTTconfidence;

int aubio_beat_tracker_setup(float fs) {
  // from: https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1
  samplerate = fs;

  printf("tempo method: %s, ", tempo_method);
  printf("buffer_size: %d, ", buffer_size);
  printf("hop_size: %d, ", hop_size);
  printf("onset_threshold: %f\n", onset_threshold);

  tt = new_aubio_tempo(tempo_method, buffer_size, hop_size, samplerate);

  if (tt == NULL)
    return 1;
  if (onset_threshold != 0.0)
    aubio_tempo_set_threshold(tt, onset_threshold);

  tempo_out = new_fvec(1); // Maybe 2??

  return 0;
}

void aubio_beat_tracker_cleanup() {
  del_aubio_tempo(tt);
  del_fvec(tempo_out);
}

void process_block(fvec_t *ibuf, fvec_t *obuf) {
  // from:
  // 	https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1
  //	https://aubio.org/doc/latest/tempo_2test-tempo_8c-example.html#a6
  aubio_tempo_do(tt, ibuf, tempo_out);
  is_beat = fvec_get_sample(tempo_out, 0); // smame as: is_beat = tempo_out->data[0]; ??
  if (silence_threshold != -90.0)
    is_silence = aubio_silence_detection(ibuf, silence_threshold);
  fvec_zeros(obuf);
  if (is_beat && !is_silence) {

  } else {
  }

  // Features
  if (is_beat) {
    gTTLastMs = aubio_tempo_get_last_ms(tt);
    gTTLastS = aubio_tempo_get_last_s(tt);
    gTTLast = aubio_tempo_get_last(tt);
    gTTbpm = aubio_tempo_get_bpm(tt);
    gTTconfidence = aubio_tempo_get_confidence(tt);
  }
}

void aubio_tempo_tracking_render(BelaContext *context, void *userData, unsigned int ch) {
  smpl_t audioIn[context->audioFrames];
  for (unsigned int n = 0; n < context->audioFrames; ++n) {
    audioIn[n] = audioRead(context, n, ch);
  }
  fvec_t ibuf = {.length = context->audioFrames, .data = audioIn};
  fvec_t obuf = {.length = context->audioFrames, .data = context->audioOut};
  process_block(&ibuf, &obuf);
}
