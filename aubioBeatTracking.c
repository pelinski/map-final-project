// Using https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c as template, with the help of @adanlbenito

#include <Bela.h>
#include <aubio.h>
#include <mathutils.h>
#include <stdio.h>

aubio_tempo_t *oTempo; // Tempo tracking object
fvec_t *out;	       // Tempo output

extern unsigned int gAudioSampleRate;
extern unsigned int gBtAnalysisBufferSize;
extern unsigned int gBtAnalysisHopSize;

// controlled in GUI
extern float pBtOnsetThreshold;
extern float pBtSilenceThreshold;

int aubio_beat_tracker_setup() {
	// from: https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1

	printf("buffer_size: %d, ", gBtAnalysisBufferSize);
	printf("hop_size: %d, ", gBtAnalysisHopSize);
	// printf("onset_threshold: %f\n", pBtOnsetThreshold);

	oTempo = new_aubio_tempo("default", gBtAnalysisBufferSize, gBtAnalysisHopSize, gAudioSampleRate);

	if (oTempo == NULL)
		return 1;

	aubio_tempo_set_threshold(oTempo, pBtOnsetThreshold);
	aubio_tempo_set_silence(oTempo, pBtSilenceThreshold);

	out = new_fvec(1); // Maybe 2??

	return 0;
}

void aubio_beat_tracker_cleanup() {
	del_aubio_tempo(oTempo);
	del_fvec(out);
}

fvec_t *process_block(fvec_t *in, fvec_t *out) {
	// from:
	// 	https://aubio.org/doc/latest/examples_2aubiotrack_8c-example.html#a1
	//	https://aubio.org/doc/latest/tempo_2test-tempo_8c-example.html#a6
	aubio_tempo_do(oTempo, in, out);

	if (out->data[0] != 0) {
		printf("beat at %.3fs, frame %d, %.2f bpm "
		       "with confidence %.2f, onset threshold %.1f\n",
		       aubio_tempo_get_last_s(oTempo), aubio_tempo_get_last(oTempo), aubio_tempo_get_bpm(oTempo), aubio_tempo_get_confidence(oTempo),
		       pBtOnsetThreshold);
	}
	return out;
}

// run as background task
fvec_t *aubio_tempo_tracking_render_bg(float *gBt_Buffer, int gBtAnalysisBufferSize) {

	aubio_tempo_set_threshold(oTempo, pBtOnsetThreshold);
	aubio_tempo_set_silence(oTempo, pBtSilenceThreshold);

	// parse gBt_Buffer (array) into fvec_t (if gBt_Buffer is passed directly it results in Segmentation Fault)
	fvec_t in = {.length = gBtAnalysisBufferSize, .data = gBt_Buffer};
	fvec_t out = {.length = gBtAnalysisBufferSize, .data = gBt_Buffer};

	fvec_t *_out = process_block(&in, &out);

	return _out;
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
