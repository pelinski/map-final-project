# map-final-project
This is a real-time delay with vibrato and a tremolo synced to the beat of the track playing. You will need a Bela board, two audio adapter cables, an AUX cable to connect your input sound source and headphones or a loudspeaker to play the output. 

(disclosure: the beattracking does not work very well.......... and the sync either)

## quickstart
Clone this project in the `Bela/projects/` folder:
```bash
$ cd /root/Bela/projects
$ git clone https://github.com/pelinski/map-final-project/
```

I use the [aubio] (https://github.com/aubio/aubio) library to detect the beat of the track playing, so you will need to compile aubio in Bela in order to run this script. You can do so by cloning the repo in the `root/` folder:

```bash
$ cd /root/
$ git clone https://github.com/aubio/aubio
```
Once the repo is cloned, enter the folder contaning it and compile the library using the command `make`.
```bash
$ cd aubio/
$ make
```
Now you can run this project using the following command:
```bash
$ make -C /root/Bela PROJECT=map-final-project CPPFLAGS=-I/root/aubio/src CFLAGS=-I/root/aubio/src LDFLAGS=-L/root/aubio/build/src
LDLIBS=-laubio run
```
## sources

I used code from various sources:
+ For the aubio wrapper: https://github.com/giuliomoro/bela-game/blob/master/aubiopitch.c  and
https://github.com/giuliomoro/bela-game/blob/master/libpd-render.cpp
+ The Music and Audio Programming companion materials from Lecture 11: Circular buffers (circular-buffer), Lecture 14: ADSR (adsr-class) and Lecture 18: Phase vocoder part 1 (fft-overlap-add-threads).
+ The Bela example code in Audio/Delay.
+ The code for implementing a delay with interpolation and feedback in `Reiss, J. D., & McPherson, A. P. (2015). Chapter 2: Delay Line Effects. In Audio effects: Theory, implementation, and application. CRC Press, Taylor & Francis Group.`