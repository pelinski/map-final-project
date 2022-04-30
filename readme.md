download and compile aubio
```bash
git clone https://github.com/aubio/aubio
make
```

assuming you did this in `/root/aubio`, you can run the project using:

```bash
cd /root/Bela/ && make PROJECT=map-final-project CPPFLAGS=-I/root/aubio/src CFLAGS=-I/root/aubio/src LDFLAGS=-L/root/aubio/build/src LDLIBS=-laubio run
```