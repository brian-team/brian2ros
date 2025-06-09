File sound order :

1 : 2kHz_C.wav
2 : 2kHz_L.wav
3 : 2kHz_R.wav
4 : 2kHz_RtoL.wav
5 : 200Hz_C.wav
6 : 200Hz_L.wav
7 : 200Hz_R.wav
8 : 200Hz_RtoL.wav
9 : 500Hz_C.wav
10 : 500Hz_L.wav
11 : 500Hz_R.wav
12 : 500Hz_RtoL.wav
13 : 200Hz_C-20.wav
14 : 200Hz_L-20.wav
15 : 200Hz_R-20.wav
16 : WhN_C.wav
17 : WhN_L.wav
18 : WhN_R.wav
19 : WhN_RtoL.wav
20 : roomnoise.wav

Command for launch audio_record :

$ colcon build --packages-select turtleaudio
$ source install/setup.bash

if you want to use a file :

$ ros2 run turtleaudio audio_record roomnoise.wav

with portaudio (audio in real time) with 500 frames :

$ ros2 run turtleaudio audio_record portaudio 500

to use the sinus test with 500 frames :

$ ros2 run turtleaudio audio_record sin 500