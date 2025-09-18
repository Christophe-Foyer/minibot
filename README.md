# Mini bot source code

Building a mini 2 wheeled robot using the stack of vzense dcam710s I now own.

If you want to build one yourself, here is a [link to the CAD files](https://cad.onshape.com/documents/859567020c25581cd00fa078/w/ef91500aa8063739cecb03bb/e/cc4760a3813c1951d3898e3d?renderMode=0&uiState=68cabb8f8419a3e3d36768c4)

Media will be added eventually (assuming this project makes it to completion).

In terms of equipment, this uses a:
- raspberry pi - for now, using a pi zero to limit thermal load because I forgot vents in the initial design, but a pi with 64 bit support would be good for ROS.
- wavshare stepper motor hat
- two nema 17s
- an rgbd camera, here a vzense dcam710 but realistically a realsense d435i would be much nicer
- and since I'm not using a realsense with an IMU I'm also going to be adding a custom soldered hat with:
  - a 9 dof IMU (3 accel, 3 gyro, 3 magnetometer)
  - a 16 bit ADC with voltage divider to read the battery voltage
- Also using some random 2 pin magnet connectors from ebay to dock autonomously, we'll see how that goes
- if using beefier compute, and probably even if not, you probably want a 40mm fan
- a bunch of filament (maybe uses ~600-800g?), I suggest PETG as something not too exotic but that survives the sun.
- you probably want a usb GPS, I'll use a random one from ebay as well.
- either:
  - a battery with a voltage < 28v and charger
  - a battery and charger and a step down converter to step down the voltage to <28v for the waveshare hat (I used an old 36v hoverboard battery + charger)

Also please disregard the quality of this code, for now, this is a very rough prototype.
There's also some very vibe-coded stuff in here (specifically around the vzense stuff with arcane documentation), I'll properly review it before this makes it to actual deployment.
Also dubious LLM outputs aren't your only enemy when interfacing with obscure hardware, the API is also pretty rough around the edges.
My goal is to get my little robot going around.
I'll also post a more detailed BOM once it's working.

Enjoy.

## Notes

I recommend installing opencv using `pip install python3-opencv` so it doesn't take a thousand years, I actually disabled that dependency for older pis so it's very recommended.

Might need to hackishly use ffmpeg to read frames...
`ffmpeg -f v4l2 -input_format mjpeg -video_size 640x360 -i /dev/video0 -frames:v 1 test.jpg`
