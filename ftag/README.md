# overview
* ftag binary captures via v4l
* 2D image coordinates,camera calibration data and tag properties are used to calculate tag position in camera coordinate system
* World position of camera is calculated from at least 4 visible tags.
* Tag coordinates are transformed to world coordinates and transmitted via tcp
* captured images are piped to ffmpeg for video streaming

# install packages
```
sudo apt install build-essential
sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libtbb-dev libjpeg-dev libpng-dev libtiff-dev
```

# download opencv
https://github.com/opencv/opencv/releases/tag/3.2.0

# build opencv
```
~/$ unzip 3.2.0.zip
opencv-3.2.0/$ mkdir build; cd build;
build/$ cmake -D WITH_LIBV4L=ON -D WITH_IPP=ON -D CMAKE_BUILD_TYPE=Debug ..
build/$ make -j4
build/$ sudo make install
```

# build ftag
```
$ git clone https://github.com/faljse/ftag
$ cd ftag
$ cmake .
$ make
```

# Camera calibration
```
./utils_calibration/aruco_calibration
Usage: (in.avi|live) out_camera_calibration.yml  [-m markermapConfig.yml (configuration of the board. If use default one (in utils), no need to set this)]    [-size <float> :(value in meters of a marker. If you provide a board that contains that information, this is ommited) ]
```

# ftag configuration
```yml
%YAML:1.0
posServer: "127.0.0.1:1337"
videoDevice: "/dev/video2"
# output of aruco_calibration
camParamsFile: "cdata/logitech_c920.yml" 
markerSize: 0.034
headless: 1

ffmpegCmd: "ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1024x576 -framerate 30 -i pipe:0 -f mpegts -c:v mpeg1video -c:a none -b:v 1000k -bf 0 http://localhost:8080/stream/input/3"

pointsScale: 0.137
points:
  - {id: 161, x:0, y:0, z:0}
  - {id: 227, x:0.5, y:0, z:0}
...
```
