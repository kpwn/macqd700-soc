# FPGA HDMI Capture

The FPGA HDMI output can be watched and sampled through a USB UVC HDMI capture
dongle with:

```sh
make fpga-video-list
make fpga-video-view
make fpga-video-still
```

The helper is [tools/fpga_video_capture.sh](../tools/fpga_video_capture.sh).  It
defaults to `/dev/video0`, `1280x720`, `60 fps`, and MJPEG, which matches common
MacroSilicon USB HDMI capture adapters.  Override those defaults either with
flags:

```sh
tools/fpga_video_capture.sh view --device /dev/video0 --size 1920x1080 --fps 30
tools/fpga_video_capture.sh still --size 1280x720 captures/current.png
```

or with environment variables:

```sh
FPGA_VIDEO_DEV=/dev/video0 FPGA_VIDEO_WIDTH=1920 FPGA_VIDEO_HEIGHT=1080 FPGA_VIDEO_FPS=30 make fpga-video-view
```

`view` prefers VLC when it is installed, then `ffplay`, then `mpv`, and falls
back to GStreamer.  This machine currently has GStreamer available, so the
fallback path is enough for live preview.  Installing VLC is optional:

```sh
sudo apt install vlc
```

For still captures, installing ffmpeg is also optional.  Without it the helper
uses GStreamer to write PNG files under `captures/`.

```sh
sudo apt install ffmpeg
```

If the preview is blank or negotiation fails, inspect modes with:

```sh
tools/fpga_video_capture.sh list
```

Then retry with the advertised size, frame rate, or raw mode:

```sh
tools/fpga_video_capture.sh view --format raw --size 640x480 --fps 60
```

On Linux, V4L2 capture usually requires membership in the `video` group.  If
the tool reports no access to `/dev/video0`, run:

```sh
sudo usermod -aG video "$USER"
```

Then log out and back in so the new group membership applies.
