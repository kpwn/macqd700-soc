#!/usr/bin/env bash
set -euo pipefail

device="${FPGA_VIDEO_DEV:-/dev/video0}"
width="${FPGA_VIDEO_WIDTH:-1280}"
height="${FPGA_VIDEO_HEIGHT:-720}"
fps="${FPGA_VIDEO_FPS:-60}"
format="${FPGA_VIDEO_FORMAT:-mjpeg}"
out_dir="${FPGA_VIDEO_OUT_DIR:-captures}"
sink="${FPGA_VIDEO_SINK:-autovideosink}"

usage() {
    cat <<'USAGE'
Usage:
  tools/fpga_video_capture.sh list
  tools/fpga_video_capture.sh view [options]
  tools/fpga_video_capture.sh still [options] [output.png]

Options:
  -d, --device DEV       V4L2 capture node (default: /dev/video0 or FPGA_VIDEO_DEV)
  -s, --size WxH         Capture size (default: 1280x720)
  -r, --fps FPS          Capture rate (default: 60)
  -f, --format FORMAT    mjpeg or raw (default: mjpeg)
      --sink SINK        GStreamer video sink for fallback view (default: autovideosink)
  -o, --out-dir DIR      Directory for timestamped stills (default: captures)
  -h, --help             Show this help

Examples:
  tools/fpga_video_capture.sh list
  tools/fpga_video_capture.sh view
  tools/fpga_video_capture.sh view -s 1920x1080 -r 30
  tools/fpga_video_capture.sh still
  tools/fpga_video_capture.sh still captures/fpga.png
USAGE
}

die() {
    echo "error: $*" >&2
    exit 1
}

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"
}

require_device_access() {
    [[ -e "$device" ]] || die "capture device does not exist: $device"
    if [[ ! -r "$device" || ! -w "$device" ]]; then
        die "no read/write access to $device; add this user to the video group, then log out/in: sudo usermod -aG video $USER"
    fi
}

parse_size() {
    local value="$1"
    [[ "$value" =~ ^([0-9]+)x([0-9]+)$ ]] || die "invalid size '$value'; expected WxH"
    width="${BASH_REMATCH[1]}"
    height="${BASH_REMATCH[2]}"
}

device_caps() {
    if command -v v4l2-ctl >/dev/null 2>&1; then
        v4l2-ctl --device="$device" --list-formats-ext
    else
        echo "Install v4l-utils for detailed capture modes: sudo apt install v4l-utils" >&2
    fi
}

list_devices() {
    if command -v v4l2-ctl >/dev/null 2>&1; then
        v4l2-ctl --list-devices
    else
        for node in /dev/video*; do
            [[ -e "$node" ]] || continue
            printf '%s' "$node"
            if command -v udevadm >/dev/null 2>&1; then
                local product
                product="$(udevadm info --query=property --name="$node" 2>/dev/null | awk -F= '/^ID_V4L_PRODUCT=/{print $2; exit}')"
                [[ -n "$product" ]] && printf '  %s' "$product"
            fi
            printf '\n'
        done
        echo
        echo "Detailed modes for $device:"
        device_caps || true
    fi
}

is_mjpeg() {
    case "$format" in
        mjpeg|mjpg|jpeg) return 0 ;;
        raw|yuyv|yuy2) return 1 ;;
        *) die "unsupported format '$format'; use mjpeg or raw" ;;
    esac
}

view_vlc() {
    local vlc_bin="$1"
    exec "$vlc_bin" "v4l2://$device" \
        ":v4l2-width=$width" \
        ":v4l2-height=$height" \
        ":v4l2-fps=$fps" \
        ":live-caching=0"
}

view_ffplay() {
    local input_format
    case "$format" in
        mjpeg|mjpg|jpeg) input_format="mjpeg" ;;
        raw|yuyv|yuy2) input_format="v4l2" ;;
        *) die "unsupported format '$format'; use mjpeg or raw" ;;
    esac
    exec ffplay -hide_banner -fflags nobuffer -flags low_delay \
        -f v4l2 -input_format "$input_format" -video_size "${width}x${height}" \
        -framerate "$fps" "$device"
}

view_mpv() {
    exec mpv --profile=low-latency --untimed \
        "av://v4l2:$device" \
        --demuxer-lavf-o="input_format=${format},video_size=${width}x${height},framerate=${fps}"
}

view_gst() {
    need_cmd gst-launch-1.0
    if is_mjpeg; then
        exec gst-launch-1.0 -e \
            v4l2src "device=$device" do-timestamp=true \
            ! "image/jpeg,width=$width,height=$height,framerate=$fps/1" \
            ! jpegdec \
            ! videoconvert \
            ! "$sink" sync=false
    fi
    exec gst-launch-1.0 -e \
        v4l2src "device=$device" do-timestamp=true \
        ! "video/x-raw,width=$width,height=$height,framerate=$fps/1" \
        ! videoconvert \
        ! "$sink" sync=false
}

view() {
    require_device_access
    if command -v vlc >/dev/null 2>&1; then
        view_vlc vlc
    elif command -v cvlc >/dev/null 2>&1; then
        view_vlc cvlc
    elif command -v ffplay >/dev/null 2>&1; then
        view_ffplay
    elif command -v mpv >/dev/null 2>&1; then
        view_mpv
    else
        view_gst
    fi
}

still_ffmpeg() {
    local output="$1"
    local input_format
    case "$format" in
        mjpeg|mjpg|jpeg) input_format="mjpeg" ;;
        raw|yuyv|yuy2) input_format="v4l2" ;;
        *) die "unsupported format '$format'; use mjpeg or raw" ;;
    esac
    ffmpeg -hide_banner -loglevel warning -y \
        -f v4l2 -input_format "$input_format" -video_size "${width}x${height}" \
        -framerate "$fps" -i "$device" -frames:v 1 "$output"
}

still_gst() {
    local output="$1"
    need_cmd gst-launch-1.0
    if is_mjpeg; then
        gst-launch-1.0 -e \
            v4l2src "device=$device" num-buffers=1 do-timestamp=true \
            ! "image/jpeg,width=$width,height=$height,framerate=$fps/1" \
            ! jpegdec \
            ! videoconvert \
            ! pngenc \
            ! filesink "location=$output"
        return
    fi
    gst-launch-1.0 -e \
        v4l2src "device=$device" num-buffers=1 do-timestamp=true \
        ! "video/x-raw,width=$width,height=$height,framerate=$fps/1" \
        ! videoconvert \
        ! pngenc \
        ! filesink "location=$output"
}

still() {
    require_device_access
    local output="${1:-}"
    if [[ -z "$output" ]]; then
        mkdir -p "$out_dir"
        output="$out_dir/fpga-video-$(date +%Y%m%d-%H%M%S).png"
    else
        mkdir -p "$(dirname "$output")"
    fi

    if command -v ffmpeg >/dev/null 2>&1; then
        still_ffmpeg "$output"
    else
        still_gst "$output"
    fi
    echo "$output"
}

cmd="${1:-}"
[[ -n "$cmd" ]] || { usage; exit 2; }
shift || true

case "$cmd" in
    -h|--help|help)
        usage
        exit 0
        ;;
esac

positional=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--device)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            device="$2"
            shift 2
            ;;
        -s|--size)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            parse_size "$2"
            shift 2
            ;;
        -r|--fps)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            fps="$2"
            shift 2
            ;;
        -f|--format)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            format="$2"
            shift 2
            ;;
        --sink)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            sink="$2"
            shift 2
            ;;
        -o|--out-dir)
            [[ $# -ge 2 ]] || die "$1 requires an argument"
            out_dir="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            positional+=("$@")
            break
            ;;
        -*)
            die "unknown option: $1"
            ;;
        *)
            positional+=("$1")
            shift
            ;;
    esac
done

case "$cmd" in
    list) list_devices ;;
    view) view ;;
    still) still "${positional[0]:-}" ;;
    *) die "unknown command '$cmd'" ;;
esac
