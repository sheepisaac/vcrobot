#!/usr/bin/env python3
"""Robot-side synchronized RealSense video capture slave."""

from camera_sync import parse_slave_args, run_slave


DEFAULT_PORT = 50311


def main():
    args = parse_slave_args(DEFAULT_PORT, __doc__)
    run_slave(
        port=args.port,
        topic=args.topic,
        listen=args.listen,
        capture_timeout=args.capture_timeout,
        queue_size=args.queue_size,
        warmup_seconds=args.warmup_seconds,
        warmup_frames=args.warmup_frames,
        camera_node=args.camera_node,
        configure_camera=args.configure_camera,
        auto_white_balance=not args.no_auto_white_balance,
        auto_exposure=not args.no_auto_exposure,
        white_balance=args.white_balance,
        exposure=args.exposure,
        color_mode=args.color_mode,
        param_timeout=args.param_timeout,
        ros_localhost_only=not args.allow_ros_network,
    )


if __name__ == "__main__":
    main()
