"""Local webcam YOLO preview; independent of ROS and robot control."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True, type=Path)
    parser.add_argument('--camera', default=0, type=int)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--confidence', default=0.5, type=float)
    parser.add_argument('--imgsz', default=416, type=int)
    parser.add_argument('--width', default=640, type=int)
    parser.add_argument('--height', default=480, type=int)
    parser.add_argument('--max-frames', default=0, type=int, help='0 means run until stopped')
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--save-last', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/webcam'))
    parser.add_argument('--stop-file', type=Path)
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error('weights must point to an existing local model')
    if not 0 < args.confidence <= 1:
        parser.error('confidence must be in (0, 1]')
    if min(args.width, args.height, args.imgsz) <= 0 or args.max_frames < 0:
        parser.error('Invalid image size or frame limit')
    if args.stop_file and args.stop_file.exists():
        parser.error('Stop file already exists; choose a new stop-file path')

    import cv2
    import torch
    from ultralytics import YOLO
    model = YOLO(str(args.weights))
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_ANY)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('Cannot open webcam. Check camera permission and other camera apps.')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    args.output.mkdir(parents=True, exist_ok=True)
    title = 'AI WORKER - Webcam YOLO | Q: quit | S: save'
    frames = 0
    annotated = None
    latest = []
    observed_classes = set()
    started = time.monotonic()
    exit_reason = 'unknown'
    previous_log = started
    failures = 0
    error = None

    def save_image(path, image):
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f'Cannot save image: {path}')

    print('WEBCAM_START: camera preview only, no robot connection', flush=True)
    try:
        if not args.no_display:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, args.width, args.height)
        while True:
            if args.stop_file and args.stop_file.exists():
                exit_reason = 'stop_file'
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 10:
                    raise RuntimeError('Webcam stopped delivering frames')
                time.sleep(0.05)
                continue
            failures = 0
            result = model.predict(frame, device=args.device, conf=args.confidence,
                                   imgsz=args.imgsz, verbose=False)[0]
            latest = [
                {'class_name': result.names[int(box.cls.item())],
                 'confidence': float(box.conf.item()),
                 'xyxy_pixels': box.xyxy[0].cpu().tolist()}
                for box in result.boxes
            ]
            observed_classes.update(d['class_name'] for d in latest)
            frames += 1
            elapsed = time.monotonic()-started
            annotated = result.plot()
            cv2.putText(annotated, f'{frames/max(elapsed, 0.001):.1f} FPS | Q: quit  S: save',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            now = time.monotonic()
            if frames == 1 or now-previous_log >= 5:
                print(json.dumps({'frames': frames, 'average_fps': round(frames/max(elapsed, .001), 2),
                                  'classes': [d['class_name'] for d in latest]}), flush=True)
                previous_log = now
            if not args.no_display:
                cv2.imshow(title, annotated)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), ord('Q'), 27):
                    exit_reason = 'keyboard'
                    break
                if key in (ord('s'), ord('S')):
                    path = args.output / f'snapshot-{time.time_ns()}.jpg'
                    save_image(path, annotated)
                    print(f'SAVED: {path}', flush=True)
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    exit_reason = 'window_closed'
                    break
            if args.max_frames and frames >= args.max_frames:
                exit_reason = 'frame_limit'
                break
    except KeyboardInterrupt:
        exit_reason = 'keyboard_interrupt'
    except Exception as exc:
        exit_reason = 'error'
        error = str(exc)
        raise
    finally:
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        if args.save_last and annotated is not None:
            save_image(args.output / 'last-frame.jpg', annotated)
        elapsed = time.monotonic()-started
        summary = {
            'mode': 'local_webcam_no_robot', 'camera_index': args.camera,
            'frames': frames, 'elapsed_seconds': elapsed,
            'average_fps': frames/max(elapsed, 0.001), 'exit_reason': exit_reason,
            'error': error, 'classes_seen': sorted(observed_classes),
            'last_frame_detections': latest,
            'torch_version': torch.__version__, 'device': args.device,
        }
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print('WEBCAM_STOP: ' + json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
