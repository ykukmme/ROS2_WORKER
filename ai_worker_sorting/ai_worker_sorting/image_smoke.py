"""Offline YOLO check. Uses local inputs and never imports robot control."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True, type=Path)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--confidence', default=0.5, type=float)
    parser.add_argument('--expect-class', default='')
    args = parser.parse_args()
    for path in (args.weights, args.image):
        if not path.is_file():
            parser.error(f'Local file does not exist: {path}')
    if not 0 < args.confidence <= 1:
        parser.error('confidence must be in (0, 1]')

    import cv2
    from ultralytics import YOLO
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f'Cannot decode image: {args.image}')
    model = YOLO(str(args.weights))
    started = time.monotonic()
    result = model.predict(image, device=args.device, conf=args.confidence, verbose=False)[0]
    elapsed = time.monotonic() - started
    detections = []
    for box in result.boxes:
        detections.append({
            'class_id': int(box.cls.item()),
            'class_name': result.names[int(box.cls.item())],
            'confidence': float(box.conf.item()),
            'xyxy_pixels': box.xyxy[0].cpu().tolist(),
        })
    args.output.mkdir(parents=True, exist_ok=True)
    annotated = args.output / 'annotated.jpg'
    if not cv2.imwrite(str(annotated), result.plot()):
        raise RuntimeError(f'Cannot write {annotated}')
    report = {
        'image': str(args.image.resolve()), 'weights': str(args.weights.resolve()),
        'device': args.device, 'elapsed_seconds': elapsed,
        'note': 'Image detections only; no robot coordinates or physical grasp verified.',
        'detections': detections,
    }
    (args.output / 'detections.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if args.expect_class and not any(d['class_name'] == args.expect_class for d in detections):
        raise SystemExit(f'Expected class was not detected: {args.expect_class}')
    print(f'PASS: {len(detections)} detections; image saved to {annotated}')


if __name__ == '__main__':
    main()
