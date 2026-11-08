"""Bundled model and automatic inference device selection; no workspace guessing."""
import os
from pathlib import Path


def inference_device(requested=None,cuda_available=None):
    requested = str(requested if requested is not None else os.environ.get('WORKER_DEVICE','auto'))
    if requested!='auto':
        return requested
    if cuda_available is None:
        import torch
        cuda_available = torch.cuda.is_available()
    return '0' if cuda_available else 'cpu'


def model_path(saved=None):
    value = saved or os.environ.get('WORKER_DEFAULT_WEIGHTS','')
    if not value:
        return None
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ValueError('YOLO 가중치가 없습니다. 저장된 모델 경로 또는 볼륨 마운트를 확인하세요: '+str(path))
    return path
