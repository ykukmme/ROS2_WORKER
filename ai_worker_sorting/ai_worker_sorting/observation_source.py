"""Keep physics observations separate from hardware and fixed mock fixtures."""


def usable_observation(obj, cfg):
    if cfg.get('dry_run', False):
        return True
    kind = getattr(obj, 'source_kind', '')
    if cfg.get('runtime_mode', 'hardware') == 'mujoco':
        return obj.simulated and kind == 'mujoco'
    return not obj.simulated and kind in ('', 'hardware')


def mark_observation(obj, cfg):
    kind = 'mock' if cfg['mock_mode'] else cfg.get('observation_source', 'hardware')
    obj.source_kind, obj.simulated = kind, kind != 'hardware'
