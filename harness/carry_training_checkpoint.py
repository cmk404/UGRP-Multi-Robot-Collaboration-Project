"""Atomic optimizer/RNG checkpoints for trusted, locally produced ACT runs."""
from pathlib import Path
import torch


def save_checkpoint(path, *, signature, step, policy, optimizer, scheduler, generator, best, selected, best_state, progress, elapsed_s=0.0):
    data={'signature':signature,'step':step,'model':policy.state_dict(),'optimizer':optimizer.state_dict(),
          'scheduler':scheduler.state_dict(),'generator':generator.get_state(),'torch_rng':torch.get_rng_state(),
          'cuda_rng':torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [],
          'elapsed_s':elapsed_s,'best':best,'selected':selected,'best_state':best_state,'progress':progress}
    path=Path(path);temp=path.with_suffix('.tmp');torch.save(data,temp);temp.replace(path)


def restore_checkpoint(path, *, signature, policy, optimizer, scheduler, generator):
    data=torch.load(path,map_location='cpu',weights_only=True)
    if data['signature']!=signature:raise ValueError('resume source/data/arm/device/steps mismatch')
    policy.load_state_dict(data['model']);optimizer.load_state_dict(data['optimizer']);scheduler.load_state_dict(data['scheduler'])
    generator.set_state(data['generator']);torch.set_rng_state(data['torch_rng'])
    if data['cuda_rng']:torch.cuda.set_rng_state_all(data['cuda_rng'])
    return data
