"""Optional ML worker with a strict explicit chronological RGB input window."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.carry_input_history import decode_request


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--torch-threads', type=int, choices=(1, 2), default=2)
    a = p.parse_args()
    import torch
    from harness.carry_input_act import InputCarryAct
    torch.set_num_threads(a.torch_threads)
    actor = InputCarryAct.load(a.model_dir, cache_features=True)
    print(json.dumps({'ready': True, 'history': actor.history}), flush=True)
    for line in sys.stdin:
        try:
            result = actor.predict(decode_request(json.loads(line), actor.history))
        except Exception as error:
            print(json.dumps({'error': str(error)}), flush=True)
            return 1
        print(json.dumps(result, allow_nan=False), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
