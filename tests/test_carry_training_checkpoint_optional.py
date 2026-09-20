import pytest
try:
    import torch
except ImportError:
    pytest.skip('optional PyTorch environment',allow_module_level=True)
from harness.carry_training_checkpoint import save_checkpoint, restore_checkpoint


def test_resume_reproduces_next_update_with_dropout_and_sampler(tmp_path):
    torch.manual_seed(7)
    model=torch.nn.Sequential(torch.nn.Linear(3,4),torch.nn.Dropout(.2),torch.nn.Linear(4,1))
    optimizer=torch.optim.AdamW(model.parameters(),lr=.01)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,10)
    generator=torch.Generator().manual_seed(123)
    def update():
        idx=torch.multinomial(torch.ones(5),4,replacement=True,generator=generator)
        loss=model(idx.float()[:,None].repeat(1,3)).square().mean()
        optimizer.zero_grad();loss.backward();optimizer.step();scheduler.step()
        return idx, {k:v.detach().clone() for k,v in model.state_dict().items()}
    update()
    path=tmp_path/'resume.pt'
    kwargs=dict(signature={'seed':7},policy=model,optimizer=optimizer,scheduler=scheduler,generator=generator)
    save_checkpoint(path,**kwargs,step=1,best=.2,selected={},best_state={},progress=[])
    expected_idx,expected=update()
    torch.manual_seed(999);generator.manual_seed(999)
    restore_checkpoint(path,**kwargs)
    actual_idx,actual=update()
    assert torch.equal(expected_idx,actual_idx)
    for key in expected:assert torch.equal(expected[key],actual[key])
    with pytest.raises(ValueError,match='mismatch'):restore_checkpoint(path,**{**kwargs,'signature':{'seed':8}})
