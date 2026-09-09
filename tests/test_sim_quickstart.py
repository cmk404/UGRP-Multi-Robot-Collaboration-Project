import pytest

from scripts.sim_quickstart import run


@pytest.mark.parametrize("seconds", [float("nan"), 0.1, 5.1])
def test_quickstart_rejects_unbounded_simulation_before_creating_output(tmp_path, seconds):
    output = tmp_path / "demo"
    with pytest.raises(ValueError, match="seconds"):
        run(output, seconds=seconds)
    assert not output.exists()


def test_quickstart_requires_a_fresh_output_directory(tmp_path):
    output = tmp_path / "demo"
    output.mkdir()
    with pytest.raises(FileExistsError):
        run(output)
