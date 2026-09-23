"""Interactive start remains a thin selector for the managed native CLI."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import sim_cli, sim_dispatch
from sim.simulation_launch_options import preview_maps


ROOT = Path(__file__).resolve().parents[1]


def _interactive(monkeypatch, answers):
    replies = iter(answers)
    monkeypatch.setattr(sim_cli.sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr("builtins.input", lambda _prompt: next(replies))
    monkeypatch.chdir(ROOT)


def test_start_dispatch_passes_selected_model_map_speed_and_task_to_managed_parser(monkeypatch, tmp_path):
    task = "dock_b로 함께 옮겨"
    _interactive(monkeypatch, ["1", "open", "3", "2", "chosen-model", task])
    seen = {}

    def fake_dispatch(argv):
        seen["dispatch"] = argv
        return 19

    def fake_managed(root, kind, argv, invoke, **_kwargs):
        seen.update(root=root, kind=kind, argv=argv)
        return invoke(tmp_path / "result")

    monkeypatch.setattr(sim_dispatch, "main", fake_dispatch)
    monkeypatch.setattr("sim.workflow_manager.run_inprocess", fake_managed)
    assert sim_cli.main(["start"]) == 19
    assert seen["root"] == ROOT and seen["kind"] == "dispatch"
    assert seen["argv"] == ["dispatch", "--task", task, "--variant", "open",
                            "--model", "chosen-model", "--realtime-factor", "2"]
    assert seen["dispatch"][:-2] == seen["argv"][1:]
    assert seen["dispatch"][-2:] == ["--output", str(tmp_path / "result")]


def test_start_preview_uses_catalog_group_and_has_no_model_task(monkeypatch, capsys):
    first_act = next(scene for scene in preview_maps() if scene.startswith("act/"))
    _interactive(monkeypatch, ["2", "?", "act", "1", "4"])
    seen = {}

    def fake_managed(root, kind, argv, invoke, **_kwargs):
        seen.update(root=root, kind=kind, argv=argv)
        return 23

    monkeypatch.setattr("sim.workflow_manager.run_inprocess", fake_managed)
    assert sim_cli.main(["start"]) == 23
    assert seen["root"] == ROOT and seen["kind"] == "local"
    assert seen["argv"] == ["run", "configs/simulation/local.json", "--scene", first_act,
                            "--realtime-factor", "4", "--paused", "--capture", "--sim-seconds", "600"]
    assert "--model" not in seen["argv"] and "--task" not in seen["argv"]
    output = capsys.readouterr().out
    assert "정책 실행·성공 검증이 아닙니다" in output
    assert "act (" in output


def test_start_preview_search_narrows_scene_list(monkeypatch, capsys):
    _interactive(monkeypatch, ["2", "/shared_crossing", "1", ""])
    monkeypatch.setattr("sim.workflow_manager.run_inprocess",
                        lambda _root, _kind, argv, _invoke, **_kwargs: argv)
    command = sim_cli.main(["start"])
    assert command[command.index("--scene") + 1] == "dispatch/shared_crossing"
    assert "dispatch/shared_crossing" in capsys.readouterr().out


def test_start_dispatch_enter_uses_default_mission(monkeypatch):
    _interactive(monkeypatch, ["", "", "", "", ""])
    monkeypatch.setattr("sim.workflow_manager.run_inprocess",
                        lambda _root, _kind, argv, _invoke, **_kwargs: argv)
    command = sim_cli.main(["start"])
    assert command[command.index("--variant") + 1] == "shared_crossing"
    assert command[command.index("--task") + 1] == "기존 beam과 box를 같은 dock으로 옮겨"


def test_start_keeps_existing_advanced_menu(monkeypatch):
    _interactive(monkeypatch, ["3"])
    monkeypatch.setattr(sim_dispatch, "choose", lambda: 17)
    assert sim_cli.main(["start"]) == 17


@pytest.mark.parametrize("answers", [["q"], ["2", "q"], ["1", "open", "q"]])
def test_start_cancel_does_not_launch(monkeypatch, answers):
    _interactive(monkeypatch, answers)
    monkeypatch.setattr("sim.workflow_manager.run_inprocess",
                        lambda *_args, **_kwargs: pytest.fail("cancelled menu launched a run"))
    assert sim_cli.main(["start"]) == 0


@pytest.mark.parametrize("error,code", [(EOFError, 0), (KeyboardInterrupt, 130)])
def test_start_terminal_interrupt_does_not_launch(monkeypatch, error, code):
    monkeypatch.setattr(sim_cli.sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(error))
    monkeypatch.setattr("sim.workflow_manager.run_inprocess",
                        lambda *_args, **_kwargs: pytest.fail("cancelled menu launched a run"))
    assert sim_cli.main(["start"]) == code


def test_start_requires_tty(monkeypatch, capsys):
    monkeypatch.setattr(sim_cli.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    assert sim_cli.main(["start"]) == 2
    assert "interactive terminal" in capsys.readouterr().err
