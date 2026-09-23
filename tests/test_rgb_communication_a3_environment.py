"""A3 definition-only counterexamples. No simulator, renderer or model calls."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from harness.rgb_communication_scenarios import (
    A3_AUDIT_CASES, AUDIT_CASES, SOURCE_FILES, assess_offline_boundary,
    assess_scene_backend_selection, scene_support_matrix,
)
from harness.rgb_skill_execution import BACKEND_ID, SKILLS
from sim.research_dispatch_arena import digest, episode


def descriptor():
    config = episode("open", 11)
    static = config["static_map"]
    return {"schema": "ugrp.rgb_skill_backend_descriptor.v1", "backend_id": BACKEND_ID,
            "seed": 11, "map_id": static["map_id"], "map_sha256": digest(static),
            "map_instance_sha256": digest(static),
            "map_group_sha256": digest({k: static[k] for k in ("bounds_m", "obstacles", "top_camera")}),
            "camera_sha256": digest(static["top_camera"]), "reset_sha256": digest(config["setup_only"]),
            "goal_frame": static["frame"], "capabilities": copy.deepcopy(SKILLS),
            "synthetic": False, "weld": False, "camera_fov_changed": False,
            "clock_owner": "single_simulator", "clock_domain": "sim",
            "skill_image_max_age_s": 1., "skill_worker_wall_limit_s": 2.,
            "map_to_scene": {"source_map_sha256": digest(static),
                             "builder": "sim.research_dispatch_arena.build_scene_xml"}}


class A3SceneDefinitions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = scene_support_matrix()
        cls.rows = {r["selection"]: r for r in cls.matrix["rows"]}

    def test_all_58_entries_are_not_transport_support_or_physical_results(self):
        self.assertEqual(self.matrix["family_counts"],
                         {"legacy": 4, "dispatch": 5, "navigation": 9, "pair_navigation": 6, "act": 22, "multi_object": 12})
        self.assertEqual(self.matrix["entry_count"], 58)
        self.assertEqual([r["selection"] for r in self.rows.values() if r["supported_skills"]], ["dispatch/open"])
        self.assertFalse(self.matrix["execution_admission"])
        self.assertTrue(all(r["physical_success"] is False and r["runtime_scene_xml_sha256"] is None for r in self.rows.values()))

    def test_same_prior_map_does_not_hide_private_obstacle_or_terrain(self):
        shared, blocked, rough = [self.rows["dispatch/"+name] for name in ("shared_crossing", "north_blocked", "rough_south")]
        self.assertEqual(shared["parent_map_sha256"], blocked["parent_map_sha256"])
        self.assertNotEqual(shared["reset_definition_sha256"], blocked["reset_definition_sha256"])
        self.assertEqual(len({r["physical_geometry_sha256"] for r in (shared, blocked, rough)}), 3)

    def test_multi_object_instances_inherit_one_train_parent_not_12_maps(self):
        rows = [r for r in self.rows.values() if r["family"] == "multi_object"]
        self.assertEqual({r["parent_case_id"] for r in rows}, {"train-open-1"})
        self.assertEqual({r["declared_split"] for r in rows}, {"train"})
        self.assertEqual(len({r["parent_map_sha256"] for r in rows}), 1)

    def test_source_closure_includes_indirect_corner_and_evaluator(self):
        sources = self.matrix["definition_source_files"]
        for path in ("maps/pair_navigation/l-corner.json", "harness/dispatch_evaluation.py",
                     "sim/masterpi_camera_profile.py", "sim/masterpi_scene_v2.xml", "harness/real_geometry.py"):
            self.assertIn(path, sources)

    def test_supported_definition_is_not_an_execution_permit(self):
        result = assess_scene_backend_selection("dispatch/open", descriptor())
        self.assertTrue(result["ready"], result)
        self.assertFalse(result["execution_admission"])
        self.assertFalse(result["physical_controls_verified"])

    def test_unsupported_native_selections_and_physical_controls_fail_closed(self):
        for selection in self.rows:
            if selection != "dispatch/open":
                self.assertFalse(assess_scene_backend_selection(selection, descriptor())["ready"], selection)
        for purpose in ("easy_development_success", "impossible_safe_stop", "live"):
            self.assertFalse(assess_scene_backend_selection("dispatch/open", descriptor(), purpose=purpose)["ready"])

    def test_reset_camera_map_skill_or_budget_change_cannot_reuse_binding(self):
        for key in ("reset_sha256", "camera_sha256", "map_instance_sha256", "capabilities",
                    "skill_image_max_age_s", "skill_worker_wall_limit_s", "weld"):
            bad = descriptor(); bad[key] = None
            self.assertFalse(assess_scene_backend_selection("dispatch/open", bad)["ready"], key)

    def test_malformed_and_boolean_numeric_descriptor_values_fail_closed(self):
        for key, value in (("weld", 0), ("skill_image_max_age_s", True), ("map_to_scene", None)):
            bad = descriptor(); bad[key] = value
            self.assertFalse(assess_scene_backend_selection("dispatch/open", bad)["ready"])
        self.assertFalse(assess_scene_backend_selection("dispatch/open", None)["ready"])
        bad = descriptor(); bad["capabilities"]["solo_transport_A"]["team_size"] = True
        self.assertFalse(assess_scene_backend_selection("dispatch/open", bad)["ready"])


class A3ReviewGeneration(unittest.TestCase):
    def test_new_generation_cannot_downgrade_or_omit_new_cases(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root/"raw.json"; raw.write_text("{}")
            ref = {"path": "raw.json", "sha256": hashlib.sha256(raw.read_bytes()).hexdigest()}
            for name in SOURCE_FILES:
                path = root/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text("# test fixture\n")
            evidence = {"schema_version": "rgb-offline-boundary-review.v2", "scope": "offline", "verdict": "pass",
                        "independent_reviewer": "A3", "source_sha": "a"*40, "config_sha256": "b"*64,
                        "components": {k: "test-fixture" for k in ("serializer_id", "backend_id", "scheduler_id")},
                        "source_files": {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in SOURCE_FILES},
                        "cases": dict.fromkeys(A3_AUDIT_CASES, "pass"), "artifacts": [ref]}
            args = {"expected_source_sha": "a"*40, "expected_config_sha256": "b"*64,
                    "expected_components": evidence["components"], "artifact_root": root, "source_root": root,
                    "required_schema": "rgb-offline-boundary-review.v2"}
            self.assertTrue(assess_offline_boundary(evidence, **args)["ready"])
            for name in A3_AUDIT_CASES:
                bad = copy.deepcopy(evidence); del bad["cases"][name]
                self.assertFalse(assess_offline_boundary(bad, **args)["ready"], name)
            old = copy.deepcopy(evidence)
            old.update(schema_version="rgb-offline-boundary-review.v1", independent_reviewer="A2", cases=dict.fromkeys(AUDIT_CASES, "pass"))
            self.assertFalse(assess_offline_boundary(old, **args)["ready"])
            del args["required_schema"]
            self.assertTrue(assess_offline_boundary(old, **args)["ready"])
            args["expected_source_sha"] = "c"*40
            self.assertFalse(assess_offline_boundary(evidence, **args)["ready"])


class A3ActLineage(unittest.TestCase):
    def test_metadata_binding_required_and_episode_roles_do_not_create_holdout(self):
        script = Path(__file__).parent/"fixtures/rgb_communication_scenarios/audit_a3_metadata.py"
        spec = importlib.util.spec_from_file_location("a3_metadata_fixture", script)
        audit = importlib.util.module_from_spec(spec); spec.loader.exec_module(audit)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); qualification = root/"qualification"; qualification.mkdir()

            def write(path, value):
                path.write_text(json.dumps(value)); return path

            checkpoint = write(root/"model.json", {"fixture": True})
            protocol = {"asset_sha256": {str(checkpoint): audit.sha(checkpoint)},
                        "test": [{"id": "fixture", "variant": "open", "seed": 11, "offset": [0, 0, 0], "dock": "dock_a"}],
                        "scope": "fixture only"}
            protocol_path = write(qualification/"acceptance-v3-full-protocol.json", protocol)
            write(qualification/"runtime-v3-manifest.json", {"protocol_sha256": audit.sha(protocol_path)})
            write(qualification/"terminal-candidate-development-audit.json", {"eligible_for_default": False})
            (qualification/"README.md").write_text("fixture only")
            data = {"train": [], "development": []}
            for role in data:
                folder = root/role; folder.mkdir()
                names = ("result.json", "pair-decisions.json", "issued-commands.json", "skill-bindings.json",
                         "committed-plan.json", "episode-setup-only.json")
                for name in names:
                    write(folder/name, episode("open", 11) if name == "episode-setup-only.json" else {})
                data[role].append({"root": str(folder), "source_sha": "a"*40,
                                   "files": {name: audit.sha(folder/name) for name in names}, "image_hashes": {}})
            dataset = write(root/"dataset.json", data)
            training = {"dataset_sha256": audit.sha(dataset), "model_sha256": audit.sha(checkpoint),
                        "selection": "fixture", "train_episodes": [str(root/"train")],
                        "development_episodes": [str(root/"development")]}
            training_path = write(root/"training.json", training)
            result = audit.audit_act(qualification, dataset, training_path, checkpoint)
            self.assertTrue(result["checked_metadata_integrity"], result["integrity_blockers"])
            self.assertEqual(result["lineage"]["dataset"]["effective_splits"], ["regression"])
            self.assertFalse(result["heldout_claim_ready"])
            self.assertFalse(result["T1_T2_admission"])
            data["train"][0]["files"] = {}
            write(dataset, data); training["dataset_sha256"] = audit.sha(dataset); write(training_path, training)
            result = audit.audit_act(qualification, dataset, training_path, checkpoint)
            self.assertFalse(result["checked_metadata_integrity"])
            self.assertFalse(result["episodes"][0]["metadata_hashes_match"])
            self.assertTrue(any("binding absent" in issue for issue in result["integrity_blockers"]))
