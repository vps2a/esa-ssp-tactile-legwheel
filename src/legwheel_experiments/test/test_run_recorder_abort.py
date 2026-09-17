import json

from legwheel_experiments.data_recorder import RunRecorder


def test_mark_aborted_persists_reason_and_changes_marker(tmp_path):
    run_directory = tmp_path / "run_1"
    run_directory.mkdir()
    (run_directory / "RUNNING").touch()
    (run_directory / "metadata.json").write_text(
        json.dumps({"run_id": "run_1", "abort_reason": None}),
        encoding="utf-8",
    )

    recorder = RunRecorder(tmp_path)
    recorder.run_directory = run_directory
    recorder.mark_aborted("camera RGB timestamp stopped advancing")

    metadata = json.loads(
        (run_directory / "metadata.json").read_text(encoding="utf-8")
    )
    assert metadata["abort_reason"] == "camera RGB timestamp stopped advancing"
    assert "end_time_utc" in metadata
    assert (run_directory / "ABORTING").is_file()
    assert not (run_directory / "RUNNING").exists()
