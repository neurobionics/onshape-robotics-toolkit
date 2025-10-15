from __future__ import annotations

from pathlib import Path

from onshape_robotics_toolkit.config import (
    AssemblyConfig,
    CADConfig,
    ClientConfig,
    DocumentConfig,
    ExportConfig,
    KinematicsConfig,
    LoggingConfig,
    NameOverrideEntry,
    NameOverrides,
    ORTConfig,
    PreprocessingConfig,
    RobotBuildConfig,
    activate_config,
    get_active_session,
    record_assembly_config,
    record_cad_config,
    record_client_config,
    record_document_config,
    record_export_config,
    record_kinematics_config,
    record_logging_config,
    record_mate_name,
    record_part_name,
    record_robot_config,
    record_session,
    record_variable_update,
    resolve_part_name,
    save_active_session,
)


def test_toolkit_config_round_trip(tmp_path: Path) -> None:
    get_active_session().reset()

    config = ORTConfig(
        logging=LoggingConfig(
            mode="default",
            console_level="INFO",
            file_level="DEBUG",
            file_path="toolkit.log",
        ),
        client=ClientConfig(env=".env", base_url="https://cad.onshape.com"),
        document=DocumentConfig(
            url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b"
        ),
        preprocessing=PreprocessingConfig(),
        assembly=AssemblyConfig(configuration="custom"),
        cad=CADConfig(max_depth=3),
        kinematics=KinematicsConfig(use_user_defined_root=False),
        robot=RobotBuildConfig(name="demo_bot", type="xml", fetch_mass_properties=False),
        export=ExportConfig(file_path="robot.xml", download_assets=False, mesh_dir="meshes"),
        names=NameOverrides(
            parts={"body": NameOverrideEntry(original="body", name="Base")},
            mates={"mate": NameOverrideEntry(original="mate", name="BaseJoint")},
        ),
    )

    config_path = tmp_path / "session.yaml"
    config.save(config_path)

    loaded = ORTConfig.load(config_path)
    assert loaded == config

    activate_config(loaded, reset=True)
    assert resolve_part_name("body") == "Base"
    assert get_active_session().auto_save_path == config_path


def test_record_session_accumulates_configuration(tmp_path: Path) -> None:
    save_path = tmp_path / "captured.yaml"

    get_active_session().reset()

    with record_session(save_path=save_path) as session:
        record_logging_config(
            LoggingConfig(
                mode="default",
                console_level="DEBUG",
                file_level="INFO",
                file_path="session.log",
                clear_existing_handlers=False,
                delay_file_creation=True,
            )
        )
        record_client_config(env=".env", base_url="https://cad.onshape.com")
        record_document_config(
            base_url="https://cad.onshape.com",
            did="doc-id",
            wtype="w",
            wid="workspace-id",
            eid="element-id",
            name="Example Assembly",
        )
        record_variable_update("variables-id", {"wheelDiameter": "180 mm"})
        record_assembly_config(
            element_id="assembly-id",
            configuration="fast",
            log_response=False,
            with_meta_data=True,
        )
        record_cad_config(max_depth=4)
        record_kinematics_config(use_user_defined_root=False)
        record_robot_config(name="bike", robot_type="urdf", fetch_mass_properties=True)
        record_export_config(file_path="robot.urdf", download_assets=True, mesh_dir="meshes")
        record_part_name("body_1", "Body 1 <1>")
        record_mate_name("mate_1", "Mate 1")

    # Ensure the context auto-saved
    assert save_path.exists()

    data = session.to_config()
    assert data.client and data.client.env == ".env"
    assert data.document and data.document.did == "doc-id"
    assert data.assembly and data.assembly.configuration == "fast"
    assert data.cad and data.cad.max_depth == 4
    assert data.kinematics and data.kinematics.use_user_defined_root is False
    assert data.robot and data.robot.name == "bike"
    assert data.export and data.export.file_path == "robot.urdf"
    assert data.names.parts["body_1"].original == "Body 1 <1>"
    assert data.names.parts["body_1"].name == "body_1"

    loaded = ORTConfig.load(save_path)
    assert loaded.document and loaded.document.name == "Example Assembly"
    assert loaded.preprocessing
    assert loaded.preprocessing.variable_updates[0].expressions["wheelDiameter"] == "180 mm"
    assert loaded.names.parts["body_1"].original == "Body 1 <1>"
    assert loaded.names.mates["mate_1"].original == "Mate 1"
    assert "mate_1" in loaded.names.mates


def test_global_session_records_without_context(tmp_path: Path) -> None:
    session = get_active_session()
    session.reset()

    record_client_config(env=".env", base_url="https://cad.onshape.com")
    record_document_config(
        base_url="https://cad.onshape.com",
        did="doc",
        wtype="w",
        wid="wid",
        eid="eid",
    )
    record_cad_config(max_depth=1)
    record_part_name("body_1", "Body 1 <1>")

    save_path = tmp_path / "global.yaml"
    save_active_session(save_path)

    loaded = ORTConfig.load(save_path)
    assert loaded.client and loaded.client.env == ".env"
    assert loaded.cad and loaded.cad.max_depth == 1
    assert resolve_part_name("body_1") == "body_1"
    assert get_active_session().auto_save_path == save_path
    get_active_session().reset()
