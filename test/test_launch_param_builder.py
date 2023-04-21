from launch_param_builder import ParameterBuilder


def test_builder():
    parameters = (
        ParameterBuilder("launch_param_builder")
        .parameter("my_parameter", 20.0)
        .file_parameter(
            "parameter_file", "config/parameter_file"
        )  # Or /absolute/path/to/file
        .yaml(file_path="config/parameters.yaml")  # Or /absolute/path/to/file
        .xacro_parameter(
            parameter_name="my_robot",
            file_path="config/parameter.xacro",  # Or /absolute/path/to/file
            mappings={"prefix": "robot"},
        )
        .to_dict()
    )

    assert parameters["my_parameter"] == 20.0, "Parameter not loaded"
    assert parameters.get("parameter_file") is not None, "Parameter file not loaded"
    assert len(parameters["ros2_versions"]) == 7, "Parameter yaml file not loaded"
    assert parameters["the_answer_to_life"] == 42
    assert parameters["package_name"] == "launch_param_builder"
    assert parameters.get("my_robot") is not None, "Parameter xacro not loaded"
