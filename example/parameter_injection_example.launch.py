# $LICENSE$

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch_param_builder import ParameterBuilder
from launch_param_builder.utils import get_path


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    injection_params_package = LaunchConfiguration("injection_params_package").perform(
        context
    )

    base_params = (
        ParameterBuilder(injection_params_package)
        .yaml(file_path="config/parameters_injection.yaml")
        .to_dict()
    )

    print("Base Params File Content: ")
    print(base_params)

    print("File path to inject: " + base_params["param_file_to_inject"])
    params_to_inject = (
        ParameterBuilder(injection_params_package)
        .yaml(base_params["param_file_to_inject"])
        .to_dict()
    )
    print("Params to inject: ")
    print(params_to_inject)

    base_params.update(params_to_inject)
    print("Update parameters: ")
    print(base_params)

    return []


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "injection_params_package",
            default_value="launch_param_builder",
            description="Package with example files to demonstrate parameters injection.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
