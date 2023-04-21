from launch_param_builder import ParameterBuilder

package = "launch_param_builder"


def test_parameter_injection():
    base_params = (
        ParameterBuilder(package)
        .yaml(file_path="config/parameters_injection.yaml")
        .to_dict()
    )

    assert len(base_params) == 3, "Parameters not loaded"

    params_to_inject = (
        ParameterBuilder(package).yaml(base_params["param_file_to_inject"]).to_dict()
    )

    assert len(params_to_inject) == 2, "Parameters to inject not loaded"

    base_params.update(params_to_inject)

    assert len(base_params) == 5, "Parameters not injected"
