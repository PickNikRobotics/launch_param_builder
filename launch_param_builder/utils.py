# Copyright 2021 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import yaml
from tempfile import NamedTemporaryFile
from pathlib import Path
from typing import List, Union, Optional
import xacro
from ament_index_python.packages import get_package_share_directory
import jinja2
import math


class ParameterBuilderFileNotFoundError(KeyError):
    pass


# Parameter value types
ParameterValueType = Union[
    str,
    int,
    float,
    bool,
    List[str],
    List[int],
    List[float],
    List[bool],
    bytes,
]


def render_template(template: Path, mappings: dict):
    with template.open("r") as file:
        jinja2_template = jinja2.Template(file.read())
        jinja2_template.globals["radians"] = math.radians
        jinja2_template.globals["degrees"] = math.degrees
    return jinja2_template.render(mappings)


def raise_if_file_not_found(file_path: Path):
    if not file_path.exists():
        raise ParameterBuilderFileNotFoundError(f"File {file_path} doesn't exist")


def create_file_from_template(file: Path, mappings: Optional[dict] = None):
    with NamedTemporaryFile(
        mode="w", prefix="launch_param_builder_", delete=False
    ) as parsed_file:
        parsed_file_path = parsed_file.name
        parsed_file.write(render_template(file, mappings or {}))
        return parsed_file_path


def load_file(file_path: Path, mappings: Optional[dict] = None):
    raise_if_file_not_found(file_path)
    if mappings is not None:
        return render_template(file_path, mappings)
    try:
        with open(file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path: Path, mappings: Optional[dict] = None):
    raise_if_file_not_found(file_path)

    try:
        return yaml.load(
            render_template(file_path, mappings or {}), Loader=yaml.FullLoader
        )
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_xacro(file_path: Path, mappings: Optional[dict] = None):
    raise_if_file_not_found(file_path)

    file = xacro.process_file(file_path, mappings=mappings)
    return file.toxml()


def get_path(package_name: str, file_path: str):
    return str(Path(get_package_share_directory(package_name)) / file_path)
