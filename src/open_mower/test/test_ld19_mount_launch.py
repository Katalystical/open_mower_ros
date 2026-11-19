#!/usr/bin/env python3
import os
import tempfile
import unittest
from unittest.mock import patch

import roslaunch.config
import roslaunch.xmlloader


LAUNCH_DIR = os.path.join(os.path.dirname(__file__), "../launch")
MOUNT_KEYS = ("X", "Y", "Z", "YAW", "PITCH", "ROLL")


class Ld19MountLaunchTest(unittest.TestCase):
    def setUp(self):
        self.params_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.params_dir.cleanup)
        with open(os.path.join(self.params_dir.name, "custom_params.yaml"), "w") as config:
            config.write("{}\n")
        environment = os.environ.copy()
        for key in MOUNT_KEYS:
            environment.pop("OM_LD19_" + key, None)
        environment.update({
            "HARDWARE_PLATFORM": "2",
            "MOWER": "CUSTOM",
            "PARAMS_PATH": self.params_dir.name,
            "RECORDINGS_PATH": self.params_dir.name,
            "OM_LEGACY_CONFIG_MODE": "false",
            "OM_LD19": "false",
            "OM_USE_NTRIP": "false",
            "OM_NO_GPS": "true",
        })
        environment_patch = patch.dict(os.environ, environment, clear=True)
        environment_patch.start()
        self.addCleanup(environment_patch.stop)

    def load(self, launch_file, args):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            os.path.join(LAUNCH_DIR, launch_file), config, argv=args, verbose=False
        )
        return [node for node in config.nodes if node.name == "base_to_laser"]

    def assert_transform(self, nodes, expected):
        self.assertEqual(1, len(nodes))
        arguments = nodes[0].args.split()
        self.assertEqual(expected, [float(value) for value in arguments[:6]])
        self.assertEqual(["base_link", "base_laser", "50"], arguments[6:])

    def test_defaults_and_disabled(self):
        for launch_file in ("open_mower.launch", "move_around.launch"):
            with self.subTest(launch=launch_file):
                self.assertEqual([], self.load(launch_file, []))
                self.assert_transform(
                    self.load(launch_file, ["use_ld19:=true"]), [0.0, 0.0, 0.18, 0.0, 0.0, 0.0]
                )

    def test_environment_and_launch_override(self):
        values = ("0.31", "-0.12", "0.24", "-1.57", "0.1", "-0.2")
        for key, value in zip(MOUNT_KEYS, values):
            os.environ["OM_LD19_" + key] = value
        for launch_file in ("open_mower.launch", "move_around.launch"):
            with self.subTest(launch=launch_file):
                self.assert_transform(
                    self.load(launch_file, ["use_ld19:=true"]),
                    [0.31, -0.12, 0.24, -1.57, 0.1, -0.2],
                )
                self.assert_transform(
                    self.load(launch_file, ["use_ld19:=true", "ld19_yaw:=0.75"]),
                    [0.31, -0.12, 0.24, 0.75, 0.1, -0.2],
                )
                self.assert_transform(
                    self.load(launch_file, [
                        "use_ld19:=true", "ld19_x:=-0.4", "ld19_y:=0.5",
                        "ld19_z:=0.6", "ld19_yaw:=0.7", "ld19_pitch:=-0.8",
                        "ld19_roll:=0.9",
                    ]),
                    [-0.4, 0.5, 0.6, 0.7, -0.8, 0.9],
                )

    def test_invalid_and_nonfinite_values_are_rejected(self):
        for key in MOUNT_KEYS:
            for value in ("not-a-number", "nan", "inf", "-inf"):
                with self.subTest(key=key, value=value):
                    with self.assertRaises(roslaunch.xmlloader.XmlParseException):
                        self.load("open_mower.launch", ["use_ld19:=true", "ld19_" + key.lower() + ":=" + value])


if __name__ == "__main__":
    unittest.main()
