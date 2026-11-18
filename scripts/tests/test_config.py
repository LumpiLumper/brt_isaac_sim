import pytest

from config import PlacementConfig, SensorConfig, from_dict, parse_config
from utils.cone import ConeType


def test_defaults():
    cfg = parse_config([])
    assert cfg.sensor == SensorConfig()
    assert cfg.placement.algorithm == "uniform_annulus"
    assert cfg.placement.cone_types == tuple(ConeType)


def test_cli_flags_override_defaults():
    cfg = parse_config(["--sensor-height", "0.8", "--cones", "7", "--placement", "grid", "--frames", "3"])
    assert cfg.sensor.height == 0.8
    assert cfg.placement.num_cones == 7
    assert cfg.placement.algorithm == "grid"
    assert cfg.frames == 3


def test_toml_then_cli_override(tmp_path):
    f = tmp_path / "run.toml"
    f.write_text('seed = 5\n[sensor]\nheight = 2.0\n[placement]\nnum_cones = 10\ncone_types = ["SMALL_BLUE"]\n')
    cfg = parse_config(["--config", str(f), "--cones", "20"])
    assert cfg.seed == 5
    assert cfg.sensor.height == 2.0
    assert cfg.placement.num_cones == 20  # CLI wins
    assert cfg.placement.cone_types == (ConeType.SMALL_BLUE,)


def test_placement_params_are_typed():
    cfg = parse_config(["--placement-param", "rows=3", "--placement-param", "spacing=1.5", "--placement-param", "mode=zig"])
    assert cfg.placement.params == {"rows": 3, "spacing": 1.5, "mode": "zig"}


def test_placement_param_needs_key_value():
    with pytest.raises(SystemExit):
        parse_config(["--placement-param", "oops"])


@pytest.mark.parametrize("bad", [
    lambda: SensorConfig(height=0),
    lambda: SensorConfig(height=-1),
    lambda: PlacementConfig(num_cones=-1),
    lambda: PlacementConfig(radius=5, min_radius=5),
    lambda: PlacementConfig(min_gap=-0.1),
    lambda: PlacementConfig(cone_types=()),
])
def test_validation(bad):
    with pytest.raises(ValueError):
        bad()


def test_unknown_keys_rejected():
    with pytest.raises(ValueError, match="unknown config keys"):
        from_dict({"nope": 1})
    with pytest.raises(ValueError, match="unknown SensorConfig keys"):
        from_dict({"sensor": {"hieght": 1}})


def test_unknown_cone_type_name_rejected():
    with pytest.raises(KeyError):
        from_dict({"placement": {"cone_types": ["PURPLE"]}})
