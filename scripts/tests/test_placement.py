import numpy as np
import pytest

from config import PlacementConfig
from scene.placement import available, get_strategy, register
from scene.placement import base
from utils.cone import Cone, ConeType


def layout(name, seed=0, **kw):
    cfg = PlacementConfig(algorithm=name, **kw)
    return cfg, get_strategy(name).place(np.random.default_rng(seed), cfg)


@pytest.mark.parametrize("name", available())
def test_every_strategy_honours_the_config(name):
    cfg, cones = layout(name, num_cones=40, radius=30.0, min_radius=3.0, min_gap=1.0)
    xy = np.array([[c.x, c.y] for c in cones])
    r = np.linalg.norm(xy, axis=1)

    assert len(cones) == 40
    assert all(isinstance(c, Cone) for c in cones)
    assert r.min() >= 3.0 and r.max() <= 30.0
    d = np.linalg.norm(xy[:, None] - xy[None], axis=2) + np.eye(len(xy)) * 1e9
    assert d.min() >= 1.0


@pytest.mark.parametrize("name", available())
def test_every_strategy_is_deterministic(name):
    assert layout(name, seed=3, num_cones=15)[1] == layout(name, seed=3, num_cones=15)[1]
    assert layout(name, seed=3, num_cones=15)[1] != layout(name, seed=4, num_cones=15)[1]


@pytest.mark.parametrize("name", available())
def test_every_strategy_only_uses_allowed_types(name):
    _, cones = layout(name, num_cones=30, cone_types=(ConeType.SMALL_BLUE, ConeType.BIG_ORANGE))
    assert {c.type for c in cones} <= {ConeType.SMALL_BLUE, ConeType.BIG_ORANGE}


def test_uniform_annulus_registered_by_default():
    assert "uniform_annulus" in available()


def test_uniform_annulus_impossible_layout_raises():
    with pytest.raises(RuntimeError, match="cannot fit"):
        layout("uniform_annulus", num_cones=500, radius=3.0, min_radius=2.0, min_gap=2.0)


def test_unknown_strategy_lists_available():
    with pytest.raises(KeyError, match="uniform_annulus"):
        get_strategy("nope")


def test_new_strategy_can_be_added_and_switched_to(monkeypatch):
    monkeypatch.setattr(base, "REGISTRY", dict(base.REGISTRY))

    @register("line")
    class Line:
        def place(self, rng, cfg):
            return [Cone(cfg.cone_types[0], float(i), 0.0, 0.0) for i in range(cfg.num_cones)]

    cfg, cones = layout("line", num_cones=3)
    assert [c.x for c in cones] == [0.0, 1.0, 2.0]
    assert "line" in available()


def test_duplicate_registration_rejected(monkeypatch):
    monkeypatch.setattr(base, "REGISTRY", dict(base.REGISTRY))
    with pytest.raises(ValueError, match="already registered"):
        register("uniform_annulus")(object)
