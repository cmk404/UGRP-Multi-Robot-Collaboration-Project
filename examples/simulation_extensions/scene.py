"""Edit this file to add obstacles, terrain or movable objects (metres/kg)."""


def build_scene(*, seed, params):
    # Use random.Random(seed), not global randomness, for reproducible variants.
    return [
        {"name": "barrier", "shape": "box", "xyz_m": [0, .8, .15],
         "size_m": [params.get("barrier_half_width_m", .35), .06, .15],
         "rgba": [.8, .2, .1, 1]},
        {"name": "ball", "shape": "sphere", "xyz_m": [.5, .8, .08],
         "size_m": [.05], "dynamic": True, "mass_kg": .08,
         "rgba": [.1, .4, .9, 1]},
    ]
