# Test data for `ballistics_cli`

Each file is **one line** of whitespace-separated values:

```
xd yd zd targetX targetY attackSpeed accelerationPath ammo_name
```

## Sample files

| File | Ammo | Scenario | CLI exit | Expected `output.txt` |
|------|------|----------|----------|------------------------|
| `sample_vog17.txt` | VOG-17 | HW1 reference | 0 | `173.759 173.759` |
| `sample_m67.txt` | M67 | Drone above target (maneuver) | 0 | maneuver line, then `490.496 232` |
| `sample_rkg-3.txt` | RKG-3 | Short offset (maneuver) | 0 | maneuver line, then `513.085 202.085` |
| `sample_gliding-vog.txt` | GLIDING-VOG | Long range, no maneuver | 0 | `242.711 242.711` |
| `sample_gliding-rkg.txt` | GLIDING-RKG | Long range, no maneuver | 0 | `966.534 404.519` |

Maneuver cases print **two** lines: intermediate point, then drop point (same as HW1).

Unit tests in `homework_06/tests/ballistics_tests.cpp` mirror these fixtures (values ±0.01).

## Optional fixture

| File | Purpose |
|------|---------|
| `unknown_ammo.txt` | Invalid `ammo_name` — CLI exit `1`, message on stderr |

## Manual check

From repo root:

```bash
cmake --build --preset debug
./build/debug/homework_06/ballistics_cli homework_06/data/sample_vog17.txt
cat output.txt
rm -f output.txt
```

`output.txt` is written in the **current working directory** (same as HW1).
