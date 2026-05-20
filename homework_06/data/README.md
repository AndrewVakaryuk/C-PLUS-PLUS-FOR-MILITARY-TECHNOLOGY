# Test data for `ballistics_cli`

Each file is **one line** of whitespace-separated values:

```
xd yd zd targetX targetY attackSpeed accelerationPath ammo_name
```

| File               | Status    | Notes |
|--------------------|-----------|-------|
| `sample_vog17.txt` | included  | HW1 reference input; expect drop point `173.759 173.759` |

## Suggested fixtures (add yourself)

| File               | Purpose |
|--------------------|---------|
| `unknown_ammo.txt` | Invalid `ammo_name` — CLI should exit `1`, message on stderr |
| (your choice)      | Maneuver scenario, gliding ammo, edge cases |

## Manual check (after Step 4)

From repo root:

```bash
./build/debug/homework_06/ballistics_cli homework_06/data/sample_vog17.txt
cat output.txt
```

`output.txt` is written in the **current working directory** (same as HW1).
