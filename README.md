# GATR2-Override

The programs that run on the robot.

| Folder | What |
|---|---|
| [lib/](lib/README.md) | Code reused across seasons: [locaGATR](lib/locaGATR/README.md) (pose), [naviGATR](lib/naviGATR/README.md) (routes), [spectaGATR](lib/spectaGATR/README.md) (live viewer) |
| `2026-27/` | This season's PROS project |
| [docs/](docs/field-frame.md) | Field frame: where x, y and heading point, on red and on blue |

Each season gets its own PROS project folder.

## Libraries

A season `Makefile` builds two sets in place:

- `lib/` through [lib/lib.mk](lib/README.md): `GATR2_LIBS := spectagatr locagatr navigatr`
- GATR2-Electronics, cloned next to this repo, through `brain/gatr2_brain.mk`:
  `GATR2_BRAIN_LIBS := locagatr`

```
C:\GATR2-Electronics
C:\GATR2-Override\2026-27
```

Use relative paths; drive letters break the toolchain include flags.

## Build

The PROS kernel files (`firmware/`, `include/pros/`) are not in git. After a
fresh clone, in the season folder:

```
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
pros make
```

## Contributing

Changes reach `main` only through a PR: threads resolved, `build` passing,
up to date, one code owner approval, merged by the owner. The rules are on
GitHub under Settings, Rules.
