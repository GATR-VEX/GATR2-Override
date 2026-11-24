# spectaGATR viewer (browser)

Plain ES modules, no build step, no CDN, no web fonts. Everything it loads comes
from the local receiver (default 127.0.0.1:8790), so it runs offline. Open the
address the receiver prints; opening `index.html` as a file is not supported.

## Files

| File | Role |
|---|---|
| `index.html`, `style.css` | Page and layout; import map points `three` at `vendor/three/` |
| `app.js` | Wiring: field selector, camera buttons, banner, readouts, animation loop |
| `feed.js` | WebSocket to `/ws` with reconnect (250 ms backoff doubling to 2 s); `FeedModel` only records messages |
| `scene.js` | three.js scene: field meshes (built once per field change), robot box, path overlay, trail, conventions overlay, cameras |
| `transforms.js` | Pure math and state: field to scene mapping, attitude quaternion, trail ring, path normalizing and segments, stale logic, smoothing |
| `route.html`, `route.css`, `route.js` | Route editor page: draws a route on the field, writes `navigatr::Route` code |
| `route_model.js` | Route editor data and math: steps, headings, `next()` numbering, code out and back in |
| `vendor/three/` | three.js r170 + OrbitControls, MIT (see its README and LICENSE) |
| `tests/` | Test page for `transforms.js`, `feed.js`, `route_model.js` and the robot/path/camera code |

Fields come from `../fields/` (served at `/fields/<name>.json`, listed at
`/api/fields`); the protocol is `../docs/PROTOCOL.md`.

## Route editor

`/route.html`, linked from the viewer's bar. It needs no Brain and no feed.

- Click the field to add a point: a line, or a curve when Curve is on. Drag
  points and curve handles. Snap rounds to 0.5 in.
- Inches and degrees in the red frame of `docs/field-frame.md` in
  GATR2-Override, the same as the robot code.
- Colors and numbers are `next()` steps: lines and curves in a row are one;
  a turn or a wait is its own.
- Robot outlines at the start and after each step, sized from the side panel.
- The code box is the output: copy it into the program. Load from code reads
  it back. `at()` actions are left out and counted; add them after pasting.
- The draft and the field choice are kept in the browser's localStorage.

## Controls

- Field: selector filled from `/api/fields`. Default is the receiver's `hello.field`;
  a choice is remembered in localStorage (if storage is blocked it is just not remembered).
- Overhead (default): top-down, +X right, +Y up, as the field diagram is drawn. Drag pans, wheel zooms.
- 3D orbit: drag rotates, right drag pans, wheel zooms. Follow robot: 3D view centered on the robot.
- Reset: back to the current view's default. Clear trail: erases the trail.
- Smooth 100 ms (off by default): blends the drawn robot to each new pose over at most
  100 ms (slerp for attitude). Readouts and trail always use received values.
- URL options for testing: `?view=overhead|orbit|follow`, `?field=<name>`, `?smooth=1`.

## States

| Shown | When |
|---|---|
| Receiver disconnected, retrying | WebSocket closed; reconnects by itself |
| Searching for VEX device | `status.link` is not `open` (detail from `status.detail`) |
| Waiting for first pose | Linked, no pose yet |
| STALE age | `status.stale` for the shown pose, or local receive age over `hello.stale_ms` |
| LIVE | Fresh poses |

When not live the robot stays at its last received pose, dimmed and labeled "last
known pose"; it is never extrapolated. The trail breaks on a new session, a new
epoch (`restart`), a reconnect, or a gap longer than `stale_ms`, and never joins
across a break. It holds the newest 4000 segments in a preallocated buffer.

Receive age is the time since the receiver accepted the shown pose, on laptop
clocks only. It is not end-to-end latency, which is not measured.

## Robot box and path

The robot box is the receiver's default (`hello.robot`, `--robot`) until a `robot`
message brings the footprint the Brain sent; the readout says which. The path is
the Brain's `path` message: straight cyan segments between adjacent points that
have arrived, a dot per point, a green dot at the start and a label at the end. A
point not received yet leaves a gap. The overlay is rebuilt only when a path
message arrives; a `hello` (reconnect) clears it until the receiver resends it.

## Conventions shown

Origin ring at the inside bottom-left corner; red +X and green +Y arrows (+X
labeled heading 0, +Y heading +90); a blue counterclockwise arc for positive
heading; on the robot a white front face and edge, a green left side and edge,
and red forward (+x), green left (+y), blue up (+z) arrows. The side panel lists
roll (about +x, positive lifts the left side), pitch (about +y, positive tips
the nose down) and R = Rz(yaw) Ry(pitch) Rx(roll). Attitude only rotates the box
about its pose origin; it never raises it.

## Tests

Module scripts do not load from `file://` in current Chrome, so serve the
directory. From GATR2-Override, with the mock feed (also serves `/tests/`):

```
python lib/spectaGATR/tools/mock_feed.py --port 8791
"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu \
    --virtual-time-budget=5000 --dump-dom 127.0.0.1:8791/tests/index.html | grep -o 'data-result="[A-Z]*">[^<]*\|FAIL [^<]*'
```

or `python -m http.server 8799 --bind 127.0.0.1 --directory lib/spectaGATR/web`
and open `127.0.0.1:8799/tests/index.html`. The page writes one PASS/FAIL line per
test and a summary (`#summary`, `data-result`).

Live page with synthetic data: `python lib/spectaGATR/tools/mock_feed.py` and open
127.0.0.1:8790 (scenarios: `--scenario figure8|attitude|restart|dropout|searching|all`,
see the script header; `--exit-after-s N` shows the disconnected state).
Headless screenshots need software WebGL: add
`--use-angle=swiftshader --enable-unsafe-swiftshader --window-size=1400,900 --screenshot=out.png`.
