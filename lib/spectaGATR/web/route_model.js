// route_model.js
// Route editor data and math, no DOM. Field inches and degrees in the frame of
// docs/field-frame.md (the red frame). Mirrors navigatr::Route: segments in a
// row are one move, the heading holds unless facing() or tangent() sets it,
// turnTo() and wait() are steps of their own.
//
//   route = { start: {x, y, heading}, steps: [step, ...] }
//   step  = {kind: 'line', end, facing?, tangent?}
//         | {kind: 'bezier', c1, c2, end, facing?, tangent?}
//         | {kind: 'turn', heading}
//         | {kind: 'wait', ms}

export const IN_PER_M = 1 / 0.0254;

export function isSegment(step) {
    return step.kind === 'line' || step.kind === 'bezier';
}

export function newRoute(start) {
    return { start: { x: start.x, y: start.y, heading: start.heading }, steps: [] };
}

function deg(rad) {
    return (rad * 180) / Math.PI;
}

function bezierAt(p0, c1, c2, p1, t) {
    const u = 1 - t;
    const a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
    return { x: a * p0.x + b * c1.x + c * c2.x + d * p1.x, y: a * p0.y + b * c1.y + c * c2.y + d * p1.y };
}

// n + 1 points along a segment that starts at from.
export function samplePoints(from, step, n = 24) {
    if (step.kind === 'line') {
        return [{ x: from.x, y: from.y }, { x: step.end.x, y: step.end.y }];
    }
    const out = [];
    for (let i = 0; i <= n; ++i) {
        out.push(bezierAt(from, step.c1, step.c2, step.end, i / n));
    }
    return out;
}

// Heading at the end of a segment that starts at from facing heading.
export function segmentEndHeading(from, heading, step) {
    if (typeof step.facing === 'number') {
        return step.facing;
    }
    if (step.tangent) {
        let d = step.kind === 'bezier' ? { x: step.end.x - step.c2.x, y: step.end.y - step.c2.y }
                                       : { x: step.end.x - from.x, y: step.end.y - from.y };
        if (Math.hypot(d.x, d.y) < 1e-9 && step.kind === 'bezier') {
            d = { x: step.end.x - from.x, y: step.end.y - from.y };
        }
        return Math.hypot(d.x, d.y) < 1e-9 ? heading : deg(Math.atan2(d.y, d.x));
    }
    return heading;
}

// Every step with where it starts and ends and which navigator step it is in
// (the index next() runs it at).
export function walk(route) {
    const out = [];
    let pos = { x: route.start.x, y: route.start.y };
    let heading = route.start.heading;
    let navStep = -1;
    let prevSegment = false;
    route.steps.forEach((step, i) => {
        const segment = isSegment(step);
        if (!segment || !prevSegment) {
            navStep += 1;
        }
        prevSegment = segment;
        const from = pos;
        const headingBefore = heading;
        if (segment) {
            heading = segmentEndHeading(from, heading, step);
            pos = { x: step.end.x, y: step.end.y };
        } else if (step.kind === 'turn') {
            heading = step.heading;
        }
        out.push({ index: i, step, navStep, from, to: pos, headingBefore, headingAfter: heading });
    });
    return out;
}

export function endPose(route) {
    const w = walk(route);
    if (w.length === 0) {
        return { ...route.start };
    }
    const last = w[w.length - 1];
    return { x: last.to.x, y: last.to.y, heading: last.headingAfter };
}

// --- C++ ---

export function fmt(v) {
    const r = Math.round(v * 10) / 10;
    return String(Object.is(r, -0) ? 0 : r);
}

function pt(p) {
    return `{${fmt(p.x)}, ${fmt(p.y)}}`;
}

export function toCode(route) {
    const s = route.start;
    const lines = [`navigatr::Route({${fmt(s.x)}, ${fmt(s.y)}, ${fmt(s.heading)}})`];
    for (const step of route.steps) {
        let call;
        if (step.kind === 'line') {
            call = `.line(${pt(step.end)})`;
        } else if (step.kind === 'bezier') {
            call = `.bezier(${pt(step.c1)}, ${pt(step.c2)}, ${pt(step.end)})`;
        } else if (step.kind === 'turn') {
            call = `.turnTo(${fmt(step.heading)})`;
        } else if (step.kind === 'wait') {
            call = `.wait(${Math.max(0, Math.round(step.ms))})`;
        }
        if (isSegment(step)) {
            if (typeof step.facing === 'number') {
                call += `.facing(${fmt(step.facing)})`;
            } else if (step.tangent) {
                call += '.tangent()';
            }
        }
        lines.push('    ' + call);
    }
    return lines.join('\n') + ';\n';
}

const NUM = /-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?/g;

function nums(text) {
    return (text.match(NUM) || []).map(Number);
}

// Reads code toCode() wrote, or hand-written code in the same shape. Returns
// {route, dropped} or {error}. at() actions are code, not geometry: they are
// counted in dropped and left out.
export function fromCode(text) {
    const startMatch = /Route\s*\(\s*\{([^}]*)\}\s*\)/.exec(text);
    if (!startMatch) {
        return { error: 'no Route({x, y, heading}) start found' };
    }
    const start = nums(startMatch[1]);
    if (start.length !== 3) {
        return { error: 'the start needs three numbers: x, y, heading' };
    }
    const route = newRoute({ x: start[0], y: start[1], heading: start[2] });
    const rest = text.slice(startMatch.index + startMatch[0].length);
    const dropped = (rest.match(/\.at\s*\(/g) || []).length;
    const call = /\.(line|bezier|facing|tangent|turnTo|wait)\s*\(([^()]*)\)/g;
    let m;
    while ((m = call.exec(rest)) !== null) {
        const [, name, args] = m;
        const v = nums(args);
        const last = route.steps[route.steps.length - 1];
        const want = { line: 2, bezier: 6, facing: 1, tangent: 0, turnTo: 1, wait: 1 }[name];
        if (v.length !== want) {
            return { error: `.${name}() needs ${want} numbers, got ${v.length}` };
        }
        if (name === 'line') {
            route.steps.push({ kind: 'line', end: { x: v[0], y: v[1] } });
        } else if (name === 'bezier') {
            route.steps.push({ kind: 'bezier', c1: { x: v[0], y: v[1] }, c2: { x: v[2], y: v[3] }, end: { x: v[4], y: v[5] } });
        } else if (name === 'turnTo') {
            route.steps.push({ kind: 'turn', heading: v[0] });
        } else if (name === 'wait') {
            route.steps.push({ kind: 'wait', ms: v[0] });
        } else if (last && isSegment(last)) {
            // facing() and tangent() set the last segment; after a turn or wait
            // navigatr ignores them too.
            if (name === 'facing') {
                last.facing = v[0];
                delete last.tangent;
            } else {
                last.tangent = true;
                delete last.facing;
            }
        }
    }
    return { route, dropped };
}

// --- field ---

// A spectagatr-field/1 document in inches: walls, tiles and object outlines.
export function fieldInInches(field) {
    const d = field.dimensions;
    const w = d.inside_x_m * IN_PER_M;
    const h = d.inside_y_m * IN_PER_M;
    const objects = [];
    for (const o of field.objects || []) {
        const x = o.x_m * IN_PER_M, y = o.y_m * IN_PER_M;
        const yaw = ((o.yaw_deg || 0) * Math.PI) / 180;
        let outline;
        if (o.kind === 'octagon') {
            const r = ((o.base_across_flats_m || 0) * IN_PER_M) / 2 / Math.cos(Math.PI / 8);
            outline = [];
            for (let i = 0; i < 8; ++i) {
                const a = yaw + Math.PI / 8 + (i * Math.PI) / 4;
                outline.push({ x: x + r * Math.cos(a), y: y + r * Math.sin(a) });
            }
        } else {
            const hx = ((o.size_x_m || 0) * IN_PER_M) / 2, hy = ((o.size_y_m || 0) * IN_PER_M) / 2;
            outline = [[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]].map(([u, v]) => ({
                x: x + u * Math.cos(yaw) - v * Math.sin(yaw),
                y: y + u * Math.sin(yaw) + v * Math.cos(yaw),
            }));
        }
        objects.push({ id: o.id, kind: o.kind, color: o.color, outline });
    }
    return { width: w, height: h, tile: (d.tile_m || 0) * IN_PER_M, walls: field.walls || {}, objects };
}

// Corners of a robot footprint at a pose, front first.
export function footprint(pose, length, width) {
    const h = (pose.heading * Math.PI) / 180;
    const c = Math.cos(h), s = Math.sin(h);
    return [[length / 2, width / 2], [-length / 2, width / 2], [-length / 2, -width / 2], [length / 2, -width / 2]].map(
        ([u, v]) => ({ x: pose.x + u * c - v * s, y: pose.y + u * s + v * c }));
}
