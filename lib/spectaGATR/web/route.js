// route.js
// Route editor page: draws the field and the route, edits it with the mouse
// and the side panel, writes navigatr::Route code. Data and math: route_model.js.

import {
    endPose, fieldInInches, fmt, footprint, fromCode, isSegment, newRoute, samplePoints, toCode, walk,
} from './route_model.js';

const $ = (id) => document.getElementById(id);
const SVG = 'http://www.w3.org/2000/svg';
const DRAFT_KEY = 'spectagatr.route.draft';
const FIELD_KEY = 'spectagatr.route.field';
const MARGIN = 6; // in around the field
const COLORS = ['#4fc3f7', '#ffb84d', '#c792ea', '#6fe0a0', '#ff8a8a']; // by next() step
const PLAIN_FIELD = { width: 140.4, height: 140.4, tile: 23.4, walls: {}, objects: [] };

let field = PLAIN_FIELD;
let route = newRoute({ x: 23.4, y: 23.4, heading: 0 });
let robot = { length: 14.5, width: 14.7 };
let addKind = 'line';
let selected = null; // {start: true} or {step: index}
let drag = null;     // {kind, step} while a handle moves
let press = null;    // {x, y} screen position of a press on the field

// --- storage (optional: blocked storage just means nothing is remembered) ---

function storeGet(key) {
    try {
        return localStorage.getItem(key);
    } catch (e) {
        return null;
    }
}

function storeSet(key, value) {
    try {
        localStorage.setItem(key, value);
    } catch (e) {
        // not remembered
    }
}

function saveDraft() {
    storeSet(DRAFT_KEY, JSON.stringify({ route, robot }));
}

function loadDraft() {
    try {
        const d = JSON.parse(storeGet(DRAFT_KEY) || 'null');
        if (d && d.route && d.route.start && Array.isArray(d.route.steps)) {
            route = d.route;
        }
        if (d && d.robot && d.robot.length > 0 && d.robot.width > 0) {
            robot = d.robot;
        }
    } catch (e) {
        // keep the defaults
    }
}

// --- svg helpers ---

const svg = $('field');
const geo = el('g', { transform: 'scale(1,-1)' }); // field inches, +y up
const text = el('g');                              // labels, upright
svg.append(geo, text);

function el(name, attrs = {}, parent = null) {
    const e = document.createElementNS(SVG, name);
    for (const [k, v] of Object.entries(attrs)) {
        e.setAttribute(k, v);
    }
    if (parent) {
        parent.appendChild(e);
    }
    return e;
}

function points(list) {
    return list.map((p) => `${p.x},${p.y}`).join(' ');
}

function label(x, y, str, color) {
    const t = el('text', { x, y: -y, fill: color, 'font-size': 4.2, 'font-weight': 700 }, text);
    t.textContent = str;
}

function toField(evt) {
    const p = new DOMPoint(evt.clientX, evt.clientY).matrixTransform(geo.getScreenCTM().inverse());
    return { x: p.x, y: p.y };
}

function snap(p) {
    const step = $('snap').checked ? 0.5 : 0.1;
    return { x: Math.round(p.x / step) * step, y: Math.round(p.y / step) * step };
}

// --- drawing ---

function drawField() {
    const { width: w, height: h } = field;
    svg.setAttribute('viewBox', `${-MARGIN} ${-(h + MARGIN)} ${w + 2 * MARGIN} ${h + 2 * MARGIN}`);
    el('rect', { x: 0, y: 0, width: w, height: h, fill: '#1b1f26' }, geo);
    if (field.tile > 0) {
        let d = '';
        for (let x = field.tile; x < w - 1e-6; x += field.tile) d += `M${x},0V${h}`;
        for (let y = field.tile; y < h - 1e-6; y += field.tile) d += `M0,${y}H${w}`;
        el('path', { d, stroke: '#2c323b', 'stroke-width': 0.3 }, geo);
    }
    const wall = (key, x, y, ww, hh) => {
        const color = (field.walls[key] && field.walls[key].color) || '#4a515c';
        el('rect', { x, y, width: ww, height: hh, fill: color }, geo);
    };
    wall('x_min', -2, -2, 2, h + 4);
    wall('x_max', w, -2, 2, h + 4);
    wall('y_min', 0, -2, w, 2);
    wall('y_max', 0, h, w, 2);
    for (const o of field.objects) {
        el('polygon', { points: points(o.outline), fill: o.color, 'fill-opacity': 0.35, stroke: o.color,
                        'stroke-width': 0.3 }, geo);
    }
    if (field.walls.y_min && field.walls.y_min.label) {
        const t = el('text', { x: w / 2, y: MARGIN - 1.5, fill: '#9aa3b2', 'font-size': 3.5, 'text-anchor': 'middle' },
                     text);
        t.textContent = field.walls.y_min.label;
    }
}

function drawRobot(pose, color, dashed) {
    const c = footprint(pose, robot.length, robot.width);
    el('polygon', { points: points(c), fill: 'none', stroke: color, 'stroke-width': 0.4, 'stroke-opacity': 0.8,
                    'stroke-dasharray': dashed ? '1.2 0.8' : 'none' }, geo);
    el('line', { x1: c[3].x, y1: c[3].y, x2: c[0].x, y2: c[0].y, stroke: color, 'stroke-width': 1.1 }, geo);
}

function handle(p, kind, step, color, size) {
    const isSel = selected && (kind === 'start' ? selected.start : selected.step === step);
    const attrs = { class: 'handle', 'data-kind': kind, 'data-step': step, fill: color,
                    stroke: isSel ? '#ffe066' : '#14171c', 'stroke-width': isSel ? 0.7 : 0.4 };
    if (kind === 'c1' || kind === 'c2') {
        el('rect', { ...attrs, x: p.x - size, y: p.y - size, width: 2 * size, height: 2 * size }, geo);
    } else {
        el('circle', { ...attrs, cx: p.x, cy: p.y, r: size }, geo);
    }
}

function drawRoute() {
    const walked = walk(route);
    const s = route.start;
    drawRobot(s, '#e6e9ef', true);

    for (const w of walked) {
        if (w.marker) {
            continue;
        }
        const color = COLORS[w.navStep % COLORS.length];
        if (isSegment(w.step)) {
            el('polyline', { points: points(samplePoints(w.from, w.step)), fill: 'none', stroke: color,
                             'stroke-width': 0.9, 'stroke-linejoin': 'round' }, geo);
        }
        drawRobot({ x: w.to.x, y: w.to.y, heading: w.headingAfter }, color, false);
    }

    // Handles on top of every line and outline.
    for (const w of walked) {
        if (w.step.kind !== 'bezier') {
            continue;
        }
        for (const [a, b] of [[w.from, w.step.c1], [w.step.c2, w.step.end]]) {
            el('line', { x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke: '#9aa3b2', 'stroke-width': 0.25,
                         'stroke-dasharray': '1 0.7' }, geo);
        }
        handle(w.step.c1, 'c1', w.index, '#9aa3b2', 1.0);
        handle(w.step.c2, 'c2', w.index, '#9aa3b2', 1.0);
    }
    for (const w of walked) {
        if (isSegment(w.step)) {
            handle(w.step.end, 'end', w.index, COLORS[w.navStep % COLORS.length], 1.5);
        }
    }
    handle(s, 'start', -1, '#3ddc84', 1.7);
    const h = (s.heading * Math.PI) / 180;
    el('line', { x1: s.x, y1: s.y, x2: s.x + 6 * Math.cos(h), y2: s.y + 6 * Math.sin(h), stroke: '#3ddc84',
                 'stroke-width': 0.6 }, geo);

    // One number per next() step, where it ends.
    const ends = new Map();
    for (const w of walked) {
        if (!w.marker) {
            ends.set(w.navStep, w);
        }
    }
    const used = new Map(); // labels already at a spot, stacked downward
    for (const [n, w] of ends) {
        const key = `${fmt(w.to.x)},${fmt(w.to.y)}`;
        const k = used.get(key) || 0;
        used.set(key, k + 1);
        label(w.to.x + 2, w.to.y + 2 - 4.5 * k, String(n + 1), COLORS[n % COLORS.length]);
    }
}

function describe(step) {
    let s;
    if (step.kind === 'line') s = `line to (${fmt(step.end.x)}, ${fmt(step.end.y)})`;
    else if (step.kind === 'bezier') s = `curve to (${fmt(step.end.x)}, ${fmt(step.end.y)})`;
    else if (step.kind === 'turn') s = `turn to ${fmt(step.heading)}`;
    else s = 'new path';
    if (isSegment(step)) {
        if (typeof step.facing === 'number') s += `, facing ${fmt(step.facing)}`;
        else if (step.tangent) s += ', tangent';
    }
    return s;
}

function drawSteps() {
    const list = $('steps');
    list.replaceChildren();
    for (const w of walk(route)) {
        const li = document.createElement('li');
        li.className = selected && selected.step === w.index ? 'selected' : '';
        const nav = document.createElement('span');
        nav.className = 'nav';
        nav.textContent = String(w.navStep + 1);
        nav.style.color = COLORS[w.navStep % COLORS.length];
        li.append(nav, describe(w.step));
        li.addEventListener('click', () => select({ step: w.index }));
        list.appendChild(li);
    }
    if (route.steps.length === 0) {
        const li = document.createElement('li');
        li.className = 'muted';
        li.textContent = 'Click the field to add the first point.';
        list.appendChild(li);
    }
}

function numberInput(value, onChange, step = 0.5) {
    const i = document.createElement('input');
    i.type = 'number';
    i.step = String(step);
    i.value = value === undefined || value === null ? '' : fmt(value);
    i.addEventListener('change', () => onChange(i.value === '' ? null : Number(i.value)));
    return i;
}

function labelled(name, input) {
    const l = document.createElement('label');
    l.append(name + ' ', input);
    return l;
}

function button(name, onClick) {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = name;
    b.addEventListener('click', onClick);
    return b;
}

function drawSelected() {
    const box = $('selected');
    box.replaceChildren();
    if (!selected || selected.start) {
        box.className = 'muted';
        box.textContent = selected ? 'Start pose: edit it above or drag the green point.' : 'Click a point or a step.';
        return;
    }
    box.className = '';
    const i = selected.step;
    const step = route.steps[i];
    const row = () => {
        const d = document.createElement('div');
        d.className = 'row';
        box.appendChild(d);
        return d;
    };
    const title = document.createElement('div');
    title.textContent = describe(step);
    box.appendChild(title);

    if (isSegment(step)) {
        row().append(
            button(step.kind === 'line' ? 'Make curve' : 'Make line', () => {
                const from = walk(route)[i].from;
                if (step.kind === 'line') {
                    const d = { x: step.end.x - from.x, y: step.end.y - from.y };
                    step.kind = 'bezier';
                    step.c1 = snap({ x: from.x + d.x / 3, y: from.y + d.y / 3 });
                    step.c2 = snap({ x: from.x + (2 * d.x) / 3, y: from.y + (2 * d.y) / 3 });
                } else {
                    step.kind = 'line';
                    delete step.c1;
                    delete step.c2;
                }
                changed();
            }));
        row().append(
            labelled('x', numberInput(step.end.x, (v) => { if (v !== null) { step.end.x = v; changed(); } })),
            labelled('y', numberInput(step.end.y, (v) => { if (v !== null) { step.end.y = v; changed(); } })));
        const tangent = document.createElement('input');
        tangent.type = 'checkbox';
        tangent.checked = !!step.tangent;
        tangent.addEventListener('change', () => {
            if (tangent.checked) {
                step.tangent = true;
                delete step.facing;
            } else {
                delete step.tangent;
            }
            changed();
        });
        row().append(
            labelled('facing', numberInput(step.facing, (v) => {
                if (v === null) {
                    delete step.facing;
                } else {
                    step.facing = v;
                    delete step.tangent;
                }
                changed();
            }, 5)),
            labelled('tangent', tangent));
        const note = document.createElement('div');
        note.className = 'muted small';
        note.textContent = 'facing empty and tangent off: the heading holds.';
        box.appendChild(note);
    } else if (step.kind === 'turn') {
        row().append(labelled('heading', numberInput(step.heading, (v) => {
            if (v !== null) { step.heading = v; changed(); }
        }, 5)));
    } else {
        const note = document.createElement('div');
        note.className = 'muted small';
        note.textContent = 'The robot stops and settles here; the next point starts a new path. Delete to join them.';
        box.appendChild(note);
    }
    row().append(button('Delete step', () => removeStep(i)));
}

function drawCode() {
    const code = $('code');
    if (document.activeElement !== code) {
        code.value = toCode(route);
    }
}

function drawStart() {
    for (const [id, v] of [['start-x', route.start.x], ['start-y', route.start.y], ['start-h', route.start.heading],
                           ['robot-l', robot.length], ['robot-w', robot.width]]) {
        if (document.activeElement !== $(id)) {
            $(id).value = fmt(v);
        }
    }
}

function render() {
    geo.replaceChildren();
    text.replaceChildren();
    drawField();
    drawRoute();
    drawSteps();
    drawCode();
    drawStart();
}

// --- editing ---

function changed() {
    saveDraft();
    render();
    drawSelected();
}

function select(sel) {
    selected = sel;
    render();
    drawSelected();
}

function removeStep(i) {
    route.steps.splice(i, 1);
    selected = null;
    changed();
}

function addSegment(p) {
    const from = endPose(route);
    if (addKind === 'line') {
        route.steps.push({ kind: 'line', end: p });
    } else {
        const h = (from.heading * Math.PI) / 180;
        const d = Math.hypot(p.x - from.x, p.y - from.y);
        route.steps.push({
            kind: 'bezier',
            c1: snap({ x: from.x + (d / 3) * Math.cos(h), y: from.y + (d / 3) * Math.sin(h) }),
            c2: snap({ x: p.x - (p.x - from.x) / 3, y: p.y - (p.y - from.y) / 3 }),
            end: p,
        });
    }
    selected = { step: route.steps.length - 1 };
    changed();
}

svg.addEventListener('pointerdown', (evt) => {
    if (evt.button !== 0) {
        return;
    }
    const kind = evt.target.getAttribute && evt.target.getAttribute('data-kind');
    if (kind) {
        const step = Number(evt.target.getAttribute('data-step'));
        drag = { kind, step };
        selected = kind === 'start' ? { start: true } : { step };
        drawSelected();
    } else {
        press = { x: evt.clientX, y: evt.clientY };
    }
    svg.setPointerCapture(evt.pointerId);
});

svg.addEventListener('pointermove', (evt) => {
    const raw = toField(evt);
    $('cursor').textContent = `x ${raw.x.toFixed(1)}  y ${raw.y.toFixed(1)}`;
    if (!drag) {
        return;
    }
    const p = snap(raw);
    if (drag.kind === 'start') {
        route.start.x = p.x;
        route.start.y = p.y;
    } else {
        route.steps[drag.step][drag.kind] = p;
    }
    render();
});

svg.addEventListener('pointerup', (evt) => {
    if (drag) {
        drag = null;
        changed();
    } else if (press && Math.hypot(evt.clientX - press.x, evt.clientY - press.y) < 4) {
        addSegment(snap(toField(evt)));
    }
    press = null;
});

svg.addEventListener('pointerleave', () => {
    $('cursor').textContent = '';
});

for (const b of document.querySelectorAll('button[data-add]')) {
    b.addEventListener('click', () => {
        addKind = b.dataset.add;
        for (const o of document.querySelectorAll('button[data-add]')) {
            o.classList.toggle('active', o === b);
        }
    });
}

$('btn-turn').addEventListener('click', () => {
    route.steps.push({ kind: 'turn', heading: (endPose(route).heading + 90) % 360 });
    selected = { step: route.steps.length - 1 };
    changed();
});

$('btn-path').addEventListener('click', () => {
    const last = route.steps[route.steps.length - 1];
    if (last && isSegment(last)) { // after a turn the next point already starts one
        route.steps.push({ kind: 'path' });
        selected = null;
        changed();
    }
});

$('btn-undo').addEventListener('click', () => {
    if (route.steps.length > 0) {
        removeStep(route.steps.length - 1);
    }
});

$('btn-clear').addEventListener('click', () => {
    route.steps = [];
    selected = null;
    changed();
});

for (const [id, set] of [
    ['start-x', (v) => { route.start.x = v; }],
    ['start-y', (v) => { route.start.y = v; }],
    ['start-h', (v) => { route.start.heading = v; }],
    ['robot-l', (v) => { if (v > 0) robot.length = v; }],
    ['robot-w', (v) => { if (v > 0) robot.width = v; }],
]) {
    $(id).addEventListener('change', () => {
        const v = Number($(id).value);
        if ($(id).value !== '' && Number.isFinite(v)) {
            set(v);
        }
        changed();
    });
}

$('btn-copy').addEventListener('click', async () => {
    const code = toCode(route);
    try {
        await navigator.clipboard.writeText(code);
        $('code-note').textContent = 'copied';
    } catch (e) {
        $('code').select();
        $('code-note').textContent = 'selected: press Ctrl+C';
    }
});

$('btn-load').addEventListener('click', () => {
    const result = fromCode($('code').value);
    if (result.error) {
        $('code-note').textContent = result.error;
        return;
    }
    route = result.route;
    selected = null;
    $('code-note').textContent =
        result.dropped > 0 ? `loaded; ${result.dropped} at(), wait() or follow() call(s) left out` : 'loaded';
    $('code').blur();
    changed();
});

document.addEventListener('keydown', (evt) => {
    const typing = evt.target instanceof HTMLInputElement || evt.target instanceof HTMLTextAreaElement;
    if (!typing && (evt.key === 'Delete' || evt.key === 'Backspace') && selected && !selected.start) {
        removeStep(selected.step);
        evt.preventDefault();
    }
});

// --- field ---

async function loadField(name) {
    try {
        const r = await fetch('fields/' + encodeURIComponent(name) + '.json', { cache: 'no-store' });
        if (!r.ok) {
            throw new Error('HTTP ' + r.status);
        }
        const f = await r.json();
        if (!f || f.schema !== 'spectagatr-field/1' || !f.dimensions) {
            throw new Error('not a spectagatr-field/1 document');
        }
        field = fieldInInches(f);
        document.body.dataset.field = name;
    } catch (e) {
        field = PLAIN_FIELD;
        document.body.dataset.field = '';
    }
    render();
}

async function loadFields() {
    const select = $('field-select');
    let names = [];
    let fallback = null;
    try {
        const r = await fetch('api/fields', { cache: 'no-store' });
        const j = await r.json();
        names = Array.isArray(j.fields) ? j.fields.filter((s) => typeof s === 'string') : [];
        fallback = typeof j.default === 'string' ? j.default : null;
    } catch (e) {
        names = [];
    }
    select.replaceChildren();
    if (names.length === 0) {
        const o = document.createElement('option');
        o.value = '';
        o.textContent = '(plain 140.4 in square)';
        select.appendChild(o);
        render();
        return;
    }
    for (const n of names) {
        const o = document.createElement('option');
        o.value = n;
        o.textContent = n;
        select.appendChild(o);
    }
    const stored = storeGet(FIELD_KEY);
    const want = names.includes(stored) ? stored : names.includes('override') ? 'override' : fallback || names[0];
    select.value = want;
    await loadField(want);
}

$('field-select').addEventListener('change', (e) => {
    if (e.target.value) {
        storeSet(FIELD_KEY, e.target.value);
        loadField(e.target.value);
    }
});

loadDraft();
render();
loadFields();
