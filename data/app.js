/**
 * app.js — BOB-CONTROL : Tableau de bord avec tuiles réorganisables.
 *
 * Navigation : clic sur une tuile → affiche la section correspondante.
 * Drag & drop : mode édition pour réorganiser les tuiles (persisté en localStorage).
 * WebSocket : télémétrie temps réel.
 */

"use strict";

/* =========================================================================
 * CONSTANTES
 * ========================================================================= */

const CH_NAMES = [
    "Thruster 1", "Thruster 2", "Thruster 3", "Thruster 4",
    "Thruster 5", "Thruster 6", "Thruster 7", "Thruster 8",
    "Pan Servo",  "Tilt Servo", "Claw",       "Claw Rot.",
    "LED Dim 1",  "LED Dim 2",  "Aux 1",      "Aux 2"
];

const CH_TYPES = [
    "motor","motor","motor","motor","motor","motor","motor","motor",
    "servo","servo","servo","servo",
    "dimmer","dimmer","aux","aux"
];

const NEUTRAL_US = 1500;
const MIN_US = 1000;
const MAX_US = 2000;

let pwmValues = new Array(16).fill(NEUTRAL_US);
let ws = null;
let testUnlocked = false;

/* =========================================================================
 * NAVIGATION PAR TUILES
 * ========================================================================= */

function initTileNav() {
    const dashboard = document.getElementById('dashboard');
    const btnBack = document.getElementById('btn-back');
    const sections = document.querySelectorAll('.section-panel');
    const tiles = document.querySelectorAll('.nav-tile');

    tiles.forEach(tile => {
        tile.addEventListener('click', (e) => {
            // Bloquer la navigation en mode édition
            const grid = document.getElementById('nav-grid');
            if (grid && grid.classList.contains('editing')) {
                e.preventDefault();
                return;
            }
            e.preventDefault();
            const targetId = tile.getAttribute('href').substring(1); // "#section-wifi" → "section-wifi"
            showSection(targetId);
        });
    });

    // Bouton retour → dashboard
    if (btnBack) {
        btnBack.addEventListener('click', (e) => {
            e.preventDefault();
            showDashboard();
        });
    }

    function showSection(id) {
        dashboard.style.display = 'none';
        sections.forEach(s => s.style.display = 'none');
        const target = document.getElementById(id);
        if (target) target.style.display = 'block';
        if (btnBack) btnBack.style.display = 'flex';
        window.scrollTo(0, 0);
    }

    function showDashboard() {
        sections.forEach(s => s.style.display = 'none');
        dashboard.style.display = 'flex';
        if (btnBack) btnBack.style.display = 'none';
        window.scrollTo(0, 0);
    }
}

/* =========================================================================
 * DASHBOARD EDITOR — DRAG & DROP
 * ========================================================================= */

function initDashboardEditor() {
    const grid = document.getElementById('nav-grid');
    const btnEdit = document.getElementById('btnEditDash');
    const btnSave = document.getElementById('btnSaveDash');
    const hint = document.getElementById('dashEditHint');
    if (!grid || !btnEdit || !btnSave) return;

    // Charger l'ordre sauvegardé
    loadDashboardOrder(grid);

    let editing = false;

    btnEdit.addEventListener('click', () => {
        editing = true;
        grid.classList.add('editing');
        btnEdit.style.display = 'none';
        btnSave.style.display = 'inline-flex';
        hint.style.display = 'inline';
        attachDragHandlers(grid);
    });

    btnSave.addEventListener('click', () => {
        editing = false;
        grid.classList.remove('editing');
        grid.classList.add('no-enter-anim');
        btnEdit.style.display = 'inline-flex';
        btnSave.style.display = 'none';
        hint.style.display = 'none';
        detachDragHandlers(grid);
        saveDashboardOrder(grid);
        setTimeout(() => grid.classList.remove('no-enter-anim'), 100);
    });
}

function loadDashboardOrder(grid) {
    try {
        const cached = localStorage.getItem('bobcontrol_dashOrder');
        if (cached) applyOrder(grid, JSON.parse(cached));
    } catch (_) {}
}

function saveDashboardOrder(grid) {
    const order = Array.from(grid.querySelectorAll('.nav-tile')).map(t => t.dataset.tileId);
    try { localStorage.setItem('bobcontrol_dashOrder', JSON.stringify(order)); } catch (_) {}
}

function applyOrder(grid, order) {
    if (!Array.isArray(order)) return;
    grid.classList.add('no-enter-anim');
    const tiles = Array.from(grid.querySelectorAll('.nav-tile'));
    const byId = {};
    tiles.forEach(t => { byId[t.dataset.tileId] = t; });
    order.forEach(id => {
        if (byId[id]) grid.appendChild(byId[id]);
    });
    setTimeout(() => grid.classList.remove('no-enter-anim'), 50);
}

/* ----- Drag & Drop via Pointer Events ----- */
let _drag = null;

function attachDragHandlers(grid) {
    grid.querySelectorAll('.nav-tile').forEach(tile => {
        tile.addEventListener('pointerdown', onPointerDown);
        tile.addEventListener('click', clickGuard);
        tile.addEventListener('dragstart', dragStartGuard);
        tile.addEventListener('touchstart', touchGuard, { passive: false });
        tile.addEventListener('touchmove', touchGuard, { passive: false });
    });
}

function detachDragHandlers(grid) {
    grid.querySelectorAll('.nav-tile').forEach(tile => {
        tile.removeEventListener('pointerdown', onPointerDown);
        tile.removeEventListener('click', clickGuard);
        tile.removeEventListener('dragstart', dragStartGuard);
        tile.removeEventListener('touchstart', touchGuard);
        tile.removeEventListener('touchmove', touchGuard);
        tile.classList.remove('drop-before', 'drop-after', 'dragging-ghost');
    });
}

function clickGuard(e) { e.preventDefault(); }
/* Bloque le drag-and-drop natif des ancres <a href> qui interrompt les Pointer Events */
function dragStartGuard(e) { e.preventDefault(); }
function touchGuard(e) {
    const grid = e.currentTarget && e.currentTarget.parentElement;
    if (grid && grid.classList.contains('editing')) e.preventDefault();
}

function onPointerDown(e) {
    if (e.button !== undefined && e.button !== 0) return;
    e.preventDefault();
    const tile = e.currentTarget;
    /* Capturer le pointeur : garantit la réception des pointermove et empêche le drag natif */
    try { tile.setPointerCapture(e.pointerId); } catch (_) {}
    const grid = tile.parentElement;
    const rect = tile.getBoundingClientRect();

    // Créer un clone flottant
    const floater = tile.cloneNode(true);
    floater.style.cssText = `position:fixed;z-index:99999;pointer-events:none;
        width:${rect.width}px;height:${rect.height}px;
        opacity:0.85;transform:scale(1.05);
        box-shadow:0 15px 40px rgba(0,0,0,0.5);
        left:${rect.left}px;top:${rect.top}px;transition:none;`;
    document.body.appendChild(floater);
    tile.classList.add('dragging-ghost');

    _drag = {
        grid, sourceTile: tile, floater,
        offsetX: e.clientX - rect.left,
        offsetY: e.clientY - rect.top,
        pointerId: e.pointerId,
        moved: false
    };

    document.addEventListener('pointermove', onPointerMove);
    document.addEventListener('pointerup', onPointerUp);
    document.addEventListener('pointercancel', onPointerUp);
}

function onPointerMove(e) {
    if (!_drag) return;
    if (e.cancelable) e.preventDefault();
    const d = _drag;
    d.moved = true;
    d.floater.style.left = (e.clientX - d.offsetX) + 'px';
    d.floater.style.top = (e.clientY - d.offsetY) + 'px';

    // Trouver la tuile la plus proche du pointeur
    const tiles = Array.from(d.grid.querySelectorAll('.nav-tile'));
    tiles.forEach(t => t.classList.remove('drop-before', 'drop-after'));

    let closest = null, minDist = Infinity;
    tiles.forEach(t => {
        if (t === d.sourceTile) return;
        const r = t.getBoundingClientRect();
        const cx = r.left + r.width / 2;
        const cy = r.top + r.height / 2;
        const dist = Math.hypot(e.clientX - cx, e.clientY - cy);
        if (dist < minDist) { minDist = dist; closest = t; }
    });

    if (closest && minDist < 200) {
        const r = closest.getBoundingClientRect();
        const before = e.clientX < r.left + r.width / 2;
        closest.classList.add(before ? 'drop-before' : 'drop-after');

        // Live reorder
        const desiredNext = before ? closest : closest.nextElementSibling;
        if (d.sourceTile.nextElementSibling !== desiredNext && d.sourceTile !== desiredNext) {
            d.grid.insertBefore(d.sourceTile, desiredNext);
        }
    }
}

function onPointerUp(e) {
    if (!_drag) return;
    const d = _drag;

    d.grid.querySelectorAll('.nav-tile').forEach(t => {
        t.classList.remove('drop-before', 'drop-after');
    });
    d.sourceTile.classList.remove('dragging-ghost');
    if (d.floater && d.floater.parentNode) d.floater.parentNode.removeChild(d.floater);
    try { d.sourceTile.releasePointerCapture && d.sourceTile.releasePointerCapture(d.pointerId); } catch (_) {}

    document.removeEventListener('pointermove', onPointerMove);
    document.removeEventListener('pointerup', onPointerUp);
    document.removeEventListener('pointercancel', onPointerUp);

    _drag = null;
}

/* =========================================================================
 * WEBSOCKET
 * ========================================================================= */

function connectWebSocket() {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${proto}//${location.host}/ws`);

    ws.onopen = () => {
        document.getElementById('badge-ws').textContent = 'WS: Connecté';
        document.getElementById('badge-ws').className = 'badge badge-ok';
    };

    ws.onclose = () => {
        document.getElementById('badge-ws').textContent = 'WS: Déconnecté';
        document.getElementById('badge-ws').className = 'badge badge-off';
        setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = () => { ws.close(); };

    ws.onmessage = (event) => {
        try { processTelemetry(JSON.parse(event.data)); }
        catch (e) { console.warn('[WS] JSON invalide:', e); }
    };
}

function processTelemetry(data) {
    /* Informations Wi-Fi temps réel (STA + AP) */
    if (data.wifi_info) {
        const w = data.wifi_info;
        const staEl = document.getElementById('wifi-sta-status');
        if (staEl) {
            if (w.sta_connected) {
                staEl.textContent = 'Connecté';
                staEl.className = 'wifi-info-value wifi-sta-ok';
            } else {
                staEl.textContent = 'Déconnecté';
                staEl.className = 'wifi-info-value wifi-sta-off';
            }
        }
        setText('wifi-sta-ssid', w.sta_ssid || '—');
        setText('wifi-sta-ip',   w.sta_ip || '—');
        setText('wifi-sta-gw',   w.sta_gateway || '—');
        setText('wifi-sta-mask', w.sta_mask || '—');
        const rssiEl = document.getElementById('wifi-sta-rssi');
        if (rssiEl) {
            if (w.sta_connected) {
                const rssi = w.sta_rssi || 0;
                let quality = 'Faible';
                if (rssi > -50) quality = 'Excellent';
                else if (rssi > -60) quality = 'Bon';
                else if (rssi > -70) quality = 'Moyen';
                rssiEl.textContent = rssi + ' dBm (' + quality + ')';
            } else {
                rssiEl.textContent = '—';
            }
        }
        setText('wifi-sta-mac', w.sta_mac || '—');
        setText('wifi-ap-ssid', w.ap_ssid || '—');
        setText('wifi-ap-ip', w.ap_ip || '—');
        setText('wifi-ap-clients', w.ap_clients !== undefined ? String(w.ap_clients) : '—');
        /* Mettre à jour aussi l'IP dans la barre de statut Wi-Fi */
        const ipEl = document.getElementById('wifi-ip');
        if (ipEl) {
            ipEl.textContent = w.sta_connected ? (w.sta_ip || '—') : (w.ap_ip || '—');
        }
    }

    if (data.wdg !== undefined) {
        const el = document.getElementById('badge-wdg');
        el.textContent = data.wdg ? 'WDG: ALERTE' : 'WDG: OK';
        el.className = data.wdg ? 'badge badge-off' : 'badge badge-ok';
    }

    if (data.pwm && Array.isArray(data.pwm)) {
        for (let i = 0; i < 16 && i < data.pwm.length; i++) {
            pwmValues[i] = data.pwm[i];
            updatePWMBar(i, data.pwm[i]);
        }
        /* Vue schématique 2D des propulseurs (M1–M8) */
        updateROVSchematic(pwmValues);
    }

    if (data.imu) {
        const roll  = data.imu.roll  || 0;
        const pitch = data.imu.pitch || 0;
        const yaw   = data.imu.yaw   || 0;
        setText('val-roll',  roll.toFixed(1) + '°');
        setText('val-pitch', pitch.toFixed(1) + '°');
        setText('val-yaw',   yaw.toFixed(1) + '°');
        /* Envoi vers le lissage LERP (pas de dessin direct) */
        avSetTargets(roll, pitch, yaw);
        setText('av-roll',  roll.toFixed(1) + '°');
        setText('av-pitch', pitch.toFixed(1) + '°');
        setText('av-hdg', String(Math.round(yaw) % 360).padStart(3, '0') + '°');
    }

    if (data.press) {
        setText('val-mbar',  (data.press.mbar  || 0).toFixed(1) + ' mbar');
        setText('val-alt',    (data.press.alt   || 0).toFixed(2) + ' m');
        setText('val-temp',  (data.press.temp  || 0).toFixed(1) + ' °C');
        updateVSI(data.press.alt || 0);
    }

    if (data.power && Array.isArray(data.power)) {
        for (let i = 0; i < 3 && i < data.power.length; i++) {
            setText('p' + i + '-v', data.power[i].v + ' mV');
            setText('p' + i + '-i', data.power[i].i + ' mA');
        }
    }

    /* Statut capteurs (badges LED) */
    if (data.sensors) {
        updateSensorBadge('sens-bno085',  data.sensors.bno085);
        updateSensorBadge('sens-ms5803',  data.sensors.ms5803);
        updateSensorBadge('sens-ina0',    data.sensors.ina0);
        updateSensorBadge('sens-ina1',    data.sensors.ina1);
        updateSensorBadge('sens-ina2',    data.sensors.ina2);
        updateSensorBadge('sens-pca',     data.sensors.pca);
    }

    /* Statut liaison RPi5 (badge header + page Câblage) */
    if (data.rpi_link) {
        const el = document.getElementById('badge-rpi');
        if (el) {
            const label = data.rpi_link.interface || '--';
            const connected = data.rpi_link.connected;
            el.textContent = 'RPi [' + label + ']: ' + (connected ? 'OK' : 'PERDU');
            el.className = 'badge ' + (connected ? 'badge-ok' : 'badge-off');
        }
        /* Mettre à jour la page Câblage : afficher le bon schéma selon le mode */
        updateWiringMode(data.rpi_link.interface);
    }

    /* Sorties physiques Dry-Run (synchronisation UI ← ESP32) */
    if (data.physical_outputs_enabled !== undefined) {
        updateDryRunUI(data.physical_outputs_enabled);
    }

    /* Contrôleur de vol : mode actif + maître */
    if (data.flight_mode !== undefined) {
        updateFlightMode(data.flight_mode);
    }
    if (data.rpi_master !== undefined) {
        updateMasterBadge(data.rpi_master);
    }
}

/**
 * Met à jour l'indicateur LED d'un capteur.
 * 0 = déconnecté (rouge), 1 = connecté (vert), 2 = émulé (jaune)
 */
function updateSensorBadge(id, status) {
    const el = document.getElementById(id);
    if (!el) return;
    const led = el.querySelector('.led');
    if (!led) return;
    led.className = 'led ' + (status === 1 ? 'led-ok' : status === 2 ? 'led-emu' : 'led-fail');
}

/* =========================================================================
 * PWM MONITOR
 * ========================================================================= */

function initPWMGrid() {
    const grid = document.querySelector('.pwm-grid');
    if (!grid) return;
    grid.innerHTML = '';

    for (let i = 0; i < 16; i++) {
        const ch = document.createElement('div');
        ch.className = 'pwm-channel';
        const isMotor = (CH_TYPES[i] === 'motor');
        ch.innerHTML = `
            <div class="ch-header">
                <span class="ch-label">CH${i}: ${CH_NAMES[i]}</span>
                <span class="ch-value" id="pwm-val-${i}">${NEUTRAL_US} µs</span>
            </div>
            <div class="${isMotor ? 'pwm-bar-bg bidir' : 'pwm-bar-bg'}" id="pwm-bar-bg-${i}">
                ${isMotor ? '<div class="pwm-center"></div>' : ''}
                <div class="pwm-bar-fill" id="pwm-fill-${i}"></div>
            </div>`;
        grid.appendChild(ch);
    }

    for (let i = 0; i < 16; i++) updatePWMBar(i, NEUTRAL_US);
}

function updatePWMBar(ch, us) {
    const valEl = document.getElementById(`pwm-val-${ch}`);
    const fillEl = document.getElementById(`pwm-fill-${ch}`);
    if (!valEl || !fillEl) return;
    valEl.textContent = us + ' µs';

    const isMotor = (CH_TYPES[ch] === 'motor');
    const range = MAX_US - MIN_US;

    if (isMotor) {
        const dev = (us - NEUTRAL_US) / (range / 2);
        const pct = Math.abs(dev) * 50;
        if (dev >= 0) {
            fillEl.style.left = '50%';
            fillEl.style.width = pct + '%';
            fillEl.style.background = dev > 0.5 ? '#ff3e9d' : '#00d9ff';
        } else {
            fillEl.style.left = (50 - pct) + '%';
            fillEl.style.width = pct + '%';
            fillEl.style.background = dev < -0.5 ? '#ff3b30' : '#00d9ff';
        }
    } else {
        const pct = ((us - MIN_US) / range) * 100;
        fillEl.style.width = Math.max(0, Math.min(100, pct)) + '%';
        fillEl.style.background = '#00d9ff';
    }
}

/* =========================================================================
 * VUE SCHÉMATIQUE 2D — PROPULSION (PWM MONITOR)
 * ========================================================================= */

/*
 * Géométrie des 8 propulseurs dans le viewBox SVG 480×400 (vue de dessus,
 * avant = haut). Le schéma est construit dynamiquement par initROVSchematic().
 *
 *  - type 'H' : propulseur horizontal en configuration X. angleDeg = direction
 *    du vecteur de poussée POSITIVE (> 1500 µs) dans le repère SVG (0° = droite,
 *    90° = bas, sens horaire). La poussée négative (< 1500 µs) inverse le
 *    vecteur de 180°.
 *      Axe 135° (avant-gauche ↔ arrière-droite) :
 *        M1 Av-D : 225° → poussée positive vers l'AVANT-GAUCHE
 *        M3 Ar-G : 225° → poussée positive vers l'AVANT-GAUCHE
 *      Axe 45° (avant-droite ↔ arrière-gauche) :
 *        M2 Ar-D : 315° → poussée positive vers l'AVANT-DROITE
 *        M4 Av-G : 315° → poussée positive vers l'AVANT-DROITE
 *    Cohérent avec le mixage de flight_controller.cpp : les 4 positifs font
 *    avancer le ROV ; Yaw : M1/M2 en négatif + M3/M4 en positif → rotation CW.
 *    Pour inverser le sens réel d'un propulseur : ajouter 180 à son angleDeg.
 *  - type 'V' : propulseur vertical (cercle vu de dessus). Poussée positive =
 *    flèche vers le HAUT (ROV poussé vers la surface), négative = vers le bas.
 *
 * (lx, ly) = position absolue du label M1..M8, (vx, vy) = valeur en µs.
 */
const ROV_THRUSTERS = [
    { ch: 0, label: 'M1', type: 'H', x: 346, y:  81, angleDeg: 225, lx: 418, ly:  77, vx: 418, vy:  92 },
    { ch: 1, label: 'M2', type: 'H', x: 346, y: 319, angleDeg: 315, lx: 418, ly: 315, vx: 418, vy: 330 },
    { ch: 2, label: 'M3', type: 'H', x: 134, y: 319, angleDeg: 225, lx:  62, ly: 315, vx:  62, vy: 330 },
    { ch: 3, label: 'M4', type: 'H', x: 134, y:  81, angleDeg: 315, lx:  62, ly:  77, vx:  62, vy:  92 },
    { ch: 4, label: 'M5', type: 'V', x: 275, y: 157, lx: 275, ly: 129, vx: 275, vy: 190 },
    { ch: 5, label: 'M6', type: 'V', x: 275, y: 243, lx: 275, ly: 215, vx: 275, vy: 286 },
    { ch: 6, label: 'M7', type: 'V', x: 205, y: 243, lx: 205, ly: 215, vx: 205, vy: 286 },
    { ch: 7, label: 'M8', type: 'V', x: 205, y: 157, lx: 205, ly: 129, vx: 205, vy: 190 }
];

const ROV_NEUTRAL_COLOR = '#666b7a';   /* Gris neutre (proposé : --text-dim) */
const rovThrEls = [];                  /* Références DOM des propulseurs (index = canal PWM) */

/**
 * Crée un élément SVG dans le namespace SVG correct.
 * @param {string} tag Nom du tag (rect, circle, line...).
 * @param {Object} attrs Attributs à appliquer (optionnel).
 * @param {SVGElement} parent Élément parent (optionnel).
 * @returns {SVGElement} Élément créé.
 */
function svgNew(tag, attrs, parent) {
    const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
    if (attrs) {
        for (const k in attrs) el.setAttribute(k, attrs[k]);
    }
    if (parent) parent.appendChild(el);
    return el;
}

/**
 * Interpole linéairement deux couleurs RGB. t ∈ [0..1].
 * @param {number[]} c1 Couleur de départ [r, g, b].
 * @param {number[]} c2 Couleur d'arrivée [r, g, b].
 * @param {number} t Facteur d'interpolation.
 * @returns {string} Couleur 'rgb(r,g,b)'.
 */
function lerpColor(c1, c2, t) {
    const r = Math.round(c1[0] + (c2[0] - c1[0]) * t);
    const g = Math.round(c1[1] + (c2[1] - c1[1]) * t);
    const b = Math.round(c1[2] + (c2[2] - c1[2]) * t);
    return 'rgb(' + r + ',' + g + ',' + b + ')';
}

/**
 * Construit le schéma SVG du ROV (châssis + 8 propulseurs) dans #rov-schematic.
 * Appelée une fois au chargement de la page.
 */
function initROVSchematic() {
    const svg = document.getElementById('rov-schematic');
    if (!svg) return;

    /* ---- Châssis (vue de dessus, avant vers le haut) ---- */
    svgNew('text', { x: 240, y: 40, 'text-anchor': 'middle', class: 'rov-dir-label' }, svg).textContent = 'AVANT';
    svgNew('path', { d: 'M240 56 L253 80 L245 80 L245 92 L235 92 L235 80 L227 80 Z', class: 'rov-nose' }, svg);
    svgNew('rect', { x: 155, y: 102, width: 170, height: 196, rx: 22, class: 'rov-hull' }, svg);
    svgNew('circle', { cx: 240, cy: 126, r: 10, class: 'rov-cam' }, svg);
    svgNew('line', { x1: 228, y1: 200, x2: 252, y2: 200, class: 'rov-center' }, svg);
    svgNew('line', { x1: 240, y1: 188, x2: 240, y2: 212, class: 'rov-center' }, svg);
    svgNew('text', { x: 240, y: 388, 'text-anchor': 'middle', class: 'rov-dir-label' }, svg).textContent = 'ARRIÈRE';

    /* ---- Propulseurs M1–M8 ---- */
    ROV_THRUSTERS.forEach(cfg => {
        const g = svgNew('g', { class: 'rov-thr', transform: 'translate(' + cfg.x + ',' + cfg.y + ')' }, svg);
        svgNew('title', {}, g).textContent = 'CH' + cfg.ch + ' : ' + CH_NAMES[cfg.ch];

        const el = { cfg: cfg, val: null, arrow: null, shaft: null, head: null, halo: null };

        if (cfg.type === 'H') {
            /* Carter du propulseur aligné sur l'axe de poussée + hélice centrale */
            svgNew('rect', { x: -16, y: -8, width: 32, height: 16, rx: 5, class: 'thr-body',
                             transform: 'rotate(' + cfg.angleDeg + ')' }, g);
            svgNew('rect', { x: -21, y: -5, width: 6, height: 10, rx: 2, class: 'thr-nozzle',
                             transform: 'rotate(' + cfg.angleDeg + ')' }, g);
            svgNew('circle', { r: 4, class: 'thr-hub' }, g);
            /* Vecteur de poussée : tracé le long de +x local puis pivoté */
            el.arrow = svgNew('g', { class: 'thr-arrow' }, g);
            el.shaft = svgNew('line', { x1: 20, y1: 0, x2: 28, y2: 0, class: 'thr-shaft' }, el.arrow);
            el.head  = svgNew('polygon', { points: '0,-5.5 11,0 0,5.5', class: 'thr-head',
                                            transform: 'translate(28,0)' }, el.arrow);
        } else {
            /* Propulseur vertical : cercle (vue de dessus) + halo d'intensité */
            el.halo  = svgNew('circle', { r: 17, class: 'thr-halo' }, g);
            svgNew('circle', { r: 17, class: 'vthr-body' }, g);
            svgNew('circle', { r: 4, class: 'thr-hub' }, g);
            /* Vecteur vertical : haut = poussée positive, bas = négative */
            el.arrow = svgNew('g', { class: 'thr-arrow' }, g);
            el.shaft = svgNew('line', { x1: 0, y1: 7, x2: 0, y2: -7, class: 'thr-shaft' }, el.arrow);
            el.head  = svgNew('polygon', { points: '0,0 4.5,7 -4.5,7', class: 'thr-head',
                                            transform: 'translate(0,-7)' }, el.arrow);
        }

        /* Label M1..M8 + valeur instantanée en µs */
        svgNew('text', { x: cfg.lx - cfg.x, y: cfg.ly - cfg.y, 'text-anchor': 'middle',
                         class: 'thr-label' }, g).textContent = cfg.label;
        el.val = svgNew('text', { x: cfg.vx - cfg.x, y: cfg.vy - cfg.y, 'text-anchor': 'middle',
                                  class: 'thr-value' }, g);
        el.val.textContent = NEUTRAL_US + ' µs';

        rovThrEls[cfg.ch] = el;
    });

    /* État initial : tout au neutre */
    updateROVSchematic(new Array(16).fill(NEUTRAL_US));
}

/**
 * Met à jour la vue schématique des propulseurs à partir des valeurs PWM (µs).
 * Branchée sur la réception des trames de télémétrie WebSocket (data.pwm).
 *
 * Dynamique : neutre (1500 µs) = vecteur gris court · poussée positive =
 * vecteur cyan→vert dont la longueur croît de 1500 à 2000 µs · poussée
 * négative = vecteur inversé orange→rouge de 1500 à 1000 µs.
 *
 * @param {number[]} values Valeurs PWM des 16 canaux en microsecondes.
 */
function updateROVSchematic(values) {
    for (let i = 0; i < ROV_THRUSTERS.length; i++) {
        const cfg = ROV_THRUSTERS[i];
        const el = rovThrEls[cfg.ch];
        if (!el || !values || values[cfg.ch] === undefined) continue;

        const us = values[cfg.ch];
        el.val.textContent = us + ' µs';

        /* Déviation normalisée [−1..+1] autour du neutre 1500 µs */
        const dev = (us - NEUTRAL_US) / (MAX_US - NEUTRAL_US);
        const mag = Math.min(1, Math.abs(dev));
        const neutral = (mag < 0.02);   /* bande morte ±10 µs anti-scintillement */

        let color;
        if (neutral) {
            color = ROV_NEUTRAL_COLOR;
        } else if (dev > 0) {
            color = lerpColor([0, 217, 255], [0, 230, 118], mag);   /* cyan → vert */
        } else {
            color = lerpColor([255, 179, 0], [255, 59, 48], mag);   /* orange → rouge */
        }

        el.val.style.fill = color;
        el.arrow.setAttribute('color', color);   /* piloté par currentColor (CSS) */
        /* Au neutre (1500 µs) : aucune flèche (poussée nulle) */
        el.arrow.setAttribute('opacity', neutral ? 0 : 0.45 + 0.55 * mag);
        el.arrow.classList.toggle('thr-active', !neutral);

        if (cfg.type === 'H') {
            /* Longueur ∝ poussée · direction inversée en marche arrière */
            const len = neutral ? 8 : 10 + 26 * mag;
            const ang = (dev >= 0) ? cfg.angleDeg : cfg.angleDeg + 180;
            el.arrow.setAttribute('transform', 'rotate(' + ang + ')');
            el.shaft.setAttribute('x2', 20 + len);
            el.head.setAttribute('transform', 'translate(' + (20 + len) + ',0)');
        } else {
            /* Flèche vers le haut (poussée positive) ou vers le bas */
            const len = neutral ? 6 : 7 + 9 * mag;
            const ang = (dev >= 0) ? 0 : 180;
            el.arrow.setAttribute('transform', 'rotate(' + ang + ')');
            el.shaft.setAttribute('y2', 7 - len);
            el.head.setAttribute('transform', 'translate(0,' + (7 - len) + ')');
            /* Halo circulaire dont le rayon et l'opacité suivent l'intensité */
            el.halo.setAttribute('r', 17 + 6 * mag);
            el.halo.setAttribute('stroke', color);
            el.halo.setAttribute('stroke-opacity', neutral ? 0 : 0.25 + 0.45 * mag);
        }
    }
}

/* =========================================================================
 * CÂBLAGE & SCHÉMAS — AFFICHAGE DYNAMIQUE
 * ========================================================================= */

/**
 * Met à jour la page Câblage pour afficher le bon schéma de liaison
 * en fonction du mode série actif (USB-CDC ou UART GPIO).
 * @param {string} interfaceLabel Label du mode (ex: "USB-CDC", "UART (GPIO 1/2)")
 */
function updateWiringMode(interfaceLabel) {
    const modeEl = document.getElementById('wiring-comm-mode');
    const usbBlock = document.getElementById('wiring-usb');
    const uartBlock = document.getElementById('wiring-uart');
    if (!usbBlock || !uartBlock) return;

    const isUart = (interfaceLabel && interfaceLabel.indexOf('UART') !== -1);

    if (modeEl) {
        modeEl.textContent = isUart ? 'UART Matériel (GPIO 1/2)' : 'USB-CDC Natif';
    }

    if (isUart) {
        usbBlock.style.display = 'none';
        uartBlock.style.display = 'block';
    } else {
        usbBlock.style.display = 'block';
        uartBlock.style.display = 'none';
    }
}

/* =========================================================================
 * MODE TÉMOIN (DRY-RUN) — SORTIES PHYSIQUES
 * ========================================================================= */

/**
 * Initialise le commutateur de sorties physiques dans l'onglet PWM Monitor.
 * Envoie la commande WebSocket {output_enable: true/false} lors du toggle.
 */
function initDryRun() {
    const chk = document.getElementById('chk-outputs-enabled');
    if (!chk) return;

    chk.addEventListener('change', () => {
        const enabled = chk.checked;
        updateDryRunUI(enabled);
        /* Envoi via WebSocket */
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ output_enable: enabled }));
        }
    });
}

/**
 * Met à jour l'affichage UI du mode Dry-Run (switch, texte, badge).
 * @param {boolean} enabled true = sorties actives, false = Dry-Run.
 */
function updateDryRunUI(enabled) {
    const chk = document.getElementById('chk-outputs-enabled');
    const stateText = document.getElementById('dryrun-state-text');
    const badge = document.getElementById('dryrun-badge');

    if (chk) chk.checked = enabled;

    if (stateText) {
        if (enabled) {
            stateText.textContent = 'ACTIVES';
            stateText.className = 'dryrun-state-text on';
        } else {
            stateText.textContent = 'NEUTRALISÉES (DRY-RUN)';
            stateText.className = 'dryrun-state-text off';
        }
    }

    if (badge) {
        if (enabled) {
            badge.textContent = '✓ PROPULSION ACTIVE';
            badge.className = 'dryrun-badge dryrun-badge-ok';
        } else {
            badge.textContent = '⚠ MODE TÉMOIN — Moteurs inactifs';
            badge.className = 'dryrun-badge dryrun-badge-warn';
        }
    }
}

/* =========================================================================
 * CONTRÔLEUR DE VOL — MODE ET MAÎTRE
 * ========================================================================= */

const MODE_LABELS = ['PASSIF', 'AUTO ROULIS ET TANGAGE', 'AUTO FULL'];

/**
 * Met à jour les boutons de mode pour refléter le mode actif.
 * @param {number} activeMode — 0=PASSIF, 1=AUTO_ROULIS (roulis+tangage), 2=AUTO_FULL
 */
function updateFlightMode(activeMode) {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        const m = parseInt(btn.dataset.mode, 10);
        if (m === activeMode) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
}

/**
 * Met à jour le badge du maître actif.
 * @param {boolean} isRPi — true si le RPi 5 est maître
 */
function updateMasterBadge(isRPi) {
    const icon = document.getElementById('master-icon');
    const label = document.getElementById('master-label');
    const badge = document.getElementById('master-badge');
    if (!icon || !label || !badge) return;

    if (isRPi) {
        icon.className = 'master-icon master-rpi';
        label.textContent = 'Maître : RPi 5';
        badge.className = 'mode-badge master-rpi-badge';
    } else {
        icon.className = 'master-icon master-esp';
        label.textContent = 'Maître : Manuel ESP32';
        badge.className = 'mode-badge master-esp-badge';
    }
}

/**
 * Initialise les event listeners des boutons de mode.
 */
function initFlightModeButtons() {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const mode = parseInt(btn.dataset.mode, 10);
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ set_mode: mode }));
            }
        });
    });
}

/* =========================================================================
 * BANC DE TEST
 * ========================================================================= */

function initTestBench() {
    const grid = document.getElementById('slider-grid');
    if (!grid) return;
    grid.innerHTML = '';

    for (let i = 0; i < 16; i++) {
        const item = document.createElement('div');
        item.className = 'slider-item';
        const minVal = (CH_TYPES[i] === 'dimmer') ? 0 : MIN_US;
        const maxVal = (CH_TYPES[i] === 'dimmer') ? 100 : MAX_US;
        const defVal = (CH_TYPES[i] === 'dimmer') ? 0 : NEUTRAL_US;

        item.innerHTML = `
            <div class="sl-header">
                <span class="sl-label">CH${i}: ${CH_NAMES[i]}</span>
                <span class="sl-value" id="sl-val-${i}">${defVal}</span>
            </div>
            <input type="range" id="sl-${i}" min="${minVal}" max="${maxVal}" value="${defVal}" disabled>`;
        grid.appendChild(item);

        const slider = item.querySelector('input[type="range"]');
        slider.addEventListener('input', () => {
            const val = parseInt(slider.value);
            document.getElementById(`sl-val-${i}`).textContent = val;
            if (testUnlocked && ws && ws.readyState === WebSocket.OPEN) {
                const arr = [...pwmValues];
                arr[i] = val;
                ws.send(JSON.stringify({ pwm: arr }));
            }
        });
    }

    const chk = document.getElementById('chk-unlock');
    const btnN = document.getElementById('btn-neutral');
    chk.addEventListener('change', () => {
        testUnlocked = chk.checked;
        btnN.disabled = !testUnlocked;
        for (let i = 0; i < 16; i++) {
            const sl = document.getElementById(`sl-${i}`);
            if (sl) sl.disabled = !testUnlocked;
        }
    });

    btnN.addEventListener('click', () => {
        if (!testUnlocked) return;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ pwm: new Array(16).fill(NEUTRAL_US) }));
        }
        for (let i = 0; i < 16; i++) {
            const sl = document.getElementById(`sl-${i}`);
            if (sl) {
                const def = (CH_TYPES[i] === 'dimmer') ? 0 : NEUTRAL_US;
                sl.value = def;
                document.getElementById(`sl-val-${i}`).textContent = def;
            }
        }
    });
}

/* =========================================================================
 * WI-FI
 * ========================================================================= */

function initWiFi() {
    const btnScan = document.getElementById('btn-scan');
    const btnConnect = document.getElementById('btn-connect');

    btnScan.addEventListener('click', async () => {
        btnScan.disabled = true;
        btnScan.textContent = 'Scan...';
        try {
            const res = await fetch('/api/wifi/scan');
            const data = await res.json();
            renderWifiList(data.networks || []);
        } catch (e) { console.error('Scan Wi-Fi:', e); }
        btnScan.disabled = false;
        btnScan.textContent = 'Scanner les réseaux';
    });

    btnConnect.addEventListener('click', async () => {
        const ssid = document.getElementById('wifi-selected-ssid').textContent;
        const pass = document.getElementById('wifi-password').value;
        btnConnect.disabled = true;
        btnConnect.textContent = 'Connexion...';
        try {
            const res = await fetch('/api/wifi/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}`
            });
            const data = await res.json();
            if (data.status === 'connected') {
                document.getElementById('wifi-ip').textContent = data.ip;
                alert('Connecté à ' + ssid + '\nIP : ' + data.ip);
            } else {
                alert('Échec : ' + (data.message || 'Vérifiez le mot de passe.'));
            }
        } catch (e) { alert('Erreur : ' + e.message); }
        btnConnect.disabled = false;
        btnConnect.textContent = 'Connecter';
    });
}

function renderWifiList(networks) {
    const list = document.getElementById('wifi-list');
    if (!networks.length) {
        list.innerHTML = '<p class="placeholder">Aucun réseau détecté.</p>';
        return;
    }
    list.innerHTML = '';
    networks.forEach(net => {
        const item = document.createElement('div');
        item.className = 'wifi-item';
        const icon = net.secure ? '🔒' : '🔓';
        item.innerHTML = `<span class="ssid">${icon} ${escapeHtml(net.ssid)}</span><span class="rssi">${net.rssi} dBm</span>`;
        item.addEventListener('click', () => {
            document.getElementById('wifi-selected-ssid').textContent = net.ssid;
            document.getElementById('wifi-form').classList.remove('hidden');
            document.getElementById('wifi-password').value = '';
            document.getElementById('wifi-password').focus();
        });
        list.appendChild(item);
    });
}

/* =========================================================================
 * MODE & PARAMÈTRES
 * ========================================================================= */

function initSettings() {
    /* Sauvegarde PID via REST API */
    document.getElementById('btn-save-pid').addEventListener('click', async () => {
        const body = new URLSearchParams({
            r_kp: document.getElementById('pid-r-kp').value,
            r_ki: document.getElementById('pid-r-ki').value,
            r_kd: document.getElementById('pid-r-kd').value,
            p_kp: document.getElementById('pid-p-kp').value,
            p_ki: document.getElementById('pid-p-ki').value,
            p_kd: document.getElementById('pid-p-kd').value,
            d_kp: document.getElementById('pid-d-kp').value,
            d_ki: document.getElementById('pid-d-ki').value,
            d_kd: document.getElementById('pid-d-kd').value,
            wdg_ms: document.getElementById('wdg-timeout').value
        });
        try {
            const res = await fetch('/api/settings/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: body.toString()
            });
            const data = await res.json();
            if (data.status === 'ok') alert('Paramètres sauvegardés avec succès.');
            else alert('Erreur : ' + (data.error || 'inconnue'));
        } catch (e) { alert('Erreur réseau : ' + e.message); }
    });

    /* Bouton sauvegarde liaison série RPi5 */
    const btnComm = document.getElementById('btn-save-comm');
    if (btnComm) {
        btnComm.addEventListener('click', async () => {
            const iface = document.getElementById('sel-comm').value;
            try {
                const res = await fetch('/api/comm/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `iface=${iface}`
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    alert(data.msg || 'Configuration appliquée.');
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                }
            } catch (e) { alert('Erreur réseau : ' + e.message); }
        });
    }

    /* Charger la config liaison série actuelle depuis l'API */
    loadCommConfig();

    /* Charger la config I2C actuelle depuis l'API */
    loadI2CConfig();

    /* Bouton sauvegarde broches I2C */
    const btnI2C = document.getElementById('btn-save-i2c-pins');
    if (btnI2C) {
        btnI2C.addEventListener('click', async () => {
            const sda = parseInt(document.getElementById('i2c-sda-pin').value, 10);
            const scl = parseInt(document.getElementById('i2c-scl-pin').value, 10);
            if (isNaN(sda) || isNaN(scl) || sda < 0 || sda > 48 || scl < 0 || scl > 48) {
                alert('GPIO invalide : valeurs entre 0 et 48.');
                return;
            }
            try {
                const res = await fetch('/api/i2c/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'sda=' + sda + '&scl=' + scl
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    alert('Broches I2C sauvegardées : SDA=' + sda + ', SCL=' + scl);
                    loadI2CConfig();
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                }
            } catch (e) { alert('Erreur réseau : ' + e.message); }
        });
    }

    /* Bouton lancer scan I2C */
    const btnScan = document.getElementById('btn-i2c-scan');
    if (btnScan) {
        btnScan.addEventListener('click', runI2CScan);
    }
}

async function loadCommConfig() {
    try {
        const res = await fetch('/api/comm/config');
        const data = await res.json();
        if (data.interface !== undefined) {
            document.getElementById('sel-comm').value = data.interface;
        }
    } catch (_) { /* silencieux si pas de connexion */ }
}

/* =========================================================================
 * DIAGNOSTIC I2C — CONFIGURATION & SCAN
 * ========================================================================= */

/**
 * Charge la configuration I2C actuelle (broches SDA/SCL) depuis l'API
 * et met à jour l'affichage + les champs de saisie.
 */
async function loadI2CConfig() {
    try {
        const res = await fetch('/api/i2c/config');
        const data = await res.json();
        const sda = data.sda !== undefined ? data.sda : 10;
        const scl = data.scl !== undefined ? data.scl : 11;
        document.getElementById('i2c-sda-pin').value = sda;
        document.getElementById('i2c-scl-pin').value = scl;
        const infoEl = document.getElementById('i2c-current-pins');
        if (infoEl) infoEl.textContent = 'SDA=GPIO ' + sda + ', SCL=GPIO ' + scl;
    } catch (_) { /* silencieux si pas de connexion */ }
}

/**
 * Lance le scan I2C via l'API REST et affiche les résultats dans un tableau.
 * Gère le spinner de chargement et le message "aucun périphérique".
 */
async function runI2CScan() {
    const spinner = document.getElementById('i2c-scan-spinner');
    const resultsDiv = document.getElementById('i2c-scan-results');
    const emptyDiv = document.getElementById('i2c-scan-empty');
    const tbody = document.getElementById('i2c-scan-tbody');
    const btn = document.getElementById('btn-i2c-scan');

    /* Afficher spinner, masquer résultats précédents */
    if (spinner) spinner.style.display = 'inline-block';
    if (resultsDiv) resultsDiv.style.display = 'none';
    if (emptyDiv) emptyDiv.style.display = 'none';
    if (btn) btn.disabled = true;

    try {
        const res = await fetch('/api/i2c/scan');
        const data = await res.json();
        const devices = data.devices || [];

        if (tbody) tbody.innerHTML = '';

        if (devices.length === 0) {
            if (emptyDiv) emptyDiv.style.display = 'block';
        } else {
            devices.forEach(function(d) {
                var tr = document.createElement('tr');
                var tdHex = document.createElement('td');
                tdHex.className = 'i2c-addr-hex';
                tdHex.textContent = d.address;
                var tdDec = document.createElement('td');
                tdDec.textContent = d.dec;
                var tdStatus = document.createElement('td');
                tdStatus.innerHTML = '<span class="i2c-ack-badge">ACK</span>';
                var tdName = document.createElement('td');
                tdName.textContent = d.name;
                tr.appendChild(tdHex);
                tr.appendChild(tdDec);
                tr.appendChild(tdStatus);
                tr.appendChild(tdName);
                if (tbody) tbody.appendChild(tr);
            });
            if (resultsDiv) resultsDiv.style.display = 'block';
        }

        /* Mettre à jour les infos de broches dans la section config */
        if (data.sda !== undefined && data.scl !== undefined) {
            const infoEl = document.getElementById('i2c-current-pins');
            if (infoEl) infoEl.textContent = 'SDA=GPIO ' + data.sda + ', SCL=GPIO ' + data.scl;
        }
    } catch (e) {
        alert('Erreur scan I2C : ' + e.message);
    } finally {
        if (spinner) spinner.style.display = 'none';
        if (btn) btn.disabled = false;
    }
}

/* =========================================================================
 * UTILITAIRES
 * ========================================================================= */

function setText(id, text) { const el = document.getElementById(id); if (el) el.textContent = text; }

function escapeHtml(str) {
    const div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
}

/* =========================================================================
 * INSTRUMENTS AVIATION — Horizon Artificiel & HSI (Canvas 2D)
 * ========================================================================= */

/* --- Lissage LERP (Linear Interpolation) anti-jitter --- */
const _avSmooth = { roll: 0, pitch: 0, yaw: 0, _raf: 0 };
const AV_LERP = 0.18;  /* Facteur de lissage [0..1] : bas = plus fluide, haut = plus réactif */

function _avLerpLoop() {
    _avSmooth.roll  += (_avSmooth._tRoll  - _avSmooth.roll)  * AV_LERP;
    _avSmooth.pitch += (_avSmooth._tPitch - _avSmooth.pitch) * AV_LERP;
    /* Yaw : gérer le wrap-around 360° */
    let dy = _avSmooth._tYaw - _avSmooth.yaw;
    if (dy > 180) dy -= 360;
    if (dy < -180) dy += 360;
    _avSmooth.yaw += dy * AV_LERP;
    if (_avSmooth.yaw < 0) _avSmooth.yaw += 360;
    if (_avSmooth.yaw >= 360) _avSmooth.yaw -= 360;

    drawArtificialHorizon(_avSmooth.roll, _avSmooth.pitch);
    drawHeadingWheel(_avSmooth.yaw);
    _avSmooth._raf = requestAnimationFrame(_avLerpLoop);
}

function avSetTargets(roll, pitch, yaw) {
    _avSmooth._tRoll  = roll;
    _avSmooth._tPitch = pitch;
    _avSmooth._tYaw   = yaw;
    if (!_avSmooth._raf) _avLerpLoop();
}

/**
 * Dessine un horizon artificiel style aviation.
 * @param {number} roll  — Roulis en degrés (+droite / -gauche)
 * @param {number} pitch — Tangage en degrés (+nez haut / -nez bas)
 */
function drawArtificialHorizon(roll, pitch) {
    const cv = document.getElementById('cv-horizon');
    if (!cv) return;
    const ctx = cv.getContext('2d');
    const W = cv.width, H = cv.height;
    const cx = W / 2, cy = H / 2, R = Math.min(cx, cy) - 6;

    ctx.clearRect(0, 0, W, H);
    ctx.save();
    ctx.translate(cx, cy);

    /* --- Cadrans circulaire clip --- */
    ctx.save();
    ctx.beginPath();
    ctx.arc(0, 0, R, 0, Math.PI * 2);
    ctx.clip();

    /* Rotation roulis + translation pitch */
    const rollRad = roll * Math.PI / 180;
    const pitchPx = pitch * 2.5;  /* ~2.5 px par degré */
    ctx.rotate(-rollRad);
    ctx.translate(0, pitchPx);

    /* Ciel */
    ctx.fillStyle = '#2a6db5';
    ctx.fillRect(-R * 2, -R * 2, R * 4, R * 2);
    /* Sol */
    ctx.fillStyle = '#6b4226';
    ctx.fillRect(-R * 2, 0, R * 4, R * 2);
    /* Ligne d'horizon */
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-R * 2, 0);
    ctx.lineTo(R * 2, 0);
    ctx.stroke();

    /* Échelle de tangage (pitch ladder) */
    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.lineWidth = 1.2;
    for (let deg = -30; deg <= 30; deg += 10) {
        if (deg === 0) continue;
        const y = -deg * 2.5;
        const half = (deg % 20 === 0) ? 40 : 22;
        ctx.beginPath();
        ctx.moveTo(-half, y);
        ctx.lineTo(half, y);
        ctx.stroke();
        if (deg % 10 === 0) {
            ctx.fillText(Math.abs(deg) + '', half + 14, y);
            ctx.fillText(Math.abs(deg) + '', -half - 14, y);
        }
    }
    ctx.restore(); /* pop clip */

    /* --- Symbole avion (fixe) --- */
    ctx.strokeStyle = '#ffcc00';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-38, 0);  ctx.lineTo(-12, 0);
    ctx.moveTo(12, 0);   ctx.lineTo(38, 0);
    ctx.moveTo(0, -4);   ctx.lineTo(0, 6);
    ctx.stroke();
    /* Point central */
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fill();

    /* --- Arc d'inclinaison (bank angle) --- */
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    const bankR = R - 4;
    ctx.beginPath();
    ctx.arc(0, 0, bankR, -Math.PI, 0);
    ctx.stroke();

    /* Marqueurs d'angle : 0, ±10, ±20, ±30, ±45, ±60 */
    const bankAngles = [0, 10, 20, 30, 45, 60];
    bankAngles.forEach(deg => {
        [-1, 1].forEach(sign => {
            const a = (-90 + sign * deg) * Math.PI / 180;
            const r1 = bankR - (deg === 0 ? 10 : 6);
            ctx.beginPath();
            ctx.moveTo(Math.cos(a) * r1, Math.sin(a) * r1);
            ctx.lineTo(Math.cos(a) * bankR, Math.sin(a) * bankR);
            ctx.stroke();
        });
    });

    /* Triangle indicateur de roulis */
    ctx.save();
    ctx.rotate(-rollRad);
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(0, -bankR + 2);
    ctx.lineTo(-6, -bankR + 14);
    ctx.lineTo(6, -bankR + 14);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    /* --- Bordure circulaire --- */
    ctx.strokeStyle = '#444857';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, R + 1, 0, Math.PI * 2);
    ctx.stroke();

    ctx.restore(); /* pop translate */
}

/**
 * Dessine une roue de cap (HSI / Directional Gyro) style aviation.
 * @param {number} yaw — Cap en degrés [0–360)
 */
function drawHeadingWheel(yaw) {
    const cv = document.getElementById('cv-hsi');
    if (!cv) return;
    const ctx = cv.getContext('2d');
    const W = cv.width, H = cv.height;
    const cx = W / 2, cy = H / 2, R = Math.min(cx, cy) - 6;

    ctx.clearRect(0, 0, W, H);
    ctx.save();
    ctx.translate(cx, cy);

    const hdgRad = yaw * Math.PI / 180;

    /* --- Rose des vents rotative --- */
    ctx.save();
    ctx.rotate(-hdgRad);

    /* Cercle extérieur */
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, R, 0, Math.PI * 2);
    ctx.stroke();

    /* Graduations : tous les 5° */
    for (let deg = 0; deg < 360; deg += 5) {
        const a = (deg - 90) * Math.PI / 180;
        const isMajor = deg % 30 === 0;
        const isMid   = deg % 10 === 0;
        const len = isMajor ? 14 : (isMid ? 10 : 6);
        ctx.strokeStyle = isMajor ? '#ffffff' : '#aaaaaa';
        ctx.lineWidth   = isMajor ? 2 : 1;
        ctx.beginPath();
        ctx.moveTo(Math.cos(a) * (R - len), Math.sin(a) * (R - len));
        ctx.lineTo(Math.cos(a) * R, Math.sin(a) * R);
        ctx.stroke();
    }

    /* Labels cardinaux et inter-cardinaux */
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 14px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const cardinals = [
        [0, 'N'], [30, '3'], [60, '6'], [90, 'E'],
        [120, '12'], [150, '15'], [180, 'S'],
        [210, '21'], [240, '24'], [270, 'W'],
        [300, '30'], [330, '33']
    ];
    const labelR = R - 24;
    cardinals.forEach(([deg, lbl]) => {
        const a = (deg - 90) * Math.PI / 180;
        ctx.save();
        ctx.translate(Math.cos(a) * labelR, Math.sin(a) * labelR);
        ctx.rotate(hdgRad); /* contre-rotation pour lisibilité */
        if (lbl === 'N') { ctx.fillStyle = '#ff3e9d'; ctx.font = 'bold 16px sans-serif'; }
        else if (['E','S','W'].includes(lbl)) { ctx.fillStyle = '#00d9ff'; }
        else { ctx.fillStyle = '#cccccc'; ctx.font = 'bold 11px sans-serif'; }
        ctx.fillText(lbl, 0, 0);
        ctx.restore();
    });

    /* Flèche Nord */
    ctx.save();
    const na = (0 - 90) * Math.PI / 180;
    ctx.translate(Math.cos(na) * (R - 44), Math.sin(na) * (R - 44));
    ctx.rotate(hdgRad);
    ctx.fillStyle = '#ff3e9d';
    ctx.beginPath();
    ctx.moveTo(0, -8);
    ctx.lineTo(-5, 5);
    ctx.lineTo(5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    ctx.restore(); /* pop rotation rose */

    /* --- Index de cap fixe (triangle en haut) --- */
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(0, -R + 1);
    ctx.lineTo(-8, -R + 16);
    ctx.lineTo(8, -R + 16);
    ctx.closePath();
    ctx.fill();

    /* --- Ligne avion centrale --- */
    ctx.strokeStyle = '#ffcc00';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, -18); ctx.lineTo(0, 18);
    ctx.moveTo(-18, 0); ctx.lineTo(18, 0);
    ctx.stroke();
    /* Ailes stylisées */
    ctx.beginPath();
    ctx.moveTo(-14, 6); ctx.lineTo(-6, 6); ctx.lineTo(-6, 14);
    ctx.moveTo(14, 6);  ctx.lineTo(6, 6);  ctx.lineTo(6, 14);
    ctx.stroke();

    /* --- Valeur HDG numérique (milieu du rayon) --- */
    ctx.fillStyle = '#00d9ff';
    ctx.font = 'bold 22px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const hdgStr = String(Math.round(yaw) % 360).padStart(3, '0') + '°';
    ctx.fillText(hdgStr, 0, R * 0.45);

    /* Bordure */
    ctx.strokeStyle = '#444857';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, R + 1, 0, Math.PI * 2);
    ctx.stroke();

    ctx.restore(); /* pop translate */
}

/* =========================================================================
 * VARIOMÈTRE (VSI) — VITESSE VERTICALE EN cm/s
 * ========================================================================= */

const VSI_MAX_CMS = 100;   /* Pleine échelle : ±100 cm/s */
let _vsiPrevAlt = null;    /* Altitude précédente (m) */
let _vsiPrevT   = 0;       /* Horodatage précédent (ms) */
let _vsiRate    = 0;       /* Vitesse verticale lissée (cm/s) */

/**
 * Met à jour le variomètre à partir de l'altitude barométrique télémétrée.
 * Dérivée temporelle lissée par filtre exponentiel (anti-gigue).
 * @param {number} altM — Altitude courante en mètres
 */
function updateVSI(altM) {
    const t = performance.now();
    if (_vsiPrevAlt !== null) {
        const dt = (t - _vsiPrevT) / 1000;
        if (dt > 0.03 && dt < 2.0) {   /* ignore trous et doublons WS */
            const raw = ((altM - _vsiPrevAlt) / dt) * 100;   /* m/s → cm/s */
            _vsiRate += (raw - _vsiRate) * 0.25;
        }
    }
    _vsiPrevAlt = altM;
    _vsiPrevT = t;

    drawVSI(_vsiRate);
    setText('av-alt', altM.toFixed(2) + ' m');
    const el = document.getElementById('av-vs');
    if (el) {
        el.textContent = (_vsiRate >= 0 ? '+' : '') + Math.round(_vsiRate) + ' cm/s';
        el.style.color = (Math.abs(_vsiRate) < 5) ? '' : (_vsiRate > 0 ? '#00e676' : '#ffb300');
    }
}

/**
 * Dessine un variomètre style aviation : zéro à 9h, montée par le haut,
 * descente par le bas, pleine échelle ±100 cm/s à 3h.
 * @param {number} vsCms — Vitesse verticale en cm/s (+ = montée)
 */
function drawVSI(vsCms) {
    const cv = document.getElementById('cv-vsi');
    if (!cv) return;
    const ctx = cv.getContext('2d');
    const W = cv.width, H = cv.height;
    const cx = W / 2, cy = H / 2, R = Math.min(cx, cy) - 6;

    ctx.clearRect(0, 0, W, H);
    ctx.save();
    ctx.translate(cx, cy);

    /* Fond du cadran */
    ctx.fillStyle = '#0d0e12';
    ctx.beginPath();
    ctx.arc(0, 0, R, 0, Math.PI * 2);
    ctx.fill();

    /* Graduations : majeures tous les 50 (étiquetées), mineures tous les 25 */
    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let v = -VSI_MAX_CMS; v <= VSI_MAX_CMS; v += 25) {
        const a = Math.PI + (v / VSI_MAX_CMS) * Math.PI;
        const major = (v % 50 === 0);
        const r1 = R - (major ? 14 : 8);
        ctx.lineWidth = major ? 2.5 : 1.5;
        ctx.beginPath();
        ctx.moveTo(Math.cos(a) * r1, Math.sin(a) * r1);
        ctx.lineTo(Math.cos(a) * (R - 2), Math.sin(a) * (R - 2));
        ctx.stroke();
        if (major) {
            ctx.font = 'bold 11px monospace';
            ctx.fillText(Math.abs(v) + '', Math.cos(a) * (R - 27), Math.sin(a) * (R - 27));
        }
    }

    /* Signes montée / descente */
    ctx.font = 'bold 15px monospace';
    ctx.fillStyle = '#00e676';
    ctx.fillText('+', -R * 0.42, -R * 0.5);
    ctx.fillStyle = '#ffb300';
    ctx.fillText('-', -R * 0.42, R * 0.5);

    /* Aiguille (contrepoids court + branche utile) */
    const v = Math.max(-VSI_MAX_CMS, Math.min(VSI_MAX_CMS, vsCms));
    const a = Math.PI + (v / VSI_MAX_CMS) * Math.PI;
    ctx.strokeStyle = '#ffcc00';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-Math.cos(a) * 18, -Math.sin(a) * 18);
    ctx.lineTo(Math.cos(a) * (R - 34), Math.sin(a) * (R - 34));
    ctx.stroke();

    /* Moyeu central */
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, Math.PI * 2);
    ctx.fill();

    /* Unité (fond opaque : reste lisible quand l'aiguille passe dessous) */
    ctx.fillStyle = '#0d0e12';
    ctx.fillRect(-24, R * 0.45 - 8, 48, 16);
    ctx.fillStyle = '#9aa0b0';
    ctx.font = 'bold 10px monospace';
    ctx.fillText('cm/s', 0, R * 0.45);

    /* Bordure circulaire */
    ctx.strokeStyle = '#444857';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, R + 1, 0, Math.PI * 2);
    ctx.stroke();

    ctx.restore();
}

/* Dessin initial (valeurs à zéro) au chargement */
function initAviationInstruments() {
    drawVSI(0);
    drawArtificialHorizon(0, 0);
    drawHeadingWheel(0);
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

document.addEventListener('DOMContentLoaded', () => {
    initTileNav();
    initDashboardEditor();
    initPWMGrid();
    initROVSchematic();
    initDryRun();
    initTestBench();
    initWiFi();
    initSettings();
    initAviationInstruments();
    initFlightModeButtons();
    connectWebSocket();
});
