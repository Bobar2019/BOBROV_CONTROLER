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

/* Noms par défaut des 16 canaux PWM — servent de placeholder tant qu'aucun
 * nom personnalisé n'est enregistré en NVS (Paramètres → « Noms des sorties »).
 * CH_NAMES = noms courants, mutés à chaud après chargement depuis /api/names. */
const CH_NAMES_DEFAULT = [
    "Thruster 1", "Thruster 2", "Thruster 3", "Thruster 4",
    "Thruster 5", "Thruster 6", "Thruster 7", "Thruster 8",
    "Pan Servo",  "Tilt Servo", "Claw",       "Claw Rot.",
    "LED Dim 1",  "LED Dim 2",  "Aux 1",      "Aux 2"
];
const CH_NAMES = CH_NAMES_DEFAULT.slice();

/* Noms par défaut des 4 sorties tout-ou-rien (GPIO1-GPIO4). */
const GPIO_NAMES_DEFAULT = ["Sortie 1", "Sortie 2", "Sortie 3", "Sortie 4"];
const GPIO_NAMES = GPIO_NAMES_DEFAULT.slice();

const NEUTRAL_US = 1500;
const MIN_US = 1000;
const MAX_US = 2000;

/* Typage des canaux PWM 10-15 (bit n ↔ canal 10+n, 1 = bidirectionnel).
 * Valeur par défaut alignée sur PWM_TYPE_DEFAULT_MASK du firmware ;
 * resynchronisée par la télémétrie WebSocket puis /api/pwm/config. */
const PWM_TYPE_FIRST = 10;
const PWM_TYPE_LAST = 15;
let _pwmTypeMask = 0x33;

/* Dernier état confirmé des 4 sorties ON/OFF (masque 4 bits — télémétrie ou
 * réponse REST). Base de calcul du basculement des boutons de test GPIO. */
let _gpioState = 0;

let pwmValues = new Array(16).fill(NEUTRAL_US);
let ws = null;
let testUnlocked = false;

/** Type d'un canal 10-15 : true = bidirectionnel, false = unidirectionnel,
 *  null = hors plage typable (canaux 0-9, comportement historique). */
function pwmChType(ch) {
    if (ch < PWM_TYPE_FIRST || ch > PWM_TYPE_LAST) return null;
    return ((_pwmTypeMask >> (ch - PWM_TYPE_FIRST)) & 1) === 1;
}

/* --- Rendu découplé + caches DOM/Canvas (optimisation perf télémétrie 20 Hz) --- */
let _pendingTelemetry = null;            /* dernière trame WS reçue, en attente de rendu */
let _renderQueued = false;               /* un requestAnimationFrame de rendu est-il programmé ? */
const _domCache = Object.create(null);   /* id → élément (évite getElementById répétés) */
const _ctxCache = Object.create(null);   /* id canvas → { cv, ctx } (évite getContext répétés) */

/* Renvoie un élément par son id, mis en cache (ne met pas en cache les absents). */
function getEl(id) {
    let el = _domCache[id];
    if (!el) { el = document.getElementById(id); if (el) _domCache[id] = el; }
    return el;
}
/* Renvoie { cv, ctx } d'un canvas par son id, mis en cache. */
function getCtx(id) {
    let c = _ctxCache[id];
    if (!c) { const cv = getEl(id); if (!cv) return null; c = { cv: cv, ctx: cv.getContext('2d') }; _ctxCache[id] = c; }
    return c;
}

/* =========================================================================
 * THÈME CLAIR / SOMBRE
 * ========================================================================= */

const THEME_KEY = 'bobcontrol_theme';

/* Applique le thème : attribut data-theme sur <html> (pilote les variables CSS
 * et l'icône ☀️/🌙 du bouton), couleur de la barre système mobile, libellés
 * accessibles du bouton. */
function applyTheme(theme) {
    const light = (theme === 'light');
    document.documentElement.setAttribute('data-theme', light ? 'light' : 'dark');
    const meta = document.querySelector('meta[name="theme-color"]');
    if (meta) meta.setAttribute('content', light ? '#eef0f4' : '#131417');
    const btn = getEl('theme-toggle');
    if (btn) {
        const label = light ? 'Passer au thème sombre' : 'Passer au thème clair';
        btn.setAttribute('aria-label', label);
        btn.title = label;
    }
}

/* Branche le bouton de bascule et resynchronise l'affichage avec la préférence
 * mémorisée (l'attribut data-theme est déjà posé par le script inline anti-FOUC
 * de index.html, avant le premier rendu). */
function initTheme() {
    const btn = getEl('theme-toggle');
    if (btn) {
        btn.addEventListener('click', () => {
            const next = document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light';
            applyTheme(next);
            try { localStorage.setItem(THEME_KEY, next); } catch (_) {}
        });
    }
    applyTheme(document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark');
}

/* =========================================================================
 * NOMS DES SORTIES (NVS) — charger / appliquer / sauvegarder
 * ========================================================================= */

/**
 * Nettoie un nom saisi (miroir du nettoyage firmware) : retire les caractères
 * HTML sensibles et de contrôle, normalise les espaces, borne à 20 caractères.
 */
function sanitizeOutputName(v) {
    return String(v || '')
        .replace(/[<>"'`&\\]/g, '')
        .replace(/[\u0000-\u001F\u007F]/g, '')
        .replace(/\s+/g, ' ')
        .trim()
        .slice(0, 20);
}

/**
 * Applique les noms issus de NVS (tableaux « ch » 16 + « gpio » 4 ; une
 * entrée vide = nom par défaut) puis rafraîchit tous les libellés de l'UI.
 */
function applyOutputNames(chArr, gpioArr) {
    for (let i = 0; i < CH_NAMES.length; i++) {
        const custom = (chArr && chArr[i]) ? sanitizeOutputName(chArr[i]) : '';
        CH_NAMES[i] = custom || CH_NAMES_DEFAULT[i];
    }
    for (let i = 0; i < GPIO_NAMES.length; i++) {
        const custom = (gpioArr && gpioArr[i]) ? sanitizeOutputName(gpioArr[i]) : '';
        GPIO_NAMES[i] = custom || GPIO_NAMES_DEFAULT[i];
    }
    refreshOutputNameUI();
}

/**
 * Reporte les noms courants sur tous les libellés : PWM Monitor, Banc de
 * Test, tooltips du schéma propulsion, états GPIO (Paramètres + banc).
 * Accès direct par id (pas le cache getEl) : les curseurs du banc sont
 * reconstruits à chaud lors d'un changement de typage PWM.
 */
function refreshOutputNameUI() {
    for (let i = 0; i < 16; i++) {
        const ml = document.getElementById('ch-lbl-' + i);
        if (ml) ml.textContent = CH_NAMES[i];
        const bl = document.getElementById('sl-lbl-' + i);
        if (bl) bl.textContent = CH_NAMES[i];
        const tt = _thrTitleEls[i];
        if (tt) tt.textContent = 'CH' + i + ' : ' + CH_NAMES[i];
    }
    for (let i = 0; i < 4; i++) {
        const gn = document.getElementById('gpio-name-' + i);
        if (gn) gn.textContent = GPIO_NAMES[i];
        const gpn = document.getElementById('gpio-pin-name-' + i);
        if (gpn) gpn.textContent = GPIO_NAMES[i];
        const bgn = document.getElementById('bench-gpio-name-' + i);
        if (bgn) bgn.textContent = GPIO_NAMES[i];
    }
    fillNamesForm();
}

/* Pré-remplit la carte Paramètres : champ vide = nom par défaut (placeholder). */
function fillNamesForm() {
    for (let i = 0; i < 16; i++) {
        const el = document.getElementById('name-ch-' + i);
        if (el) el.value = (CH_NAMES[i] === CH_NAMES_DEFAULT[i]) ? '' : CH_NAMES[i];
    }
    for (let i = 0; i < 4; i++) {
        const el = document.getElementById('name-gpio-' + i);
        if (el) el.value = (GPIO_NAMES[i] === GPIO_NAMES_DEFAULT[i]) ? '' : GPIO_NAMES[i];
    }
}

/* Charge les noms personnalisés depuis NVS (silencieux si API absente). */
async function loadOutputNames() {
    try {
        const res = await fetch('/api/names');
        if (!res.ok) return;
        const data = await res.json();
        applyOutputNames(data.ch, data.gpio);
    } catch (_) { /* noms par défaut si API indisponible */ }
}

/**
 * Branche le bouton « Appliquer les noms » de la carte Paramètres : envoie
 * les 20 champs (16 PWM + 4 GPIO) à POST /api/names et applique la réponse
 * normalisée par le firmware (nettoyage + bornage).
 */
function initOutputNames() {
    const btn = document.getElementById('btn-save-names');
    if (!btn) return;
    btn.addEventListener('click', async () => {
        const chArr = [], gpioArr = [];
        for (let i = 0; i < 16; i++) {
            const el = document.getElementById('name-ch-' + i);
            chArr.push(sanitizeOutputName(el ? el.value : ''));
        }
        for (let i = 0; i < 4; i++) {
            const el = document.getElementById('name-gpio-' + i);
            gpioArr.push(sanitizeOutputName(el ? el.value : ''));
        }
        btn.disabled = true;
        const status = document.getElementById('names-status');
        const setStatus = (msg, color) => {
            if (status) { status.textContent = msg; status.style.color = color || ''; }
        };
        /* Délai court : sans cela le navigateur peut rester bloqué 30 s et plus
         * si le contrôleur est injoignable (AP perdu, redémarrage en cours). */
        const ctrl = new AbortController();
        const timer = setTimeout(() => ctrl.abort(), 6000);
        try {
            const res = await fetch('/api/names', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'names=' + encodeURIComponent(JSON.stringify({ ch: chArr, gpio: gpioArr })),
                signal: ctrl.signal
            });
            const ct = res.headers.get('content-type') || '';
            if (!res.ok || ct.indexOf('json') === -1) throw new Error('non-json');
            const data = await res.json();
            if (data.status === 'ok') {
                applyOutputNames(data.ch, data.gpio);
                setStatus('Noms enregistrés en NVS et appliqués.', 'var(--success)');
            } else {
                setStatus('Erreur : ' + (data.error || 'inconnue'), 'var(--danger)');
            }
        } catch (e) {
            setStatus(e.name === 'AbortError'
                ? 'Contrôleur injoignable (pas de réponse en 6 s). Vérifiez la connexion Wi-Fi.'
                : 'Le contrôleur ne répond pas sur /api/names — le firmware déployé est ancien ?\n'
                  + 'Flashez le firmware (pio run -t upload) puis réessayez.', 'var(--danger)');
        } finally {
            clearTimeout(timer);
            btn.disabled = false;
        }
    });
}

/* =========================================================================
 * VERSION LOGICIELLE (pied de page global)
 * ========================================================================= */

/**
 * Affiche la version Git du firmware dans le pied de page (#app-version) —
 * champ « version » de GET /api/system/status, alimenté par la macro
 * GIT_VERSION injectée à la compilation (scripts/git_version.py :
 * git describe --tags --always --dirty). Silencieux en cas d'échec : le
 * libellé statique du footer reste alors affiché.
 */
async function loadAppVersion() {
    try {
        const res = await fetch('/api/system/status', { cache: 'no-store' });
        if (!res.ok) return;
        const data = await res.json();
        if (data && typeof data.version === 'string' && data.version) {
            const el = document.getElementById('app-version');
            if (el) el.textContent = data.version.startsWith('v') ? data.version : 'v' + data.version;
        }
    } catch (_) { /* version statique conservée si l'API est indisponible */ }
}

/* =========================================================================
 * CALIBRATION ALTIMÉTRIQUE QNH (Paramètres — v1.0.1)
 * ========================================================================= */

/** Met à jour l'affichage de la carte QNH depuis une réponse /api/qnh/config. */
function updateQnhUI(data) {
    const toggle = document.getElementById('qnh-auto-enable');
    if (toggle && document.activeElement !== toggle) toggle.checked = !!data.auto_enable;
    const status = document.getElementById('qnh-status');
    if (status) {
        const q = (typeof data.qnh === 'number') ? data.qnh.toFixed(1) : '—';
        let txt = 'QNH actuel utilisé : ' + q + ' mbar';
        /* Offset matériel compensé (v1.0.1) — affiché seulement s'il existe */
        if (typeof data.hw_offset === 'number' && data.hw_offset !== 0) {
            txt += ' — offset matériel : ' + (data.hw_offset > 0 ? '+' : '')
                 + data.hw_offset.toFixed(1) + ' mbar';
        }
        if (data.sta_connected === false) txt += ' — pas de connexion Internet (AP seul)';
        status.textContent = txt;
        status.style.color = '';
    }
}

/** Charge l'état QNH (option + valeur appliquée) — silencieux si API absente. */
async function loadQnhConfig() {
    try {
        const res = await fetch('/api/qnh/config', { cache: 'no-store' });
        if (!res.ok) return;
        updateQnhUI(await res.json());
    } catch (_) { /* firmware ancien : carte muette */ }
}

/**
 * Branche la carte QNH : interrupteur (NVS via POST /api/qnh/config) et
 * bouton « Actualiser » (POST /api/qnh/refresh puis polling de la config
 * tant que « fetching » est vrai — le fetch tourne côté ESP32 en tâche
 * détachée, jamais dans le navigateur).
 */
function initQnhCard() {
    const toggle = document.getElementById('qnh-auto-enable');
    if (toggle) {
        toggle.addEventListener('change', async () => {
            const status = document.getElementById('qnh-status');
            const setStatus = (msg, color) => {
                if (status) { status.textContent = msg; status.style.color = color || ''; }
            };
            toggle.disabled = true;
            const ctrl = new AbortController();
            const timer = setTimeout(() => ctrl.abort(), 6000);
            try {
                const res = await fetch('/api/qnh/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'enable=' + (toggle.checked ? 'true' : 'false'),
                    signal: ctrl.signal
                });
                const ct = res.headers.get('content-type') || '';
                if (!res.ok || ct.indexOf('json') === -1) throw new Error('non-json');
                const data = await res.json();
                if (data.status === 'ok') {
                    setStatus(toggle.checked
                        ? 'Calibration automatique activée — requête Open-Meteo au prochain démarrage.'
                        : 'Calibration désactivée — retour au QNH standard (1013.25 mbar).', 'var(--success)');
                    loadQnhConfig();   /* resynchronise (QNH standard si désactivé) */
                } else {
                    setStatus('Erreur : ' + (data.error || 'inconnue'), 'var(--danger)');
                }
            } catch (e) {
                setStatus(e.name === 'AbortError'
                    ? 'Contrôleur injoignable (pas de réponse en 6 s).'
                    : 'Le contrôleur ne répond pas sur /api/qnh/config — firmware ancien ?', 'var(--danger)');
            } finally {
                clearTimeout(timer);
                toggle.disabled = false;
            }
        });
    }

    const btn = document.getElementById('btn-qnh-refresh');
    if (btn) {
        btn.addEventListener('click', async () => {
            const spinner = document.getElementById('qnh-refresh-spinner');
            const status = document.getElementById('qnh-status');
            const setStatus = (msg, color) => {
                if (status) { status.textContent = msg; status.style.color = color || ''; }
            };
            btn.disabled = true;
            if (spinner) spinner.hidden = false;
            const ctrl = new AbortController();
            const timer = setTimeout(() => ctrl.abort(), 6000);
            try {
                const res = await fetch('/api/qnh/refresh', { method: 'POST', signal: ctrl.signal });
                const ct = res.headers.get('content-type') || '';
                if (!res.ok || ct.indexOf('json') === -1) throw new Error('non-json');
                const data = await res.json();
                if (data.status === 'started') {
                    /* Polling (500 ms, max 15 s) jusqu'à la fin du fetch ESP32 */
                    const deadline = Date.now() + 15000;
                    while (Date.now() < deadline) {
                        await new Promise(r => setTimeout(r, 500));
                        try {
                            const cfgRes = await fetch('/api/qnh/config', { cache: 'no-store' });
                            if (!cfgRes.ok) break;
                            const cfg = await cfgRes.json();
                            if (!cfg.fetching) {
                                if (cfg.last_fetch_ok) {
                                    updateQnhUI(cfg);
                                    setStatus('QNH Open-Meteo appliqué : ' + cfg.qnh.toFixed(1) + ' mbar.', 'var(--success)');
                                } else {
                                    setStatus('Échec de la requête Open-Meteo — QNH courant conservé (fallback).', 'var(--danger)');
                                }
                                return;
                            }
                        } catch (_) { /* nouveau tick de polling au tour suivant */ }
                    }
                    setStatus('Requête toujours en cours — rechargez la carte dans un instant.');
                } else if (data.status === 'busy') {
                    setStatus('Une requête QNH est déjà en cours…');
                } else {
                    setStatus('Erreur : ' + (data.error || 'inconnue'), 'var(--danger)');
                }
            } catch (e) {
                setStatus(e.name === 'AbortError'
                    ? 'Contrôleur injoignable (pas de réponse en 6 s).'
                    : (e.message === 'non-json'
                        ? 'Pas de connexion Internet — rejoignez un réseau externe (onglet Wi-Fi), ou firmware ancien.'
                        : 'Erreur réseau : ' + e.message), 'var(--danger)');
            } finally {
                clearTimeout(timer);
                if (spinner) spinner.hidden = true;
                btn.disabled = false;
            }
        });
    }
}

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
 * DASHBOARD EDITOR — RÉORGANISATION DES TUILES (EXPÉRIENCE iOS)
 * ========================================================================= *
 * Entrée en édition : bouton « Réorganiser », ou appui long (500 ms) sur
 * n'importe quelle tuile — le doigt restant posé, la tuile se soulève et
 * suit immédiatement le doigt, sans relâcher (comme sur l'écran d'accueil
 * iPhone). Sortie : bouton « Terminer » ou tap en dehors des tuiles.
 *
 * En édition, toutes les tuiles tremblent (jiggle CSS ±1,5° désynchronisé,
 * voir style.css) et l'ordre est sauvegardé dans localStorage À CHAQUE dépôt.
 *
 * Souris ET tactile : Pointer Events lorsqu'ils sont disponibles (unifient
 * souris/doigt/stylet), sinon repli explicite touchstart/touchmove/touchend.
 * Jamais l'attribut draggable HTML5 : il interrompt les Pointer Events et ne
 * fonctionne pas sur iOS Safari.
 * ========================================================================= */

const LONG_PRESS_MS        = 500;  /* durée d'appui long avant le mode édition */
const PRESS_MOVE_TOLERANCE = 10;   /* déplacement (px) annulant l'appui long   */

let _dashEditing = false;          /* mode édition actif ?                     */
let _drag  = null;                 /* déplacement en cours (voir dragStart)    */
let _press = null;                 /* appui long en cours (voir pressStart)    */

function initDashboardEditor() {
    const grid = document.getElementById('nav-grid');
    const btnEdit = document.getElementById('btnEditDash');
    const btnSave = document.getElementById('btnSaveDash');
    if (!grid || !btnEdit || !btnSave) return;

    /* Restaurer l'ordre mémorisé (propre à chaque appareil) */
    loadDashboardOrder(grid);

    /* Écouteurs PERMANENTS sur les tuiles : le guard tactile vérifie l'état
     * dynamiquement — hors édition un touchmove défile normalement la page,
     * en édition il est bloqué pour laisser le drag suivre le doigt. */
    grid.querySelectorAll('.nav-tile').forEach(tile => {
        if (window.PointerEvent) {
            tile.addEventListener('pointerdown', onTilePointerDown);
        } else {
            /* Repli explicite pour les navigateurs sans Pointer Events :
             * pas de preventDefault au touchstart pour laisser le tap
             * déclencher la navigation (le click natif doit survivre). */
            tile.addEventListener('touchstart', onTileTouchStart, { passive: true });
        }
        tile.addEventListener('touchmove', touchGuard, { passive: false });
        /* L'ancre <a> ne doit JAMAIS déclencher le drag natif HTML5 */
        tile.addEventListener('dragstart', dragStartGuard);
        /* Pas de menu contextuel (clic droit / long-press Android) : il
         * interromprait l'appui long */
        tile.addEventListener('contextmenu', contextMenuGuard);
    });

    /* Suivi global du pointeur : dispatch selon l'état (_drag puis _press).
     * Persistants — inactifs (retour immédiat) hors interactions tuiles. */
    if (window.PointerEvent) {
        document.addEventListener('pointermove', onGlobalPointerMove);
        document.addEventListener('pointerup', onGlobalPointerEnd);
        document.addEventListener('pointercancel', onGlobalPointerEnd);
    } else {
        document.addEventListener('touchmove', onGlobalTouchMove, { passive: false });
        document.addEventListener('touchend', onGlobalTouchEnd);
        document.addEventListener('touchcancel', onGlobalTouchEnd);
    }

    btnEdit.addEventListener('click', () => enterDashEditMode());
    btnSave.addEventListener('click', () => exitDashEditMode());

    /* Tap en dehors des tuiles (et de la barre d'édition) → sortie du mode
     * édition, comme un tap sur l'écran d'accueil iPhone. */
    document.addEventListener('click', (e) => {
        if (!_dashEditing) return;
        if (!(e.target instanceof Element)) return;
        if (e.target.closest('.nav-tile') || e.target.closest('.dash-edit-bar')) return;
        exitDashEditMode();
    });
}

/* ----- Entrée / sortie du mode édition ----- */

function enterDashEditMode() {
    if (_dashEditing) return;
    _dashEditing = true;
    const grid = document.getElementById('nav-grid');
    if (grid) grid.classList.add('editing');
    _setDashEditUI(true);
}

function exitDashEditMode() {
    if (!_dashEditing) return;
    _dashEditing = false;
    cancelLongPress();
    if (_drag) dragEnd(_drag.pointerId);
    const grid = document.getElementById('nav-grid');
    if (grid) {
        grid.classList.add('no-enter-anim');
        grid.classList.remove('editing');
        grid.querySelectorAll('.nav-tile').forEach(t => {
            t.classList.remove('drop-before', 'drop-after', 'dragging-ghost', 'pressing');
        });
        /* Sûreté : l'ordre est déjà sauvegardé à chaque dépôt */
        saveDashboardOrder(grid);
        setTimeout(() => grid.classList.remove('no-enter-anim'), 100);
    }
    _setDashEditUI(false);
}

/* Bascule les contrôles de la barre d'édition (bouton/hint). */
function _setDashEditUI(editing) {
    const btnEdit = document.getElementById('btnEditDash');
    const btnSave = document.getElementById('btnSaveDash');
    const hint = document.getElementById('dashEditHint');
    if (btnEdit) btnEdit.style.display = editing ? 'none' : 'inline-flex';
    if (btnSave) btnSave.style.display = editing ? 'inline-flex' : 'none';
    if (hint) hint.style.display = editing ? 'inline' : 'none';
}

/* ----- Appui long : entrée en édition + soulèvement immédiat (iOS) ----- */

/* Démarre l'attente d'appui long sur une tuile (pointeur encore posé). */
function pressStart(tile, x, y, pointerId) {
    cancelLongPress();
    _press = { tile: tile, pointerId: pointerId, x: x, y: y };
    /* Grossissement léger : feedback immédiat pendant l'attente */
    tile.classList.add('pressing');
    _press.timer = setTimeout(() => {
        const p = _press;
        _press = null;
        if (!p || !p.tile || !p.tile.isConnected) return;
        p.tile.classList.remove('pressing');
        enterDashEditMode();
        /* Le doigt est toujours posé (sinon ce timer aurait été annulé) :
         * la tuile se soulève immédiatement et suit le doigt sans attendre
         * un nouvel appui — fidèle à l'écran d'accueil iPhone. */
        dragStart(p.tile, p.x, p.y, p.pointerId);
    }, LONG_PRESS_MS);
}

/* Annule l'appui long si le pointeur s'éloigne (scroll/tap, pas un appui). */
function pressMove(x, y) {
    if (!_press) return;
    if (Math.hypot(x - _press.x, y - _press.y) > PRESS_MOVE_TOLERANCE) cancelLongPress();
}

/* Annule l'appui long en cours (relâchement, sortie d'édition…). */
function cancelLongPress() {
    if (!_press) return;
    clearTimeout(_press.timer);
    if (_press.tile) _press.tile.classList.remove('pressing');
    _press = null;
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

/* ----- Glisser-déposer (clone flottant + réordonnancement en direct) ----- */

/* Soulève la tuile : clone flottant suivant le pointeur + trou translucide
 * dans la grille (dragging-ghost). Le pointeur est déjà capturé par
 * onTilePointerDown — les déplacements continuent d'arriver en bulle. */
function dragStart(tile, x, y, pointerId) {
    if (_drag) dragEnd(_drag.pointerId);
    const grid = tile.parentElement;
    if (!grid) return;
    const rect = tile.getBoundingClientRect();

    /* Clone flottant : zoom + ombre portée (tuile « soulevée ») ; hors
     * .nav-grid.editing → aucune animation, il ne tremble pas. */
    const floater = tile.cloneNode(true);
    floater.style.cssText = `position:fixed;z-index:99999;pointer-events:none;
        width:${rect.width}px;height:${rect.height}px;
        opacity:0.85;transform:scale(1.05);
        box-shadow:0 15px 40px rgba(0,0,0,0.5);
        left:${rect.left}px;top:${rect.top}px;transition:none;`;
    document.body.appendChild(floater);
    tile.classList.add('dragging-ghost');

    _drag = {
        grid: grid, sourceTile: tile, floater: floater,
        offsetX: x - rect.left,
        offsetY: y - rect.top,
        pointerId: pointerId,
        moved: false
    };
}

/* Déplace le clone et réordonne la grille en direct : la tuile la plus
 * proche du pointeur fait place (marquage drop-before/after + insertion). */
function dragMove(x, y) {
    if (!_drag) return;
    const d = _drag;
    d.moved = true;
    d.floater.style.left = (x - d.offsetX) + 'px';
    d.floater.style.top  = (y - d.offsetY) + 'px';

    const tiles = Array.from(d.grid.querySelectorAll('.nav-tile'));
    tiles.forEach(t => t.classList.remove('drop-before', 'drop-after'));

    let closest = null, minDist = Infinity;
    tiles.forEach(t => {
        if (t === d.sourceTile) return;
        const r = t.getBoundingClientRect();
        const dist = Math.hypot(x - (r.left + r.width / 2), y - (r.top + r.height / 2));
        if (dist < minDist) { minDist = dist; closest = t; }
    });

    if (closest && minDist < 200) {
        const r = closest.getBoundingClientRect();
        const before = x < r.left + r.width / 2;
        closest.classList.add(before ? 'drop-before' : 'drop-after');

        /* Live reorder : la tuile survolée fait place dynamiquement */
        const desiredNext = before ? closest : closest.nextElementSibling;
        if (d.sourceTile.nextElementSibling !== desiredNext && d.sourceTile !== desiredNext) {
            d.grid.insertBefore(d.sourceTile, desiredNext);
        }
    }
}

/* Dépose la tuile : nettoyage + sauvegarde IMMÉDIATE de l'ordre — il survit
 * à un rechargement même sans passer par « Terminer ». */
function dragEnd(pointerId) {
    if (!_drag) return;
    const d = _drag;
    _drag = null;

    d.grid.querySelectorAll('.nav-tile').forEach(t => t.classList.remove('drop-before', 'drop-after'));
    d.sourceTile.classList.remove('dragging-ghost');
    if (d.floater && d.floater.parentNode) d.floater.parentNode.removeChild(d.floater);
    try {
        if (pointerId !== null && d.sourceTile.releasePointerCapture) {
            d.sourceTile.releasePointerCapture(pointerId);
        }
    } catch (_) {}

    if (d.moved) saveDashboardOrder(d.grid);
}

/* ----- Aiguillage Pointer Events (souris + tactile unifiés) ----- */

function onTilePointerDown(e) {
    if (e.button !== undefined && e.button !== 0) return;
    e.preventDefault();
    const tile = e.currentTarget;
    /* Capturer le pointeur : garantit la réception des pointermove et empêche le drag natif */
    try { tile.setPointerCapture(e.pointerId); } catch (_) {}
    if (_dashEditing) dragStart(tile, e.clientX, e.clientY, e.pointerId);
    else pressStart(tile, e.clientX, e.clientY, e.pointerId);
}

function onGlobalPointerMove(e) {
    if (_drag) {
        if (e.cancelable) e.preventDefault();
        dragMove(e.clientX, e.clientY);
    } else if (_press) {
        if (_press.pointerId !== null && e.pointerId !== _press.pointerId) return;
        pressMove(e.clientX, e.clientY);
    }
}

function onGlobalPointerEnd(e) {
    if (_drag) {
        if (_drag.pointerId !== null && e.pointerId !== undefined &&
            e.pointerId !== _drag.pointerId) return;
        dragEnd(e.pointerId !== undefined ? e.pointerId : null);
    } else if (_press) {
        if (_press.pointerId !== null && e.pointerId !== undefined &&
            e.pointerId !== _press.pointerId) return;
        cancelLongPress();
    }
}

/* ----- Repli tactile explicite (navigateurs sans Pointer Events) ----- */

function onTileTouchStart(e) {
    const t = e.touches[0];
    if (!t) return;
    /* Pas de preventDefault ici : le tap doit déclencher le click natif
     * (navigation) — le scroll est bloqué par touchGuard en édition. */
    const tile = e.currentTarget;
    if (_dashEditing) dragStart(tile, t.clientX, t.clientY, null);
    else pressStart(tile, t.clientX, t.clientY, null);
}

function onGlobalTouchMove(e) {
    const t = e.touches[0];
    if (!t) return;
    if (_drag) {
        if (e.cancelable) e.preventDefault();
        dragMove(t.clientX, t.clientY);
    } else if (_press) {
        pressMove(t.clientX, t.clientY);
    }
}

function onGlobalTouchEnd() {
    if (_drag) dragEnd(null);
    else if (_press) cancelLongPress();
}

/* ----- Gardes d'événements ----- */

function dragStartGuard(e) { e.preventDefault(); }
function contextMenuGuard(e) { e.preventDefault(); }
/* Bloque le défilement tactile pendant l'édition (le drag suit le doigt) ;
 * hors édition, le scroll reste possible — l'appui long est alors annulé par
 * le déplacement (pressMove), comme sur iPhone. */
function touchGuard(e) {
    const grid = e.currentTarget && e.currentTarget.parentElement;
    if (grid && grid.classList.contains('editing')) e.preventDefault();
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
        /* Réception découplée du rendu : on stocke la trame, le rendu est fait une
         * seule fois par frame (requestAnimationFrame). Évite l'accumulation des
         * trames et les "forced reflow" quand la télémétrie arrive à 20 Hz. */
        try { _pendingTelemetry = JSON.parse(event.data); }
        catch (e) { console.warn('[WS] JSON invalide:', e); return; }
        if (!_renderQueued) {
            _renderQueued = true;
            requestAnimationFrame(_flushTelemetry);
        }
    };
}

/* Rend la dernière trame reçue (appelé par requestAnimationFrame, 1×/frame max). */
function _flushTelemetry() {
    _renderQueued = false;
    const data = _pendingTelemetry;
    _pendingTelemetry = null;
    if (data) processTelemetry(data);
}

function processTelemetry(data) {
    /* Informations Wi-Fi temps réel (STA + AP) */
    if (data.wifi_info) {
        const w = data.wifi_info;
        const staEl = getEl('wifi-sta-status');
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
        const rssiEl = getEl('wifi-sta-rssi');
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
        const ipEl = getEl('wifi-ip');
        if (ipEl) {
            ipEl.textContent = w.sta_connected ? (w.sta_ip || '—') : (w.ap_ip || '—');
        }
    }

    if (data.wdg !== undefined) {
        const el = getEl('badge-wdg');
        if (el) {
            const txt = data.wdg ? 'WDG: ALERTE' : 'WDG: OK';
            const cls = data.wdg ? 'badge badge-off' : 'badge badge-ok';
            if (el.textContent !== txt) el.textContent = txt;
            if (el.className !== cls) el.className = cls;
        }
    }

    /* Typage des canaux PWM 10-15 (synchronisation UI ← ESP32) */
    if (data.pwm_type_mask !== undefined) {
        applyPwmTypeMask(data.pwm_type_mask);
    }

    /* État réel des 4 sorties ON/OFF (synchronisation UI ← ESP32) */
    if (data.gpio_state !== undefined) {
        updateGpioStateUI(data.gpio_state);
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
        /* Offset de tare Nord appliqué au cap (affiché près du bouton « Régler Nord ») */
        const noEl = getEl('north-offset');
        if (noEl) {
            const off = Number(data.imu.north_offset) || 0;
            noEl.textContent = 'offset ' + (off >= 0 ? '+' : '') + off.toFixed(1) + '°';
        }
        /* Source de cap : magnétique (Rotation Vector 9-DOF du BNO085) ou secours
         * relatif — la calibration du magnétomètre (0–3) monte en déplaçant le
         * ROV en « 8 » dans des orientations variées. */
        const csEl = getEl('cap-source');
        if (csEl) {
            const cal = Number(data.imu.mag_cal) || 0;
            if (data.imu.mag_active) {
                csEl.textContent = cal >= 3
                    ? 'CAP : MAGNÉTIQUE (calibré)'
                    : 'CAP : MAGNÉTIQUE — calib. ' + cal + '/3, déplacer le ROV en « 8 »';
                csEl.className = 'rov-3d-cap-source' + (cal >= 3 ? ' rov-3d-cap-ok' : ' rov-3d-cap-warn');
            } else {
                csEl.textContent = 'CAP : RELATIF — magnéto indisponible, « Régler Nord » requis';
                csEl.className = 'rov-3d-cap-source rov-3d-cap-warn';
            }
        }
    }

    if (data.press) {
        /* Backend MS5803 avec tare surface : statut d'immersion, profondeur eau
         * douce (m, positive vers le bas) et altitude barométrique réelle (m). */
        const immersed = !!data.press.immersed;
        const depthM   = data.press.depth    || 0;
        const baroAlt  = data.press.baro_alt || 0;
        setText('val-mbar',    (data.press.mbar    || 0).toFixed(1) + ' mbar');
        setText('val-surface', (data.press.surface || 0).toFixed(1) + ' mbar');
        setText('val-temp',    (data.press.temp    || 0).toFixed(1) + ' °C');

        /* Statut d'immersion (bleu = immergé, ambre = hors de l'eau) */
        const subEl = getEl('val-substate');
        if (subEl) {
            const stTxt = immersed ? 'En immersion' : 'Hors de l\'eau';
            if (subEl.textContent !== stTxt) subEl.textContent = stTxt;
            subEl.style.color = immersed ? 'var(--info)' : 'var(--warning)';
        }

        /* Profondeur : nulle hors de l'eau, calculée en immersion */
        setText('val-depth', (immersed ? depthM : 0).toFixed(2) + ' m');

        /* Altitude barométrique : réelle hors de l'eau, figée/masquée en immersion */
        setText('val-alt', immersed ? '— (immersion)' : baroAlt.toFixed(1) + ' m');

        /* Vitesse verticale basée sur la profondeur (0 hors de l'eau) */
        updateVSI(immersed ? depthM : 0);
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
        /* Détail de la dernière erreur d'init BNO085 — diagnostic visible depuis
         * l'interface Web sans câble USB (tooltip sur le badge + ligne dédiée). */
        const bnoErr = (data.sensors.bno085_err || '').trim();
        const bnoBadge = getEl('sens-bno085');
        if (bnoBadge) {
            const tip = bnoErr ? ('BNO085 : ' + bnoErr) : 'BNO085 opérationnel';
            if (bnoBadge.title !== tip) bnoBadge.title = tip;
        }
        setText('sens-bno085-err', bnoErr ? ('BNO085 : ' + bnoErr) : '');
    }

    /* Statut liaison RPi5 (badge header + page Câblage) */
    if (data.rpi_link) {
        const el = getEl('badge-rpi');
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
const _sensorLeds = Object.create(null);   /* id → élément .led (cache) */
function updateSensorBadge(id, status) {
    let led = _sensorLeds[id];
    if (!led) {
        const el = getEl(id);
        if (!el) return;
        led = el.querySelector('.led');
        if (!led) return;
        _sensorLeds[id] = led;
    }
    const cls = 'led ' + (status === 1 ? 'led-ok' : status === 2 ? 'led-emu' : 'led-fail');
    if (led.className !== cls) led.className = cls;
}

/* =========================================================================
 * PWM MONITOR
 * ========================================================================= */

function initPWMGrid() {
    const grid = document.querySelector('.pwm-grid');
    if (!grid) return;
    grid.innerHTML = '';
    _pwmEls.length = 0;
    _pwmLast.fill(null);

    for (let i = 0; i < 16; i++) {
        const ch = document.createElement('div');
        ch.className = 'pwm-channel';
        const variante = _pwmBarVariant(i);
        ch.innerHTML = `
            <div class="ch-header">
                <span class="ch-label">CH${i}: <span id="ch-lbl-${i}"></span></span>
                <span class="ch-value" id="pwm-val-${i}">${NEUTRAL_US} µs</span>
            </div>
            <div class="pwm-bar-bg${variante.className}" id="pwm-bar-bg-${i}">
                ${variante.center ? '<div class="pwm-center"></div>' : ''}
                <div class="pwm-bar-fill" id="pwm-fill-${i}"></div>
            </div>`;
        grid.appendChild(ch);
        getEl('ch-lbl-' + i).textContent = CH_NAMES[i];   /* nom courant (NVS appliqué ensuite) */
        _pwmEls[i] = {
            val: ch.querySelector('.ch-value'),
            fill: ch.querySelector('.pwm-bar-fill'),
            bg: ch.querySelector('.pwm-bar-bg')
        };
    }

    for (let i = 0; i < 16; i++) updatePWMBar(i, NEUTRAL_US);
}

const _pwmEls = [];                        /* Références DOM des 16 canaux (val, fill, bg) */
const _pwmLast = new Array(16).fill(null); /* Dernière valeur rendue par canal (anti-réécriture) */

/** Variante de rendu d'un canal : 'center' = curseur centré (canaux 0-7 et
 *  10-15 bidirectionnels), 'gauge' = jauge pleine échelle 0-100 % (10-15
 *  unidirectionnels), 'plain' = barre simple historique (8-9, servos). */
function _pwmBarVariant(ch) {
    if (ch <= 7 || pwmChType(ch) === true) return { mode: 'center', className: ' bidir', center: true };
    if (pwmChType(ch) === false) return { mode: 'gauge', className: ' gauge', center: false };
    return { mode: 'plain', className: '', center: false };
}

function updatePWMBar(ch, us) {
    const refs = _pwmEls[ch];
    if (!refs || !refs.val || !refs.fill) return;
    if (_pwmLast[ch] === us) return;       /* inchangé → aucune écriture DOM */
    _pwmLast[ch] = us;
    const valEl = refs.val, fillEl = refs.fill;
    const range = MAX_US - MIN_US;
    const variante = _pwmBarVariant(ch);

    if (variante.mode === 'center') {
        valEl.textContent = us + ' µs';
        const dev = (us - NEUTRAL_US) / (range / 2);
        const pct = Math.abs(dev) * 50;
        fillEl.style.clipPath = '';
        if (dev >= 0) {
            fillEl.style.left = '50%';
            fillEl.style.width = pct + '%';
            fillEl.style.background = dev > 0.5 ? 'var(--accent-magenta)' : 'var(--accent-cyan)';
        } else {
            fillEl.style.left = (50 - pct) + '%';
            fillEl.style.width = pct + '%';
            fillEl.style.background = dev < -0.5 ? 'var(--danger)' : 'var(--accent-cyan)';
        }
    } else if (variante.mode === 'gauge') {
        /* Jauge pleine échelle 0-100 % : le fond affiche le dégradé complet
         * vert → orange → rouge ; le remplissage (même dégradé) révèle par
         * clip-path la tranche correspondant à la valeur. */
        const pct = Math.max(0, Math.min(100, ((us - MIN_US) / range) * 100));
        valEl.textContent = Math.round(pct) + ' %';
        fillEl.style.left = '';
        fillEl.style.width = '100%';
        fillEl.style.background = '';
        fillEl.style.clipPath = 'inset(0 ' + (100 - pct).toFixed(1) + '% 0 0)';
    } else {
        const pct = ((us - MIN_US) / range) * 100;
        fillEl.style.clipPath = '';
        fillEl.style.width = Math.max(0, Math.min(100, pct)) + '%';
        fillEl.style.background = 'var(--accent-cyan)';
    }
}

/**
 * Applique un nouveau masque de typage des canaux 10-15 : reconstruit la
 * structure de rendu des canaux concernés (curseur centré ↔ jauge 0-100 %)
 * puis force un nouveau rendu. Adapte aussi le Banc de Test (curseurs
 * µs ↔ 0-100 %, positions de sécurité). Appelée par la télémétrie WebSocket
 * et par la sauvegarde de l'onglet Paramètres.
 */
function applyPwmTypeMask(mask) {
    const m = (Number(mask) || 0) & 0x3F;
    if (m === _pwmTypeMask) return;
    _pwmTypeMask = m;
    for (let i = PWM_TYPE_FIRST; i <= PWM_TYPE_LAST; i++) {
        const refs = _pwmEls[i];
        if (!refs || !refs.bg) continue;
        const variante = _pwmBarVariant(i);
        const cls = 'pwm-bar-bg' + variante.className;
        if (refs.bg.className !== cls) refs.bg.className = cls;
        refs.bg.innerHTML = (variante.center ? '<div class="pwm-center"></div>' : '')
            + '<div class="pwm-bar-fill" id="pwm-fill-' + i + '"></div>';
        refs.fill = refs.bg.querySelector('.pwm-bar-fill');
        _pwmLast[i] = null;   /* structure modifiée → forcer le rendu */
    }
    for (let i = PWM_TYPE_FIRST; i <= PWM_TYPE_LAST; i++) updatePWMBar(i, pwmValues[i]);

    /* Le Banc de Test suit le typage : ses curseurs basculent µs ↔ 0-100 %
     * et reviennent aux positions de sécurité des canaux concernés. */
    buildTestBenchSliders();
}

/**
 * Met à jour les pastilles d'état et les boutons de test des 4 sorties
 * ON/OFF (masque 4 bits — bit 0 = sortie 1). Mémorise l'état confirmé dans
 * _gpioState (base du basculement) ; le libellé du bouton = action opposée.
 */
function updateGpioStateUI(mask) {
    const m = Number(mask) || 0;
    _gpioState = m;   /* état confirmé (base du basculement des boutons de test) */
    for (let i = 0; i < 4; i++) {
        const on = ((m >> i) & 1) === 1;
        const txt = on ? 'ON' : 'OFF';
        const cls = 'gpio-st ' + (on ? 'gpio-st-on' : 'gpio-st-off');

        /* Pastilles d'état : carte Paramètres + Banc de Test */
        for (const id of ['gpio-st-' + i, 'bench-gpio-st-' + i]) {
            const el = getEl(id);
            if (!el) continue;
            if (el.textContent !== txt) el.textContent = txt;
            if (el.className !== cls) el.className = cls;
        }

        /* Boutons associés : libellé = action, teinte = état */
        const label = on ? 'Couper' : 'Activer';
        for (const id of ['gpio-test-' + i, 'bench-gpio-test-' + i]) {
            const btn = getEl(id);
            if (!btn) continue;
            if (btn.textContent !== label) btn.textContent = label;
            if (btn.classList.contains('on') !== on) btn.classList.toggle('on', on);
        }
    }
}

/**
 * Affiche (ou restaure) le message du hint des boutons de test GPIO.
 * @param {boolean} isMaster true si le RPi 5 est maître (test écrasable).
 */
function showGpioTestHint(isMaster) {
    const msg = isMaster
        ? 'RPi 5 maître actif : le test a été appliqué, mais la prochaine trame descendante écrasera les sorties ON/OFF.'
        : "Boutons de test (établi) : forcent l'état des sorties — le firmware relit et renvoie l'état réel. Écrasés par le watchdog, l'E-Stop ou une trame RPi 5 maître.";
    for (const id of ['gpio-test-hint', 'bench-gpio-hint']) {
        const hint = getEl(id);
        if (!hint) continue;
        hint.textContent = msg;
        hint.style.color = isMaster ? 'var(--warning)' : '';
    }
}

/**
 * Bascule l'état d'une sortie ON/OFF via REST /api/gpio/test (masque envoyé
 * = état confirmé _gpioState avec le bit de la sortie inversé). Partagée par
 * les boutons de test de Paramètres ET du Banc de Test.
 *
 * Délai court de 6 s : sans cela le navigateur peut rester bloqué 30 s et
 * plus si le contrôleur est injoignable (AP perdu, redémarrage en cours).
 */
async function toggleGpioOutput(i, btn) {
    btn.disabled = true;
    const mask = _gpioState ^ (1 << i);   /* bascule la sortie i */
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 6000);
    try {
        const res = await fetch('/api/gpio/test', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'mask=' + mask,
            signal: ctrl.signal
        });
        const ct = res.headers.get('content-type') || '';
        if (!res.ok || ct.indexOf('json') === -1) {
            /* Réponse non-JSON : route absente du firmware déployé (la requête
             * est retombée sur le portail captif) ou erreur serveur */
            throw new Error('non-json');
        }
        const data = await res.json();
        if (data.status === 'ok') {
            updateGpioStateUI(data.gpio_state);
            showGpioTestHint(data.rpi5_master === true);
        } else {
            alert('Erreur : ' + (data.error || 'inconnue'));
        }
    } catch (e) {
        alert(e.name === 'AbortError'
            ? 'Test GPIO : contrôleur injoignable (pas de réponse en 6 s). Vérifiez la connexion Wi-Fi au ROV.'
            : 'Test GPIO : le contrôleur ne répond pas sur /api/gpio/test — le firmware déployé est ancien ?\n'
              + 'Flashez le firmware (pio run -t upload) puis réessayez.');
    } finally {
        clearTimeout(timer);
        btn.disabled = false;
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
/* Références des <title> SVG des propulseurs — mis à jour lors d'un
 * renommage des sorties (NVS → carte « Noms des sorties »). */
const _thrTitleEls = [];

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
        const titleEl = svgNew('title', {}, g);
        titleEl.textContent = 'CH' + cfg.ch + ' : ' + CH_NAMES[cfg.ch];
        _thrTitleEls[cfg.ch] = titleEl;   /* refreshOutputNameUI met à jour le tooltip */

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
const _rovLast = new Array(16).fill(null); /* Dernière valeur rendue par canal (schéma ROV) */

function updateROVSchematic(values) {
    for (let i = 0; i < ROV_THRUSTERS.length; i++) {
        const cfg = ROV_THRUSTERS[i];
        const el = rovThrEls[cfg.ch];
        if (!el || !values || values[cfg.ch] === undefined) continue;

        const us = values[cfg.ch];
        if (_rovLast[cfg.ch] === us) continue;   /* inchangé → pas de réécriture SVG */
        _rovLast[cfg.ch] = us;
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
let _lastWiring = null;
function updateWiringMode(interfaceLabel) {
    if (_lastWiring === interfaceLabel) return;   /* mode inchangé → pas de réécriture */
    const modeEl = getEl('wiring-comm-mode');
    const usbBlock = getEl('wiring-usb');
    const uartBlock = getEl('wiring-uart');
    if (!usbBlock || !uartBlock) return;
    _lastWiring = interfaceLabel;

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
let _lastDryRun = null;
function updateDryRunUI(enabled) {
    const chk = getEl('chk-outputs-enabled');
    const stateText = getEl('dryrun-state-text');
    const badge = getEl('dryrun-badge');

    if (chk) chk.checked = enabled;
    if (_lastDryRun === enabled) return;   /* état inchangé → badges déjà à jour */
    _lastDryRun = enabled;

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
let _modeBtns = null;
let _lastFlightMode = null;
function updateFlightMode(activeMode) {
    if (_lastFlightMode === activeMode) return;   /* mode inchangé → pas de requête DOM */
    _lastFlightMode = activeMode;
    if (!_modeBtns) _modeBtns = document.querySelectorAll('.mode-btn');
    _modeBtns.forEach(btn => {
        const m = parseInt(btn.dataset.mode, 10);
        btn.classList.toggle('active', m === activeMode);
    });
}

/**
 * Met à jour le badge du maître actif.
 * @param {boolean} isRPi — true si le RPi 5 est maître
 */
let _lastMaster = null;
function updateMasterBadge(isRPi) {
    const icon = getEl('master-icon');
    const label = getEl('master-label');
    const badge = getEl('master-badge');
    if (!icon || !label || !badge) return;
    if (_lastMaster === isRPi) return;   /* inchangé */
    _lastMaster = isRPi;

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
 * BANC DE TEST — ADAPTATIF À LA CONFIGURATION (typage PWM + noms + GPIO)
 * ========================================================================= */

/* Position de sécurité d'un canal (miroir firmware setAllNeutral) : 1500 µs
 * pour un canal bidirectionnel, 1000 µs (0 %) pour un canal unidirectionnel. */
function benchSafeUs(i) {
    return (pwmChType(i) === false) ? MIN_US : NEUTRAL_US;
}

/**
 * Reconstruit les 16 curseurs selon le typage PWM en vigueur : un canal 10-15
 * unidirectionnel → curseur 0-100 % (converti en µs à l'envoi : 1000+%×10),
 * tous les autres → 1000-2000 µs. Les libellés affichent les noms NVS.
 * Rejouable à chaud (changement de typage, renommage) : chaque reconstuction
 * recrée les écouteurs avec les éléments, sans doublon sur le DOM statique.
 */
function buildTestBenchSliders() {
    const grid = document.getElementById('slider-grid');
    if (!grid) return;
    grid.innerHTML = '';

    for (let i = 0; i < 16; i++) {
        const unidir = (pwmChType(i) === false);
        const minVal = unidir ? 0 : MIN_US;
        const maxVal = unidir ? 100 : MAX_US;
        const defVal = unidir ? 0 : NEUTRAL_US;

        const item = document.createElement('div');
        item.className = 'slider-item';
        item.innerHTML = `
            <div class="sl-header">
                <span class="sl-label">CH${i}: <span id="sl-lbl-${i}"></span></span>
                <span class="sl-value" id="sl-val-${i}"></span>
            </div>
            <input type="range" id="sl-${i}" min="${minVal}" max="${maxVal}" value="${defVal}"${testUnlocked ? '' : ' disabled'}>`;
        grid.appendChild(item);

        const lblEl = item.querySelector('.sl-label span');
        if (lblEl) lblEl.textContent = CH_NAMES[i];
        const valEl = item.querySelector('.sl-value');
        if (valEl) valEl.textContent = unidir ? defVal + ' %' : defVal + ' µs';

        const slider = item.querySelector('input[type="range"]');
        slider.addEventListener('input', () => {
            const val = parseInt(slider.value);
            if (valEl) valEl.textContent = unidir ? val + ' %' : val + ' µs';
            if (testUnlocked && ws && ws.readyState === WebSocket.OPEN) {
                const arr = [...pwmValues];
                arr[i] = unidir ? (MIN_US + val * 10) : val;   /* % → µs : 0 %=1000, 100 %=2000 */
                ws.send(JSON.stringify({ pwm: arr }));
            }
        });
    }

    refreshBenchNeutralBtn();
}

/* Libellé du bouton de sécurité : mentionne les deux cibles (1500 µs / 0 %)
 * dès qu'au moins un canal typé 10-15 est unidirectionnel. */
function refreshBenchNeutralBtn() {
    const btnN = document.getElementById('btn-neutral');
    if (!btnN) return;
    let anyUnidir = false;
    for (let i = PWM_TYPE_FIRST; i <= PWM_TYPE_LAST; i++) {
        if (pwmChType(i) === false) { anyUnidir = true; break; }
    }
    btnN.textContent = anyUnidir
        ? 'Tout en position de sécurité (1500 µs / 0 %)'
        : 'Tout au Neutre (1500 µs)';
}

/* Construit les 4 tuiles ON/OFF du banc de test (noms NVS, état réel relu par
 * le firmware — synchronisé par la télémétrie WebSocket). */
function buildBenchGpio() {
    const grid = document.getElementById('bench-gpio-grid');
    if (!grid) return;
    grid.innerHTML = '';
    for (let i = 0; i < 4; i++) {
        const item = document.createElement('div');
        item.className = 'gpio-test-item bench-gpio-item';
        item.innerHTML = `
            <span id="bench-gpio-name-${i}"></span>
            <span class="gpio-st gpio-st-off" id="bench-gpio-st-${i}">OFF</span>
            <button type="button" class="gpio-test-btn" id="bench-gpio-test-${i}">Activer</button>`;
        grid.appendChild(item);

        const nameEl = item.querySelector('span:first-child');
        if (nameEl) nameEl.textContent = GPIO_NAMES[i];
        const btn = item.querySelector('.gpio-test-btn');
        btn.addEventListener('click', () => toggleGpioOutput(i, btn));
    }
    updateGpioStateUI(_gpioState);   /* applique l'état courant aux nouvelles tuiles */
}

function initTestBench() {
    buildTestBenchSliders();
    buildBenchGpio();

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
        /* Positions de sécurité par canal : miroir du firmware setAllNeutral()
         * (1500 µs bidir, 1000 µs = 0 % pour les canaux unidirectionnels). */
        const arr = new Array(16);
        for (let i = 0; i < 16; i++) arr[i] = benchSafeUs(i);
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ pwm: arr }));
        }
        for (let i = 0; i < 16; i++) {
            const sl = document.getElementById(`sl-${i}`);
            if (!sl) continue;
            const unidir = (pwmChType(i) === false);
            const def = unidir ? 0 : NEUTRAL_US;
            sl.value = def;
            const valEl = document.getElementById(`sl-val-${i}`);
            if (valEl) valEl.textContent = unidir ? def + ' %' : def + ' µs';
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

    /* Charger le typage PWM (canaux 10-15) et la config GPIO ON/OFF */
    loadPwmConfig();
    loadGpioConfig();

    /* Bouton sauvegarde broches I2C bus n°1 — même comportement que le bus
     * n°2 : sauvegarde NVS puis redémarrage automatique du contrôleur après
     * 3 s (les nouvelles broches ne sont appliquées qu'au boot). */
    const btnI2C = document.getElementById('btn-save-i2c-pins');
    if (btnI2C) {
        btnI2C.addEventListener('click', async () => {
            const sda = parseInt(document.getElementById('i2c-sda-pin').value, 10);
            const scl = parseInt(document.getElementById('i2c-scl-pin').value, 10);
            if (isNaN(sda) || isNaN(scl) || sda < 0 || sda > 48 || scl < 0 || scl > 48) {
                alert('GPIO invalide : valeurs entre 0 et 48.');
                return;
            }
            if (!confirm('Sauvegarder les broches du bus n°1 (SDA=GPIO ' + sda
                        + ', SCL=GPIO ' + scl + ') et redémarrer le contrôleur ?')) {
                return;
            }
            btnI2C.disabled = true;
            try {
                const res = await fetch('/api/i2c/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'bus=1&sda=' + sda + '&scl=' + scl
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    showRestartOverlay(3, 1, sda, scl);
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                    btnI2C.disabled = false;
                }
            } catch (e) {
                alert('Erreur réseau : ' + e.message);
                btnI2C.disabled = false;
            }
        });
    }

    /* Bouton sauvegarde broches I2C bus n°2 — même comportement que le bus
     * n°1 : sauvegarde NVS puis redémarrage automatique du contrôleur après
     * 3 s (le bus est partagé avec les sorties PWM : les nouvelles broches
     * ne peuvent être appliquées qu'au boot). */
    const btnI2C2 = document.getElementById('btn-save-i2c2-pins');
    if (btnI2C2) {
        btnI2C2.addEventListener('click', async () => {
            const sda = parseInt(document.getElementById('i2c2-sda-pin').value, 10);
            const scl = parseInt(document.getElementById('i2c2-scl-pin').value, 10);
            if (isNaN(sda) || isNaN(scl) || sda < 0 || sda > 48 || scl < 0 || scl > 48) {
                alert('GPIO invalide : valeurs entre 0 et 48.');
                return;
            }
            if (!confirm('Sauvegarder les broches du bus n°2 (SDA=GPIO ' + sda
                        + ', SCL=GPIO ' + scl + ') et redémarrer le contrôleur ?')) {
                return;
            }
            btnI2C2.disabled = true;
            try {
                const res = await fetch('/api/i2c/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'bus=2&sda=' + sda + '&scl=' + scl
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    showRestartOverlay(3, 2, sda, scl);
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                    btnI2C2.disabled = false;
                }
            } catch (e) {
                alert('Erreur réseau : ' + e.message);
                btnI2C2.disabled = false;
            }
        });
    }

    /* Typage des canaux PWM 10-15 — appliqué à chaud, sans redémarrage */
    const btnPwmTypes = document.getElementById('btn-save-pwm-types');
    if (btnPwmTypes) {
        btnPwmTypes.addEventListener('click', async () => {
            let mask = 0;
            for (let i = 0; i < 6; i++) {
                const sel = document.getElementById('sel-pwm-type-' + (10 + i));
                if (sel && sel.value === '1') mask |= (1 << i);
            }
            btnPwmTypes.disabled = true;
            try {
                const res = await fetch('/api/pwm/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'mask=' + mask
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    applyPwmTypeMask(data.type_mask !== undefined ? data.type_mask : mask);
                    alert('Typage appliqué : masque 0x'
                          + mask.toString(16).toUpperCase().padStart(2, '0'));
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                }
            } catch (e) { alert('Erreur réseau : ' + e.message); }
            btnPwmTypes.disabled = false;
        });
    }

    /* Sauvegarde de l'assignation des 4 sorties GPIO ON/OFF — redémarrage différé */
    const btnGpioPins = document.getElementById('btn-save-gpio-pins');
    if (btnGpioPins) {
        btnGpioPins.addEventListener('click', async () => {
            const pins = [];
            for (let i = 1; i <= 4; i++) {
                pins.push(parseInt(document.getElementById('gpio-pin-' + i).value, 10));
            }
            for (const p of pins) {
                if (isNaN(p) || p < 0 || p > 48) {
                    alert('GPIO invalide : valeurs entre 0 et 48.');
                    return;
                }
            }
            if (new Set(pins).size !== 4) {
                alert('Les 4 GPIO doivent être différents.');
                return;
            }
            if (!confirm('Sauvegarder les sorties ON/OFF (GPIO ' + pins.join(', ')
                        + ') et redémarrer le contrôleur ?')) {
                return;
            }
            btnGpioPins.disabled = true;
            try {
                const res = await fetch('/api/gpio/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'p1=' + pins[0] + '&p2=' + pins[1] + '&p3=' + pins[2] + '&p4=' + pins[3]
                });
                const data = await res.json();
                if (data.status === 'ok') {
                    showGpioRestartOverlay(3, pins);
                } else {
                    alert('Erreur : ' + (data.error || 'inconnue'));
                    btnGpioPins.disabled = false;
                }
            } catch (e) {
                alert('Erreur réseau : ' + e.message);
                btnGpioPins.disabled = false;
            }
        });
    }

    /* Boutons de test des 4 sorties ON/OFF — bascule directe (REST /api/gpio/test,
     * fonction toggleGpioOutput partagée avec le Banc de Test). */
    for (let i = 0; i < 4; i++) {
        const btnTest = document.getElementById('gpio-test-' + i);
        if (btnTest) btnTest.addEventListener('click', () => toggleGpioOutput(i, btnTest));
    }

    /* Bouton lancer scan I2C */
    const btnScan = document.getElementById('btn-i2c-scan');
    if (btnScan) {
        btnScan.addEventListener('click', runI2CScan);
    }

    /* Bouton mise à jour OTA (firmware ou fichiers Web) */
    const btnOta = document.getElementById('btn-ota-upload');
    if (btnOta) {
        btnOta.addEventListener('click', () => {
            const fileInput = document.getElementById('ota-file');
            const file = fileInput && fileInput.files[0];
            if (!file) {
                alert('Sélectionnez un fichier .bin (firmware.bin ou littlefs.bin).');
                return;
            }
            const target = document.getElementById('ota-target').value;
            const label = (target === 'firmware') ? 'le micrologiciel' : 'les fichiers Web';
            if (!confirm('Téléverser « ' + file.name + ' » (' + Math.round(file.size / 1024)
                        + ' Ko) vers ' + label + ' et redémarrer le contrôleur ?')) {
                return;
            }
            startOtaUpload(target, file);
        });
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
 * MISE À JOUR OTA — TÉLÉVERSEMENT FIRMWARE / FICHIERS WEB
 * ========================================================================= */

/**
 * Téléverse un fichier .bin (firmware ou image LittleFS) vers l'ESP32 via
 * /api/ota/<cible>. La propulsion est neutralisée côté firmware avant
 * l'écriture flash ; l'interface affiche la progression dans un overlay plein
 * écran (ne rien fermer pendant l'écriture de la partition fichiers Web),
 * puis attend la preuve du redémarrage avant de se recharger.
 * @param {string} target 'firmware' | 'filesystem'
 * @param {File} file Fichier .bin à téléverser
 */
async function startOtaUpload(target, file) {
    /* Uptime actuel : après redémarrage il repartira de zéro — c'est la
     * preuve que le contrôleur a bien rebooté (et pas seulement répondu). */
    let uptimeBefore = 0;
    try {
        const res = await fetch('/api/system/status', { cache: 'no-store' });
        const data = await res.json();
        uptimeBefore = Number(data.uptime_ms) || 0;
    } catch (_) { /* pas bloquant : fallback à délai fixe dans waitForReboot */ }

    const overlay = document.createElement('div');
    overlay.className = 'restart-overlay';
    const setStatus = (html) => {
        overlay.innerHTML = '<div class="restart-box">'
            + '<h3>Mise à jour OTA</h3>'
            + '<div class="ota-progress"><div class="ota-progress-bar" id="ota-bar"></div></div>'
            + '<p>' + html + '</p>'
            + '</div>';
    };
    setStatus('Préparation…');
    document.body.appendChild(overlay);

    const setProgress = (pct) => {
        const bar = document.getElementById('ota-bar');
        if (bar) bar.style.width = pct + '%';
    };

    /* Corps brut (pas de FormData multipart) : le parseur multipart du
     * serveur perd silencieusement les gros corps multi-paquets — le .bin
     * est envoyé tel quel en application/octet-stream. */
    const xhr = new XMLHttpRequest();
    xhr.upload.onprogress = (ev) => {
        if (ev.lengthComputable) {
            const pct = Math.round((ev.loaded / ev.total) * 100);
            setProgress(pct);
            setStatus('Transfert : ' + pct + ' % (' + Math.round(ev.loaded / 1024)
                     + ' / ' + Math.round(ev.total / 1024) + ' Ko)<br>'
                     + 'Ne pas fermer la page.');
        }
    };
    xhr.onload = () => {
        if (xhr.status === 200) {
            setProgress(100);
            setStatus('Écriture flash terminée.<br>Redémarrage du contrôleur…');
            waitForReboot(uptimeBefore);
            return;
        }
        overlay.remove();
        let msg = 'erreur inconnue';
        try { msg = JSON.parse(xhr.responseText).error || msg; } catch (_) {}
        alert('Échec de la mise à jour : ' + msg);
    };
    xhr.onerror = () => {
        overlay.remove();
        alert('Erreur réseau pendant le transfert.');
    };
    xhr.open('POST', '/api/ota/' + target);
    xhr.setRequestHeader('Content-Type', 'application/octet-stream');
    xhr.send(file);
}

/**
 * Attend le redémarrage effectif du contrôleur après une mise à jour OTA :
 * l'API /api/system/status ne déclenche le rechargement que lorsque l'uptime
 * repart de zéro (un simple code 200 peut venir de l'ancien firmware encore
 * vivant pendant le compte à rebours de 3 s).
 * @param {number} uptimeBefore Uptime (ms) relevé avant la mise à jour (0 = inconnu)
 */
async function waitForReboot(uptimeBefore) {
    /* Redémarrage programmé à 3 s + boot complet (~6-10 s) : première sonde
     * après cette fenêtre ; sans référence d'uptime, marge fixe plus large. */
    await new Promise(r => setTimeout(r, uptimeBefore > 0 ? 6000 : 14000));
    const ping = setInterval(async () => {
        try {
            const res = await fetch('/api/system/status', { cache: 'no-store' });
            if (!res.ok) return;
            const data = await res.json();
            const up = Number(data.uptime_ms) || 0;
            const rebooted = (uptimeBefore > 0) ? up < uptimeBefore : up < 60000;
            if (rebooted) { clearInterval(ping); location.reload(); }
        } catch (_) { /* ESP32 en cours de redémarrage : nouvelle tentative */ }
    }, 2000);
}

/* =========================================================================
 * DIAGNOSTIC I2C — CONFIGURATION & SCAN
 * ========================================================================= */

/**
 * Construit le schéma ASCII d'un bus I2C (rails SDA/SCL, modules, pull-ups)
 * avec les broches réellement configurées. L'alignement des boîtes est
 * calculé : tout reste droit quel que soit le nombre de chiffres des GPIO.
 * @param {number} sda Broche SDA du bus
 * @param {number} scl Broche SCL du bus
 * @param {Array<{name:string, addr:string}>} modules Modules branchés sur le bus
 * @returns {string} Texte du schéma (rendu dans un <pre>)
 */
function buildI2CBusDiagram(sda, scl, modules) {
    const inner = Math.max(...modules.map(m => Math.max(m.name.length, m.addr.length)));
    const bw    = inner + 2;                       /* largeur totale d'une boîte */
    const off   = Math.floor((bw - 1) / 2);        /* position du nœud dans la bordure */
    const gap   = bw + 2;                          /* écart entre modules */
    const l1    = ' GPIO ' + sda + ' (SDA)';
    const l2    = ' GPIO ' + scl + ' (SCL)';
    const w     = Math.max(l1.length, l2.length);  /* labels calés sur la même largeur */
    const head  = w + 3;                           /* colonne du premier nœud */

    const center = (s) => {
        const esp = inner - s.length;
        return ' '.repeat(Math.ceil(esp / 2)) + s + ' '.repeat(Math.floor(esp / 2));
    };
    const rail = (lbl, node) => lbl.padEnd(w) + ' ──'
        + modules.map((_, i) => (i ? '─'.repeat(gap - 1) : '') + node).join('')
        + '───[4,7 kΩ]── 3V3';
    const vline = () => ' '.repeat(head)
        + modules.map((_, i) => (i ? ' '.repeat(gap - 1) : '') + '│').join('');

    let top = '', names = '', addrs = '', bot = '';
    let cursor = 0;
    modules.forEach((m, i) => {
        const start = head + i * gap - off;        /* colonne du coin ┌ de la boîte i */
        const sp    = ' '.repeat(start - cursor);
        const dashB = '─'.repeat(off - 1);         /* tirets avant le nœud */
        const dashA = '─'.repeat(bw - off - 2);    /* tirets après le nœud */
        top   += sp + '┌' + dashB + '┴' + dashA + '┐';
        names += sp + '│' + center(m.name) + '│';
        addrs += sp + '│' + center(m.addr) + '│';
        bot   += sp + '└' + dashB + '┬' + dashA + '┘';
        cursor = start + bw;
    });

    return rail(l1, '┬') + '\n' + vline() + '\n' + top + '\n' + names + '\n'
         + addrs + '\n' + bot + '\n' + vline() + '\n' + rail(l2, '┴') + '\n\n'
         + ' 3V3 ──► VCC des ' + modules.length + ' modules        '
         + 'GND ──► GND des ' + modules.length + ' modules\n';
}

/**
 * Met à jour toute la page « Câblage & Schémas » (titres, schémas ASCII,
 * tableaux de brochage et de modules) avec les broches réellement
 * configurées des deux bus.
 */
function updateWiringPins(sda, scl, sda2, scl2) {
    const bus1Title = document.getElementById('wiring-bus1-title');
    const bus2Title = document.getElementById('wiring-bus2-title');
    if (bus1Title) bus1Title.textContent =
        'Bus n°1 — Wire (GPIO ' + sda + ' / GPIO ' + scl + ') : BNO085 + INA226 ×3';
    if (bus2Title) bus2Title.textContent =
        'Bus n°2 — Wire1 (GPIO ' + sda2 + ' / GPIO ' + scl2 + ') : MS5837 + PCA9685';

    const bus1 = document.getElementById('wiring-diagram-bus1');
    if (bus1) bus1.textContent = buildI2CBusDiagram(sda, scl, [
        { name: 'BNO085', addr: '0x4A' },
        { name: 'INA226', addr: '0x41' },
        { name: 'INA226', addr: '0x44' },
        { name: 'INA226', addr: '0x45' }
    ]);
    const bus2 = document.getElementById('wiring-diagram-bus2');
    if (bus2) bus2.textContent = buildI2CBusDiagram(sda2, scl2, [
        { name: 'MS5837', addr: '0x76' },
        { name: 'PCA9685', addr: '0x40' }
    ]);

    const setPin = (id, v) => {
        const el = document.getElementById(id);
        if (el) el.textContent = 'GPIO ' + v;
    };
    setPin('pin-b1-sda', sda);
    setPin('pin-b1-scl', scl);
    setPin('pin-b2-sda', sda2);
    setPin('pin-b2-scl', scl2);

    document.querySelectorAll('.mod-bus1').forEach(el => {
        el.textContent = 'n°1 (GPIO ' + sda + '/' + scl + ')';
    });
    document.querySelectorAll('.mod-bus2').forEach(el => {
        el.textContent = 'n°2 (GPIO ' + sda2 + '/' + scl2 + ')';
    });

    const empty = document.getElementById('i2c-scan-empty');
    if (empty) empty.textContent =
        'Aucun périphérique détecté sur les deux bus I2C. Vérifiez le câblage : '
        + 'bus n°1 (SDA=GPIO ' + sda + ', SCL=GPIO ' + scl + '), '
        + 'bus n°2 (SDA=GPIO ' + sda2 + ', SCL=GPIO ' + scl2 + ').';
}

/**
 * Affiche un compte à rebours avant redémarrage du contrôleur (broches d'un
 * bus I2C), puis attend que l'API confirme les NOUVELLES broches avant de
 * recharger l'interface — preuve que l'ESP32 a réellement redémarré et relu
 * le NVS (un simple code 200 peut venir de l'ancien firmware encore vivant,
 * qui servirait les anciennes broches).
 * @param {number} seconds Décompte avant redémarrage de l'ESP32
 * @param {number} bus Bus concerné (1 ou 2)
 * @param {number} sda Nouvelle broche SDA attendue après redémarrage
 * @param {number} scl Nouvelle broche SCL attendue après redémarrage
 */
function showRestartOverlay(seconds, bus, sda, scl) {
    let s = seconds;
    const overlay = document.createElement('div');
    overlay.className = 'restart-overlay';
    const render = () => {
        overlay.innerHTML = '<div class="restart-box">'
            + '<h3>Redémarrage du contrôleur</h3>'
            + '<p>Broches du bus n°' + bus + ' enregistrées.<br>'
            + 'L\'ESP32 redémarre dans <span class="restart-count">' + s + '</span> s…</p>'
            + '</div>';
    };
    render();
    document.body.appendChild(overlay);

    const countdown = setInterval(() => {
        if (s > 1) { s--; render(); return; }
        clearInterval(countdown);
        overlay.innerHTML = '<div class="restart-box">'
            + '<h3>Reconnexion…</h3><p>Attente du retour du contrôleur.</p></div>';
        const ping = setInterval(async () => {
            try {
                const res = await fetch('/api/i2c/config', { cache: 'no-store' });
                if (!res.ok) return;
                const data = await res.json();
                const applied = (bus === 2)
                    ? Number(data.sda2) === sda && Number(data.scl2) === scl
                    : Number(data.sda) === sda && Number(data.scl) === scl;
                if (applied) { clearInterval(ping); location.reload(); }
            } catch (_) { /* ESP32 pas encore prêt : nouvelle tentative */ }
        }, 2000);
    }, 1000);
}

/**
 * Affiche un compte à rebours avant redémarrage du contrôleur (assignation
 * des 4 sorties GPIO ON/OFF), puis attend que l'API confirme les NOUVELLES
 * broches avant de recharger l'interface — même principe que pour les
 * broches I2C : le redémarrage réel est la seule preuve que le NVS relu.
 * @param {number} seconds Décompte avant redémarrage de l'ESP32
 * @param {number[]} pins Nouvelles broches attendues après redémarrage
 */
function showGpioRestartOverlay(seconds, pins) {
    let s = seconds;
    const overlay = document.createElement('div');
    overlay.className = 'restart-overlay';
    const render = () => {
        overlay.innerHTML = '<div class="restart-box">'
            + '<h3>Redémarrage du contrôleur</h3>'
            + '<p>Sorties ON/OFF enregistrées (GPIO ' + pins.join(', ') + ').<br>'
            + 'L\'ESP32 redémarre dans <span class="restart-count">' + s + '</span> s…</p>'
            + '</div>';
    };
    render();
    document.body.appendChild(overlay);

    const countdown = setInterval(() => {
        if (s > 1) { s--; render(); return; }
        clearInterval(countdown);
        overlay.innerHTML = '<div class="restart-box">'
            + '<h3>Reconnexion…</h3><p>Attente du retour du contrôleur.</p></div>';
        const ping = setInterval(async () => {
            try {
                const res = await fetch('/api/gpio/config', { cache: 'no-store' });
                if (!res.ok) return;
                const data = await res.json();
                const applied = Array.isArray(data.pins)
                    && data.pins.length === 4
                    && data.pins.every((v, idx) => Number(v) === pins[idx]);
                if (applied) { clearInterval(ping); location.reload(); }
            } catch (_) { /* ESP32 pas encore prêt : nouvelle tentative */ }
        }, 2000);
    }, 1000);
}

/**
 * Charge la configuration I2C actuelle (broches SDA/SCL des deux bus) depuis
 * l'API, met à jour l'affichage, les champs de saisie et la page Câblage.
 */
async function loadI2CConfig() {
    try {
        const res  = await fetch('/api/i2c/config');
        const data = await res.json();
        const sda  = data.sda  !== undefined ? data.sda  : 10;
        const scl  = data.scl  !== undefined ? data.scl  : 11;
        const sda2 = data.sda2 !== undefined ? data.sda2 : 6;
        const scl2 = data.scl2 !== undefined ? data.scl2 : 7;
        document.getElementById('i2c-sda-pin').value = sda;
        document.getElementById('i2c-scl-pin').value = scl;
        const infoEl = document.getElementById('i2c-current-pins');
        if (infoEl) infoEl.textContent = 'SDA=GPIO ' + sda + ', SCL=GPIO ' + scl;
        document.getElementById('i2c2-sda-pin').value = sda2;
        document.getElementById('i2c2-scl-pin').value = scl2;
        const info2El = document.getElementById('i2c2-current-pins');
        if (info2El) info2El.textContent = 'SDA=GPIO ' + sda2 + ', SCL=GPIO ' + scl2;
        updateWiringPins(sda, scl, sda2, scl2);
    } catch (_) { /* silencieux si pas de connexion */ }
}

/**
 * Charge le typage actuel des canaux PWM 10-15 depuis l'API, l'applique aux
 * sélecteurs de l'onglet Paramètres et au rendu du PWM Monitor.
 */
async function loadPwmConfig() {
    try {
        const res  = await fetch('/api/pwm/config');
        const data = await res.json();
        const mask = (data.type_mask !== undefined) ? (data.type_mask & 0x3F) : 0x33;
        for (let i = 0; i < 6; i++) {
            const sel = document.getElementById('sel-pwm-type-' + (10 + i));
            if (sel) sel.value = ((mask >> i) & 1) ? '1' : '0';
        }
        applyPwmTypeMask(mask);
    } catch (_) { /* silencieux si pas de connexion */ }
}

/**
 * Charge l'assignation des 4 sorties GPIO ON/OFF depuis l'API et met à jour
 * les champs de saisie + les pastilles d'état (masque réel relu côté firmware).
 */
async function loadGpioConfig() {
    try {
        const res  = await fetch('/api/gpio/config');
        const data = await res.json();
        if (Array.isArray(data.pins)) {
            for (let i = 0; i < 4; i++) {
                const el = document.getElementById('gpio-pin-' + (i + 1));
                if (el && data.pins[i] !== undefined) el.value = data.pins[i];
            }
        }
        if (data.state !== undefined) updateGpioStateUI(data.state);
    } catch (_) { /* silencieux si pas de connexion */ }
}

/**
 * Lance le scan des deux bus I2C via l'API REST et affiche les résultats
 * dans un tableau (colonne Bus : n°1 = BNO085 + INA226, n°2 = MS5837 + PCA9685).
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
                var tdBus = document.createElement('td');
                tdBus.textContent = (d.bus === 2)
                    ? 'n°2 (GPIO 6/7)'
                    : 'n°1 (GPIO ' + data.sda + '/' + data.scl + ')';
                var tdStatus = document.createElement('td');
                tdStatus.innerHTML = '<span class="i2c-ack-badge">ACK</span>';
                var tdName = document.createElement('td');
                tdName.textContent = d.name;
                tr.appendChild(tdHex);
                tr.appendChild(tdDec);
                tr.appendChild(tdBus);
                tr.appendChild(tdStatus);
                tr.appendChild(tdName);
                if (tbody) tbody.appendChild(tr);
            });
            if (resultsDiv) resultsDiv.style.display = 'block';
        }

        /* Mettre à jour les infos de broches dans la section config */
        if (data.sda !== undefined && data.scl !== undefined) {
            const infoEl = document.getElementById('i2c-current-pins');
            if (infoEl) infoEl.textContent = 'SDA=GPIO ' + data.sda + ', SCL=GPIO ' + data.scl
                + ' — bus n°2 (fixe) : SDA=GPIO 6 / SCL=GPIO 7';
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

function setText(id, text) { const el = getEl(id); if (el && el.textContent !== text) el.textContent = text; }

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

    /* Ne replanifier que tant que la convergence n'est pas atteinte : évite de
     * redessiner les canvas 60×/s quand l'attitude est stable (perf). */
    const settled =
        Math.abs(_avSmooth._tRoll  - _avSmooth.roll)  < 0.05 &&
        Math.abs(_avSmooth._tPitch - _avSmooth.pitch) < 0.05 &&
        Math.abs(dy) < 0.05;
    _avSmooth._raf = settled ? 0 : requestAnimationFrame(_avLerpLoop);
}

function avSetTargets(roll, pitch, yaw) {
    _avSmooth._tRoll  = roll;
    _avSmooth._tPitch = pitch;
    _avSmooth._tYaw   = yaw;
    if (_avSmooth._raf) return;   /* boucle déjà active : elle lira les nouvelles cibles */
    /* Boucle arrêtée (convergée) : ne relancer que si les cibles ont vraiment bougé,
     * sinon on redessinerait les canvas pour rien à chaque trame. */
    let dy = yaw - _avSmooth.yaw;
    if (dy > 180) dy -= 360;
    if (dy < -180) dy += 360;
    if (Math.abs(roll  - _avSmooth.roll)  > 0.05 ||
        Math.abs(pitch - _avSmooth.pitch) > 0.05 ||
        Math.abs(dy) > 0.05) {
        _avLerpLoop();
    }
}

/**
 * Dessine un horizon artificiel style aviation.
 * @param {number} roll  — Roulis en degrés (+droite / -gauche)
 * @param {number} pitch — Tangage en degrés (+nez haut / -nez bas)
 */
function drawArtificialHorizon(roll, pitch) {
    const c = getCtx('cv-horizon');
    if (!c) return;
    const cv = c.cv, ctx = c.ctx;
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
    const c = getCtx('cv-hsi');
    if (!c) return;
    const cv = c.cv, ctx = c.ctx;
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
 * VITESSE VERTICALE (VSI sous-marin) — TAUX DE REMONTÉE / DESCENTE EN cm/s
 * ========================================================================= */

const VSI_MAX_CMS = 100;   /* Pleine échelle : ±100 cm/s */
const VSI_MIN_DT_S = 0.4;  /* Fenêtre minimale (s) entre deux calculs de dérivée
                            * (~période baro 500 ms). Empêche les pics dus aux
                            * re-broadcasts 20 Hz à profondeur inchangée. */
const VSI_REST_EPS = 1.0;  /* En dessous (cm/s), l'aiguille est ramenée au repos franc */
let _vsiPrevDepth = null;  /* Profondeur de la dernière fenêtre de mesure (m) */
let _vsiPrevT   = 0;       /* Horodatage de la dernière fenêtre de mesure (ms) */
let _vsiRate    = 0;       /* Vitesse verticale lissée (cm/s) : + = remontée, − = descente */

/**
 * Met à jour l'indicateur de vitesse verticale à partir de la profondeur
 * télémétrée (usage ROV / sous-marin).
 *
 * Convention : la profondeur est positive vers le bas. La vitesse affichée suit
 * la convention « + = remontée, − = descente » (aiguille vers le haut quand le
 * ROV remonte), cohérente avec le cadran. Dérivée temporelle lissée par filtre
 * exponentiel (anti-gigue).
 *
 * @param {number} depthM — Profondeur courante en mètres (positive vers le bas)
 */
function updateVSI(depthM) {
    const t = performance.now();
    /* Baromètre rafraîchi à ~2 Hz côté firmware, mais re-broadcasté à 20 Hz : la
     * dérivée est recalculée sur une FENÊTRE TEMPORELLE (>= VSI_MIN_DT_S) et non
     * à chaque changement de valeur. Ainsi, à profondeur stabilisée, Δdepth ≈ 0
     * sur la fenêtre → la vitesse tend vers 0 et l'aiguille REVIENT AU REPOS
     * (l'ancien déclenchement « si la valeur a changé » la laissait figée). */
    if (_vsiPrevDepth === null) {
        _vsiPrevDepth = depthM;
        _vsiPrevT = t;
    } else {
        const dt = (t - _vsiPrevT) / 1000;
        if (dt >= VSI_MIN_DT_S) {
            /* d(profondeur)/dt > 0 quand le ROV descend → signe inversé pour
             * obtenir « + = remontée ». m/s → cm/s. */
            const raw = -((depthM - _vsiPrevDepth) / dt) * 100;
            _vsiRate += (raw - _vsiRate) * 0.4;
            _vsiPrevDepth = depthM;
            _vsiPrevT = t;
        }
    }

    /* Repos franc : évite une aiguille résiduelle asymptotique proche de 0 */
    if (Math.abs(_vsiRate) < VSI_REST_EPS) _vsiRate = 0;

    drawVSI(_vsiRate);
    setText('av-alt', depthM.toFixed(2) + ' m');
    const el = getEl('av-vs');
    if (el) {
        const vsTxt = (_vsiRate >= 0 ? '+' : '') + Math.round(_vsiRate) + ' cm/s';
        if (el.textContent !== vsTxt) el.textContent = vsTxt;
        /* Vert = remontée, ambre = descente */
        el.style.color = (Math.abs(_vsiRate) < 5) ? '' : (_vsiRate > 0 ? 'var(--success)' : 'var(--warning)');
    }
}

/**
 * Dessine l'indicateur de vitesse verticale sous-marin : zéro à 9h, remontée
 * par le haut, descente par le bas, pleine échelle ±100 cm/s à 3h.
 * @param {number} vsCms — Vitesse verticale en cm/s (+ = remontée, − = descente)
 */
function drawVSI(vsCms) {
    const c = getCtx('cv-vsi');
    if (!c) return;
    const cv = c.cv, ctx = c.ctx;
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

    /* Indicateurs remontée / descente (usage sous-marin) */
    ctx.font = 'bold 13px monospace';
    ctx.fillStyle = '#00e676';
    ctx.fillText('▲', -R * 0.42, -R * 0.5);   /* remontée */
    ctx.fillStyle = '#ffb300';
    ctx.fillText('▼', -R * 0.42, R * 0.5);    /* descente */

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

/**
 * Initialise le bouton « Tare Surface » : envoie {tare_surface:true} au firmware
 * pour ré-étalonner la pression de surface P_surface. À utiliser capteur HORS de
 * l'eau (avant immersion) afin que la profondeur soit mesurée depuis la surface.
 */
function initTareButton() {
    const btn = getEl('btn-tare-surface');
    if (!btn) return;
    btn.addEventListener('click', () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ tare_surface: true }));
            btn.textContent = 'Tare demandée…';
            setTimeout(() => { btn.textContent = 'Tare Surface'; }, 1500);
        }
    });
}

/**
 * Initialise le bouton « Régler Nord » : envoie {tare_north:true} au firmware
 * pour remettre le cap à zéro sur le Nord réel. Procédure : ROV immobile, nez
 * pointé vers le Nord (boussole du téléphone ou repère connu), de préférence en
 * mode PASSIF — sinon le hold de cap de l'AUTO FULL rattraperait l'ancien cap.
 * L'offset s'applique à la source côté firmware : télémétrie, HSI, modèle 3D
 * et PID de cap partagent le même Nord.
 * Avec le magnétomètre actif (Rotation Vector 9-DOF du BNO085), la tare est un
 * ajustage fin de l'écart résiduel (déclinaison locale, léger biais) : une fois
 * suffit. En secours relatif (Game Rotation Vector, magnéto indisponible), elle
 * redevient la référence de cap obligatoire après chaque redémarrage.
 */
function initNorthTareButton() {
    const btn = getEl('btn-tare-north');
    if (!btn) return;
    btn.addEventListener('click', () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ tare_north: true }));
            btn.textContent = 'Tare Nord…';
            setTimeout(() => { btn.textContent = 'Régler Nord'; }, 1500);
        }
    });
}

/* =========================================================================
 * MODÈLE 3D DU ROV — bob_rov_3D.glb animé par l'IMU (three.js r128 local)
 * =========================================================================
 *
 * Remplace l'ancien affichage Euler du cadre « Attitude (IMU) » : le modèle
 * exporté d'Onshape tourne en temps réel avec le roulis, le tangage et le cap.
 * three.js est servi localement (/vendor/three.min.js) — aucun CDN : le ROV
 * diffuse son propre point d'accès sans Internet. Le chargement du .glb passe
 * par un parseur GLB minimal embarqué (le modèle n'utilise que des couleurs
 * plates, sans texture) plutôt que par la bibliothèque GLTFLoader.
 * ========================================================================= */

/* Signes d'orientation : si un axe parait inversé ou en miroir sur votre
 * modèle, inversez le signe correspondant. Conventions (modèle glTF standard
 * avant -Z, droite +X) : heading + = rotation horaire vue du dessus ;
 * pitch + = nez qui monte ; roll + = épaule droite qui descend. */
/* Signes d'orientation — retours terrain successifs (17/09) :
 * - pitch +1 et heading -1 validés après remappage du montage BNO085 À LA
 *   SOURCE : lever l'avant → pitch + (le nez du modèle monte), cap suivi.
 * - roll +1 (test terrain 17/09 soir) : lever X+ (droite du ROV) doit
 *   montrer, À L'ÉCRAN, la droite du modèle DESCENDRE. Le roulis est le
 *   SEUL axe qui dépend du point de vue (tangage/cap invariants) : les
 *   tests se font FACE AU NEZ du ROV, à l'opposé de la caméra par défaut
 *   (vue arrière-quarter) — le sens « réplique vue de l'arrière » (roll
 *   -1, théorique) paraissait inversé à l'utilisateur. Convention alignée
 *   sur l'horizon artificiel (validé « parfait »). */
const R3D_SIGN = { heading: -1, pitch: 1, roll: 1 };

/* Réalignement du modèle : l'avant du GLB exporté d'Onshape pointe +X
 * (décalé de 90° par rapport à la convention glTF avant = -Z) — confirmé au
 * test du 17/09 : horizon correct après remappage BNO085 à la source, mais
 * tangage/roulis PERMUTÉS sur le seul modèle 3D avec 0. Les DEUX rotations
 * de 90° sont indépendantes et se CUMULENT : remappage du MONTAGE CAPTEUR
 * côté firmware (quaternion → repère véhicule) + rotation du MODÈLE côté
 * affichage (géométrie d'export du GLB). Passer à -90 si le sens reste
 * opposé après test. */
const R3D_MODEL_YAW_DEG = 90;

/* Vue caméra par défaut — le double-clic y revient ; survit aux rechargements
 * et aux redémarrages via localStorage (même mécanisme que l'ordre des tuiles). */
const R3D_VIEW_KEY = 'bobcontrol_rov3dView';
const R3D_VIEW_DEFAULT = { theta: Math.PI / 5, phi: Math.PI / 3.1, dist: 4.4 };

const _rov3d = {
    ready: false,
    renderer: null, scene: null, camera: null, pivot: null, host: null,
    raf: 0, mats: [],
    /* Orbite maison : glisser = tourner autour, molette = zoom, dbl-clic = reset */
    theta: R3D_VIEW_DEFAULT.theta, phi: R3D_VIEW_DEFAULT.phi, dist: R3D_VIEW_DEFAULT.dist,
    _drag: false, _px: 0, _py: 0
};

/* Restaure la vue mémorisée (zoom + orientation) — valeurs revalidées
 * (nombres finis, phi/dist clampés aux bornes d'interaction) par sécurité. */
(function _rov3dRestoreView() {
    try {
        const v = JSON.parse(localStorage.getItem(R3D_VIEW_KEY) || 'null');
        if (v && isFinite(v.theta) && isFinite(v.phi) && isFinite(v.dist)) {
            _rov3d.theta = v.theta;
            _rov3d.phi   = Math.min(Math.max(v.phi, 0.2), Math.PI - 0.2);
            _rov3d.dist  = Math.min(Math.max(v.dist, 2.2), 10);
        }
    } catch (_) {}
})();

/* Mémorise la vue caméra — débouncée (300 ms) pour n'écrire qu'une fois par
 * rafale de molette ou de glisser, pas à chaque pixel. */
let _rov3dSaveTimer = 0;
function _rov3dSaveView() {
    clearTimeout(_rov3dSaveTimer);
    _rov3dSaveTimer = setTimeout(() => {
        try {
            localStorage.setItem(R3D_VIEW_KEY, JSON.stringify({
                theta: _rov3d.theta, phi: _rov3d.phi, dist: _rov3d.dist
            }));
        } catch (_) {}
    }, 300);
}

function _rov3dMsg(html, isError) {
    const st = getEl('rov-3d-status');
    if (!st) return;
    st.innerHTML = html;
    st.style.display = 'flex';
    st.style.color = isError ? 'var(--danger)' : '';
}

/**
 * Applique l'état des cases « Filaire » et « Vert » aux matériaux du modèle :
 * wireframe sur tous, lignes teintées vert (couleur + émissif pour rester
 * lisibles quel que soit l'éclairage) si « Vert » est coché. « Vert » n'agit
 * que sur le filaire — le modèle plein garde ses couleurs d’origine, mémorisées
 * dans userData._baseColor à la collecte des matériaux.
 */
function _rov3dApplyWire() {
    const wEl = getEl('chk-rov-wire');
    const gEl = getEl('chk-rov-green');
    const wire = wEl ? wEl.checked : false;
    const green = gEl ? gEl.checked : false;
    const tint = new THREE.Color(0x00e676).convertSRGBToLinear();
    _rov3d.mats.forEach(m => {
        m.wireframe = wire;
        if (wire && green) {
            m.color.copy(tint);
            m.emissive.copy(tint);
            m.emissiveIntensity = 0.35;
        } else if (m.userData._baseColor) {
            m.color.copy(m.userData._baseColor);
            m.emissive.setRGB(0, 0, 0);
            m.emissiveIntensity = 1;
        }
    });
}

/**
 * Parseur GLB minimal (glTF 2.0 binaire) — sous-ensemble sans texture.
 *
 * Le modèle bob_rov_3D.glb exporté d'Onshape n'utilise que : attributs
 * POSITION/NORMAL (VEC3 float), indices uint32, matériaux PBR à couleur
 * plate (baseColorFactor), nodes sans transformation. Reste défensif
 * (matrices TRS, indices uint16) pour d'autres exports.
 * @param {ArrayBuffer} arrayBuffer Contenu brut du fichier .glb
 * @returns {THREE.Group} Hiérarchie complète du modèle
 */
function parseGLB(arrayBuffer) {
    const dv = new DataView(arrayBuffer);
    if (arrayBuffer.byteLength < 20 || dv.getUint32(0, true) !== 0x46546C67) {
        throw new Error('magic glTF absent (pas un .glb)');
    }

    /* En-tête (12 o) puis chunks : JSON (0x4E4F534A) + BIN (0x004E4942) */
    let json = null, bin = null;
    let off = 12;
    while (off + 8 <= arrayBuffer.byteLength) {
        const len = dv.getUint32(off, true);
        const type = dv.getUint32(off + 4, true);
        if (type === 0x4E4F534A) {
            json = JSON.parse(new TextDecoder().decode(new Uint8Array(arrayBuffer, off + 8, len)));
        } else if (type === 0x004E4942) {
            bin = new Uint8Array(arrayBuffer, off + 8, len);
        }
        off += 8 + len;
    }
    if (!json || !bin) throw new Error('chunks JSON/BIN introuvables');

    /* Lecture d'un accessor → tableau typé (alignement 4 garanti par glTF) */
    function acc(idx) {
        const a = json.accessors[idx];
        const bv = json.bufferViews[a.bufferView];
        if (bv.byteStride) throw new Error('bufferView intercalé non supporté');
        const base = (bv.byteOffset || 0) + (a.byteOffset || 0) + bin.byteOffset;
        if (a.type === 'VEC3' && a.componentType === 5126) return new Float32Array(bin.buffer, base, a.count * 3);
        if (a.type === 'VEC2' && a.componentType === 5126) return new Float32Array(bin.buffer, base, a.count * 2);
        if (a.type === 'SCALAR' && a.componentType === 5123) return new Uint16Array(bin.buffer, base, a.count);
        if (a.type === 'SCALAR' && a.componentType === 5125) return new Uint32Array(bin.buffer, base, a.count);
        throw new Error('accessor non supporté (' + a.type + '/' + a.componentType + ')');
    }

    /* Matériaux PBR à couleur plate (baseColorFactor est en sRGB d'après la
     * spec glTF → conversion linéaire pour un rendu fidèle en sortie sRGB) */
    const mats = (json.materials || []).map(m => {
        const pbr = m.pbrMetallicRoughness || {};
        const c = pbr.baseColorFactor || [0.75, 0.75, 0.78];
        return new THREE.MeshStandardMaterial({
            color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(),
            metalness: pbr.metallicFactor !== undefined ? pbr.metallicFactor : 0.1,
            roughness: pbr.roughnessFactor !== undefined ? pbr.roughnessFactor : 0.85,
            side: m.doubleSided ? THREE.DoubleSide : THREE.FrontSide
        });
    });
    if (!mats.length) mats.push(new THREE.MeshStandardMaterial({ color: 0x808080 }));

    /* Construction récursive de la hiérarchie de nodes */
    function node(idx) {
        const n = json.nodes[idx];
        const obj = new THREE.Group();
        obj.name = n.name || ('node' + idx);
        if (n.matrix) {
            obj.applyMatrix4(new THREE.Matrix4().fromArray(n.matrix));
        } else {
            if (n.translation) obj.position.fromArray(n.translation);
            if (n.rotation) obj.quaternion.fromArray(n.rotation);
            if (n.scale) obj.scale.fromArray(n.scale);
        }
        if (n.mesh !== undefined) {
            (json.meshes[n.mesh].primitives || []).forEach(p => {
                const g = new THREE.BufferGeometry();
                g.setAttribute('position', new THREE.BufferAttribute(acc(p.attributes.POSITION), 3));
                if (p.attributes.NORMAL !== undefined) {
                    g.setAttribute('normal', new THREE.BufferAttribute(acc(p.attributes.NORMAL), 3));
                }
                if (p.indices !== undefined) {
                    g.setIndex(new THREE.BufferAttribute(acc(p.indices), 1));
                }
                obj.add(new THREE.Mesh(g, mats[p.material] || mats[0]));
            });
        }
        (n.children || []).forEach(ci => obj.add(node(ci)));
        return obj;
    }

    const root = new THREE.Group();
    (json.scenes[json.scene || 0].nodes || []).forEach(i => root.add(node(i)));
    return root;
}

/**
 * Initialise le cadre « ROV 3D » : scène, éclairage, chargement du modèle et
 * boucle de rendu. Messages d'erreur explicites si three.js ou le modèle
 * manquent sur le serveur (uploadfs non exécuté).
 */
function initRov3D() {
    const host = getEl('rov-3d-view');
    if (!host) return;
    _rov3d.host = host;

    if (typeof THREE === 'undefined') {
        _rov3dMsg('three.js introuvable sur le serveur<br>(/vendor/three.min.js — exécuter uploadfs)', true);
        return;
    }

    /* Renderer WebGL — fond transparent laissant apparaître le fond « abyssal » */
    try {
        _rov3d.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch (e) {
        _rov3dMsg('WebGL indisponible sur cet appareil', true);
        return;
    }
    _rov3d.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    _rov3d.renderer.setSize(260, 260);
    _rov3d.renderer.outputEncoding = THREE.sRGBEncoding;
    host.appendChild(_rov3d.renderer.domElement);

    _rov3d.scene = new THREE.Scene();
    _rov3d.camera = new THREE.PerspectiveCamera(42, 1, 0.05, 100);

    /* Éclairage : ambiant + clé + contre-jour bleuté (aucune face noire) */
    _rov3d.scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const key = new THREE.DirectionalLight(0xffffff, 0.85);
    key.position.set(3, 5, 4);
    _rov3d.scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aaff, 0.35);
    fill.position.set(-4, 2, -3);
    _rov3d.scene.add(fill);

    /* Références fixes : grille polaire « plan d'eau » + flèche du Nord (cap) */
    const grid = new THREE.PolarGridHelper(1.7, 8, 3, 48, 0x2a2f3a, 0x232833);
    grid.position.y = -1.35;
    _rov3d.scene.add(grid);
    _rov3d.scene.add(new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, -1.28, 0),
        0.55, 0x00d9ff, 0.14, 0.07));

    _rov3dMsg('Chargement du modèle 3D…');

    fetch('/3D/bob_rov_3D.glb', { cache: 'no-store' })
        .then(res => { if (!res.ok) throw new Error('HTTP ' + res.status); return res.arrayBuffer(); })
        .then(buf => {
            const model = parseGLB(buf);

            /* Normalisation : centrage + échelle pour tenir dans ~2,6 unités */
            const box = new THREE.Box3().setFromObject(model);
            const size = box.getSize(new THREE.Vector3());
            const center = box.getCenter(new THREE.Vector3());
            model.position.set(-center.x, -center.y, -center.z);

            /* Réalignement du GLB : l'avant exporté d'Onshape pointe +X,
             * le frame le ramène sur -Z (convention glTF) — sans lui, le
             * tangage agit comme roulis et inversement (test terrain 17/09). */
            const frame = new THREE.Group();
            frame.rotation.y = R3D_MODEL_YAW_DEG * Math.PI / 180;
            frame.add(model);

            const holder = new THREE.Group();   /* pivot d'attitude */
            holder.add(frame);
            holder.scale.setScalar(2.6 / Math.max(size.x, size.y, size.z, 1e-6));
            holder.rotation.order = 'YXZ';      /* cap → tangage → roulis */
            _rov3d.pivot = holder;
            _rov3d.scene.add(holder);

            /* Matériaux pour « Filaire » / « Vert » : collecte + conservation des
             * couleurs d’origine, puis application de l’état courant des cases
             * (elles ont pu être cochées avant la fin du chargement). */
            _rov3d.mats = [];
            model.traverse(o => {
                if (o.isMesh) {
                    _rov3d.mats.push(...(Array.isArray(o.material) ? o.material : [o.material]));
                }
            });
            _rov3d.mats.forEach(m => { m.userData._baseColor = m.color.clone(); });
            _rov3dApplyWire();

            const st = getEl('rov-3d-status');
            if (st) st.style.display = 'none';
            _rov3d.ready = true;
            if (!_rov3d.raf) _rov3dLoop();
        })
        .catch(err => {
            _rov3dMsg('Modèle 3D indisponible<br>(' + err.message + ' — /3D/bob_rov_3D.glb)', true);
        });

    _rov3dBindControls();

    /* Cases « Filaire » et « Vert » : appliquent wireframe (+ teinte verte) */
    ['chk-rov-wire', 'chk-rov-green'].forEach(id => {
        const el = getEl(id);
        if (el) el.addEventListener('change', _rov3dApplyWire);
    });
}

/* Interactions caméra : glisser = orbite, molette = zoom, dbl-clic = reset */
function _rov3dBindControls() {
    const cv = _rov3d.renderer.domElement;
    cv.style.touchAction = 'none';
    cv.addEventListener('pointerdown', e => {
        _rov3d._drag = true;
        _rov3d._px = e.clientX;
        _rov3d._py = e.clientY;
        cv.setPointerCapture(e.pointerId);
    });
    cv.addEventListener('pointermove', e => {
        if (!_rov3d._drag) return;
        _rov3d.theta -= (e.clientX - _rov3d._px) * 0.008;
        _rov3d.phi = Math.min(Math.max(_rov3d.phi - (e.clientY - _rov3d._py) * 0.008, 0.2), Math.PI - 0.2);
        _rov3d._px = e.clientX;
        _rov3d._py = e.clientY;
    });
    /* Fin de glisser : mémoriser l'orientation obtenue */
    const end = () => {
        if (_rov3d._drag) _rov3dSaveView();
        _rov3d._drag = false;
    };
    cv.addEventListener('pointerup', end);
    cv.addEventListener('pointercancel', end);
    cv.addEventListener('dblclick', () => {
        _rov3d.theta = R3D_VIEW_DEFAULT.theta;
        _rov3d.phi   = R3D_VIEW_DEFAULT.phi;
        _rov3d.dist  = R3D_VIEW_DEFAULT.dist;
        _rov3dSaveView();   /* la vue par défaut devient la vue mémorisée */
    });
    cv.addEventListener('wheel', e => {
        e.preventDefault();
        _rov3d.dist = Math.min(Math.max(_rov3d.dist * (1 + e.deltaY * 0.001), 2.2), 10);
        _rov3dSaveView();
    }, { passive: false });
}

/**
 * Boucle de rendu : applique l'attitude lissée (_avSmooth, même source que
 * l'horizon artificiel et le HSI) au modèle et place la caméra orbitale.
 * Sautée quand la tuile Télémétrie est masquée (perf) — reprise automatique.
 */
function _rov3dLoop() {
    _rov3d.raf = requestAnimationFrame(_rov3dLoop);
    if (!_rov3d.ready || !_rov3d.host || _rov3d.host.offsetWidth === 0) return;

    const rad = Math.PI / 180;
    const r = _rov3d.pivot.rotation;
    r.y = R3D_SIGN.heading * _avSmooth.yaw * rad;
    r.x = R3D_SIGN.pitch * _avSmooth.pitch * rad;
    r.z = R3D_SIGN.roll * _avSmooth.roll * rad;

    const sp = Math.sin(_rov3d.phi);
    _rov3d.camera.position.set(
        _rov3d.dist * sp * Math.sin(_rov3d.theta),
        _rov3d.dist * Math.cos(_rov3d.phi),
        _rov3d.dist * sp * Math.cos(_rov3d.theta));
    _rov3d.camera.lookAt(0, 0, 0);
    _rov3d.renderer.render(_rov3d.scene, _rov3d.camera);
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

document.addEventListener('DOMContentLoaded', () => {
    initTheme();
    initTileNav();
    initDashboardEditor();
    initPWMGrid();
    initROVSchematic();
    initDryRun();
    initTareButton();
    initNorthTareButton();
    initTestBench();
    initOutputNames();
    loadOutputNames();
    loadAppVersion();
    initQnhCard();
    loadQnhConfig();
    initWiFi();
    initSettings();
    initAviationInstruments();
    initRov3D();
    initFlightModeButtons();
    connectWebSocket();
});
