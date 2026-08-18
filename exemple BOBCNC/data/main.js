/**
 * ============================================
 * FICHIER JAVASCRIPT - ANIMATIONS GSAP
 * Style inspire CodePen avec effets avances
 * ============================================
 */

// Polyfill GSAP minimal pour le mode hors-ligne (ESP32 sans Internet)
// Les animations sont simplement ignorees au lieu de crasher.
if (typeof gsap === 'undefined') {
    window.gsap = {
        to:        function() { return arguments[0]; },
        fromTo:    function() { return arguments[0]; },
        from:      function() { return arguments[0]; },
        set:       function() {},
        registerPlugin: function() {},
        timeline:  function() { return { to:function(){return this;}, fromTo:function(){return this;}, from:function(){return this;} }; }
    };
    console.warn('[BOBCNC] GSAP CDN indisponible (mode hors-ligne) - animations desactivees');
}

// ============================================================
//  BOBCNC SETTINGS GLOBAUX (Start / End G-code)
//  Source unique : /bobcnc_settings.json sur LittleFS de l'ESP32-S3
//  BOBCNC. AUCUN stockage localStorage : la persistance est physique
//  sur la flash de la carte BOBCNC, indépendante de FluidNC.
// ============================================================
window.bobcncSettings = window.bobcncSettings || {
    startGcode: '',
    endGcode:   '',
    loaded:     false
};

// Charge les Start/End G-code depuis l'ESP32 (GET /api/settings)
window.loadBobcncSettings = function() {
    return fetch('/api/settings', { cache: 'no-store' })
        .then(function(r) { return r.ok ? r.json() : Promise.reject(r.status); })
        .then(function(data) {
            window.bobcncSettings.startGcode = (data && typeof data.startGcode === 'string') ? data.startGcode : '';
            window.bobcncSettings.endGcode   = (data && typeof data.endGcode   === 'string') ? data.endGcode   : '';
            window.bobcncSettings.loaded     = true;
            return window.bobcncSettings;
        })
        .catch(function(err) {
            console.warn('[BOBCNC] /api/settings indisponible :', err);
            window.bobcncSettings.loaded = true; // évite de bloquer indéfiniment
            return window.bobcncSettings;
        });
};

// Sauvegarde les Start/End G-code sur l'ESP32 (POST /api/settings)
window.saveBobcncSettings = function(startGcode, endGcode) {
    var payload = {
        startGcode: typeof startGcode === 'string' ? startGcode : '',
        endGcode:   typeof endGcode   === 'string' ? endGcode   : ''
    };
    return fetch('/api/settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(function(r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
    }).then(function(j) {
        if (j && j.success) {
            window.bobcncSettings.startGcode = payload.startGcode;
            window.bobcncSettings.endGcode   = payload.endGcode;
            window.bobcncSettings.loaded     = true;
        }
        return j;
    });
};

/**
 * Encapsule un G-code généré ("core") avec le Start G-code en tête
 * et le End G-code en queue. Retourne la chaîne complète à envoyer
 * à FluidNC. Si les paramètres ne sont pas chargés ou vides, le
 * core est renvoyé tel quel.
 */
window.wrapWithStartEndGcode = function(coreGcode) {
    if (typeof coreGcode !== 'string' || coreGcode.length === 0) {
        return coreGcode || '';
    }
    var s = (window.bobcncSettings && typeof window.bobcncSettings.startGcode === 'string')
            ? window.bobcncSettings.startGcode.replace(/\r\n/g, '\n').trim() : '';
    var e = (window.bobcncSettings && typeof window.bobcncSettings.endGcode === 'string')
            ? window.bobcncSettings.endGcode.replace(/\r\n/g, '\n').trim() : '';
    if (!s && !e) return coreGcode;
    var parts = [];
    if (s) {
        parts.push('(===== START G-CODE BOBCNC =====)');
        parts.push(s);
        parts.push('(===== FIN START G-CODE =====)');
        parts.push('');
    }
    parts.push(coreGcode);
    if (e) {
        parts.push('');
        parts.push('(===== END G-CODE BOBCNC =====)');
        parts.push(e);
        parts.push('(===== FIN END G-CODE =====)');
    }
    return parts.join('\n');
};

// Chargement automatique au démarrage de toute page incluant main.js
document.addEventListener('DOMContentLoaded', function() {
    if (typeof window.loadBobcncSettings === 'function') {
        window.loadBobcncSettings();
    }
});

// Attendre que le DOM soit complètement chargé
document.addEventListener('DOMContentLoaded', function() {
    
    // ============================================
    // ANIMATIONS D'ENTRÉE DES PAGES
    // Gérées en CSS (@keyframes) pour fiabilité
    // GSAP ne gère que les interactions
    // ============================================
    
    // Les tuiles .nav-tile et le header .index-header
    // utilisent les classes CSS .tile-enter et .header-enter
    
    // ============================================
    // EFFET DE HOVER SUR LES TUILES
    // Animation GSAP pour un effet fluide
    // ============================================
    const navTiles = document.querySelectorAll('.nav-tile');
    navTiles.forEach(tile => {
        tile.addEventListener('mouseenter', function() {
            // En mode édition (drag & drop), on n'applique pas le hover GSAP
            // pour ne pas écraser l'animation wobble + le grab cursor.
            if (this.parentElement && this.parentElement.classList.contains('editing')) return;
            gsap.to(this, {
                duration: 0.3,
                scale: 1.03,
                ease: 'power2.out'
            });
            
            // Animer l'icône SVG
            const svg = this.querySelector('svg');
            if (svg) {
                gsap.to(svg, {
                    duration: 0.3,
                    rotation: 5,
                    ease: 'power2.out'
                });
            }
        });
        
        tile.addEventListener('mouseleave', function() {
            if (this.parentElement && this.parentElement.classList.contains('editing')) return;
            gsap.to(this, {
                duration: 0.3,
                scale: 1,
                ease: 'power2.out'
            });
            
            // Réinitialiser l'icône
            const svg = this.querySelector('svg');
            if (svg) {
                gsap.to(svg, {
                    duration: 0.3,
                    rotation: 0,
                    ease: 'power2.out'
                });
            }
        });
    });
    
    // ============================================
    // ANIMATION DU BOUTON RETOUR
    // Gérée en CSS, pas besoin de JS
    // ============================================
    
    // ============================================
    // EFFET DE PARALLAXE SUR LE HEADER
    // ============================================
    let scrollTimeout;
    window.addEventListener('scroll', function() {
        if (scrollTimeout) return;
        
        scrollTimeout = setTimeout(() => {
            const scrolled = window.pageYOffset;
            const parallaxElements = document.querySelectorAll('.index-header, .page-header');
            
            parallaxElements.forEach(el => {
                gsap.to(el, {
                    duration: 0.3,
                    y: scrolled * 0.2,
                    ease: 'power1.out'
                });
            });
            
            scrollTimeout = null;
        }, 10);
    });
    
    // ============================================
    // EFFET DE BRILLANCE AU SURVOL
    // (anciennement appliqué aux liens du menu hamburger - supprimé)
    // ============================================
    
    // Header animé en CSS
    
    console.log('🚀 Menu GSAP style CodePen initialisé avec succès !');

    // ============================================
    // RÉORGANISATION DES TUILES DU TABLEAU DE BORD
    // (drag & drop persisté côté ESP32)
    // ============================================
    if (document.querySelector('.nav-grid')) {
        initDashboardEditor();
    }
});

/**
 * ==============================================================
 *  RÉORGANISATION DES TUILES DU TABLEAU DE BORD
 *  ----------------------------------------------------------------
 *  - Charge l'ordre depuis l'ESP32 (GET /api/dashboard-order)
 *  - Bouton "Réorganiser" : active le mode édition
 *  - Drag & drop multi-plateforme via Pointer Events (souris+tactile)
 *  - Bouton "Terminer" : POST /api/dashboard-order et quitte le mode
 *  - Persistance NVS Preferences côté ESP32 (même en mode AP)
 * ==============================================================
 */
function initDashboardEditor() {
    const grid     = document.querySelector('.nav-grid');
    const btnEdit  = document.getElementById('btnEditDash');
    const btnSave  = document.getElementById('btnSaveDash');
    const hint     = document.getElementById('dashEditHint');
    if (!grid || !btnEdit || !btnSave) return;

    // 1) Charger l'ordre mémorisé depuis l'ESP32 et réorganiser le DOM.
    loadDashboardOrder(grid);

    let editing = false;

    // --- Activation du mode édition ---
    btnEdit.addEventListener('click', () => {
        editing = true;
        grid.classList.add('editing');
        btnEdit.style.display = 'none';
        btnSave.style.display = 'inline-flex';
        if (hint) hint.style.display = 'inline';
        attachTileDragHandlers(grid);
    });

    // --- Sortie du mode édition + sauvegarde ESP32 ---
    btnSave.addEventListener('click', async () => {
        editing = false;
        grid.classList.remove('editing');
        btnSave.style.display = 'none';
        btnEdit.style.display = 'inline-flex';
        if (hint) hint.style.display = 'none';
        detachTileDragHandlers(grid);

        const order = Array.from(grid.querySelectorAll('.nav-tile'))
                           .map(t => t.dataset.tileId)
                           .filter(Boolean);
        try {
            await saveDashboardOrder(order);
            console.log('✅ Ordre des tuiles sauvegardé sur l\'ESP32', order);
        } catch (e) {
            console.error('❌ Échec de sauvegarde de l\'ordre des tuiles :', e);
            alert('Erreur : impossible de sauvegarder l\'organisation sur l\'ESP32.');
        }
    });
}

/**
 * Récupère l'ordre stocké sur l'ESP32 et réarrange le DOM en conséquence.
 * On applique immédiatement un cache localStorage pour éviter le "flash"
 * (le fetch peut prendre 50-200ms en mode AP).
 */
function loadDashboardOrder(grid) {
    // Cache local immédiat (UX) — source de vérité = ESP32
    try {
        const cached = localStorage.getItem('dashOrder');
        if (cached) applyDashboardOrder(grid, JSON.parse(cached));
    } catch (_) {}

    fetch('/api/dashboard-order', { cache: 'no-store' })
        .then(r => r.ok ? r.json() : null)
        .then(data => {
            if (!data || !Array.isArray(data.order) || data.order.length === 0) return;
            applyDashboardOrder(grid, data.order);
            try { localStorage.setItem('dashOrder', JSON.stringify(data.order)); } catch (_) {}
        })
        .catch(err => console.warn('Ordre des tuiles : ESP32 indisponible, cache local utilisé.', err));
}

/**
 * Réordonne les tuiles selon la liste d'ids fournie.
 * Les tuiles non listées restent à la fin (compat. ascendante : nouvelles tuiles
 * ajoutées plus tard apparaissent automatiquement à la fin).
 */
function applyDashboardOrder(grid, order) {
    const tiles = Array.from(grid.querySelectorAll('.nav-tile'));
    const map   = new Map(tiles.map(t => [t.dataset.tileId, t]));
    grid.classList.add('no-enter-anim'); // pas d'animation d'entrée en re-render
    order.forEach(id => {
        const el = map.get(id);
        if (el) { grid.appendChild(el); map.delete(id); }
    });
    // Tuiles inconnues / nouvelles : append à la fin (même ordre que le DOM initial)
    map.forEach(el => grid.appendChild(el));
    // Réactiver l'anim au prochain reflow naturel (pas nécessaire ici, mais prévoit)
    requestAnimationFrame(() => grid.classList.remove('no-enter-anim'));
}

/**
 * Envoie le nouvel ordre vers l'ESP32 (POST JSON).
 */
function saveDashboardOrder(order) {
    try { localStorage.setItem('dashOrder', JSON.stringify(order)); } catch (_) {}
    return fetch('/api/dashboard-order', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ order })
    }).then(r => {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json().catch(() => ({}));
    });
}

// ---------------------------------------------------------------------
//  Drag & drop multi-plateforme via Pointer Events
// ---------------------------------------------------------------------
let _dashDrag = null;  // état global du drag courant

function attachTileDragHandlers(grid) {
    grid.querySelectorAll('.nav-tile').forEach(tile => {
        tile.addEventListener('pointerdown', _onTilePointerDown);
        // En mode édition, on bloque la navigation au clic sur la tuile.
        tile.addEventListener('click', _editingClickGuard);
        // iPad : bloquer le menu contextuel (long-press) qui interrompt le drag.
        tile.addEventListener('contextmenu', _editingClickGuard);
        // Filet de sécurité supp : si touch-action est ignoré, on stoppe le scroll natif.
        tile.addEventListener('touchstart', _editingTouchGuard, { passive: false });
        tile.addEventListener('touchmove',  _editingTouchGuard, { passive: false });
    });
}

function detachTileDragHandlers(grid) {
    grid.querySelectorAll('.nav-tile').forEach(tile => {
        tile.removeEventListener('pointerdown', _onTilePointerDown);
        tile.removeEventListener('click', _editingClickGuard);
        tile.removeEventListener('contextmenu', _editingClickGuard);
        tile.removeEventListener('touchstart', _editingTouchGuard);
        tile.removeEventListener('touchmove',  _editingTouchGuard);
        tile.classList.remove('drop-before', 'drop-after', 'dragging-ghost');
    });
}

function _editingClickGuard(e) {
    // Pendant l'édition, on ne navigue pas, mais on laisse le clic se faire si
    // l'utilisateur n'a pas dragé (évite les redirections accidentelles).
    e.preventDefault();
}

function _editingTouchGuard(e) {
    // iPad/iOS : on doit interrompre la propagation du touch natif pour que le
    // navigateur ne tente pas de scroller la page pendant le drag.
    const grid = e.currentTarget && e.currentTarget.parentElement;
    if (grid && grid.classList.contains('editing')) {
        e.preventDefault();
    }
}

function _onTilePointerDown(e) {
    if (e.button !== undefined && e.button !== 0) return;  // bouton gauche / touch
    const tile = e.currentTarget;
    const grid = tile.parentElement;
    if (!grid || !grid.classList.contains('editing')) return;

    e.preventDefault();
    e.stopPropagation();
    try { tile.setPointerCapture && tile.setPointerCapture(e.pointerId); } catch (_) {}

    const rect = tile.getBoundingClientRect();
    const floater = tile.cloneNode(true);
    floater.classList.add('nav-tile-floater');
    floater.classList.remove('drop-before', 'drop-after');
    floater.style.width  = rect.width  + 'px';
    floater.style.height = rect.height + 'px';
    floater.style.left   = rect.left + 'px';
    floater.style.top    = rect.top  + 'px';
    document.body.appendChild(floater);

    tile.classList.add('dragging-ghost');

    _dashDrag = {
        grid,
        sourceTile: tile,
        floater,
        offsetX: e.clientX - rect.left,
        offsetY: e.clientY - rect.top,
        pointerId: e.pointerId,
        moved: false
    };

    // IMPORTANT : non-passif obligatoire pour que preventDefault() bloque le
    // scroll natif sur iPad / iOS Safari.
    document.addEventListener('pointermove',   _onDashPointerMove, { passive: false });
    document.addEventListener('pointerup',     _onDashPointerUp,   { once: true });
    document.addEventListener('pointercancel', _onDashPointerUp,   { once: true });
    // Fallback tactile pur (Safari iOS plus anciens / WKWebView)
    document.addEventListener('touchmove',     _onDashTouchMove,   { passive: false });
}

function _onDashPointerMove(e) {
    if (!_dashDrag) return;
    // Bloque le scroll/zoom natif d'iOS pendant le drag.
    if (e.cancelable) e.preventDefault();
    const d = _dashDrag;
    d.moved = true;
    d.floater.style.left = (e.clientX - d.offsetX) + 'px';
    d.floater.style.top  = (e.clientY - d.offsetY) + 'px';

    // ----- Stratégie robuste : on cherche la tuile dont le centre est
    // le plus proche du pointeur (ignore les gaps de grille, fonctionne en
    // multi-lignes, plus de "manqué" si on est entre deux tuiles).
    const tiles = d.grid.querySelectorAll('.nav-tile');
    let closest = null;
    let closestDist = Infinity;
    let closestRect = null;
    for (let i = 0; i < tiles.length; i++) {
        const t = tiles[i];
        if (t === d.sourceTile) continue;
        const r = t.getBoundingClientRect();
        const cx = r.left + r.width  / 2;
        const cy = r.top  + r.height / 2;
        const dx = e.clientX - cx;
        const dy = e.clientY - cy;
        const dist = dx * dx + dy * dy;
        if (dist < closestDist) {
            closestDist = dist;
            closest = t;
            closestRect = r;
        }
    }

    // Nettoyer indicateurs visuels (hover « fantôme »)
    for (let i = 0; i < tiles.length; i++) {
        tiles[i].classList.remove('drop-before', 'drop-after');
    }

    if (!closest || !closestRect) { d._target = null; return; }

    // Décider avant/après en combinant Y (ligne) puis X (colonne).
    // Si le pointeur est nettement au-dessus du centre Y -> insert avant.
    // Si nettement en dessous -> insert après.
    // Si quasi même ligne -> on regarde X.
    let before;
    const cy = closestRect.top + closestRect.height / 2;
    if (e.clientY < closestRect.top) {
        before = true;
    } else if (e.clientY > closestRect.bottom) {
        before = false;
    } else {
        before = (e.clientX - closestRect.left) < closestRect.width / 2;
    }

    closest.classList.add(before ? 'drop-before' : 'drop-after');

    // ----- LIVE REORDER : on réarrange le DOM immédiatement.
    // Cela donne un retour visuel instantané (la tuile prend sa place pendant
    // le drag), éliminant le besoin de « s'y reprendre plusieurs fois ».
    const desiredNext = before ? closest : closest.nextSibling;
    // Éviter les insertions inutiles (ne déplace que si position différente)
    if (d.sourceTile.nextSibling !== desiredNext && d.sourceTile !== desiredNext) {
        d.grid.insertBefore(d.sourceTile, desiredNext);
    }

    d._target = closest;
    d._before = before;
}

// Fallback Touch Events (sécurité pour iOS) : convertit en coordonnées client
function _onDashTouchMove(e) {
    if (!_dashDrag) return;
    if (e.cancelable) e.preventDefault();
    if (e.touches && e.touches.length > 0) {
        const t = e.touches[0];
        // Synthétise un événement compatible avec _onDashPointerMove
        _onDashPointerMove({
            clientX: t.clientX,
            clientY: t.clientY,
            cancelable: false,
            preventDefault: () => {}
        });
    }
}

function _onDashPointerUp() {
    document.removeEventListener('pointermove', _onDashPointerMove, { passive: false });
    document.removeEventListener('touchmove',   _onDashTouchMove,   { passive: false });
    if (!_dashDrag) return;
    const d = _dashDrag;

    // Plus besoin d'insérer ici : l'insertion est faite en live pendant le drag.
    // On nettoie simplement les indicateurs visuels et le floater.
    d.grid.querySelectorAll('.nav-tile').forEach(t => {
        t.classList.remove('drop-before', 'drop-after');
    });
    d.sourceTile.classList.remove('dragging-ghost');
    if (d.floater && d.floater.parentNode) d.floater.parentNode.removeChild(d.floater);
    try { d.sourceTile.releasePointerCapture && d.sourceTile.releasePointerCapture(d.pointerId); } catch (_) {}

    _dashDrag = null;
}

/**
 * ============================================
 * MODULE DE CONTRÔLE CNC - WEBSOCKET & FLUIDNC
 * ============================================
 */

// === Variables globales CNC ===
let cncSocket = null;
let cncConnected = false;
let currentStep = 1.0; // Pas de jog par défaut
let jogFeedRate = 1000; // Vitesse de jog en mm/min
let machineState = 'OFFLINE';
window.machineState = machineState;  // expose des l'init pour le mini-dashboard
let workPosition = { X: 0, Y: 0, Z: 0, A: 0 };
let machinePosition = { X: 0, Y: 0, Z: 0, A: 0 };
// [SOURCE UNIQUE DE VÉRITÉ] Exposition globale pour les pages secondaires
// (Palpage_Lamage.html, decoupe.html, etc.) qui souhaitent lire la position
// temps réel sans dupliquer le parser. Lecture seule côté pages.
window.workPosition = workPosition;
window.machinePosition = machinePosition;

// === Registre de handlers additionnels WebSocket ===
// Permet aux pages d'enregistrer des callbacks appelés à chaque message reçu.
window._cncSocketMessageHandlers = [];
window._cncSocketStateHandlers = [];  // callbacks(connected:bool) appelés sur open/close
window.addCncSocketMessageHandler = function(fn) {
    if (typeof fn === 'function') window._cncSocketMessageHandlers.push(fn);
};
window.addCncSocketStateHandler = function(fn) {
    if (typeof fn === 'function') window._cncSocketStateHandlers.push(fn);
};

// === Commandes temps-réel FluidNC (ne passent PAS par la queue) ===
// Ces caractères/commandes sont traitées immédiatement par FluidNC,
// sans attendre un 'ok' du buffer de planification.
const RT_COMMANDS = new Set([
    '?',                        // Rapport de statut
    '~',                        // Cycle Start / Resume
    '!',                        // Feed Hold
    String.fromCharCode(0x18),  // Ctrl-X : Soft Reset
    String.fromCharCode(0x85),  // Jog Cancel
    String.fromCharCode(0x84),  // Safety Door
    String.fromCharCode(0x7E),  // Cycle Start (alias ~)
]);

// === File d'attente "Wait for OK" ===
// FluidNC répond 'ok' ou 'error:N' pour chaque commande non temps-réel.
// On ne doit pas saturer le buffer de planification (16 slots max).
const CMD_QUEUE = [];           // file des commandes en attente
let   _queueBusy = false;       // vrai si on attend un 'ok' ou 'error'
const QUEUE_MAX_IN_FLIGHT = 4;  // commandes envoyées sans 'ok' reçu (< 16 slots FluidNC)
let   _inFlight = 0;            // commandes envoyées sans réponse

// === Heartbeat ===
let _heartbeatTimer = null;
let _lastSentAt = 0;            // timestamp du dernier envoi (ms)
const HEARTBEAT_INTERVAL = 800; // ms — envoie '?' si aucune commande depuis ce délai

// === Reconnexion ===
let _reconnectTimer = null;
let _reconnectAttempts = 0;
const RECONNECT_DELAY_MS = 2000;
const RECONNECT_MAX_DELAY = 15000;

// === Jog hold-to-move ===
let _jogActive = false;         // vrai pendant qu'un bouton jog est maintenu enfoncé
let _jogBtn = null;             // bouton jog actif
let _jogMode = 'step';          // 'step' (par pas, distance exacte) | 'continu' (hold-to-move)
let _jogStepInFlight = false;   // anti-rebond mode 'step' : verrou logiciel pendant qu'un coup est en cours
let _jogLastStepAt   = 0;       // timestamp (ms) du dernier coup envoyé — debounce matériel
const JOG_STEP_DEBOUNCE_MS = 120; // fenêtre minimale entre deux coups (anti-clic fantôme)
let _jogResyncTimers = [];      // timers actifs de resync DRO post Jog Cancel

// === Palpeur 3D (Probe Macro System) ===
let _lastProbeResult = null;    // dernier resultat [PRB:X,Y,Z:S]
let _probeResolve = null;       // Promise resolver pour G38.2
let _okResolve = null;          // Promise resolver pour mouvement G1
let _probeMacroRunning = false; // vrai pendant l'execution d'une macro
let _probeMacroAbort = false;   // flag d'abort
let probeConfig = { ball_diameter: 0, offset_x: 0, offset_y: 0, offset_z: 0 };

// === Initialisation du module CNC ===
document.addEventListener('DOMContentLoaded', function() {
    // Connexion WebSocket sur TOUTES les pages (pas seulement controle.html)
    connectWebSocket();

    // Initialiser les contrôles CNC (jog, step, etc.) uniquement sur controle.html
    if (document.querySelector('.cnc-container')) {
        initCNCControl();
    }
});

/**
 * Initialise le centre de contrôle CNC
 */
function initCNCControl() {
    console.log('🔧 Initialisation du contrôle CNC...');
    
    // Initialiser les contrôles (WebSocket déjà connecté par le DOMContentLoaded global)
    initJogControls();
    initStepSelector();
    initJogModeToggle();
    initSpeedSlider();
    initLightControl();
    initVacuumControl();
    initCoolantControl();
    initSafetyButtons();
    initHomingButtons();
    initTerminal();
    
    // Animer l'apparition des cartes
    animateCardsEntry();
    
    // Initialiser les accordéons avec persistance
    initAccordions();
    
    // Initialiser les contrôles d'offsets
    initOffsetControls();
}

/**
 * Connexion WebSocket au serveur
 */
function connectWebSocket() {
    // Eviter les connexions multiples simultanées
    if (cncSocket && (cncSocket.readyState === WebSocket.CONNECTING ||
                      cncSocket.readyState === WebSocket.OPEN)) return;

    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/ws`;

    if (_reconnectAttempts > 0) {
        addTerminalLine('Système', `Reconnexion #${_reconnectAttempts}...`, 'info');
    }

    cncSocket = new WebSocket(wsUrl);

    cncSocket.onopen = function() {
        console.log('WebSocket connecté');
        cncConnected = true;
        _reconnectAttempts = 0;
        _inFlight = 0;
        CMD_QUEUE.length = 0;
        _queueBusy = false;
        updateConnectionStatus(true);
        addTerminalLine('Système', 'Connexion WebSocket établie', 'success');
        // Dispatch aux handlers d'état enregistrés par les pages
        var sh = window._cncSocketStateHandlers;
        if (sh) for (var i = 0; i < sh.length; i++) { try { sh[i](true); } catch(e) {} }

        // Synchronisation initiale des coordonnées
        setTimeout(() => {
            sendRealtime('?');
            addTerminalLine('Système', 'Synchronisation des coordonnées...', 'info');
        }, 500);

        // Démarrer le heartbeat
        _startHeartbeat();
    };

    cncSocket.onclose = function(event) {
        console.log('WebSocket déconnecté', event.code);
        _onDisconnect();
        // Dispatch aux handlers d'état enregistrés par les pages
        var sh = window._cncSocketStateHandlers;
        if (sh) for (var i = 0; i < sh.length; i++) { try { sh[i](false); } catch(e) {} }
        // Reconnexion avec backoff exponentiel plafonné
        const delay = Math.min(RECONNECT_DELAY_MS * Math.pow(1.5, _reconnectAttempts), RECONNECT_MAX_DELAY);
        _reconnectAttempts++;
        _reconnectTimer = setTimeout(connectWebSocket, delay);
    };

    cncSocket.onerror = function(error) {
        console.error('Erreur WebSocket:', error);
        // onclose sera appelé juste après — pas besoin de reconnect ici
    };

    cncSocket.onmessage = function(event) {
        handleWebSocketMessage(event.data);
        // Dispatch aux handlers enregistrés par les pages (taquin, decoupe, etc.)
        var handlers = window._cncSocketMessageHandlers;
        if (handlers && handlers.length > 0) {
            for (var i = 0; i < handlers.length; i++) {
                try { handlers[i](event.data); } catch(e) {}
            }
        }
    };
}

function _onDisconnect() {
    cncConnected = false;
    _inFlight = 0;
    CMD_QUEUE.length = 0;
    _queueBusy = false;
    updateConnectionStatus(false);
    _stopHeartbeat();
    // Annuler tout jog actif proprement côté UI
    if (_jogActive) { _jogActive = false; if (_jogBtn) { _jogBtn.classList.remove('jog-held'); _jogBtn = null; } }
    addTerminalLine('Système', 'Connexion WebSocket perdue', 'error');
}

// ============================================================
//  HEARTBEAT — keep-alive '?' toutes les HEARTBEAT_INTERVAL ms
//  Envoyé uniquement si aucune autre commande récente.
// ============================================================
function _startHeartbeat() {
    _stopHeartbeat();
    _heartbeatTimer = setInterval(() => {
        if (!cncConnected) return;
        if (Date.now() - _lastSentAt >= HEARTBEAT_INTERVAL) {
            sendRealtime('?');
        }
    }, HEARTBEAT_INTERVAL);
}

function _stopHeartbeat() {
    if (_heartbeatTimer) { clearInterval(_heartbeatTimer); _heartbeatTimer = null; }
}

// ============================================================
//  VISIBILITÉ DE L'ONGLET — Reconnexion immédiate au retour
//  Les navigateurs throttlent les timers des tabs en arrière-plan,
//  ce qui empêche le reconnect automatique de fonctionner.
//  Au retour sur la page, on force une reconnexion si nécessaire.
// ============================================================
document.addEventListener('visibilitychange', function() {
    if (document.hidden) return;
    // L'onglet redevient visible : vérifier l'état du WebSocket
    if (!cncSocket || cncSocket.readyState === WebSocket.CLOSED ||
        cncSocket.readyState === WebSocket.CLOSING) {
        console.log('[WS] Onglet actif — reconnexion forcée');
        // Annuler tout timer de reconnexion en cours (potentiellement throttlé)
        if (_reconnectTimer) { clearTimeout(_reconnectTimer); _reconnectTimer = null; }
        _reconnectAttempts = 0;
        connectWebSocket();
    }
});

/**
 * Traite les messages WebSocket entrants
 */
function handleWebSocketMessage(data) {
    try {
        const message = JSON.parse(data);

        if (message.type === 'fluidnc') {
            const fluidData = message.data;
            parseMachineStatus(fluidData);
            _handleFluidNCResponse(fluidData);  // DOIT recevoir les 'ok' : gère _inFlight et CMD_QUEUE
            const isStatusReport = fluidData.startsWith('<') && (fluidData.includes('WPos:') || fluidData.includes('MPos:'));
            const isOkAck = fluidData.trim() === 'ok' || /^ok x\d+$/.test(fluidData.trim()); // Acquittements : inutiles dans le terminal
            if (!isStatusReport && !isOkAck) {
                addTerminalLine('FluidNC', fluidData, 'received');
            }
        } else if (message.type === 'light') {
            handleLightFeedback(message);
        } else if (message.type === 'vacuum') {
            handleVacuumFeedback(message);
        }
    } catch (e) {
        // Message non-JSON brut (certaines configurations ESP32 renvoient du texte plat)
        _handleFluidNCResponse(data);
        if (data.startsWith('Offset/')) {
            handleOffsetData(data);
        } else {
            addTerminalLine('Reçu', data, 'received');
        }
    }
}

/**
 * Traite la réponse FluidNC pour drainer la file d'attente.
 * FluidNC répond 'ok' ou 'error:N' pour chaque commande non temps-réel.
 * Détecte aussi les réponses [PRB:X,Y,Z:S] du palpeur.
 */
function _handleFluidNCResponse(msg) {
    if (typeof msg !== 'string') return;
    const trimmed = msg.trim();

    // --- Détection PRB (réponse palpeur G38.x) ---
    const prbMatch = trimmed.match(/\[PRB:([-\d.]+),([-\d.]+),([-\d.]+):(\d+)\]/);
    if (prbMatch) {
        _lastProbeResult = {
            x: parseFloat(prbMatch[1]),
            y: parseFloat(prbMatch[2]),
            z: parseFloat(prbMatch[3]),
            success: parseInt(prbMatch[4]) === 1
        };
        console.log('[PROBE] PRB recu:', _lastProbeResult);
        if (_probeResolve) {
            const resolve = _probeResolve;
            _probeResolve = null;
            resolve(_lastProbeResult);
        }
        return;
    }

    // --- Détection Alarm ---
    if (trimmed.startsWith('ALARM:') && _probeMacroRunning) {
        _probeMacroAbort = true;
        if (_probeResolve) _probeResolve = null;
        if (_okResolve) _okResolve = null;
    }

    // --- ok / error ---
    if (trimmed === 'ok' || trimmed.startsWith('error:')) {
        if (_inFlight > 0) _inFlight--;
        _drainQueue();
        if (_okResolve) {
            const resolve = _okResolve;
            _okResolve = null;
            resolve(trimmed);
        }
        if (trimmed.startsWith('error:') && _probeMacroRunning) {
            _probeMacroAbort = true;
            if (_probeResolve) _probeResolve = null;
        }
        return;
    }
    // --- ok x N (coalescence C++) ---
    // L'ESP32 regroupe les 'ok' consécutifs en "ok xN" pour éviter de saturer
    // la queue AsyncWebSocket (cap. 8 slots). On décrémente _inFlight de N.
    const okBatch = trimmed.match(/^ok x(\d+)$/);
    if (okBatch) {
        const n = parseInt(okBatch[1], 10);
        _inFlight = Math.max(0, _inFlight - n);
        _drainQueue();
        // Résolution d'une promesse en attente (une seule, le premier ok du batch)
        if (_okResolve) { const r = _okResolve; _okResolve = null; r('ok'); }
    }
}

// ============================================================================
// === ProbeMacroExecutor — Systeme de macros palpage base sur Promises ===
// ============================================================================

/**
 * Envoie une commande G38.2 (palpage) et attend la reponse PRB.
 * Resout avec l'objet {x, y, z, success} ou rejette sur timeout/abort.
 * @param {string} cmd - Commande G38.2 (ex: "G91 G38.2 X-50 F200")
 * @param {number} [timeoutMs=30000] - Timeout en ms
 * @returns {Promise<{x:number, y:number, z:number, success:boolean}>}
 */
function sendProbeCommand(cmd, timeoutMs) {
    timeoutMs = timeoutMs || 30000;
    return new Promise(function(resolve, reject) {
        if (_probeMacroAbort) { reject('Macro aborted'); return; }
        _probeResolve = resolve;
        sendCommand(cmd);
        // Timeout de securite
        setTimeout(function() {
            if (_probeResolve === resolve) {
                _probeResolve = null;
                reject('Timeout probe (' + timeoutMs + 'ms)');
            }
        }, timeoutMs);
    });
}

/**
 * Envoie une commande de mouvement (G1/G0) et attend 'ok'.
 * Resout avec la reponse ('ok' ou 'error:...') ou rejette sur abort.
 * @param {string} cmd - Commande G-code de mouvement
 * @param {number} [timeoutMs=10000] - Timeout en ms
 * @returns {Promise<string>}
 */
function sendMoveCommand(cmd, timeoutMs) {
    timeoutMs = timeoutMs || 10000;
    return new Promise(function(resolve, reject) {
        if (_probeMacroAbort) { reject('Macro aborted'); return; }
        _okResolve = resolve;
        sendCommand(cmd);
        setTimeout(function() {
            if (_okResolve === resolve) {
                _okResolve = null;
                reject('Timeout move (' + timeoutMs + 'ms)');
            }
        }, timeoutMs);
    });
}

/**
 * Execute une macro palpage (tableau de fonctions async).
 * Chaque etape est une fonction async qui sera await-ee.
 * Gere l'affichage de progression si updateMacroProgress() existe.
 * @param {Array<Function>} steps - Tableau de fonctions async
 */
async function runProbeMacro(steps) {
    if (_probeMacroRunning) return;
    _probeMacroRunning = true;
    _probeMacroAbort = false;
    try {
        for (var i = 0; i < steps.length; i++) {
            if (_probeMacroAbort) throw 'Macro aborted';
            // Mise a jour progression (fonction optionnelle de la page)
            if (typeof updateMacroProgress === 'function') {
                updateMacroProgress(i + 1, steps.length);
            }
            await steps[i]();
        }
    } catch (e) {
        console.error('[PROBE] Macro erreur:', e);
        if (typeof notify === 'function') {
            notify('Macro palpage : ' + e, 'error');
        }
    }
    _probeMacroRunning = false;
    if (typeof updateMacroProgress === 'function') {
        updateMacroProgress(0, 0); // reset
    }
}

/**
 * Arrete une macro palpage en cours.
 * Envoie Jog Cancel (0x85) puis Soft Reset (0x18) a FluidNC.
 */
function abortProbeMacro() {
    _probeMacroAbort = true;
    // Jog Cancel + Soft Reset
    if (cncSocket && cncSocket.readyState === WebSocket.OPEN) {
        cncSocket.send('\x85');  // Jog Cancel
        setTimeout(function() {
            if (cncSocket && cncSocket.readyState === WebSocket.OPEN) {
                cncSocket.send('\x18');  // Soft Reset (Ctrl-X)
            }
        }, 200);
    }
    // Rejeter les Promises en attente
    if (_probeResolve) { _probeResolve = null; }
    if (_okResolve) { _okResolve = null; }
    console.log('[PROBE] Macro aborted');
}

/**
 * Charge la configuration du palpeur depuis l'API REST.
 * Met a jour la variable globale probeConfig.
 * @returns {Promise<boolean>} true si charge avec succes
 */
async function fetchProbeConfig() {
    try {
        var r = await fetch('/api/probe');
        if (!r.ok) throw new Error('HTTP ' + r.status);
        probeConfig = await r.json();
        console.log('[PROBE] Config chargee:', probeConfig);
        return true;
    } catch (e) {
        console.error('[PROBE] Erreur chargement config:', e);
        return false;
    }
}

// ============================================================================
// === Fin ProbeMacroExecutor ===
// ============================================================================

// --- Exposition globale sur window pour les pages externes (Palpage_Lamage.html, etc.) ---
// Les `let` top-level d'un script classique ne sont pas attachées à window,
// donc on les expose explicitement pour qu'elles soient utilisables depuis
// les <script> inline d'autres pages.
window.probeConfig        = probeConfig;
window.fetchProbeConfig   = fetchProbeConfig;
window.sendProbeCommand   = sendProbeCommand;
window.sendMoveCommand    = sendMoveCommand;
window.runProbeMacro      = runProbeMacro;
window.abortProbeMacro    = abortProbeMacro;
// Wrapper pour que les pages externes accèdent toujours à la valeur à jour
// de probeConfig (la valeur est réaffectée dans fetchProbeConfig).
Object.defineProperty(window, 'probeConfig', {
    get: function() { return probeConfig; },
    set: function(v) { probeConfig = v; },
    configurable: true
});

/**
 * Gère le feedback de l'état de l'éclairage depuis l'ESP32
 */
function handleLightFeedback(message) {
    const btnLight = document.getElementById('btnLight');
    const lightLabel = document.getElementById('lightLabel');
    
    if (!btnLight) return;
    
    const state = message.state === 'on';
    
    // Mettre à jour l'interface
    if (state) {
        btnLight.classList.add('on');
        btnLight.setAttribute('data-state', 'on');
        if (lightLabel) lightLabel.textContent = 'Allumé';
    } else {
        btnLight.classList.remove('on');
        btnLight.setAttribute('data-state', 'off');
        if (lightLabel) lightLabel.textContent = 'Éteint';
    }
}

/**
 * Gère le feedback de l'état de l'aspiration depuis l'ESP32
 */
function handleVacuumFeedback(message) {
    const btnVacuum = document.getElementById('btnVacuum');
    const vacuumLabel = document.getElementById('vacuumLabel');
    
    if (!btnVacuum) return;
    
    const state = message.state === 'on';
    
    // Mettre à jour l'interface
    if (state) {
        btnVacuum.classList.add('on');
        btnVacuum.setAttribute('data-state', 'on');
        if (vacuumLabel) vacuumLabel.textContent = 'Marche';
    } else {
        btnVacuum.classList.remove('on');
        btnVacuum.setAttribute('data-state', 'off');
        if (vacuumLabel) vacuumLabel.textContent = 'Arrêt';
    }
}

/**
 * Parse les messages de statut FluidNC
 * Formats supportés:
 * - <Idle|WPos:0.000,0.000,0.000|FS:0,0>
 * - <Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>
 * - <Idle|WPos:0.000,0.000,0.000,0.000|FS:0,0> (avec axe A)
 */
function parseMachineStatus(data) {
    // Vérifier si c'est un message de statut (commence par <)
    if (!data.startsWith('<')) return;
    
    // Détection de l'état de la machine
    const stateMatch = data.match(/<(\w+)/);
    if (stateMatch) {
        const newState = stateMatch[1];
        if (newState !== machineState) {
            machineState = newState;
            window.machineState = machineState;  // expose pour le mini-dashboard
            updateMachineState(machineState);
        }
    }
    
    // Extraction des positions (WPos = Work Position)
    // Format: WPos:X,Y,Z ou WPos:X,Y,Z,A
    const wposMatch = data.match(/WPos:([-\d.]+),([-\d.]+),([-\d.]+)(?:,([-\d.]+))?/);
    if (wposMatch) {
        workPosition.X = parseFloat(wposMatch[1]);
        workPosition.Y = parseFloat(wposMatch[2]);
        workPosition.Z = parseFloat(wposMatch[3]);
        workPosition.A = wposMatch[4] ? parseFloat(wposMatch[4]) : 0;
        updateDRO();
    }
    
    // Extraction des positions machine (MPos)
    // IMPORTANT : on met TOUJOURS à jour machinePosition quand MPos est présent,
    // même si WPos l'est aussi. machinePosition sert d'offset zéro-machine pour
    // les visualiseurs 3D (Palpage_Lamage, etc.). Le fallback DRO ne s'applique
    // que si WPos est absent.
    const mposMatch = data.match(/MPos:([-\d.]+),([-\d.]+),([-\d.]+)(?:,([-\d.]+))?/);
    if (mposMatch) {
        machinePosition.X = parseFloat(mposMatch[1]);
        machinePosition.Y = parseFloat(mposMatch[2]);
        machinePosition.Z = parseFloat(mposMatch[3]);
        machinePosition.A = mposMatch[4] ? parseFloat(mposMatch[4]) : 0;

        if (!wposMatch) {
            // Pas de WPos : on utilise MPos pour alimenter le DRO
            workPosition.X = machinePosition.X;
            workPosition.Y = machinePosition.Y;
            workPosition.Z = machinePosition.Z;
            workPosition.A = machinePosition.A;
            updateDRO();
        }
    }

    // Champ A: du status report FluidNC (accessoires coolant)
    // Non traité ici : les boutons coolant sont gérés localement
    // par initCoolantControl() comme l'éclairage et l'aspiration.
}

/**
 * Met à jour l'affichage des coordonnées DRO
 */
function updateDRO() {
    const droX = document.getElementById('droX');
    const droY = document.getElementById('droY');
    const droZ = document.getElementById('droZ');
    const droA = document.getElementById('droA');
    
    // Vérifier que les valeurs sont des nombres valides
    const x = isNaN(workPosition.X) ? 0 : workPosition.X;
    const y = isNaN(workPosition.Y) ? 0 : workPosition.Y;
    const z = isNaN(workPosition.Z) ? 0 : workPosition.Z;
    const a = isNaN(workPosition.A) ? 0 : workPosition.A;
    
    if (droX) droX.textContent = x.toFixed(3);
    if (droY) droY.textContent = y.toFixed(3);
    if (droZ) droZ.textContent = z.toFixed(3);
    if (droA) droA.textContent = a.toFixed(3);

    // [SOURCE UNIQUE DE VÉRITÉ] Aucun stockage local du DRO :
    // la position est demandée à FluidNC via '?' à chaque connexion WS
    // (voir cncSocket.onopen) et rafraîchie par le heartbeat.
}

/**
 * Met à jour l'indicateur de connexion
 */
function updateConnectionStatus(connected) {
    const statusLed = document.getElementById('statusLed');
    const statusText = document.getElementById('statusText');
    
    if (statusLed && statusText) {
        if (connected) {
            statusLed.classList.add('connected');
            statusText.textContent = 'Connecté';
        } else {
            statusLed.classList.remove('connected');
            statusText.textContent = 'Déconnecté';
        }
    }
}

/**
 * Met à jour l'affichage de l'état de la machine
 */
function updateMachineState(state) {
    const stateBadge = document.getElementById('stateBadge');
    if (stateBadge) {
        stateBadge.textContent = state;
        stateBadge.className = 'state-badge';
        
        // Ajouter la classe correspondante à l'état
        const stateLower = state.toLowerCase();
        if (stateLower === 'idle') stateBadge.classList.add('state-idle');
        else if (stateLower === 'run') stateBadge.classList.add('state-run');
        else if (stateLower === 'hold') stateBadge.classList.add('state-hold');
        else if (stateLower === 'alarm') stateBadge.classList.add('state-alarm');
        else if (stateLower === 'home') stateBadge.classList.add('state-home');
        else stateBadge.classList.add('state-offline');
    }
    
    // Gérer l'effet d'alarme sur toute la page
    const body = document.body;
    if (state.toLowerCase() === 'alarm') {
        body.classList.add('alarm-active');
    } else {
        body.classList.remove('alarm-active');
    }
}

/**
 * sendRealtime() — Envoi immédiat, bypass total de la queue.
 * Réservé aux commandes temps-réel : '?', '~', '!', \x85, \x18 ...
 */
function sendRealtime(command) {
    if (!cncConnected || !cncSocket) return false;
    try {
        cncSocket.send(command);
        _lastSentAt = Date.now();
        return true;
    } catch (e) {
        addTerminalLine('Erreur', `Echec envoi RT: ${e.message}`, 'error');
        return false;
    }
}
// Exposer pour le mini-dashboard (feedrate override, etc.)
window.sendCncRealtime = sendRealtime;

/**
 * sendCommand() — Point d'entrée pour toutes les commandes G-code et $-commands.
 * • Commandes temps-réel (RT_COMMANDS) → envoi immédiat, pas de queue.
 * • Toutes les autres → file d'attente avec limite QUEUE_MAX_IN_FLIGHT.
 */
function sendCommand(command) {
    if (!cncConnected || !cncSocket) {
        addTerminalLine('Erreur', 'Non connecté au serveur', 'error');
        return false;
    }
    // Commande temps-réel : bypass queue immédiat
    if (RT_COMMANDS.has(command)) {
        return sendRealtime(command);
    }
    // Mise en file d'attente
    CMD_QUEUE.push(command);
    _drainQueue();
    return true;
}

/**
 * _drainQueue() — Déverse la file dans la limite du QUEUE_MAX_IN_FLIGHT.
 * Appelé après sendCommand() et après chaque 'ok'/'error' reçu de FluidNC.
 */
function _drainQueue() {
    while (CMD_QUEUE.length > 0 && _inFlight < QUEUE_MAX_IN_FLIGHT && cncConnected) {
        const cmd = CMD_QUEUE.shift();
        try {
            cncSocket.send(cmd);
            _lastSentAt = Date.now();
            _inFlight++;
            addTerminalLine('Envoyé', cmd, 'sent');
        } catch (e) {
            addTerminalLine('Erreur', `Echec envoi: ${e.message}`, 'error');
            break;
        }
    }
}

/**
 * initJogControls() — Pointer Events unifiés (souris + tactile, pas de doublon).
 *
 * Mode 'continu' :
 *   pointerdown  → envoie $J=G91 X9999 F... (la machine avance jusqu'au relâchement)
 *   pointerup    → envoie \x85 (Jog Cancel) puis force 3 demandes de statut '?'
 *                  pour resynchroniser le DRO sur la position réelle finale
 *
 * Mode 'step' :
 *   pointerdown  → envoie $J=G91 X<currentStep> F... UNE seule fois
 *                  N'envoie JAMAIS \x85 au relâchement → le mouvement va à son terme exact.
 *                  Verrou _jogStepInFlight évite tout double-tap parasite.
 *
 * Pour les commandes G-code (go-to-home…) le comportement reste un simple clic.
 */
function initJogControls() {
    const jogButtons = document.querySelectorAll('.jog-btn[data-cmd]');

    function _jogStart(btn, ev) {
        const cmd = btn.getAttribute('data-cmd');
        if (cmd.startsWith('G')) { sendCommand(cmd); return; } // go-to : simple clic

        // ----- MODE PAR PAS : un coup, distance exacte, jamais de Cancel -----
        if (_jogMode === 'step') {
            // Debounce horodaté — ignore tout coup arrivant dans la fenêtre minimale.
            // Couvre les "clics fantômes" iOS (pointerdown + click synthétique).
            const now = Date.now();
            if (_jogStepInFlight) return;
            if (now - _jogLastStepAt < JOG_STEP_DEBOUNCE_MS) return;
            _jogLastStepAt = now;
            _jogStepInFlight = true;
            btn.classList.add('jog-held');
            _sendJogStep(cmd);
            // Libère le verrou après un délai estimé sur la durée du déplacement
            // (distance / vitesse) avec un plancher de 200 ms et un plafond de 8 s.
            const estMs = Math.max(200, Math.min(8000, Math.round((currentStep / Math.max(1, jogFeedRate)) * 60000) + 250));
            setTimeout(() => {
                _jogStepInFlight = false;
                btn.classList.remove('jog-held');
                // Demande un statut frais pour figer le DRO sur la cible exacte
                sendRealtime('?');
            }, estMs);
            return;
        }

        // ----- MODE CONTINU : hold-to-move classique -----
        if (_jogActive) return;
        _jogActive = true;
        _jogBtn = btn;
        btn.classList.add('jog-held');
        _sendJogStart(cmd);
    }

    function _jogStop() {
        // En mode 'step' : ne JAMAIS envoyer \x85 (couperait le mouvement de précision).
        if (_jogMode === 'step') return;
        if (!_jogActive) return;
        _jogActive = false;
        if (_jogBtn) { _jogBtn.classList.remove('jog-held'); _jogBtn = null; }
        sendRealtime(String.fromCharCode(0x85)); // Jog Cancel — bypass queue
        // Anti-glissement DRO : force plusieurs '?' rapprochés pour resync
        // sur la position réelle dès la fin de décélération FluidNC.
        _resyncDROAfterCancel();
    }

    jogButtons.forEach(btn => {
        // Pointer Events couvrent souris + tactile (pas besoin de touchstart en plus).
        btn.addEventListener('pointerdown',   function(e) { e.preventDefault(); _jogStart(this, e); });
        btn.addEventListener('pointerup',     _jogStop);
        btn.addEventListener('pointerleave',  _jogStop);
        btn.addEventListener('pointercancel', _jogStop);
        // Bloque le 'click' synthétique iOS qui pouvait re-déclencher pointerdown
        btn.addEventListener('click', function(e) { e.preventDefault(); e.stopPropagation(); });
        // Bloque le menu contextuel sur appui long tactile
        btn.addEventListener('contextmenu', function(e) { e.preventDefault(); });
    });
}

/**
 * _resyncDROAfterCancel() — Force 3 demandes de statut '?' à 0/80/220 ms
 * après un Jog Cancel pour aligner le DRO sur la position finale dès que
 * la machine a fini sa décélération. Évite l'effet "chiffres qui glissent".
 */
function _resyncDROAfterCancel() {
    // Annule les timers d'une précédente séquence (en cas de double appui rapide)
    _jogResyncTimers.forEach(t => clearTimeout(t));
    _jogResyncTimers = [];
    sendRealtime('?');
    _jogResyncTimers.push(setTimeout(() => sendRealtime('?'), 80));
    _jogResyncTimers.push(setTimeout(() => sendRealtime('?'), 220));
    _jogResyncTimers.push(setTimeout(() => sendRealtime('?'), 500));
}

/**
 * initJogModeToggle() — Câble les boutons "Par Pas" / "Continu".
 * Le bouton "Pas" (sous-sélecteur 0.1/1/10/100) reste visible mais grimé en mode continu.
 */
function initJogModeToggle() {
    const modeButtons = document.querySelectorAll('#jogModeButtons .mode-btn');
    const stepDiv = document.getElementById('stepSelectorDiv');
    if (!modeButtons.length) return;

    function applyMode(mode) {
        _jogMode = (mode === 'continu') ? 'continu' : 'step';
        modeButtons.forEach(b => b.classList.toggle('active', b.getAttribute('data-mode') === _jogMode));
        if (stepDiv) {
            stepDiv.style.opacity   = (_jogMode === 'continu') ? '0.45' : '1';
            stepDiv.style.pointerEvents = (_jogMode === 'continu') ? 'none' : 'auto';
        }
        try { localStorage.setItem('jogMode', _jogMode); } catch(e) {}
        console.log(`🔄 Mode jog : ${_jogMode}`);
    }

    modeButtons.forEach(btn => {
        btn.addEventListener('click', function() { applyMode(this.getAttribute('data-mode')); });
    });

    // Restauration du dernier choix utilisateur (sinon valeur active du HTML)
    let saved = null;
    try { saved = localStorage.getItem('jogMode'); } catch(e) {}
    if (saved === 'step' || saved === 'continu') {
        applyMode(saved);
    } else {
        const activeBtn = document.querySelector('#jogModeButtons .mode-btn.active');
        applyMode(activeBtn ? activeBtn.getAttribute('data-mode') : 'step');
    }
}

/**
 * Initialise le slider de vitesse Jog
 */
function initSpeedSlider() {
    const speedSlider = document.getElementById('speedSlider');
    const speedValue = document.getElementById('speedValue');
    
    if (speedSlider && speedValue) {
        speedSlider.addEventListener('input', function() {
            jogFeedRate = parseInt(this.value);
            speedValue.textContent = jogFeedRate;
            
            // Animation de la valeur
            gsap.fromTo(speedValue, 
                { scale: 1.2, color: '#00d9ff' },
                { scale: 1, color: '#ffffff', duration: 0.3, ease: 'power2.out' }
            );
        });
        
        // Définir la valeur initiale
        jogFeedRate = parseInt(speedSlider.value);
        speedValue.textContent = jogFeedRate;
    }
}

/**
 * Initialise le contrôle d'éclairage broche (GPIO 8) et du message NeoLED (GPIO 11).
 * Pattern identique a initCoolantControl : etat local toggle, UI immediate, zero dependance
 * au status report.
 */
function initLightControl() {
    const btnSpindle  = document.getElementById('btnSpindleLight');
    const btnNeoLed   = document.getElementById('btnNeoLedMsg');
    const slider      = document.getElementById('neoledBrightness');
    const sliderVal   = document.getElementById('neoledBrightnessVal');
    const spindleLabel = document.getElementById('spindleLightLabel');
    const neoledLabel  = document.getElementById('neoledMsgLabel');

    let spindleState = false;
    let neoledState  = false;

    // --- Éclairage Broche (GPIO 8) ---
    if (btnSpindle) {
        btnSpindle.addEventListener('click', function() {
            spindleState = !spindleState;
            sendCommand(spindleState ? 'Message/SpindleLight=1' : 'Message/SpindleLight=0');
            addTerminalLine('Systeme', spindleState
                ? 'Eclairage Broche: ON (GPIO 8)'
                : 'Eclairage Broche: OFF (GPIO 8)', spindleState ? 'success' : 'info');
            updateSpindleUI(spindleState);
        });
    }

    function updateSpindleUI(state) {
        if (!btnSpindle) return;
        if (state) {
            btnSpindle.classList.add('on');
            btnSpindle.setAttribute('data-state', 'on');
            if (spindleLabel) spindleLabel.textContent = 'Allumé';
        } else {
            btnSpindle.classList.remove('on');
            btnSpindle.setAttribute('data-state', 'off');
            if (spindleLabel) spindleLabel.textContent = 'Éclairage';
        }
        gsap.fromTo(btnSpindle, { scale: 0.92 }, { scale: 1, duration: 0.25, ease: 'back.out(2)' });
    }

    // --- Message NeoLED (GPIO 11) ---
    if (btnNeoLed) {
        btnNeoLed.addEventListener('click', function() {
            neoledState = !neoledState;
            const brightness = slider ? slider.value : 80;
            sendCommand(neoledState
                ? 'Message/NeoLed=' + brightness
                : 'Message/NeoLed=0');
            addTerminalLine('Systeme', neoledState
                ? 'NeoLED message: ON (GPIO 11, luminosite ' + brightness + ')'
                : 'NeoLED message: OFF (GPIO 11)', neoledState ? 'success' : 'info');
            updateNeoLedUI(neoledState);
        });
    }

    function updateNeoLedUI(state) {
        if (!btnNeoLed) return;
        if (state) {
            btnNeoLed.classList.add('on');
            btnNeoLed.setAttribute('data-state', 'on');
            if (neoledLabel) neoledLabel.textContent = 'Actif';
        } else {
            btnNeoLed.classList.remove('on');
            btnNeoLed.setAttribute('data-state', 'off');
            if (neoledLabel) neoledLabel.textContent = 'NeoLED';
        }
        gsap.fromTo(btnNeoLed, { scale: 0.92 }, { scale: 1, duration: 0.25, ease: 'back.out(2)' });
    }

    // --- Slider luminosite NeoLED ---
    if (slider && sliderVal) {
        slider.addEventListener('input', function() {
            sliderVal.textContent = slider.value;
            // Si la NeoLED est active, mettre a jour la luminosite en temps reel
            if (neoledState) {
                sendCommand('Message/NeoLed=' + slider.value);
            }
        });
    }
}

// --- Popups Info NeoLED ---
function openNeoLedInfo() {
    const popup = document.getElementById('neoledInfoPopup');
    if (!popup) return;
    popup.classList.add('active');
    if (typeof gsap !== 'undefined') {
        gsap.from('#neoledInfoPopup .popup-content', {
            scale: 0.85, opacity: 0, duration: 0.25, ease: 'back.out(1.7)'
        });
    }
}

function closeNeoLedInfo() {
    const popup = document.getElementById('neoledInfoPopup');
    if (!popup) return;
    popup.classList.remove('active');
}

/**
 * Met à jour l'apparence des boutons Système de Vide.
 * Gardée pour compatibilité — l'UI est gérée localement par initCoolantControl.
 */
function updateCoolantButtons(floodOn, mistOn) {
    // Appel délégué à _setState si disponible (aucun effet sur le heartbeat)
    if (typeof initCoolantControl._setState === 'function') {
        initCoolantControl._setState(floodOn, mistOn);
    }
}

/**
 * Initialise les boutons du Système de Vide (M8/M7/M9 coolant FluidNC).
 * Pattern identique à initLightControl / initVacuumControl :
 * état local toggle, mise à jour UI immédiate au clic, zéro dépendance
 * au status report (pour éviter le clignotement dû au heartbeat 200ms).
 */
function initCoolantControl() {
    const btnPump       = document.getElementById('btnPump');
    const btnValve      = document.getElementById('btnValve');
    const btnCoolantOff = document.getElementById('btnCoolantOff');
    const pumpLabel     = document.getElementById('pumpLabel');
    const valveLabel    = document.getElementById('valveLabel');
    const badge         = document.getElementById('coolantStatusBadge');

    let pumpState  = false;
    let valveState = false;

    function updateBadge() {
        if (!badge) return;
        badge.className = 'coolant-status-badge';
        if (pumpState && valveState) {
            badge.textContent = 'Pompe + Vanne actives';
            badge.classList.add('both-on');
        } else if (pumpState) {
            badge.textContent = 'Pompe active (M8)';
            badge.classList.add('pump-on');
        } else if (valveState) {
            badge.textContent = 'Vanne active (M7)';
            badge.classList.add('valve-on');
        } else {
            badge.textContent = 'Inactif';
        }
    }

    function updatePumpUI(state) {
        if (!btnPump) return;
        if (state) {
            btnPump.classList.add('on');
            btnPump.setAttribute('data-state', 'on');
            if (pumpLabel) pumpLabel.textContent = 'ON';
            gsap.to(btnPump.querySelector('.coolant-icon'), { scale: 1.1, duration: 0.3, ease: 'back.out(2)' });
        } else {
            btnPump.classList.remove('on');
            btnPump.setAttribute('data-state', 'off');
            if (pumpLabel) pumpLabel.textContent = 'Pompe';
            gsap.to(btnPump.querySelector('.coolant-icon'), { scale: 1, duration: 0.3, ease: 'power2.out' });
        }
        gsap.fromTo(btnPump, { scale: 0.95 }, { scale: 1, duration: 0.2, ease: 'back.out(2)' });
    }

    function updateValveUI(state) {
        if (!btnValve) return;
        if (state) {
            btnValve.classList.add('on');
            btnValve.setAttribute('data-state', 'on');
            if (valveLabel) valveLabel.textContent = 'ON';
            gsap.to(btnValve.querySelector('.coolant-icon'), { scale: 1.1, duration: 0.3, ease: 'back.out(2)' });
        } else {
            btnValve.classList.remove('on');
            btnValve.setAttribute('data-state', 'off');
            if (valveLabel) valveLabel.textContent = 'Vanne';
            gsap.to(btnValve.querySelector('.coolant-icon'), { scale: 1, duration: 0.3, ease: 'power2.out' });
        }
        gsap.fromTo(btnValve, { scale: 0.95 }, { scale: 1, duration: 0.2, ease: 'back.out(2)' });
    }

    if (btnPump) {
        btnPump.addEventListener('click', function() {
            pumpState = !pumpState;
            if (pumpState) {
                sendCommand('M8');
                addTerminalLine('Vide', 'Pompe ON → M8', 'success');
            } else {
                // M9 coupe aussi la vanne — on réflète ça dans l'UI
                sendCommand('M9');
                addTerminalLine('Vide', 'Pompe OFF → M9', 'info');
                valveState = false;
                updateValveUI(false);
            }
            updatePumpUI(pumpState);
            updateBadge();
        });
    }

    if (btnValve) {
        btnValve.addEventListener('click', function() {
            valveState = !valveState;
            if (valveState) {
                sendCommand('M7');
                addTerminalLine('Vide', 'Vanne ON → M7', 'success');
            } else {
                // M9 coupe aussi la pompe — on réflète ça dans l'UI
                sendCommand('M9');
                addTerminalLine('Vide', 'Vanne OFF → M9', 'info');
                pumpState = false;
                updatePumpUI(false);
            }
            updateValveUI(valveState);
            updateBadge();
        });
    }

    if (btnCoolantOff) {
        btnCoolantOff.addEventListener('click', function() {
            pumpState  = false;
            valveState = false;
            sendCommand('M9');
            addTerminalLine('Vide', 'Tout coupé → M9', 'info');
            updatePumpUI(false);
            updateValveUI(false);
            updateBadge();
            gsap.fromTo(btnCoolantOff, { scale: 0.95 }, { scale: 1, duration: 0.2, ease: 'back.out(2)' });
        });
    }
}

/**
 * Initialise le contrôle d'aspiration (M64/M65 P2)
 */
function initVacuumControl() {
    const btnVacuum = document.getElementById('btnVacuum');
    const vacuumLabel = document.getElementById('vacuumLabel');
    
    if (!btnVacuum) return;
    
    let vacuumState = false;
    
    btnVacuum.addEventListener('click', function() {
        vacuumState = !vacuumState;
        
        // Envoyer directement la commande M-code à FluidNC
        if (vacuumState) {
            sendCommand('M64 P2');
            addTerminalLine('Système', 'Aspiration: M64 P2 (ON)', 'success');
        } else {
            sendCommand('M65 P2');
            addTerminalLine('Système', 'Aspiration: M65 P2 (OFF)', 'info');
        }
        
        // Mettre à jour l'interface
        updateVacuumUI(vacuumState);
    });
    
    function updateVacuumUI(state) {
        if (state) {
            btnVacuum.classList.add('on');
            btnVacuum.setAttribute('data-state', 'on');
            if (vacuumLabel) vacuumLabel.textContent = 'Marche';
            gsap.to(btnVacuum.querySelector('.vacuum-icon'), {
                rotation: 0, scale: 1.1, duration: 0.3, ease: 'back.out(2)'
            });
        } else {
            btnVacuum.classList.remove('on');
            btnVacuum.setAttribute('data-state', 'off');
            if (vacuumLabel) vacuumLabel.textContent = 'Arrêt';
            gsap.to(btnVacuum.querySelector('.vacuum-icon'), {
                rotation: -180, scale: 1, duration: 0.3, ease: 'power2.out'
            });
        }
        gsap.fromTo(btnVacuum, { scale: 0.95 }, { scale: 1, duration: 0.2, ease: 'back.out(2)' });
    }
}

/**
 * Exécute une commande de jog
 */
/**
 * _sendJogStart(cmd) — Construit et envoie la commande $J= pour le hold-to-move.
 * Utilise une distance très grande (9999 mm) : FluidNC s'arrête sur le Hard Limit
 * ou sur la commande Jog Cancel (\x85) envoyée au pointerup.
 * Le pas (currentStep) n'est PAS utilisé ici — il sert uniquement au mode 'step'.
 */
function _sendJogStart(cmd) {
    let jogCmd = '$J=G91';
    const axisPattern = /([XYZA])([+-])/g;
    let match;
    while ((match = axisPattern.exec(cmd)) !== null) {
        const axis = match[1];
        const dir  = match[2] === '+' ? 1 : -1;
        jogCmd += ` ${axis}${(9999 * dir).toFixed(0)}`;
    }
    jogCmd += ` F${jogFeedRate}`;
    // Envoi direct (bypass queue) : le jog est une commande temps-réel de fait
    sendRealtime(jogCmd);
}

/**
 * _sendJogStep(cmd) — Mode "Par Pas" : envoie UN SEUL $J=G91 avec la distance exacte
 * définie par currentStep (0.1 / 1 / 10 / 100 mm). Aucun Jog Cancel n'est envoyé —
 * la machine s'arrête seule à la cible mathématique. Précision absolue garantie.
 */
function _sendJogStep(cmd) {
    let jogCmd = '$J=G91';
    const axisPattern = /([XYZA])([+-])/g;
    let match;
    let hasAxis = false;
    while ((match = axisPattern.exec(cmd)) !== null) {
        const axis = match[1];
        const dir  = match[2] === '+' ? 1 : -1;
        jogCmd += ` ${axis}${(currentStep * dir).toFixed(3)}`;
        hasAxis = true;
    }
    if (!hasAxis) return;
    jogCmd += ` F${jogFeedRate}`;
    sendRealtime(jogCmd);
}

/**
 * executeJogCommand(cmd) — Mode clic simple (pas défini par le sélecteur de pas).
 * Conservé pour compatibilité avec d'éventuels appels externes.
 */
function executeJogCommand(cmd) {
    if (cmd.startsWith('G')) { sendCommand(cmd); return; }
    let jogCmd = '$J=G91';
    const axisPattern = /([XYZA])([+-])/g;
    let match;
    while ((match = axisPattern.exec(cmd)) !== null) {
        const axis = match[1];
        const dir  = match[2] === '+' ? 1 : -1;
        jogCmd += ` ${axis}${(currentStep * dir).toFixed(3)}`;
    }
    jogCmd += ` F${jogFeedRate}`;
    sendRealtime(jogCmd);
}

/**
 * Initialise le sélecteur de pas
 */
function initStepSelector() {
    const stepButtons = document.querySelectorAll('.step-btn');
    
    stepButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            // Retirer la classe active de tous les boutons
            stepButtons.forEach(b => b.classList.remove('active'));
            // Ajouter la classe active au bouton cliqué
            this.classList.add('active');
            // Mettre à jour le pas courant
            currentStep = parseFloat(this.getAttribute('data-step'));
            console.log(`📏 Pas de jog: ${currentStep}mm`);
        });
    });
}

/**
 * Initialise les boutons de sécurité
 */
function initSafetyButtons() {
    // Bouton STOP (Arrêt d'urgence - Hold puis Reset)
    const btnStop = document.getElementById('btnStop');
    if (btnStop) {
        btnStop.addEventListener('click', function() {
            // Séquence d'arrêt : Hold -> Soft Reset
            // Cela met la machine en état HOLD, permettant un Unlock ultérieur
            
            // 1. Hold immédiat (0x85)
            sendCommand(String.fromCharCode(0x85));
            
            // 2. Soft Reset après 100ms (0x18 = Ctrl-X)
            setTimeout(() => {
                sendCommand(String.fromCharCode(0x18));
            }, 100);
            
            // Feedback visuel immédiat
            addTerminalLine('Système', '⛔ ARRÊT D\'URGENCE ACTIVÉ ⛔', 'error');
            addTerminalLine('Système', 'Machine en HOLD - Utilisez Unlock pour reprendre', 'info');
            
            // Animation du bouton
            gsap.fromTo(this, 
                { scale: 0.95 },
                { scale: 1, duration: 0.2, ease: 'back.out(2)' }
            );
            
            // Mettre à jour l'état visuel
            updateMachineState('HOLD');
        });
    }
    
    // Bouton Unlock ($X) - Gère aussi la sortie de HOLD
    const btnUnlock = document.getElementById('btnUnlock');
    if (btnUnlock) {
        btnUnlock.addEventListener('click', function() {
            // Si la machine est en HOLD, envoyer d'abord ~ pour reprendre
            if (machineState === 'HOLD') {
                sendCommand(String.fromCharCode(0x7E)); // ~ = Cycle Start/Resume
                addTerminalLine('Système', 'Reprise du cycle (sortie de HOLD)', 'info');
            }
            
            // Envoyer le unlock pour déverrouiller les alarmes
            sendCommand('$X');
            
            // Animation du bouton
            gsap.fromTo(this, 
                { scale: 0.95 },
                { scale: 1, duration: 0.15, ease: 'back.out(2)' }
            );
        });
    }
    
    // Bouton Reset ($System/Reset)
    const btnReset = document.getElementById('btnReset');
    if (btnReset) {
        btnReset.addEventListener('click', function() {
            sendCommand('$System/Reset');
        });
    }
}

/**
 * Initialise les boutons de homing
 */
function initHomingButtons() {
    // Home All
    const btnHomeAll = document.getElementById('btnHomeAll');
    if (btnHomeAll) {
        btnHomeAll.addEventListener('click', function() {
            sendCommand('$H');
        });
    }
    
    // Home individuel par axe
    const axisButtons = document.querySelectorAll('.homing-btn[data-axis]');
    axisButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            const axis = this.getAttribute('data-axis');
            sendCommand(`$H${axis}`);
        });
    });
}

/**
 * Initialise le terminal
 */
function initTerminal() {
    const terminalInput = document.getElementById('terminalInput');
    const btnSend = document.getElementById('btnSend');
    const btnCopy = document.getElementById('btnCopyTerminal');
    const btnClear = document.getElementById('btnClearTerminal');
    const terminalOutput = document.getElementById('terminalOutput');
    
    if (terminalInput) {
        // Envoi avec la touche Entrée
        terminalInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                sendTerminalCommand();
            }
        });
    }
    
    if (btnSend) {
        btnSend.addEventListener('click', sendTerminalCommand);
    }
    
    // Bouton copier tout le terminal
    if (btnCopy) {
        btnCopy.addEventListener('click', copyAllTerminal);
    }
    
    // Bouton effacer tout le terminal
    if (btnClear) {
        btnClear.addEventListener('click', clearTerminal);
    }
    
    // Gestion du défilement manuel vs auto-scroll
    if (terminalOutput) {
        let isUserScrolling = false;
        let scrollTimeout;
        
        terminalOutput.addEventListener('scroll', function() {
            isUserScrolling = true;
            clearTimeout(scrollTimeout);
            
            // Détecter si l'utilisateur est en bas du terminal
            const isAtBottom = terminalOutput.scrollHeight - terminalOutput.scrollTop - terminalOutput.clientHeight < 50;
            
            if (isAtBottom) {
                isUserScrolling = false;
            } else {
                // Réactiver l'auto-scroll après 3 secondes d'inactivité
                scrollTimeout = setTimeout(() => {
                    isUserScrolling = false;
                }, 3000);
            }
        });
        
        // Stocker l'état pour addTerminalLine
        terminalOutput.isUserScrolling = () => isUserScrolling;
    }
}

/**
 * Copie un texte dans le presse-papiers avec repli HTTP.
 * navigator.clipboard n'est disponible qu'en contexte sécurisé (HTTPS / localhost).
 * L'ESP32 servant la page en HTTP brut, on utilise document.execCommand('copy')
 * via un textarea temporaire en repli.
 * @returns {Promise<boolean>} résolu à true si la copie a réussi
 */
function copyTextToClipboard(text) {
    return new Promise((resolve) => {
        // 1) Tentative API moderne (HTTPS / localhost)
        if (navigator.clipboard && window.isSecureContext) {
            navigator.clipboard.writeText(text)
                .then(() => resolve(true))
                .catch(() => resolve(fallbackCopy(text)));
            return;
        }
        // 2) Repli compatible HTTP (ESP32)
        resolve(fallbackCopy(text));
    });
}

/**
 * Repli de copie via document.execCommand('copy')
 */
function fallbackCopy(text) {
    try {
        const ta = document.createElement('textarea');
        ta.value = text;
        // Hors-écran mais sélectionnable
        ta.style.position = 'fixed';
        ta.style.top = '-1000px';
        ta.style.left = '-1000px';
        ta.style.opacity = '0';
        ta.setAttribute('readonly', '');
        document.body.appendChild(ta);
        ta.focus();
        ta.select();
        ta.setSelectionRange(0, text.length);
        const ok = document.execCommand('copy');
        document.body.removeChild(ta);
        return ok;
    } catch (e) {
        console.error('Repli copie échoué :', e);
        return false;
    }
}

/**
 * Copie tout le contenu du terminal dans le presse-papiers
 */
function copyAllTerminal() {
    const terminalOutput = document.getElementById('terminalOutput');
    if (!terminalOutput) return;
    
    // Récupérer tout le texte des lignes
    const lines = terminalOutput.querySelectorAll('.terminal-line');
    let text = '';
    lines.forEach(line => {
        text += line.textContent + '\n';
    });
    
    // Copier dans le presse-papiers (avec repli HTTP)
    copyTextToClipboard(text).then((ok) => {
        if (ok) {
            addTerminalLine('Système', 'Terminal copié dans le presse-papiers', 'success');
        } else {
            addTerminalLine('Erreur', 'Impossible de copier le terminal', 'error');
        }
    });
}

/**
 * Efface tout le contenu de la console FluidNC
 */
function clearTerminal() {
    const terminalOutput = document.getElementById('terminalOutput');
    if (!terminalOutput) return;
    
    // Vider l'intégralité du terminal
    terminalOutput.innerHTML = '';
    
    // Réafficher l'en-tête système
    addTerminalLine('Système', '=== Terminal FluidNC ===', 'system');
    addTerminalLine('Système', 'Console effacée', 'system');
    addTerminalLine('Système', 'Cliquez sur une ligne pour la copier', 'system');
}

/**
 * Copie une ligne spécifique dans le presse-papiers
 */
function copyLineToClipboard(lineElement) {
    const text = lineElement.textContent;
    copyTextToClipboard(text).then((ok) => {
        if (ok) {
            // Feedback visuel
            lineElement.style.background = 'rgba(0, 217, 255, 0.2)';
            setTimeout(() => {
                lineElement.style.background = '';
            }, 300);
        } else {
            console.error('Copie de la ligne impossible');
        }
    });
}

/**
 * Envoie la commande du terminal
 */
function sendTerminalCommand() {
    const terminalInput = document.getElementById('terminalInput');
    if (terminalInput) {
        const command = terminalInput.value.trim();
        if (command) {
            sendCommand(command);
            terminalInput.value = '';
        }
    }
}

/**
 * Ajoute une ligne au terminal
 * Gère la déduplication des messages "ok" consécutifs
 */
function addTerminalLine(source, text, type = 'info') {
    const terminalOutput = document.getElementById('terminalOutput');
    if (!terminalOutput) return;
    
    // Vérifier si c'est un message "ok" (insensible à la casse et trim)
    const isOkMessage = text.toString().trim().toLowerCase() === 'ok';
    
    if (isOkMessage) {
        // Récupérer la dernière ligne
        const lastLine = terminalOutput.lastElementChild;
        
        if (lastLine && lastLine.dataset.isOk === 'true') {
            // C'est déjà un "ok", incrémenter le compteur
            let okCount = parseInt(lastLine.dataset.okCount || '1') + 1;
            lastLine.dataset.okCount = okCount;
            
            // Mettre à jour l'affichage avec le compteur
            const now = new Date();
            const time = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
            lastLine.innerHTML = `<span class="terminal-time">[${time}]</span> <span class="terminal-source">${source}:</span> <span class="terminal-text">ok <span class="ok-counter">(×${okCount})</span></span>`;
            
            // Auto-scroll si nécessaire
            const isUserScrolling = terminalOutput.isUserScrolling ? terminalOutput.isUserScrolling() : false;
            if (!isUserScrolling) {
                terminalOutput.scrollTop = terminalOutput.scrollHeight;
            }
            return;
        }
    }
    
    const line = document.createElement('div');
    line.className = `terminal-line ${type}`;
    line.title = 'Cliquez pour copier cette ligne';
    
    // Marquer si c'est un message ok
    if (isOkMessage) {
        line.dataset.isOk = 'true';
        line.dataset.okCount = '1';
    }
    
    // Ajouter un timestamp
    const now = new Date();
    const time = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    
    line.innerHTML = `<span class="terminal-time">[${time}]</span> <span class="terminal-source">${source}:</span> <span class="terminal-text">${escapeHtml(text)}</span>`;
    
    // Ajouter l'événement de copie au clic
    line.addEventListener('click', function() {
        copyLineToClipboard(this);
    });
    
    terminalOutput.appendChild(line);
    
    // Auto-scroll vers le bas uniquement si l'utilisateur ne fait pas défiler manuellement
    const isUserScrolling = terminalOutput.isUserScrolling ? terminalOutput.isUserScrolling() : false;
    if (!isUserScrolling) {
        terminalOutput.scrollTop = terminalOutput.scrollHeight;
    }
    
    // Limiter le nombre de lignes (garder les 200 dernières)
    while (terminalOutput.children.length > 200) {
        terminalOutput.removeChild(terminalOutput.firstChild);
    }
}

/**
 * Échappe les caractères HTML
 */
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

/**
 * Anime l'apparition des cartes au chargement
 */
function animateCardsEntry() {
    const cards = document.querySelectorAll('.control-card');
    
    gsap.from(cards, {
        duration: 0.6,
        opacity: 0,
        y: 30,
        stagger: 0.1,
        ease: 'power3.out',
        delay: 0.2
    });
}

/**
 * Initialise les accordéons avec persistance des états
 */
function initAccordions() {
    // Charger les états sauvegardés depuis localStorage
    const savedStates = JSON.parse(localStorage.getItem('accordionStates') || '{}');
    
    // Appliquer les états sauvegardés
    Object.keys(savedStates).forEach(id => {
        const card = document.querySelector(`[data-accordion="${id}"]`);
        if (card && savedStates[id]) {
            card.classList.add('expanded');
        }
    });
    
    // Le DRO est toujours ouvert par défaut
    const droCard = document.querySelector('[data-accordion="dro"]');
    if (droCard) {
        droCard.classList.add('expanded');
    }
}

/**
 * Bascule l'état d'un accordéon
 */
function toggleAccordion(id) {
    const card = document.querySelector(`[data-accordion="${id}"]`);
    if (!card) return;
    
    const isExpanded = card.classList.contains('expanded');
    
    if (isExpanded) {
        card.classList.remove('expanded');
    } else {
        card.classList.add('expanded');
    }
    
    // Sauvegarder l'état dans localStorage
    const savedStates = JSON.parse(localStorage.getItem('accordionStates') || '{}');
    savedStates[id] = !isExpanded;
    localStorage.setItem('accordionStates', JSON.stringify(savedStates));
    
    // Animation de l'icône
    const icon = card.querySelector('.accordion-icon');
    if (icon) {
        gsap.to(icon, {
            rotation: isExpanded ? 0 : 180,
            duration: 0.3,
            ease: 'power2.out'
        });
    }
}

// ==================== GESTION DES OFFSETS G54-G57 ====================

let currentOffset = 'G54';
let offsetData = {
    'G54': { X: 0, Y: 0, Z: 0, A: 0 },
    'G55': { X: 0, Y: 0, Z: 0, A: 0 },
    'G56': { X: 0, Y: 0, Z: 0, A: 0 },
    'G57': { X: 0, Y: 0, Z: 0, A: 0 }
};

/**
 * Initialise les contrôles d'offsets
 * Charge les valeurs depuis l'ESP32 (Preferences) au démarrage
 */
function initOffsetControls() {
    // Attendre que la connexion WebSocket soit établie
    setTimeout(() => {
        if (cncConnected) {
            // Demander tous les offsets à l'ESP32
            loadOffsetsFromESP32();
            
            // Afficher l'offset actif après réception
            setTimeout(() => {
                displayOffset(currentOffset);
            }, 500);
        }
    }, 1000);
}

/**
 * Sélectionne un offset (G54-G57) et charge les valeurs depuis l'ESP32
 */
function selectOffset(offset) {
    // Sauvegarder les valeurs actuelles avant de changer
    saveCurrentOffsetValues();
    
    // Mettre à jour l'offset actif
    currentOffset = offset;
    
    // Mettre à jour les onglets visuels
    document.querySelectorAll('.offset-tab').forEach(tab => {
        tab.classList.remove('active');
        if (tab.dataset.offset === offset) {
            tab.classList.add('active');
        }
    });
    
    // Demander les valeurs à l'ESP32
    sendCommand(`Offset/Get/${offset}`);
    
    // Afficher les valeurs locales en attendant la réponse
    displayOffset(offset);
}

/**
 * Affiche les valeurs d'un offset dans les inputs
 */
function displayOffset(offset) {
    const data = offsetData[offset];
    if (!data) return;
    
    document.getElementById('offsetX').value = data.X.toFixed(3);
    document.getElementById('offsetY').value = data.Y.toFixed(3);
    document.getElementById('offsetZ').value = data.Z.toFixed(3);
    document.getElementById('offsetA').value = data.A.toFixed(3);
}

/**
 * Sauvegarde les valeurs actuelles des inputs dans offsetData
 */
function saveCurrentOffsetValues() {
    offsetData[currentOffset] = {
        X: parseFloat(document.getElementById('offsetX').value) || 0,
        Y: parseFloat(document.getElementById('offsetY').value) || 0,
        Z: parseFloat(document.getElementById('offsetZ').value) || 0,
        A: parseFloat(document.getElementById('offsetA').value) || 0
    };
}

/**
 * Applique la position actuelle de la machine dans les champs offset
 */
function applyCurrentPosition() {
    document.getElementById('offsetX').value = workPosition.X.toFixed(3);
    document.getElementById('offsetY').value = workPosition.Y.toFixed(3);
    document.getElementById('offsetZ').value = workPosition.Z.toFixed(3);
    document.getElementById('offsetA').value = workPosition.A.toFixed(3);
    
    // Sauvegarder dans offsetData
    saveCurrentOffsetValues();
    
    addTerminalLine('Offset', `Position actuelle appliquée à ${currentOffset}`, 'info');
}

// Variables temporaires pour la popup
let pendingOffsetData = null;

/**
 * Ouvre la popup de confirmation avec les valeurs à sauver
 */
function confirmAndSaveOffset() {
    // Sauvegarder les valeurs actuelles
    saveCurrentOffsetValues();
    
    const data = offsetData[currentOffset];
    pendingOffsetData = data;
    
    // Remplir la popup
    document.getElementById('popupOffsetName').textContent = currentOffset;
    document.getElementById('popupX').textContent = data.X.toFixed(3);
    document.getElementById('popupY').textContent = data.Y.toFixed(3);
    document.getElementById('popupZ').textContent = data.Z.toFixed(3);
    document.getElementById('popupA').textContent = data.A.toFixed(3);
    
    // Afficher la popup
    const popup = document.getElementById('offsetPopup');
    popup.classList.add('active');
    
    // Animation d'entrée
    gsap.from('.popup-content', {
        scale: 0.8,
        opacity: 0,
        duration: 0.3,
        ease: 'back.out(1.7)'
    });
}

/**
 * Ferme la popup de confirmation
 */
function closeOffsetPopup() {
    const popup = document.getElementById('offsetPopup');
    popup.classList.remove('active');
    pendingOffsetData = null;
}

/**
 * Exécute la sauvegarde après confirmation
 */
function executeSaveOffset() {
    if (!pendingOffsetData) return;
    
    // Fermer la popup
    closeOffsetPopup();
    
    // Exécuter le vrai applyAndSaveOffset
    applyAndSaveOffset();
}

// Variable temporaire pour la popup GO TO
let pendingGoToOffset = null;

/**
 * Ouvre la popup de confirmation GO TO
 */
function goToOffset() {
    const data = offsetData[currentOffset];
    if (!data) {
        addTerminalLine('Offset', `Erreur: ${currentOffset} non défini`, 'error');
        return;
    }
    
    // Stocker les données pour exécution
    pendingGoToOffset = data;
    
    // Remplir la popup
    document.getElementById('gotoPopupOffsetName').textContent = currentOffset;
    document.getElementById('gotoPopupX').textContent = data.X.toFixed(3);
    document.getElementById('gotoPopupY').textContent = data.Y.toFixed(3);
    document.getElementById('gotoPopupZ').textContent = data.Z.toFixed(3);
    document.getElementById('gotoPopupA').textContent = data.A.toFixed(3);
    document.getElementById('gotoConfirmOffset').textContent = currentOffset;
    
    // Afficher la popup
    const popup = document.getElementById('gotoPopup');
    popup.classList.add('active');
    
    // Animation d'entrée
    gsap.from('#gotoPopup .popup-content', {
        scale: 0.8,
        opacity: 0,
        duration: 0.3,
        ease: 'back.out(1.7)'
    });
}

/**
 * Ferme la popup GO TO
 */
function closeGotoPopup() {
    const popup = document.getElementById('gotoPopup');
    popup.classList.remove('active');
    pendingGoToOffset = null;
}

/**
 * Exécute le déplacement GO TO après confirmation.
 *
 * Séquence sécurisée (G90 + G53 pour remonter Z en coordonnées machine) :
 *   1) G90                — mode absolu (anti G91 residuel)
 *   2) G53 G0 Z0          — remonter Z en coordonnées MACHINE (ignorant tout offset)
 *   3) G54/G55/G56/G57    — activer le workspace cible
 *   4) G0 X0 Y0           — aller a l'origine XY de la piece
 *   5) G0 Z0              — descendre a l'origine Z de la piece
 *
 * IMPORTANT : On va a X0 Y0 Z0 du WORKSPACE, pas aux valeurs d'offset.
 * Les offsetData.X/Y/Z sont les coordonnees machine de l'origine piece,
 * pas les coordonnees de destination.
 * L'erreur precedente (envoyer data.X/Y/Z + axe A) causait :
 *   - error:29 "gcode unsupported" (axe A inexistant sur machine 3 axes)
 *   - mouvement Z dangereux (pas de G90 explicite, pas de G53)
 */
function executeGoToOffset() {
    if (!pendingGoToOffset) return;

    // Fermer la popup
    closeGotoPopup();

    const offset = currentOffset; // ex: 'G56'

    addTerminalLine('Offset', `GO TO ${offset} : sequence securisee...`, 'info');

    // Etape 1 : mode absolu (securite anti G91 residuel)
    sendCommand('G90');

    // Etape 2 : remonter Z en coordonnees MACHINE (G53 ignore les offsets)
    // Delai de 100 ms pour laisser FluidNC traiter le G90 avant la suite
    setTimeout(() => {
        addTerminalLine('Offset', 'Etape 1/4: Montee Z securite (G53 G0 Z0)...', 'info');
        sendCommand('G53 G0 Z0');

        // Etape 3 : activer le workspace cible + aller a l'origine XY
        setTimeout(() => {
            addTerminalLine('Offset', `Etape 2/4: Activation ${offset} + G0 X0 Y0...`, 'info');
            sendCommand(offset);

            // Petit delai pour que le workspace soit actif
            setTimeout(() => {
                sendCommand('G0 X0 Y0');

                // Etape 4 : descendre Z a l'origine piece
                setTimeout(() => {
                    addTerminalLine('Offset', 'Etape 3/4: G0 Z0 (origine piece)...', 'info');
                    sendCommand('G0 Z0');

                    addTerminalLine('Offset', `Position ${offset} atteinte !`, 'success');
                    highlightOffsetTab(offset);
                }, 400);
            }, 200);
        }, 400);
    }, 100);
}

/**
 * Sauvegarde l'offset dans l'ESP32 et met à jour l'affichage
 */
function saveOffset() {
    // Sauvegarder les valeurs actuelles
    saveCurrentOffsetValues();
    
    // Envoyer à l'ESP32 via WebSocket
    const data = offsetData[currentOffset];
    const cmd = `Offset/${currentOffset}=${data.X},${data.Y},${data.Z},${data.A}`;
    
    if (sendCommand(cmd)) {
        addTerminalLine('Offset', `${currentOffset} sauvegardé dans ESP32`, 'success');
        
        // Mettre à jour l'affichage pour confirmer
        displayOffset(currentOffset);
    }
}

/**
 * Construit la commande G10 L2 correcte pour FluidNC.
 *
 * Règles :
 *  - Le paramètre P est l'INDEX du repère (P1=G54, P2=G55, P3=G56, P4=G57).
 *    Formule : parseInt(offset.replace('G','')) - 53
 *    Exemples : G54→P1, G55→P2, G56→P3, G57→P4
 *  - L'axe A n'est PAS inclus : FluidNC renvoie error:29 sur les machines
 *    3 axes si un axe non configuré est mentionné dans G10.
 *
 * @param {string} offset  - ex: "G54"
 * @param {{X,Y,Z}} data   - valeurs de l'offset
 * @returns {string}        - ex: "G10 L2 P1 X50.000 Y50.000 Z-70.000"
 */
function buildG10Command(offset, data) {
    const pNum = parseInt(offset.replace('G', '')) - 53;
    const x = parseFloat(data.X).toFixed(3);
    const y = parseFloat(data.Y).toFixed(3);
    const z = parseFloat(data.Z).toFixed(3);
    return `G10 L2 P${pNum} X${x} Y${y} Z${z}`;
}

/**
 * Applique l'offset à FluidNC (G10 L2 Pn X Y Z)
 */
function applyOffset() {
    saveCurrentOffsetValues();

    const data = offsetData[currentOffset];
    const cmd = buildG10Command(currentOffset, data);

    if (sendCommand(cmd)) {
        setTimeout(() => {
            sendCommand(currentOffset);
            addTerminalLine('Offset', `${currentOffset} appliqué et activé`, 'success');
        }, 150);
    }
}

/**
 * Sauvegarde ET applique l'offset en une seule action
 * Met aussi à jour l'affichage
 */
function applyAndSaveOffset() {
    saveCurrentOffsetValues();

    const data = offsetData[currentOffset];

    // 1. Sauvegarder dans l'ESP32
    const saveCmd = `Offset/${currentOffset}=${data.X},${data.Y},${data.Z},${data.A}`;

    // 2. Appliquer à FluidNC avec G10 (P correct, sans axe A)
    const applyCmd = buildG10Command(currentOffset, data);

    // Envoyer la sauvegarde
    if (sendCommand(saveCmd)) {
        addTerminalLine('Offset', `${currentOffset} sauvegardé dans ESP32`, 'success');
        
        // Mettre à jour l'affichage immédiatement
        displayOffset(currentOffset);
        
        // Puis appliquer après un court délai
        setTimeout(() => {
            sendCommand(applyCmd);
            
            // Activer le système de coordonnées
            setTimeout(() => {
                sendCommand(currentOffset);
                addTerminalLine('Offset', `${currentOffset} appliqué et activé`, 'success');
                
                // Animation visuelle de confirmation
                highlightOffsetTab(currentOffset);
            }, 100);
        }, 100);
    }
}

/**
 * Anime visuellement l'onglet de l'offset actif pour confirmer
 */
function highlightOffsetTab(offset) {
    const tab = document.querySelector(`.offset-tab[data-offset="${offset}"]`);
    if (tab) {
        // Animation GSAP de confirmation
        gsap.to(tab, {
            scale: 1.1,
            duration: 0.15,
            yoyo: true,
            repeat: 1,
            ease: 'power2.out'
        });
        
        // Effet de glow temporaire
        tab.style.boxShadow = '0 0 20px rgba(0, 255, 136, 0.6)';
        setTimeout(() => {
            tab.style.boxShadow = '';
        }, 1000);
    }
}

/**
 * Charge tous les offsets depuis l'ESP32
 */
function loadOffsetsFromESP32() {
    // Demander les offsets à l'ESP32
    sendCommand('Offset/GetAll');
}

/**
 * Met à jour les données d'offset reçues de l'ESP32
 */
function handleOffsetData(message) {
    // Format: Offset/G54=10.000,20.000,5.000,0.000
    const match = message.match(/Offset\/(G5[4-7])=([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)/);
    if (match) {
        const offset = match[1];
        offsetData[offset] = {
            X: parseFloat(match[2]),
            Y: parseFloat(match[3]),
            Z: parseFloat(match[4]),
            A: parseFloat(match[5])
        };
        
        // Mettre à jour l'affichage si c'est l'offset actuel
        if (offset === currentOffset) {
            displayOffset(offset);
        }
        
        // Log pour debug
        addTerminalLine('Offset', `${offset} chargé depuis ESP32`, 'info');
    }
}
        // Mettre à jour l'affichage si c'est l'offset actuel
        if (offset === currentOffset) {
            displayOffset(offset);
        }
        
        // Log pour debug
        addTerminalLine('Offset', `${offset} chargé depuis ESP32`, 'info');
    }
}
        // Mettre à jour l'affichage si c'est l'offset actuel
        if (offset === currentOffset) {
            displayOffset(offset);
        }
        
        // Log pour debug
        addTerminalLine('Offset', `${offset} chargé depuis ESP32`, 'info');
    }
}
