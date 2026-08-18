/* =============================================================================
 *  BOBCNC — Mini Dashboard Global
 *  ---------------------------------
 *  Barre d'etat fixee en haut de chaque page, visible en permanence.
 *
 *  Composants :
 *    - Coordonnees MPos X, Y, Z (temps reel via WebSocket FluidNC)
 *    - Temperature coffret (DS18B20 GPIO 10, poll via /api/temperature)
 *    - Bouton Controle (raccourci vers l'onglet CONTROLE CNC)
 *    - Bouton Rapport de Bug (integre, remplace le FAB flottant)
 *
 *  Inclusion : <script src="/mini_dashboard.js" defer></script> dans le <head>.
 *  Idempotent : la garde window.__MINIDASH_INIT__ evite la double injection.
 * ========================================================================== */
(function() {
    'use strict';
    if (window.__MINIDASH_INIT__) return;
    window.__MINIDASH_INIT__ = true;

    // ---------------------------------------------------------------- STYLES
    const css = `
        :root { --mdash-h: 55px; }
        @media (max-width: 760px) {
            :root { --mdash-h: 48px; }
        }
        body { padding-top: var(--mdash-h) !important; }

        .mdash-bar {
            position: fixed; top: 0; left: 0; right: 0; z-index: 99990;
            height: var(--mdash-h);
            background: #0c1016;
            background: rgba(10, 14, 20, 0.96);
            backdrop-filter: blur(8px);
            -webkit-backdrop-filter: blur(8px);
            border-bottom: 1px solid rgba(0, 217, 255, 0.22);
            display: flex; align-items: center; gap: 10px; padding: 0 13px;
            font-family: 'Source Code Pro', 'Courier New', monospace;
            font-size: 1.03rem; color: #c8d6e5;
            box-shadow: 0 1px 8px rgba(0,0,0,0.5);
            user-select: none; -webkit-user-select: none;
        }
        @media (max-width: 760px) {
            .mdash-bar { gap: 5px; padding: 0 6px; font-size: 0.81rem; }
        }

        .mdash-mpos { display: flex; align-items: center; gap: 15px; flex: 1;
            min-width: 0; overflow: hidden; }
        @media (max-width: 760px) { .mdash-mpos { gap: 5px; } }

        .mdash-axis { display: flex; align-items: baseline; gap: 3px; white-space: nowrap; }
        .mdash-axis .label { color: #5a7a9a; font-weight: 600; font-size: 0.9rem; }
        .mdash-axis .value { font-weight: 600; min-width: 70px; text-align: right;
            font-size: 1.2rem; transition: color 0.3s; }
        .mdash-axis.x .value { color: #00ff88; }
        .mdash-axis.y .value { color: #00aaff; }
        .mdash-axis.z .value { color: #ff4466; }
        @media (max-width: 760px) {
            .mdash-axis .label { font-size: 0.73rem; }
            .mdash-axis .value { min-width: 53px; font-size: 0.88rem; }
        }

        .mdash-sep { width: 1px; height: 22px; background: rgba(255,255,255,0.1); margin: 0 5px;
            flex-shrink: 0; }
        @media (max-width: 760px) { .mdash-sep { margin: 0 3px; height: 18px; } }

        .mdash-temp { display: flex; align-items: center; gap: 5px; white-space: nowrap;
            flex-shrink: 0; }
        .mdash-temp .icon { font-size: 1.06rem; }
        .mdash-temp .value { color: #ff9f43; font-weight: 500; min-width: 53px; text-align: right; }
        .mdash-temp .value.error { color: #666; }
        @media (max-width: 760px) {
            .mdash-temp .value { min-width: 43px; font-size: 0.75rem; }
        }

        .mdash-btn {
            flex-shrink: 0; width: 36px; height: 36px; border-radius: 8px;
            border: 1px solid rgba(255,255,255,0.12); background: rgba(255,255,255,0.04);
            color: #8d99a8; font-size: 1.06rem; cursor: pointer; display: flex;
            align-items: center; justify-content: center; padding: 0;
            transition: all 0.2s; line-height: 1;
        }
        .mdash-btn:hover { background: rgba(255,255,255,0.1); color: #fff;
            border-color: rgba(255,255,255,0.25); }
        .mdash-btn:active { transform: scale(0.93); }
        .mdash-btn.ctrl { color: #00d9ff; }
        .mdash-btn.ctrl:hover { border-color: rgba(0,217,255,0.5); box-shadow: 0 0 8px rgba(0,217,255,0.25); }
        .mdash-btn.bug { color: #ffb400; }
        .mdash-btn.bug:hover { border-color: rgba(255,180,0,0.5); box-shadow: 0 0 8px rgba(255,180,0,0.25); }
        .mdash-btn.fullscreen { color: #b070ff; }
        .mdash-btn.fullscreen:hover { border-color: rgba(176,112,255,0.5); box-shadow: 0 0 8px rgba(176,112,255,0.25); }
        .mdash-btn svg { width: 18px; height: 18px; }
        @media (max-width: 760px) {
            .mdash-btn { width: 30px; height: 30px; }
            .mdash-btn svg { width: 15px; height: 15px; }
        }

        /* ---- Etat machine (pastille + texte) ---- */
        .mdash-state { display: flex; align-items: center; gap: 6px; white-space: nowrap;
            flex-shrink: 0; font-size: 0.88rem; font-weight: 500; }
        .mdash-state .dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0;
            box-shadow: 0 0 6px currentColor; transition: background 0.3s, box-shadow 0.3s; }
        .mdash-state .label { color: #8899aa; }
        .mdash-state.idle .dot { background: #00ff88; box-shadow: 0 0 6px rgba(0,255,136,0.6); }
        .mdash-state.idle .label { color: #00ff88; }
        .mdash-state.run .dot { background: #00aaff; box-shadow: 0 0 6px rgba(0,170,255,0.6); }
        .mdash-state.run .label { color: #00aaff; }
        .mdash-state.jog .dot { background: #ff9f43; box-shadow: 0 0 6px rgba(255,159,67,0.6); }
        .mdash-state.jog .label { color: #ff9f43; }
        .mdash-state.hold .dot { background: #ffb400; box-shadow: 0 0 6px rgba(255,180,0,0.6); }
        .mdash-state.hold .label { color: #ffb400; }
        .mdash-state.alarm .dot { background: #ff3333; box-shadow: 0 0 8px rgba(255,51,51,0.8);
            animation: mdash-alarm-pulse 0.6s ease-in-out infinite; }
        .mdash-state.alarm .label { color: #ff3333; animation: mdash-alarm-pulse 0.6s ease-in-out infinite; }
        .mdash-state.home .dot { background: #00d9ff; box-shadow: 0 0 6px rgba(0,217,255,0.6); }
        .mdash-state.home .label { color: #00d9ff; }
        .mdash-state.offline .dot { background: #555; box-shadow: none; }
        .mdash-state.offline .label { color: #555; }
        @keyframes mdash-alarm-pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.4; }
        }
        @media (max-width: 760px) {
            .mdash-state { font-size: 0.75rem; gap: 4px; }
            .mdash-state .dot { width: 8px; height: 8px; }
        }

        /* ---- Feedrate Override ---- */
        .mdash-feed { display: flex; align-items: center; gap: 5px; white-space: nowrap;
            flex-shrink: 0; }
        .mdash-feed-label { color: #5a7a9a; font-weight: 500; font-size: 0.81rem; }
        .mdash-feed-val { color: #ff9f43; font-weight: 600; font-size: 1.03rem;
            min-width: 42px; text-align: right; transition: color 0.2s; }
        .mdash-feed-val.boost { color: #00ff88; }
        .mdash-feed-val.reduced { color: #ff6b6b; }
        .mdash-fbtn { flex-shrink: 0; height: 32px; min-width: 36px; padding: 0 10px;
            border-radius: 6px; border: 1px solid rgba(255,255,255,0.12);
            background: rgba(255,255,255,0.04); color: #8d99a8;
            font-size: 0.81rem; font-weight: 600; font-family: inherit;
            cursor: pointer; display: flex; align-items: center; justify-content: center;
            transition: all 0.15s; line-height: 1; user-select: none; }
        .mdash-fbtn:hover { background: rgba(0,217,255,0.12); color: #00d9ff;
            border-color: rgba(0,217,255,0.4); }
        .mdash-fbtn:active { transform: scale(0.92); }
        .mdash-fbtn.reset { color: #ff9f43; border-color: rgba(255,159,67,0.25); }
        .mdash-fbtn.reset:hover { background: rgba(255,159,67,0.12); color: #ffb84d;
            border-color: rgba(255,159,67,0.5); }
        @media (max-width: 760px) {
            .mdash-feed { gap: 3px; }
            .mdash-feed-label { display: none; }
            .mdash-feed-val { font-size: 0.81rem; min-width: 35px; }
            .mdash-fbtn { height: 26px; min-width: 28px; padding: 0 6px; font-size: 0.73rem; }
        }
    `;
    const styleEl = document.createElement('style');
    styleEl.id = 'mdash-styles';
    styleEl.textContent = css;

    // ---------------------------------------------------------------- HTML
    const bar = document.createElement('div');
    bar.className = 'mdash-bar';
    bar.setAttribute('role', 'status');
    bar.setAttribute('aria-label', 'Mini tableau de bord');
    bar.innerHTML = `
        <div class="mdash-mpos">
            <span class="mdash-axis x"><span class="label">X:</span><span class="value" id="mdash-x">0.000</span></span>
            <span class="mdash-axis y"><span class="label">Y:</span><span class="value" id="mdash-y">0.000</span></span>
            <span class="mdash-axis z"><span class="label">Z:</span><span class="value" id="mdash-z">0.000</span></span>
        </div>
        <span class="mdash-sep"></span>
        <span class="mdash-temp">
            <span class="icon">🌡️</span>
            <span class="value" id="mdash-temp">--.-°C</span>
        </span>
        <span class="mdash-sep"></span>
        <span class="mdash-state" id="mdash-state">
            <span class="dot"></span>
            <span class="label">---</span>
        </span>
        <span class="mdash-sep"></span>
        <span class="mdash-feed">
            <span class="mdash-feed-label">Avance:</span>
            <span class="mdash-feed-val" id="mdash-feed">100%</span>
            <button class="mdash-fbtn" id="mdash-feed-down" type="button" title="Avance -10%">−10%</button>
            <button class="mdash-fbtn reset" id="mdash-feed-reset" type="button" title="Avance 100%">100%</button>
            <button class="mdash-fbtn" id="mdash-feed-up" type="button" title="Avance +10%">+10%</button>
        </span>
        <span class="mdash-sep"></span>
        <button class="mdash-btn fullscreen" id="mdash-fullscreen" type="button" title="Plein écran" aria-label="Plein écran">
            <svg class="icon-expand" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/>
            </svg>
            <svg class="icon-compress" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="display:none">
                <path d="M9 4v5H4M15 4v5h5M9 20v-5H4M15 20v-5h5"/>
            </svg>
        </button>
        <button class="mdash-btn ctrl" id="mdash-ctrl" type="button" title="Aller au CONTROLE CNC" aria-label="CONTROLE CNC">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <circle cx="12" cy="12" r="3"/><path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42"/>
            </svg>
        </button>
        <button class="mdash-btn bug" id="mdash-bug" type="button" title="Signaler un bug" aria-label="Signaler un bug">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
                <ellipse cx="12" cy="13" rx="8" ry="7" stroke-width="2"/>
                <line x1="12" y1="6" x2="12" y2="13" stroke-width="2"/>
                <circle cx="8" cy="11" r="1.2" fill="currentColor" stroke="none"/>
                <circle cx="15" cy="11" r="1.2" fill="currentColor" stroke="none"/>
                <circle cx="10" cy="15" r="0.9" fill="currentColor" stroke="none"/>
                <circle cx="14" cy="15" r="0.9" fill="currentColor" stroke="none"/>
                <path d="M5 8 Q2 2 6 6" stroke-width="1.5"/>
                <path d="M19 8 Q22 2 18 6" stroke-width="1.5"/>
                <path d="M6 19 Q2 22 5 20" stroke-width="1.5"/>
                <path d="M18 19 Q22 22 19 20" stroke-width="1.5"/>
            </svg>
        </button>
    `;

    // ---------------------------------------------------------------- INJECTION
    function injectDom() {
        if (!document.body) { setTimeout(injectDom, 30); return; }
        document.head.appendChild(styleEl);
        document.body.insertBefore(bar, document.body.firstChild);
        console.log('[MiniDash] Barre injectee (z-index:99990, body padding-top:' + getComputedStyle(document.body).paddingTop + ')');
        resolveDOMElements();
        wireEvents();
        startTemperaturePoll();
        hookWebSocket();
    }

    // ---------------------------------------------------------------- EVENEMENTS
    function wireEvents() {
        const btnCtrl = document.getElementById('mdash-ctrl');
        const btnBug  = document.getElementById('mdash-bug');

        if (btnCtrl) {
            btnCtrl.addEventListener('click', function() {
                // Navigation vers CONTROLE CNC
                window.location.href = '/controle.html';
            });
        }

        if (btnBug) {
            btnBug.addEventListener('click', function() {
                // Utilise le bug reporter global s'il est charge
                if (typeof window.openBugReporter === 'function') {
                    window.openBugReporter();
                } else {
                    // Fallback : navigation vers configuration (qui a aussi le reporter)
                    window.location.href = '/configuration.html';
                }
            });
        }

        // ---- Feedrate Override buttons ----
        // 0x90 = reset 100%, 0x91 = +10%, 0x92 = -10% (commandes temps-reel GRBL)
        const btnFeedDown  = document.getElementById('mdash-feed-down');
        const btnFeedReset = document.getElementById('mdash-feed-reset');
        const btnFeedUp    = document.getElementById('mdash-feed-up');

        if (btnFeedDown)  btnFeedDown.addEventListener('click',  function() { sendFeedOverride(0x92); });
        if (btnFeedReset) btnFeedReset.addEventListener('click', function() { sendFeedOverride(0x90); });
        if (btnFeedUp)    btnFeedUp.addEventListener('click',    function() { sendFeedOverride(0x91); });

        // ---- Bouton Plein Ecran ----
        const btnFs = document.getElementById('mdash-fullscreen');
        if (btnFs) btnFs.addEventListener('click', toggleFullScreen);

        // Mettre a jour l'icone quand l'etat fullscreen change
        document.addEventListener('fullscreenchange',        updateFsIcon);
        document.addEventListener('webkitfullscreenchange',  updateFsIcon);
        document.addEventListener('mozfullscreenchange',     updateFsIcon);
        document.addEventListener('MSFullscreenChange',      updateFsIcon);
    }

    // ---- Gestion Plein Ecran (multi-navigateur) ----
    // La difficulté : le navigateur quitte le plein écran à chaque navigation.
    // Astuce : mémoriser l'état dans sessionStorage et restaurer au 1er clic
    // sur la nouvelle page. Le timeout sur fullscreenchange distingue Esc
    // (efface le flag) de la navigation (pagehide annule le timeout → flag préservé).
    var _fsClearTimer = null;

    function _isFullscreen() {
        return !!(document.fullscreenElement || document.webkitFullscreenElement
               || document.mozFullScreenElement || document.msFullscreenElement);
    }

    function _enterFullscreen() {
        var el = document.documentElement;
        if (el.requestFullscreen)        el.requestFullscreen();
        else if (el.webkitRequestFullscreen) el.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
        else if (el.mozRequestFullScreen)    el.mozRequestFullScreen();
        else if (el.msRequestFullscreen)     el.msRequestFullscreen();
    }

    function _exitFullscreen() {
        if (document.exitFullscreen)        document.exitFullscreen();
        else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
        else if (document.mozCancelFullScreen)   document.mozCancelFullScreen();
        else if (document.msExitFullscreen)      document.msExitFullscreen();
    }

    function toggleFullScreen() {
        if (!_isFullscreen()) {
            sessionStorage.setItem('bobcnc_fs', '1');
            _enterFullscreen();
        } else {
            sessionStorage.removeItem('bobcnc_fs');
            _exitFullscreen();
        }
    }

    function updateFsIcon() {
        var btn = document.getElementById('mdash-fullscreen');
        if (!btn) return;
        var isFs = _isFullscreen();
        var expand = btn.querySelector('.icon-expand');
        var compress = btn.querySelector('.icon-compress');
        if (expand)   expand.style.display   = isFs ? 'none'   : '';
        if (compress) compress.style.display = isFs ? ''       : 'none';
        btn.title = isFs ? 'Quitter le plein écran' : 'Plein écran';

        // Gestion du flag sessionStorage pour persistance inter-pages
        if (!isFs) {
            // Plein écran perdu (Esc ou navigation)
            // Délai avant effacement : si c'est une navigation, pagehide
            // annulera ce timer et préservera le flag
            _fsClearTimer = setTimeout(function() {
                sessionStorage.removeItem('bobcnc_fs');
            }, 300);
        } else {
            // Plein écran actif → s'assurer que le flag est bien présent
            if (_fsClearTimer) { clearTimeout(_fsClearTimer); _fsClearTimer = null; }
            sessionStorage.setItem('bobcnc_fs', '1');
        }
    }

    // Restaurer le plein écran sur la nouvelle page au 1er geste utilisateur
    function tryRestoreFullscreen() {
        if (sessionStorage.getItem('bobcnc_fs') !== '1') return;
        function restore() {
            if (!_isFullscreen()) _enterFullscreen();
            document.removeEventListener('click', restore);
            document.removeEventListener('keydown', restore);
            document.removeEventListener('touchstart', restore);
        }
        document.addEventListener('click', restore);
        document.addEventListener('keydown', restore);
        document.addEventListener('touchstart', restore);
    }

    // Annuler l'effacement du flag lors d'une navigation (préserve le plein écran)
    window.addEventListener('pagehide', function() {
        if (_fsClearTimer) { clearTimeout(_fsClearTimer); _fsClearTimer = null; }
    });

    // ---------------------------------------------------------------- MPos (WebSocket)
    // Utilise window.workPosition (WPos) comme le DRO de controle,
    // avec fallback sur window.machinePosition (MPos) si WPos absent.
    let mposX = 0, mposY = 0, mposZ = 0;
    let elX = null, elY = null, elZ = null;  // Resolus apres injection DOM
    let elState = null;  // Etat machine
    let elFeed = null;   // Feedrate override

    function resolveDOMElements() {
        elX = document.getElementById('mdash-x');
        elY = document.getElementById('mdash-y');
        elZ = document.getElementById('mdash-z');
        elState = document.getElementById('mdash-state');
        elFeed = document.getElementById('mdash-feed');
    }

    function updateMPosDisplay() {
        if (elX) elX.textContent = mposX.toFixed(3);
        if (elY) elY.textContent = mposY.toFixed(3);
        if (elZ) elZ.textContent = mposZ.toFixed(3);
    }

    // Etats possibles de FluidNC, mappes vers des labels courts
    const STATE_LABELS = {
        'Idle': 'REPOS', 'Run': 'USINAGE', 'Jog': 'JOG', 'Hold': 'PAUSE',
        'Alarm': 'ALARME', 'Home': 'HOME', 'Check': 'CHECK', 'Sleep': 'SLEEP',
        'Door': 'PORTE'
    };
    let lastState = '';

    function updateStateDisplay() {
        if (!elState) return;
        const raw = (window.machineState || 'OFFLINE').toString();
        const key = raw.charAt(0).toUpperCase() + raw.slice(1).toLowerCase();
        const label = STATE_LABELS[key] || raw.toUpperCase();
        const cls = key.toLowerCase();
        if (raw === lastState) return;
        lastState = raw;
        elState.className = 'mdash-state ' + cls;
        const lblEl = elState.querySelector('.label');
        if (lblEl) lblEl.textContent = label;
    }

    // ---- Feedrate Override ----
    // Format FluidNC : |Ov:100,100,100  (feed, spindle, rapid)
    let feedOverride = 100;

    function updateFeedDisplay() {
        if (!elFeed) return;
        elFeed.textContent = feedOverride + '%';
        elFeed.classList.remove('boost', 'reduced');
        if (feedOverride > 100) elFeed.classList.add('boost');
        else if (feedOverride < 100) elFeed.classList.add('reduced');
    }

    function sendFeedOverride(byteVal) {
        // Envoi d'un octet pur (commande temps-reel GRBL) via le WebSocket existant
        if (typeof window.sendCncRealtime === 'function') {
            // main.js expose sendRealtime qui envoie directement sur cncSocket
            window.sendCncRealtime(String.fromCharCode(byteVal));
            // Mise a jour optimiste : FluidNC renverra la vraie valeur dans le prochain status
            if (byteVal === 0x90) feedOverride = 100;
            else if (byteVal === 0x91) feedOverride = Math.min(feedOverride + 10, 400);
            else if (byteVal === 0x92) feedOverride = Math.max(feedOverride - 10, 10);
            updateFeedDisplay();
        }
    }

    function hookWebSocket() {
        // Lecture de window.workPosition (WPos) — meme source que le DRO de controle
        // window.workPosition est mis a jour par main.js (updateDRO, ligne 1096)

        function pollMPos() {
            updateStateDisplay();  // met a jour la pastille etat machine
            const pos = window.workPosition || window.machinePosition;
            if (pos) {
                if (pos.X !== mposX || pos.Y !== mposY || pos.Z !== mposZ) {
                    mposX = pos.X; mposY = pos.Y; mposZ = pos.Z;
                    updateMPosDisplay();
                }
            }
            requestAnimationFrame(pollMPos);
        }
        requestAnimationFrame(pollMPos);

        // Fallback : si machinePosition n'est pas disponible (page sans main.js),
        // on ecoute les io_status WebSocket qui contiennent parfois les positions
        // via le hook global de bug_reporter.js
        if (window.addCncSocketMessageHandler) {
            window.addCncSocketMessageHandler(function(msg) {
                if (typeof msg !== 'string') return;
                // Tenter d'extraire MPos du message brut FluidNC
                if (msg.includes('MPos:')) {
                    const m = msg.match(/MPos:([-\d.]+),([-\d.]+),([-\d.]+)/);
                    if (m) {
                        mposX = parseFloat(m[1]);
                        mposY = parseFloat(m[2]);
                        mposZ = parseFloat(m[3]);
                        updateMPosDisplay();
                    }
                }
                // Extraire l'etat machine : <Idle|...>, <Run|...>, etc.
                if (msg.startsWith('<')) {
                    const st = msg.match(/<(\w+)/);
                    if (st) {
                        window.machineState = st[1];  // injecte dans le scope global
                        updateStateDisplay();
                    }
                    // Extraire le feedrate override : |Ov:100,100,100
                    const ov = msg.match(/Ov:(\d+),/);
                    if (ov) {
                        const newFeed = parseInt(ov[1], 10);
                        if (newFeed !== feedOverride) {
                            feedOverride = newFeed;
                            updateFeedDisplay();
                        }
                    }
                }
            });
        }
    }

    // ---------------------------------------------------------------- TEMPERATURE
    const elTemp = document.getElementById('mdash-temp');
    let tempFailCount = 0;

    async function fetchTemperature() {
        try {
            const resp = await fetch('/api/temperature', { signal: AbortSignal.timeout(3000) });
            if (!resp.ok) throw new Error('HTTP ' + resp.status);
            const data = await resp.json();
            if (data.valid && data.temperature > -50) {
                if (elTemp) {
                    elTemp.textContent = data.temperature.toFixed(1) + '°C';
                    elTemp.classList.remove('error');
                }
                tempFailCount = 0;
            } else {
                showTempError();
            }
        } catch(e) {
            showTempError();
        }
    }

    function showTempError() {
        tempFailCount++;
        if (tempFailCount >= 3 && elTemp) {
            elTemp.textContent = '--.-°C';
            elTemp.classList.add('error');
        }
    }

    function startTemperaturePoll() {
        fetchTemperature();  // premiere immediate
        setInterval(fetchTemperature, 5000);  // toutes les 5 secondes
    }

    // ---------------------------------------------------------------- DEMARRAGE
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function() {
            injectDom();
            tryRestoreFullscreen();
        });
    } else {
        injectDom();
        tryRestoreFullscreen();
    }

    // Exposer l'API pour le bug reporter (pour qu'il sache ne pas injecter son FAB)
    window.__MINIDASH_ACTIVE__ = true;
    // Exposer toggleFullScreen pour appel externe
    window.toggleFullScreen = toggleFullScreen;
})();
