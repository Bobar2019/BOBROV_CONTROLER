/* =============================================================================
 *  BOBCNC — Bug Reporter Global
 *  -----------------------------
 *  Script auto-injecté sur toutes les pages de l'interface BOBCNC pour permettre
 *  aux bêta-testeurs de signaler un bug en 2 clics, en arrière-plan et silencieux.
 *
 *  Cycle de vie :
 *    1. POST /api/bug/report     -> ESP32 stocke en LittleFS (FIFO 20 max)
 *    2. POST <bugWebhookUrl>     -> Webhook externe (Formspree par défaut)
 *       Les deux POST sont déclenchés en parallèle. Le succès du webhook est la
 *       condition principale du feedback "Rapport envoyé en arrière-plan".
 *
 *  Inclusion : <script src="/bug_reporter.js" defer></script> dans le <head>.
 *  Idempotent : la garde window.__BUGREPORTER_INIT__ évite la double injection.
 * ========================================================================== */
(function() {
    'use strict';
    if (window.__BUGREPORTER_INIT__) return;
    window.__BUGREPORTER_INIT__ = true;

    // ---------------------------------------------------------------- STYLES
    const css = `
        .br-fab { position: fixed; right: 14px; bottom: 14px; z-index: 9000;
            width: 44px; height: 44px; border-radius: 50%; border: 1px solid rgba(255,180,0,0.55);
            background: rgba(255,180,0,0.14); color: #ffb400; font-size: 22px; line-height: 1;
            display: flex; align-items: center; justify-content: center; cursor: pointer;
            box-shadow: 0 4px 14px rgba(0,0,0,0.45); transition: all 0.2s; padding: 0; }
        .br-fab:hover { background: rgba(255,180,0,0.28); transform: scale(1.07);
            box-shadow: 0 0 14px rgba(255,180,0,0.45); }
        .br-fab:active { transform: scale(0.95); }
        @media (max-width: 760px) { .br-fab { right: 10px; bottom: 76px; width: 40px; height: 40px; font-size: 20px; } }

        .br-overlay { display: none; position: fixed; inset: 0; z-index: 9100;
            background: rgba(0,0,0,0.72); backdrop-filter: blur(3px);
            align-items: center; justify-content: center; padding: 16px; }
        .br-overlay.active { display: flex; }
        .br-modal { background: #1a1f25; color: #e8edf2; border: 1px solid #2a3441;
            border-radius: 12px; padding: 18px 20px 20px; width: 100%; max-width: 460px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.65);
            font-family: system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif; }
        .br-head { display: flex; justify-content: space-between; align-items: center;
            margin-bottom: 12px; gap: 10px; }
        .br-head h3 { margin: 0; font-size: 1rem; color: #ffb400; display: flex;
            align-items: center; gap: 8px; }
        .br-close { background: none; border: none; color: #8d99a8; font-size: 1.5rem;
            cursor: pointer; padding: 0 6px; line-height: 1; border-radius: 4px; }
        .br-close:hover { background: rgba(255,255,255,0.08); color: #fff; }
        .br-modal label { display: block; font-size: 0.72rem; color: #8d99a8;
            text-transform: uppercase; letter-spacing: 0.4px; margin: 10px 0 4px; font-weight: 600; }
        .br-modal textarea { width: 100%; min-height: 70px; padding: 8px 10px;
            background: #0c1014; border: 1px solid #2a3441; border-radius: 6px;
            color: #e8edf2; font-family: inherit; font-size: 0.85rem; resize: vertical; outline: none; }
        .br-modal textarea:focus { border-color: #00d9ff; }
        .br-meta { margin-top: 10px; padding: 8px 10px; background: rgba(0,217,255,0.06);
            border: 1px solid rgba(0,217,255,0.25); border-radius: 6px;
            font-size: 0.7rem; font-family: 'Source Code Pro', monospace; color: #8d99a8;
            line-height: 1.5; word-break: break-word; }
        .br-meta strong { color: #00d9ff; }
        .br-actions { display: flex; gap: 8px; margin-top: 14px; justify-content: flex-end; }
        .br-btn { padding: 8px 14px; border-radius: 6px; border: 1px solid #2a3441;
            background: #1a1f25; color: #e8edf2; font-size: 0.82rem; cursor: pointer;
            transition: all 0.18s; }
        .br-btn:hover { border-color: #4a5562; background: #232931; }
        .br-btn.primary { background: rgba(0,255,136,0.18); border-color: rgba(0,255,136,0.55);
            color: #00ff88; font-weight: 600; }
        .br-btn.primary:hover { background: rgba(0,255,136,0.28); }
        .br-btn:disabled { opacity: 0.5; cursor: not-allowed; }
        .br-status { font-size: 0.75rem; color: #8d99a8; margin-top: 8px; min-height: 1.1em; }
        .br-status.success { color: #00ff88; }
        .br-status.error { color: #ff6666; }
    `;
    const styleEl = document.createElement('style');
    styleEl.id = 'br-styles';
    styleEl.textContent = css;

    // ---------------------------------------------------------------- DOM
    const fab = document.createElement('button');
    fab.className = 'br-fab';
    fab.type = 'button';
    fab.setAttribute('aria-label', 'Signaler un bug');
    fab.title = 'Signaler un bug';
    fab.innerHTML = '\uD83D\uDC1E';   // \ud83d\udc1e

    const overlay = document.createElement('div');
    overlay.className = 'br-overlay';
    overlay.setAttribute('role', 'dialog');
    overlay.setAttribute('aria-modal', 'true');
    overlay.innerHTML = `
        <div class="br-modal" role="document">
            <div class="br-head">
                <h3>\uD83D\uDC1E Signaler un bug</h3>
                <button class="br-close" type="button" aria-label="Fermer">&times;</button>
            </div>
            <label for="br-what">Que s'est-il pass\u00e9 ?</label>
            <textarea id="br-what" placeholder="D\u00e9crivez le probl\u00e8me observ\u00e9..."></textarea>
            <label for="br-repro">Comment reproduire ?</label>
            <textarea id="br-repro" placeholder="\u00c9tape 1, \u00e9tape 2, ..."></textarea>
            <div class="br-meta" id="br-meta"></div>
            <div class="br-actions">
                <button class="br-btn" type="button" id="br-cancel">Annuler</button>
                <button class="br-btn primary" type="button" id="br-send">Envoyer le rapport</button>
            </div>
            <div class="br-status" id="br-status"></div>
        </div>
    `;

    // Injection diff\u00e9r\u00e9e jusqu'\u00e0 ce que <body> existe
    function injectDom() {
        if (!document.body) { setTimeout(injectDom, 30); return; }
        // Si le mini-dashboard est actif, ne PAS injecter le FAB flottant
        if (window.__MINIDASH_ACTIVE__) {
            document.head.appendChild(styleEl);
            document.body.appendChild(overlay);
            wireEvents();
            return;
        }
        document.head.appendChild(styleEl);
        document.body.appendChild(fab);
        document.body.appendChild(overlay);
        wireEvents();
    }

    // ---------------------------------------------------------------- CONTEXTE
    // ----- Buffer circulaire des logs FluidNC (max 20 entrées) -----
    // L'ESP32 ne stocke pas l'historique pour préserver la RAM. On le fait
    // donc côté navigateur en hookant le WebSocket FluidNC déjà géré par main.js
    // (mécanisme window.addCncSocketMessageHandler).
    const FLUIDNC_LOG_MAX = 20;
    const fluidncLogBuffer = [];
    let currentEndstops = 'Aucun';

    function pushFluidLog(line) {
        if (typeof line !== 'string' || !line) return;
        // Tronque les lignes trop longues pour ne pas exploser le payload
        const trimmed = line.length > 240 ? (line.substring(0, 240) + '...') : line;
        fluidncLogBuffer.push(trimmed);
        while (fluidncLogBuffer.length > FLUIDNC_LOG_MAX) fluidncLogBuffer.shift();
    }

    // Trame statut FluidNC: <Idle|MPos:0,0,0|FS:0,0|Pn:XYZ>
    // Pn:XYZP = pins déclenchées (X/Y/Z/P=probe, etc.). Absence du champ = aucune.
    function parseEndstopsFromStatus(msg) {
        if (typeof msg !== 'string') return;
        if (msg.charAt(0) !== '<') return;       // pas une trame statut
        const m = msg.match(/\|Pn:([^|>]+)/);
        if (m && m[1]) {
            // Espacé pour lisibilité dans l'e-mail Formspree
            currentEndstops = m[1].split('').join(' ');
        } else {
            currentEndstops = 'Aucun';
        }
    }

    // Handler dédié au WebSocket FluidNC. Accepte le format JSON {type,data}
    // et le texte brut (les deux sont émis par main.js selon la config).
    function handleFluidWsMessage(raw) {
        if (raw == null) return;
        let payload = raw;
        if (typeof raw === 'string') {
            // Tente le parsing JSON, sinon traite comme texte brut
            try {
                const obj = JSON.parse(raw);
                if (obj && obj.type === 'fluidnc' && typeof obj.data === 'string') {
                    payload = obj.data;
                } else if (obj && typeof obj === 'object') {
                    // Autres types JSON (light/vacuum/io_status...) : on n'enregistre pas
                    return;
                }
            } catch (_) { /* texte brut, on garde tel quel */ }
        } else { return; }

        parseEndstopsFromStatus(payload);
        // On évite de spammer le buffer avec les trames statut '?' pures.
        // On ne stocke que les messages "intéressants" : non-statut, ou statut
        // qui contient au moins une fin de course active.
        const isStatus = payload.charAt(0) === '<';
        if (!isStatus || /\|Pn:/.test(payload)) {
            pushFluidLog(payload);
        }
    }

    // Tente d'enregistrer le handler. main.js peut être chargé après
    // bug_reporter.js (defer), donc on retente avec un petit polling borné.
    function hookFluidSocket() {
        if (typeof window.addCncSocketMessageHandler === 'function') {
            window.addCncSocketMessageHandler(handleFluidWsMessage);
            return true;
        }
        return false;
    }
    if (!hookFluidSocket()) {
        let attempts = 0;
        const t = setInterval(function() {
            attempts++;
            if (hookFluidSocket() || attempts > 30) clearInterval(t); // ~15 s max
        }, 500);
    }

    function shortId() {
        const ts = Date.now().toString(36).toUpperCase();
        const rnd = Math.floor(Math.random() * 0xFFFF).toString(36).toUpperCase().padStart(3, '0');
        return 'BUG-' + ts + '-' + rnd;
    }

    function pageName() {
        const p = (window.location.pathname || '/').split('/').filter(Boolean).pop();
        return p || 'index.html';
    }

    function buildContext() {
        const wp = window.workPosition  || null;     // exposé par main.js
        const mp = window.machinePosition || null;
        const fluidState = (typeof window.machineState === 'string' && window.machineState)
            ? window.machineState
            : (document.getElementById('stateBadge')?.textContent || 'Unknown');
        let bobcncVersion = (window.BOBCNC_VERSION || '');
        return {
            id:        shortId(),
            timestamp: new Date().toISOString(),
            page:      pageName(),
            url:       window.location.href,
            fluid_state: fluidState,
            wpos:      wp ? { x: +(wp.X||0).toFixed(3), y: +(wp.Y||0).toFixed(3), z: +(wp.Z||0).toFixed(3) } : null,
            mpos:      mp ? { x: +(mp.X||0).toFixed(3), y: +(mp.Y||0).toFixed(3), z: +(mp.Z||0).toFixed(3) } : null,
            bobcnc_version: bobcncVersion,
            user_agent: navigator.userAgent,
            viewport:  window.innerWidth + 'x' + window.innerHeight,
            language:  navigator.language || '',
            // —— Diagnostic FluidNC ——
            endstops_state:    currentEndstops || 'Aucun',
            fluidnc_last_logs: fluidncLogBuffer.slice() // copie défensive
        };
    }

    function renderMeta(ctx) {
        const wp = ctx.wpos ? `WPos ${ctx.wpos.x}/${ctx.wpos.y}/${ctx.wpos.z}` : 'WPos n/a';
        const mp = ctx.mpos ? `MPos ${ctx.mpos.x}/${ctx.mpos.y}/${ctx.mpos.z}` : 'MPos n/a';
        const logsCnt = Array.isArray(ctx.fluidnc_last_logs) ? ctx.fluidnc_last_logs.length : 0;
        return `
            <div><strong>ID</strong> ${ctx.id}</div>
            <div><strong>Page</strong> ${ctx.page}  &mdash;  <strong>${ctx.fluid_state}</strong></div>
            <div><strong>Date</strong> ${new Date(ctx.timestamp).toLocaleString()}</div>
            <div>${wp}  &middot;  ${mp}</div>
            <div><strong>Endstops</strong> ${ctx.endstops_state}  &middot;  <strong>${logsCnt}</strong> log(s) FluidNC</div>
            <div style="opacity:0.7">${ctx.viewport}  &middot;  ${(ctx.user_agent||'').substring(0,80)}</div>
        `;
    }

    // ---------------------------------------------------------------- ENVOI
    var BUG_WEBHOOK_DEFAULT = 'https://formspree.io/f/mgobykgk';
    
    // Construit le payload destiné au webhook externe (Formspree, EmailJS, etc.).
    // Clés explicites + ASCII-friendly pour s'intégrer dans les notifications mail.
    function buildWebhookPayload(ctx, what, repro) {
        const logsArr = Array.isArray(ctx.fluidnc_last_logs) ? ctx.fluidnc_last_logs : [];
        const logsStr = logsArr.length ? logsArr.join('\n') : '(aucun log capturé)';
        return {
            subject:           '[BOBCNC Bug] ' + ctx.id + ' depuis la page ' + ctx.page,
            id:                ctx.id,
            timestamp:         ctx.timestamp,
            page:              ctx.page,
            url:               ctx.url,
            description:       what  || '(non renseigné)',
            reproduction:      repro || '(non renseigné)',
            version:           ctx.bobcnc_version || '',
            machine_state:     ctx.fluid_state || 'Unknown',
            wpos:              ctx.wpos ? (ctx.wpos.x + ' / ' + ctx.wpos.y + ' / ' + ctx.wpos.z) : 'n/a',
            mpos:              ctx.mpos ? (ctx.mpos.x + ' / ' + ctx.mpos.y + ' / ' + ctx.mpos.z) : 'n/a',
            endstops_state:    ctx.endstops_state || 'Aucun',
            fluidnc_last_logs: logsStr,
            browser:           ctx.user_agent,
            viewport:          ctx.viewport,
            language:          ctx.language || ''
        };
    }
    
    // POST vers /api/bug/report (ESP32). Renvoie { ok, webhookUrl?, id?, error? }.
    async function postToEsp32(payload) {
        const ctrl = new AbortController();
        const t = setTimeout(() => ctrl.abort(), 6000);
        try {
            const r = await fetch('/api/bug/report', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload),
                signal: ctrl.signal
            });
            clearTimeout(t);
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return await r.json();
        } catch (e) {
            clearTimeout(t);
            return { ok: false, error: String(e && e.message || e) };
        }
    }
    
    // POST direct vers le webhook externe configuré. Renvoie { ok, status?, error? }.
    async function postToWebhook(webhookUrl, payload) {
        const ctrl = new AbortController();
        const t = setTimeout(() => ctrl.abort(), 10000);
        try {
            const r = await fetch(webhookUrl, {
                method: 'POST',
                headers: {
                    'Accept':       'application/json',
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(payload),
                signal: ctrl.signal
            });
            clearTimeout(t);
            return { ok: r.ok, status: r.status };
        } catch (e) {
            clearTimeout(t);
            return { ok: false, error: String(e && e.message || e) };
        }
    }
    
    // Récupère l'URL du webhook configurée côté ESP32 (avec fallback défaut).
    async function fetchWebhookUrl() {
        try {
            const r = await fetch('/api/settings', { cache: 'no-store' });
            if (!r.ok) return BUG_WEBHOOK_DEFAULT;
            const j = await r.json();
            return (j && typeof j.bugWebhookUrl === 'string' && j.bugWebhookUrl)
                ? j.bugWebhookUrl
                : BUG_WEBHOOK_DEFAULT;
        } catch (_) {
            return BUG_WEBHOOK_DEFAULT;
        }
    }

    // ---------------------------------------------------------------- UI
    let currentCtx = null;

    function open() {
        currentCtx = buildContext();
        document.getElementById('br-meta').innerHTML = renderMeta(currentCtx);
        document.getElementById('br-status').textContent = '';
        document.getElementById('br-status').className = 'br-status';
        document.getElementById('br-what').value  = '';
        document.getElementById('br-repro').value = '';
        overlay.classList.add('active');
        setTimeout(() => document.getElementById('br-what').focus(), 50);
    }

    function close() { overlay.classList.remove('active'); }

    async function send() {
        const sendBtn = document.getElementById('br-send');
        const status  = document.getElementById('br-status');
        const what    = document.getElementById('br-what').value.trim();
        const repro   = document.getElementById('br-repro').value.trim();
        if (!what && !repro) {
            status.textContent = 'Veuillez décrire au moins un élément.';
            status.className = 'br-status error';
            return;
        }
    
        // UI : verrouillage bouton + label "Envoi en cours..."
        const originalLabel = sendBtn.textContent;
        sendBtn.disabled   = true;
        sendBtn.textContent = 'Envoi en cours...';
        status.textContent  = '';
        status.className    = 'br-status';
    
        // Récupération de l'URL du webhook (configurée ou défaut Formspree)
        const webhookUrl = await fetchWebhookUrl();
    
        // Construction des deux payloads
        const esp32Payload   = Object.assign({}, currentCtx, { what: what, repro: repro });
        const webhookPayload = buildWebhookPayload(currentCtx, what, repro);
    
        // Envoi en parallèle : ESP32 (sauvegarde locale) + Webhook (notification)
        const [espRes, hookRes] = await Promise.all([
            postToEsp32(esp32Payload),
            postToWebhook(webhookUrl, webhookPayload)
        ]);
    
        // Le succès du webhook est la condition principale du feedback positif.
        if (hookRes.ok) {
            status.textContent = 'Rapport envoyé en arrière-plan ! (' + currentCtx.id + ')';
            status.className   = 'br-status success';
            // Vide le formulaire et ferme la modale après 2 s
            setTimeout(function() {
                document.getElementById('br-what').value  = '';
                document.getElementById('br-repro').value = '';
                close();
                sendBtn.disabled   = false;
                sendBtn.textContent = originalLabel;
            }, 2000);
        } else {
            // Échec webhook : on indique au moins si la sauvegarde locale a fonctionné.
            const localOk = espRes && espRes.ok;
            status.textContent = localOk
                ? ('Webhook injoignable. Rapport conservé localement (' + currentCtx.id + ').')
                : ('Échec d’envoi : ' + (hookRes.error || 'webhook indisponible'));
            status.className = 'br-status error';
            sendBtn.disabled   = false;
            sendBtn.textContent = originalLabel;
        }
    }

    function wireEvents() {
        fab.addEventListener('click', open);
        overlay.addEventListener('click', function(e) {
            if (e.target === overlay) close();
        });
        overlay.querySelector('.br-close').addEventListener('click', close);
        document.getElementById('br-cancel').addEventListener('click', close);
        document.getElementById('br-send').addEventListener('click', send);
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape' && overlay.classList.contains('active')) close();
        });
    }

    // ---------------------------------------------------------------- API publique
    window.openBugReporter = open;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', injectDom);
    } else {
        injectDom();
    }
})();
