// =====================================================================
//  taquin3d_humain.js — Simulation 3D mode Bulldozer (BOBCNC)
// =====================================================================
//
//  Variante dediee au mode "Comme un humain" (Bulldozer).
//  Difference cle avec taquin3d.js :
//    - Lors d'une poussee Bulldozer, la ventouse saisit la tuile la
//      plus eloignee et la glisse d'un entraxe. Pendant ce glissement,
//      TOUTES les tuiles poussees se deplacent en parallele, animees
//      comme un bloc solide.
//
//  API publique exposee sur window :
//    openTaquinHumainVisualizer(params, modalId, canvasId)
//      params = {
//         size, tileSize, zSafe, zPick,
//         initialBoard, moves, solveMode, chains,
//         pushedTilesMap  // Map<moveIndex, [{tileValue,fromX,fromY,toX,toY}]>
//      }
//
//  Ce module est entierement autonome. Aucun couplage avec taquin3d.js.
// =====================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------------------------------------------------------------------
//  Couleurs / materiaux
// ---------------------------------------------------------------------
const COLOR_BG          = 0x101418;
const COLOR_BOARD       = 0x2a2f3a;
const COLOR_BOARD_FRAME = 0x44505f;
const COLOR_TILE_TOP    = 0xffe9b0;
const COLOR_TILE_SIDE   = 0xc8a06a;
const COLOR_SUCTION_TUBE = 0x666c75;
const COLOR_SUCTION_PAD_OFF = 0x884444;
const COLOR_SUCTION_PAD_ON  = 0x33ff66;

const SUCTION_DIAMETER  = 10;
const SUCTION_TUBE_LEN  = 28;
const SUCTION_PAD_DIAM  = 12;
const SUCTION_PAD_THICK = 2;
const TILE_THICKNESS    = 4;

// ---------------------------------------------------------------------
//  Texture numero de tuile
// ---------------------------------------------------------------------
function makeNumberTexture(num) {
    const size = 256;
    const c = document.createElement('canvas');
    c.width = size; c.height = size;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#ffe9b0';
    ctx.fillRect(0, 0, size, size);
    ctx.strokeStyle = '#a0793a';
    ctx.lineWidth = 8;
    ctx.strokeRect(4, 4, size - 8, size - 8);
    ctx.fillStyle = '#1a1f25';
    ctx.font = 'bold ' + (num >= 10 ? 130 : 160) + 'px "Source Code Pro", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(num), size / 2, size / 2 + 8);
    const tex = new THREE.CanvasTexture(c);
    tex.anisotropy = 4;
    tex.needsUpdate = true;
    return tex;
}

// =====================================================================
//  Classe principale — Mode Humain (Bulldozer)
// =====================================================================
class TaquinHumainVisualizer3D {
    constructor(canvasId) {
        this.canvasId = canvasId;
        this.container = null;
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.boardGroup = null;
        this.tileMeshes = {};
        this.suction = null;
        this.suctionPad = null;
        this.attachedTile = null;
        this._raf = null;
        this._timeline = null;
        this._stepIdx = 0;
        this._stepTime = 0;
        this._lastT = 0;
        this._playing = false;
        this._speed = 1.0;
        this.params = null;
        this._onResize = null;
        // Tuiles en cours de glissement Bulldozer (non attachees a la ventouse)
        this._slidingTiles = [];
    }

    // -----------------------------------------------------------------
    //  INIT (identique a taquin3d.js)
    // -----------------------------------------------------------------
    async init() {
        this.container = document.getElementById(this.canvasId);
        if (!this.container) throw new Error('canvas container introuvable: ' + this.canvasId);

        const w = this.container.clientWidth || 800;
        const h = this.container.clientHeight || 500;

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(COLOR_BG);

        this.camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 5000);
        this.camera.up.set(0, 0, 1);
        this.camera.position.set(150, -180, 180);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
        const pr = (window.matchMedia && window.matchMedia('(max-width:500px)').matches) ? 1 : Math.min(window.devicePixelRatio, 1.5);
        this.renderer.setPixelRatio(pr);
        this.renderer.setSize(w, h);
        this.renderer.shadowMap.enabled = false;
        this.container.appendChild(this.renderer.domElement);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0, 0, 0);
        this.controls.addEventListener('change', () => { this._renderNeeded = true; });

        const ambient = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambient);
        const dir = new THREE.DirectionalLight(0xffffff, 0.7);
        dir.position.set(120, -150, 250);
        this.scene.add(dir);

        const grid = new THREE.GridHelper(400, 20, 0x223040, 0x1a2028);
        grid.rotation.x = Math.PI / 2;
        grid.position.z = -0.01;
        this.scene.add(grid);

        this._onResize = () => {
            const ww = this.container.clientWidth;
            const hh = this.container.clientHeight;
            if (ww > 0 && hh > 0) {
                this.camera.aspect = ww / hh;
                this.camera.updateProjectionMatrix();
                this.renderer.setSize(ww, hh);
                this._renderNeeded = true;
            }
        };
        window.addEventListener('resize', this._onResize);

        this._renderNeeded = true;
        this._lastT = performance.now() / 1000;
        const loop = () => {
            this._raf = requestAnimationFrame(loop);
            const now = performance.now() / 1000;
            const dt = Math.min(0.1, now - this._lastT);
            this._lastT = now;
            if (this._playing) {
                this._tickAnimation(dt * this._speed);
                this._renderNeeded = true;
            }
            const moved = this.controls.update();
            if (moved) this._renderNeeded = true;
            if (this._renderNeeded) {
                this.renderer.render(this.scene, this.camera);
                this._renderNeeded = false;
            }
        };
        loop();
    }

    // -----------------------------------------------------------------
    //  CHARGEMENT D'UN PUZZLE (identique a taquin3d.js)
    // -----------------------------------------------------------------
    loadPuzzle(params) {
        this.params = params;
        this._clearWorld();

        const { size, tileSize } = params;
        const totalLen = size * tileSize;

        this.boardGroup = new THREE.Group();
        this.scene.add(this.boardGroup);

        const baseThick = 3;
        const frameW = 4;
        const baseGeom = new THREE.BoxGeometry(totalLen + 2 * frameW, totalLen + 2 * frameW, baseThick);
        const baseMat = new THREE.MeshStandardMaterial({ color: COLOR_BOARD_FRAME, roughness: 0.85 });
        const base = new THREE.Mesh(baseGeom, baseMat);
        base.position.set(totalLen / 2, totalLen / 2, -baseThick / 2);
        this.boardGroup.add(base);

        const cellGeom = new THREE.BoxGeometry(tileSize - 0.5, tileSize - 0.5, baseThick - 0.4);
        const cellMat = new THREE.MeshStandardMaterial({ color: COLOR_BOARD, roughness: 0.95 });
        for (let r = 0; r < size; r++) {
            for (let c = 0; c < size; c++) {
                const cell = new THREE.Mesh(cellGeom, cellMat);
                cell.position.set(
                    c * tileSize + tileSize / 2,
                    (size - 1 - r) * tileSize + tileSize / 2,
                    -baseThick / 2 + 0.05
                );
                this.boardGroup.add(cell);
            }
        }

        const orig = new THREE.Mesh(
            new THREE.BoxGeometry(2, 2, 2),
            new THREE.MeshBasicMaterial({ color: 0x00ff88 })
        );
        orig.position.set(0, 0, 0);
        this.boardGroup.add(orig);

        const tileGeom = new THREE.BoxGeometry(tileSize - 1, tileSize - 1, TILE_THICKNESS);
        const sideMat = new THREE.MeshStandardMaterial({ color: COLOR_TILE_SIDE, roughness: 0.7 });

        for (let i = 0; i < size * size; i++) {
            const v = params.initialBoard[i];
            if (v === 0) continue;
            const r = (i / size) | 0;
            const c = i % size;
            const topMat = new THREE.MeshStandardMaterial({
                map: makeNumberTexture(v), roughness: 0.55
            });
            const materials = [sideMat, sideMat, sideMat, sideMat, topMat, sideMat];
            const tile = new THREE.Mesh(tileGeom, materials);
            tile.position.set(
                c * tileSize + tileSize / 2,
                (size - 1 - r) * tileSize + tileSize / 2,
                TILE_THICKNESS / 2
            );
            this.scene.add(tile);
            this.tileMeshes[v] = tile;
        }

        // Ventouse
        this.suction = new THREE.Group();
        const tubeGeom = new THREE.CylinderGeometry(
            SUCTION_DIAMETER / 2, SUCTION_DIAMETER / 2, SUCTION_TUBE_LEN, 16
        );
        tubeGeom.rotateX(Math.PI / 2);
        const tubeMat = new THREE.MeshStandardMaterial({
            color: COLOR_SUCTION_TUBE, metalness: 0.55, roughness: 0.4
        });
        const tube = new THREE.Mesh(tubeGeom, tubeMat);
        tube.position.set(0, 0, SUCTION_PAD_THICK + SUCTION_TUBE_LEN / 2);
        this.suction.add(tube);

        const padGeom = new THREE.CylinderGeometry(
            SUCTION_PAD_DIAM / 2, SUCTION_PAD_DIAM / 2, SUCTION_PAD_THICK, 16
        );
        padGeom.rotateX(Math.PI / 2);
        const padMat = new THREE.MeshStandardMaterial({
            color: COLOR_SUCTION_PAD_OFF, roughness: 0.5
        });
        this.suctionPad = new THREE.Mesh(padGeom, padMat);
        this.suctionPad.position.set(0, 0, SUCTION_PAD_THICK / 2);
        this.suction.add(this.suctionPad);

        this.suction.position.set(totalLen / 2, totalLen / 2, params.zSafe);
        this.scene.add(this.suction);

        this._fitCameraToBoard();
        this._buildTimeline();

        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
        this.attachedTile = null;
        this._slidingTiles = [];
        this._setSuctionOn(false);
        this._renderNeeded = true;
    }

    // -----------------------------------------------------------------
    //  TIMELINE — Mode Bulldozer avec tuiles poussees synchronisees
    // -----------------------------------------------------------------
    _buildTimeline() {
        const p = this.params;
        const tl = [];
        const tileSize = p.tileSize;
        const totalLen = p.size * tileSize;

        let cx = totalLen / 2;
        let cy = totalLen / 2;
        let cz = p.zSafe;

        const D_RAPID_XY = 0.55;
        const D_SLIDE_XY = 0.65;
        const D_DOWN     = 0.40;
        const D_UP       = 0.40;
        const D_PAUSE    = 0.18;

        const chains    = p.chains    || [];
        const moves     = p.moves    || [];
        const pushedMap = p.pushedTilesMap || {};

        let moveIdx = 0;
        for (let ci = 0; ci < chains.length; ci++) {
            const chain = chains[ci];
            const isMultiMove = chain.end > chain.start;
            const chainMoveCount = chain.end - chain.start + 1;

            if (isMultiMove) {
                // === CHAINE BULLDOZER ===
                const firstMove = moves[moveIdx];
                const lastMoveIdx = moveIdx + (chain.end - chain.start);
                const lastMove = moves[lastMoveIdx];
                if (firstMove && lastMove) {
                    const xStart = lastMove.xA;
                    const yStart = lastMove.yA;
                    const xEnd = lastMove.xB;
                    const yEnd = lastMove.yB;

                    // 1) XY rapide au-dessus de la tuile la plus eloignee
                    tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                              x0: cx, y0: cy, z0: cz,
                              x1: xStart, y1: yStart, z1: p.zSafe });
                    cx = xStart; cy = yStart; cz = p.zSafe;

                    // 2) Descente
                    tl.push({ type: 'moveSuction', dur: D_DOWN,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zPick });
                    cz = p.zPick;

                    // 3) Grab tuile la plus eloignee
                    tl.push({ type: 'grab', dur: D_PAUSE, tileValue: lastMove.tileValue });

                    // 4) Poussee Bulldozer : ventouse + tuiles poussees en parallele
                    const dist = Math.abs(xEnd - cx) + Math.abs(yEnd - cy);
                    const bulldozeDur = D_SLIDE_XY * Math.max(1, dist / p.tileSize);
                    const pushed = pushedMap[moveIdx] || [];
                    tl.push({ type: 'moveSuction', dur: bulldozeDur,
                              x0: cx, y0: cy, z0: cz,
                              x1: xEnd, y1: yEnd, z1: p.zPick,
                              pushedTiles: pushed });
                    cx = xEnd; cy = yEnd; cz = p.zPick;

                    // 5) Release
                    tl.push({ type: 'release', dur: D_PAUSE });

                    // 6) Remontee Z safe
                    tl.push({ type: 'moveSuction', dur: D_UP,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zSafe });
                    cz = p.zSafe;
                }
                moveIdx = lastMoveIdx + 1;
            } else {
                // === MOUVEMENT ISOLE : Pick & Place classique ===
                const m = moves[moveIdx];
                if (m) {
                    tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                              x0: cx, y0: cy, z0: cz,
                              x1: m.xA, y1: m.yA, z1: p.zSafe });
                    cx = m.xA; cy = m.yA; cz = p.zSafe;

                    tl.push({ type: 'moveSuction', dur: D_DOWN,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zPick });
                    cz = p.zPick;

                    tl.push({ type: 'grab', dur: D_PAUSE, tileValue: m.tileValue });

                    tl.push({ type: 'moveSuction', dur: D_UP,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zSafe });
                    cz = p.zSafe;

                    tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                              x0: cx, y0: cy, z0: cz,
                              x1: m.xB, y1: m.yB, z1: p.zSafe });
                    cx = m.xB; cy = m.yB; cz = p.zSafe;

                    tl.push({ type: 'moveSuction', dur: D_DOWN,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zPick });
                    cz = p.zPick;

                    tl.push({ type: 'release', dur: D_PAUSE });

                    tl.push({ type: 'moveSuction', dur: D_UP,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: p.zSafe });
                    cz = p.zSafe;
                }
                moveIdx++;
            }
        }

        tl.push({ type: 'done', dur: 0.001 });
        this._timeline = tl;
        this._totalDur = tl.reduce((s, st) => s + st.dur, 0);
    }

    // -----------------------------------------------------------------
    //  TICK ANIMATION — avec interpolation des tuiles poussees
    // -----------------------------------------------------------------
    _tickAnimation(dt) {
        if (!this._timeline || this._stepIdx >= this._timeline.length) return;
        let step = this._timeline[this._stepIdx];
        this._stepTime += dt;

        while (step && this._stepTime >= step.dur) {
            this._completeStep(step);
            this._stepTime -= step.dur;
            this._stepIdx++;
            step = this._timeline[this._stepIdx];
            if (!step) break;
            this._beginStep(step);
        }
        if (!step) {
            this._playing = false;
            this._notifyProgress();
            return;
        }

        // Interpolation ventouse
        if (step.type === 'moveSuction') {
            const u = step.dur > 0 ? Math.min(1, this._stepTime / step.dur) : 1;
            this._setSuctionPos(
                step.x0 + (step.x1 - step.x0) * u,
                step.y0 + (step.y1 - step.y0) * u,
                step.z0 + (step.z1 - step.z0) * u
            );
            // === INTERPOLATION DES TUILES POUSSEES EN PARALLELE ===
            if (step.pushedTiles && step.pushedTiles.length > 0) {
                for (const pt of step.pushedTiles) {
                    const mesh = this.tileMeshes[pt.tileValue];
                    if (mesh && mesh !== this.attachedTile) {
                        mesh.position.x = pt.fromX + (pt.toX - pt.fromX) * u;
                        mesh.position.y = pt.fromY + (pt.toY - pt.fromY) * u;
                    }
                }
            }
        }
        this._notifyProgress();
    }

    _beginStep(step) {
        // Rien de special
    }

    _completeStep(step) {
        if (step.type === 'moveSuction') {
            this._setSuctionPos(step.x1, step.y1, step.z1);
            // Finaliser les positions des tuiles poussees
            if (step.pushedTiles && step.pushedTiles.length > 0) {
                for (const pt of step.pushedTiles) {
                    const mesh = this.tileMeshes[pt.tileValue];
                    if (mesh && mesh !== this.attachedTile) {
                        mesh.position.x = pt.toX;
                        mesh.position.y = pt.toY;
                    }
                }
            }
        } else if (step.type === 'grab') {
            this._setSuctionOn(true);
            const tile = this.tileMeshes[step.tileValue];
            if (tile && tile !== this.attachedTile) {
                this.scene.remove(tile);
                tile.position.set(0, 0, -TILE_THICKNESS / 2 - 0.1);
                this.suction.add(tile);
                this.attachedTile = tile;
            }
        } else if (step.type === 'release') {
            this._setSuctionOn(false);
            if (this.attachedTile) {
                const t = this.attachedTile;
                const wp = new THREE.Vector3();
                t.getWorldPosition(wp);
                this.suction.remove(t);
                this.scene.add(t);
                t.position.copy(wp);
                this.attachedTile = null;
            }
        } else if (step.type === 'done') {
            this._playing = false;
        }
    }

    _setSuctionPos(x, y, z) {
        if (!this.suction) return;
        this.suction.position.set(x, y, z);
    }

    _setSuctionOn(on) {
        if (!this.suctionPad) return;
        this.suctionPad.material.color.setHex(on ? COLOR_SUCTION_PAD_ON : COLOR_SUCTION_PAD_OFF);
    }

    // -----------------------------------------------------------------
    //  CONTROLES PUBLICS (identiques)
    // -----------------------------------------------------------------
    play() {
        if (!this._timeline) return;
        if (this._stepIdx >= this._timeline.length) this.reset();
        this._playing = true;
        this._renderNeeded = true;
    }

    pause() { this._playing = false; }

    reset() {
        if (!this.params) return;
        if (this.attachedTile) {
            const t = this.attachedTile;
            this.suction.remove(t);
            this.scene.add(t);
            this.attachedTile = null;
        }
        const { size, tileSize, initialBoard } = this.params;
        for (let i = 0; i < size * size; i++) {
            const v = initialBoard[i];
            if (v === 0) continue;
            const t = this.tileMeshes[v];
            if (!t) continue;
            const r = (i / size) | 0;
            const c = i % size;
            t.position.set(
                c * tileSize + tileSize / 2,
                (size - 1 - r) * tileSize + tileSize / 2,
                TILE_THICKNESS / 2
            );
        }
        const totalLen = size * tileSize;
        this._setSuctionPos(totalLen / 2, totalLen / 2, this.params.zSafe);
        this._setSuctionOn(false);
        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
        this._slidingTiles = [];
        this._renderNeeded = true;
        this._notifyProgress();
    }

    setSpeed(s) { this._speed = Math.max(0.1, Math.min(5, s)); }

    setView(mode) {
        if (!this.params) return;
        const { size, tileSize } = this.params;
        const total = size * tileSize;
        const cx = total / 2, cy = total / 2;
        if (mode === 'top') {
            this.camera.position.set(cx, cy, total * 1.7);
            this.controls.target.set(cx, cy, 0);
        } else {
            this.camera.position.set(cx + total * 0.95, cy - total * 1.15, total * 1.05);
            this.controls.target.set(cx, cy, 0);
        }
        this.camera.up.set(0, 0, 1);
        this.controls.update();
        this._renderNeeded = true;
    }

    _fitCameraToBoard() { this.setView('iso'); }

    _notifyProgress() {
        if (typeof this.onProgress !== 'function') return;
        let elapsed = 0;
        for (let i = 0; i < this._stepIdx; i++) elapsed += this._timeline[i].dur;
        if (this._stepIdx < this._timeline.length) elapsed += this._stepTime;
        const pct = this._totalDur > 0 ? Math.min(1, elapsed / this._totalDur) : 1;
        const moveCount = (this.params && this.params.moves) ? this.params.moves.length : 0;
        this.onProgress({ pct, moveIdx: this._stepIdx, moveTotal: moveCount });
    }

    // -----------------------------------------------------------------
    //  NETTOYAGE
    // -----------------------------------------------------------------
    _clearWorld() {
        for (const k in this.tileMeshes) {
            const t = this.tileMeshes[k];
            if (t.parent) t.parent.remove(t);
            if (Array.isArray(t.material)) t.material.forEach(m => { if (m.map) m.map.dispose(); m.dispose && m.dispose(); });
            else if (t.material) { if (t.material.map) t.material.map.dispose(); t.material.dispose(); }
            t.geometry && t.geometry.dispose();
        }
        this.tileMeshes = {};
        if (this.suction) {
            this.scene.remove(this.suction);
            this.suction.traverse(o => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
                    else o.material.dispose();
                }
            });
            this.suction = null;
        }
        if (this.boardGroup) {
            this.scene.remove(this.boardGroup);
            this.boardGroup.traverse(o => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
                    else o.material.dispose();
                }
            });
            this.boardGroup = null;
        }
        this.attachedTile = null;
    }

    dispose() {
        if (this._raf) cancelAnimationFrame(this._raf);
        this._raf = null;
        this._playing = false;
        if (this._onResize) window.removeEventListener('resize', this._onResize);
        this._clearWorld();
        if (this.controls) this.controls.dispose();
        if (this.renderer) {
            this.renderer.dispose();
            if (this.renderer.domElement && this.renderer.domElement.parentNode) {
                this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
            }
        }
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
    }
}

// =====================================================================
//  Point d'entree global — mode Humain
// =====================================================================
async function openTaquinHumainVisualizer(params, modalId, canvasId) {
    const modal = document.getElementById(modalId);
    if (!modal) return null;
    modal.classList.add('active');

    await new Promise(r => setTimeout(r, 50));

    const viz = new TaquinHumainVisualizer3D(canvasId);
    await viz.init();
    viz.loadPuzzle(params);
    modal._taquinViz = viz;

    const btnPlay  = document.getElementById('tqVizPlay');
    const btnPause = document.getElementById('tqVizPause');
    const btnReset = document.getElementById('tqVizReset');
    const btnIso   = document.getElementById('tqVizIso');
    const btnTop   = document.getElementById('tqVizTop');
    const speed    = document.getElementById('tqVizSpeed');
    const speedVal = document.getElementById('tqVizSpeedVal');
    const close    = document.getElementById('tqVizClose');
    const closeX   = document.getElementById('tqVizCloseX');
    const progress = document.getElementById('tqVizProgress');
    const moveLbl  = document.getElementById('tqVizMove');

    function refreshPlayBtn() {
        if (!btnPlay) return;
        if (viz._playing) btnPlay.classList.add('active');
        else              btnPlay.classList.remove('active');
    }

    if (btnPlay)  btnPlay.onclick  = () => { viz.play();  refreshPlayBtn(); };
    if (btnPause) btnPause.onclick = () => { viz.pause(); refreshPlayBtn(); };
    if (btnReset) btnReset.onclick = () => { viz.reset(); refreshPlayBtn(); };
    if (btnIso)   btnIso.onclick   = () => viz.setView('iso');
    if (btnTop)   btnTop.onclick   = () => viz.setView('top');
    if (speed) {
        speed.value = '1';
        const updateSpeed = () => {
            const v = parseFloat(speed.value) || 1;
            viz.setSpeed(v);
            if (speedVal) speedVal.textContent = v.toFixed(2) + 'x';
        };
        speed.oninput = updateSpeed;
        updateSpeed();
    }

    viz.onProgress = ({ pct, moveIdx, moveTotal }) => {
        if (progress) progress.style.width = (pct * 100).toFixed(1) + '%';
        if (moveLbl)  moveLbl.textContent  = moveIdx + ' / ' + moveTotal;
        refreshPlayBtn();
    };

    setTimeout(() => { viz.play(); refreshPlayBtn(); }, 100);

    const closeFn = () => {
        viz.dispose();
        modal.classList.remove('active');
        modal._taquinViz = null;
    };
    if (close)  close.onclick  = closeFn;
    if (closeX) closeX.onclick = closeFn;
    const onBg = (e) => {
        if (e.target === modal) {
            closeFn();
            modal.removeEventListener('click', onBg);
        }
    };
    modal.addEventListener('click', onBg);

    return viz;
}

window.TaquinHumainVisualizer3D    = TaquinHumainVisualizer3D;
window.openTaquinHumainVisualizer  = openTaquinHumainVisualizer;
if (window._onTaquinHumainVizReady) window._onTaquinHumainVizReady();
