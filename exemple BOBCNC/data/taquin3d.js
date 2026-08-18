// =====================================================================
//  taquin3d.js — Simulation 3D dédiée au jeu Taquin (BOBCNC)
// =====================================================================
//
//  Visualise sur un canvas WebGL :
//    1. Le plateau de jeu SIZE×SIZE (cadre + cases creuses)
//    2. Les tuiles numérotées (cubes plats avec numéro sur le dessus)
//    3. La ventouse Ø 10 mm (tube + disque) qui réalise les pick & place
//    4. L'animation chronologique de chaque mouvement de la solution :
//         — Z safe au-dessus de la tuile A
//         — Descente Z prise (M3 = succion ON)
//         — Remontée Z safe (la tuile suit la ventouse)
//         — Translation XY vers la case vide
//         — Descente Z prise (M5 = succion OFF)
//         — Remontée Z safe
//
//  API publique exposée sur window :
//    openTaquinVisualizer(params, modalId, canvasId)
//      params = {
//         size:         4 | 5 | 6,
//         tileSize:     mm (cote case),
//         zSafe:        mm,
//         zPick:        mm,
//         initialBoard: int[size^2]   (etat AVANT resolution),
//         moves: [
//           { tileValue, fromR, fromC, toR, toC, xA, yA, xB, yB }, ...
//         ],
//         solveMode:    'pickplace' | 'slide',
//         chains:       [{ dir, start, end }, ...]  (mode slide uniquement)
//      }
//
//  Le module est entièrement autonome : il charge ses propres lumières,
//  caméras, contrôles d'orbite, et ressources (textures canvas pour les
//  numéros). Aucun couplage avec visualizer.js.
// =====================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------------------------------------------------------------------
//  Couleurs / matériaux
// ---------------------------------------------------------------------
const COLOR_BG          = 0x101418;
const COLOR_BOARD       = 0x2a2f3a;
const COLOR_BOARD_FRAME = 0x44505f;
const COLOR_TILE_TOP    = 0xffe9b0;   // bois clair
const COLOR_TILE_SIDE   = 0xc8a06a;
const COLOR_SUCTION_TUBE = 0x666c75;
const COLOR_SUCTION_PAD_OFF = 0x884444;
const COLOR_SUCTION_PAD_ON  = 0x33ff66;

// Dimensions ventouse (en mm)
const SUCTION_DIAMETER  = 10;   // Ø10 mm comme demandé par l'utilisateur
const SUCTION_TUBE_LEN  = 28;
const SUCTION_PAD_DIAM  = 12;
const SUCTION_PAD_THICK = 2;

// Épaisseur des tuiles (mm)
const TILE_THICKNESS = 4;

// ---------------------------------------------------------------------
//  Génère une texture canvas pour le numéro d'une tuile.
// ---------------------------------------------------------------------
function makeNumberTexture(num) {
    const size = 256;
    const c = document.createElement('canvas');
    c.width = size; c.height = size;
    const ctx = c.getContext('2d');
    // Fond clair
    ctx.fillStyle = '#ffe9b0';
    ctx.fillRect(0, 0, size, size);
    // Bordure
    ctx.strokeStyle = '#a0793a';
    ctx.lineWidth = 8;
    ctx.strokeRect(4, 4, size - 8, size - 8);
    // Numéro
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
//  Classe principale
// =====================================================================
class TaquinVisualizer3D {
    constructor(canvasId) {
        this.canvasId = canvasId;
        this.container = null;

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // Maillages
        this.boardGroup = null;
        this.tileMeshes = {};      // tileValue -> mesh
        this.suction = null;       // Group {tube, pad}
        this.suctionPad = null;
        this.attachedTile = null;  // mesh actuellement "collé" à la ventouse

        // Animation
        this._raf = null;
        this._timeline = null;     // séquence interne (étapes simples)
        this._stepIdx = 0;
        this._stepTime = 0;        // temps écoulé dans l'étape courante (s)
        this._lastT = 0;
        this._playing = false;
        this._speed = 1.0;

        // Paramètres puzzle
        this.params = null;

        // Resize
        this._onResize = null;
    }

    // -----------------------------------------------------------------
    //  INIT
    // -----------------------------------------------------------------
    async init() {
        this.container = document.getElementById(this.canvasId);
        if (!this.container) throw new Error('canvas container introuvable: ' + this.canvasId);

        const w = this.container.clientWidth || 800;
        const h = this.container.clientHeight || 500;

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(COLOR_BG);

        // Camera (perspective)
        this.camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 5000);
        this.camera.up.set(0, 0, 1);   // Z = up (convention CNC)
        this.camera.position.set(150, -180, 180);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
        // pixelRatio limité à 1 sur mobile pour gagner en perf (89 ms → ~10 ms)
        const pr = (window.matchMedia && window.matchMedia('(max-width:500px)').matches) ? 1 : Math.min(window.devicePixelRatio, 1.5);
        this.renderer.setPixelRatio(pr);
        this.renderer.setSize(w, h);
        // Ombres désactivées : rendu éclair, gain massif (était la cause du Violation 89ms)
        this.renderer.shadowMap.enabled = false;
        this.container.appendChild(this.renderer.domElement);

        // Controls (orbit)
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0, 0, 0);
        // Événements de la caméra → déclenche un rendu à la demande
        this.controls.addEventListener('change', () => { this._renderNeeded = true; });

        // Lights (sans ombres)
        const ambient = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambient);
        const dir = new THREE.DirectionalLight(0xffffff, 0.7);
        dir.position.set(120, -150, 250);
        this.scene.add(dir);

        // Sol (grille de référence) — discret
        const grid = new THREE.GridHelper(400, 20, 0x223040, 0x1a2028);
        grid.rotation.x = Math.PI / 2;       // pour aligner sur XY (Z up)
        grid.position.z = -0.01;
        this.scene.add(grid);

        // Resize handler
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

        // Premier rendu
        this._renderNeeded = true;

        // Boucle render — RENDU À LA DEMANDE :
        // on ne fait des cycles caméra/render que si _playing OU _renderNeeded.
        // Quand l'animation est en pause et que la caméra ne bouge pas, le GPU dort.
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

            // Damping de OrbitControls : ne consomme du CPU que si actif.
            // controls.update() retourne true s'il a changé quelque chose en mode damping.
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
    //  CHARGEMENT D'UN PUZZLE
    // -----------------------------------------------------------------
    /**
     * @param {Object} params
     *   - size, tileSize, zSafe, zPick
     *   - initialBoard: int[size²]
     *   - moves: array of pick & place steps
     */
    loadPuzzle(params) {
        this.params = params;
        this._clearWorld();

        const { size, tileSize } = params;
        const totalLen = size * tileSize;

        // ---- Plateau ----
        this.boardGroup = new THREE.Group();
        this.scene.add(this.boardGroup);

        const baseThick = 3;
        const frameW = 4;
        const baseGeom = new THREE.BoxGeometry(totalLen + 2 * frameW, totalLen + 2 * frameW, baseThick);
        const baseMat = new THREE.MeshStandardMaterial({ color: COLOR_BOARD_FRAME, roughness: 0.85 });
        const base = new THREE.Mesh(baseGeom, baseMat);
        base.position.set(totalLen / 2, totalLen / 2, -baseThick / 2);
        this.boardGroup.add(base);

        // Cases creuses (un creux légèrement plus grand)
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

        // Repère origine (petit cube vert au coin sud-ouest = (0,0,0))
        const orig = new THREE.Mesh(
            new THREE.BoxGeometry(2, 2, 2),
            new THREE.MeshBasicMaterial({ color: 0x00ff88 })
        );
        orig.position.set(0, 0, 0);
        this.boardGroup.add(orig);

        // ---- Tuiles ----
        const tileGeom = new THREE.BoxGeometry(tileSize - 1, tileSize - 1, TILE_THICKNESS);
        const sideMat = new THREE.MeshStandardMaterial({ color: COLOR_TILE_SIDE, roughness: 0.7 });

        for (let i = 0; i < size * size; i++) {
            const v = params.initialBoard[i];
            if (v === 0) continue;
            const r = (i / size) | 0;
            const c = i % size;

            // Matériaux : 4 côtés en bois, dessous neutre, dessus avec numéro
            const topMat = new THREE.MeshStandardMaterial({
                map: makeNumberTexture(v),
                roughness: 0.55
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

        // ---- Ventouse Ø10 mm ----
        this.suction = new THREE.Group();

        // Tube principal (cylindre vertical Ø10 mm, hauteur SUCTION_TUBE_LEN)
        const tubeGeom = new THREE.CylinderGeometry(
            SUCTION_DIAMETER / 2, SUCTION_DIAMETER / 2,
            SUCTION_TUBE_LEN, 16
        );
        // Cylindre par défaut axe Y → on le bascule sur Z
        tubeGeom.rotateX(Math.PI / 2);
        const tubeMat = new THREE.MeshStandardMaterial({
            color: COLOR_SUCTION_TUBE, metalness: 0.55, roughness: 0.4
        });
        const tube = new THREE.Mesh(tubeGeom, tubeMat);
        tube.position.set(0, 0, SUCTION_PAD_THICK + SUCTION_TUBE_LEN / 2);
        this.suction.add(tube);

        // Pad caoutchouc (Ø12 mm épais 2 mm) — change de couleur quand succion ON
        const padGeom = new THREE.CylinderGeometry(
            SUCTION_PAD_DIAM / 2, SUCTION_PAD_DIAM / 2,
            SUCTION_PAD_THICK, 16
        );
        padGeom.rotateX(Math.PI / 2);
        const padMat = new THREE.MeshStandardMaterial({
            color: COLOR_SUCTION_PAD_OFF, roughness: 0.5
        });
        this.suctionPad = new THREE.Mesh(padGeom, padMat);
        this.suctionPad.position.set(0, 0, SUCTION_PAD_THICK / 2);
        this.suction.add(this.suctionPad);

        // Position initiale = au-dessus du centre du plateau, à zSafe
        this.suction.position.set(totalLen / 2, totalLen / 2, params.zSafe);
        this.scene.add(this.suction);

        // Recadrer la caméra sur le plateau
        this._fitCameraToBoard();

        // Construire la timeline d'animation
        this._buildTimeline();

        // Reset état d'animation
        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
        this.attachedTile = null;
        this._setSuctionOn(false);
        this._renderNeeded = true;
    }

    // -----------------------------------------------------------------
    //  TIMELINE — séquence d'étapes simples interpolées par _tickAnimation
    // -----------------------------------------------------------------
    //  Chaque étape = { type, dur, ...payload }
    //    type 'moveSuction' : interpolation linéaire (x0,y0,z0 → x1,y1,z1)
    //    type 'grab'        : tuile devient enfant ventouse, pad ON
    //    type 'release'     : tuile détachée, pad OFF
    //    type 'wait'        : pause (P)
    //    type 'done'        : marqueur fin
    // -----------------------------------------------------------------
    _buildTimeline() {
        const p = this.params;
        const tl = [];

        const tileSize = p.tileSize;
        const totalLen = p.size * tileSize;

        // Position de depart ventouse
        let cx = totalLen / 2;
        let cy = totalLen / 2;
        let cz = p.zSafe;

        // Durees (s) a vitesse 1.0
        const D_RAPID_XY = 0.55;
        const D_SLIDE_XY = 0.65;  // Glissement XY a Z prise (un peu plus lent)
        const D_DOWN     = 0.40;
        const D_UP       = 0.40;
        const D_PAUSE    = 0.18;

        const solveMode = p.solveMode || 'pickplace';
        const chains    = p.chains    || [];
        const moves     = p.moves    || [];

        if (solveMode === 'slide' && chains.length > 0) {
            // ---- MODE GLISSEMENT : chaines de mouvements optimisees ----
            let moveIdx = 0; // index global dans moves[]
            for (let ci = 0; ci < chains.length; ci++) {
                const chain = chains[ci];
                const isMultiMove = chain.end > chain.start;

                if (isMultiMove) {
                    // Chaine multi-mouvements : mode BULLDOZER
                    // Une seule prise sur la tuile la plus eloignee,
                    // une seule poussee jusqu'a la case vide initiale.
                    const firstMove = moves[moveIdx];
                    const lastMoveIdx = moveIdx + (chain.end - chain.start);
                    const lastMove = moves[lastMoveIdx];
                    if (firstMove && lastMove) {
                        // DEPART = position de la tuile la plus eloignee (dernier move de la chaine)
                        const xStart = lastMove.xA;
                        const yStart = lastMove.yA;
                        // ARRIVEE = depart + un seul entraxe (xB/yB du dernier move)
                        const xEnd = lastMove.xB;
                        const yEnd = lastMove.yB;

                        // 1) XY rapide au-dessus de la tuile la plus eloignee
                        tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: xStart, y1: yStart, z1: p.zSafe });
                        cx = xStart; cy = yStart; cz = p.zSafe;

                        // 2) Descente sur la tuile
                        tl.push({ type: 'moveSuction', dur: D_DOWN,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: p.zPick });
                        cz = p.zPick;

                        // 3) Grab la tuile la plus eloignee
                        tl.push({ type: 'grab', dur: D_PAUSE, tileValue: lastMove.tileValue });

                        // 4) Poussee Bulldozer : glisser directement jusqu'a la case vide
                        //    (mouvement plus long, duree proportionnelle)
                        const dist = Math.abs(xEnd - cx) + Math.abs(yEnd - cy);
                        const bulldozeDur = D_SLIDE_XY * Math.max(1, dist / p.tileSize);
                        tl.push({ type: 'moveSuction', dur: bulldozeDur,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: xEnd, y1: yEnd, z1: p.zPick });
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
                    // Mouvement isole : Pick & Place classique
                    const m = moves[moveIdx];
                    if (m) {
                        // 1) XY rapide vers tuile A
                        tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: m.xA, y1: m.yA, z1: p.zSafe });
                        cx = m.xA; cy = m.yA; cz = p.zSafe;

                        // 2) Descente sur tuile A
                        tl.push({ type: 'moveSuction', dur: D_DOWN,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: p.zPick });
                        cz = p.zPick;

                        // 3) Grab
                        tl.push({ type: 'grab', dur: D_PAUSE, tileValue: m.tileValue });

                        // 4) Remontee Z safe
                        tl.push({ type: 'moveSuction', dur: D_UP,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: p.zSafe });
                        cz = p.zSafe;

                        // 5) XY rapide vers case vide B
                        tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: m.xB, y1: m.yB, z1: p.zSafe });
                        cx = m.xB; cy = m.yB; cz = p.zSafe;

                        // 6) Descente
                        tl.push({ type: 'moveSuction', dur: D_DOWN,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: p.zPick });
                        cz = p.zPick;

                        // 7) Release
                        tl.push({ type: 'release', dur: D_PAUSE });

                        // 8) Remontee
                        tl.push({ type: 'moveSuction', dur: D_UP,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: p.zSafe });
                        cz = p.zSafe;
                    }
                    moveIdx++;
                }
            }
        } else {
            // ---- MODE PICK & PLACE (comportement classique) ----
            for (const m of moves) {
                // 1) XY rapide vers tuile A (au-dessus)
                tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                          x0: cx, y0: cy, z0: cz,
                          x1: m.xA, y1: m.yA, z1: p.zSafe });
                cx = m.xA; cy = m.yA; cz = p.zSafe;

                // 2) Descente sur tuile A
                tl.push({ type: 'moveSuction', dur: D_DOWN,
                          x0: cx, y0: cy, z0: cz,
                          x1: cx, y1: cy, z1: p.zPick });
                cz = p.zPick;

                // 3) Grab (M3)
                tl.push({ type: 'grab', dur: D_PAUSE, tileValue: m.tileValue });

                // 4) Remontee Z safe
                tl.push({ type: 'moveSuction', dur: D_UP,
                          x0: cx, y0: cy, z0: cz,
                          x1: cx, y1: cy, z1: p.zSafe });
                cz = p.zSafe;

                // 5) XY rapide vers case vide B
                tl.push({ type: 'moveSuction', dur: D_RAPID_XY,
                          x0: cx, y0: cy, z0: cz,
                          x1: m.xB, y1: m.yB, z1: p.zSafe });
                cx = m.xB; cy = m.yB; cz = p.zSafe;

                // 6) Descente
                tl.push({ type: 'moveSuction', dur: D_DOWN,
                          x0: cx, y0: cy, z0: cz,
                          x1: cx, y1: cy, z1: p.zPick });
                cz = p.zPick;

                // 7) Release (M5)
                tl.push({ type: 'release', dur: D_PAUSE });

                // 8) Remontee
                tl.push({ type: 'moveSuction', dur: D_UP,
                          x0: cx, y0: cy, z0: cz,
                          x1: cx, y1: cy, z1: p.zSafe });
                cz = p.zSafe;
            }
        }

        tl.push({ type: 'done', dur: 0.001 });
        this._timeline = tl;
        this._totalDur = tl.reduce((s, st) => s + st.dur, 0);
    }

    // -----------------------------------------------------------------
    //  TICK animation — appelée à chaque frame quand _playing = true.
    // -----------------------------------------------------------------
    _tickAnimation(dt) {
        if (!this._timeline || this._stepIdx >= this._timeline.length) return;
        let step = this._timeline[this._stepIdx];
        this._stepTime += dt;

        // Avancer les étapes "instantanées" successives
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

        // Interpolation pour les étapes de type moveSuction
        if (step.type === 'moveSuction') {
            const u = step.dur > 0 ? Math.min(1, this._stepTime / step.dur) : 1;
            this._setSuctionPos(
                step.x0 + (step.x1 - step.x0) * u,
                step.y0 + (step.y1 - step.y0) * u,
                step.z0 + (step.z1 - step.z0) * u
            );
        }
        this._notifyProgress();
    }

    _beginStep(step) {
        // Hook pour effets visuels d'entrée (rien à faire pour move)
    }

    _completeStep(step) {
        if (step.type === 'moveSuction') {
            this._setSuctionPos(step.x1, step.y1, step.z1);
        } else if (step.type === 'grab') {
            this._setSuctionOn(true);
            const tile = this.tileMeshes[step.tileValue];
            if (tile && tile !== this.attachedTile) {
                // Reparenter tile au groupe ventouse en conservant la position monde
                this.scene.remove(tile);
                tile.position.set(0, 0, -TILE_THICKNESS / 2 - 0.1);
                this.suction.add(tile);
                this.attachedTile = tile;
            }
        } else if (step.type === 'release') {
            this._setSuctionOn(false);
            if (this.attachedTile) {
                const t = this.attachedTile;
                // Calculer la position monde actuelle puis détacher
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
    //  CONTRÔLES PUBLICS
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
        // Détacher la tuile collée le cas échéant
        if (this.attachedTile) {
            const t = this.attachedTile;
            this.suction.remove(t);
            this.scene.add(t);
            this.attachedTile = null;
        }
        // Replacer toutes les tuiles à leur position initiale
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
        // Repositionner la ventouse au centre, à zSafe
        const totalLen = size * tileSize;
        this._setSuctionPos(totalLen / 2, totalLen / 2, this.params.zSafe);
        this._setSuctionOn(false);
        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
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
        } else { // iso
            this.camera.position.set(cx + total * 0.95, cy - total * 1.15, total * 1.05);
            this.controls.target.set(cx, cy, 0);
        }
        this.camera.up.set(0, 0, 1);
        this.controls.update();
        this._renderNeeded = true;
    }

    _fitCameraToBoard() {
        this.setView('iso');
    }

    // Pourcentage et message de progression — exposé via callback
    _notifyProgress() {
        if (typeof this.onProgress !== 'function') return;
        // Calcul simple : (étapes complètes + fraction de l'étape courante) / total
        let elapsed = 0;
        for (let i = 0; i < this._stepIdx; i++) elapsed += this._timeline[i].dur;
        if (this._stepIdx < this._timeline.length) elapsed += this._stepTime;
        const pct = this._totalDur > 0 ? Math.min(1, elapsed / this._totalDur) : 1;
        const moveCount = (this.params && this.params.moves) ? this.params.moves.length : 0;
        // Étapes par mouvement = 8 (move XY, down, grab, up, move XY, down, release, up)
        const STEPS_PER_MOVE = 8;
        const mIdx = Math.min(moveCount, Math.floor(this._stepIdx / STEPS_PER_MOVE) + 1);
        this.onProgress({ pct, moveIdx: mIdx, moveTotal: moveCount });
    }

    // -----------------------------------------------------------------
    //  NETTOYAGE
    // -----------------------------------------------------------------
    _clearWorld() {
        // Retire tuiles + ventouse + plateau de la scène
        const toRemove = [];
        this.scene.traverse(obj => {
            if (obj.userData && obj.userData._taquinNonRemovable) return;
            if (obj.isMesh || obj.isGroup) {
                if (obj === this.scene) return;
                if (obj.parent === this.scene) toRemove.push(obj);
            }
        });
        // On garde lumières et grille → on retire seulement ce qu'on a créé
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
//  Point d'entrée global — appelé depuis taquin.html
// =====================================================================
async function openTaquinVisualizer(params, modalId, canvasId) {
    const modal = document.getElementById(modalId);
    if (!modal) return null;
    modal.classList.add('active');

    // Attendre que le modal soit visible (taille du canvas)
    await new Promise(r => setTimeout(r, 50));

    const viz = new TaquinVisualizer3D(canvasId);
    await viz.init();
    viz.loadPuzzle(params);
    modal._taquinViz = viz;

    // Wiring boutons (s'ils existent dans le DOM)
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

    // Indicateur visuel «en lecture» : btn play actif quand _playing
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
        // Si l'animation s'est arrêtée (fin atteinte), rafraîchir le bouton play
        refreshPlayBtn();
    };

    // → Auto-play dès l'ouverture pour que l'utilisateur voit immédiatement la simulation
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

window.TaquinVisualizer3D    = TaquinVisualizer3D;
window.openTaquinVisualizer  = openTaquinVisualizer;
if (window._onTaquinVizReady) window._onTaquinVizReady();
