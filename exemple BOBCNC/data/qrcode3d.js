// =====================================================================
//  qrcode3d.js — Simulation 3D dédiée au perçage QR Code (BOBCNC)
// =====================================================================
//
//  Visualise sur un canvas WebGL :
//    1. La plaque de matériau (bois clair) à percer
//    2. Les positions cibles des trous (matrice QR)
//    3. Le foret cylindrique au DIAMETRE DE L'OUTIL sélectionné
//       (corps + pointe conique)
//    4. L'animation chronologique de chaque perçage :
//         — Déplacement rapide XY vers le trou (à zSafe)
//         — Descente G1 dans la matière (jusqu'à -depth)
//         — Si débourrage : passes successives avec remontée
//         — Remontée G0 vers zSafe
//
//  API publique exposée sur window :
//    openQRCodeVisualizer(params, modalId, canvasId)
//      params = {
//         sizeMM:        mm (côté du QR Code),
//         depth:         mm (profondeur perçage),
//         zSafe:         mm,
//         peckDepth:     mm,
//         usePeck:       bool,
//         retractTotal:  bool,
//         toolDiameter:  mm (diamètre du foret = TIGE),
//         holes:         [{ x, y }, ...]   (positions XY en mm, repère pièce)
//      }
//
//  Inspiré de taquin3d.js — entièrement autonome.
// =====================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------------------------------------------------------------------
//  Couleurs / matériaux
// ---------------------------------------------------------------------
const COLOR_BG           = 0x101418;
const COLOR_PLATE        = 0xc8a06a;   // bois clair
const COLOR_PLATE_EDGE   = 0x8a6a3a;
const COLOR_HOLE_PENDING = 0x2a3040;   // trou pas encore percé
const COLOR_HOLE_DONE    = 0x101418;   // trou percé (sombre)
const COLOR_DRILL_BODY   = 0xd6d6e0;   // acier
const COLOR_DRILL_TIP    = 0xa0a0aa;
const COLOR_DRILL_CUTTING = 0xff6633;  // pointe rouge quand elle perce

// Épaisseur visuelle de la plaque (mm)
const PLATE_THICKNESS = 6;
// Marge autour du QR sur la plaque
const PLATE_MARGIN    = 6;
// Longueur totale du foret (corps + pointe), en mm
const DRILL_TOTAL_LEN = 38;

// =====================================================================
//  Classe principale
// =====================================================================
class QRCodeVisualizer3D {
    constructor(canvasId) {
        this.canvasId = canvasId;
        this.container = null;

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // Maillages
        this.plateGroup = null;
        this.holeMeshes = [];      // {mesh, done}
        this.drill = null;         // Group {body, tip}
        this.drillBody = null;
        this.drillTip = null;

        // Animation
        this._raf = null;
        this._timeline = null;
        this._stepIdx = 0;
        this._stepTime = 0;
        this._lastT = 0;
        this._playing = false;
        this._speed = 1.0;
        this._renderNeeded = true;

        this.params = null;
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

        // Camera (perspective) — Z up (CNC)
        this.camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 5000);
        this.camera.up.set(0, 0, 1);
        this.camera.position.set(80, -120, 100);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
        const pr = (window.matchMedia && window.matchMedia('(max-width:500px)').matches) ? 1 : Math.min(window.devicePixelRatio, 1.5);
        this.renderer.setPixelRatio(pr);
        this.renderer.setSize(w, h);
        this.renderer.shadowMap.enabled = false;
        this.container.appendChild(this.renderer.domElement);

        // OrbitControls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0, 0, 0);
        this.controls.addEventListener('change', () => { this._renderNeeded = true; });

        // Lumières
        const ambient = new THREE.AmbientLight(0xffffff, 0.75);
        this.scene.add(ambient);
        const dir = new THREE.DirectionalLight(0xffffff, 0.7);
        dir.position.set(120, -150, 250);
        this.scene.add(dir);

        // Grille de référence (sous la plaque)
        const grid = new THREE.GridHelper(400, 20, 0x223040, 0x1a2028);
        grid.rotation.x = Math.PI / 2;       // dans le plan XY (Z up)
        grid.position.z = -PLATE_THICKNESS - 0.1;
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

        this._renderNeeded = true;

        // Boucle render à la demande (économe en CPU/GPU)
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
    //  CHARGEMENT DU PROJET DE PERCAGE
    // -----------------------------------------------------------------
    loadProject(params) {
        this.params = params;
        this._clearWorld();

        const { sizeMM, depth, zSafe, toolDiameter, holes } = params;

        // ---- Plaque (matériau à percer) ----
        this.plateGroup = new THREE.Group();
        this.scene.add(this.plateGroup);

        const plateSize = (sizeMM || 50) + 2 * PLATE_MARGIN;
        const plateGeom = new THREE.BoxGeometry(plateSize, plateSize, PLATE_THICKNESS);
        const plateMat = new THREE.MeshStandardMaterial({ color: COLOR_PLATE, roughness: 0.85 });
        const plate = new THREE.Mesh(plateGeom, plateMat);
        plate.position.set(0, 0, -PLATE_THICKNESS / 2);
        this.plateGroup.add(plate);

        // Bordure foncée (cadre)
        const edgeGeom = new THREE.EdgesGeometry(plateGeom);
        const edgeMat = new THREE.LineBasicMaterial({ color: COLOR_PLATE_EDGE });
        const edges = new THREE.LineSegments(edgeGeom, edgeMat);
        edges.position.set(0, 0, -PLATE_THICKNESS / 2);
        this.plateGroup.add(edges);

        // Repère origine pièce (X=0, Y=0, Z=0) → petit cube vert
        const orig = new THREE.Mesh(
            new THREE.BoxGeometry(2, 2, 2),
            new THREE.MeshBasicMaterial({ color: 0x00ff88 })
        );
        orig.position.set(0, 0, 0.5);
        this.plateGroup.add(orig);

        // ---- Marqueurs de trous (disques sur le dessus de la plaque) ----
        this.holeMeshes = [];
        const holeR = Math.max(0.3, (toolDiameter || 1) / 2);
        const ringGeom = new THREE.CircleGeometry(holeR, 20);
        for (const h of (holes || [])) {
            const mat = new THREE.MeshBasicMaterial({
                color: COLOR_HOLE_PENDING, side: THREE.DoubleSide
            });
            const m = new THREE.Mesh(ringGeom, mat);
            // Légèrement au-dessus de la plaque pour éviter z-fighting
            m.position.set(h.x, h.y, 0.02);
            this.plateGroup.add(m);
            this.holeMeshes.push({ mesh: m, done: false });
        }

        // ---- Foret (TIGE du diamètre de l'outil) ----
        this.drill = new THREE.Group();
        const drillR = Math.max(0.25, (toolDiameter || 1) / 2);
        const tipLen = Math.max(1, drillR * 1.2);            // pointe ~ 1.2x rayon
        const bodyLen = Math.max(8, DRILL_TOTAL_LEN - tipLen);

        // Corps cylindrique (acier brillant)
        const bodyGeom = new THREE.CylinderGeometry(drillR, drillR, bodyLen, 20);
        bodyGeom.rotateX(Math.PI / 2);                       // axe Y -> Z
        const bodyMat = new THREE.MeshStandardMaterial({
            color: COLOR_DRILL_BODY, metalness: 0.75, roughness: 0.28
        });
        this.drillBody = new THREE.Mesh(bodyGeom, bodyMat);
        // En local : apex (pointe) à z=0, corps positionné au-dessus de la pointe
        this.drillBody.position.set(0, 0, tipLen + bodyLen / 2);
        this.drill.add(this.drillBody);

        // Pointe conique (apex pointe vers -Z = vers le bas)
        const tipGeom = new THREE.ConeGeometry(drillR, tipLen, 20);
        // Cone par défaut : apex à +Y. rotateX(-PI/2) → apex à -Z (parfait).
        tipGeom.rotateX(-Math.PI / 2);
        const tipMat = new THREE.MeshStandardMaterial({
            color: COLOR_DRILL_TIP, metalness: 0.6, roughness: 0.4
        });
        this.drillTip = new THREE.Mesh(tipGeom, tipMat);
        // Position telle que l'apex soit à z=0 (le 0 local = pointe du foret)
        this.drillTip.position.set(0, 0, tipLen / 2);
        this.drill.add(this.drillTip);

        // Spirales (hélices) : simples lignes pour l'effet visuel
        const helixGroup = new THREE.Group();
        const helixSegs = 32;
        const helixTurns = 3;
        for (let s = 0; s < 2; s++) {
            const pts = [];
            for (let i = 0; i <= helixSegs; i++) {
                const u = i / helixSegs;
                const ang = u * helixTurns * Math.PI * 2 + s * Math.PI;
                const z = tipLen + u * bodyLen;
                pts.push(new THREE.Vector3(Math.cos(ang) * drillR * 1.02, Math.sin(ang) * drillR * 1.02, z));
            }
            const g = new THREE.BufferGeometry().setFromPoints(pts);
            const m = new THREE.LineBasicMaterial({ color: 0x444a55 });
            helixGroup.add(new THREE.Line(g, m));
        }
        this.drill.add(helixGroup);

        // Position de départ : au centre de la plaque, à zSafe
        this.drill.position.set(0, 0, zSafe || 5);
        this.scene.add(this.drill);

        // Recadrer la caméra
        this._fitCameraToPlate();

        // Construire la timeline
        this._buildTimeline();

        // Reset état d'animation
        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
        this._setCutting(false);
        this._renderNeeded = true;
    }

    // -----------------------------------------------------------------
    //  TIMELINE — séquence d'étapes interpolées
    //  Types :
    //    'moveDrill' : interpolation linéaire (x0,y0,z0 → x1,y1,z1)
    //    'cutting'   : flag pointe rouge (perçage en cours)
    //    'markDone'  : marque un trou comme percé (changement couleur)
    //    'done'      : fin
    // -----------------------------------------------------------------
    _buildTimeline() {
        const p = this.params;
        const tl = [];

        const zSafe     = p.zSafe;
        const depth     = p.depth;
        const peckDepth = Math.max(0.01, p.peckDepth || depth);
        const usePeck   = !!p.usePeck && depth > peckDepth;
        const retractTotal = !!p.retractTotal;

        const holes = p.holes || [];

        // Vitesses (durée à vitesse 1.0)
        const D_RAPID_XY  = 0.25;   // déplacement rapide XY
        const D_PLUNGE    = 0.30;   // descente G1
        const D_RETRACT_G0 = 0.18;  // remontée G0
        const D_PECK_PARTIAL = 0.10;

        let cx = 0, cy = 0, cz = zSafe;

        for (let i = 0; i < holes.length; i++) {
            const h = holes[i];

            // 1) Déplacement rapide XY vers le trou (à zSafe)
            tl.push({ type: 'moveDrill', dur: D_RAPID_XY,
                      x0: cx, y0: cy, z0: cz,
                      x1: h.x, y1: h.y, z1: zSafe });
            cx = h.x; cy = h.y; cz = zSafe;

            if (usePeck) {
                // Débourrage : passes successives
                let currentZ = 0;
                let passNum = 0;
                while (currentZ < depth) {
                    passNum++;
                    const nextZ = Math.min(currentZ + peckDepth, depth);

                    if (passNum > 1) {
                        // Remontée intermédiaire (totale ou partielle)
                        const retZ = retractTotal ? zSafe : (zSafe * 0.3);
                        tl.push({ type: 'moveDrill', dur: D_PECK_PARTIAL,
                                  x0: cx, y0: cy, z0: cz,
                                  x1: cx, y1: cy, z1: retZ });
                        cz = retZ;
                    }

                    // Descente G1 (perçage)
                    tl.push({ type: 'cutting', dur: 0.001, on: true });
                    tl.push({ type: 'moveDrill', dur: D_PLUNGE * (peckDepth / depth) + 0.05,
                              x0: cx, y0: cy, z0: cz,
                              x1: cx, y1: cy, z1: -nextZ });
                    cz = -nextZ;
                    tl.push({ type: 'cutting', dur: 0.001, on: false });
                    currentZ = nextZ;
                }
            } else {
                // Perçage simple
                tl.push({ type: 'cutting', dur: 0.001, on: true });
                tl.push({ type: 'moveDrill', dur: D_PLUNGE,
                          x0: cx, y0: cy, z0: cz,
                          x1: cx, y1: cy, z1: -depth });
                cz = -depth;
                tl.push({ type: 'cutting', dur: 0.001, on: false });
            }

            // Marquer le trou comme fait
            tl.push({ type: 'markDone', dur: 0.001, holeIdx: i });

            // Remontée G0 finale vers zSafe
            tl.push({ type: 'moveDrill', dur: D_RETRACT_G0,
                      x0: cx, y0: cy, z0: cz,
                      x1: cx, y1: cy, z1: zSafe });
            cz = zSafe;
        }

        tl.push({ type: 'done', dur: 0.001 });
        this._timeline = tl;
        this._totalDur = tl.reduce((s, st) => s + st.dur, 0);
    }

    // -----------------------------------------------------------------
    //  TICK animation
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
        }
        if (!step) {
            this._playing = false;
            this._notifyProgress();
            return;
        }

        // Interpolation linéaire pour moveDrill
        if (step.type === 'moveDrill') {
            const u = step.dur > 0 ? Math.min(1, this._stepTime / step.dur) : 1;
            this._setDrillPos(
                step.x0 + (step.x1 - step.x0) * u,
                step.y0 + (step.y1 - step.y0) * u,
                step.z0 + (step.z1 - step.z0) * u
            );
        }
        this._notifyProgress();
    }

    _completeStep(step) {
        if (step.type === 'moveDrill') {
            this._setDrillPos(step.x1, step.y1, step.z1);
        } else if (step.type === 'cutting') {
            this._setCutting(!!step.on);
        } else if (step.type === 'markDone') {
            const h = this.holeMeshes[step.holeIdx];
            if (h && !h.done) {
                h.done = true;
                if (h.mesh && h.mesh.material) {
                    h.mesh.material.color.setHex(COLOR_HOLE_DONE);
                }
            }
        } else if (step.type === 'done') {
            this._playing = false;
        }
    }

    _setDrillPos(x, y, z) {
        if (!this.drill) return;
        this.drill.position.set(x, y, z);
    }

    _setCutting(on) {
        if (!this.drillTip) return;
        this.drillTip.material.color.setHex(on ? COLOR_DRILL_CUTTING : COLOR_DRILL_TIP);
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
        // Réinitialiser les marqueurs de trous
        for (const h of this.holeMeshes) {
            h.done = false;
            if (h.mesh && h.mesh.material) {
                h.mesh.material.color.setHex(COLOR_HOLE_PENDING);
            }
        }
        // Repositionner le foret au centre, à zSafe
        this._setDrillPos(0, 0, this.params.zSafe);
        this._setCutting(false);
        this._stepIdx = 0;
        this._stepTime = 0;
        this._playing = false;
        this._renderNeeded = true;
        this._notifyProgress();
    }

    setSpeed(s) { this._speed = Math.max(0.1, Math.min(8, s)); }

    setView(mode) {
        if (!this.params) return;
        const sz = this.params.sizeMM || 50;
        if (mode === 'top') {
            this.camera.position.set(0, 0, sz * 1.8);
            this.controls.target.set(0, 0, 0);
        } else { // iso
            this.camera.position.set(sz * 0.95, -sz * 1.1, sz * 1.0);
            this.controls.target.set(0, 0, 0);
        }
        this.camera.up.set(0, 0, 1);
        this.controls.update();
        this._renderNeeded = true;
    }

    _fitCameraToPlate() {
        this.setView('iso');
    }

    _notifyProgress() {
        if (typeof this.onProgress !== 'function') return;
        let elapsed = 0;
        for (let i = 0; i < this._stepIdx; i++) elapsed += this._timeline[i].dur;
        if (this._stepIdx < this._timeline.length) elapsed += this._stepTime;
        const pct = this._totalDur > 0 ? Math.min(1, elapsed / this._totalDur) : 1;
        const total = (this.params && this.params.holes) ? this.params.holes.length : 0;
        // Compter les trous déjà marqués done
        let doneCount = 0;
        for (const h of this.holeMeshes) if (h.done) doneCount++;
        this.onProgress({ pct, holeIdx: doneCount, holeTotal: total });
    }

    // -----------------------------------------------------------------
    //  NETTOYAGE
    // -----------------------------------------------------------------
    _clearWorld() {
        for (const h of this.holeMeshes) {
            if (h.mesh) {
                if (h.mesh.parent) h.mesh.parent.remove(h.mesh);
                if (h.mesh.material) h.mesh.material.dispose();
                if (h.mesh.geometry) h.mesh.geometry.dispose();
            }
        }
        this.holeMeshes = [];
        if (this.drill) {
            this.scene.remove(this.drill);
            this.drill.traverse(o => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
                    else o.material.dispose();
                }
            });
            this.drill = null;
            this.drillBody = null;
            this.drillTip = null;
        }
        if (this.plateGroup) {
            this.scene.remove(this.plateGroup);
            this.plateGroup.traverse(o => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
                    else o.material.dispose();
                }
            });
            this.plateGroup = null;
        }
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
//  Point d'entrée global
// =====================================================================
async function openQRCodeVisualizer(params, modalId, canvasId) {
    const modal = document.getElementById(modalId);
    if (!modal) return null;
    modal.classList.add('active');

    // Attendre que le modal soit visible (taille du canvas)
    await new Promise(r => setTimeout(r, 50));

    const viz = new QRCodeVisualizer3D(canvasId);
    await viz.init();
    viz.loadProject(params);
    modal._qrViz = viz;

    // Wiring boutons
    const btnPlay  = document.getElementById('qrVizPlay');
    const btnPause = document.getElementById('qrVizPause');
    const btnReset = document.getElementById('qrVizReset');
    const btnIso   = document.getElementById('qrVizIso');
    const btnTop   = document.getElementById('qrVizTop');
    const speed    = document.getElementById('qrVizSpeed');
    const speedVal = document.getElementById('qrVizSpeedVal');
    const close    = document.getElementById('qrVizClose');
    const closeX   = document.getElementById('qrVizCloseX');
    const progress = document.getElementById('qrVizProgress');
    const holeLbl  = document.getElementById('qrVizHole');

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

    viz.onProgress = ({ pct, holeIdx, holeTotal }) => {
        if (progress) progress.style.width = (pct * 100).toFixed(1) + '%';
        if (holeLbl)  holeLbl.textContent  = holeIdx + ' / ' + holeTotal;
        refreshPlayBtn();
    };

    // Auto-play à l'ouverture
    setTimeout(() => { viz.play(); refreshPlayBtn(); }, 100);

    const closeFn = () => {
        viz.dispose();
        modal.classList.remove('active');
        modal._qrViz = null;
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

window.QRCodeVisualizer3D   = QRCodeVisualizer3D;
window.openQRCodeVisualizer = openQRCodeVisualizer;
if (window._onQRVizReady) window._onQRVizReady();
