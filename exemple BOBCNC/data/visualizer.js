/**
 * BOBCNC - Simulateur CNC 3D
 * visualizer.js - Refonte complete
 *
 * Architecture :
 *   1. GCodeParser   - Parsing G-code pur (aucune dep Three.js)
 *   2. CNCVisualizer - Scene 3D, camera, controles, rendu
 *   3. Integration   - openCNCVisualizer() exposee sur window
 *
 * Compatible : decoupe.html, poche.html
 */

// ====================================================================
//  IMPORTS Three.js (static ES module)
// ====================================================================
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// ====================================================================
//  CACHE STL (persiste entre ouvertures du modal)
// ====================================================================
let _stlGeometryCache = null;

// ====================================================================
//  1. G-CODE PARSER
// ====================================================================
class GCodeParser {
    /**
     * Parse un programme G-code complet.
     * @param {string} gcode - Le programme G-code brut
     * @param {Object} offsets - { g54:{x,y}, g55:{x,y}, g56:{x,y}, g57:{x,y} }
     * @param {string} activeOffset - 'G54'|'G55'|'G56'|'G57'
     * @returns {{ segments: Array, waypoints: Array, totalLength: number }}
     */
    static parse(gcode, offsets = {}, activeOffset = 'G54') {
        const segments = [];   // { type:'G0'|'G1'|'ARC', points:[{x,y,z}], feed:number }
        const waypoints = [];  // points ordonnes pour l'animation de l'outil
        let totalLength = 0;

        let mode = 0;  // 0=G0, 1=G1, 2=G2, 3=G3
        let x = 0, y = 0, z = 0;
        let absMode = true;
        let feedRate = 1000;

        // Offset actif
        let currentOffset = GCodeParser._getOffset(offsets, activeOffset);

        // Buffer pour regrouper les segments contigus du meme type
        let currentSeg = null;

        function flushSeg() {
            if (currentSeg && currentSeg.points.length >= 2) {
                segments.push(currentSeg);
            }
            currentSeg = null;
        }

        function startSeg(type, startPt) {
            currentSeg = { type, points: [startPt], feed: feedRate };
        }

        // Ajouter le point initial au waypoints
        waypoints.push({ x: 0, y: 0, z: 0, dist: 0 });

        const lines = gcode.split('\n');

        for (const raw of lines) {
            // Retirer les commentaires (parentheses et point-virgule)
            let line = raw.split(';')[0]; // Retirer commentaire ;
            line = line.replace(/\(.*?\)/g, ''); // Retirer commentaires (...)
            line = line.trim();
            if (!line || line.startsWith('%')) continue;

            // Parser tous les mots de la ligne
            const allWords = GCodeParser._parseWords(line);

            // Traiter les G-codes (il peut y en avoir plusieurs par ligne)
            const gCodes = allWords.filter(w => w.letter === 'G');
            const params = {};
            for (const w of allWords) {
                if (w.letter !== 'G' && w.letter !== 'M') {
                    params[w.letter] = w.value;
                }
            }

            // Traiter les M-codes
            const mCodes = allWords.filter(w => w.letter === 'M');
            for (const mc of mCodes) {
                if (mc.value === 2 || mc.value === 30) {
                    flushSeg();
                    // Fin de programme
                    return { segments, waypoints, totalLength };
                }
            }

            // Traiter chaque G-code
            let modeChanged = false;
            for (const gc of gCodes) {
                const g = gc.value;
                if (g === 0) { if (mode !== 0) flushSeg(); mode = 0; modeChanged = true; }
                else if (g === 1) { if (mode !== 1) flushSeg(); mode = 1; modeChanged = true; }
                else if (g === 2) { if (mode !== 2) flushSeg(); mode = 2; modeChanged = true; }
                else if (g === 3) { if (mode !== 3) flushSeg(); mode = 3; modeChanged = true; }
                else if (g === 90) absMode = true;
                else if (g === 91) absMode = false;
                else if (g >= 54 && g <= 57) {
                    currentOffset = GCodeParser._getOffset(offsets, 'G' + g);
                }
                // G17, G21, G41, G40, etc. : ignores pour la visualisation
            }

            // Feed rate
            if (params['F'] !== undefined) feedRate = params['F'];

            // Calculer la nouvelle position
            const hasMove = params['X'] !== undefined || params['Y'] !== undefined || params['Z'] !== undefined;
            const hasArc = params['I'] !== undefined || params['J'] !== undefined || params['R'] !== undefined;

            if (!hasMove && !hasArc) continue;

            const nx = params['X'] !== undefined ? (absMode ? params['X'] + currentOffset.x : x + params['X']) : x;
            const ny = params['Y'] !== undefined ? (absMode ? params['Y'] + currentOffset.y : y + params['Y']) : y;
            const nz = params['Z'] !== undefined ? (absMode ? params['Z'] : z + params['Z']) : z;

            if (mode === 0 || mode === 1) {
                // Mouvement lineaire
                const segType = mode === 0 ? 'G0' : 'G1';
                const segLen = Math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2);

                if (!currentSeg || currentSeg.type !== segType) {
                    flushSeg();
                    startSeg(segType, { x, y, z });
                }
                currentSeg.points.push({ x: nx, y: ny, z: nz });
                totalLength += segLen;

                waypoints.push({ x: nx, y: ny, z: nz, dist: totalLength });

                x = nx; y = ny; z = nz;

            } else if (mode === 2 || mode === 3) {
                // Arc
                const clockwise = (mode === 2);
                const iVal = params['I'] || 0;
                const jVal = params['J'] || 0;
                const rVal = params['R'] || 0;

                const arcPts = GCodeParser._computeArc(
                    x, y, z, nx, ny, nz, iVal, jVal, rVal, clockwise, 32
                );

                if (!currentSeg || currentSeg.type !== 'ARC') {
                    flushSeg();
                    startSeg('ARC', { x, y, z });
                }

                let prevPt = { x, y, z };
                for (const pt of arcPts) {
                    currentSeg.points.push(pt);
                    const d = Math.sqrt((pt.x - prevPt.x) ** 2 + (pt.y - prevPt.y) ** 2 + (pt.z - prevPt.z) ** 2);
                    totalLength += d;
                    waypoints.push({ x: pt.x, y: pt.y, z: pt.z, dist: totalLength });
                    prevPt = pt;
                }

                x = nx; y = ny; z = nz;
            }
        }

        flushSeg();
        return { segments, waypoints, totalLength };
    }

    /** Parse les mots d'une ligne G-code -> [{ letter, value }] */
    static _parseWords(line) {
        const words = [];
        const regex = /([A-Z])([+-]?\d+\.?\d*)/gi;
        let m;
        while ((m = regex.exec(line)) !== null) {
            words.push({ letter: m[1].toUpperCase(), value: parseFloat(m[2]) });
        }
        return words;
    }

    /** Recupere un offset par nom */
    static _getOffset(offsets, name) {
        const key = name.toLowerCase();
        if (offsets[key]) return { x: offsets[key].x || 0, y: offsets[key].y || 0 };
        return { x: 0, y: 0 };
    }

    /** Calcul d'arc G2/G3, retourne un tableau de {x,y,z} */
    static _computeArc(sx, sy, sz, ex, ey, ez, i, j, r, clockwise, segments) {
        const points = [];

        if (i !== 0 || j !== 0) {
            const cx = sx + i;
            const cy = sy + j;
            const radius = Math.sqrt(i * i + j * j);
            let startAngle = Math.atan2(sy - cy, sx - cx);
            let endAngle = Math.atan2(ey - cy, ex - cx);

            if (clockwise) {
                while (endAngle >= startAngle) endAngle -= 2 * Math.PI;
            } else {
                while (endAngle <= startAngle) endAngle += 2 * Math.PI;
            }

            // Cercle complet : si start ~= end, forcer un tour complet
            if (Math.abs(sx - ex) < 0.001 && Math.abs(sy - ey) < 0.001) {
                endAngle = startAngle + (clockwise ? -2 * Math.PI : 2 * Math.PI);
            }

            for (let s = 0; s <= segments; s++) {
                const t = s / segments;
                const angle = startAngle + (endAngle - startAngle) * t;
                points.push({
                    x: cx + radius * Math.cos(angle),
                    y: cy + radius * Math.sin(angle),
                    z: sz + (ez - sz) * t
                });
            }
        } else if (r !== 0) {
            const dx = ex - sx;
            const dy = ey - sy;
            const d = Math.sqrt(dx * dx + dy * dy);

            if (d > Math.abs(r) * 2) {
                // Rayon trop petit, ligne droite fallback
                for (let s = 0; s <= segments; s++) {
                    const t = s / segments;
                    points.push({ x: sx + dx * t, y: sy + dy * t, z: sz + (ez - sz) * t });
                }
            } else {
                const h = Math.sqrt(r * r - (d / 2) ** 2);
                const mx = (sx + ex) / 2;
                const my = (sy + ey) / 2;

                let cx, cy;
                if (r > 0) { cx = mx + h * dy / d; cy = my - h * dx / d; }
                else { cx = mx - h * dy / d; cy = my + h * dx / d; }

                const radius = Math.abs(r);
                let startAngle = Math.atan2(sy - cy, sx - cx);
                let endAngle = Math.atan2(ey - cy, ex - cx);

                if (clockwise) { while (endAngle >= startAngle) endAngle -= 2 * Math.PI; }
                else { while (endAngle <= startAngle) endAngle += 2 * Math.PI; }

                for (let s = 0; s <= segments; s++) {
                    const t = s / segments;
                    const angle = startAngle + (endAngle - startAngle) * t;
                    points.push({
                        x: cx + radius * Math.cos(angle),
                        y: cy + radius * Math.sin(angle),
                        z: sz + (ez - sz) * t
                    });
                }
            }
        } else {
            points.push({ x: ex, y: ey, z: ez });
        }

        return points;
    }
}

// ====================================================================
//  2. CNC VISUALIZER
// ====================================================================
class CNCVisualizer {
    constructor(containerId) {
        this.containerId = containerId;
        this.container = document.getElementById(containerId);

        // Three.js objets
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.pathGroup = null;     // Trajectoires G-code
        this.batiGroup = null;     // Modele STL
        this.toolGroup = null;     // Outil 3D
        this.gridGroup = null;     // Grilles + axes
        this.labelSprites = [];    // Sprites labels (pour nettoyage)

        // Donnees parsees
        this._parsedData = null;   // { segments, waypoints, totalLength }
        this._lastGCode = '';
        this._offsets = {};
        this._activeOffset = 'G54';

        // Camera
        this.frustumSize = 200;
        this._camRotX = 0.7555;
        this._camRotY = 2.7412;
        this._camDist = 150;
        this._camTarget = new THREE.Vector3(0, 0, 0);

        // Controles
        this.brightness = 0.8;
        this.stlOpacity = 0.3;
        this.showAxes = true;
        this.showSTL = true;
        this.thicknessZ = 0;

        // Animation outil
        this._animProgress = 0;    // 0.0 - 1.0
        this._animPlaying = false;
        this._animSpeed = 1.0;     // multiplicateur vitesse
        this._animFrameId = null;
        this._animLastTime = 0;

        // Event listeners (pour nettoyage dans dispose)
        this._listeners = [];

        // Render flag
        this._renderNeeded = false;
    }

    // ================================================================
    //  INIT / DISPOSE
    // ================================================================
    async init() {
        this._setupScene();
        this._setupCamera();
        this._setupRenderer();
        this._setupLights();
        this._setupGrid();
        this._setupTool();
        this._setupControls();
        this._createUI();
        await this._loadOffsets();
        this._loadSettings();
        this._render();
    }

    dispose() {
        // Sauvegarder les parametres
        this._saveSettings();

        // Arreter l'animation
        this._animPlaying = false;
        if (this._animFrameId) {
            cancelAnimationFrame(this._animFrameId);
            this._animFrameId = null;
        }

        // Supprimer TOUS les event listeners
        for (const { target, type, handler, options } of this._listeners) {
            target.removeEventListener(type, handler, options);
        }
        this._listeners = [];

        // Disposer toutes les ressources Three.js
        if (this.scene) {
            this.scene.traverse(child => {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            });
        }

        // Disposer le renderer
        if (this.renderer) {
            this.renderer.dispose();
            if (this.renderer.domElement && this.renderer.domElement.parentNode) {
                this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
            }
        }

        // Supprimer les elements DOM crees
        const ui = this.container.querySelector('.viz-ui-panel');
        if (ui) ui.remove();
        const camLabel = this.container.querySelector('.viz-cam-label');
        if (camLabel) camLabel.remove();

        this.scene = null;
        this.camera = null;
        this.renderer = null;
    }

    // ================================================================
    //  SCENE
    // ================================================================
    _setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a1a);

        this.pathGroup = new THREE.Group();
        this.scene.add(this.pathGroup);

        this.batiGroup = new THREE.Group();
        this.scene.add(this.batiGroup);

        this.toolGroup = new THREE.Group();
        this.toolGroup.visible = true;
        this.scene.add(this.toolGroup);

        this.gridGroup = new THREE.Group();
        this.scene.add(this.gridGroup);
    }

    // ================================================================
    //  CAMERA ORTHOGRAPHIQUE
    // ================================================================
    _setupCamera() {
        const w = this.container.clientWidth || 600;
        const h = this.container.clientHeight || 500;
        const d = this.frustumSize;
        const aspect = w / h;
        this.camera = new THREE.OrthographicCamera(
            -d * aspect / 2, d * aspect / 2,
            d / 2, -d / 2,
            0.1, 5000
        );
        this.camera.up.set(0, 0, 1);
        this._applyCameraPosition();
    }

    _applyCameraPosition() {
        this.camera.position.set(
            this._camTarget.x + this._camDist * Math.cos(this._camRotX) * Math.sin(this._camRotY),
            this._camTarget.y + this._camDist * Math.cos(this._camRotX) * Math.cos(this._camRotY),
            this._camTarget.z + this._camDist * Math.sin(this._camRotX)
        );
        this.camera.lookAt(this._camTarget);
    }

    setView(mode) {
        if (mode === 'top') {
            this._camRotX = Math.PI / 2 - 0.01;
            this._camRotY = 0;
        } else {
            // Isometrique
            this._camRotX = 0.7555;
            this._camRotY = 2.7412;
        }
        this._applyCameraPosition();
        this.camera.updateProjectionMatrix();
        this._requestRender();
    }

    _updateFrustum() {
        const w = this.container.clientWidth || 600;
        const h = this.container.clientHeight || 500;
        const aspect = w / h;
        const d = this.frustumSize;
        this.camera.left = -d * aspect / 2;
        this.camera.right = d * aspect / 2;
        this.camera.top = d / 2;
        this.camera.bottom = -d / 2;
        this.camera.updateProjectionMatrix();
    }

    // ================================================================
    //  RENDERER
    // ================================================================
    _setupRenderer() {
        const w = this.container.clientWidth || 600;
        const h = this.container.clientHeight || 500;
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
        this.renderer.setSize(w, h);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.container.appendChild(this.renderer.domElement);
    }

    // ================================================================
    //  LUMIERES
    // ================================================================
    _setupLights() {
        this._ambLight = new THREE.AmbientLight(0x404060, this.brightness * 0.6);
        this.scene.add(this._ambLight);
        this._dirLight = new THREE.DirectionalLight(0xffffff, this.brightness);
        this._dirLight.position.set(100, 100, 200);
        this.scene.add(this._dirLight);
    }

    // ================================================================
    //  GRILLE + AXES
    // ================================================================
    _setupGrid() {
        const size = 300;
        const divisions = 30;

        // Grille fine
        const fineGrid = new THREE.GridHelper(size, divisions * 10, 0x333344, 0x222233);
        fineGrid.rotation.x = Math.PI / 2;
        fineGrid.position.z = -0.1;
        this.gridGroup.add(fineGrid);

        // Grille principale
        const mainGrid = new THREE.GridHelper(size, divisions, 0x555566, 0x444455);
        mainGrid.rotation.x = Math.PI / 2;
        this.gridGroup.add(mainGrid);

        // Axes
        this._addAxis(new THREE.Vector3(160, 0, 0), 0xff4444, 'X');
        this._addAxis(new THREE.Vector3(0, 160, 0), 0x44ff44, 'Y');
        this._addAxis(new THREE.Vector3(0, 0, 160), 0x4488ff, 'Z');

        // Graduations tous les 10mm
        for (let i = -size / 2; i <= size / 2; i += 10) {
            if (i === 0) continue;
            this._addTick(new THREE.Vector3(i, -2, 0), new THREE.Vector3(i, 2, 0), 0x888899);
            this._addTick(new THREE.Vector3(-2, i, 0), new THREE.Vector3(2, i, 0), 0x888899);
        }

        // Labels tous les 50mm (moins de sprites = moins de memoire)
        for (let i = -size / 2; i <= size / 2; i += 50) {
            if (i === 0) continue;
            this._addLabel(i.toString(), new THREE.Vector3(i, -8, 0), 0xaaaaaa, 1.5);
            this._addLabel(i.toString(), new THREE.Vector3(-8, i, 0), 0xaaaaaa, 1.5);
        }

        // Label origine
        this._addLabel('0', new THREE.Vector3(-3, -3, 0), 0xffffff, 1.5);
    }

    _addAxis(dir, color, label) {
        const mat = new THREE.LineBasicMaterial({ color });
        const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), dir]);
        this.gridGroup.add(new THREE.Line(geo, mat));
        if (label) this._addLabel(label, dir, color, 8);
    }

    _addTick(start, end, color) {
        const mat = new THREE.LineBasicMaterial({ color });
        const geo = new THREE.BufferGeometry().setFromPoints([start, end]);
        this.gridGroup.add(new THREE.Line(geo, mat));
    }

    _addLabel(text, pos, color, size) {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        canvas.width = 128;
        canvas.height = 128;
        ctx.fillStyle = '#' + color.toString(16).padStart(6, '0');
        ctx.font = 'bold 64px Arial';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, 64, 64);

        const tex = new THREE.CanvasTexture(canvas);
        const mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
        const sprite = new THREE.Sprite(mat);
        sprite.position.copy(pos);
        sprite.scale.set(size * 2, size * 2, 1);
        this.gridGroup.add(sprite);
        this.labelSprites.push(sprite);
    }

    // ================================================================
    //  OUTIL 3D (cone + cylindre)
    // ================================================================
    _setupTool() {
        // Pointe de l'outil (cone rouge, pointe vers -Z)
        const coneGeo = new THREE.ConeGeometry(3, 10, 16);
        coneGeo.rotateX(-Math.PI / 2); // Pointe vers -Z (bas CNC)
        const coneMat = new THREE.MeshStandardMaterial({ color: 0xff4444, roughness: 0.4, metalness: 0.6 });
        this._toolCone = new THREE.Mesh(coneGeo, coneMat);

        // Corps de l'outil (cylindre gris)
        const cylGeo = new THREE.CylinderGeometry(2.5, 3, 20, 16);
        cylGeo.rotateX(Math.PI / 2);
        cylGeo.translate(0, 0, 10);
        const cylMat = new THREE.MeshStandardMaterial({ color: 0x888899, roughness: 0.3, metalness: 0.7 });
        this._toolCyl = new THREE.Mesh(cylGeo, cylMat);

        this.toolGroup.add(this._toolCone);
        this.toolGroup.add(this._toolCyl);

        // Position initiale
        this.toolGroup.position.set(0, 0, 50);
    }

    // ================================================================
    //  CONTROLES SOURIS + TACTILE
    // ================================================================
    _setupControls() {
        const el = this.renderer.domElement;

        // --- SOURIS : ORBITE (clic gauche) + PAN (clic droit / Shift+clic) ---
        let isDragging = false, isPanning = false;
        let prevX = 0, prevY = 0;

        const onMouseDown = (e) => {
            prevX = e.clientX;
            prevY = e.clientY;
            if (e.button === 2 || e.shiftKey) {
                isPanning = true;
            } else if (e.button === 0) {
                isDragging = true;
            }
        };

        const onMouseMove = (e) => {
            const dx = e.clientX - prevX;
            const dy = e.clientY - prevY;
            prevX = e.clientX;
            prevY = e.clientY;

            if (isDragging) {
                // Orbite
                this._camRotY += dx * 0.005;
                this._camRotX += dy * 0.005;
                this._camRotX = Math.max(-Math.PI / 2 + 0.01, Math.min(Math.PI / 2 - 0.01, this._camRotX));
                this._applyCameraPosition();
                this._requestRender();
            } else if (isPanning) {
                // Pan : deplacer la cible de la camera
                const panScale = this.frustumSize / this.container.clientHeight;
                // Calculer les vecteurs right et up dans l'espace monde
                const right = new THREE.Vector3();
                const up = new THREE.Vector3();
                this.camera.getWorldDirection(new THREE.Vector3());
                right.crossVectors(this.camera.up, new THREE.Vector3().subVectors(this.camera.position, this._camTarget).normalize()).normalize();
                up.copy(this.camera.up).normalize();

                this._camTarget.addScaledVector(right, -dx * panScale);
                this._camTarget.addScaledVector(up, dy * panScale);
                this._applyCameraPosition();
                this._requestRender();
            }
        };

        const onMouseUp = () => { isDragging = false; isPanning = false; };

        // Context menu (empecher sur clic droit)
        const onContextMenu = (e) => { e.preventDefault(); };

        // Zoom molette (centre sur curseur)
        const onWheel = (e) => {
            e.preventDefault();
            // Zoom inverse : molette bas = zoom avant (convention CNC)
            const factor = e.deltaY > 0 ? 0.95 : 1.05;
            this.frustumSize *= factor;
            this.frustumSize = Math.max(20, Math.min(1000, this.frustumSize));
            this._updateFrustum();
            this._requestRender();
        };

        // --- TACTILE ---
        let lastTouchDist = 0;
        let lastTouchX = 0, lastTouchY = 0;
        let touchMode = 'none'; // 'orbit' | 'pinch'

        const onTouchStart = (e) => {
            if (e.touches.length === 1) {
                touchMode = 'orbit';
                lastTouchX = e.touches[0].clientX;
                lastTouchY = e.touches[0].clientY;
            } else if (e.touches.length === 2) {
                touchMode = 'pinch';
                const dx = e.touches[0].clientX - e.touches[1].clientX;
                const dy = e.touches[0].clientY - e.touches[1].clientY;
                lastTouchDist = Math.sqrt(dx * dx + dy * dy);
                lastTouchX = (e.touches[0].clientX + e.touches[1].clientX) / 2;
                lastTouchY = (e.touches[0].clientY + e.touches[1].clientY) / 2;
            }
        };

        const onTouchMove = (e) => {
            e.preventDefault();
            if (touchMode === 'orbit' && e.touches.length === 1) {
                const dx = e.touches[0].clientX - lastTouchX;
                const dy = e.touches[0].clientY - lastTouchY;
                this._camRotY += dx * 0.005;
                this._camRotX += dy * 0.005;
                this._camRotX = Math.max(-Math.PI / 2 + 0.01, Math.min(Math.PI / 2 - 0.01, this._camRotX));
                lastTouchX = e.touches[0].clientX;
                lastTouchY = e.touches[0].clientY;
                this._applyCameraPosition();
                this._requestRender();
            } else if (touchMode === 'pinch' && e.touches.length === 2) {
                const dx = e.touches[0].clientX - e.touches[1].clientX;
                const dy = e.touches[0].clientY - e.touches[1].clientY;
                const dist = Math.sqrt(dx * dx + dy * dy);
                // Ecarter = zoom avant
                const scale = lastTouchDist / dist;
                this.frustumSize *= scale;
                this.frustumSize = Math.max(20, Math.min(1000, this.frustumSize));
                lastTouchDist = dist;
                this._updateFrustum();
                this._requestRender();
            }
        };

        const onTouchEnd = () => { touchMode = 'none'; };

        // Resize
        const onResize = () => {
            const w = this.container.clientWidth;
            const h = this.container.clientHeight;
            if (w > 0 && h > 0) {
                this.renderer.setSize(w, h);
                this._updateFrustum();
                this._requestRender();
            }
        };

        // Enregistrer les listeners
        this._on(el, 'mousedown', onMouseDown);
        this._on(el, 'mousemove', onMouseMove);
        this._on(el, 'mouseup', onMouseUp);
        this._on(el, 'mouseleave', onMouseUp);
        this._on(el, 'contextmenu', onContextMenu);
        this._on(el, 'wheel', onWheel, { passive: false });
        this._on(el, 'touchstart', onTouchStart, { passive: true });
        this._on(el, 'touchmove', onTouchMove, { passive: false });
        this._on(el, 'touchend', onTouchEnd);
        this._on(window, 'resize', onResize);
    }

    /** Enregistre un event listener et le stocke pour nettoyage */
    _on(target, type, handler, options) {
        target.addEventListener(type, handler, options);
        this._listeners.push({ target, type, handler, options });
    }

    // ================================================================
    //  RENDU
    // ================================================================
    _render() {
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }

    _requestRender() {
        if (!this._renderNeeded) {
            this._renderNeeded = true;
            requestAnimationFrame(() => {
                this._renderNeeded = false;
                this._render();
            });
        }
    }

    // ================================================================
    //  OFFSETS G54-G57
    // ================================================================
    async _loadOffsets() {
        try {
            const response = await fetch('/api/offsets');
            if (response.ok) {
                this._offsets = await response.json();
            }
        } catch (e) {
            // Valeurs par defaut si ESP32 non accessible
            this._offsets = {
                g54: { x: 0, y: 0 }, g55: { x: 0, y: 0 },
                g56: { x: 0, y: 0 }, g57: { x: 0, y: 0 }
            };
        }
    }

    setPieceOrigin(origin) {
        this._activeOffset = origin || 'G54';
    }

    // ================================================================
    //  PARSING G-CODE + DESSIN
    // ================================================================
    parseGCode(gcode) {
        this._lastGCode = gcode;

        // Nettoyer l'ancien trajet
        this._clearGroup(this.pathGroup);

        // Parser
        this._parsedData = GCodeParser.parse(gcode, this._offsets, this._activeOffset);

        // Appliquer le decalage Z via la position du groupe (pas dans le parser)
        this.pathGroup.position.z = this.thicknessZ;

        // Dessiner les segments
        const colors = {
            'G0': { color: 0xff3333, dash: true },   // Rouge pointille
            'G1': { color: 0x00ff88, dash: false },   // Vert plein
            'ARC': { color: 0x00ccff, dash: false }    // Cyan plein
        };

        for (const seg of this._parsedData.segments) {
            const style = colors[seg.type] || colors['G1'];
            const points = seg.points.map(p => new THREE.Vector3(p.x, p.y, p.z));
            if (points.length < 2) continue;

            const geo = new THREE.BufferGeometry().setFromPoints(points);

            let mat;
            if (style.dash) {
                mat = new THREE.LineDashedMaterial({
                    color: style.color,
                    dashSize: 3,
                    gapSize: 2,
                    linewidth: 1
                });
            } else {
                mat = new THREE.LineBasicMaterial({ color: style.color, linewidth: 1 });
            }

            const line = new THREE.Line(geo, mat);
            if (style.dash) line.computeLineDistances();
            this.pathGroup.add(line);
        }

        // Positionner l'outil au debut
        this._animProgress = 0;
        this._updateToolPosition(0);

        // Ajuster la vue
        this._fitView();
    }

    _clearGroup(group) {
        while (group.children.length > 0) {
            const child = group.children[0];
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
            group.remove(child);
        }
    }

    // ================================================================
    //  AJUSTEMENT VUE
    // ================================================================
    _fitView() {
        if (this.pathGroup.children.length === 0) return;
        const box = new THREE.Box3().setFromObject(this.pathGroup);
        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());

        // Utiliser uniquement X et Y pour le frustum (pas Z)
        const maxDim = Math.max(size.x, size.y, 10);
        this.frustumSize = maxDim * 1.2;

        // Centrer la camera sur la trajectoire
        this._camTarget.copy(center);
        this._updateFrustum();
        this._applyCameraPosition();
        this._requestRender();
    }

    // ================================================================
    //  OUTIL : POSITION LE LONG DU TRAJET
    // ================================================================
    _updateToolPosition(progress) {
        if (!this._parsedData || this._parsedData.waypoints.length < 2) return;

        const wp = this._parsedData.waypoints;
        const targetDist = progress * this._parsedData.totalLength;

        // Trouver le segment correspondant (recherche binaire simple)
        let idx = 0;
        for (let i = 1; i < wp.length; i++) {
            if (wp[i].dist >= targetDist) { idx = i; break; }
            idx = i;
        }

        if (idx === 0) {
            this.toolGroup.position.set(wp[0].x, wp[0].y, wp[0].z + this.thicknessZ);
            return;
        }

        // Interpolation lineaire entre waypoints[idx-1] et waypoints[idx]
        const prev = wp[idx - 1];
        const curr = wp[idx];
        const segDist = curr.dist - prev.dist;
        const t = segDist > 0 ? (targetDist - prev.dist) / segDist : 0;

        this.toolGroup.position.set(
            prev.x + (curr.x - prev.x) * t,
            prev.y + (curr.y - prev.y) * t,
            (prev.z + (curr.z - prev.z) * t) + this.thicknessZ
        );
    }

    // ================================================================
    //  ANIMATION
    // ================================================================
    _startAnimation() {
        if (this._animPlaying) return;
        this._animPlaying = true;
        this._animLastTime = performance.now();

        const animate = (time) => {
            if (!this._animPlaying) return;

            const dt = (time - this._animLastTime) / 1000; // secondes
            this._animLastTime = time;

            // Vitesse : parcours complet en ~10 secondes a speed=1
            const speed = this._animSpeed / 10;
            this._animProgress += dt * speed;

            if (this._animProgress >= 1) {
                this._animProgress = 1;
                this._animPlaying = false;
            }

            this._updateToolPosition(this._animProgress);

            // Mettre a jour le slider
            const slider = this.container.querySelector('.viz-anim-slider');
            if (slider) slider.value = (this._animProgress * 1000).toFixed(0);

            // Mettre a jour le pourcentage
            const pct = this.container.querySelector('.viz-anim-pct');
            if (pct) pct.textContent = (this._animProgress * 100).toFixed(1) + '%';

            this._render();

            if (this._animPlaying) {
                this._animFrameId = requestAnimationFrame(animate);
            } else {
                // Mettre a jour le bouton
                const btn = this.container.querySelector('.viz-play-btn');
                if (btn) btn.textContent = '\u25B6';
            }
        };

        this._animFrameId = requestAnimationFrame(animate);
    }

    _stopAnimation() {
        this._animPlaying = false;
        if (this._animFrameId) {
            cancelAnimationFrame(this._animFrameId);
            this._animFrameId = null;
        }
    }

    _toggleAnimation() {
        if (this._animPlaying) {
            this._stopAnimation();
            const btn = this.container.querySelector('.viz-play-btn');
            if (btn) btn.textContent = '\u25B6';
        } else {
            if (this._animProgress >= 1) this._animProgress = 0;
            this._startAnimation();
            const btn = this.container.querySelector('.viz-play-btn');
            if (btn) btn.textContent = '\u23F8';
        }
    }

    _resetAnimation() {
        this._stopAnimation();
        this._animProgress = 0;
        this._updateToolPosition(0);
        const slider = this.container.querySelector('.viz-anim-slider');
        if (slider) slider.value = '0';
        const pct = this.container.querySelector('.viz-anim-pct');
        if (pct) pct.textContent = '0.0%';
        const btn = this.container.querySelector('.viz-play-btn');
        if (btn) btn.textContent = '\u25B6';
        this._requestRender();
    }

    // ================================================================
    //  CHARGEMENT STL (avec cache)
    // ================================================================
    async loadBati() {
        try {
            let geometry;
            if (_stlGeometryCache) {
                geometry = _stlGeometryCache.clone();
            } else {
                const loader = new STLLoader();
                geometry = await new Promise((resolve, reject) => {
                    loader.load('/BATITCNC.stl', resolve, undefined, reject);
                });
                // Y-up -> Z-up
                geometry.rotateX(Math.PI / 2);
                // Metres -> mm
                geometry.scale(1000, 1000, 1000);
                geometry.computeBoundingBox();
                _stlGeometryCache = geometry.clone();
            }

            const mat = new THREE.MeshStandardMaterial({
                color: 0x216666,
                roughness: 0.6,
                metalness: 0.4,
                transparent: true,
                opacity: this.stlOpacity,
                depthWrite: false
            });

            const mesh = new THREE.Mesh(geometry, mat);
            this.batiGroup.add(mesh);
            this.batiGroup.visible = this.showSTL;
            this._requestRender();
        } catch (e) {
            // STL non disponible - pas grave
        }
    }

    // ================================================================
    //  PANNEAU UI (DOM, pas de CSS inline)
    // ================================================================
    _createUI() {
        // Etiquette camera
        const camLabel = document.createElement('div');
        camLabel.className = 'viz-cam-label';
        this.container.appendChild(camLabel);
        this._camLabel = camLabel;
        this._updateCamLabel();

        // Panneau de controles
        const panel = document.createElement('div');
        panel.className = 'viz-ui-panel';
        panel.innerHTML = `
            <div class="viz-ui-title">CONTROLES</div>
            <div class="viz-ui-row">
                <span>Eclairage</span>
                <input type="range" class="viz-bright-slider" min="0" max="100" value="80">
                <span class="viz-bright-val">80%</span>
            </div>
            <div class="viz-ui-row">
                <span>STL</span>
                <input type="range" class="viz-opacity-slider" min="0" max="100" value="30">
                <span class="viz-opacity-val">30%</span>
            </div>
            <div class="viz-ui-row">
                <span>Epaisseur Z</span>
                <input type="range" class="viz-thick-slider" min="-50" max="50" value="0" step="0.5">
                <span class="viz-thick-val">0 mm</span>
            </div>
            <div class="viz-ui-row">
                <label><input type="checkbox" class="viz-axes-toggle" checked> Grille/Axes</label>
            </div>
            <div class="viz-ui-row">
                <label><input type="checkbox" class="viz-stl-toggle" checked> Bati STL</label>
            </div>
            <div class="viz-ui-sep"></div>
            <div class="viz-ui-title">ANIMATION OUTIL</div>
            <div class="viz-ui-row viz-anim-row">
                <button class="viz-play-btn" title="Lecture/Pause">\u25B6</button>
                <button class="viz-reset-btn" title="Rembobiner">\u23EE</button>
                <input type="range" class="viz-anim-slider" min="0" max="1000" value="0">
                <span class="viz-anim-pct">0.0%</span>
            </div>
            <div class="viz-ui-row">
                <span>Vitesse</span>
                <input type="range" class="viz-speed-slider" min="1" max="50" value="10">
                <span class="viz-speed-val">x1.0</span>
            </div>
        `;
        this.container.appendChild(panel);

        // Connecter les evenements
        this._setupUIEvents(panel);
    }

    _setupUIEvents(panel) {
        // Eclairage
        const brightSlider = panel.querySelector('.viz-bright-slider');
        const brightVal = panel.querySelector('.viz-bright-val');
        this._on(brightSlider, 'input', () => {
            const v = parseInt(brightSlider.value);
            this.brightness = v / 100;
            brightVal.textContent = v + '%';
            this._ambLight.intensity = this.brightness * 0.6;
            this._dirLight.intensity = this.brightness;
            this._saveSettings();
            this._requestRender();
        });

        // Opacite STL
        const opacitySlider = panel.querySelector('.viz-opacity-slider');
        const opacityVal = panel.querySelector('.viz-opacity-val');
        this._on(opacitySlider, 'input', () => {
            const v = parseInt(opacitySlider.value);
            this.stlOpacity = v / 100;
            opacityVal.textContent = v + '%';
            this.batiGroup.traverse(c => {
                if (c.isMesh && c.material) c.material.opacity = this.stlOpacity;
            });
            this._saveSettings();
            this._requestRender();
        });

        // Epaisseur Z
        const thickSlider = panel.querySelector('.viz-thick-slider');
        const thickVal = panel.querySelector('.viz-thick-val');
        this._on(thickSlider, 'input', () => {
            this.thicknessZ = parseFloat(thickSlider.value);
            thickVal.textContent = this.thicknessZ.toFixed(1) + ' mm';
            // Deplacer le groupe au lieu de re-parser
            this.pathGroup.position.z = this.thicknessZ;
            this._updateToolPosition(this._animProgress);
            this._saveSettings();
            this._requestRender();
        });

        // Grille/Axes
        const axesToggle = panel.querySelector('.viz-axes-toggle');
        this._on(axesToggle, 'change', () => {
            this.showAxes = axesToggle.checked;
            this.gridGroup.visible = this.showAxes;
            this._saveSettings();
            this._requestRender();
        });

        // STL
        const stlToggle = panel.querySelector('.viz-stl-toggle');
        this._on(stlToggle, 'change', () => {
            this.showSTL = stlToggle.checked;
            this.batiGroup.visible = this.showSTL;
            this._saveSettings();
            this._requestRender();
        });

        // Animation : Play/Pause
        const playBtn = panel.querySelector('.viz-play-btn');
        this._on(playBtn, 'click', () => this._toggleAnimation());

        // Animation : Reset
        const resetBtn = panel.querySelector('.viz-reset-btn');
        this._on(resetBtn, 'click', () => this._resetAnimation());

        // Animation : Slider (scrub)
        const animSlider = panel.querySelector('.viz-anim-slider');
        const animPct = panel.querySelector('.viz-anim-pct');
        this._on(animSlider, 'input', () => {
            this._stopAnimation();
            this._animProgress = parseInt(animSlider.value) / 1000;
            animPct.textContent = (this._animProgress * 100).toFixed(1) + '%';
            this._updateToolPosition(this._animProgress);
            const btn = panel.querySelector('.viz-play-btn');
            if (btn) btn.textContent = '\u25B6';
            this._requestRender();
        });

        // Vitesse
        const speedSlider = panel.querySelector('.viz-speed-slider');
        const speedVal = panel.querySelector('.viz-speed-val');
        this._on(speedSlider, 'input', () => {
            this._animSpeed = parseInt(speedSlider.value) / 10;
            speedVal.textContent = 'x' + this._animSpeed.toFixed(1);
        });
    }

    _updateCamLabel() {
        if (!this._camLabel || !this.camera) return;
        const p = this.camera.position;
        this._camLabel.innerHTML =
            `<b>CAM</b> X:${p.x.toFixed(1)} Y:${p.y.toFixed(1)} Z:${p.z.toFixed(1)} | Zoom:${this.frustumSize.toFixed(0)}`;
    }

    // ================================================================
    //  PERSISTENCE SETTINGS
    // ================================================================
    _saveSettings() {
        try {
            localStorage.setItem('cnc_viz_settings', JSON.stringify({
                brightness: this.brightness,
                stlOpacity: this.stlOpacity,
                showAxes: this.showAxes,
                showSTL: this.showSTL,
                thicknessZ: this.thicknessZ
            }));
        } catch (e) { /* quota depasse */ }
    }

    _loadSettings() {
        try {
            const saved = localStorage.getItem('cnc_viz_settings');
            if (!saved) return;
            const s = JSON.parse(saved);
            this.brightness = s.brightness ?? 0.8;
            this.stlOpacity = s.stlOpacity ?? 0.3;
            this.showAxes = s.showAxes ?? true;
            this.showSTL = s.showSTL ?? true;
            this.thicknessZ = s.thicknessZ ?? 0;
            this._applySettings();
        } catch (e) { /* JSON invalide */ }
    }

    _applySettings() {
        // Lumieres
        if (this._ambLight) this._ambLight.intensity = this.brightness * 0.6;
        if (this._dirLight) this._dirLight.intensity = this.brightness;

        // Visibilite
        if (this.gridGroup) this.gridGroup.visible = this.showAxes;
        if (this.batiGroup) this.batiGroup.visible = this.showSTL;

        // Epaisseur Z
        if (this.pathGroup) this.pathGroup.position.z = this.thicknessZ;

        // Mettre a jour les controles UI
        const panel = this.container.querySelector('.viz-ui-panel');
        if (!panel) return;

        const bright = panel.querySelector('.viz-bright-slider');
        if (bright) {
            bright.value = Math.round(this.brightness * 100);
            panel.querySelector('.viz-bright-val').textContent = Math.round(this.brightness * 100) + '%';
        }
        const opacity = panel.querySelector('.viz-opacity-slider');
        if (opacity) {
            opacity.value = Math.round(this.stlOpacity * 100);
            panel.querySelector('.viz-opacity-val').textContent = Math.round(this.stlOpacity * 100) + '%';
        }
        const thick = panel.querySelector('.viz-thick-slider');
        if (thick) {
            thick.value = this.thicknessZ;
            panel.querySelector('.viz-thick-val').textContent = this.thicknessZ.toFixed(1) + ' mm';
        }
        const axes = panel.querySelector('.viz-axes-toggle');
        if (axes) axes.checked = this.showAxes;
        const stl = panel.querySelector('.viz-stl-toggle');
        if (stl) stl.checked = this.showSTL;

        this._requestRender();
    }

    // ================================================================
    //  API PUBLIQUE
    // ================================================================
    getTotalLength() {
        return this._parsedData ? this._parsedData.totalLength : 0;
    }
}

// ====================================================================
//  3. INTEGRATION
// ====================================================================
async function openCNCVisualizer(gcode, modalId, canvasId, pieceOrigin = 'G54') {
    const modal = document.getElementById(modalId);
    if (!modal) return;
    modal.classList.add('active');

    // Attendre que le modal soit affiche
    await new Promise(r => setTimeout(r, 50));

    const viz = new CNCVisualizer(canvasId);
    await viz.init();

    viz.setPieceOrigin(pieceOrigin);
    viz.parseGCode(gcode);
    viz.loadBati();

    modal._viz = viz;

    // Boutons de vue
    const btnIso = document.getElementById('btnViewIso');
    const btnTop = document.getElementById('btnViewTop');
    if (btnIso) btnIso.onclick = () => viz.setView('iso');
    if (btnTop) btnTop.onclick = () => viz.setView('top');

    // Longueur totale
    const lenEl = document.getElementById('vizLength');
    if (lenEl) lenEl.textContent = viz.getTotalLength().toFixed(1) + ' mm';

    // Estimation temps (a feed rate moyen)
    const timeEl = document.getElementById('vizTime');
    if (timeEl && viz._parsedData) {
        const avgFeed = 500; // mm/min par defaut
        const minutes = viz.getTotalLength() / avgFeed;
        const sec = Math.round(minutes * 60);
        timeEl.textContent = `~${Math.floor(sec / 60)}m${(sec % 60).toString().padStart(2, '0')}s`;
    }

    // Fermeture
    const closeBtn = document.getElementById(modalId + 'Close');
    if (closeBtn) {
        closeBtn.onclick = () => {
            viz.dispose();
            modal.classList.remove('active');
        };
    }

    // Fermeture clic hors modal
    const onBgClick = (e) => {
        if (e.target === modal) {
            viz.dispose();
            modal.classList.remove('active');
            modal.removeEventListener('click', onBgClick);
        }
    };
    modal.addEventListener('click', onBgClick);
}

// Exposer sur window pour les onclick HTML et le canvas 2D
window.GCodeParser = GCodeParser;
window.openCNCVisualizer = openCNCVisualizer;
if (window._onParserReady) window._onParserReady();
