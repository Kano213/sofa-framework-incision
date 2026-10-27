"""
=============================================================================
PROJET : SOFA-INCISION - SIMULATION D'INCISION DE LA PEAU (SOFA FRAMEWORK)
Version SOFA : v26.06.00 (SofaPython3 / Python 3.12)
Étape 5 : Incision chirurgicale haute fidélité sur tout le champ opératoire.
          - Zone opératoire large (4.8 x 4.8 cm) à très haute résolution (pas 0.24 cm)
          - Entaille fine et nette (tolérance de coupe affinée à 0.19 cm)
          - Découpe chirurgicale valide en tout point (centre, côtés, diagonales)
          - Libération de raideur FEM (localStiffnessFactor = 0) + rafraîchissement
            visuel haute précision sans tranchée grossière ni instabilité.
=============================================================================
"""

import os
import sys
import math
import ctypes
import numpy as np
import Sofa
import Sofa.Core
import SofaRuntime

# =============================================================================
# PARAMÈTRES CHIRURGICAUX ET PHYSIQUES (RÉGLABLES EN HAUT DE FICHIER)
# =============================================================================
DT = 0.02                     # Pas de temps d'intégration (secondes)
GRAVITY = [0.0, -9.81, 0.0]   # Gravité terrestre

# Propriétés mécaniques du bloc de peau (FEM corotationnel)
SKIN_YOUNG_MODULUS = 3000.0   # Module d'Young (Pa)
SKIN_POISSON_RATIO = 0.45     # Incompressibilité
SKIN_TOTAL_MASS = 0.15        # Masse totale (kg)
SKIN_SURFACE_Y = 1.50         # Hauteur nominale de l'épiderme (cm)

# Propriétés de contact et de friction
CONTACT_FRICTION_MU = 0.3     # Coefficient de friction avec le bistouri
CONTACT_ALARM_DIST = 0.08     # Distance d'alarme de détection (cm)
CONTACT_DIST = 0.02           # Distance de contact effectif (cm)

# =============================================================================
# PARAMÈTRES DE LA MACHINE À ÉTATS & CRITÈRES D'INCISION
# =============================================================================
FORCE_THRESHOLD = 3.5         # Seuil de poussée normal requis pour inciser (N)
FORCE_RELEASE_THRESHOLD = 1.8 # Seuil de poussée bas pour maintenir l'incision (N)
MIN_SLIDING_SPEED = 0.02      # Vitesse min de glissement requise pour couper (cm/s)
MAX_INCISION_ANGLE_DEG = 75.0 # Angle max toléré entre lame et direction (degrés)
MAX_INCISION_DEPTH = 1.0      # Profondeur max d'enfoncement autorisée (cm)

# Tolérance latérale affinée pour une fente chirurgicale fine et nette (cm)
CUT_WIDTH_TOL = 0.19

# Vitesses de commande manuelle du bistouri
TRANS_SPEED_KEY = 0.04        # Vitesse de déplacement au clavier (cm/pas)
ROT_SPEED_KEY = 1.8           # Vitesse de rotation au clavier (degrés/pas)
MOUSE_SCALE_XZ = 0.0035       # Sensibilité souris sur le plan horizontal
MOUSE_SCALE_Y = 0.0035        # Sensibilité souris en hauteur


# =============================================================================
# CHARGEMENT DES PLUGINS SOFA REQUIS
# =============================================================================
REQUIRED_PLUGINS = [
    "Sofa.Component.AnimationLoop",
    "Sofa.Component.Collision.Detection.Algorithm",
    "Sofa.Component.Collision.Detection.Intersection",
    "Sofa.Component.Collision.Geometry",
    "Sofa.Component.Collision.Response.Contact",
    "Sofa.Component.Constraint.Lagrangian.Solver",
    "Sofa.Component.Constraint.Lagrangian.Correction",
    "Sofa.Component.Constraint.Lagrangian.Model",
    "Sofa.Component.Constraint.Projective",
    "Sofa.Component.Controller",
    "Sofa.Component.Engine.Select",
    "Sofa.Component.IO.Mesh",
    "Sofa.Component.LinearSolver.Direct",
    "Sofa.Component.LinearSolver.Iterative",
    "Sofa.Component.Mapping.Linear",
    "Sofa.Component.Mapping.NonLinear",
    "Sofa.Component.Mass",
    "Sofa.Component.MechanicalLoad",
    "Sofa.Component.ODESolver.Backward",
    "Sofa.Component.Setting",
    "Sofa.Component.SolidMechanics.FEM.Elastic",
    "Sofa.Component.SolidMechanics.Spring",
    "Sofa.Component.StateContainer",
    "Sofa.Component.Topology.Container.Constant",
    "Sofa.Component.Topology.Container.Dynamic",
    "Sofa.Component.Topology.Mapping",
    "Sofa.Component.Visual",
    "Sofa.GL.Component.Rendering3D",
]

for plugin_name in REQUIRED_PLUGINS:
    try:
        SofaRuntime.importPlugin(plugin_name)
    except Exception:
        pass


# =============================================================================
# UTILITAIRES WINDOWS POUR CONTRÔLES SOURIS ET CLAVIER
# =============================================================================
try:
    user32 = ctypes.windll.user32
except Exception:
    user32 = None


class POINT(ctypes.Structure):
    _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]


def is_vkey_down(vkey):
    """Vérifie l'état matériel d'une touche sous Windows sans perte d'événement."""
    if user32 is None:
        return False
    try:
        return bool(user32.GetAsyncKeyState(vkey) & 0x8000)
    except Exception:
        return False


def get_mouse_cursor_pos():
    """Retourne la position globale (X, Y) du curseur de la souris."""
    if user32 is None:
        return 0, 0
    try:
        pt = POINT()
        user32.GetCursorPos(ctypes.byref(pt))
        return pt.x, pt.y
    except Exception:
        return 0, 0


# =============================================================================
# GÉOMÉTRIE : CHAMP OPÉRATOIRE HAUTE RÉSOLUTION (8 x 1.5 x 8 CM)
# =============================================================================
def generate_skin_mesh():
    """
    Génère un bloc de peau volumique de 8 x 1.5 x 8 cm avec un champ opératoire
    élargi (-2.4 à +2.4 cm) à haute résolution chirurgicale (pas fin de 0.24 cm).
    Permet des incisions fines, réalistes et nettes sur tout le plateau opératoire.
    """
    center_x = list(np.linspace(-2.4, 2.4, 21))
    xs = [-4.0, -3.1] + center_x + [3.1, 4.0]

    center_z = list(np.linspace(-2.4, 2.4, 21))
    zs = [-4.0, -3.1] + center_z + [3.1, 4.0]

    ys = [0.0, 0.5, 1.0, 1.5]

    nx = len(xs)
    ny = len(ys)
    nz = len(zs)

    verts = []
    idx_map = {}
    for iy, y in enumerate(ys):
        for iz, z in enumerate(zs):
            for ix, x in enumerate(xs):
                idx_map[(ix, iy, iz)] = len(verts)
                verts.append([round(float(x), 4), round(float(y), 4), round(float(z), 4)])

    tetras = []
    for iy in range(ny - 1):
        for iz in range(nz - 1):
            for ix in range(nx - 1):
                p0 = idx_map[(ix, iy, iz)]
                p1 = idx_map[(ix + 1, iy, iz)]
                p2 = idx_map[(ix + 1, iy + 1, iz)]
                p3 = idx_map[(ix, iy + 1, iz)]
                p4 = idx_map[(ix, iy, iz + 1)]
                p5 = idx_map[(ix + 1, iy, iz + 1)]
                p6 = idx_map[(ix + 1, iy + 1, iz + 1)]
                p7 = idx_map[(ix, iy + 1, iz + 1)]

                if (ix + iy + iz) % 2 == 0:
                    tets = [
                        [p1, p0, p3, p4],
                        [p2, p1, p3, p6],
                        [p4, p1, p5, p6],
                        [p4, p3, p6, p7],
                        [p3, p1, p4, p6],
                    ]
                else:
                    tets = [
                        [p1, p0, p2, p5],
                        [p2, p0, p3, p7],
                        [p4, p0, p5, p7],
                        [p5, p2, p6, p7],
                        [p5, p0, p2, p7],
                    ]
                tetras.extend(tets)

    return verts, tetras, xs, ys, zs


def extract_surface_triangles(tetrahedra):
    """
    Extrait exactement les triangles de surface externe d'un ensemble de tétraèdres.
    Chaque face interne partagée par 2 tétraèdres s'annule (compte = 2).
    Seules les faces apparaissant une seule fois forment la peau externe et les berges.
    """
    face_counts = {}
    for tet in tetrahedra:
        v0, v1, v2, v3 = tet
        faces = [
            (v0, v2, v1),
            (v0, v1, v3),
            (v1, v2, v3),
            (v2, v0, v3),
        ]
        for f in faces:
            sorted_f = tuple(sorted(f))
            if sorted_f in face_counts:
                face_counts[sorted_f][0] += 1
            else:
                face_counts[sorted_f] = [1, f]

    surface_triangles = []
    for sorted_f, (cnt, original_f) in face_counts.items():
        if cnt == 1:
            surface_triangles.append(list(original_f))
    return surface_triangles


# =============================================================================
# MATHÉMATIQUES ROTATIONS ET QUATERNIONS
# =============================================================================
def euler_to_quaternion(pitch_deg, yaw_deg, roll_deg):
    """Convertit pitch, yaw, roll en quaternion unitaire [qx, qy, qz, qw]."""
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    r = math.radians(roll_deg) * 0.5

    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        return [qx / norm, qy / norm, qz / norm, qw / norm]
    return [0.0, 0.0, 0.0, 1.0]


# =============================================================================
# CONTRÔLEUR DU BISTOURI AVEC COUPE FINE HAUTE QUALITÉ
# =============================================================================
class ScalpelController(Sofa.Core.Controller):
    """
    Contrôleur chirurgical manuel haute fidélité.
    États : LIBRE -> CONTACT -> INCISION -> FIN.
    Découpe fine et nette le long de la pointe du bistouri.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.scalpel_dofs = kwargs.get("scalpel_dofs")
        self.skin_dofs = kwargs.get("skin_dofs")
        self.skin_topo = kwargs.get("skin_topo")
        self.skin_fem = kwargs.get("skin_fem")
        self.vis_topo = kwargs.get("vis_topo")
        self.vis_ogl = kwargs.get("vis_ogl")
        self.initial_tetras = kwargs.get("initial_tetras", [])
        self.initial_verts = kwargs.get("initial_verts", [])

        # Masque booléen des tétraèdres actifs
        self.active_tet_mask = np.ones(len(self.initial_tetras), dtype=bool)

        # Centroïdes précis de chaque tétraèdre
        verts_np = np.array(self.initial_verts)
        self.tet_centroids = np.zeros((len(self.initial_tetras), 3), dtype=float)
        for i, tet in enumerate(self.initial_tetras):
            self.tet_centroids[i] = np.mean(verts_np[tet], axis=0)

        # Position initiale du bistouri
        self.init_pos = [0.0, 1.85, 2.0]
        self.init_pitch = -25.0
        self.init_yaw = 0.0
        self.init_roll = 0.0

        self.pos = list(self.init_pos)
        self.pitch = self.init_pitch
        self.yaw = self.init_yaw
        self.roll = self.init_roll

        # États et télémétrie
        self.state = "LIBRE"
        self.depth_cm = 0.0
        self.push_force_n = 0.0
        self.sliding_speed = 0.0
        self.cut_distance_cm = 0.0
        self.max_cut_depth_mm = 0.0
        self.total_tets_cut = 0

        # Contrôles et souris
        self.prev_x_down = False
        self.mouse_mode = False
        self.prev_tab_down = False
        self.prev_mouse_x = None
        self.prev_mouse_y = None

        self.step_count = 0
        self.telemetry_cooldown = 0

    def init(self):
        self._sync_scalpel()
        self.printControls()

    def printControls(self):
        print("=" * 82)
        print(" [SOFA-INCISION] ÉTAPE 5 : COUPE HAUTE PRÉCISION (LARGE CHAMP OPÉRATOIRE)")
        print("=" * 82)
        print("  CONTRÔLES MANUELS DU BISTOURI :")
        print("     [F]                                 : APPUYER SUR LA PEAU (Augmente la poussée)")
        print("     [R]                                 : Relever le bistouri (Relâcher la poussée)")
        print("     [Z] / [S] ou [Flèches Haut/Bas]     : Glisser Avancer / Reculer (Axe Z)")
        print("     [Q] / [D] ou [Flèches Gauche/Droite]: Glisser Gauche / Droite (Axe X)")
        print("     [J] / [L]                           : Pivoter lacet (Yaw gauche / droite)")
        print("     [I] / [K]                           : Incliner angle d'attaque (Pitch +/-)")
        print("     [A] / [E]                           : Roulis de la lame (Roll +/-)")
        print("     [U]                                 : Replacer le bistouri en position haute")
        print("     [X]                                 : Réinitialiser la peau intacte")
        print("     [TAB]                               : Activer / Désactiver le mode souris")
        print("-" * 82)
        print("  GESTES POUR INCISER :")
        print("     1. Déplacez le bistouri sur le champ opératoire ([Q], [D], [Z], [S])")
        print(f"     2. Appuyez sur [F] pour enfoncer la pointe (Poussée >= {FORCE_THRESHOLD} N)")
        print(f"     3. Glissez avec [Z], [S], [Q] ou [D] (Vitesse >= {MIN_SLIDING_SPEED} cm/s)")
        print("     -> L'entaille s'ouvre sous vos yeux avec un trait fin et net !")
        print("=" * 82)

    def _sync_scalpel(self):
        """Met à jour les coordonnées du corps rigide dans SOFA."""
        if self.scalpel_dofs is None:
            return
        quat = euler_to_quaternion(self.pitch, self.yaw, self.roll)
        self.scalpel_dofs.position.value = [[
            self.pos[0], self.pos[1], self.pos[2],
            quat[0], quat[1], quat[2], quat[3]
        ]]

    def _get_blade_positions(self):
        """Calcule la position monde 3D exacte de la pointe de la lame."""
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw)

        tip_dy = 0.45 * math.sin(pitch_rad)
        tip_dz = -0.45 * math.cos(pitch_rad)

        tip_x = self.pos[0] - tip_dz * math.sin(yaw_rad)
        tip_y = self.pos[1] + tip_dy
        tip_z = self.pos[2] + tip_dz * math.cos(yaw_rad)

        return [tip_x, tip_y, tip_z]

    def _cut_tetrahedra_along_path(self, tip_prev, tip_curr, blade_depth_y):
        """
        Supprime/libère avec précision les tétraèdres traversés par le fil de la lame.
        Utilise une tolérance étroite (0.19 cm) pour un trait de coupe fin et chirurgical.
        """
        if self.skin_fem is None:
            return 0

        p0 = np.array([tip_prev[0], tip_prev[2]])
        p1 = np.array([tip_curr[0], tip_curr[2]])
        v = p1 - p0
        v_sq = float(np.dot(v, v))
        c_xz = self.tet_centroids[:, [0, 2]]

        if v_sq > 1e-8:
            t = np.clip(np.dot(c_xz - p0, v) / v_sq, 0.0, 1.0)
            proj = p0 + t[:, None] * v
            dist_sq = np.sum((c_xz - proj) ** 2, axis=1)
        else:
            dist_sq = np.sum((c_xz - p0) ** 2, axis=1)

        # Seuil Y : couche supérieure (cy >= 0.95), et couche intermédiaire si enfoncement > 6 mm
        depth_thresh = 0.95 if blade_depth_y >= 0.90 else 0.45

        mask = (
            self.active_tet_mask
            & (dist_sq <= (CUT_WIDTH_TOL ** 2))
            & (self.tet_centroids[:, 1] >= depth_thresh)
        )
        cut_indices = np.where(mask)[0]

        if len(cut_indices) == 0:
            return 0

        # Désactivation des tétraèdres coupés
        self.active_tet_mask[cut_indices] = False
        self.total_tets_cut += len(cut_indices)

        # 1. Annulation de la raideur FEM (localStiffnessFactor = 0.0 pour les éléments incisés)
        stiffness_factors = [1.0 if act else 0.0 for act in self.active_tet_mask]
        self.skin_fem.localStiffnessFactor.value = stiffness_factors

        # 2. Mise à jour dynamique de la surface visuelle (triangles extérieurs)
        remaining_tets = [self.initial_tetras[i] for i, act in enumerate(self.active_tet_mask) if act]
        new_surface_tris = extract_surface_triangles(remaining_tets)
        if self.vis_topo is not None:
            self.vis_topo.triangles.value = new_surface_tris
        if self.vis_ogl is not None:
            self.vis_ogl.triangles.value = new_surface_tris

        return len(cut_indices)

    def _reset_skin(self):
        """Restaure la peau intacte dans son état d'origine."""
        self.active_tet_mask.fill(True)
        self.total_tets_cut = 0
        if self.skin_fem is not None:
            self.skin_fem.localStiffnessFactor.value = [1.0] * len(self.initial_tetras)
        init_tris = extract_surface_triangles(self.initial_tetras)
        if self.vis_topo is not None:
            self.vis_topo.triangles.value = init_tris
        if self.vis_ogl is not None:
            self.vis_ogl.triangles.value = init_tris
        self.pos = list(self.init_pos)
        self.pitch = self.init_pitch
        self.yaw = self.init_yaw
        self.roll = self.init_roll
        self.state = "LIBRE"
        self._sync_scalpel()
        print("=" * 82)
        print(f" [RÉINITIALISATION] Peau restaurée intacte ({len(self.initial_tetras)} tétraèdres).")
        print("=" * 82)

    def _check_incision_criteria(self, dx, dz):
        """Vérifie si les 3 critères chirurgicaux sont satisfaits."""
        pushed = (self.push_force_n >= FORCE_THRESHOLD)
        vx = dx / DT
        vz = dz / DT
        self.sliding_speed = math.sqrt(vx * vx + vz * vz)
        speed_ok = (self.sliding_speed >= MIN_SLIDING_SPEED)

        attack_angle = -self.pitch
        pitch_ok = (10.0 <= attack_angle <= 75.0)
        roll_ok = (abs(self.roll) <= 50.0)

        if speed_ok:
            move_heading_deg = math.degrees(math.atan2(vx, -vz))
            yaw_mod = self.yaw % 360.0
            angle_diff = abs((move_heading_deg - yaw_mod + 180.0) % 360.0 - 180.0)
            blade_align_diff = min(angle_diff, abs(180.0 - angle_diff))
            align_ok = (blade_align_diff <= MAX_INCISION_ANGLE_DEG)
        else:
            align_ok = False

        orient_ok = (pitch_ok and roll_ok and align_ok)
        return pushed, speed_ok, orient_ok

    def onAnimateBeginEvent(self, event):
        self.step_count += 1
        moved = False
        dx, dy, dz = 0.0, 0.0, 0.0

        # Position de la pointe avant mouvement
        tip_prev = self._get_blade_positions()

        # ---------------------------------------------------------------------
        # 1. Touche de réinitialisation [X]
        # ---------------------------------------------------------------------
        x_down = is_vkey_down(0x58)
        if x_down and not self.prev_x_down:
            self._reset_skin()
        self.prev_x_down = x_down

        # ---------------------------------------------------------------------
        # 2. Contrôles manuels au Clavier
        # ---------------------------------------------------------------------
        # Glissement Z (Avancer / Reculer)
        if is_vkey_down(0x5A) or is_vkey_down(0x26):  # [Z] ou [Haut] : Avancer vers -Z
            dz -= TRANS_SPEED_KEY
            moved = True
        if is_vkey_down(0x53) or is_vkey_down(0x28):  # [S] ou [Bas] : Reculer vers +Z
            dz += TRANS_SPEED_KEY
            moved = True

        # Glissement latéral X (Gauche / Droite)
        if is_vkey_down(0x51) or is_vkey_down(0x25):  # [Q] ou [Gauche] : Gauche -X
            dx -= TRANS_SPEED_KEY
            moved = True
        if is_vkey_down(0x44) or is_vkey_down(0x27):  # [D] ou [Droite] : Droite +X
            dx += TRANS_SPEED_KEY
            moved = True

        # Hauteur / Pression Y
        if is_vkey_down(0x52):  # [R] : Monter / Relâcher
            dy += TRANS_SPEED_KEY
            moved = True
        if is_vkey_down(0x46):  # [F] : Descendre / APPUYER
            dy -= TRANS_SPEED_KEY
            moved = True

        # Rotations d'orientation
        if is_vkey_down(0x4A):  # [J] : Yaw Gauche
            self.yaw -= ROT_SPEED_KEY
            moved = True
        if is_vkey_down(0x4C):  # [L] : Yaw Droite
            self.yaw += ROT_SPEED_KEY
            moved = True
        if is_vkey_down(0x49):  # [I] : Pitch Haut
            self.pitch = max(-85.0, min(10.0, self.pitch - ROT_SPEED_KEY))
            moved = True
        if is_vkey_down(0x4B):  # [K] : Pitch Bas
            self.pitch = max(-85.0, min(10.0, self.pitch + ROT_SPEED_KEY))
            moved = True
        if is_vkey_down(0x41):  # [A] : Roll Gauche
            self.roll = max(-60.0, min(60.0, self.roll - ROT_SPEED_KEY))
            moved = True
        if is_vkey_down(0x45):  # [E] : Roll Droite
            self.roll = max(-60.0, min(60.0, self.roll + ROT_SPEED_KEY))
            moved = True

        # Reset pose [U]
        if is_vkey_down(0x55):
            self.pos = list(self.init_pos)
            self.pitch = self.init_pitch
            self.yaw = self.init_yaw
            self.roll = self.init_roll
            self.state = "LIBRE"
            moved = True

        # ---------------------------------------------------------------------
        # 3. Contrôle à la souris ([TAB] pour basculer)
        # ---------------------------------------------------------------------
        tab_down = is_vkey_down(0x09)
        if tab_down and not self.prev_tab_down:
            self.mouse_mode = not self.mouse_mode
            status = "ACTIVÉ (Clic G = Glisser XZ, Clic D = Appuyer/Monter Y)" if self.mouse_mode else "DÉSACTIVÉ"
            print(f" [SOURIS] Mode souris {status}")
        self.prev_tab_down = tab_down

        mx, my = get_mouse_cursor_pos()
        if self.mouse_mode:
            if self.prev_mouse_x is not None and self.prev_mouse_y is not None:
                dmx = mx - self.prev_mouse_x
                dmy = my - self.prev_mouse_y
                if is_vkey_down(0x01):  # Clic Gauche : Glissement horizontal XZ
                    dx += dmx * MOUSE_SCALE_XZ
                    dz += dmy * MOUSE_SCALE_XZ
                    moved = True
                if is_vkey_down(0x02):  # Clic Droit : Hauteur Y
                    dy -= dmy * MOUSE_SCALE_Y
                    moved = True
            self.prev_mouse_x = mx
            self.prev_mouse_y = my
        else:
            self.prev_mouse_x = None
            self.prev_mouse_y = None

        # ---------------------------------------------------------------------
        # 4. Cinématique et contact
        # ---------------------------------------------------------------------
        self.pos[0] = max(-3.5, min(3.5, self.pos[0] + dx))
        self.pos[1] = max(SKIN_SURFACE_Y - MAX_INCISION_DEPTH, min(3.0, self.pos[1] + dy))
        self.pos[2] = max(-3.5, min(3.5, self.pos[2] + dz))

        # Position de la pointe après mouvement
        tip_curr = self._get_blade_positions()
        tip_y = tip_curr[1]

        # Calcul de pénétration et poussée
        if tip_y < SKIN_SURFACE_Y:
            self.depth_cm = SKIN_SURFACE_Y - tip_y
            self.push_force_n = self.depth_cm * 36.0
        else:
            self.depth_cm = 0.0
            self.push_force_n = 0.0

        pushed, speed_ok, orient_ok = self._check_incision_criteria(dx, dz)

        # ---------------------------------------------------------------------
        # 5. Machine à états
        # ---------------------------------------------------------------------
        old_state = self.state

        if self.state == "LIBRE":
            if self.depth_cm > 0.0:
                self.state = "CONTACT"
                print(f" [MACHINE À ÉTATS] --> CONTACT avec la peau à X={self.pos[0]:.2f}, Z={self.pos[2]:.2f} cm")

        elif self.state == "CONTACT":
            if self.depth_cm <= 0.0:
                self.state = "LIBRE"
                print(" [MACHINE À ÉTATS] --> BISTOURI RELEVÉ (Retour à LIBRE)")
            elif pushed and speed_ok and orient_ok:
                self.state = "INCISION"
                self.cut_distance_cm = 0.0
                self.max_cut_depth_mm = self.depth_cm * 10.0
                print("=" * 82)
                print(f" [MACHINE À ÉTATS] >>> DÉBUT DE L'INCISION ! <<<")
                print(f"   Position initiale : X={self.pos[0]:.2f} cm, Z={self.pos[2]:.2f} cm")
                print(f"   Poussée : {self.push_force_n:.2f} N (>= {FORCE_THRESHOLD} N)")
                print(f"   Vitesse : {self.sliding_speed:.2f} cm/s")
                print("=" * 82)

        elif self.state == "INCISION":
            if self.depth_cm <= 0.0 or self.push_force_n < FORCE_RELEASE_THRESHOLD:
                self.state = "FIN"
                print("=" * 82)
                print(f" [MACHINE À ÉTATS] <-- FIN D'INCISION (Bistouri relevé ou relâché)")
                print(f"   Longueur incisée : {self.cut_distance_cm:.2f} cm | Tétraèdres coupés : {self.total_tets_cut}")
                print("=" * 82)
            else:
                if moved:
                    segment = math.sqrt(dx * dx + dz * dz)
                    self.cut_distance_cm += segment
                    self.max_cut_depth_mm = max(self.max_cut_depth_mm, self.depth_cm * 10.0)

        elif self.state == "FIN":
            if self.depth_cm <= 0.0:
                self.state = "LIBRE"
            elif pushed and speed_ok and orient_ok:
                self.state = "INCISION"
                print(" [MACHINE À ÉTATS] >>> REPRISE DE L'INCISION ! <<<")

        # ---------------------------------------------------------------------
        # 6. Exécution de la coupe topologique fine et nette
        # ---------------------------------------------------------------------
        if self.state == "INCISION" and moved:
            n_cut = self._cut_tetrahedra_along_path(tip_prev, tip_curr, tip_y)
            if n_cut > 0:
                print(
                    f" [COUPE TÉTRAÉDRIQUE] +{n_cut} tétraèdres libérés "
                    f"(Total: {self.total_tets_cut}) | "
                    f"Pos: (X={self.pos[0]:4.2f}, Z={self.pos[2]:4.2f}) cm | "
                    f"Prof: {self.depth_cm * 10.0:3.1f} mm"
                )

        # Synchronisation position bistouri
        self._sync_scalpel()

        # ---------------------------------------------------------------------
        # 7. Affichage console télémétrie
        # ---------------------------------------------------------------------
        self.telemetry_cooldown += 1
        state_changed = (self.state != old_state)

        if state_changed or (moved and self.telemetry_cooldown >= 12):
            self.telemetry_cooldown = 0
            depth_mm = self.depth_cm * 10.0

            extra = ""
            if self.state == "INCISION":
                extra = f" | Trajet: {self.cut_distance_cm:4.2f} cm | Total coupés: {self.total_tets_cut}"
            elif self.state == "CONTACT":
                needed = []
                if not pushed:
                    needed.append(f"Poussée {self.push_force_n:.1f}/{FORCE_THRESHOLD}N [F]")
                if not speed_ok:
                    needed.append("Glisser [Z]/[S]/[Q]/[D]")
                if not orient_ok:
                    needed.append("Aligner lame [J]/[L]")
                extra = f" | Attente: {', '.join(needed)}"

            print(
                f" [BISTOURI] État: {self.state:<8} | "
                f"Pos: ({self.pos[0]:4.2f}, {self.pos[2]:4.2f}) cm | "
                f"Poussée: {self.push_force_n:4.1f} N | "
                f"Prof: {depth_mm:4.1f} mm"
                f"{extra}"
            )


# =============================================================================
# SCÈNE SOFA
# =============================================================================
def createScene(root):
    root.name = "root"
    root.gravity = GRAVITY
    root.dt = DT

    root.addObject("FreeMotionAnimationLoop", name="animationLoop")
    root.addObject(
        "BlockGaussSeidelConstraintSolver",
        name="constraintSolver",
        tolerance=1e-4,
        maxIterations=250,
        multithreading=False,
    )

    root.addObject("CollisionPipeline", name="collisionPipeline", verbose=False)
    root.addObject("BruteForceBroadPhase", name="broadPhase")
    root.addObject("BVHNarrowPhase", name="narrowPhase")
    root.addObject(
        "LocalMinDistance",
        name="proximity",
        alarmDistance=CONTACT_ALARM_DIST,
        contactDistance=CONTACT_DIST,
        angleCone=0.0,
    )
    root.addObject(
        "CollisionResponse",
        name="response",
        response="FrictionContactConstraint",
        responseParams=f"mu={CONTACT_FRICTION_MU}",
    )

    root.addObject(
        "VisualStyle",
        displayFlags="showVisual hideBehaviorModels hideForceFields hideCollisionModels",
    )

    # DÉCOR (conservé partout : sol blanc opaque, fond bleu ciel, caméra en biais)
    root.addObject("BackgroundSetting", color=[0.55, 0.75, 0.95, 1.0])
    root.addObject(
        "InteractiveCamera",
        name="camera",
        position=[0.0, 5.0, 7.5],
        lookAt=[0.0, 0.75, 0.0],
        fieldOfView=45,
    )

    floor = root.addChild("Floor")
    floor.addObject(
        "MeshTopology",
        name="topo",
        position="-12 -0.01 -12  12 -0.01 -12  12 -0.01 12  -12 -0.01 12",
        triangles="0 1 2  0 2 3",
    )
    floor.addObject("OglModel", name="visual", color=[0.96, 0.96, 0.96, 1.0])

    # =========================================================================
    # BLOC DE PEAU DÉFORMABLE (FEM) HAUTE QUALITÉ
    # =========================================================================
    skin = root.addChild("SkinBlock")
    skin.addObject("EulerImplicitSolver", name="odeSolver", rayleighStiffness=0.15, rayleighMass=0.15)
    skin.addObject("SparseLDLSolver", name="directSolver", template="CompressedRowSparseMatrixd")

    verts, tetras, xs, ys, zs = generate_skin_mesh()
    skin_topo = skin.addObject("TetrahedronSetTopologyContainer", name="topo", position=verts, tetrahedra=tetras)
    skin.addObject("TetrahedronSetTopologyModifier", name="modifier")
    skin.addObject("TetrahedronSetGeometryAlgorithms", name="algo")

    skin_dofs = skin.addObject("MechanicalObject", name="dofs", template="Vec3d", src="@topo")
    skin.addObject("MeshMatrixMass", name="mass", totalMass=SKIN_TOTAL_MASS)
    skin_fem = skin.addObject(
        "TetrahedralCorotationalFEMForceField",
        name="FEM",
        youngModulus=SKIN_YOUNG_MODULUS,
        poissonRatio=SKIN_POISSON_RATIO,
        method="polar",
    )

    # Base et pourtour extérieur ancrés
    fixed_indices = []
    for idx, v in enumerate(verts):
        x, y, z = v
        if y <= 0.01 or abs(x) >= 3.99 or abs(z) >= 3.99:
            fixed_indices.append(idx)

    skin.addObject("FixedProjectiveConstraint", name="fixBase", indices=fixed_indices)
    skin.addObject("LinearSolverConstraintCorrection", name="constraintCorrection")

    # Modèle de collision
    skin_coll = skin.addChild("Collision")
    skin_coll.addObject("TriangleSetTopologyContainer", name="collTopo")
    skin_coll.addObject("TriangleSetTopologyModifier", name="collModifier")
    skin_coll.addObject("Tetra2TriangleTopologicalMapping", input="@../topo", output="@collTopo")
    skin_coll.addObject("MechanicalObject", name="collDofs", position=verts)
    skin_coll.addObject("TriangleCollisionModel", name="triangles", simulated=True, moving=True)
    skin_coll.addObject("LineCollisionModel", name="lines", simulated=True, moving=True)
    skin_coll.addObject("PointCollisionModel", name="points", simulated=True, moving=True)
    skin_coll.addObject("IdentityMapping", input="@../dofs", output="@collDofs")

    # Modèle visuel dynamique haute fidélité
    init_surface_tris = extract_surface_triangles(tetras)
    skin_vis = skin.addChild("Visual")
    vis_topo = skin_vis.addObject(
        "TriangleSetTopologyContainer",
        name="visTopo",
        position=verts,
        triangles=init_surface_tris,
    )
    skin_vis.addObject("TriangleSetTopologyModifier", name="visModifier")
    vis_ogl = skin_vis.addObject(
        "OglModel",
        name="visualModel",
        color=[0.90, 0.68, 0.55, 1.0],
        src="@visTopo",
    )
    skin_vis.addObject("IdentityMapping", input="@../dofs", output="@visualModel")

    # =========================================================================
    # BISTOURI RIGIDE (MANCHE + LAME N°10)
    # =========================================================================
    scalpel = root.addChild("Scalpel")
    scalpel.addObject("EulerImplicitSolver", name="odeSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    scalpel.addObject("CGLinearSolver", name="linearSolver", iterations=25, tolerance=1e-5)

    q_init = euler_to_quaternion(-25.0, 0.0, 0.0)
    scalpel_dofs = scalpel.addObject(
        "MechanicalObject",
        name="dofs",
        template="Rigid3d",
        position=[[0.0, 1.85, 2.0, q_init[0], q_init[1], q_init[2], q_init[3]]],
    )
    scalpel.addObject("UniformMass", name="mass", totalMass=0.04)
    scalpel.addObject("UncoupledConstraintCorrection")

    mesh_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes", "scalpel.obj")
    scalpel_vis = scalpel.addChild("Visual")
    if os.path.exists(mesh_path):
        scalpel_vis.addObject("MeshOBJLoader", name="loader", filename=mesh_path)
        scalpel_vis.addObject("OglModel", name="visualModel", src="@loader", color=[0.78, 0.80, 0.84, 1.0])
        scalpel_vis.addObject("RigidMapping", input="@../dofs", output="@visualModel")

    scalpel_coll = scalpel.addChild("Collision")
    blade_collision_pts = [
        [0.0, 0.0, 0.0],
        [0.0, -0.05, -0.15],
        [0.0, -0.10, -0.30],
        [0.0, -0.12, -0.45],
        [0.0, -0.06, -0.60],
        [0.0, 0.02, -0.75],
    ]
    scalpel_coll.addObject("MechanicalObject", name="bladePoints", template="Vec3d", position=blade_collision_pts)
    scalpel_coll.addObject("SphereCollisionModel", name="bladeSpheres", radius=0.04, simulated=False, moving=True)
    scalpel_coll.addObject("RigidMapping", input="@../dofs", output="@bladePoints")

    # =========================================================================
    # CONTRÔLEUR D'INCISION CHIRURGICALE
    # =========================================================================
    controller = ScalpelController(
        name="ScalpelController",
        scalpel_dofs=scalpel_dofs,
        skin_dofs=skin_dofs,
        skin_topo=skin_topo,
        skin_fem=skin_fem,
        vis_topo=vis_topo,
        vis_ogl=vis_ogl,
        initial_tetras=tetras,
        initial_verts=verts,
    )
    root.addObject(controller)

    return root
