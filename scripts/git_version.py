"""Injection de la version Git dans le firmware (include/git_version.h).

Exécuté par PlatformIO avant la compilation (extra_scripts = pre:...) :
récupère `git describe --tags --always --dirty` à la racine du projet et
génère include/git_version.h définissant la macro GIT_VERSION sous forme de
littéral de chaîne C (ex. "v1.2.0-4-g1a2b3c-dirty").

Pourquoi un en-tête généré plutôt que -D GIT_VERSION="..." sur la ligne de
compilation : la valeur contient des caractères non identifiants (tirets,
points) et la couche SCons/PlatformIO altère les guillemets des CPPDEFINES
selon la plateforme — le corps de macro peut arriver déquoté (erreur
« numeric literal operator » à la compilation). Un en-tête régénéré à
chaque build est déterministe et indépendant de l'échappement shell.

Le fichier n'est réécrit que si son contenu change (builds incrémentaux
préservés). Fallback "dev" si git est absent ou si le projet n'est pas un
dépôt Git — le build ne doit jamais échouer ici. Fichier ignoré par Git.
"""

import os
import subprocess

Import("env")  # noqa: F401 — fourni par PlatformIO dans les extra_scripts

HEADER_PATH = os.path.join(str(env["PROJECT_INCLUDE_DIR"]), "git_version.h")


def _git_version():
    """Retourne la chaîne de version Git du projet, ou "dev" en échec."""
    try:
        out = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=str(env["PROJECT_DIR"]),
            capture_output=True,
            text=True,
            timeout=10,
        )
        version = out.stdout.strip()
        if out.returncode == 0 and version:
            return version
    except Exception:
        pass
    return "dev"


GIT_VERSION = _git_version()

# Échappement C minimal (un tag contenant " ou \ est quasi impossible, mais
# jamais assez prudent sur un littéral généré).
safe_version = GIT_VERSION.replace("\\", "\\\\").replace('"', '\\"')

header = (
    "/* Fichier GÉNÉRÉ automatiquement par scripts/git_version.py à chaque\n"
    " * build PlatformIO (extra_scripts = pre:) — ne pas éditer ni versionner.\n"
    " * Macro GIT_VERSION : sortie de git describe --tags --always --dirty. */\n"
    "#pragma once\n"
    '#define GIT_VERSION "{}"\n'.format(safe_version)
)

# Réécriture seulement si le contenu change : évite d'invalider inutilement
# les builds incrémentaux quand la version n'a pas bougé.
try:
    with open(HEADER_PATH, "r", encoding="utf-8") as f:
        existing = f.read()
except OSError:
    existing = None
if existing != header:
    with open(HEADER_PATH, "w", encoding="utf-8") as f:
        f.write(header)

print('BOB-CONTROL : GIT_VERSION="{}" -> {}'.format(GIT_VERSION, HEADER_PATH))
