#!/usr/bin/env python3
"""Project-local Ogre 2.3 texture-buffer fallback repair for UTM Core OpenGL.

The upstream compatibility layer maps samplerBuffer to sampler2D when
GL_ARB_texture_buffer_range is missing. All scalar/read-only fetch macros must
then use bufferFetch, which translates the linear index to 2D coordinates.
On drivers supporting the extension bufferFetch is an alias for texelFetch.
System ROS/Gazebo files are never modified.
"""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz/gz-rendering8')
DEST = ROOT / 'runtime-resources/ogre2-utm'
REL = Path('ogre2/media/Hlms/Common/GLSL/CrossPlatformSettings_piece_all.glsl')
replacements = {
    '#define bufferFetch1( buffer, idx ) texelFetch( buffer, idx ).x':
    '#define bufferFetch1( buffer, idx ) bufferFetch( buffer, idx ).x',
    '#define readOnlyFetch( buffer, idx ) texelFetch( buffer, idx )':
    '#define readOnlyFetch( buffer, idx ) bufferFetch( buffer, idx )',
    '#define readOnlyFetch1( buffer, idx ) texelFetch( buffer, idx ).x':
    '#define readOnlyFetch1( buffer, idx ) bufferFetch( buffer, idx ).x',
}
original = (SOURCE / REL).read_text()
patched = original
for old, new in replacements.items():
    if original.count(old) != 1:
        raise SystemExit(f'Installed shaders changed; review patch before applying: {old}')
    patched = patched.replace(old, new)
shutil.copytree(SOURCE, DEST, dirs_exist_ok=True)
(DEST / REL).write_text(patched)
(DEST / 'patch-manifest.json').write_text(json.dumps({
    'source': str(SOURCE / REL),
    'source_sha256': hashlib.sha256(original.encode()).hexdigest(),
    'patched_sha256': hashlib.sha256(patched.encode()).hexdigest(),
    'replacements': replacements,
}, indent=2))
print(DEST)
