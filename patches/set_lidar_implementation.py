"""Point the Kraken lidars at the GPU implementation.

The prefabs ship with "Scene Queries", which raycasts PhysX colliders. RGL
raycasts the visual meshes on the GPU instead, which is what this project needs
to see the orchard.

Prefab edits do not reach the running game until the assets are rebaked:

    AssetProcessorBatch --project-path=/o3de/ROSConDemo/Project
"""
import os
import pathlib
import shutil

OLD = '"lidarImplementation": "Scene Queries"'
NEW = '"lidarImplementation": "RobotecGPULidar"'

root = pathlib.Path(os.environ.get("O3DE_DIR", "/o3de"))
prefabs = [
    root / "ROSConDemo/Project/Assets/Kraken/apple_kraken_v1/apple_kraken_v1.prefab",
    root / "ROSConDemo/Project/Assets/Kraken/apple_kraken_v2/apple_kraken_v2.prefab",
]

for prefab in prefabs:
    text = prefab.read_text()
    if NEW in text:
        print("already RGL:", prefab.name)
        continue
    if text.count(OLD) != 1:
        raise SystemExit("%s: expected one %r, found %d" % (prefab.name, OLD, text.count(OLD)))
    backup = prefab.with_suffix(".prefab.pre-rgl")
    if not backup.exists():
        shutil.copy2(prefab, backup)
    prefab.write_text(text.replace(OLD, NEW))
    print("switched to RGL:", prefab.name)
