"""
Rewrite <mesh filename="package://.../meshes/hand_v0/x.dae"> references in a
flattened URDF to plain relative paths ("meshes/x.dae"), so the file can be
read by tools outside ROS that don't understand package:// URIs. Used by
export.sh after xacro expansion.

Usage: python3 rewrite_mesh_paths.py <in.urdf> <out.urdf>
"""

import sys
import xml.etree.ElementTree as ET


def main():
    in_path, out_path = sys.argv[1], sys.argv[2]
    tree = ET.parse(in_path)

    for mesh in tree.getroot().iter("mesh"):
        uri = mesh.get("filename")
        if uri and uri.startswith("package://"):
            mesh.set("filename", "meshes/" + uri.rsplit("/", 1)[-1])

    tree.write(out_path, xml_declaration=True, encoding="utf-8")


if __name__ == "__main__":
    main()
