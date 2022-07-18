import open3d as o3d
import argparse
import os
import numpy as np

from utils import load_mesh, save_mesh#, translation_dict, get_trans


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fdir", type=str, default="", help="Directory to the 3d files")
    parser.add_argument("--inext", type=str, default="ply", help="Input file extension")
    parser.add_argument("--outext", type=str, default="stl", help="Output file extension")
    args = parser.parse_args()
    if args.fdir == "":
        raise Exception("No file directory specified")

    if args.inext == args.outext:
        raise Exception("Input and output extension are the same")
    r = len(args.inext)
    len_ending = r + 1 # e.g. .ply
    repaired_end = "_repaired"
    len_repaired = len(repaired_end)
    files_converted = 0
    for file in os.listdir(args.fdir):
        if not file.endswith(args.inext):
            continue
        if not file[:-len_ending].endswith(repaired_end):
            continue
        filename = args.fdir + "/" + file
        mesh = load_mesh(file=filename, verbose=False)
        verts = np.asarray(mesh.vertices)
        verts -= np.mean(verts, axis=0)
        file = file[:-(len_ending + len_repaired)]
        """
        if file in translation_dict:
            min_bounds = mesh.get_min_bound()
            max_bounds = mesh.get_max_bound()
            bounds = np.vstack((min_bounds[None, :], max_bounds[None, :]))
            trans = get_trans(bounds, translation_dict[file])
            verts += trans
        """
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.compute_vertex_normals()
        filename = args.fdir + "/" + file + "." +  args.outext
        save_mesh(mesh=mesh, file=filename, verbose=False)
        files_converted += 1
    print("{0} files converted".format(files_converted))


if __name__ == "__main__":
    main()