import trimesh
import os
import argparse
import h5py
import numpy as np
from tqdm import tqdm
import utils


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--b_dir", default="./BodyParts", type=str, help="Directory where the body parts are stored.")
    parser.add_argument("--disable_tqdm", type=bool, default=False, help="Disable tqdm bar")
    args = parser.parse_args()

    ply_files = os.listdir(args.b_dir)
    hf = h5py.File("{0}/props.h5".format(args.b_dir), "w")
    for i in tqdm(range(len(ply_files)), desc="Calculate mesh properties", disable=args.disable_tqdm):
        ply_file = "{0}/{1}".format(args.b_dir, ply_files[i])
        if not ply_file.endswith(".ply"):
            continue
        if ply_file.endswith("_repaired.ply"):
            continue
        #print("Load '{0}'".format(ply_file))
        mesh = trimesh.load(ply_file)
        bs_name = ply_file[:-4]
        bs_name = bs_name.split("/")[-1]
        if not mesh.is_watertight:
            print("Body segment '{0}' is not watertight".format(bs_name))
            continue
        #print("Calculate moment inertia for '{0}'".format(bs_name))
        trimesh.repair.fix_normals(mesh)

        pos = np.mean(mesh.vertices, axis=0)
        pivot = utils.get_pivot(b_dir=args.b_dir, bs_name=bs_name, pos=pos)
        
        mesh.vertices -= pos
        inertia = mesh.moment_inertia
        
        """
        trans = np.zeros((3, ), dtype=np.float32)
        if bs_name in utils.translation_dict:
            bounds = mesh.bounds
            trans = utils.get_trans(bounds, utils.translation_dict[bs_name])
            #pos += trans
        """
        #print(bs_name)
        hf.create_dataset(bs_name + "_Inertia", data=inertia)
        hf.create_dataset(bs_name + "_Position", data=pos)
        hf.create_dataset(bs_name + "_MassCenter", data=mesh.center_mass)
        hf.create_dataset(bs_name + "_Volume", data=mesh.volume)
        hf.create_dataset(bs_name + "_Bounds", data=mesh.bounds)
        hf.create_dataset(bs_name + "_Pivot", data=pivot)
        #hf.create_dataset(bs_name + "_Trans", data=trans)

        mesh.vertices += pos
        mesh.export("{0}/{1}_repaired.ply".format(args.b_dir, bs_name))
        #mesh.center_mass
    hf.close()


if __name__ == "__main__":
    main()