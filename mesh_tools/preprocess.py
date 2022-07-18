import argparse

from utils import load_mesh,\
    save_mesh,\
    filter_multiple_vertices,\
    pick_points_o3d,\
    render_o3d,\
    file_exists,\
    del_points,\
    filter_single_points,\
    center_mesh,\
    align


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, default="", help="File that should be processed")
    parser.add_argument("--ext", type=str, default="ply", help="Extension of the output file.")
    parser.add_argument("--glb", type=bool, default=False, help="If True, processed meshes will also be stored as glb.")
    parser.add_argument("--nr", type=int, default=-1, help="Set to a preprocessed file number.")
    args = parser.parse_args()
    
    raw_filename = args.file[:-4]
    if args.nr > -1:
        n_filename = "{0}_filtered_{1:03d}.{2}".format(raw_filename, args.nr, args.ext)
    else:
        n_filename = "{0}_filtered.{1}".format(raw_filename, args.ext)
    if file_exists(n_filename):
        mesh_ = load_mesh(file=n_filename)
        #render_o3d(mesh=mesh_, w_co=True)
    else:
        mesh = load_mesh(file=args.file)
        #render_o3d(mesh=mesh, w_co=True)
        mesh_ = filter_multiple_vertices(mesh=mesh)
        mesh_ = filter_single_points(mesh=mesh_)
        mesh_ = center_mesh(mesh=mesh_)
        mesh_ = align(mesh=mesh_)
        #return
        save_mesh(mesh=mesh_, file=n_filename)
    
    inpt = ""
    nr = 0
    while inpt != "e":
        points = pick_points_o3d(mesh=mesh_)
        if len(points) > 0:
            mesh_ = del_points(mesh=mesh_, picked_points=points)
            mesh_ = filter_single_points(mesh=mesh_)
            mesh_ = center_mesh(mesh=mesh_)
            mesh_ = align(mesh=mesh_)
            #"""
            if args.ext == "glb":
                n_filename = "{0}_filtered_{1:03d}.{2}".format(raw_filename, nr, "glb")
                save_mesh(mesh=mesh_, file=n_filename)
            else:
                if args.glb:
                    n_filename = "{0}_filtered_{1:03d}.{2}".format(raw_filename, nr, "glb")
                    save_mesh(mesh=mesh_, file=n_filename)
                n_filename = "{0}_filtered_{1:03d}.{2}".format(raw_filename, nr, args.ext)
                save_mesh(mesh=mesh_, file=n_filename)
            nr += 1
            #"""
        inpt = input("Enter 'e' to escape: ")



if __name__ == "__main__":
    main()