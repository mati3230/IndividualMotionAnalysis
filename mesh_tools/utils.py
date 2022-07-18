import numpy as np
import open3d as o3d
from tqdm import tqdm
import os.path


def mkdir(directory):
    if not os.path.isdir(directory):
        os.makedirs(directory)

pivot_dict =\
{
    "HIPS": "/",
    "HEAD": "-y",
    "SPINE": "-y",
    "CHEST": "-y",
    "NECK": "-y",
    "SHOULDER_R": "+x",
    "SHOULDER_L": "-x",
    "ARM_R":  "+x",
    "ARM_L":  "-x",
    "FOREARM_R":  "+x",
    "FOREARM_L":  "-x",
    "HAND_R": "+x",
    "HAND_L": "-x",
    "UPLEG_R":  "+y",
    "UPLEG_L": "+y",
    "LEG_R":  "+y",
    "LEG_L":  "+y",
    "FOOT_R":  "+y",
    "FOOT_L": "+y"
}


def get_vec_from(line):
    numbers = line.split(" ")
    fnumbers = [float(nr) for nr in numbers]
    fnumbers = np.array(fnumbers, dtype=np.float32)
    return fnumbers


def smaller(a, b):
    return a < b


def greater(a, b):
    return a > b


def get_pivot_from_loops(bs_name, pivot_arg, pos, loop_idxs, ply_lines, offset):
    if pivot_arg == "/":
        return pos
    axis = 0
    if pivot_arg[1] == "y":
        axis = 1
    #print(axis, pivot_arg[0])
    func = smaller
    if pivot_arg[0] == "+":
        func = greater

    idx = loop_idxs[0]
    line_idx = idx + offset
    line = ply_lines[line_idx]
    #print(line)
    p = get_vec_from(line=line)
    best_val = p[axis]
    best_p = p
    #print(best_p)
    for i in range(1, len(loop_idxs)):
        idx = loop_idxs[i]
        line_idx = idx + offset
        line = ply_lines[line_idx]
        #print(line)
        p = get_vec_from(line=line)
        val = p[axis]
        if func(val, best_val):
            best_val = val
            best_p = p
            #print(best_p)
    return best_p


def get_pivot(b_dir, bs_name, pos):
    loop_idxs = np.loadtxt(b_dir + "/" + bs_name + ".txt", dtype=np.uint32)
    if len(loop_idxs.shape) == 0:
        loop_idxs = np.array([loop_idxs])
    ply_file = open(b_dir + "/" + bs_name + ".ply")
    ply_lines = ply_file.readlines()
    #print(len(ply_lines))

    offset = 0
    for i in range(len(ply_lines)):
        line = ply_lines[i]
        line = line[:-1]
        if i == 12:
            break
        offset += 1
        if line == "end_header":
            break
    #print(offset)

    if loop_idxs.shape[0] == 1:
        line_idx = loop_idxs[0] + offset
        line = ply_lines[line_idx]
        #print(line)
        pivot = get_vec_from(line=line)
        return pivot
    elif loop_idxs.shape[0] > 1:
        if bs_name in pivot_dict:
            #print(bs_name, loop_idxs)
            pivot = get_pivot_from_loops(bs_name=bs_name, pivot_arg=pivot_dict[bs_name], pos=pos,
                loop_idxs=loop_idxs, ply_lines=ply_lines, offset=offset)
        else:
            ply_file.close()
            raise Exception("{0} unknown".format(bs_name))
            #pivot = mesh.vertices[loop_idxs[0]]
    else:

        ply_file.close()
        raise Exception("No loops")
    return pivot


"""
translation_dict =\
{
    "SPINE": "+y",
    "CHEST": "+y",
    "NECK": "+y",
    "SHOULDER_R": "-x",
    "SHOULDER_L": "+x",
    "ARM_R":  "-x",
    "ARM_L":  "+x",
    "FOREARM_R":  "-x",
    "FOREARM_L":  "+x",
    "UPLEG_R":  "-y",
    "UPLEG_L": "-y",
    "LEG_R":  "-y",
    "LEG_L":  "-y",
    "FOOT_R":  "-y",
    "FOOT_L": "-y"
}


def get_trans(bounds, co):
    #bounds(np.ndarray(2, 3))
    neg = co[0] == "-"
    axis = co[1]
    trans = np.zeros((3,), dtype=np.float32)
    axis_i = 0
    if axis == "x":
        axis_i = 0
    elif axis == "y":
        axis_i = 1
    minT = bounds[0, axis_i]
    maxT = bounds[1, axis_i]        
    trans[axis_i] = (maxT - minT) / 2
    if neg:
        trans *= -1
    return trans
"""

def file_exists(file):
    return os.path.isfile(file) 


def save_mesh(mesh, file, verbose=True):
    if verbose:
        print("Save mesh: {0}".format(file))
    o3d.io.write_triangle_mesh(file, mesh, write_vertex_normals=False, write_vertex_colors=False, write_triangle_uvs=False)
    if verbose:
        print("Done")


def load_mesh(file, verbose=True):
    if verbose:
        print("Load mesh: {0}".format(file))
    mesh = o3d.io.read_triangle_mesh(file)
    if verbose:
        print("Done")
    return mesh


def get_mesh(V, T, C, use_color=False):
    mesh = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(V),
        triangles=o3d.utility.Vector3iVector(T))
    if use_color:
        mesh.vertex_colors = o3d.utility.Vector3dVector(C)
    return mesh


def filter_single_points(mesh):
    V = np.asarray(mesh.vertices)
    C = np.asarray(mesh.vertex_colors)
    T = np.asarray(mesh.triangles)
    v_idxs = np.unique(T)
    all_v_idxs = np.arange(V.shape[0])
    if all_v_idxs.shape[0] == v_idxs.shape[0]:
        return mesh
    remaining = np.delete(all_v_idxs, v_idxs)
    
    V_, T, C_ = del_points_(V=V, T=T, C=C, idxs=remaining, del_tris=False)
    return get_mesh(V=V_, T=T, C=C_, use_color=C.shape[0] > 0)



def filter_multiple_vertices(mesh):
    print("Filter multiple vertices with the same coordinates")
    V = np.asarray(mesh.vertices)
    C = np.asarray(mesh.vertex_colors)
    T = np.asarray(mesh.triangles)
    n = V.shape[0]
    print("Input mesh has {0} vertices".format(n))
    n_vertex_idxs = []
    subst = {}
    idx = 0
    for i in tqdm(range(n)):
        if i in subst: 
            continue
        v_search = V[i]
        n_vertex_idxs.append(i)
        subst[i] = idx
        if i+1 < n:
            V_c = V[i+1:]
            res = (v_search == V_c).all(axis=1)
            #print(V_c.shape, res.shape)
            idxs = np.where(res == True)[0]
            if idxs.shape[0] > 0:
                idxs += i+1
                for v_idx in idxs:
                    subst[v_idx] = idx
        idx += 1

    n_vertices = V[n_vertex_idxs]
    n_colors = None
    if C.shape[0] > 0:
        n_vertices = V[n_vertex_idxs]
        n_colors[idx] = C[n_vertex_idxs]

    n_tris = np.zeros((T.shape[0], 3), dtype=np.uint32)
    for i in range(T.shape[0]):
        tri = T[i]
        for j in range(3):
            v_idx = tri[j]
            n_v_idx = subst[v_idx]
            tri[j] = n_v_idx
        n_tris[i] = tri
    mesh_ = get_mesh(V=n_vertices, T=n_tris, C=n_colors, use_color=C.shape[0] > 0)
    print("Output mesh has {0} vertices".format(len(n_vertex_idxs)))
    return mesh_


def pick_points_o3d(mesh):
    print("-----------------Remove Vertices-----------------")
    print("1) Pick vertices that should be deleted [shift + left click]")
    print("Press [shift + right click] to undo a selection")
    print("2) After picking the vertices, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    pp = vis.get_picked_points()
    picked_points = len(pp)*[None]
    for i in range(len(pp)):
        picked_points[i] = pp[i].index
    return picked_points


def render_o3d(mesh, w_co=False):
    if w_co:
        o3d.visualization.draw_geometries([mesh, coordinate_system()])
    else:
        o3d.visualization.draw_geometries([mesh])


def coordinate_system():
    line_set = o3d.geometry.LineSet()
    points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    lines = np.array([[0, 1], [0, 2], [0, 3]]).astype(int)
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    return line_set


def del_points_(V, T, C, idxs, del_tris=True):
    V_ = np.delete(V, idxs, axis=0)
    C_ = None
    if C.shape[0] > 0:
        C_ = np.delete(C, idxs, axis=0)

    idxs = np.sort(idxs)
    if del_tris:
        for pp in idxs[::-1]:
            # will delete pp
            rows = np.where(T == pp)[0]
            if rows.shape[0] == 0:
                continue
            T = np.delete(T, rows, axis=0)
    for pp in idxs[::-1]:
        T[T > pp] -= 1
    return V_, T, C_


def del_points(mesh, picked_points):
    V = np.asarray(mesh.vertices)
    T = np.asarray(mesh.triangles)
    C = np.asarray(mesh.vertex_colors)

    picked_points = np.unique(picked_points)

    V_, T, C_ = del_points_(V=V, T=T, C=C, idxs=picked_points, del_tris=True)
    mesh_ = get_mesh(V=V_, T=T, C=C_, use_color=C.shape[0] > 0)
    return mesh_


def center_mesh(mesh):
    verts = np.asarray(mesh.vertices)
    center = np.mean(verts, axis=0)
    verts -= center
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    return mesh


def align(mesh):
    obb = mesh.get_oriented_bounding_box()
    rot_mat = np.asarray(obb.R)
    mesh.rotate(np.linalg.inv(rot_mat))
    #obb.rotate(np.linalg.inv(rot_mat))
    #o3d.visualization.draw_geometries([mesh, obb, coordinate_system()])
    return mesh