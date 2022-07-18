import trimesh
import numpy as np
import argparse
from tqdm import tqdm
from utils import mkdir
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, default="", help="File that should be processed")
    parser.add_argument("--out_dir", type=str, default="../submeshes", help="Directory where the submeshes will be stored")
    parser.add_argument("--out_filename", type=str, default="submesh", help="Name of a submesh")
    parser.add_argument("--out_type", type=str, default="ply", help="Resulting file type")
    args = parser.parse_args()
    mkdir(args.out_dir)
    mesh = trimesh.load(args.file)
    #print("Mesh has {0} points, {1} triangles".format(mesh.vertices.shape, mesh.vertex_faces.shape))
    #"""
    graph = mesh.vertex_adjacency_graph
    edges = np.asarray(graph.edges)
    nodes = np.asarray(graph.nodes)
    clusters = trimesh.graph.connected_components(edges=edges, min_len=1, nodes=nodes)
    print("Extracted {0} clusters".format(len(clusters)))
    faces_sequences = []
    for i in tqdm(range(len(clusters)), desc="Extract Faces"):
        cluster = clusters[i]
        face_list = []
        for j in range(len(cluster)):
            v_idx = cluster[j]
            face_idxs = mesh.vertex_faces[v_idx]
            face_list.extend(face_idxs)
        uni_faces = np.unique(face_list)
        uni_faces = uni_faces[uni_faces != -1]
        faces_sequence=uni_faces.tolist()
        faces_sequences.append(faces_sequence)
    submeshes = mesh.submesh(faces_sequence=faces_sequences)
    for i in tqdm(range(len(submeshes)), desc="Save Submeshes"):
        submeshes[i].export("{0}/{1}_{2}.{3}".format(args.out_dir, args.out_filename, i, args.out_type))
if __name__ == "__main__":
    main()