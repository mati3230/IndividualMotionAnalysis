using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Globalization;

namespace BodyPartSegmentation{
    
    public class Frame{
        // container that stores frame
        public Vector3 pos{get;}
        public Quaternion rot{get;}
        public int nr{get;}
        public float duration{get;}
        public Frame(Vector3 pos, Quaternion rot, int nr, float duration){
            this.pos = pos;
            this.rot = rot;
            this.nr = nr;
            this.duration = duration;
        }
    }


    public class Edge{
        // Assignment of edge in the mesh to a list of triangles (edge can be shared by multiple triangles)
        // Note that the triangles are stored as indeces
        public int i{get;}
        public int j{get;}
        public List<int> triangles{get;set;}
        public Edge(Edge e){
            this.i = e.i;
            this.j = e.j;
            this.triangles = new List<int>(e.triangles);
        }
        public Edge(int i, int j){
            this.i = i;
            this.j = j;
            this.triangles = new List<int>();
        }
        public Edge(int i, int j, int tIdx){
            // Initialize the list of triangles with the triangle with the index tIdx
            this.i = i;
            this.j = j;
            this.triangles = new List<int>(){tIdx};
        }

        public void AddT(int tIdx){
            // Add a triangle index
            if(!this.triangles.Contains(tIdx))
                this.triangles.Add(tIdx);
        }


        private static bool FindLoop(List<Edge> edges, List<int> edgeIdxs, out List<int> loop){
            // Finds loops (holes) in a mesh - a loop is defined as list of adjacent edges where every edge only shares one face
            // Insert a list of (border) edges with their corresponding edge indices (id's)
            loop = new List<int>(){edgeIdxs[0]};
            if (edgeIdxs.Count <= 2) return false; // need at least 3 edges for a loop
            // flag that indicates if any loop (path) was found
            bool loopFound = false;
            // first edge will be the start edge
            int startVertex = edges[edgeIdxs[0]].j;
            int stopVertex = edges[edgeIdxs[0]].i;
            int nextConnectionVertex = startVertex;
            // we search the (border) edges till a loop can be rejected
            bool loopIsPossible = true;
            
            while(loopIsPossible){
                // search for a connection (adjacant edge) to last start, i.e. an edge (i, j) should continue by (j, k)
                bool connection = false;
                for (int i=1; i<edgeIdxs.Count; i++){ // we can skip the start (first) edge
                    int eIdx = edgeIdxs[i];
                    if(loop.Contains(eIdx)) continue; // edge is already in loop
                    Edge edge = edges[eIdx];
                    // determine if the edge is an adjacent edge
                    if (edge.i == nextConnectionVertex){
                        nextConnectionVertex = edge.j; // for the next iteration
                        loop.Add(eIdx); // add this edge to the edges of the loop
                        connection = true;
                        break;
                    }
                    else if(edge.j == nextConnectionVertex){
                        nextConnectionVertex = edge.i;
                        loop.Add(eIdx);
                        connection = true;
                        break;
                    }
                }
                if(connection){ // if we found an adjacent (follow up) edge
                    // check if we reached the end of the loop
                    if(loop.Count >= 2 && nextConnectionVertex == stopVertex){
                        loopIsPossible = false;
                        loopFound = true;
                    }
                }
                else{ // no loop - at least one edge is missing in the loop 
                    loopIsPossible = false;
                    loopFound = false;
                }
            }
            return loopFound;
        }


        public static bool IsLoop(List<Edge> edges, out List<List<int>> edgeLoops){
            // Finds loops (holes) in a mesh - a loop is a list of edges
            // The edges should be border edges
            edgeLoops = new List<List<int>>();
            if (edges.Count <= 2) return false; // need at least 3 edges for a loop
            
            // check for the same edge multiple times
            for(int i=0; i<edges.Count; i++){
                for(int j=0; j<edges.Count; j++){
                    if (i==j) continue;
                    if(edges[i] == edges[j])
                        return false;
                }   
            }

            // flag that indicates if any loop was found
            bool anyLoopFound = false;

            // store the edge idxs (edge id) of each edge
            List<int> edgeIdxs = new List<int>();
            for(int i=0; i<edges.Count; i++){
                edgeIdxs.Add(i);
            }

            // we search for multiple loops
            while(edgeIdxs.Count >= 2){ // if a loop is still possible
                List<int> loop = null;
                bool loopFound = FindLoop(edges, edgeIdxs, out loop);
                if (loopFound){
                    // store the loop
                    edgeLoops.Add(new List<int>(loop));
                    anyLoopFound = true;
                }
                // edges of the loop do not have to be considered for the next iteration
                foreach(int eIdx in loop){
                    edgeIdxs.Remove(eIdx);
                }
            }
            return anyLoopFound;
        }

        public static bool operator ==(Edge a, Edge b){
            // Convinience function to compare two edges
            return ((a.i == b.i) && (a.j == b.j)) || 
                ((a.j == b.i) && (a.i == b.j));
        }

        public static bool operator !=(Edge a, Edge b){
            // Convinience function to compare two edges
            bool equal = a == b;
            return !equal;
        }

        public override bool Equals(object obj){
            // Convinience function to compare two edges
            if(!(obj is Edge))
                return false;
            Edge edge = (Edge) obj;
            return this == edge;
        }

        public override int GetHashCode(){
            return base.GetHashCode();
        }
    }

    public class Utils
    {

        public static void FilterMultipleVertices(Vector3[] vertices, int[] tris, out Vector3[] nVertices, out int[] nTris){
            // we will have |V'| <= |V| but |T'| == |T|, i.e. we have equal or less number of vertices but the same number of triangles
            nVertices = null;
            nTris = new int[tris.Length];

            // unique vertex indices of the old vertices array 
            List<int> nVertexIdxs = new List<int>();
            // new vertex index (value) per old vertex index (key)
            Dictionary<int, int> subst = new Dictionary<int, int>();
            int idx = 0;
            for(int i=0; i< vertices.Length; i++){
                // skip vertices that we already considered
                if (subst.ContainsKey(i)) continue;
                Vector3 vSearch = vertices[i];
                
                // add unique old vertex index
                nVertexIdxs.Add(i);
                // substitution of the i-th vertex has the index idx
                subst.Add(i, idx);
                
                if (i+1 < vertices.Length){
                    for(int j=i+1; j<vertices.Length; j++){
                        //if (i == j) continue; // same i-th vertex - skip
                        Vector3 vC = vertices[j]; // extract the j-th vertex
                        if (vC != vSearch) continue; // this is another vertex - skip
                        subst.Add(j, idx); // the j-th vertex has also the substitution idx
                    }
                }
                // consider the next unique vertex with the index idx++
                idx++;
            }

            // we store the found unique vertices (in nVertexIdxs) in nVertices
            nVertices = new Vector3[nVertexIdxs.Count];
            idx = 0;
            foreach(int i in nVertexIdxs){
                nVertices[idx] = vertices[i];
                idx++;
            }

            // we substitute each vertex index with its unique index counterpart
            for(int i = 0; i < tris.Length; i++){
                int vIdx = tris[i]; // extract the old vertex index
                int n_vIdx = subst[vIdx]; // query the susbtitution
                nTris[i] = n_vIdx; // assign the unique index
            }
            //Debug.LogFormat("Filtered {0} double vertices", vertices.Length - nVertices.Length);
        }   


        private static bool TryAddInVertex(ref Dictionary<int, int> triIdxMapping, ref List<Vector3> iVertices,
                ref int vIdx, ref Vector3 center, int i1, Vector3 v1){
            if(triIdxMapping.ContainsKey(i1)) return false;

            triIdxMapping.Add(i1, vIdx);
            iVertices.Add(v1);
            center += v1;
            vIdx++;
            return true;
        }


        private static void AddEdge(ref List<Edge> edges, Edge e, int tIdx){
            bool found = false;
            for (int i=0; i<edges.Count; i++){
                Edge edge = edges[i];
                if(edge != e) continue;
                found = true;
                edge.AddT(tIdx);
                edges[i] = edge;
            }
            if (found) return;
            edges.Add(e);
        }


        private static void AddEdges(ref List<Edge> edges, Vector3Int tri, int tIdx){
            AddEdge(ref edges, new Edge(tri.x, tri.y, tIdx), tIdx);
            AddEdge(ref edges, new Edge(tri.x, tri.z, tIdx), tIdx);
            AddEdge(ref edges, new Edge(tri.y, tri.z, tIdx), tIdx);
        }


        public static bool TrySaveIntersection(string filename, Vector3[] verticesA, int[] trisA, SegmentationBox segBox, bool centered){
            // crossing triangle := triangle that crosses a segmentation plane
            //SegmentationBox segBox = new SegmentationBox(box);
            //Debug.Log(trisA.Length);

            // every vertex that is in the segmentation box
            List<Vector3> iVertices = new List<Vector3>();
            // every triangle that in the segmentation box
            List<Vector3Int> iTris = new List<Vector3Int>();

            // mapping from old vertex index to a new vertex index (substitution)
            Dictionary<int, int> triIdxMapping = new Dictionary<int, int>();
            int vIdx = 0; // new vertex index
            // vertex center of the segmentation
            Vector3 center = Vector3.zero;

            List<Edge> edges = new List<Edge>();
            for(int i=0; i<trisA.Length; i+=3){
                // extract the triangle and corresponding vertices
                int i0 = trisA[i+0];
                int i1 = trisA[i+1];
                int i2 = trisA[i+2];
                Vector3 v0 = verticesA[i0];
                Vector3 v1 = verticesA[i1];
                Vector3 v2 = verticesA[i2];
                // determine triangles that are fully within the segmentation box
                bool inTri = segBox.InPoint(v0) && segBox.InPoint(v1) && segBox.InPoint(v2);
                if(!inTri) continue;
                
                // store the vertices
                TryAddInVertex(ref triIdxMapping, ref iVertices, ref vIdx, ref center, i0, v0);
                TryAddInVertex(ref triIdxMapping, ref iVertices, ref vIdx, ref center, i1, v1);
                TryAddInVertex(ref triIdxMapping, ref iVertices, ref vIdx, ref center, i2, v2);

                // store the triangle with the new indices
                int i0n = triIdxMapping[i0];
                int i1n = triIdxMapping[i1];
                int i2n = triIdxMapping[i2];
                Vector3Int nTri = new Vector3Int(i0n, i1n, i2n);

                AddEdges(ref edges, nTri, iTris.Count);

                iTris.Add(nTri);
            }
            center /= (float) iVertices.Count;

            // determine triangles that are only shared by one face (border edges)
            List<Edge> remainingEdges = new List<Edge>();
            for(int i=0; i<edges.Count; i++){
                Edge edge = edges[i];
                if(edge.triangles.Count == 1){
                    remainingEdges.Add(edge);
                }
            }

            //Debug.LogFormat("Try to find loops with {0} edges", remainingEdges.Count);
            List<List<int>> edgeLoops = null;
            bool loopsFound = Edge.IsLoop(remainingEdges, out edgeLoops);

            List<int> loop_center_idxs = new List<int>();
            //Debug.LogFormat("Found {0} loops", edgeLoops.Count);
            for(int i=0; i< edgeLoops.Count; i++){
                //Debug.LogFormat("Loop {0} has {1} edges", i, edgeLoops[i].Count);
                
                List<int> loop = edgeLoops[i];
                int loop_center_idx = CloseLoop(edgeLoops[i], remainingEdges, center, ref iVertices, ref iTris);
                loop_center_idxs.Add(loop_center_idx);
            }

            if(iTris.Count == 0)
                return false;

            if(centered){
                for(int i=0; i<iVertices.Count; i++){
                    iVertices[i] -= center;
                }
            }

            // trimesh (python) library expects an inverted z axis
            for(int i=0; i<iVertices.Count; i++){
                Vector3 v = iVertices[i];
                iVertices[i] = new Vector3(v.x, v.y, v.z);
            }

            TryWriteCSV(filename + ".txt", loop_center_idxs);
            WritePLY(filename + ".ply", iVertices, iTris);
            return true;
        }


        public static int CloseLoop(List<int> loop, List<Edge> edges, Vector3 boxCenter, 
                ref List<Vector3> verts, ref List<Vector3Int> tris){
            // Closes a loop (hole) by determining the center of the loop (hole) and adds triangles that connect the (border) edges with the center
            int idx = -1;
            Vector3 center = Vector3.zero;
            List<int> eIdxs = new List<int>();
            int cIdx = verts.Count;
            
            // determine the center of the loop (hole)
            // for each edge of the loop (hole)
            for(int i=0; i<loop.Count; i++){
                // extract the (border) edge
                int eIdx = loop[i];
                Edge edge = edges[eIdx];

                // consider all vertices of the loop once
                int ei = edge.i;
                int ej = edge.j;
                Vector3 vi = verts[ei];
                Vector3 vj = verts[ej];
                if(!eIdxs.Contains(ei)){
                    eIdxs.Add(ei);
                    center += vi;
                }
                if(!eIdxs.Contains(ej)){
                    eIdxs.Add(ej);
                    center += vj;
                }
            }
            
            // center is the average of all vertices in the loop
            center /= (float) eIdxs.Count;
            // add the center as new vertex
            verts.Add(center);
            idx = verts.Count - 1;

            // calculate the triangles between the (border) edges and the center vertex
            for(int i=0; i<loop.Count; i++){
                // vertices of the (border) edge
                int eIdx = loop[i];
                Edge edge = edges[eIdx];
                int ei = edge.i;
                int ej = edge.j;
                Vector3 vi = verts[ei];
                Vector3 vj = verts[ej];

                // create the triangle with the center vertex
                Vector3Int nTri = OrientTri(vi, vj, center, boxCenter, ei, ej, cIdx);
                tris.Add(nTri);
            }
            return idx;
        }


        public static Vector3Int OrientTri(Vector3 vi, Vector3 vj, Vector3 vC, Vector3 center, int i, int j, int c){
            // Orient a triangle such that its normal points outwards of the segment

            // determine the center of the triangle
            Vector3 centerTri = (vi + vj + vC) / 3f;
            // vector that points from the center of the segment to the center of the triangle
            centerTri = (centerTri - center).normalized;

            // normal of the triangle
            Vector3 n = NormalTriangle(vC, vi, vj);
            // determine the angle between the normal of the triangle and the centerTri vector (center segment -> center triangle)
            float angle = Vector3.Dot(n, centerTri) / (n.magnitude * centerTri.magnitude);
            angle = 180f*Mathf.Acos(angle) / Mathf.PI;
            
            // normal of the triangle and centerTri vector should approx. point in the same direction 
            if(angle < 90f) return new Vector3Int(c, i, j);
            return new Vector3Int(c, j, i);
        }


        public static Vector3 NormalTriangle(Vector3 p0, Vector3 p1, Vector3 p2){
            // determines the normal of a triangle
            Vector3 u = p1 - p0;
            Vector3 v = p2 - p0;
            Vector3 n = Vector3.Cross(u, v).normalized;
            return n;
        }


        public static void WritePLY(string filename, List<Vector3> vertices, List<Vector3Int> triangles){
            // writes the vertices and triangles as .ply (see https://de.wikipedia.org/wiki/Polygon_File_Format)
            List<string> data = new List<string>{
                "ply",
                "format ascii 1.0",
                "element vertex " + vertices.Count.ToString(),
                "property float x",
                "property float y",
                "property float z",
                "element face " + triangles.Count.ToString(),
                "property list uchar int vertex_indices",
                "end_header"
            };

            // write floating point numbers with a point (e.g. 4.234 instead of 4,234)
            CultureInfo ci = CultureInfo.CreateSpecificCulture("en-US");

            for(int i=0; i<vertices.Count; i++){
                Vector3 v = vertices[i];
                data.Add(v.x.ToString(ci) + " " + v.y.ToString(ci) + " " + v.z.ToString(ci));
            }

            for(int i=0; i<triangles.Count; i++){
                Vector3Int tri = triangles[i];
                data.Add("3 " + tri.x.ToString() + " " + tri.y.ToString() + " " + tri.z.ToString());
            }

            string dataStr = "";
            for(int i=0; i<data.Count; i++){
                dataStr += data[i] + "\n";
            }

            File.WriteAllText(filename, dataStr);
        }


        public static bool IsCSV(string fileName)
        {
            // check if a file is csv
            if (fileName == string.Empty)
            {
                return false;
            }
            if (fileName.Length <= 4)
            {
                return false;
            }
            if (!fileName.EndsWith(".csv"))
            {
                return false;
            }
            return true;
        }

        public static bool TryWriteCSV(string filename, List<int> loop_center_idxs){
            string out_string = "";
            for(int i = 0; i < loop_center_idxs.Count; i++){
                int idx = loop_center_idxs[i];
                out_string += idx.ToString() + "\n";
            }
            try{
                File.WriteAllText(filename, out_string);
            }
            catch(System.Exception e){
                Debug.LogError(e.Message);
                return false;
            }
            return true;
        }


        public static bool TryReadCSV(string filename, out Dictionary<string, Frame[]> animation, out int nFrames){
            // Reads a csv with the MMH animation and stores is as array of frames per joint (Dictionary<string, Frame[]> animation)
            animation = new Dictionary<string, Frame[]>();
            nFrames = 0;
            if (!IsCSV(filename)) return false;
            if (!File.Exists(filename)) return false;

            CultureInfo ci = CultureInfo.CreateSpecificCulture("en-US");
            // string containing the whole csv file
            string csvData = File.ReadAllText(filename);
            // split it into rows
            string[] rows = csvData.Split('\n');

            // header row
            string header = rows[0];
            // column names
            string[] header_elements = header.Split(',');

            nFrames = rows.Length - 2;

            // extract the body parts in the csv from the header
            List<string> bps = new List<string>();
            for(int i = 1; i < header_elements.Length; i++){ // skip first column (frame duration)
                string he = header_elements[i];
                string[] bodyParts = he.Split('_');
                string bodyPart = "";
                if (bodyParts.Length == 2)
                    bodyPart = bodyParts[0].Remove(0, 1);
                else{
                    bodyPart += bodyParts[0].Remove(0, 1);
                    for(int j = 1; j < bodyParts.Length - 1; j++){
                        bodyPart += "_" + bodyParts[j];
                    }
                }
                if (bodyPart == "")
                    continue;
                if (animation.ContainsKey(bodyPart))
                    continue;
                animation.Add(bodyPart, new Frame[nFrames]);
                bps.Add(bodyPart);
                //Debug.LogFormat("Add body part '{0}'", bodyPart);
            }

            // extract the frames of the body parts
            for(int i = 1; i < rows.Length-1; i++){
                string row = rows[i];
                string[] fdata = row.Split(',');
                //Debug.Log(fdata[0]);
                float duration = float.Parse(fdata[0], ci);
                int frameNr = i - 1;
                int k = 0;
                for(int j = 1; j < fdata.Length; j+=7){
                    string bodyPart = bps[k];
                    Vector3 pos = new Vector3(float.Parse(fdata[j+0], ci), float.Parse(fdata[j+1], ci), float.Parse(fdata[j+2], ci));
                    Quaternion rot = new Quaternion(float.Parse(fdata[j+3], ci), float.Parse(fdata[j+4], ci), float.Parse(fdata[j+5], ci), float.Parse(fdata[j+6], ci));
                    Frame frame = new Frame(pos, rot, frameNr, duration);
                    animation[bodyPart][frameNr] = frame;
                    k++;
                }
            }
            return true;
        }


        public static void AddBody(Transform T, ref Dictionary<string, Transform> bodies){
            for(int i=0; i<T.childCount; i++){
                Transform childT = T.GetChild(i);
                string bodyPart = childT.name;
                bodies.Add(bodyPart, childT);
                //Debug.LogFormat("Add body: {0} ({1})", bodyPart, childT.gameObject.name);
                AddBody(childT, ref bodies);
            }
        }


        public static bool AssignBodyParts(Transform T, out Dictionary<string, Transform> bodies){
            bodies = new Dictionary<string, Transform>();
            if (T.childCount == 0)
                return false;
            AddBody(T, ref bodies);
            /*foreach(KeyValuePair<string, Transform> bp in bodies){
                Debug.LogFormat("Added body part '{0}'", bp.Key);
            }*/
            return true;
        }


        public static Vector3 Abs(Vector3 v){
            return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
        }


        // Dictionary in order to determine if segmentation box should be scaled horizontally (x direction) or vertically (y direction)
        public static Dictionary<string, bool> Part2XScale = new Dictionary<string, bool>(){
            {"HIPS", false},
            {"CHEST", false},
            {"HEAD", false},
            {"HAND_R", true}, 
            {"HAND_L", true},
            {"FOOT_L", false},
            {"FOOT_R", false},
            {"LEG_L", false},
            {"LEG_R", false},
            {"UPLEG_L", false},
            {"UPLEG_R", false},
            {"SPINE", false},
            {"NECK", false},
            {"SHOULDER_L", true},
            {"SHOULDER_R", true},
            {"ARM_L", true},
            {"ARM_R", true},
            {"FOREARM_L", true},
            {"FOREARM_R", true}
        };

        // Default scale of each body part -- will be applied where Part2XScale is false (e.g. should be large enough for the radius of hip segment)
        public static Dictionary<string, float> DefaultScale = new Dictionary<string, float>(){
            {"HIPS", 0.35f},
            {"CHEST", 0.35f},
            {"HEAD", 0.1f},
            {"HAND_R", 0.1f}, 
            {"HAND_L", 0.1f},
            {"FOOT_L", 0.1f},
            {"FOOT_R", 0.1f},
            {"LEG_L", 0.2f},
            {"LEG_R", 0.2f},
            {"UPLEG_L", 0.2f},
            {"UPLEG_R", 0.2f},
            {"SPINE", 0.35f},
            {"NECK", 0.35f},
            {"SHOULDER_L", 0.2f},
            {"SHOULDER_R", 0.2f},
            {"ARM_L", 0.2f},
            {"ARM_R", 0.2f},
            {"FOREARM_L", 0.2f},
            {"FOREARM_R", 0.2f}
        };


        public static void ApplyFrame(Dictionary<string, Transform> bodies, Dictionary<string, Frame[]> animation, float framePos, int nFrames){
            // Apply MMH frame to segmentation boxes so that they are automatically positioned
            
            // get the absolute frame number
            int frameNr = Mathf.FloorToInt(framePos * (nFrames - 1));
            // rotation per body part of the target frame
            Dictionary<string, Quaternion> rotations = new Dictionary<string, Quaternion>();
            
            foreach(KeyValuePair<string, Transform> bpT in bodies){
                string bodyPart = bpT.Key;
                Transform T = bpT.Value;
                if(!animation.ContainsKey(bodyPart)){
                    throw new System.Exception("Do not have data of body part: " + bodyPart);
                }
                Frame[] frames = animation[bodyPart];
                Frame frame = frames[frameNr];
                // position of the box
                T.position = frame.pos;

                //T.rotation = frame.rot;
                rotations.Add(bodyPart, frame.rot);
            }

            // we store the hips position in order to translate the debugging spheres (see in foreach loop below)
            Vector3 hipsPos = bodies["HIPS"].position;
            // translate the hip in the origin (this also translate the child boxes)
            bodies["HIPS"].position = Vector3.zero;
            // we calculate the mean position of a joint's position and its child joints positions as center of a segmentation box for a joint 
            Dictionary<string, Vector3> meanPositions = new Dictionary<string, Vector3>();
            // scale of a box in the x or y direction
            Dictionary<string, float> scales = new Dictionary<string, float>();
            // for every body part
            foreach(KeyValuePair<string, Transform> bpT in bodies){
                string bodyPart = bpT.Key;
                
                // Create a sphere for every joint position in order to validate the positioning of the segmentation boxes
                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.position = animation[bodyPart][frameNr].pos - hipsPos;
                sphere.transform.localScale *= 0.05f;
                sphere.name = bodyPart + "_MMH";

                // calculate the mean position
                Transform T = bpT.Value;
                int nChildren = T.childCount;
                Vector3 meanPos = T.position;
                for(int i=0; i<nChildren; i++){
                    Transform childT = T.GetChild(i);
                    Vector3 pos = childT.position;                    
                    meanPos += pos;
                }
                meanPos /= (float) (nChildren + 1);
                
                // determine the maximum deviation of the mean position to considered joints
                Vector3 diff = Vector3.zero;
                diff = meanPos - T.position;
                diff = Abs(diff);
                for(int i=0; i<nChildren; i++){
                    Transform childT = T.GetChild(i);
                    Vector3 pos = childT.position;
                    Vector3 tmpDiff = meanPos - pos;
                    tmpDiff = Abs(tmpDiff);
                    diff = Vector3.Max(diff, tmpDiff);
                }

                // store the mean positions
                meanPositions.Add(bodyPart, meanPos);
                float scale = 0.1f;
                if (nChildren > 0){
                    // do we need the max deviation in x or y direction? (e.g. arms need scale in x direction, hips need scale in y direction)
                    bool xScale = Part2XScale[bodyPart];
                    scale = (xScale) ? diff.x : diff.y;
                    scale *= 2f;
                }
                else{
                    // scale for head, hands and feet
                    scale = DefaultScale[bodyPart];
                }
                // TODO: work with relative values in case of end joints (e.g. a hand might be 50% as long as the forearm)
                scales.Add(bodyPart, scale);
            }

            // Destroy hierarchy of segmentation boxes in order to transform boxes individually without affecting other boxes (important for usability)
            foreach(KeyValuePair<string, Transform> bpT in bodies){
                string bodyPart = bpT.Key;
                Transform T = bpT.Value;
                T.parent = null;
            }

            // finally we position, scale and rotate each box
            foreach(KeyValuePair<string, Vector3> mp in meanPositions){
                string bodyPart = mp.Key;
                Vector3 pos = mp.Value;
                // extract the transform of the box
                Transform T = bodies[bodyPart];

                // extract the scales
                float scale = scales[bodyPart];
                Vector3 s = T.localScale;

                // apply the scales
                bool xScale = Part2XScale[bodyPart];
                float defaultScale = DefaultScale[bodyPart];
                T.localScale = (xScale) ? new Vector3(scale, defaultScale, 0.35f) : new Vector3(defaultScale, scale, 0.35f);
                
                // apply the positions
                // TODO should be also adapted for head and feet?
                if (bodyPart == "HAND_L"){
                    // heuristic positioning as hands have no child transforms
                    T.position = new Vector3(pos.x + scale/2f, pos.y, pos.z);
                }
                else if(bodyPart == "HAND_R"){
                    T.position = new Vector3(pos.x - scale/2f, pos.y, pos.z);
                }
                else {
                    T.position = pos;
                }

                // apply rotation
                Quaternion rot = rotations[bodyPart];
                T.rotation = rot;
            }
        }
    }
}