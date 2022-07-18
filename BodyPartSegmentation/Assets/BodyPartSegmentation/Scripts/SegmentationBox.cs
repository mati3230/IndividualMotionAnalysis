using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BodyPartSegmentation
{
    public class SegmentationBox
    {
        public static int MaxSegmentationPlanes = 6;
        // Corner points in order to calculate the segmentation planes -- every plane will point to the center of the box
        private List<Vector3[]> planes = new List<Vector3[]>()
        {
            new Vector3[]{ // upper
                new Vector3(-0.5f,  0.5f,  0.5f),
                new Vector3(-0.5f,  0.5f, -0.5f),
                new Vector3( 0.5f,  0.5f,  0.5f)
            },
            new Vector3[]{ // lower
                new Vector3(-0.5f, -0.5f,  0.5f),
                new Vector3( 0.5f, -0.5f,  0.5f),
                new Vector3(-0.5f, -0.5f, -0.5f)
            },
            new Vector3[]{ // right
                new Vector3(-0.5f,  0.5f,  0.5f),
                new Vector3(-0.5f, -0.5f,  0.5f),
                new Vector3(-0.5f,  0.5f, -0.5f) 
            },
            new Vector3[]{ // left
                new Vector3( 0.5f,  0.5f,  0.5f),
                new Vector3( 0.5f,  0.5f, -0.5f), 
                new Vector3( 0.5f, -0.5f,  0.5f)
            },
            new Vector3[]{ // front
                new Vector3( 0.5f,  0.5f,  0.5f),
                new Vector3( 0.5f, -0.5f,  0.5f), 
                new Vector3(-0.5f,  0.5f,  0.5f)
            },
            new Vector3[]{ // behind
                new Vector3( 0.5f,  0.5f, -0.5f),
                new Vector3(-0.5f,  0.5f, -0.5f),
                new Vector3( 0.5f, -0.5f, -0.5f) 
            }
        };
        public List<Vector3[]> Planes{get{return planes;}}

        private List<SegmentationPlane> segPlanes = new List<SegmentationPlane>();
        public List<SegmentationPlane> SegPlanes {get{return segPlanes;}}

        private string name;
        public string Name{get{return name;}}

        public SegmentationBox(Transform boxT){
            this.name = boxT.gameObject.name;
            InitPlanes(boxT);
        }

        private void InitPlanes(Transform boxT){
            // Apply the transformation of the box in the editor to the segmentation box
            Vector3 pos = boxT.position;
            Vector3 scale = boxT.localScale;
            Quaternion rotation = boxT.rotation;

            // transform every corner point of each plane (see definition of 'planes' above) according to the transformation of a box in the editor
            for(int i=0; i<this.planes.Count; i++){
                Vector3[] plane = this.planes[i];
                for(int j=0; j<plane.Length; j++){
                    Vector3 v = plane[j];
                    v = new Vector3(scale.x*v.x, scale.y*v.y, scale.z*v.z);
                    v = rotation * v;
                    v += pos;
                    plane[j] = v;
                }

                this.planes[i] = plane;
                // initialize the segmentation plane
                this.segPlanes.Add(new SegmentationPlane(plane[0], plane[1], plane[2]));
            }

        }

        public bool InPoint(Vector3 p){
            // Determine if a point p is in the segmentation box
            for(int i=0; i<this.segPlanes.Count; i++){
                float distance = this.segPlanes[i].SignedDistance(p);
                // point is behind a plane -> point is out of the box
                if (distance < 0){
                    return false;
                }
            }
            return true;
        }

        /*
        public Vector3 ProjectOnSegPlane(int planeIdx, Vector3 p){
            return this.segPlanes[planeIdx].Project(p);
        }

        public Vector3 CreatePoint(int planeIdx, Vector2 u, Vector3 q){
            return this.segPlanes[planeIdx].CreatePoint(u, q);
        }
        */
    }

    public class SegmentationPlane{
        private Vector3 q;
        public Vector3 Q{get{return this.q;}}
        private Vector3 a;
        public Vector3 A{get{return this.a;}}
        private Vector3 b;
        public Vector3 B{get{return this.b;}}
        private Vector3 n;
        public Vector3 N{get{return this.n;}}
        private float d;
        public float D{get{return this.d;}}
        public SegmentationPlane(Vector3 q, Vector3 a, Vector3 b){
            // a: corner point of the plane
            // b: corner point of the plane
            // q: center point of a plane
            this.q = q; // position
            this.a = a - q; // first direction vector
            this.b = b - q; // second direction vector
            this.n = Vector3.Cross(this.a, this.b).normalized; // normal
            this.a.Normalize();
            this.b.Normalize();
            this.d = -Vector3.Dot(this.n, q); // used to determine the signed distance to the plane 
            /* uncomment for debugging visualization
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.transform.position = a;
            cube.transform.localScale = new Vector3(0.2f,0.2f,0.2f);
            GameObject cubeN = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cubeN.transform.position = a + n;
            cubeN.transform.localScale = new Vector3(0.2f,0.2f,0.2f);
            */
        }
        public float SignedDistance(Vector3 p){
            // Calculates the signed distance of a point to the plane
            // if positive, then the point p is on side of the normal, otherwise p is behind the normal (and the plane) 
            float distance_ = Vector3.Dot(this.n, p) + this.d;
            //Debug.Log(distance_);
            float distance = distance_ / this.n.magnitude;
            //Debug.Log(distance);
            return distance;   
        }
        public float Distance(Vector3 p){
            // Absolute Distance of a point to the plane
            float distance = SignedDistance(p);
            distance = Mathf.Abs(distance);
            //Debug.Log(distance);
            return distance;
        }
        /*
        public Vector3 Project(Vector3 p){
            return Vector3.ProjectOnPlane(p, this.n);
        }
        public Vector3 CreatePoint(Vector2 u, Vector3 q){
            return u.x * this.a + u.y * this.b + q;
        }
        */
    }
}
