using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Threading;
using BodyPartSegmentation.UI;
using UnityEngine.UI;

namespace BodyPartSegmentation{
    public class BodyPartSegmentation : MonoBehaviour
    {
        // The parent of the segmentation boxes
        private Transform body;
        [SerializeField, Tooltip("The body scan that should be segmented")]
        private MeshFilter bodyScan;
        [SerializeField, Tooltip("Store the segmented body parts in the origin")]
        private bool centered = true;
        [SerializeField, Tooltip("Name of the csv of the T-Pose")]
        private string csvName;
        [SerializeField, Range(0, 1), Tooltip("Relative frame of the T-Pose that should be used for automatic positioning of the boxes")]
        private float framePos = 0.5f;
        // List of segmentation boxes
        List<SegmentationBox> segBoxes;

        // UI
        private StatusImage statusImage;
        private Button btnSeg;

        // The segmentation is executed in a thread so that Unity is not blocked
        private Thread thread;
        // Assignment of body part names (see MMH documentation) to the segmentation boxes
        Dictionary<string, Transform> bodies;

        void Start()
        {
            // find the necessary components in the scene
            body = GameObject.Find("Body_T").transform;
            if(body == null){
                throw new Exception("Miss Transform 'Body_T'");
            }
            btnSeg = GameObject.Find("BSegmentation").GetComponent<Button>();
            if(btnSeg == null){
                throw new Exception("Miss Button 'BSegmentation'");
            }
            btnSeg.onClick.AddListener(Segmentation);
            statusImage = GameObject.Find("ImgSegmentation").GetComponent<StatusImage>();
            if(statusImage == null){
                throw new Exception("Miss StatusImage 'ImgSegmentation'");   
            }

            // Read the T-Pose that is stored in a csv file 
            string csvPath = Application.streamingAssetsPath + "/" + csvName + ".csv";
            Dictionary<string, Frame[]> animation = null;
            int nFrames = 0;
            if(!Utils.TryReadCSV(csvPath, out animation, out nFrames))
                throw new Exception("Cannot read csv file");

            // Search the segmentation boxes in the scene
            bodies = null;
            if(!Utils.AssignBodyParts(body, out bodies))
                throw new Exception("Unable to assign body parts");
            
            // Automatic positioning of the boxes
            Utils.ApplyFrame(bodies, animation, framePos, nFrames);

            EnableSegmentationUI();
        }


        void EnableSegmentationUI(){
            // Enable the segmentation UI
            statusImage.ToggleImage(true);
            btnSeg.interactable = true;
        }


        void DisableSegmentationUI(){
            // Disable the segmentation UI
            statusImage.ToggleImage(false);
            btnSeg.interactable = false;
        }


        void InitSegBoxes(Dictionary<string, Transform> bodies){
            // Initialize the segmentation boxes with the current position ofthe boxes in scene
            segBoxes = new List<SegmentationBox>();
            foreach(KeyValuePair<string, Transform> pair in bodies){
                Transform T = pair.Value;
                segBoxes.Add(new SegmentationBox(T));
            }
        }


        void Seg(string dataPath, Vector3[] verticesA, int[] trisA, List<SegmentationBox> segBoxes){
            // Execution of the body part segmentation
            Debug.Log("Start Segmentation");
            Directory.CreateDirectory(dataPath);
            // Save the segmentation of each box
            for(int i=0; i<segBoxes.Count; i++){
                SegmentationBox segBox = segBoxes[i];
                string filename = dataPath + "/" + segBox.Name;
                // Execution of the segmentation
                if(!Utils.TrySaveIntersection(filename, verticesA, trisA, segBox, centered)){
                    Debug.LogFormat("Cannot save segment for '{0}'", segBox.Name);
                }
            }
            Debug.Log("Done");
        }


        public void Segmentation(){
            // Starts the segmentation 
            InitSegBoxes(bodies);
            DisableSegmentationUI();
            // Determine folder to save the body parts
            string dataPath = Application.dataPath + "/BodyParts";
            if(Application.isEditor)
                dataPath = Application.dataPath + "/../BodyParts";
            
            // Apply transformation of editor to mesh
            Mesh mA = bodyScan.mesh;
            Vector3[] verticesA = mA.vertices;
            for (int i=0; i<verticesA.Length; i++){
                verticesA[i] = bodyScan.transform.rotation * verticesA[i];
                verticesA[i] += bodyScan.transform.position;
            }
            int[] trisA = mA.triangles;

            // Start the segmentation thread
            thread = new Thread( () => {
                    Seg(dataPath, verticesA, trisA, this.segBoxes);
            } );
            //Debug.Log(thread.ThreadState); // Unstarted
            thread.Start();
            //Debug.Log(thread.ThreadState); // Running
            
            // Wait till the segmentation is finished (non-blocking)
            StartCoroutine(WaitThread());
        }


        IEnumerator WaitThread()
        {
            // Waits till the segmentation is finished (non-blocking)
            //Debug.Log(thread.ThreadState); // Running
            while(thread.ThreadState == ThreadState.Running || thread.ThreadState == ThreadState.WaitSleepJoin){
                //Debug.Log("Wait");
                yield return new WaitForSeconds(3);
            }
            //Debug.Log(thread.ThreadState); // Stopped
            // Segmentation is finished
            EnableSegmentationUI();
        }
    }
}
