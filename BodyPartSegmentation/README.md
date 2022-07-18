# Preperation

1. Start this project with [Unity3D](https://unity.com/).
2. Drag the .ply file of the body scan from the explorer into the folder './Assets/BodyPartSegmentation/Meshes'. 
3. Open the scene './Assets/Scenes/MatchMMH.unity'.
4. Delete the sample scan 'sample_scan_filtered_003' in the Hierarchy. 
5. Drag the imported mesh of your body scan from step 1. from the unity explorer into the scene.
6. Click on the folder './Assets/BodyPartSegmentation/Materials' in the Project tab of the unity editor.
7. Drag the material 'BodyScan' (white) onto the body scan in the Hierarchy so that backfaces of the triangles are rendered.
8. Click on the body scan in the Hierarchy view and reset the transform in the Inspector.
9. Transform the body scan until the mesh stays upright (this step can be skipped if the scan stays already upright).
10. Click on the object 'BodyPartSegmentation' in the Hierarchy.
11. Drag the scan from the Hierarchy into the field 'Body Scan' of the component 'BodyPartSegmentation' in the Inspector (hover with the mouse over the other options of the component 'BodyPartSegmentation' (such as 'Centered') to get more information about these options).
12. Drag the .csv of the body scan from the explorer into the folder './Assets/StreamingAssets'.
13. Click on the object 'BodyPartSegmentation' in the Hierarchy.
14. Type the name of the .csv of the body scan into the field 'Csv Name' of the component 'BodyPartSegmentation' in the inspector.

# Body Part Segmentation

1. Subsequently to the preparation steps, click on the play button in the Unity Editor.
2. Transform the body scan and the segmentation boxes in the Scene View.
3. Click on the 'Segmentation' button in the Game View. The body part segments will be stored in the folder './BodyParts' as .ply files. See the messages in the Console of the unity editor for any body parts that could not be segmented, e.g. if a box does not encapsulate any triangle of the body scan.  