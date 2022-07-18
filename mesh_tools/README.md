# Remove Clutter from a Body Scan

In order to remove clutter from the scan, one can use the [preprocess](./preprocess.py) tool. Start the tool with: 

python preprocess --file ./path/to/body/scan.glb

After a preprocessing (filter double points, alignment, ...) of the mesh, the mesh can be cleaned. Concretely, an [open3d](http://www.open3d.org/docs/release/getting_started.html) window opens where you can select points with 'Shift + drag rectangle with LMB' that should be deleted. After that, press 'q' in the window and the selected points as well as the corresponding triangles will be deleted. Press 'e' to exit the application. The processed meshes will be enumerated and stored in the same directory as the [.glb](https://en.wikipedia.org/wiki/GlTF) file (i.e. './path/to/body') as [.ply](https://de.wikipedia.org/wiki/Polygon_File_Format) files. You can use one of the generated [.ply](https://de.wikipedia.org/wiki/Polygon_File_Format) files.

If you want to continue the cleaning of a certain mesh type:

python preprocess --file ./path/to/body/scan.glb --nr x

where x is the file number that should be loaded. For example enter 

python preprocess.py --file ../sample_scan.glb --nr 3 

in order to continue the cleaning of the file '../sample_filtered_003.ply'. 

# Estimation of the Moment of Inertia

The [moment of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia) is estimated for each segmented body part with the [props](./props.py) tool. Start the tool with:

python props.py --b_dir ./path/to/segmented/body/parts/folder

For instance: 

python props.py --b_dir ../BodyPartSegmentation/BodyParts

The resulting intertia tensors will be stored in a [.h5](https://docs.h5py.org/en/stable/quick.html) file at './path/to/segmented/body/parts/folder/intertia.h5'. 