# Requirements

## Mirevi Motion Hub (MMH) (optional)

Optionally, you can install a release of the [Mirevi Motion Hub (MMH)](https://github.com/Mirevi/MotionHub) in order to record animations and a [T-Pose](https://en.wikipedia.org/wiki/T-pose). 

## Unity

Install [Unity](https://unity.com/) 2019.4.25f1 in order to execute the BodyPartSegmentation. This tool is realized with the [TriLib](https://assetstore.unity.com/packages/tools/modeling/trilib-model-loader-package-91777) package which is necessary to run the body part segmentation in order to load the ply body scans. 

## Python

Install a python 3 interpreter with the [packages](./requirements.txt) that can be found in the [requirements.txt](./requirements.txt) file. We recommend to install [miniconda](https://docs.conda.io/en/latest/miniconda.html). [miniconda](https://docs.conda.io/en/latest/miniconda.html) manages python environments. For instance, you can create a python 3.7 and another python 3.8 environment to meet the requirements of two different projects. 

### Miniconda Environment Creation with Windows

When the [miniconda installation](https://docs.conda.io/en/latest/miniconda.html) is finished, open the Start menu and type *Anaconda Prompt*. Start the terminal *Anaconda Prompt (miniconda3)*. After that, we create an environment, called *kiku*, and install the required packages: 

* cd path/to/this/project
* conda create -n kiku python=3.6.8 -y
* conda activate kiku
* pip install -r requirements.txt

A complete list of [miniconda commands](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) can be found [here](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

## [OpenSim](https://simtk.org/projects/opensim)

In order to download [OpenSim](https://simtk.org/projects/opensim), you have to sign up. There are binary files for [Windows](https://simtk.org/frs/download_confirm.php/file/6589/OpenSim-4.4-2022-06-11-win64.exe?group_id=91) and [Mac OSX](https://simtk.org/frs/download_confirm.php/file/6588/OpenSim-4.4-2022-06-11-mac.pkg?group_id=91). 

### Use [OpenSim](https://simtk.org/projects/opensim) in [Matlab](https://www.mathworks.com/products/matlab.html)

Download and install [Matlab](https://www.mathworks.com/products/matlab.html) and follow these [instructions](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+with+Matlab).

# Quickstart: Open Scanned Model in [OpenSim](https://simtk.org/projects/opensim) and Load a Custom Motion

Open the [OpenSim](https://simtk.org/projects/opensim) GUI, click *File->Open Model...* and choose the *kiku_model.osim* file that we have already created. Then, click on *File->Load Motion...* and select the [sample_movement.mot](./sample_movement.mot) file. In the remaining sections, we provide instructions to integrate custom body scans and motion trajectories in [OpenSim](https://simtk.org/projects/opensim).

# Workflow Body Scan with iPad and [PolyCam](https://apps.apple.com/de/app/polycam-lidar-3d-scanner/id1532482376)

1. Body Scan with the iPad. The user should be in a T-Pose. Subsequently to the scan with the iPad, record the user in the [T-Pose](https://en.wikipedia.org/wiki/T-pose) with the [Azure Kinect](https://azure.microsoft.com/de-de/services/kinect-dk/) with the [Mirevi Motion Hub (MMH)](https://github.com/Mirevi/MotionHub) for a few seconds. 
2. Remove clutter from the scan (export as [.ply](https://de.wikipedia.org/wiki/Polygon_File_Format) file) and convert the [T-Pose](https://en.wikipedia.org/wiki/T-pose) [MMH](https://github.com/Mirevi/MotionHub) recording (.mmh file) as .csv file
3. Apply the body part segmentation

We also provide already preprocessed files:

* sample_scan.ply: Scan from the [PolyCam](https://apps.apple.com/de/app/polycam-lidar-3d-scanner/id1532482376) app
* sample_scan_filtered.ply: Removed clutter from the sample_scan.ply file
* sample_scan_tpose.mmh: .mmh file of a T-Pose which was done subsequently after the scan which resulted in sample_scan.ply
* sample_scan_tpose.csv: Conversion of the sample_scan_tpose.mmh file as .csv file with the [mmh2csv](./mmh_tools/mmh2csv.py) tool

## Body Scan

Currently, we use the application [PolyCam](https://apps.apple.com/de/app/polycam-lidar-3d-scanner/id1532482376) with an [iPad Pro](https://www.apple.com/de/shop/buy-ipad/ipad-pro). We export the mesh as [.glb](https://en.wikipedia.org/wiki/GlTF) file. 

### Remove Clutter

In order to remove clutter from the scan, one can use the [preprocess](./mesh_tools/preprocess.py) tool. Have a look at the [README](./mesh_tools/README.md) for more information. 

## T-Pose to CSV

In order to automatically position the segmentation boxes with the BodyPartSegmentation-Tool ([Unity](https://unity.com/) Project), you need a .csv file with Tracking-Data of a [T-Pose](https://en.wikipedia.org/wiki/T-pose). You can record the [T-Pose](https://en.wikipedia.org/wiki/T-pose) with the [Azure Kinect](https://azure.microsoft.com/de-de/services/kinect-dk/) with the [Mirevi Motion Hub (MMH)](https://github.com/Mirevi/MotionHub). The recording is stored in the 'path/to/MMH/data' folder as .mmh file. This file can be converted with the [mmh2csv](./mmh_tools/mmh2csv.py) tool. See the [README](./mmh_tools/README.md) for more information the conversion. 

## Body Part Segmentation

The scanned body can be segmented into parts BodyPartSegmentation-Tool ([Unity](https://unity.com/) Project). See the [README](./BodyPartSegmentation/README.md) for more information. 

## Estimation of the Moment of Inertia

The [moment of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia) is estimated for each segmented body part with the [props](./mesh_tools/props.py) tool. It stores a file that contains the inertia tensor for each body part. See the [README](./mesh_tools/README.py) for more information.

# Create the [OpenSim](https://simtk.org/projects/opensim) Model in [Matlab](https://www.mathworks.com/products/matlab.html)

Open the file [osim_model.m](./osim_model.m) with [Matlab](https://www.mathworks.com/products/matlab.html)
Check if the variable *path* points to the folder which contains the body parts which where segmented with the Unity tool. Now, execute the script in [Matlab](https://www.mathworks.com/products/matlab.html) and a file called *kiku_model.osim* will be stored. After that, you can start the [OpenSim](https://simtk.org/projects/opensim) GUI. Then, click *File->Open Model...* and choose the *kiku_model.osim* file that we have created. 

# Apply Custom Trajectories in OpenSim

Record a motion trajectory with the [MMH](https://github.com/Mirevi/MotionHub) and export it as .mmh file. After that, convert the .mmh file to a .mot file with the [mmh2mot](./mmh_tools/mmh2mot.py) tool (its usage is described in the [README](./mmh_tools/mmh2mot.py)). Now, open the [OpenSim](https://simtk.org/projects/opensim) GUI and open a model (e.g. the custom model from the previous section). Click on *File->Load Motion...* and select the converted .mot file. Alternatively, you can open an already converted file called [sample_movement.mot](./sample_movement.mot) which was recorded with the [Microsoft Azure Kinect](https://azure.microsoft.com/de-de/services/kinect-dk/).  