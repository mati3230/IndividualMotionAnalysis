# MMH Tools

## MMH2CSV

This tool converts the [MMH](https://github.com/Mirevi/MotionHub)-orientations into a csv-file. If you use [miniconda](https://docs.conda.io/en/latest/miniconda.html), the first step is to start the *Anaconda Prompt* and type:

* cd path/to/this/project
* conda activate kiku

Note that *kiku* is the name we specified for our [miniconda](https://docs.conda.io/en/latest/miniconda.html) environment. After that, you can execute the program by typing:

* python mmh2csv.py --mmh_file path/to/mmh_file.mmh --csv_file path/to/csv_file.csv

By default, the left toe will be the root joint. The hip joint can also be set as root:

* python mmh2csv.py --mmh_file path/to/mmh_file.mmh --csv_file path/to/csv_file.csv --hip_root True

You can also render the animation by typing:

* python mmh2csv.py --mmh_file path/to/mmh_file.mmh --csv_file path/to/csv_file.csv --render rendermode

where *rendermode* can be: 

* *pl* for the rendering of the local positions
* *pg* for the rendering of the local positions
* *ol* for the rendering of the local orientations
* *og* for the rendering of the global orientations

For instance, you can render the local positions with: 

* python mmh2csv.py --mmh_file path/to/mmh_file.mmh --csv_file path/to/csv_file.csv --render pl

If the rendering is enabled, a stickfigure will be rendered. 

## MMH2MOT

This tool converts the a motion trajectory of the [MMH](https://github.com/Mirevi/MotionHub) to a .mot file. 

* python mmh2mot.py --mmh_file path/to/mmh_file.mmh --mot_file path/to/mot_file.mot --hip_root True

By default, the left toe will be the root joint. The hip joint is set as root joint by specifying the option *--hip_root True*. Omit this option to set the left toe will be the root joint:

* python mmh2mot.py --mmh_file path/to/mmh_file.mmh --mot_file path/to/mot_file.mot

You can also convert only a part of a motion trajectory by specifying start and stop frame. For example:

* python mmh2mot.py --mmh_file path/to/mmh_file.mmh --mot_file path/to/mot_file.mot --hip_root True --start_f 10 --stop_f 100

will convert only convert the frames from 10 to 99. 

## Body

This tool measures the length of the segments between the joints, the height of a person and the median filtered joint position. A mmh-recording with a person standing in X-Pose should be used as input mmh-file for this tool. You can use the tool as follows: 

* python body.py --mmh_file --mmh_file path/to/mmh_file.mmh --csv_file path/to/csv_file.csv

The tool outputs a csv file that contains the information described above. Currently, we measure the approx. body height as difference of the median filtered the middle of the toes and the head. 

## Export Animation for the BodyPartSegmentation

In order to segment the body parts with the [Unity](https://unity.com/) segmentation tool, we export an .mmh [T-Pose](https://en.wikipedia.org/wiki/T-pose) recording as .csv with local data such as (TOE_L is the root joint):

python mmh2csv.py --mmh_file ../sample_scan_tpose.mmh --csv_file ../sample_scan_tpose.csv --quats True --global_data True