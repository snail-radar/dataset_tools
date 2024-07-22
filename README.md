# dataset_tools
converters for the generic 4D radar dataset which is [here](https://snail-radar.github.io).

# Converters

* convert the rosbag to a generic folder of a 4D radar dataset sequence

```
python3 folder_to_bag.py

```

* convert the content of a generic folder of a 4D radar dataset sequence to a rosbag

```
python3 bag_to_folder.py
```

# Calibration

The frame definitions and extrinsic calibrations are provided in the [matlab](./matlab) folder.

The intrinsic parameters of the zed2i stereo camera are in the topics of each bag,
/zed2i/zed_node/left_raw/camera_info and /zed2i/zed_node/right_raw/camera_info.


# Dependencies

## Python scripts
The python scripts require a ros1 distribution to run, e.g., ros1 noetic which can be installed for ubuntu <=20.04 following [instructions](https://wiki.ros.org/noetic/Installation).

For higher ubuntu versions say 22.04, a [docker container](https://github.com/JzHuai0108/kalibr?tab=readme-ov-file#docker) or the [robostack](https://robostack.github.io/index.html) can be used to run this program.
Both are pretty easy to install and use.

## Matlab scripts
The matlab scripts require a MATLAB >=2007a installation.


<!--- # Work with the raw data
For those interested in how to process the raw data, the source code is available [here](https://bitbucket.org/BinliangWang/radar_data_preprocess/). -->
