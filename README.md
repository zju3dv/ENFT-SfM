# ENFT-SfM

**Current version**: 1.0

This source code provides a reference implementation for ENFT-SfM.

For ENFT(**E**fficient **N**on-consecutive **F**eature **T**racking) method implementation, please go to [ENFT](https://github.com/ZJUCVG/ENFT).

For Segment-Based Bundle Adjustment implementation, please go to [SegmentBA](https://github.com/ZJUCVG/SegmentBA).

## 1. Introduction

ENFT (**E**fficient **N**on-consecutive **F**eature **T**racking) is a feature tracking method which can efficiently match feature point correspondences among one or multiple video sequences. ENFT-SfM is a complete structure-from-motion system which uses ENFT method for feature tracking and SegmentBA for bundle adjustment optimization.

## 2. Related Publications


[1] Guofeng Zhang*, Haomin Liu, Zilong Dong, Jiaya Jia, Tien-Tsin Wong, and Hujun Bao*. **Efficient Non-Consecutive Feature Tracking for Robust Structure-from-Motion**. IEEE Transactions on Image Processing, 25(12): 5957 – 5970, 2016.
[**[arXiv report](http://arxiv.org/abs/1510.08012)**] [**[video](http://www.cad.zju.edu.cn/home/gfzhang/projects/tracking/featuretracking/ENFT-video.wmv)**]

[2] Guofeng Zhang, Zilong Dong, Jiaya Jia, Tien-Tsin Wong, and Hujun Bao. **Efficient Non-Consecutive Feature Tracking for Structure-from-Motion**. *European Conference on Computer Vision (ECCV)*, 2010.

## 3. License


This software is for non-commercial use only. Any modification based on this work must be open source and prohibited for commercial use.

If you need a closed-source version of ENFT-SfM for commercial purposes, please contact [Guofeng Zhang](mailto:zhangguofeng@cad.zju.edu.cn).

If you use this source code for your academic publication, please cite our TIP paper:

	@article{
	  title={Efficient Non-Consecutive Feature Tracking for Robust Structure-from-Motion},
	  author={Guofeng Zhang, Haomin Liu, Zilong Dong, Jiaya Jia, Tien-Tsin Wong, Hujun Bao},
	  journal={IEEE Transactions on Image Processing},
	  volume = {25},
	  number = {12},
	  papges = {5957--5970},
	  doi = {10.1109/TIP.2016.2607425},
	  year={2016}
	}
​	
## 4. Dependencies

- [GLUT](https://www.opengl.org/resources/libraries/glut/)
- [GLEW](http://glew.sourceforge.net/)
- [CVD - 20121025.2](https://github.com/edrosten/libcvd/releases/tag/RELEASE_20121025.2)
- [CLAPACK - 3.2.1](http://www.netlib.org/clapack/)
- [LEVMAR - 2.6](http://www.ics.forth.gr/~lourakis/levmar/)


## 5. Usage

The project has been tested in Visual Studio 2015 and 16.04 (GCC Version > 5.4). We provide almost all the prebuilt x64 libraries in`3rdparty/`. You can run the program directly under the default project setting.

For Windows:

 `./ENFT.exe path/to/your/config.txt`

For Ubuntu 16.04:

```bash
sudo apt-get install libx11-dev libglew-dev freeglut3-dev libjpeg-dev libtiff-dev libpng-dev
cd /path/to/the/project
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./runENFT path/to/your/config.txt
```

#### Configuration File

We provide three configuration file example in `ENFT\config\`, which show how to run ENFT-SfM in single sequence mode ( support varying focal length and constant focal length datasets) and multiple sequences mode (the camera intrinsic parameters should be known). Please refer to **Dataset** part to download these corresponding datasets.

1.`videos_number` is the number of videos.

2.`window_width` and `window_height` indicate the size of window.

3.`video_i_*`give the information of the i'th video.

4.`calib_file_name` is the file name which provide the camera intrinsic parameters (fx,fy,cx,cy). If not provided, the system will assume the focal length is contant but unkown (if const_focal = 1) or varied (if const_focal = 0).

5.`const_focal` will be used when calib_file_name not given, set 1 if the focal length of the camera is constant.

6.`radio_distortion` set 1 when camera distortion not rectified.

5.`param_directory` is the param file directory and it contains the detailed params of tracking and bundle adjustment.

5.`output_directory` is the output file directory, it would be the video directory when not given or do not exist.

6.`min_frame_number` and `max_frame_number` is the param of segmentBA for spliting a video to several sequences.

8.`use_temporary_file` set 1 to save and reuse (if exist) temporary files for some steps in SfM. 

7.`view` set 1 to show the result.

#### Dataset

- Single Sequence Datasets
  - Varying Focal Length "Plant" ([Our Website](http://www.cad.zju.edu.cn/home/gfzhang/projects/tracking/featuretracking/data/plant.rar)) , use `config_plant.txt` 
  - Constant Focal Length "KITTI Seq00" ( [Google Drive](https://drive.google.com/file/d/1hvpwvf6Y9tTcIniI4fgXg_I02nrrbDbD/view?usp=sharing) , [Baidu Net Disk](https://pan.baidu.com/s/1tv1mEV9yps-LV3VqxRbRHg)), use `config_KITTI.txt`
- Multiple Sequences Dataset "Gangwan Street" ([Google Drive](https://drive.google.com/file/d/0B82Mv44r3F25ZGhtSWdNZ3FQNUE/view?usp=sharing), [Baidu Net Disk](http://pan.baidu.com/s/1kTzsTwV)), use `config_gangwan.txt`

#### Note

1. For Windows: To ensure that the program works properly, it is recommended to adjust the graphic card priority in NVidia control pannel, and use the NVidia graphics card (recommend NVidia GTX 780 or above) for ENFT executable program.
2. For Linux: Currently,  the Linux version may be not as efficient as that of the Windows version. It still has problems to run with NVidia driver. So for running the program properly, it is recommended to switch the graphic driver to nouveau. We will try to address this problem in the future.
