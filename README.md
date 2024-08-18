# Depth-Maps-to-Point-Clouds-using-Open3D

### Setup Env
```
conda create -n depthanything python=3.9
conda activate depthanything

pip install -r requirements.txt
pip install open3d
```
### Download Model Weight for Depth-Anything-V2

link: https://github.com/DepthAnything/Depth-Anything-V2#pre-trained-models

put them under the checkpoints directory.

### run the code 

```
python run_video_pc.py   --encoder vits   --video-path <video-path>

python run_video_pc.py   --encoder vits   --video-path video.mp4

python run_video_pc.py   --encoder vits   --video-path <video-path>  --grayscale

python run_video_pc.py   --encoder vits   --video-path video.mp4  --grayscale

```
