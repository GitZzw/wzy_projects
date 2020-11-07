import pyrealsense2 as rs
from time import time
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
cfg = pipeline.start(config)
for i in range(1,1000):
    for j in range(1,10000):
        k = i+j
profile = cfg.get_stream(rs.stream.depth)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr) 

print('*************************\n')


profile2 = cfg.get_stream(rs.stream.color)
intr2 = profile2.as_video_stream_profile().get_intrinsics()
print(intr2) 
