import argparse
import numpy as np

from io_utils import load_mmh, save_mot_euler
from conversion_utils import get_local_transformations, convert_to_euler

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mmh_file", default="", type=str, help="Path to mmh file.")
    parser.add_argument("--mot_file", default="", type=str, help="Path where mot file will be stored.")
    parser.add_argument("--hip_root", default=False, help="If False, then the left toe will be the root joint. If True, then the hip will be the root joint")
    parser.add_argument("--start_f", default=0, type=int, help="Start frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--stop_f", default=-1, type=int, help="Stop frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--normalized_time", default=False, help="If True, then the time will be stored in a normalized format in the range [0,1]. The time in seconds will be stored otherwise.")
    args = parser.parse_args()
    
    animation, durations = load_mmh(filename=args.mmh_file)
    animation = animation[args.start_f:args.stop_f]
    durations = durations[args.start_f:args.stop_f]
    #print(durations)
    
    n_frames = min(len(durations), len(animation))
    raw_times = np.zeros((n_frames, ), dtype=np.float32)
    for i in range(n_frames-1):
        raw_times[i+1] = raw_times[i] + durations[i]

    duration = np.sum(durations[:n_frames])
    if args.normalized_time:
        n_times = raw_times / duration
    else:
        n_times = raw_times

    root_j = "TOE_L"
    if args.hip_root:
        root_j = "HIPS"
    local_animation = get_local_transformations(animation=animation, use_mmh_skeleton=args.hip_root)
    euler_animation = convert_to_euler(animation=local_animation, mx=-1, my=1, mz=-1)
    name = args.mot_file[:-4]
    name = name.split("/")[-1]
    #print(name)
    save_mot_euler(filename=args.mot_file, animation=euler_animation, n_times=n_times, ignore=["TOE_L", "TOE_R"], root_j=root_j, name=name)


if __name__ == "__main__":
    main()