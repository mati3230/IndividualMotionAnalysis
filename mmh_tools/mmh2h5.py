import argparse
import numpy as np
from io_utils import load_mmh, save_h5
from conversion_utils import get_local_transformations, convert_to_euler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mmh_file", default="", type=str, help="Path to mmh file.")
    parser.add_argument("--h5_file", default="", type=str, help="Path where h5 file will be stored.")
    parser.add_argument("--hip_root", default=False, help="If False, then the left toe will be the root joint. If True, then the hip will be the root joint")
    parser.add_argument("--start_f", default=0, type=int, help="Start frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--stop_f", default=-1, type=int, help="Stop frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--render_f", default=-1, type=int, help="Stop frame with 0 < start_f < stop_f <= #frames")
    args = parser.parse_args()
    
    animation, durations = load_mmh(filename=args.mmh_file)
    animation = animation[args.start_f:args.stop_f]
    durations = durations[args.start_f:args.stop_f]

    n_frames = min(len(durations), len(animation))
    raw_times = np.zeros((n_frames, ), dtype=np.float32)
    for i in range(n_frames-1):
        raw_times[i+1] = raw_times[i] + durations[i]

    duration = np.sum(durations[:n_frames])
    
    n_times = raw_times / duration

    local_animation = get_local_transformations(animation=animation, use_mmh_skeleton=args.hip_root)
    euler_animation = convert_to_euler(animation=local_animation, mx=-1, my=1, mz=-1)

    save_h5(fname=args.h5_file, durations=durations, anim=euler_animation, n_times=n_times)

if __name__ == "__main__":
    main()