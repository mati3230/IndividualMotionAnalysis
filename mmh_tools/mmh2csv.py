import argparse

from io_utils import load_mmh, save_csv
from conversion_utils import get_local_transformations, convert_to_euler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mmh_file", default="", type=str, help="Path to mmh file.")
    parser.add_argument("--csv_file", default="", type=str, help="Path where csv file will be stored.")
    parser.add_argument("--hip_root", default=False, help="If False, then the left toe will be the root joint. If True, then the hip will be the root joint")
    parser.add_argument("--start_f", default=0, type=int, help="Start frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--stop_f", default=-1, type=int, help="Stop frame with 0 < start_f < stop_f <= #frames")
    parser.add_argument("--render_f", default=-1, type=int, help="Stop frame with 0 < start_f < stop_f <= #frames")
    args = parser.parse_args()
    
    animation, durations = load_mmh(filename=args.mmh_file)
    animation = animation[args.start_f:args.stop_f]
    durations = durations[args.start_f:args.stop_f]

    local_animation = get_local_transformations(animation=animation, use_mmh_skeleton=args.hip_root)
    euler_animation = convert_to_euler(animation=local_animation)
    save_csv(fname=args.csv_file, durations=durations, anim=euler_animation, is_euler=True)

if __name__ == "__main__":
    main()