import argparse
from io_utils import load_mmh, save_body_csv
from skeleton_utils import measure_body


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mmh_file", default="", type=str, help="Path to mmh file.")
    parser.add_argument("--csv_file", default="", type=str, help="Path where csv file will be stored.")
    args = parser.parse_args()
    animation, durations = load_mmh(filename=args.mmh_file)
    segment_lengths, avg_positions, height = measure_body(animation=animation)
    save_body_csv(fname=args.csv_file, height=height, segment_lengths=segment_lengths, avg_positions=avg_positions)


if __name__ == '__main__':
    main()