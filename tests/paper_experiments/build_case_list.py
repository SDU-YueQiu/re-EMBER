#!/usr/bin/env python3
"""Convert thesis ember_paper_cases/ CSVs to benchmark-cases.json.

Reads pairs.csv and transforms.csv from the thesis experiment directory
and writes a minimal benchmark-cases.json containing only the 100 pairs
used in our paper, with their transform matrices.

The transform matrices originate from the EMBER paper supplemental material
(Trettner et al. 2022). Only the selection and stratification are ours.
"""
import csv
import json
import os
import sys

def parse_transform_row(row):
    """Parse a row-major flat 4x4 matrix from CSV into column-major nested dict."""
    cols = []
    for c in range(4):
        col = {}
        for r in range(3):
            key = f"m{r}{c}"
            col[chr(ord('x') + r)] = float(row[key])
        col["w"] = float(row[f"m3{c}"]) if c < 3 else 1.0
        cols.append(col)
    return cols  # [col0, col1, col2, col3]


def main():
    thesis_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "Shandong-University-Undergraduate-Thesis-Design-Template-main",
        "experiments", "ember_paper_cases"
    )

    pairs_path = os.path.join(thesis_dir, "pairs.csv")
    transforms_path = os.path.join(thesis_dir, "transforms.csv")

    if not os.path.exists(pairs_path):
        print(f"Error: {pairs_path} not found. Pass thesis experiments dir as argument.")
        sys.exit(1)

    # Read pairs
    pairs = {}
    with open(pairs_path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            pairs[row["pair_id"]] = {
                "id_a": int(row["lhs_id"]),
                "id_b": int(row["rhs_id"]),
            }

    # Read transforms
    transforms = {}  # (pair_id, side) -> [col0, col1, col2, col3]
    with open(transforms_path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            key = (row["pair_id"], row["side"])
            transforms[key] = parse_transform_row(row)

    # Build output
    cases = []
    for pair_id in sorted(pairs.keys()):
        entry = pairs[pair_id]
        lhs = transforms.get((pair_id, "lhs"))
        rhs = transforms.get((pair_id, "rhs"))
        if not lhs or not rhs:
            print(f"Warning: missing transform for {pair_id}")
            continue

        col_keys = ["col0", "col1", "col2", "col3"]
        cases.append({
            "id_a": entry["id_a"],
            "id_b": entry["id_b"],
            "transform_a": {k: v for k, v in zip(col_keys, lhs)},
            "transform_b": {k: v for k, v in zip(col_keys, rhs)},
        })

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "benchmark-cases.json")
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(cases, f, indent=2)

    print(f"Wrote {len(cases)} cases to {out_path}")


if __name__ == "__main__":
    main()
