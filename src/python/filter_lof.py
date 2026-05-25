"""Local Outlier Factor filtering for reconstructed 3D point clouds.

This script reads a text file containing points in the format:

    x:<value>,y:<value>,z:<value>

It applies Local Outlier Factor (LOF) filtering to remove isolated noisy points
and writes the remaining points to a new text file using the same format.

Example
-------
python src/python/filter_lof.py data/raw/raw_points.txt data/processed/filtered_points.txt
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from sklearn.neighbors import LocalOutlierFactor


def read_point_file(input_path: str | Path) -> np.ndarray:
    """Read a point-cloud text file into an Nx3 NumPy array.

    Parameters
    ----------
    input_path:
        Path to a text file where each line has the format
        ``x:<value>,y:<value>,z:<value>``.

    Returns
    -------
    np.ndarray
        Array with shape ``(N, 3)`` containing x, y, z coordinates.
    """
    points: list[list[float]] = []
    with Path(input_path).open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                parts = line.split(",")
                x = float(parts[0].split(":")[1])
                y = float(parts[1].split(":")[1])
                z = float(parts[2].split(":")[1])
                points.append([x, y, z])
            except Exception as exc:
                print(f"Warning: could not parse line {line_number}: {line} -> {exc}")
    return np.asarray(points, dtype=float)


def write_filtered_file(output_path: str | Path, points: np.ndarray, keep_mask: np.ndarray) -> None:
    """Write only the points marked as valid by the LOF filter."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file:
        for point, keep in zip(points, keep_mask):
            if keep:
                file.write(f"x:{point[0]},y:{point[1]},z:{point[2]}\n")
    print(f"Filtered file saved: {output_path}")


def choose_number_of_neighbors(points: np.ndarray) -> int:
    """Choose a conservative LOF neighbourhood size from the point count.

    LOF needs at least ``dimension + 1`` neighbours. For larger point clouds,
    a logarithmic value is used to make the filter less sensitive to local
    sampling fluctuations.
    """
    if len(points) < 2:
        return 1
    dimensions = points.shape[1]
    k = dimensions + 1
    if len(points) > 500:
        k = max(k, int(np.log(len(points))))
    return min(k, len(points) - 1)


def filter_with_lof(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Apply Local Outlier Factor filtering.

    Returns
    -------
    keep_mask:
        Boolean array where ``True`` means the point is retained.
    lof_values:
        Positive LOF scores. Larger values indicate stronger outlier behaviour.
    """
    if len(points) == 0:
        return np.asarray([], dtype=bool), np.asarray([], dtype=float)

    number_of_neighbors = choose_number_of_neighbors(points)
    lof = LocalOutlierFactor(n_neighbors=number_of_neighbors, contamination="auto")
    labels = lof.fit_predict(points)
    keep_mask = labels == 1
    lof_values = -lof.negative_outlier_factor_

    total_points = len(points)
    retained_points = int(np.sum(keep_mask))
    removed_points = total_points - retained_points
    retained_percentage = retained_points / total_points * 100 if total_points > 0 else 0.0

    print("------------ LOF Filtering Result ------------")
    print(f"Total number of points: {total_points}")
    print(f"Removed outliers: {removed_points}")
    print(f"Remaining points: {retained_points}")
    print(f"Remaining percentage: {retained_percentage:.2f}%")
    print("----------------------------------------------")

    return keep_mask, lof_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Filter reconstructed 3D points using Local Outlier Factor.")
    parser.add_argument("input_file", help="Input point file in x:<value>,y:<value>,z:<value> format.")
    parser.add_argument("output_file", help="Output file for filtered points.")
    args = parser.parse_args()

    points = read_point_file(args.input_file)
    keep_mask, _ = filter_with_lof(points)
    write_filtered_file(args.output_file, points, keep_mask)


if __name__ == "__main__":
    main()
