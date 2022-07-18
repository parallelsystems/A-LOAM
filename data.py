"""Shim for getting TXYZ files into LOAM
"""


import cv2
import json
import os
import shutil

UP_DIR = "/media/nas/UP Oxnard PTC Data/"
LIDAR_DIR = os.path.join(UP_DIR, "lidar/")
DATA_DIR = "/home/jack/catkin_ws/src/A-LOAM/luminar/"
MAX_TIME_S = 60

with open(os.path.join(UP_DIR, "meta_lidar/valid_meta.json"), "r") as f:
    meta = json.load(f)

def idxs_valid(idxs: list):
    """Assert idxs are ordered properly and close together in time
    """
    meta_subset = [meta[i] for i in idxs]
    for i in range(len(meta_subset)-1):
        if meta_subset[i+1][0] <= meta_subset[i][0]:
            return False
        elif meta_subset[i+1][0] - meta_subset[i][0] > MAX_TIME_S:
            return False
    return True
    
def get_meta_subset(idxs: list):
    """Generates `files.txt`, `times.txt` for `idxs`
    """
    meta_subset = [meta[i] for i in idxs]
    times, names = [m[0] for m in meta_subset], [m[1] for m in meta_subset]
    with open(f"{DATA_DIR}times.txt", "w") as f:
        f.writelines([f'{t}\n' for t in times])

    with open(f"{DATA_DIR}files.txt", "w") as f:
        f.writelines([f'{n}\n' for n in names])
    
def pull_files(idxs: list):
    """Pulls luminar xyz files specified by `files` into local storage
    so A-LOAM can see
    """
    assert idxs_valid(idxs)
    
    # generate times.txt, files.txt
    get_meta_subset(idxs)
    
    # copy files over
    for i in idxs:
        shutil.copy(
            LIDAR_DIR + "xyz_new/" + meta[i][1],
            DATA_DIR
        )

def timestamp_from_txyz(file_name):
    with open(file_name, "r") as f:
        lines = f.readlines()
    time = lines[0].split(",")[0]
    return float(time)

def pull_video(meta):
    file_name, num_frames, mapping = meta["file_name"], meta["num_frames"], meta["map"]

    cap = cv2.VideoCapture(file_name)
    c = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imwrite(f"{DATA_DIR}{c}.png", frame)
        c += 1

    if c != num_frames:
        raise ValueError(f"meta says num_frames = {num_frames}, only {c+1} found")
    
    names = [m[1] for m in mapping]
    times = [timestamp_from_txyz(LIDAR_DIR + "xyz_new/" + n) for n in names]

    with open(f"{DATA_DIR}times.txt", "w") as f:
        f.writelines([f'{t}\n' for t in times])

    with open(f"{DATA_DIR}files.txt", "w") as f:
        f.writelines([f'{n}\n' for n in names])

    for n in names:
        shutil.copy(
            LIDAR_DIR + "xyz_new/" + n,
            DATA_DIR
        )


if __name__ == "__main__":
    import random
    N_SCENES = 10
    SCENE_LENGTH = 500

    selected = []
    valid_scenes = range(len(meta) // SCENE_LENGTH)
    while len(selected) < N_SCENES:
        scene = random.choice(valid_scenes)
        scene = range(scene)
        try:
            pull_files()

