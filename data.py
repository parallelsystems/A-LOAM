"""Shim for getting TXYZ files into LOAM
"""


import cv2
import json
import os
import shutil
from tqdm import tqdm

UP_DIR = "/media/nas/scratch/jack/data/datasets/union_pacific/"
LIDAR_DIR = os.path.join(UP_DIR, "lidar/")
DATA_DIR = "/home/jack/catkin_ws/src/A-LOAM/luminar/"
MAX_TIME_S = 60

with open(os.path.join(UP_DIR, "meta_lidar/sorted_timestamps.json"), "r") as f:
    meta = json.load(f)

def idxs_valid(idxs: list):
    """Assert idxs are ordered properly and close together in time
    """
    meta_subset = [meta[i] for i in idxs]
    for i in range(len(meta_subset)-1):
        # assert ordering
        if meta_subset[i+1][0] <= meta_subset[i][0]:
            return False
        # assert these files are sufficiently close together
        # this is simple check that all files in `idxs` were
        # collected as part of the same run
        elif meta_subset[i+1][0] - meta_subset[i][0] > MAX_TIME_S:
            return False
    return True
    
def make_metafiles(idxs: list, scene_name):
    """Generates `files.txt`, `times.txt` for `idxs`
    """
    meta_subset = [meta[i] for i in idxs]
    times, names = [m[0] for m in meta_subset], [m[1] for m in meta_subset]

    dest_dir = os.path.join(DATA_DIR, scene_name)
    if not os.path.isdir(dest_dir):
        os.mkdir(dest_dir)

    with open(os.path.join(dest_dir, "times.txt"), "w") as f:
        f.writelines([f'{t}\n' for t in times])

    with open(os.path.join(dest_dir, "files.txt"), "w") as f:
        f.writelines([f'{n}\n' for n in names])
    
def pull_txyzs(idxs: list, scene_name: str):
    """Pulls luminar txyz files specified by `files` into local storage
    so A-LOAM can see
    """
    assert idxs_valid(idxs)
    
    # generate times.txt, files.txt
    make_metafiles(idxs, scene_name)

    dest_dir = os.path.join(DATA_DIR, scene_name + "/")
    if not os.path.isdir(dest_dir):
        os.mkdir(dest_dir)

    # copy files over
    print("Copying...")
    for i in tqdm(idxs):
        shutil.copy(
            os.path.join(LIDAR_DIR, "xyz/", meta[i][1]),
            dest_dir
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
    times = [timestamp_from_txyz(LIDAR_DIR + "xyz/" + n) for n in names]

    with open(f"{DATA_DIR}times.txt", "w") as f:
        f.writelines([f'{t}\n' for t in times])

    with open(f"{DATA_DIR}files.txt", "w") as f:
        f.writelines([f'{n}\n' for n in names])

    for n in names:
        shutil.copy(
            LIDAR_DIR + "xyz/" + n,
            DATA_DIR
        )


if __name__ == "__main__":
    """randomly selects N_SCENES scenes to copy over, does some weak checking to confirm
    selected xyz files represent a single scene, not multiple
    """
    import random
    random.seed(42)
    N_SCENES = 5
    SCENE_LENGTH = 500

    selected = []
    valid_scenes = range(len(meta) // SCENE_LENGTH)
    while len(selected) < N_SCENES:
        scene_start = random.choice(valid_scenes) * SCENE_LENGTH
        scene_idxs = range(scene_start, scene_start + SCENE_LENGTH)

        # don't overwrite existing scenes
        scene_name = str(len(os.listdir(DATA_DIR)))
        try:
            pull_txyzs(scene_idxs, scene_name)
        except Exception as e:
            print(f"Encountered {e} at scene with start index {scene_start}")
            import code
            code.interact(local=locals())

