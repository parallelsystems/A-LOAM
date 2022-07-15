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
