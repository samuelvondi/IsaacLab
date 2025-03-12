import h5py

def print_hdf5_structure(file_path):
    """Prints the hierarchy of an HDF5 file."""
    def visit_fn(name, obj):
        obj_type = "Group" if isinstance(obj, h5py.Group) else "Dataset"
        print(f"{obj_type}: {name}")
    
    with h5py.File(file_path, "r") as f:
        print(f"Structure of {file_path}:")
        f.visititems(visit_fn)

# Paths to your two HDF5 files
file1 = "/home/matthias/IsaacLab/datasets/dataset.hdf5"
file2 = "/home/matthias/IsaacLab/datasets/dataset_lift.hdf5"

# print_hdf5_structure(file1)
# print("\n" + "="*50 + "\n")
# print_hdf5_structure(file2)

file1 = "/home/matthias/IsaacLab/datasets/dataset.hdf5"
file2 = "/home/matthias/IsaacLab/datasets/dataset_lift.hdf5"

# def check_mask(file_path):
#     with h5py.File(file_path, "r") as f:
#         if "mask" in f:
#             print(f"Mask found in {file_path}")
#             print("Keys in mask:", list(f["mask"].keys()))
#         else:
#             print(f"No mask found in {file_path}")

# check_mask(file1)
# check_mask(file2)

# import h5py

# file_path = "/home/matthias/IsaacLab/datasets/dataset.hdf5"

# with h5py.File(file_path, "r") as f:
#     if "mask" in f:
#         print("Mask group exists. Available keys:", list(f["mask"].keys()))
#     else:
#         print("No mask group found.")
