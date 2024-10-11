import h5py

# Path to your HDF5 file
file_path = '/workspace/events/events_0.h5'  # Replace with your actual file name

# Open the HDF5 file in read mode
with h5py.File(file_path, 'r') as h5_file:
    breakpoint()
    # Print the names of the groups in the file
    print("Groups in the file:")
    for group in h5_file.keys():
        print(f"  - {group}")

    # Access the CD group and print its datasets
    cd_group = h5_file['CD']
    print("\nDatasets in 'CD' group:")
    for dataset in cd_group.keys():
        print(f"  - {dataset}")

        # Read the dataset
        data = cd_group[dataset][:]
        print(f"Data in '{dataset}':\n", data)

    # If you want to access the EXT_TRIGGER group similarly:
    if 'EXT_TRIGGER' in h5_file:
        ext_trigger_group = h5_file['EXT_TRIGGER']
        print("\nDatasets in 'EXT_TRIGGER' group:")
        for dataset in ext_trigger_group.keys():
            print(f"  - {dataset}")

            # Read the dataset
            data = ext_trigger_group[dataset][:]
            print(f"Data in '{dataset}':\n", data)
