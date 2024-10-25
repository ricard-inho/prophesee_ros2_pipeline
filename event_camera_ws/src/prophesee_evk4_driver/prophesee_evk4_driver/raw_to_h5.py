import os

import numpy as np
import h5py
from matplotlib import pyplot as plt

from metavision_ml.preprocessing.viz import filter_outliers
from metavision_ml.preprocessing.hdf5 import generate_hdf5

input_path = "/workspace/event_camera_ws/recording_241025_163353.raw"

output_folder = "/workspace/events/"
output_path = output_folder + os.sep + os.path.basename(input_path).replace('.raw', '.h5')
if not os.path.exists(output_path):
    generate_hdf5(paths=input_path, output_folder=output_folder, preprocess="timesurface", delta_t=250000, height=None, width=None,
              start_ts=0, max_duration=None)

print('\nOriginal file \"{}" is of size: {:.3f}MB'.format(input_path, os.path.getsize(input_path)/1e6))
print('\nResult file \"{}" is of size: {:.3f}MB'.format(output_path, os.path.getsize(output_path)/1e6))

f  = h5py.File(output_path, 'r')  # open the HDF5 tensor file in read mode
print(f['data'])  # show the 'data' dataset

hdf5_shape = f['data'].shape
print(hdf5_shape)
print(f['data'].dtype)

print("Attributes :\n")
for key in f['data'].attrs:
    print('\t', key, ' : ', f['data'].attrs[key])

for i, timesurface in enumerate(f['data'][:10]):

    plt.imshow(filter_outliers(timesurface[0], 7)) #filter out some noise
    plt.title("{:s} feature computed at time {:d} μs".format(f['data'].attrs['events_to_tensor'],
                                                          f['data'].attrs["delta_t"] * i))
    plt.pause(0.01)