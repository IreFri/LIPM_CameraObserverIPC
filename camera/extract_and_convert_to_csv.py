#!/usr/bin/env python3

import sys
import os

keys_to_extract = "Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_x Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_y Observers_LIPMWalkingObserverPipeline_CameraLeft_ground_points_z Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_x Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_y Observers_LIPMWalkingObserverPipeline_CameraRight_ground_points_z"

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print("Please give the path of the directory with the .bin")
    sys.exit()

  path = sys.argv[1]
  print("The chosen path is '{}'".format(path))

  if not os.path.isdir(path):
    print("The path is not a directory")
    sys.exit()


  path_csv = path[:-1] + "_csv/"
  if not os.path.exists(path_csv):
    os.mkdir(path_csv)

  for filename in os.listdir(path):
    file_path = os.path.join(path, filename)
    # checking if it is a file
    if os.path.isfile(file_path):
      if not file_path.endswith('.bin'):
        print("File {} is not a '.bin'".format(file_path))
        continue
      # Extract with keys
      os.system("mc_bin_utils extract --in {} --out /tmp/export.bin --keys {}".format(file_path, keys_to_extract))
      # Name of the .csv
      name_csv = filename[:-3]+"csv"
      print("Export '.csv' to {}{}".format(path_csv, name_csv))
      if not os.path.exists(path_csv):
        os.mkdir(path_csv)
      # Convert to .csv
      # os.system("mc_bin_to_log /tmp/export.bin {}{}".format(path_csv, name_csv))
      # Or if was created with re-process_bin
      os.system("mc_bin_to_log {} {}{}".format(file_path, path_csv, name_csv))