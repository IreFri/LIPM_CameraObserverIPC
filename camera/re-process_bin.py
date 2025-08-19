#!/usr/bin/env python3

import sys
import os

exec_path = "/path/to/build"

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print("Please give the path of the directory with the .bin")
    sys.exit()

  path = sys.argv[1]
  print("The chosen path is '{}'".format(path))

  if not os.path.isdir(path):
    print("The path is not a directory")
    sys.exit()

  path_new_bin = path + "_new_bin/"
  if not os.path.exists(path_new_bin):
    os.mkdir(path_new_bin)

  for filename in os.listdir(path):
    file_path = os.path.join(path, filename)
    # checking if it is a file
    if os.path.isfile(file_path):
      if not file_path.endswith('.bin'):
        print("File {} is not a '.bin'".format(file_path))
        continue
      # Extract with keys
      os.system("{}/camera/ground_estimation_2 {}".format(exec_path, file_path))
      # Name of the .bin
      name_bin = filename
      print("Export '.bin' to {}{}".format(path_new_bin, name_bin))

      path_to_bin = os.readlink("/tmp/export-res-latest.bin")
      # Convert to .bin
      os.system("mv {} {}{}".format(path_to_bin, path_new_bin, name_bin))