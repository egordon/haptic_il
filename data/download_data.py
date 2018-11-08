#!/usr/bin/python3

import sys
import urllib.request
import tarfile
import errno    
import os

# Get current script directory
script_dir = sys.path[0]

# Make folder if not exist
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

def downloadFile(url, file_name):
	"""
	Writes file from "url" to "file_name"
	Adds progress bar
	"""
	u = urllib.request.urlopen(url)
	f = open(file_name, 'wb')
	file_size = int(u.getheader("Content-Length"))
	print("Downloading: %s\nBytes: %s" % (file_name, file_size))

	file_size_dl = 0
	block_sz = 8192
	while True:
		buffer = u.read(block_sz)
		if not buffer:
			break
		file_size_dl += len(buffer)
		f.write(buffer)
		status = r"%10d  [%3.2f%%]" % (file_size_dl, file_size_dl * 100. / file_size)
		status = status + chr(8)*(len(status)+1)
		sys.stdout.write("\r" + status)
		sys.stdout.flush()
	f.close()
	print()

if __name__ == "__main__":
	print("Note: this repository focuses on downloading the dataset for scooping potato salad.")
	print("Downloading files from Harvard dataverse...")

	fileList = ["JZ6HSW", "88GEPP", "4HT4IL", "0LOI7G", "IEAS8J", "GKMZ4K", "8GMOOI", "CVIQ3P", "RGFXQR", "WYWS5Y", "FW0I3U", "M7WKLL"]
	url_base = "https://dataverse.harvard.edu/api/access/datafile/:persistentId?persistentId=doi:10.7910/DVN/8TTXZ7/"
	file_base = script_dir + "/dataverse"
	mkdir_p(file_base)
	unpack_base = script_dir + "/raw"
	mkdir_p(unpack_base)

	for sNum in range(12):
		url = url_base + fileList[sNum]
		file_name = file_base + "/subject" + str(sNum + 1) + "_potato_salad_wrenches_poses.tar.gz"
		downloadFile(url, file_name)
		print("Unpacking file...")
		tar = tarfile.open(file_name, "r:gz")
		tar.extractall(path=unpack_base)
		tar.close()
