import os
import shutil
import sys

import bag_to_folder as b2f

bags = ["20230920/data1", "20230920/data2", "20230921/data4", "20230921/data2", "20230921/data3", "20230921/data5", 
"20231007/data4", "20231007/data2", "20231019/data2",
"20231105/data6", "20231019/data1", "20231105/data4",
"20231105_aft/data1", "20231105/data1", "20231105/data5",
"20231105_aft/data2", "20231105/data2", "20231105_aft/data5",
  "20231105/data3", "20231109/data4", "20231105_aft/data4",  
  "20231109/data3", "20231208/data1", "20231201/data2",
"20231213/data1", "20231201/data3", "20240113/data1",
"20231208/data5", "20231213/data2", "20231213/data3",
  "20240113/data2", "20240113/data3", "20240116_eve/data4",
"20240116/data2", "20231208/data4", "20240113/data5",
"20240116_eve/data3", "20231213/data4", "20240115/data2",
"20240123/data2", "20231213/data5", "20240116/data4",
  "20240115/data3",  "20240116/data5",   "20240116_eve/data5", 
  "20240123/data3"]


def getBagFullPaths(folders, bags):
    fullbagpaths = []
    for bag in bags:
        for folder in folders:
            date = bag.split("/")[0]
            run = bag.split("/")[1]
            path = os.path.join(folder, date, run + "_aligned.bag")
            if os.path.exists(path):
                fullbagpaths.append(path)

    return fullbagpaths


def sizeOfBags(bags):
    totalsize = 0
    for bag in bags:
        size = os.path.getsize(bag)
        print("{}: {}".format(bag, size))
        totalsize += size
    return totalsize

def zipBags(bags, outputfolder):
    for bag in bags:
        print("zipping bag {}".format(bag))
        bagname = os.path.basename(bag)
        bagdir = os.path.dirname(bag)
        date = os.path.basename(bagdir)
        run = bagname.split('_')[0]
        newfolder = os.path.join(outputfolder, date)
        if not os.path.exists(newfolder):
            os.makedirs(newfolder)
        # create a symlink for bag
        newbagname = os.path.join(newfolder, run + ".bag")
        if bag != newbagname:
            shutil.copy(bag, newbagname)
        zipname = os.path.join(newfolder, run + ".bag")
        print("Zipping {} to {}".format(bag, zipname))
        shutil.make_archive(zipname, 'zip', newfolder, run +".bag")
        os.remove(newbagname)

def bagToFolder(bags, outputfolder):
    folders = []
    for bag in bags:
        bagname = os.path.basename(bag)
        bagdir = os.path.dirname(bag)
        date = os.path.basename(bagdir)
        run = bagname.split('_')[0]
        newfolder = os.path.join(outputfolder, date, run)
        if not os.path.exists(newfolder):
            os.makedirs(newfolder)
        print("Converting bag {} to folder {}".format(bag, newfolder))
        b2f.extract_and_save(bag, newfolder)
        folders.append(newfolder)
    return folders

def zipFolders(folders, outputfolder):
    """
    zip folders and put the zip files in output folder
    """
    for folder in folders:
        print("Zipping folder {}".format(folder))
        date = os.path.basename(os.path.dirname(folder))
        run = os.path.basename(folder)
        zipname = os.path.join(outputfolder, date, run)
        print("Zipping {} to {}".format(folder, zipname))
        shutil.make_archive(zipname, 'zip', folder)
        # shutil.rmtree(folder)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python zip_bags.py <inputdir> <outputdir>")
        sys.exit(1)
    inputfolder = sys.argv[1]
    outputfolder = sys.argv[2]
    sortedbags = sorted(bags)
    fullbagpaths = getBagFullPaths([inputfolder], sortedbags)
    print("Number of full bag paths {}".format(len(fullbagpaths)))
    totalsize = sizeOfBags(fullbagpaths)
    print("Total size of bags: {}".format(totalsize))
    print("Output folder {}".format(outputfolder))
    zipBags(fullbagpaths, outputfolder)
    tmpfolder = os.path.join(outputfolder, "tmp")
    folders = bagToFolder(fullbagpaths, tmpfolder)
    zipFolders(folders, outputfolder)
    print("Done")

