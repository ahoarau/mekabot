#!/bin/bash

function create_directory_structure {
        # Mount the ISO
	mkdir /tmp/livecd
	mount -o loop $1 /tmp/livecd

	# Copy all the content except the squashfs to the cd dir
	mkdir -p $2/cd
	mkdir $2/squashfs
	mkdir $2/custom
	echo -e "Copying ISO cd structure ...\n"
	rsync --exclude=/casper/filesystem.squashfs -a /tmp/livecd/ $2/cd

	# Mount and copy the content of the squashfs to the custom/ dir
	mount -t squashfs -o loop /tmp/livecd/casper/filesystem.squashfs $2/squashfs/

	echo -e "Copying squashfs content ...\n"
	cp -a $2/squashfs/* $2/custom/

	chmod +w $2/cd/casper/filesystem.manifest

        umount $2/squashfs
	umount /tmp/livecd
	rm -r /tmp/livecd
}



if [ $# -ne 2 ]
then
  echo "Usage: `basename $0` isoname workingdir"
  exit 65
fi 

create_directory_structure $1 $2
