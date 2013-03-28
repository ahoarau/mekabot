#!/bin/bash

CUSTOM_DIR="custom"
CD_DIR="cd"
INITIAL_DIR=`pwd`

function generate_manifest {
	chroot $1/$CUSTOM_DIR dpkg-query -W --showformat='${Package} ${Version}\n' > $1/$CD_DIR/casper/filesystem.manifest
	cp $1/$CD_DIR/casper/filesystem.manifest $1/$CD_DIR/casper/filesystem.manifest-desktop
}

function create_squashfs {
	if [ -e $1/$CD_DIR/casper/filesystem.squashfs ]
        then
		rm $1/$CD_DIR/casper/filesystem.squashfs
        fi
	mksquashfs $1/$CUSTOM_DIR $1/$CD_DIR/casper/filesystem.squashfs
}

function update_md5sum {
	rm $1/$CD_DIR/md5sum.txt
	(cd $1/$CD_DIR && find . -type f -print0 | xargs -0 md5sum > md5sum.txt)
	cd $INITIAL_DIR
}

function create_isofs {
	cd $1/$CD_DIR/
	mkisofs -r -V "mekabot-ubuntu" -b isolinux/isolinux.bin -c isolinux/boot.cat -cache-inodes -J -l -no-emul-boot -boot-load-size 4 -boot-info-table -o $INITIAL_DIR/$2 .
	cd $INITIAL_DIR
}

if [ $# -ne 2 ]
then
  echo "Usage: `basename $0` basedir isopath"
  exit 65
fi 

echo -e "Generating manifest file ...\n"
generate_manifest $1

echo -e "Creating squashfs. It might take a long time. Be patient ...\n"
create_squashfs $1

echo -e "Updating MD5 ...\n"
update_md5sum $1

echo -e "Creating isofs ...\n"
create_isofs $1 $2
