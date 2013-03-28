#!/bin/bash

CUSTOM_DIR="custom"

function mount_fs {
	mount --bind /proc $1/$CUSTOM_DIR/proc
	mount --bind /sys $1/$CUSTOM_DIR/sys
	mount --bind /dev $1/$CUSTOM_DIR/dev
	mount --bind /dev/pts $1/$CUSTOM_DIR/dev/pts

	touch $1/$CUSTOM_DIR/etc/resolv.conf
	mount --bind /etc/resolv.conf $1/$CUSTOM_DIR/etc/resolv.conf
}

function umount_fs {
	umount -l $1/$CUSTOM_DIR/dev/pts
	umount -l $1/$CUSTOM_DIR/dev
	umount -l $1/$CUSTOM_DIR/sys
	umount -l $1/$CUSTOM_DIR/proc

	umount -l $1/$CUSTOM_DIR/etc/resolv.conf
}

function install_autoclean {

	cp -a $1/$CUSTOM_DIR/etc/bash.bashrc $1/$CUSTOM_DIR/etc/bash.bashrc_original
	echo '
		mv /etc/bash.bashrc_original /etc/bash.bashrc
		/bin/bash

		echo -e "Cleaning up the chroot..."
		apt-get clean
		rm -rf /tmp/*
        echo -e "Updating database for mlocate (updatedb) ..."
        updatedb
		exit' >> $1/$CUSTOM_DIR/etc/bash.bashrc

}

function chroot_iso {
	mount_fs $1

	echo -e "Chrooting into the working ISO $1 ... \n"

	install_autoclean $1

	chroot $1/$CUSTOM_DIR

	umount_fs $1

	#cleanup $1
}

if [ ! -n "$1" ]
then
  echo "Usage: `basename $0` basedir"
  exit 65
fi 

chroot_iso $1
