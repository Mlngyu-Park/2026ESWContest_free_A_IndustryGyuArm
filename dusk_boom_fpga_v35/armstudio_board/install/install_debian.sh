#!/bin/sh
# install_debian.sh - run ONCE on the board (factory Debian 8, root) with SD partition 3 at /mnt/p3.
# Installs Debian 12 armhf (from the bundle made on the DGX) + ARM STUDIO into partition 3.
# Log: /mnt/p3/install.log
P=/mnt/p3
exec > $P/install.log 2>&1
set -x
cd $P || exit 1
step() { echo "STEP $1 $(date +%T)"; }

step 1-join
[ -f dusk_bookworm_armhf.tgz ] || cat parts/part?? > dusk_bookworm_armhf.tgz
echo "e4c989263e1f9ef46c8d2da6db426d1d5c1d1c5baee606c146d8769794ad62a7  dusk_bookworm_armhf.tgz" | sha256sum -c - || { echo INSTALL_FAIL bundle; exit 1; }

step 2-debootstrap
mkdir -p /root/dbs && dpkg-deb -x debootstrap.deb /root/dbs
DEBOOTSTRAP_DIR=/root/dbs/usr/share/debootstrap /root/dbs/usr/sbin/debootstrap --arch=armhf \
    --variant=minbase --include=python3,python3-numpy,python3-scipy,python3-pil,python3-fastapi,python3-uvicorn,python3-multipart,python3-plotly,python3-serial,python3-sympy,python3-websockets,xserver-xorg-core,xserver-xorg-input-libinput,xinit,xauth,x11-xserver-utils,firefox-esr,libgl1-mesa-dri,fonts-nanum,fonts-dejavu-core,udev,dbus,dbus-x11,matchbox-window-manager,procps \
    --unpack-tarball=$P/dusk_bookworm_armhf.tgz bookworm $P http://deb.debian.org/debian \
    || { echo INSTALL_FAIL debootstrap; tail -30 $P/debootstrap/debootstrap.log; exit 1; }

step 3-extra
mkdir -p $P/root/extra && tar xzf extra_debs.tgz -C $P/root/extra
mount -t proc proc $P/proc
chroot $P dpkg -i /root/extra/libspatialindex6_*.deb /root/extra/libspatialindex-c6_*.deb /root/extra/python3-rtree_*.deb \
    || echo "WARN rtree"
# 브라우저 없는 ARM STUDIO 화면: pygame (SDL2) + evdev (키보드 · 마우스 직접 읽기)
mkdir -p $P/root/extra2 && tar xzf extra2_debs.tgz -C $P/root/extra2
chroot $P sh -c 'dpkg -i /root/extra2/*.deb' || echo "WARN pygame"

step 4-armstudio
tar xzf armstudio_board.tgz -C $P --no-same-owner
echo dusk-boom > $P/etc/hostname
mkdir -p $P/var/lib/dusk $P/boot

step 4b-panda
# ARM STUDIO 본체 = 라떼판다 판 (2026-11-05) + 묶음에서 빠졌던 실행용 파일 (vocab.json 등, 이미 있는 파일은 그대로)
tar xzf armstudio_panda_1105.tgz -C $P/home/drboom --no-same-owner
tar xzf armstudio_supp_1105.tgz -C $P/home/drboom --no-same-owner --skip-old-files

step 4c-rachel-tpu
# 더블 레이첼 = FPGA TPU 판. rachel_cpp/rachel 자리에는 TPU 판을 부르는 작은 스크립트 (보드 복사본만)
mkdir -p $P/opt/dusk/rachel
cp rachel/rachel_tpu $P/opt/dusk/rachel/ && chmod +x $P/opt/dusk/rachel/rachel_tpu
cp $P/home/drboom/2026_project/Industry_gyu_arm/nano_double/double.bin $P/opt/dusk/rachel/
R=$P/home/drboom/2026_project/Industry_gyu_arm/rachel_cpp
cp rachel/rachel_cpu_armhf $R/rachel.cpu_armhf
cp rachel/rachel_wrapper.sh $R/rachel && chmod +x $R/rachel

step 5-swapfile
[ -f $P/swapfile ] || { dd if=/dev/zero of=$P/swapfile bs=1M count=512 && chmod 600 $P/swapfile && chroot $P mkswap /swapfile; }

step 6-check
chroot $P python3 -c "import numpy, scipy, sympy, fastapi, uvicorn, multipart, plotly, PIL, serial, rtree; import sys; sys.path.insert(0,'/opt/dusk/py'); import trimesh, ikpy, scservo_sdk, gradio, fast_simplification, dusk_board_patches, pygame, evdev; print('PY_IMPORTS_OK', numpy.__version__, fastapi.__version__, trimesh.__version__)"
chroot $P sh -c 'ls /usr/lib/xorg/Xorg /usr/bin/firefox-esr /usr/bin/xinit /usr/bin/matchbox-window-manager /lib/systemd/systemd-udevd'
umount $P/proc
sync
df -h $P
echo INSTALL_DONE
