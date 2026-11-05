#!/bin/bash
# build_linux.sh — DUSK_BOOM v3.5 Zynq Linux for the microSD (runs on the DGX Spark, aarch64 host)
#
#   ./build_linux.sh            all stages
#   ./build_linux.sh uboot      only some stages: fetch uboot busybox robotd rootfs kernel sd
#
# Sources / tools live in $WORK (default ~/opt/dusk_zynq_linux):
#   arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-linux-gnueabihf   (ARM, aarch64-hosted)
#   u-boot-xlnx  tag xilinx-v2024.2 · linux-xlnx tag xilinx-v2024.2 (6.6) · busybox 1.36.1
# Output -> ../sd/ : u-boot.elf (for BOOT.bin), zImage (kernel + built-in root fs), system.dtb
# CPU: at most 10 cores on the Spark (taskset -c 0-9, make -j10).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${WORK:-$HOME/opt/dusk_zynq_linux}
TC=$WORK/arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-linux-gnueabihf
export PATH=$TC/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabihf-
J=10
RUN="taskset -c 0-9"
OUT=$HERE/../sd
ROOTFS=$WORK/rootfs
mkdir -p "$OUT"

stages=${*:-"fetch uboot busybox robotd rootfs kernel sd"}
has() { [[ " $stages " == *" $1 "* ]]; }

# --------------------------------------------------------------------------- fetch
if has fetch; then
    cd "$WORK"
    T=arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-linux-gnueabihf
    [ -d $T ] || { curl -sSL -o dl/$T.tar.xz https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/$T.tar.xz && tar xf dl/$T.tar.xz; }
    [ -d u-boot-xlnx ] || git clone -q --depth 1 -b xilinx-v2024.2 https://github.com/Xilinx/u-boot-xlnx.git
    [ -d linux-xlnx ]  || git clone -q --depth 1 -b xilinx-v2024.2 https://github.com/Xilinx/linux-xlnx.git
    [ -d busybox-1.36.1 ] || { curl -sSL -o dl/busybox-1.36.1.tar.bz2 https://busybox.net/downloads/busybox-1.36.1.tar.bz2 && tar xf dl/busybox-1.36.1.tar.bz2; }
fi

# add "<name>.dtb" to a dts Makefile list right after an existing entry (once)
add_dtb() {   # makefile existing.dtb new.dtb
    grep -q "$3" "$1" || sed -i "s|\t$2 \\\\|\t$2 \\\\\n\t$3 \\\\|" "$1"
    grep -q "$3" "$1" || { echo "could not add $3 to $1"; exit 1; }
}

# --------------------------------------------------------------------------- U-Boot
if has uboot; then
    cd "$WORK/u-boot-xlnx"
    cp "$HERE/dts/uboot-zynq-dusk-boom.dts" arch/arm/dts/zynq-dusk-boom.dts
    add_dtb arch/arm/dts/Makefile zynq-zed.dtb zynq-dusk-boom.dtb
    make -s xilinx_zynq_virt_defconfig
    scripts/kconfig/merge_config.sh -m .config "$HERE/config/uboot_dusk.config" >/dev/null
    make -s olddefconfig
    # the SPL path needs board/xilinx/zynq/zynq-dusk-boom/ps7_init_gpl.c (make_sd_from_xsa.sh);
    # without it only U-Boot proper (u-boot.elf, loaded by the FSBL) is used
    $RUN make -s -j$J NO_SDL=1 2>&1 | grep -v "custom_hw_platform\|ps7_init_gpl" || true
    test -f u-boot.elf
    cp u-boot.elf "$OUT/u-boot.elf"
    echo "[uboot] $(ls -la u-boot.elf)"
fi

# --------------------------------------------------------------------------- busybox
if has busybox; then
    cd "$WORK/busybox-1.36.1"
    if [ ! -f .config ]; then
        make -s defconfig
        sed -i 's/^# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
        # tc does not build with recent kernel headers; not needed
        sed -i 's/^CONFIG_TC=y/# CONFIG_TC is not set/' .config
        make -s oldconfig </dev/null >/dev/null
    fi
    $RUN make -s -j$J
    rm -rf _install && make -s install >/dev/null
    echo "[busybox] $(file busybox | cut -d, -f1-2)"
fi

# --------------------------------------------------------------------------- robotd
if has robotd; then
    make -s -C "$HERE/robotd" CC=${CROSS_COMPILE}gcc clean all
    echo "[robotd] $(ls "$HERE/robotd/build/")"
fi

# --------------------------------------------------------------------------- root file system
if has rootfs; then
    rm -rf "$ROOTFS"
    mkdir -p "$ROOTFS"
    cp -a "$WORK/busybox-1.36.1/_install/." "$ROOTFS/"
    mkdir -p "$ROOTFS"/{dev,proc,sys,tmp,run,root,boot,mnt,var/log,usr/share/udhcpc}
    cp -a "$HERE/rootfs_overlay/." "$ROOTFS/"
    cp "$WORK/busybox-1.36.1/examples/udhcp/simple.script" "$ROOTFS/usr/share/udhcpc/default.script"
    chmod +x "$ROOTFS/usr/share/udhcpc/default.script" "$ROOTFS/etc/init.d/rcS" "$ROOTFS"/etc/dusk/*.sh
    install -m 755 "$HERE/robotd/build/dusk_robotd" "$HERE/robotd/build/dusk_servo" "$ROOTFS/usr/bin/"
    install -m 755 "$HERE/robotd/dusk_esp" "$ROOTFS/usr/bin/"
    ln -sf sbin/init "$ROOTFS/init"
    # device nodes the kernel needs before devtmpfs is mounted (initramfs list file)
    cat > "$WORK/initramfs.list" <<EOF
dir /dev 755 0 0
nod /dev/console 600 0 0 c 5 1
nod /dev/null 666 0 0 c 1 3
EOF
    echo "[rootfs] $(du -sh "$ROOTFS" | cut -f1)"
fi

# --------------------------------------------------------------------------- kernel
if has kernel; then
    cd "$WORK/linux-xlnx"
    cp "$HERE/dts/zynq-dusk-boom.dts" arch/arm/boot/dts/xilinx/zynq-dusk-boom.dts
    add_dtb arch/arm/boot/dts/xilinx/Makefile zynq-zed.dtb zynq-dusk-boom.dtb
    make -s xilinx_zynq_defconfig
    scripts/kconfig/merge_config.sh -m .config "$HERE/config/kernel_dusk.config" >/dev/null
    scripts/config --set-str INITRAMFS_SOURCE "$ROOTFS $WORK/initramfs.list"
    # files of the build user become root:root inside the initramfs
    scripts/config --set-val INITRAMFS_ROOT_UID "$(id -u)" --set-val INITRAMFS_ROOT_GID "$(id -g)"
    make -s olddefconfig
    $RUN make -s -j$J zImage xilinx/zynq-dusk-boom.dtb
    for o in USB_SERIAL_CH341 USB_ACM USB_CHIPIDEA_HOST USB_ULPI_VIEWPORT GPIO_SYSFS LEDS_TRIGGER_HEARTBEAT; do
        grep -q "^CONFIG_$o=y" .config || { echo "kernel option $o missing"; exit 1; }
    done
    echo "[kernel] $(ls -la arch/arm/boot/zImage)"
fi

# --------------------------------------------------------------------------- SD files
if has sd; then
    cp "$WORK/linux-xlnx/arch/arm/boot/zImage" "$OUT/zImage"
    cp "$WORK/linux-xlnx/arch/arm/boot/dts/xilinx/zynq-dusk-boom.dtb" "$OUT/system.dtb"
    cp "$WORK/u-boot-xlnx/u-boot.elf" "$OUT/u-boot.elf"
    ( cd "$OUT" && sha256sum u-boot.elf zImage system.dtb > SHA256SUMS )
    echo "[sd] $OUT:"; ls -la "$OUT"
fi
