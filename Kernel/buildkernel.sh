#!/bin/sh

TOP_DIR=`pwd`

echo "========================================================="
echo "* Build wisky mid kernel start......"
echo "========================================================="

#create config file
cd wisky/include/
./build_wisky_cfg.sh
cd ${TOP_DIR}

#build logo
cd wisky/logo/build_logo/
./ppmtoc.sh
cd ${TOP_DIR}
make kernel.img -j8  || exit

#create out directory
OUTPUT_DIR=${TOP_DIR}/out
IMAGE_OUT_DIR=${OUTPUT_DIR}/image
rm -rf ${IMAGE_OUT_DIR}
mkdir -p $IMAGE_OUT_DIR
echo
echo "Copy <kernel.img> & <Image> to directory:${IMAGE_OUT_DIR}"
cp ./arch/arm/boot/Image ./kernel.img ${IMAGE_OUT_DIR}
cp ./kernel.img ${IMAGE_OUT_DIR}

echo "done"

echo "========================================================="
echo "* Build wisky mid kernel done!"
echo "========================================================="
