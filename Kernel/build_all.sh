#!/bin/sh

#build wisky all project, do't modify
#cd huang@20110512

TOP_DIR=`pwd`

#create out directory
OUTPUT_DIR=${TOP_DIR}/out
rm -rf ${OUTPUT_DIR}/image_all
mkdir -p ${OUTPUT_DIR}

WISKY_ENV_FILE=${TOP_DIR}/wisky/include/wisky_env.h
WISKY_ENV_BACKUP_FILE=${TOP_DIR}/wisky/include/wisky_env.h.old
TMP_WISKY_ENV_FILE=${OUTPUT_DIR}/.wisky_env.h.tmp
TMP_ALL_PROJECT_NAME=${OUTPUT_DIR}/.tmp_all_project_name
TMP_ALL_OEM_NAME=${OUTPUT_DIR}/.tmp_all_oem_name
TMP_PROJECT_NAME=${OUTPUT_DIR}/.tmp_project_name
TMP_OEM_NAME=${OUTPUT_DIR}/.tmp_oem_name
IMAGE_OUT_DIR=${OUTPUT_DIR}/image_all

mkdir -p $IMAGE_OUT_DIR

echo ""
echo "========================================================="
echo "* Build wisky mid ALL project start......"
echo "========================================================="
echo ""

#backup wisky_env file
cp -f ${WISKY_ENV_FILE} ${WISKY_ENV_BACKUP_FILE}

sed -i 's/^#define[[:space:]][[:space:]]*WISKY_BOARD_\(.*\)1.*/\/\/&/' ${WISKY_ENV_FILE}
sed -i 's/^#define[[:space:]][[:space:]]*WISKY_OEM_\(.*\)1.*/\/\/&/' ${WISKY_ENV_FILE}

cp -f ${WISKY_ENV_FILE} ${TMP_WISKY_ENV_FILE}

sed -i -e 's/[[:space:]][[:space:]]*\(#.*\)/\1/g'  ${TMP_WISKY_ENV_FILE}
sed -i -e 's/[[:space:]][[:space:]]*\(define.*\)/\1/g'  ${TMP_WISKY_ENV_FILE}

#create project board name file
sed -n -e 's/.*#define[[:space:]][[:space:]]*WISKY_BOARD_\(.*\)1.*/WISKY_BOARD_\1/p' ${TMP_WISKY_ENV_FILE} > ${TMP_ALL_PROJECT_NAME}
sed -i -e 's/\r//g' ${TMP_ALL_PROJECT_NAME}
sed -i -e 's/\(.*\)\/\/.*/\1/' ${TMP_ALL_PROJECT_NAME}
sed -i -e 's/\(.*\)\/\*.*/\1/' ${TMP_ALL_PROJECT_NAME}

sed -i -e 's/[[:space:]][[:space:]]*1//g' ${TMP_ALL_PROJECT_NAME}
sed -i -e 's/[[:space:]][[:space:]]*//g' ${TMP_ALL_PROJECT_NAME}


#create eom name file
sed -n -e 's/.*#define[[:space:]][[:space:]]*WISKY_OEM_\(.*\)1.*/WISKY_OEM_\1/p' ${TMP_WISKY_ENV_FILE} > ${TMP_ALL_OEM_NAME}
sed -i -e 's/\r//g' ${TMP_ALL_OEM_NAME}
sed -i -e 's/\(.*\)\/\/.*/\1/' ${TMP_ALL_OEM_NAME}
sed -i -e 's/\(.*\)\/\*.*/\1/' ${TMP_ALL_OEM_NAME}

sed -i -e 's/[[:space:]][[:space:]]*1//g' ${TMP_ALL_OEM_NAME}
sed -i -e 's/[[:space:]][[:space:]]*//g' ${TMP_ALL_OEM_NAME}


while read project
do
	echo $project
	sed -i 's/^#define[[:space:]][[:space:]]*WISKY_BOARD_\(.*\)1.*/\/\/&/' ${WISKY_ENV_FILE}
	sed -i -e 's/\/\/\(.*'$project'[[:space:]][[:space:]]*.*\)/\1/' ${WISKY_ENV_FILE}
	echo $project | sed -n 's/WISKY_BOARD_\(.*\)/\1/p' > $TMP_PROJECT_NAME
	read project_name < $TMP_PROJECT_NAME
	echo ${project_name}
	while read oem
	do
		echo "***********************************************************************************"
		echo "* Build wisky mid Project ${project} with OEM ${oem} start......"
		echo "***********************************************************************************"
		
		sed -i 's/^#define[[:space:]][[:space:]]*WISKY_OEM_\(.*\)1.*/\/\/&/' ${WISKY_ENV_FILE}
		sed -i -e 's/\/\/\(.*'$oem'[[:space:]][[:space:]]*.*\)/\1/' ${WISKY_ENV_FILE}
		
		cd $TOP_DIR/wisky/include
		./build_wisky_cfg.sh
		
		cd $TOP_DIR/wisky/logo/build_logo/
		./ppmtoc.sh
		cd $TOP_DIR
		###make clean
		make kernel.img || exit
		
		echo $oem | sed -n 's/WISKY_OEM_\(.*\)/\1/p' > $TMP_OEM_NAME
		read oem_name < $TMP_OEM_NAME
		echo ${oem_name}
		
		echo "Copy target image to directory ${IMAGE_OUT_DIR}"
		#cp ./kernel.img $IMAGE_OUT_DIR/${project_name}_${oem_name}_`date +%F`_kernel.img
		#cp ./arch/arm/boot/Image $IMAGE_OUT_DIR/${project_name}_${oem_name}_`date +%F`_Image
		#copy image without time
		cp ./kernel.img $IMAGE_OUT_DIR/${project_name}_${oem_name}_kernel.img
		cp ./arch/arm/boot/Image $IMAGE_OUT_DIR/${project_name}_${oem_name}_Image
		echo "***********************************************************************************"
		echo "* Build wisky mid Project ${project} with OEM ${oem} done!"
		echo "***********************************************************************************"
		echo ""
		#sleep 2		
	done < ${TMP_ALL_OEM_NAME}
	
done < ${TMP_ALL_PROJECT_NAME}


rm -f ${TMP_WISKY_ENV_FILE}
rm -f ${TMP_ALL_PROJECT_NAME}
rm -f ${TMP_ALL_OEM_NAME}
rm -f ${TMP_PROJECT_NAME}
rm -f ${TMP_OEM_NAME}

echo ""
echo "========================================================="
echo "* Build wisky mid ALL project done!"
echo "========================================================="
echo ""

