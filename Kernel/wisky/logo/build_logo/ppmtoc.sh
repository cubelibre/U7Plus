#!/bin/sh

LOGO_DIR=`pwd`
PPM_DIR=${LOGO_DIR}/ppm
LOGO_DATA_DIR=${LOGO_DIR}/../logo_data_all

rm -rf ${TARGET_FILE}
rm -rf ${LOGO_DATA_DIR}

mkdir ${LOGO_DATA_DIR}

echo "Build boot logo start..."

for logo_path in $PPM_DIR/*.ppm;do
	#Ŀ��ͼƬ������·��
	LOGO_FILE=${logo_path#${PPM_DIR}/*}
	#Ŀ��ͼƬ����������׺	
	LOGO_NAME=${LOGO_FILE%.ppm}
	echo " $LOGO_NAME"
	
	${PPM_DIR}/pnmtologo -t clut224 -n ${LOGO_NAME} -o ${LOGO_DATA_DIR}/${LOGO_NAME}.c ${PPM_DIR}/${LOGO_NAME}.ppm
done

echo "Build boot logo finish."
echo
