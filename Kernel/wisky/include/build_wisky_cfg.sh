#!/bin/sh

#create wisky auto config file, do't modify
#cd huang@20110429

CONFIG_BUILD_DIR=.

WISKY_ENV_FILE=${CONFIG_BUILD_DIR}/wisky_env.h
TMP_WISKY_ENV_FILE=${CONFIG_BUILD_DIR}/.wisky_env.h.tmp.h
TMP_BOARD_CFG_NAME=${CONFIG_BUILD_DIR}/.tmp_board_cfg_name
TMP_BOARD_CFG_FILE=${CONFIG_BUILD_DIR}/.tmp_board_cfg_file.h
TMP_AUTO_CFG_FILE=${CONFIG_BUILD_DIR}/.tmp_auto_cfg_file
WISKY_AUTO_CFG_FILE=${CONFIG_BUILD_DIR}/wisky_auto.conf

rm -f ${WISKY_AUTO_CFG_FILE}
cp -f ${WISKY_ENV_FILE} ${TMP_WISKY_ENV_FILE}

sed -i -e 's/[[:space:]][[:space:]]*\(#.*\)/\1/g'  ${TMP_WISKY_ENV_FILE}
sed -i -e 's/[[:space:]][[:space:]]*\(define.*\)/\1/g'  ${TMP_WISKY_ENV_FILE}

#create temp board file
sed -n -e 's/^#define * WISKY_BOARD_\(.*\)1.*/.\/cfg\/wisky_cfg_\1/p' ${TMP_WISKY_ENV_FILE} > ${TMP_BOARD_CFG_NAME}
sed -i -e 's/\r//g' ${TMP_BOARD_CFG_NAME}
sed -i -e 's/\(.*\)\/\/.*/\1/' ${TMP_BOARD_CFG_NAME}
sed -i -e 's/\(.*\)\/\*.*/\1/' ${TMP_BOARD_CFG_NAME}


sed -i -e 's/[[:space:]][[:space:]]*1//g' ${TMP_BOARD_CFG_NAME}
sed -i 's/\(.*\)/\1.h/' ${TMP_BOARD_CFG_NAME}
sed -i -e 's/[[:space:]][[:space:]]*//g' ${TMP_BOARD_CFG_NAME}
#to lower letter
path=`awk '{print tolower($1)}' ${TMP_BOARD_CFG_NAME}`

##create auto config file
cp -f ${path} ${TMP_BOARD_CFG_FILE}
sed -i -e 's/\r//g' ${TMP_BOARD_CFG_FILE}
sed -i -e 's/[[:space:]][[:space:]]*\(#.*\)/\1/g'  ${TMP_BOARD_CFG_FILE}
sed -i -e 's/[[:space:]][[:space:]]*\(define.*\)/\1/g'  ${TMP_BOARD_CFG_FILE}
sed -i -e 's/[[:space:]][[:space:]]*/ /g'  ${TMP_BOARD_CFG_FILE}
sed -i -n -e 's/^#define[[:space:]]\(WISKY_.*\)/\1/p' ${TMP_BOARD_CFG_FILE}
sed -i -e 's/\(.*\)\/\/.*/\1/' ${TMP_BOARD_CFG_FILE}
sed -i -e 's/\(.*\)\/\*.*/\1/' ${TMP_BOARD_CFG_FILE}

awk '{if($2=="1") $2="y"} {if($2=="0") $2="n"}{print}' ${TMP_BOARD_CFG_FILE} > ${TMP_AUTO_CFG_FILE}
sed -i -e 's/[[:space:]][[:space:]]*/=/' ${TMP_AUTO_CFG_FILE}

cp -f ${TMP_AUTO_CFG_FILE} ${WISKY_AUTO_CFG_FILE}

rm -f ${TMP_WISKY_ENV_FILE}
rm -f ${TMP_BOARD_CFG_NAME}
rm -f ${TMP_BOARD_CFG_FILE}
rm -f ${TMP_AUTO_CFG_FILE}



