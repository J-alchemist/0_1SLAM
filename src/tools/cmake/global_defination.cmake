#设置WORK_SPACE_PATH的值 要与global_defination.h.in里@  @之间的对应
set(WS_PATH ${PROJECT_SOURCE_DIR}/..)

#configure_file：将global_defination.h.in里@  @标注的变量值替换为 ${PROJECT_SOURCE_DIR}，并生成.h文件
configure_file (
    ${PROJECT_SOURCE_DIR}/include/tools/global_defination.h.in
    ${PROJECT_SOURCE_DIR}/include/tools/global_defination.h
)








