# 各个文件夹路径
DIR_OBJ =./obj
DIR_BIN = ./bin
DIR_SRC = ./coptermaster

# 执行程序，名称和路径
TARGET 		= Server
BIN_TARGET  = ${DIR_BIN}/${TARGET}

# 执行参数： 调试信息显示,不生成任何警告信息，头文件位置自定义
CFLAGS  = -std=c++11 -g -w
CC      = g++

# 所有的文件目录
SUBDIRS = $(shell find $(DIR_SRC) -maxdepth 1 -type d ! -path $(DIR_SRC))

# 找到所有cpp文件
# wildcard把 指定目录 下的所有后缀是c的文件全部展开。
# subst 制定字符串替换
# patsubst把变量符合后缀是.c的全部替换成.o
# join 字符串连接
SRCS = $(foreach n,$(SUBDIRS),$(wildcard $(n)/*.cpp))
# 编译后将文件名 改为xxx.cpp到xxxx.o，并将路径改成 Moadel/xxx.o,例如Main/test.o
# OBJ = $(foreach n,$(SRCS),$(join ./Debug/,$(notdir $(patsubst %.cpp,%.o,$(n)))))
OBJ = $(foreach n,$(SRCS), $(patsubst %.cpp,%.o,$(n)))
# ./coptermaster/StorageManager/StorageManager.o
all:
	@echo "make compile	:编译文件"
	@echo "make exec 	:执行文件"
	@echo "make clean 	:清除文件"

compile:$(BIN_TARGET)
# 编译所有的cpp文件成中间文件,加@可以不显示 语句，%表示OBJ中除.o的部分
# 例如 g++ -c -std=c++11 -g -w -I./include src./coptermatser/XXX/test.cpp -o ./Debug/test.o
$(OBJ):%.o: %.cpp
	$(CC) -c $(CFLAGS) $< -o $(join ${DIR_OBJ}/,$(notdir $@))

# 链接所有的中间文件
$(BIN_TARGET):$(OBJ)
	$(CC) $(CFLAGS) -o $@ $(foreach n,$^,$(join ${DIR_OBJ}/,$(notdir $(n))))

#执行程序
exec:
	./$(BIN_TARGET)

# 统计文件夹下所有文件
test:
	@echo $(OBJ)
