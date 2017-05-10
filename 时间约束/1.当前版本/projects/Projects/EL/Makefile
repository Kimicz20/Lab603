# 循环编译 两个子程序的 make文件
# 需要make的文件目录
SUBDIRS = Controller\
	Coptermaster

all:
	@echo "make compile	:编译文件"
	@echo "make exec 	:执行文件"
	@echo "make clean 	:清除文件"

compile_all:$(SUBDIRS)

$(SUBDIRS): CHO
	@make -s -C $@ compile

CHO:
	@echo -e "############		开始循环编译cpp文件		############\\n\\t编译Controller文件"

compile_c:
	@make -s -C Controller compile

compile_p:
	@make -s -C ElevatorSimulation compile

exec:ECHO
	@make -s -C Controller exec
execp:
	@make -s -C ElevatorSimulation exec
ECHO:
	@echo "############		执行控制程序		############"
