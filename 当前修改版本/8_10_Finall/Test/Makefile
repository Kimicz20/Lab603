# 循环编译 两个子程序的 make文件
# 需要make的文件目录
SUBDIRS = Controller\
	Function\
	Performance

all:
	@echo "make compile	:编译文件"
	@echo "make exec 	:执行文件"

compile_all:$(SUBDIRS)

$(SUBDIRS): CHO
	@make -s -C $@ compile

compile_c:
	@make -s -C Controller compile

CHO:
	@echo -e "############		开始循环编译cpp文件		############\\n\\t编译Controller文件"

exec:ECHO
	@make -s -C Controller exec

ECHO:
	@echo "############		执行控制程序		############"
