* build文件夹用来存放编译过程中的中间文件，hello sec world 都是源文件，thirdparty是第三方库文件。 

* 调用顺序：src中的main文件调用hello和world中的功能函数生成的动态库，然后再调用thirdparty中的第三方库文件。
