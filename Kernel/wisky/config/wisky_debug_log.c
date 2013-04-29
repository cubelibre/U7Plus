/*
    wisky kernel debug interface -- write log info to debug file

    export global function :
	int wisky_debug_write_log_thread(char *plog);
*/

//#define WISKY_DEBUG
#include <linux/wisky_linux.h>

#include   <linux/kernel.h> 
#include   <linux/module.h> 
#include   <linux/init.h> 
#include <linux/slab.h>
#include   <linux/fs.h> 
#include   <linux/string.h> 
#include   <linux/mm.h> 
#include   <linux/syscalls.h> 
#include   <asm/unistd.h> 
#include   <asm/uaccess.h>


#define   DEBUG_LOG_FILEPATH   "/root/logfile"

struct file *debug_fp;

//
//write data in log file
//
static int thread_write_debug_log(void *p)
{
	mm_segment_t fs;
	char *buffer=(char*)p;

	fs=get_fs();
	set_fs(KERNEL_DS);

	vfs_write(debug_fp,buffer,strlen(buffer)+1,&debug_fp->f_pos);

	set_fs(fs);

	kfree(p);

	return 0;
}

//
//if succeed return 1.else function return 0
//
int wisky_debug_write_log_thread(char *plog) 
{
	int len=strlen(plog);

	char *buff=	(char*) kzalloc(len+2, GFP_KERNEL);
	strcpy(buff,plog);

	kernel_thread(thread_write_debug_log,buff,CLONE_FILES|SIGCHLD);

	return 1;
}

static int wisky_debug_log_init(void)
{
	debug_fp = filp_open(DEBUG_LOG_FILEPATH,O_RDWR | O_APPEND | O_CREAT, 0777);

	if(IS_ERR(debug_fp))  
	{ 
		WPRINTK( "WISKY_DEBUG################################## create debug file error\n "); 
		return   0; 
	} 

	return 1;
}

static void wisky_debug_log_exit(void)
{
    filp_close(debug_fp,NULL);
}

module_init(wisky_debug_log_init);
module_exit(wisky_debug_log_exit);
MODULE_LICENSE("GPL");



