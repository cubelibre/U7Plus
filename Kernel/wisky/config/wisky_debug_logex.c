/*
    wisky kernel debug interface -- write log info to debug file

    export global function :
	int wisky_debug_write_log_directly(char *plog);
*/

//#define WISKY_DEBUG
#include <linux/wisky_linux.h>

#include   <linux/kernel.h> 
#include   <linux/module.h> 
#include   <linux/init.h> 
#include   <linux/fs.h> 
#include <linux/slab.h>
#include   <linux/string.h> 
#include   <linux/mm.h> 
#include   <linux/syscalls.h> 
#include   <asm/unistd.h> 
#include   <asm/uaccess.h> 


#define   DEBUG_LOG_FILEPATHDIR   "/root/logfiledir"


//
//if succeed return 1.else function return 0
//
int wisky_debug_write_log_directlyex(char *filepath,char *plog) 
{ 
	struct file *file = NULL; 
	mm_segment_t   old_fs; 

	file =  filp_open(filepath, O_RDWR | O_APPEND | O_CREAT, 0777); 
	
	if(IS_ERR(file))  
	{ 
		WPRINTK( "WISKY_DEBUG################################## create debug file error\n "); 
		return   0; 
	} 

	old_fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file-> f_op-> write(file,(char *)plog,strlen((char*)plog),&file-> f_pos); 
	set_fs(old_fs); 

	filp_close(file,   NULL); 

	return 1;
} 

//
//if succeed return 1.else function return 0
//
int wisky_debug_write_log_directly(char *plog) 
{ 
	return wisky_debug_write_log_directlyex(DEBUG_LOG_FILEPATHDIR,plog) ;
} 


//
//if succeed return 1.else function return 0
//
void* wisky_debug_kfile_open(char *filepath)
{	
	struct file *file = NULL; 

	file =  filp_open(filepath, O_WRONLY | O_CREAT | O_APPEND | O_TRUNC, 0777); 
	
	if(IS_ERR(file))  
	{ 
		WPRINTK( "WISKY_DEBUG################################## create debug file error\n "); 
		return   0; 
	}

	return file;
}

void wisky_debug_kfile_write(void *hfile,char *plog)
{
	struct file *file =(struct file *) hfile; 
	mm_segment_t   old_fs; 

	old_fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file-> f_op-> write(file,(char *)plog,strlen((char*)plog),&file-> f_pos); 
	set_fs(old_fs); 
}

void wisky_debug_kfile_write_thread__(void *p)
{
	char *pbuff=(char*)p;
	void *hfile=(void*)(*(int*)p);

	wisky_debug_kfile_write(hfile,pbuff+4);

	kfree(p);
}

int wisky_debug_kfile_write_thread(void *hfile,char *plog) 
{
	int len=strlen(plog);

	char *buff=	(char*) kzalloc(len+8, GFP_KERNEL);
	*(int*)buff=(int)hfile;
	strcpy(buff+4,plog);

	kernel_thread(wisky_debug_kfile_write_thread__,buff,CLONE_FILES|SIGCHLD);

	return 1;
}

void wisky_debug_kfile_close(void *hfile)
{	
	struct file *file =(struct file *) hfile;

	filp_close(file, NULL);
}