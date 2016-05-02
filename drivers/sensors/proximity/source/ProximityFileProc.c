#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/ProximityBasic.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include "../../../../fs/proc/internal.h"

#define LOGPREFIX "[ProximityFileSource]" 
#define FILENAME_PREFIX "driver/%s"
#define MAX_SIZE_OF_PATH_NAME (50)

struct File_Proc_Status;
typedef struct File_Proc_Status{
    const char *name;
    int event_status;
    ProximityEventHandler *handler;
}File_Proc_Status;

//ASUS Shawn_Huang +++
static int readflag = 0;
static ssize_t asus_proc_file_read_file(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
    char print_buf[32];
    unsigned int ret = 0,iret = 0;
    File_Proc_Status *File_Status = PDE_DATA(file_inode(filp));

    sprintf(print_buf, "%d\n", File_Status->event_status);
    ret = strlen(print_buf);

    return simple_read_from_buffer(buff, len, off, print_buf, ret);
}

static ssize_t asus_proc_file_write_file(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
    File_Proc_Status *File_Status = PDE_DATA(file_inode(filp));
    int val;

    char messages[256];

    if (len > 256) {
        len = 256;
    }

    if (copy_from_user(messages, buff, len)) {
        return -EFAULT;
    }

    val = (int)simple_strtol(messages, NULL, 10);

    if(val >= PROXIMITY_EVNET_MAX){
        printk(KERN_ERR"%s:Wrong paramenter %d\n",LOGPREFIX, val);
        return -EFAULT;
    }

    if(File_Status->event_status != val){
        File_Status->event_status = val;
        File_Status->handler->onEvent(File_Status->handler, File_Status->name, File_Status->event_status);
    }

    return len;
}

static struct file_operations create_asus_proc_file = {
    .read = asus_proc_file_read_file,
    .write = asus_proc_file_write_file,
};

//ASUS Shawn_Huang ---

void create_ProximityTest_proc_file(
    const char *name, ProximityEventHandler *handler)
{
    File_Proc_Status *file_status = NULL;
    char buf[MAX_SIZE_OF_PATH_NAME];
    struct proc_dir_entry *proc_file = NULL;

    snprintf(buf, 
        MAX_SIZE_OF_PATH_NAME,
        FILENAME_PREFIX, 
        name);

    file_status =  kzalloc(sizeof(File_Proc_Status), GFP_KERNEL);
    if (!file_status) {
        printk("%s file_status kzalloc fail!\n", __FUNCTION__);
        return;
    }
    file_status->handler = handler;
    file_status->name = name;
    file_status->event_status = 0;
    file_status->handler->onEvent(file_status->handler, file_status->name, file_status->event_status);
    
    proc_file = proc_create_data(buf, 0644, NULL, &create_asus_proc_file, file_status);

    return;
}
