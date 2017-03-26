#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>   // <asus-olaf20150715+>

static struct proc_dir_entry *project_id_proc_file;
static int project_id_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_PRJ_ID);
	return 0;
}

static int project_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_id_proc_read, NULL);
}


static struct file_operations project_id_proc_ops = {
	.open = project_id_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_id_proc_file(void)
{
    printk("create_project_id_proc_file\n");
    project_id_proc_file = proc_create("apid", 0444,NULL, &project_id_proc_ops);
    if(project_id_proc_file){
        printk("create project_id_proc_file sucessed!\n");
    }else{
		printk("create project_id_proc_file failed!\n");
    }
}

// <asus-olaf20150904+>

static int __init proc_asusPRJ_init(void)
{
	create_project_id_proc_file();
	return 0;
}
module_init(proc_asusPRJ_init);
