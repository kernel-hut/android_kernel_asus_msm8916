/* //20100930 jack_wong for asus debug mechanisms +++++
 *  asusdebug.c
 * //20100930 jack_wong for asus debug mechanisms -----
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/export.h>
#include <linux/slab.h>
//+++ [ZC550KL] ASUS_BSP suri_gu@asus.com for screen off in factory mode when sys.screentimeout=1
#include <linux/switch.h>
//---
#include <linux/err.h>
#include <linux/oem_functions.h>
#ifdef ASUS_ZC550KL_PROJECT
extern int g_uart_dbg_mode;
#else
extern int g_user_dbg_mode;
#endif

#include <linux/rtc.h>
#include "rtmutex_common.h"
//add dump_boot_reasons ++++
#include <soc/qcom/smem.h>
//add dump_boot_reasons ----
#ifdef CONFIG_HAS_EARLYSUSPEND
int entering_suspend = 0;
#endif
phys_addr_t PRINTK_BUFFER_PA = 0x8FE00000;
void *PRINTK_BUFFER_VA;
phys_addr_t RTB_BUFFER_PA = 0x8FE00000 + SZ_1M;
extern struct timezone sys_tz;
#define RT_MUTEX_HAS_WAITERS	1UL
#define RT_MUTEX_OWNER_MASKALL	1UL
struct mutex fake_mutex;
struct completion fake_completion;
struct rt_mutex fake_rtmutex;
int asus_rtc_read_time(struct rtc_time *tm)
{
    struct timespec ts;
    getnstimeofday(&ts);
    ts.tv_sec -= sys_tz.tz_minuteswest * 60;
    rtc_time_to_tm(ts.tv_sec, tm);
    printk("now %04d%02d%02d-%02d%02d%02d, tz=%d\r\n", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, sys_tz.tz_minuteswest);
    return 0;
}
EXPORT_SYMBOL(asus_rtc_read_time);
#if 1
//--------------------   debug message logger   ------------------------------------
struct workqueue_struct *ASUSDebugMsg_workQueue;
EXPORT_SYMBOL(ASUSDebugMsg_workQueue);
//--------------  phone hang log part  --------------------------------------------------
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
//                             all thread information
///////////////////////////////////////////////////////////////////////////////////////////////////

//+++ [ZC550KL] ASUS_BSP suri_gu@asus.com for screen off in factory mode when sys.screentimeout=1
static int screenofftimeout = 0;//0 not timeout , 1 timeout
struct switch_dev asus_screenofftimeout_switch;
static ssize_t asus_screenofftimeout_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "screenofftimeout\n");
}

static ssize_t asus_screenofftimeout_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", screenofftimeout);
}

static int AsusscreenofftimeoutSwitchInitialize(void)
{
	int ret = 0;
	printk(" %s: register switch dev! %d\n", __FUNCTION__, ret);
	asus_screenofftimeout_switch.name = "screenofftimeout";
	asus_screenofftimeout_switch.print_state = asus_screenofftimeout_switch_state;
	asus_screenofftimeout_switch.print_name = asus_screenofftimeout_switch_name;
	ret = switch_dev_register(&asus_screenofftimeout_switch);
	if (ret < 0) {
		printk(" %s: Unable to register screenofftimeout dev! %d\n", __FUNCTION__, ret);
		return -1;
	}
	return 0;
}



static ssize_t asus_screenofftimeout_switch_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	return sprintf(buffer, "%d\n", screenofftimeout);
}

static ssize_t asus_screenofftimeout_switch_store(struct file *dev, const char *buff, size_t count, loff_t *loff)
{
	int set_val = -1;
	int real_set_system_mode = -1;
	char messages[8];
	if (count > 8) {
		count = 8;
	}
	if (copy_from_user(messages, buff, count)) {
		pr_err("copy_from_user fail !!\n");
		return -EFAULT;
	}
	sscanf(messages, "%d",&set_val);
	if(set_val<=0)
	{
		real_set_system_mode = 0;
	}
	else
	{
		real_set_system_mode = 1;
	}
	printk(" %s:  %d\n", __FUNCTION__, real_set_system_mode);
	screenofftimeout = real_set_system_mode;
	switch_set_state(&asus_screenofftimeout_switch, screenofftimeout);
	return count;
}


static const struct file_operations asus_screenofftimeout_switch_proc_fops = {
	.read = asus_screenofftimeout_switch_show,
	.write = asus_screenofftimeout_switch_store,
};
//---

/*
 * memset for non cached memory
 */
void *memset_nc(void *s, int c, size_t count)
{
	u8 *p = s;
	while (count--)
		*p++ = c;
	return s;
}
EXPORT_SYMBOL(memset_nc);

static char* g_phonehang_log;
static int g_iPtr = 0;
int save_log(const char *f, ...)
{
    va_list args;
    int len;

    if (g_iPtr < PHONE_HANG_LOG_SIZE)
    {
        va_start(args, f);
        len = vsnprintf(g_phonehang_log + g_iPtr, PHONE_HANG_LOG_SIZE - g_iPtr, f, args);
        va_end(args);
        //printk("%s", g_phonehang_log + g_iPtr);
        if (g_iPtr < PHONE_HANG_LOG_SIZE)
        {
            g_iPtr += len;
            return 0;
        }
    }
    g_iPtr = PHONE_HANG_LOG_SIZE;
    return -1;
}

static char *task_state_array[] = {
    "RUNNING",      /*  0 */
    "INTERRUPTIBLE",     /*  1 */
    "UNINTERRUPTIB",   /*  2 */
    "STOPPED",      /*  4 */
    "TRACED", /*  8 */
    "EXIT ZOMBIE",       /* 16 */
    "EXIT DEAD",      /* 32 */
    "DEAD",      /* 64 */
    "WAKEKILL",      /* 128 */
    "WAKING",     /* 256 */
    "PARKED"      /* 512 */
};
struct thread_info_save;
struct thread_info_save
{
    struct task_struct *pts;
    pid_t pid;
    u64 sum_exec_runtime;
    u64 vruntime;
    struct thread_info_save* pnext;
};
static char * print_state(long state)
{
    int i;
    if(state == 0)
        return task_state_array[0];
    for(i = 1; i <= 10; i++)
    {
        if(1<<(i-1) & state)
            return task_state_array[i];
    }
    return "NOTFOUND";

}

/*
 * Ease the printing of nsec fields:
 */
static long long nsec_high(unsigned long long nsec)
{
    if ((long long)nsec < 0) {
        nsec = -nsec;
        do_div(nsec, 1000000);
        return -nsec;
    }
    do_div(nsec, 1000000);

    return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
    unsigned long long nsec1;
    if ((long long)nsec < 0)
        nsec = -nsec;

    nsec1 =  do_div(nsec, 1000000);
    return do_div(nsec1, 1000000);
}
#define MAX_STACK_TRACE_DEPTH   64
struct stack_trace_data {
    struct stack_trace *trace;
    unsigned int no_sched_functions;
    unsigned int skip;
};

struct stackframe {
    unsigned long fp;
    unsigned long sp;
    unsigned long lr;
    unsigned long pc;
};
int unwind_frame(struct stackframe *frame);
void notrace walk_stackframe(struct stackframe *frame,
             int (*fn)(struct stackframe *, void *), void *data);

void show_stack1(struct task_struct *p1, void *p2)
{
    struct stack_trace trace;
    unsigned long *entries;
    int i;

    entries = kmalloc(MAX_STACK_TRACE_DEPTH * sizeof(*entries), GFP_KERNEL);
    if (!entries)
    {
        printk("entries malloc failure\n");
        return;
    }
    trace.nr_entries    = 0;
    trace.max_entries   = MAX_STACK_TRACE_DEPTH;
    trace.entries       = entries;
    trace.skip      = 0;
    save_stack_trace_tsk(p1, &trace);

    for (i = 0; i < trace.nr_entries; i++)
    {
        if (entries[i] != ULONG_MAX)
        {
            save_log("%pS\n", (void *)entries[i]);
        }
    }
    kfree(entries);
}


#define SPLIT_NS(x) nsec_high(x), nsec_low(x)
void print_all_thread_info(void)
{
    struct task_struct *pts;
    struct thread_info *pti;
    struct rtc_time tm;
    asus_rtc_read_time(&tm);

    #if 1
    g_phonehang_log = (char*)PHONE_HANG_LOG_BUFFER;//phys_to_virt(PHONE_HANG_LOG_BUFFER);
    g_iPtr = 0;
    memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);
    #endif

    save_log("PhoneHang-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);
    save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-Binder----Waiting\r\n");

    for_each_process(pts)
    {
        pti = task_thread_info(pts);
        save_log("-----------------------------------------------------\r\n");
        save_log(" %-7d", pts->pid);

        if(pts->parent)
            save_log("%-8d", pts->parent->pid);
        else
            save_log("%-8d", 0);

        save_log("%-20s", pts->comm);
        save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
        save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
        save_log("%-5d", pts->static_prio);
        save_log("%-5d", pts->normal_prio);
        save_log("%-15s", print_state((pts->state & TASK_REPORT) | pts->exit_state));

#ifndef ASUS_SHIP_BUILD
//        save_log("%-6d", pts->binder_call_to_proc_pid);
#endif
        save_log("%-6d", pti->preempt_count);


        if(pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL)
        {
			if (pti->pWaitingMutex->name)
			{
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			}
			else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug)
			{
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm)
			{
				save_log(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}

		if(pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion!=NULL)
		{
			if (pti->pWaitingCompletion->name)
	            save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name );
	        else
				printk("pti->pWaitingCompletion->name == NULL\r\n");
		}

        if(pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL)
        {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
            else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}

	save_log("\r\n");
        show_stack1(pts, NULL);

        save_log("\r\n");

        if(!thread_group_empty(pts))
        {
            struct task_struct *p1 = next_thread(pts);
            do
            {
                pti = task_thread_info(p1);
                save_log(" %-7d", p1->pid);

                if(pts->parent)
                    save_log("%-8d", p1->parent->pid);
                else
                    save_log("%-8d", 0);

                save_log("%-20s", p1->comm);
                save_log("%lld.%06ld", SPLIT_NS(p1->se.sum_exec_runtime));
                save_log("     %lld.%06ld     ", SPLIT_NS(p1->se.vruntime));
                save_log("%-5d", p1->static_prio);
                save_log("%-5d", p1->normal_prio);
                save_log("%-15s", print_state((p1->state & TASK_REPORT) | p1->exit_state));
#ifndef ASUS_SHIP_BUILD
//                save_log("%-6d", pts->binder_call_to_proc_pid);
#endif
                save_log("%-6d", pti->preempt_count);

        if(pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL)
        {
			if (pti->pWaitingMutex->name)
			{
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			}
			else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug)
			{
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm)
			{
				save_log(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}

		if(pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion!=NULL)
		{
			if (pti->pWaitingCompletion->name)
	            save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name );
	        else
				printk("pti->pWaitingCompletion->name == NULL\r\n");
		}

        if(pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL)
        {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
            else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}
                save_log("\r\n");
                show_stack1(p1, NULL);

                save_log("\r\n");
                p1 = next_thread(p1);
            }while(p1 != pts);
        }
        save_log("-----------------------------------------------------\r\n\r\n\r\n");

    }
    save_log("\r\n\r\n\r\n\r\n");


    #if 1
    //iounmap(g_phonehang_log);
    #endif
}

struct thread_info_save *ptis_head = NULL;
int find_thread_info(struct task_struct *pts, int force)
{
    struct thread_info *pti;
    struct thread_info_save *ptis, *ptis_ptr;
    u64 vruntime = 0, sum_exec_runtime;

    if(ptis_head != NULL)
    {
        ptis = ptis_head->pnext;
        ptis_ptr = NULL;
        //printk("initial ptis %x,\n\r", ptis);
        while(ptis)
        {
            //printk("initial ptis->pts %x, pts=%x\n\r", ptis->pts, pts);
            if(ptis->pid == pts->pid && ptis->pts == pts)
            {
                //printk("found pts=%x\n\r", pts);
                ptis_ptr = ptis;
                break;
            }
            ptis = ptis->pnext;
        }
        //printk("ptis_ptr=%x\n\r", ptis_ptr);
        if(ptis_ptr)
        {
            //printk("found ptis->pid=%d, sum_exec_runtime  new:%lld.%06ld  old:%lld.%06ld \n\r", ptis->pid, SPLIT_NS(pts->se.sum_exec_runtime), SPLIT_NS(ptis->sum_exec_runtime));
            sum_exec_runtime = pts->se.sum_exec_runtime - ptis->sum_exec_runtime;
            //printk("difference=%lld.%06ld  \n\r", SPLIT_NS(sum_exec_runtime));
        }
        else
        {
            sum_exec_runtime = pts->se.sum_exec_runtime;
        }
        //printk("utime=%d, stime=%d\n\r", utime, stime);
        if(sum_exec_runtime > 0 || force)
        {
            pti = task_thread_info(pts);
            save_log(" %-7d", pts->pid);

            if(pts->parent)
                save_log("%-8d", pts->parent->pid);
            else
                save_log("%-8d", 0);

            save_log("%-20s", pts->comm);
            save_log("%lld.%06ld", SPLIT_NS(sum_exec_runtime));
            if(nsec_high(sum_exec_runtime) > 1000)
                save_log(" ******");
            save_log("     %lld.%06ld     ", SPLIT_NS(vruntime));
            save_log("%-5d", pts->static_prio);
            save_log("%-5d", pts->normal_prio);
            save_log("%-15s", print_state(pts->state));
            save_log("%-6d", pti->preempt_count);

        if(pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL)
        {
			if (pti->pWaitingMutex->name)
			{
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			}
			else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug)
			{
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm)
			{
				save_log(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}

            if(pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion!=NULL)
			{
				if (pti->pWaitingCompletion->name)
					save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name );
				else
					printk("pti->pWaitingCompletion->name == NULL\r\n");
			}

        if(pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL)
        {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
            else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}

            save_log("\r\n");

            show_stack1(pts, NULL);
            save_log("\r\n");
        }
        else
            return 0;
    }
    return 1;

}

void save_all_thread_info(void)
{
    struct task_struct *pts;
    struct thread_info *pti;
    struct thread_info_save *ptis = NULL, *ptis_ptr = NULL;
#ifndef ASUS_SHIP_BUILD
//    struct worker *pworker ;
#endif

    struct rtc_time tm;
    asus_rtc_read_time(&tm);
    #if 1
    g_phonehang_log = (char*)PHONE_HANG_LOG_BUFFER;//phys_to_virt(PHONE_HANG_LOG_BUFFER);
    g_iPtr = 0;
    memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);
    #endif

    save_log("ASUSSlowg-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);
    save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-binder----Waiting\r\n");


    if(ptis_head != NULL)
    {
        struct thread_info_save *ptis_next = ptis_head->pnext;
        struct thread_info_save *ptis_next_next;
        while(ptis_next)
        {
            ptis_next_next = ptis_next->pnext;
            kfree(ptis_next);
            ptis_next = ptis_next_next;
        }
        kfree(ptis_head);
        ptis_head = NULL;
    }

    if(ptis_head == NULL)
    {
        ptis_ptr = ptis_head = kmalloc(sizeof( struct thread_info_save), GFP_KERNEL);
        if(!ptis_head)
        {
            printk("kmalloc ptis_head failure\n");
            return;
        }
        memset(ptis_head, 0, sizeof( struct thread_info_save));
    }

    for_each_process(pts)
    {
        pti = task_thread_info(pts);
        //printk("for pts %x, ptis_ptr=%x\n\r", pts, ptis_ptr);
        ptis = kmalloc(sizeof( struct thread_info_save), GFP_KERNEL);
        if(!ptis)
        {
            printk("kmalloc ptis failure\n");
            return;
        }
        memset(ptis, 0, sizeof( struct thread_info_save));

        save_log("-----------------------------------------------------\r\n");
        save_log(" %-7d", pts->pid);
        if(pts->parent)
            save_log("%-8d", pts->parent->pid);
        else
            save_log("%-8d", 0);

        save_log("%-20s", pts->comm);
        save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
        save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
        save_log("%-5d", pts->static_prio);
        save_log("%-5d", pts->normal_prio);
        save_log("%-15s", print_state((pts->state & TASK_REPORT) | pts->exit_state));
#ifndef ASUS_SHIP_BUILD
//        save_log("call_proc:%-6d  ", pts->binder_call_to_proc_pid);
 //       save_log("call_thread:%-6d  ", pts->binder_call_to_thread_pid);
 //       save_log("call_code:%-6d  ", pts->binder_call_code);
#endif
        save_log("%-6d", pti->preempt_count);

        if(pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL)
        {
			if (pti->pWaitingMutex->name)
			{
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			}
			else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug)
			{
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm)
			{
				save_log(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}
	if(pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion!=NULL)
	{
		if (pti->pWaitingCompletion->name)
			save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name );
		else
			printk("pti->pWaitingCompletion->name == NULL\r\n");
	}

        if(pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL)
        {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
            else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}


        save_log("\r\n");
        show_stack1(pts, NULL);

        save_log("\r\n");


        ptis->pid = pts->pid;
        ptis->pts = pts;
        ptis->sum_exec_runtime = pts->se.sum_exec_runtime;
        //printk("saving %d, sum_exec_runtime  %lld.%06ld  \n\r", ptis->pid, SPLIT_NS(ptis->sum_exec_runtime));
        ptis->vruntime = pts->se.vruntime;
        //printk("newing ptis %x utime=%d, stime=%d\n\r", ptis, pts->utime, pts->stime);

        ptis_ptr->pnext = ptis;
        ptis_ptr = ptis;

        if(!thread_group_empty(pts))
        {
            struct task_struct *p1 = next_thread(pts);
            do
            {
                pti = task_thread_info(p1);
                //printk("for pts %x, ptis_ptr=%x\n\r", pts, ptis_ptr);
                ptis = kmalloc(sizeof( struct thread_info_save), GFP_KERNEL);
                if(!ptis)
                {
                    printk("kmalloc ptis 2 failure\n");
                    return;
                }
                memset(ptis, 0, sizeof( struct thread_info_save));

                ptis->pid = p1->pid;
                ptis->pts = p1;
                ptis->sum_exec_runtime = p1->se.sum_exec_runtime;
                //printk("saving %d, sum_exec_runtime  %lld.%06ld  \n\r", ptis->pid, SPLIT_NS(ptis->sum_exec_runtime));
                ptis->vruntime = p1->se.vruntime;
                //printk("newing ptis %x utime=%d, stime=%d\n\r", ptis, pts->utime, pts->stime);

                ptis_ptr->pnext = ptis;
                ptis_ptr = ptis;
                save_log(" %-7d", p1->pid);

                if(pts->parent)
                    save_log("%-8d", p1->parent->pid);
                else
                    save_log("%-8d", 0);

                save_log("%-20s", p1->comm);
                save_log("%lld.%06ld", SPLIT_NS(p1->se.sum_exec_runtime));
                save_log("     %lld.%06ld     ", SPLIT_NS(p1->se.vruntime));
                save_log("%-5d", p1->static_prio);
                save_log("%-5d", p1->normal_prio);
                save_log("%-15s", print_state((p1->state & TASK_REPORT) | p1->exit_state));
#ifndef ASUS_SHIP_BUILD
//                save_log("call_proc:%-6d  ", pts->binder_call_to_proc_pid);
 //               save_log("call_thread:%-6d  ", pts->binder_call_to_thread_pid);
 //               save_log("call_code:%-6d  ", pts->binder_call_code);
#endif
                save_log("%-6d", pti->preempt_count);

        if(pti->pWaitingMutex != &fake_mutex && pti->pWaitingMutex != NULL)
        {
			if (pti->pWaitingMutex->name)
			{
				save_log("    Mutex:%s,", pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,", pti->pWaitingMutex->name + 1);
			}
			else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug)
			{
				save_log(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)", pti->pWaitingMutex->mutex_owner_asusdebug->pid);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm)
			{
				save_log(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
				printk(" %s",pti->pWaitingMutex->mutex_owner_asusdebug->comm);
			}
			else
				printk("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}

		if(pti->pWaitingCompletion != &fake_completion && pti->pWaitingCompletion!=NULL)
		{
			if (pti->pWaitingCompletion->name)
	            save_log("    Completion:wait_for_completion %s", pti->pWaitingCompletion->name );
	        else
				printk("pti->pWaitingCompletion->name == NULL\r\n");
		}

        if(pti->pWaitingRTMutex != &fake_rtmutex && pti->pWaitingRTMutex != NULL)
        {
			struct task_struct *temp = rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)", temp->pid);
            else
				printk("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}
                save_log("\r\n");
                show_stack1(p1, NULL);

                save_log("\r\n");

                p1 = next_thread(p1);
            }while(p1 != pts);
        }


    }

}

EXPORT_SYMBOL(save_all_thread_info);

void delta_all_thread_info(void)
{
    struct task_struct *pts;
    int ret = 0, ret2 = 0;

    struct rtc_time tm;
    asus_rtc_read_time(&tm);
    #if 1
    g_phonehang_log = (char*)PHONE_HANG_LOG_BUFFER;//phys_to_virt(PHONE_HANG_LOG_BUFFER);
    g_iPtr = 0;
    memset_nc(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);
    #endif

    save_log("ASUSSlowg-%04d%02d%02d-%02d%02d%02d-delta.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ASUS_SW_VER);

    save_log("DELTA INFO----------------------------------------------------------------------------------------------\r\n");
    save_log(" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt----Waiting\r\n");
    for_each_process(pts)
    {
        //printk("for pts %x,\n\r", pts);
        ret = find_thread_info(pts, 0);
        if(!thread_group_empty(pts))
        {
            struct task_struct *p1 = next_thread(pts);
            ret2 = 0;
            do
            {
                ret2 += find_thread_info(p1, 0);
                p1 = next_thread(p1);
            }while(p1 != pts);
            if(ret2 && !ret)
                find_thread_info(pts, 1);
        }
        if(ret || ret2)
            save_log("-----------------------------------------------------\r\n\r\n-----------------------------------------------------\r\n");
    }
    save_log("\r\n\r\n\r\n\r\n");
}
EXPORT_SYMBOL(delta_all_thread_info);
///////////////////////////////////////////////////////////////////////////////////////////////////
void printk_buffer_rebase(void);
static mm_segment_t oldfs;
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}

char messages[256];
void save_phone_hang_log(int delta)
{
    int file_handle;
    int ret;
    //---------------saving phone hang log if any -------------------------------
    g_phonehang_log = (char*)PHONE_HANG_LOG_BUFFER;
    printk("save_phone_hang_log PRINTK_BUFFER=%p, PHONE_HANG_LOG_BUFFER=%p\n", PRINTK_BUFFER_VA, PHONE_HANG_LOG_BUFFER);
    if(g_phonehang_log && ((strncmp(g_phonehang_log, "PhoneHang", 9) == 0) || (strncmp(g_phonehang_log, "ASUSSlowg", 9) == 0)) )
    {
        printk("save_phone_hang_log-1\n");
        initKernelEnv();
        memset(messages, 0, sizeof(messages));
        strcpy(messages, ASUS_ASDF_BASE_DIR);
        if(delta)
        	strncat(messages, g_phonehang_log, 29+6);
        else
        	strncat(messages, g_phonehang_log, 29);
        file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC, 0);
        printk("save_phone_hang_log-2 file_handle %d, name=%s\n", file_handle, messages);
        if(!IS_ERR((const void *)(ulong)file_handle))
        {
            ret = sys_write(file_handle, (unsigned char*)g_phonehang_log, strlen(g_phonehang_log));
            sys_close(file_handle);
        }
        deinitKernelEnv();

    }
    if(g_phonehang_log && file_handle >0 && ret >0)
    {
        g_phonehang_log[0] = 0;
        //iounmap(g_phonehang_log);
    }
}
EXPORT_SYMBOL(save_phone_hang_log);
void save_last_shutdown_log(char* filename)
{
    char *last_shutdown_log;
    int file_handle;
    unsigned long long t;
    unsigned long nanosec_rem;

    t = cpu_clock(0);
	nanosec_rem = do_div(t, 1000000000);
    last_shutdown_log = (char*)PRINTK_BUFFER_VA;
	sprintf(messages, ASUS_ASDF_BASE_DIR"LastShutdown_%lu.%06lu.txt",
				(unsigned long) t,
				nanosec_rem / 1000);

    initKernelEnv();
    file_handle = sys_open(messages, O_CREAT|O_RDWR|O_SYNC, 0);
    if(!IS_ERR((const void *)(ulong)file_handle))
    {
        sys_write(file_handle, (unsigned char*)last_shutdown_log, PRINTK_BUFFER_SLOT_SIZE);
        sys_close(file_handle);
    } else {
		printk("[ASDF] save_last_shutdown_error: [%d]\n", file_handle);
	}
    deinitKernelEnv();

}

#if defined(CONFIG_MSM_RTB)
extern struct msm_rtb_state msm_rtb;

int g_saving_rtb_log = 1;

void save_rtb_log(void)
{
	char *rtb_log;
	char rtb_log_path[256] = {0};
	int file_handle;
	unsigned long long t;
	unsigned long nanosec_rem;

	rtb_log = (char*)msm_rtb.rtb;
	t = cpu_clock(0);
	nanosec_rem = do_div(t, 1000000000);
	snprintf(rtb_log_path, sizeof(rtb_log_path)-1, ASUS_ASDF_BASE_DIR"/rtb_%lu.%06lu.bin",
				(unsigned long) t,
				nanosec_rem / 1000);

	initKernelEnv();
	file_handle = sys_open(rtb_log_path, O_CREAT|O_RDWR|O_SYNC, 0);
	if(!IS_ERR((const void *)(ulong)file_handle))
	{
		sys_write(file_handle, (unsigned char*)rtb_log, msm_rtb.size);
		sys_close(file_handle);
	} else {
		printk("[ASDF] save_rtb_log_error: [%d]\n", file_handle);
	}
	deinitKernelEnv();

}
#endif

void get_last_shutdown_log(void)
{
    ulong *printk_buffer_slot2_addr;

    printk_buffer_slot2_addr = (ulong *)PRINTK_BUFFER_SLOT2;
    printk("get_last_shutdown_log: printk_buffer_slot2=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);
    if(*printk_buffer_slot2_addr == (ulong)PRINTK_BUFFER_MAGIC) {
        save_last_shutdown_log("LastShutdown");
    }
    printk_buffer_rebase();
}
EXPORT_SYMBOL(get_last_shutdown_log);

extern int nSuspendInProgress;
static struct workqueue_struct *ASUSEvtlog_workQueue;
static int g_hfileEvtlog = -MAX_ERRNO;
static int g_bEventlogEnable = 1;
static char g_Asus_Eventlog[ASUS_EVTLOG_MAX_ITEM][ASUS_EVTLOG_STR_MAXLEN];
static int g_Asus_Eventlog_read = 0;
static int g_Asus_Eventlog_write = 0;

static void do_write_event_worker(struct work_struct *work);
static DECLARE_WORK(eventLog_Work, do_write_event_worker);
//add dump_boot_reasons ++++
static void dump_boot_reasons(void)
{
	char buffer[256] = {0};
	unsigned smem_size = 0;
	unsigned char* pmic_boot_reasons = NULL;

	pmic_boot_reasons = (unsigned char *)(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size,false,true));
	if(NULL == pmic_boot_reasons || smem_size < 8)
	{
		printk(KERN_ERR "%s get boot reasons failed.", __func__);
		return;
	}

	snprintf(buffer, 255, "PMIC Boot Reasons:%02X %02X %02X %02X %02X %02X %02X %02X\n",
			 pmic_boot_reasons[0], pmic_boot_reasons[1], pmic_boot_reasons[2], pmic_boot_reasons[3],
			 pmic_boot_reasons[4], pmic_boot_reasons[5], pmic_boot_reasons[6], pmic_boot_reasons[7]);

	printk(KERN_NOTICE "%s", buffer);
	sys_write(g_hfileEvtlog, buffer, strlen(buffer));
}
//add dump_boot_reasons ----
static struct mutex mA;
#define AID_SDCARD_RW 1015
static void do_write_event_worker(struct work_struct *work)
{
	char buffer[256];
	memset(buffer, 0, sizeof(char)*256);

	if (IS_ERR((const void *)(ulong)g_hfileEvtlog)) {
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		sys_chown(ASUS_EVTLOG_PATH".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"_old.txt");
			sys_rename(ASUS_EVTLOG_PATH".txt", ASUS_EVTLOG_PATH"_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		}
		sprintf(buffer, "\n\n---------------System Boot----%s---------\n", ASUS_SW_VER);

		sys_write(g_hfileEvtlog, buffer, strlen(buffer));
		//add dump_boot_reasons ++++
		if(!IS_ERR((const void*)(ulong)g_hfileEvtlog))
		{
			dump_boot_reasons();
		}
		//add dump_boot_reasons ----
		sys_close(g_hfileEvtlog);
	}
	if (!IS_ERR((const void *)(ulong)g_hfileEvtlog)) {
		int str_len;
		char *pchar;
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		sys_chown(ASUS_EVTLOG_PATH".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"_old.txt");
			sys_rename(ASUS_EVTLOG_PATH".txt", ASUS_EVTLOG_PATH"_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		}

		while (g_Asus_Eventlog_read != g_Asus_Eventlog_write) {
			mutex_lock(&mA);
			str_len = strlen(g_Asus_Eventlog[g_Asus_Eventlog_read]);
			pchar = g_Asus_Eventlog[g_Asus_Eventlog_read];
			g_Asus_Eventlog_read++;
			g_Asus_Eventlog_read %= ASUS_EVTLOG_MAX_ITEM;
			mutex_unlock(&mA);

			if (pchar[str_len - 1] != '\n' ) {
				if(str_len + 1 >= ASUS_EVTLOG_STR_MAXLEN)
					str_len = ASUS_EVTLOG_STR_MAXLEN - 2;
				pchar[str_len] = '\n';
				pchar[str_len + 1] = '\0';
			}

			sys_write(g_hfileEvtlog, pchar, strlen(pchar));
			sys_fsync(g_hfileEvtlog);
		}
		sys_close(g_hfileEvtlog);
	}
}

extern struct timezone sys_tz;

void ASUSEvtlog(const char *fmt, ...)
{

	va_list args;
	char *buffer;

	if (g_bEventlogEnable == 0)
		return;
	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_lock(&mA);

	buffer = g_Asus_Eventlog[g_Asus_Eventlog_write];
	g_Asus_Eventlog_write++;
	g_Asus_Eventlog_write %= ASUS_EVTLOG_MAX_ITEM;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_unlock(&mA);

	memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);
	if (buffer) {
		struct rtc_time tm;
		struct timespec ts;

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(ts.tv_sec, &tm);
		getrawmonotonic(&ts);
		sprintf(buffer, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :", ts.tv_sec, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		/*printk(buffer);*/
		va_start(args, fmt);
		vscnprintf(buffer + strlen(buffer), ASUS_EVTLOG_STR_MAXLEN - strlen(buffer), fmt, args);
		va_end(args);
		printk("%s",buffer);
		queue_work(ASUSEvtlog_workQueue, &eventLog_Work);
	} else {
		printk("ASUSEvtlog buffer cannot be allocated\n");
	}
}
EXPORT_SYMBOL(ASUSEvtlog);

static ssize_t evtlogswitch_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if(strncmp(buf, "0", 1) == 0) {
		ASUSEvtlog("ASUSEvtlog disable !!");
		printk("ASUSEvtlog disable !!\n");
		flush_work(&eventLog_Work);
		g_bEventlogEnable = 0;
	}
	if (strncmp(buf, "1", 1) == 0) {
		g_bEventlogEnable = 1;
		ASUSEvtlog("ASUSEvtlog enable !!");
		printk("ASUSEvtlog enable !!\n");
	}

	return count;
}
static ssize_t asusevtlog_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;
	ASUSEvtlog("%s",messages);

	return count;
}

static int asusdebug_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int asusdebug_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t asusdebug_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	return 0;
}

#include <linux/reboot.h>
extern int rtc_ready;
#ifdef ASUS_ZC550KL_PROJECT
int watchdog_test = 0;
#endif
int asus_asdf_set = 0;
static ssize_t asusdebug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	u8 messages[256] = {0};

	if (count > 256)
		count = 256;
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	if (strncmp(messages, "panic", strlen("panic")) == 0) {
		panic("panic test");
#ifdef ASUS_ZC550KL_PROJECT
	} else if (strncmp(messages, "enuart", strlen("enuart")) == 0) {
		g_uart_dbg_mode = 1;
		printk("Kernel  uart dbg mode = %d\n", g_uart_dbg_mode);
	} else if(strncmp(messages, "disuart", strlen("disuart")) == 0) {
		g_uart_dbg_mode = 0;
		printk("Kernel uart dbg mode = %d\n", g_uart_dbg_mode);
#else
	} else if (strncmp(messages, "dbg", strlen("dbg")) == 0) {
		g_user_dbg_mode = 1;
		printk("Kernel dbg mode = %d\n", g_user_dbg_mode);
	} else if(strncmp(messages, "ndbg", strlen("ndbg")) == 0) {
		g_user_dbg_mode = 0;
		printk("Kernel dbg mode = %d\n", g_user_dbg_mode);
#endif
	} else if(strncmp(messages, "get_asdf_log",
				strlen("get_asdf_log")) == 0) {
#ifdef CONFIG_MSM_RTB
		extern int g_saving_rtb_log;
#endif
		ulong *printk_buffer_slot2_addr;

		printk_buffer_slot2_addr = (ulong *)PRINTK_BUFFER_SLOT2;
		printk("[ASDF] printk_buffer_slot2_addr=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);

		if(!asus_asdf_set) {
			asus_asdf_set = 1;
			save_phone_hang_log(1);
			get_last_shutdown_log();
			printk("[ASDF] get_last_shutdown_log: printk_buffer_slot2_addr=%p, value=0x%lx\n", printk_buffer_slot2_addr, *printk_buffer_slot2_addr);
#ifdef CONFIG_MSM_RTB
			if ((*printk_buffer_slot2_addr) == (ulong)PRINTK_BUFFER_MAGIC )
				save_rtb_log();
#endif
			if ((*printk_buffer_slot2_addr) == (ulong)PRINTK_BUFFER_MAGIC ) {
				printk("[ASDF] after saving asdf log, then reboot\n");
				sys_sync();
				kernel_restart(NULL);
			}

			(*printk_buffer_slot2_addr)=(ulong)PRINTK_BUFFER_MAGIC;
		}
#ifdef CONFIG_MSM_RTB
		g_saving_rtb_log = 0;
#endif
	} else if(strncmp(messages, "slowlog", strlen("slowlog")) == 0) {
		printk("start to gi chk\n");
		save_all_thread_info();
		save_phone_hang_log(0);
		msleep(5 * 1000);

		printk("start to gi delta\n");
		delta_all_thread_info();
		save_phone_hang_log(1);
		return count;
#ifdef ASUS_ZC550KL_PROJECT		
	} else if(strncmp(messages, "watchdog_test", 13) == 0)
    {
		printk("start watchdog test...\r\n");
		watchdog_test = 1;
#endif
	}    

	return count;
}

static const struct file_operations proc_evtlogswitch_operations = {
	.write	  = evtlogswitch_write,
};
static const struct file_operations proc_asusevtlog_operations = {
	.write	  = asusevtlog_write,
};
static const struct file_operations proc_asusdebug_operations = {
	.read	   = asusdebug_read,
	.write	  = asusdebug_write,
	.open	   = asusdebug_open,
	.release	= asusdebug_release,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void asusdebug_early_suspend(struct early_suspend *h)
{
    entering_suspend = 1;
}

static void asusdebug_early_resume(struct early_suspend *h)
{
    entering_suspend = 0;
}
EXPORT_SYMBOL(entering_suspend);

struct early_suspend asusdebug_early_suspend_handler = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = asusdebug_early_suspend,
    .resume = asusdebug_early_resume,
};
#endif

unsigned int asusdebug_enable = 0;
unsigned int readflag = 0;
static ssize_t turnon_asusdebug_proc_read(struct file *filp, char __user *buff, size_t len, loff_t *off)
{
	char print_buf[32];
	unsigned int ret = 0,iret = 0;
	sprintf(print_buf, "asusdebug: %s\n", asusdebug_enable? "off":"on");
	ret = strlen(print_buf);
	iret = copy_to_user(buff, print_buf, ret);
	if (!readflag) {
		readflag = 1;
		return ret;
	}
	else {
		readflag = 0;
		return 0;
	}
}
static ssize_t turnon_asusdebug_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	if (strncmp(messages, "off", 3) == 0) {
		asusdebug_enable = 0x11223344;
	} else if(strncmp(messages, "on", 2) == 0) {
		asusdebug_enable = 0;
	}
	return len;
}
static struct file_operations turnon_asusdebug_proc_ops = {
	.read = turnon_asusdebug_proc_read,
	.write = turnon_asusdebug_proc_write,
 };
static int gKmsgconfig = 0;
static int Kmsgconfig_proc_show(struct seq_file *m, void *v)
{
       seq_printf(m, "%d", gKmsgconfig);
       return 0;
}

static int Kmsgconfig_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, Kmsgconfig_proc_show, NULL);
}

static ssize_t Kmsgconfig_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
       char num[10];
       memset(num, 0, sizeof(num));

       /* no data be written */
       if (!count) {
	       return 0;
	}

       /* Input size is too large to write our buffer(num) */
       if (count > (sizeof(num) - 1)) {
       return -EINVAL;
       }

       if (copy_from_user(num, buf, count)) {
       return -EFAULT;
	}

       if (strncmp(num, "0", 1) == 0) {
       		gKmsgconfig = 0;
       } else if (strncmp(num, "1", 1) == 0) {
		gKmsgconfig = 1;
	} else {
		printk("gKmsgconfig unknown data!!\n");
	}

       return count;
}

int get_Kmsgconfig(void)
{
       return gKmsgconfig;
}
EXPORT_SYMBOL(get_Kmsgconfig);

static const struct file_operations proc_kmsgconfig_operations = {
       .open = Kmsgconfig_proc_open,
       .read = seq_read,
       .write = Kmsgconfig_proc_write,
};
#ifdef ASUS_ZC550KL_PROJECT
///////////////////////////////////////////////////////////////////////
//
// printk controller
//
///////////////////////////////////////////////////////////////////////
static int klog_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_user_klog_mode);
	return 0;
}

static int klog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, klog_proc_show, NULL);
}


static ssize_t klog_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *pos)
{
	char lbuf[32];

	if (count >= sizeof(lbuf))
		count = sizeof(lbuf)-1;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;
	lbuf[count] = 0;

	if(0 == strncmp(lbuf, "1", 1))
	{
		g_user_klog_mode = 1;
	}
	else
	{
		g_user_klog_mode = 0;
	}

	return count;
}

static const struct file_operations klog_proc_fops = {
	.open		= klog_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= klog_proc_write,
};
#endif
static int __init proc_asusdebug_init(void)
{
	proc_create("asusdebug", S_IALLUGO, NULL, &proc_asusdebug_operations);
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	proc_create("asusevtlog-switch", S_IRWXUGO, NULL, &proc_evtlogswitch_operations);
	proc_create("asusdebug-switch", S_IRWXUGO, NULL, &turnon_asusdebug_proc_ops);
	 proc_create("kmsgconfig", S_IRUGO | S_IWUSR, NULL,&proc_kmsgconfig_operations);
	//+++ [ZC550KL] ASUS_BSP suri_gu@asus.com for screen off in factory mode when sys.screentimeout=1
	proc_create_data("driver/screenofftimeout", 0666, NULL, &asus_screenofftimeout_switch_proc_fops, NULL);
	//---
#ifdef ASUS_ZC550KL_PROJECT
     proc_create_data("asusklog", S_IRWXUGO, NULL, &klog_proc_fops, NULL);
#endif
	PRINTK_BUFFER_VA = ioremap(PRINTK_BUFFER_PA, PRINTK_BUFFER_SIZE);
	mutex_init(&mA);
	fake_mutex.owner = current;
	fake_mutex.mutex_owner_asusdebug = current;
	fake_mutex.name = " fake_mutex";
	strcpy(fake_completion.name," fake_completion");
	fake_rtmutex.owner = current;
	ASUSEvtlog_workQueue  = create_singlethread_workqueue("ASUSEVTLOG_WORKQUEUE");

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&asusdebug_early_suspend_handler);
#endif
	//+++ [ZC550KL] ASUS_BSP suri_gu@asus.com for screen off in factory mode when sys.screentimeout=1
	AsusscreenofftimeoutSwitchInitialize();
	//---
	return 0;
}
module_init(proc_asusdebug_init);

#define ASUS_DEBUG_PERSIST_DATA_PARTITION  "/dev/block/platform/7824900.sdhci/by-name/aboot"
#define ASUS_DEBUG_PERSIST_DATA_POS  		(512 * 3)
#define ASUS_DEBUG_PERSIST_DATA_MAGIC1		0x58A52487
#define ASUS_DEBUG_PERSIST_DATA_MAGIC2		0xE8D6A895

struct ASUS_DEBUG_PERSIST_DATA
{
	uint32_t magic1;
	uint32_t magic2;
	uint32_t size;
	uint32_t checksum;
	uint32_t modem_debug_count;
	uint32_t rpm_turbo_require_count;
	uint32_t rpm_turbo_require_reset_time;
	uint32_t rpm_turbo_target_count;
};

static uint32_t get_asus_persist_data_checksum(uint32_t* data, size_t size)
{
	size_t i;
	uint32_t checksum = 0;

	size /= sizeof(uint32_t);
	for(i = 0; i < size; i++)
	{
		checksum += data[i];
	}

	return ~checksum;
}

static int get_asus_persist_data(struct ASUS_DEBUG_PERSIST_DATA* pout)
{
	int ret = -1;
	mm_segment_t old_fs;
	struct file* nodefile = NULL;
	loff_t pos;
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	ssize_t ioBytes;
	uint32_t oldChecksum;

	old_fs = get_fs(); // Save the current FS segment
	set_fs(get_ds());

	do
	{
		nodefile = filp_open(ASUS_DEBUG_PERSIST_DATA_PARTITION, O_RDONLY, 0);
		if(IS_ERR(nodefile))
		{
			printk("Cann't open device node %s\n", ASUS_DEBUG_PERSIST_DATA_PARTITION);
			break;
		}

		pos = vfs_llseek(nodefile, -ASUS_DEBUG_PERSIST_DATA_POS, SEEK_END);
		ioBytes = vfs_read(nodefile, (char*)&dbgData, sizeof(struct ASUS_DEBUG_PERSIST_DATA), &pos);
		if(ioBytes < sizeof(struct ASUS_DEBUG_PERSIST_DATA))
		{
			printk("Read asus debug persist data failed.\n");
			break;
		}

		if(dbgData.magic1 != ASUS_DEBUG_PERSIST_DATA_MAGIC1 || dbgData.magic2 != ASUS_DEBUG_PERSIST_DATA_MAGIC2)
		{
			printk("Invalid asus persist debug data magic number.\n");
			break;
		}

		oldChecksum = dbgData.checksum;
		dbgData.checksum = 0;
		if(oldChecksum != get_asus_persist_data_checksum((uint32_t*)&dbgData, sizeof(dbgData)))
		{
			printk("Invalid asus persist debug data checksum.\n");
			break;
		}

		dbgData.checksum = oldChecksum;
		memcpy(pout, &dbgData, sizeof(struct ASUS_DEBUG_PERSIST_DATA));

		ret = 0;
	} while(0);

	if(!IS_ERR(nodefile))
	{
		filp_close(nodefile, NULL);
	}

	set_fs(old_fs);

	return ret;
}

static int set_asus_persist_data(const struct ASUS_DEBUG_PERSIST_DATA* pin)
{
	int ret = -1;
	mm_segment_t old_fs;
	struct file* nodefile = NULL;
	loff_t pos;
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	ssize_t ioBytes;

	old_fs = get_fs(); // Save the current FS segment
	set_fs(get_ds());

	do
	{
		nodefile = filp_open(ASUS_DEBUG_PERSIST_DATA_PARTITION, O_WRONLY | O_DSYNC, 0);
		if(IS_ERR(nodefile))
		{
			printk("Cann't open device node %s\n", ASUS_DEBUG_PERSIST_DATA_PARTITION);
			break;
		}

		memcpy(&dbgData, pin, sizeof(struct ASUS_DEBUG_PERSIST_DATA));

		dbgData.magic1 = ASUS_DEBUG_PERSIST_DATA_MAGIC1;
		dbgData.magic2 = ASUS_DEBUG_PERSIST_DATA_MAGIC2;
		dbgData.size = sizeof(struct ASUS_DEBUG_PERSIST_DATA);
		dbgData.checksum = 0;
		dbgData.checksum = get_asus_persist_data_checksum((uint32_t*)&dbgData, sizeof(dbgData));

		pos = vfs_llseek(nodefile, -ASUS_DEBUG_PERSIST_DATA_POS, SEEK_END);
		ioBytes = vfs_write(nodefile, (const char*)&dbgData, sizeof(struct ASUS_DEBUG_PERSIST_DATA), &pos);
		if(ioBytes < sizeof(struct ASUS_DEBUG_PERSIST_DATA))
		{
			printk("Write asus debug persist data failed.\n");
			break;
		}

		//nodefile->f_op->flush(nodefile);

		ret = 0;
	} while(0);

	if(!IS_ERR(nodefile))
	{
		filp_close(nodefile, NULL);
	}

	set_fs(old_fs);

	return ret;
}

uint32_t get_modem_debug_value(void)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("get_modem_debug_value: get asus persist data failed.\n");
		return 0;
	}
	else
	{
		return dbgData.modem_debug_count;
	}
}

void set_modem_debug_value(uint32_t val)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_modem_debug_value: get old asus persist data failed, reset it.\n");
		memset(&dbgData, 0, sizeof(struct ASUS_DEBUG_PERSIST_DATA));
	}

	dbgData.modem_debug_count = val;
	if(set_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_modem_debug_value: set new asus persist data failed.\n");
		return;
	}
}

uint32_t get_rpm_turbo_require_count(void)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("get_rpm_turbo_require_count: get asus persist data failed.\n");
		return 0;
	}
	else
	{
		return dbgData.rpm_turbo_require_count;
	}
}

void set_rpm_turbo_require_count(uint32_t val)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_require_count: get old asus persist data failed, reset it.\n");
		memset(&dbgData, 0, sizeof(struct ASUS_DEBUG_PERSIST_DATA));
	}

	dbgData.rpm_turbo_require_count = val;
	if(set_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_require_count: set new asus persist data failed.\n");
		return;
	}
}

uint32_t get_rpm_turbo_require_reset_time(void)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("get_rpm_turbo_require_reset_time: get asus persist data failed.\n");
		return 0;
	}
	else
	{
		return dbgData.rpm_turbo_require_reset_time;
	}
}

void set_rpm_turbo_require_reset_time(uint32_t val)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_require_reset_time: get old asus persist data failed, reset it.\n");
		memset(&dbgData, 0, sizeof(struct ASUS_DEBUG_PERSIST_DATA));
	}

	dbgData.rpm_turbo_require_reset_time = val;
	if(set_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_require_reset_time: set new asus persist data failed.\n");
		return;
	}
}

uint32_t get_rpm_turbo_target_count(void)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("get_rpm_turbo_target_count: get asus persist data failed.\n");
		return 0;
	}
	else
	{
		return dbgData.rpm_turbo_target_count;
	}
}

void set_rpm_turbo_target_count(uint32_t val)
{
	struct ASUS_DEBUG_PERSIST_DATA dbgData;
	if(get_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_target_count: get old asus persist data failed, reset it.\n");
		memset(&dbgData, 0, sizeof(struct ASUS_DEBUG_PERSIST_DATA));
	}

	dbgData.rpm_turbo_target_count = val;
	if(set_asus_persist_data(&dbgData) < 0 )
	{
		printk("set_rpm_turbo_target_count: set new asus persist data failed.\n");
		return;
	}
}

