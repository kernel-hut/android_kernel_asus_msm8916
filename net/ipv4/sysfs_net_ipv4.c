/*
 * net/ipv4/sysfs_net_ipv4.c
 *
 * sysfs-based networking knobs (so we can, unlike with sysctl, control perms)
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <net/tcp.h>

#define CREATE_IPV4_FILE(_name, _var) \
static ssize_t _name##_show(struct kobject *kobj, \
			    struct kobj_attribute *attr, char *buf) \
{ \
	return sprintf(buf, "%d\n", _var); \
} \
static ssize_t _name##_store(struct kobject *kobj, \
			     struct kobj_attribute *attr, \
			     const char *buf, size_t count) \
{ \
	int val, ret; \
	ret = sscanf(buf, "%d", &val); \
	if (ret != 1) \
		return -EINVAL; \
	if (val < 0) \
		return -EINVAL; \
	_var = val; \
	return count; \
} \
static struct kobj_attribute _name##_attr = \
	__ATTR(_name, 0644, _name##_show, _name##_store)

//ASUS_BSP+++ SYN FIREWALL
struct port_link
{
	int port;
	int read;
	int deleting;
	struct port_link* next;
}; 
DECLARE_COMPLETION(listen_event);
extern struct port_link syn_firewall_port_link_head;
extern spinlock_t listen_port_lock;
int lp_modem_restart=0;//ASUS_BSP Johnny +++ return listen_port when modem restart
//ASUS_BSP+++ turn off firewall when tethering on and turn on firewall when tethering off
extern int if_ip_forward_on;
extern int if_ip_forward_off;
//ASUS_BSP---
static ssize_t listen_port_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) 
{ 
	unsigned long r=0;
	int port_num=0;
	struct port_link *p=&syn_firewall_port_link_head;
	printk("[SYN] listen_port_show\n");

	r=wait_for_completion_interruptible(&listen_event);
	if(0==r){
		printk("[SYN]listen port event complete 0\n");
    }
	else{
		printk("[SYN] should be suspending\n");
	    return sprintf(buf, "3"); 
	}
	spin_lock_bh(&listen_port_lock);
	
	//ASUS_BSP Johnny +++ return listen_port when modem restart
        if(lp_modem_restart==1)
        {

            printk("[SYN]modem restart return listen port\n");
            complete(&listen_event);
            lp_modem_restart=0;
            spin_unlock_bh(&listen_port_lock);

            return sprintf(buf,"4 1");
        }
        //ASUS_BSP ---

        //ASUS_BSP+++ turn off firewall when tethering on and turn on firewall when tethering off
        if(if_ip_forward_on==1)
        {
                if_ip_forward_on=0;
                spin_unlock_bh(&listen_port_lock);
                printk("[SYN] Tethering on stop SYN\n");
                return sprintf(buf, "13 0");
        }
        if(if_ip_forward_off==1)
        {
                if_ip_forward_off=0;
                spin_unlock_bh(&listen_port_lock);
                printk("[SYN] Tethering off turn on SYN\n");
                return sprintf(buf, "14 0");

        }
        //ASUS_BSP---

	while(p->next!=NULL)
	{
		struct port_link *prev;
		prev=p;
		p=p->next;
		if(0==p->read){
			p->read=1;
			if(p->next!=NULL)
				complete(&listen_event);
			spin_unlock_bh(&listen_port_lock);
			
			printk("[SYN] add port %d\n", p->port);

			return sprintf(buf, "9 %d",p->port);
		}
	}
	p=&syn_firewall_port_link_head;
	while(p->next!=NULL)
	{
		struct port_link *prev,*next;
		prev=p;
		p=p->next;
		next=p->next;
		if(1==p->deleting){
			p->read=1;
			if(p->next!=NULL)
				complete(&listen_event);
			prev->next=next;
			port_num=p->port;
			spin_unlock_bh(&listen_port_lock);
			kfree(p);
			printk("[SYN] delete port %d %p %p %p\n",port_num,prev,p,next);
			return sprintf(buf, "10 %d",port_num);
		}
	}
	spin_unlock_bh(&listen_port_lock);
			

	return sprintf(buf, "3"); 
}

static ssize_t listen_port_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) 
{ 
	printk("[SYN] listen_port_store\n");
	return count; 
} 


static struct kobj_attribute listen_port_attr = 
	__ATTR(listen_port, 0664, listen_port_show, listen_port_store);
//ASUS_BSP---
CREATE_IPV4_FILE(tcp_wmem_min, sysctl_tcp_wmem[0]);
CREATE_IPV4_FILE(tcp_wmem_def, sysctl_tcp_wmem[1]);
CREATE_IPV4_FILE(tcp_wmem_max, sysctl_tcp_wmem[2]);

CREATE_IPV4_FILE(tcp_rmem_min, sysctl_tcp_rmem[0]);
CREATE_IPV4_FILE(tcp_rmem_def, sysctl_tcp_rmem[1]);
CREATE_IPV4_FILE(tcp_rmem_max, sysctl_tcp_rmem[2]);

CREATE_IPV4_FILE(tcp_delack_seg, sysctl_tcp_delack_seg);
CREATE_IPV4_FILE(tcp_use_userconfig, sysctl_tcp_use_userconfig);

static struct attribute *ipv4_attrs[] = {
	&tcp_wmem_min_attr.attr,
	&tcp_wmem_def_attr.attr,
	&tcp_wmem_max_attr.attr,
	&tcp_rmem_min_attr.attr,
	&tcp_rmem_def_attr.attr,
	&tcp_rmem_max_attr.attr,
	&tcp_delack_seg_attr.attr,
	&tcp_use_userconfig_attr.attr,
   &listen_port_attr.attr, //ASUS_BSP+ SYN FIREWALL
	NULL
};

static struct attribute_group ipv4_attr_group = {
	.attrs = ipv4_attrs,
};

static __init int sysfs_ipv4_init(void)
{
	struct kobject *ipv4_kobject;
	int ret;

	ipv4_kobject = kobject_create_and_add("ipv4", kernel_kobj);
	if (!ipv4_kobject)
		return -ENOMEM;

	ret = sysfs_create_group(ipv4_kobject, &ipv4_attr_group);
	if (ret) {
		kobject_put(ipv4_kobject);
		return ret;
	}

	return 0;
}

subsys_initcall(sysfs_ipv4_init);
