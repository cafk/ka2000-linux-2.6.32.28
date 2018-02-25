
#include <linux/autoconf.h>

#ifdef CONFIG_ARCH_KA2000
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>

#define request_firmware ka_request_firmware
static int
ka_request_firmware(const struct firmware **firmware_p, const char *name,
		 struct device *device)
{
	struct firmware *firmware = NULL;
	int retval;
	struct file *fp = NULL;
	mm_segment_t fs;
	struct inode *inode;
	int fsize;
	char path[256];
	
	if (!firmware_p)
		return -EINVAL;

	sprintf(path, "/lib/firmware/%s", name);
	fp = filp_open(path, O_RDONLY, 0);
	if (fp)
	{		
		inode = fp->f_dentry->d_inode;
		fsize = inode->i_size;
		printk("Load %s\n", path);

		*firmware_p = firmware = kzalloc(sizeof(*firmware) + fsize + 256, GFP_KERNEL);
		if (!firmware) 
		{
			dev_err(device, "%s: kmalloc(struct firmware) failed\n",
				__func__);
			retval = -ENOMEM;
			goto out;
		}
		else
		{
			fs = get_fs();
	        set_fs(KERNEL_DS);	 
			firmware->data = firmware + sizeof(*firmware);
			firmware->size = 0;
			
			if (fp->f_op && fp->f_op->read)
			{
				firmware->size = fp->f_op->read(fp, firmware->data, fsize, &fp->f_pos); 		
				printk("firmware read size %d, pos %d\n", firmware->size, fp->f_pos);
			}
			 
			set_fs(fs);
			filp_close(fp,NULL); 
			return 0;
		}
	}	 
	else
		return -EINVAL;
	
	goto out;

error_kfree_fw:
	if (firmware)
	{
		kfree(firmware);
		*firmware_p = NULL;
	}
	if (fp)
	{
		filp_close(fp, NULL); 
	}
out:
	return retval;
}
#endif
