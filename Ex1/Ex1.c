#include <linux/module.h> // kernal header for macro like module_init, module_exit
#include <linux/fs.h> // library for allocating and deallocating a device number
#include <linux/device.h> // library for creating a device file.
#include <linux/slab.h> // library for kmalloc and kfree
#include <linux/cdev.h> // library to work with cdev
#include <linux/uaccess.h> // library to exchange data between user space and kernel space
#include "vchar_driver.h" 


#define DRIVER_AUTHOR "ThinhP.Tran trphthinh@gmail.com"
#define DRIVER_VERSION "1.0"

typedef struct vchar_dev {
  unsigned char * control_regs; 
  unsigned char * status_regs;
  unsigned char * data_regs; 
} vchar_dev_t; 

struct _cdrv {
dev_t dev_num; 
struct class * dev_class; 
struct device * dev; 
vchar_dev_t * vchar_hw; 
struct cdev * vcdev; 
unsigned int open_cnt; 
} cdrv; 

static int vchar_driver_open(struct inode * inode, struct file * filp) {
  cdrv.open_cnt++; 
  printk("Handle opened event (%d)\n", cdrv.open_cnt); 
  return 0; 
}

static int vchar_driver_release(struct inode * inode, struct file * filp) {
  printk("Handle closed event\n"); 
  return 0; 
}

int vchar_hw_read_data(vchar_dev_t *hw, int start_reg, int num_regs, char*kbuf)
{
  int read_bytes = num_regs;

  // Check whether we have access right 
  if ((hw->control_regs[CONTROL_ACCESS_REG] & CTRL_READ_DATA_BIT ) == DISABLE)
    return -1; 

  // Check if the address of kernel buffer is legal.
  if (kbuf == NULL) 
    return -1; 

  // Check if the position of register is approriate
  if (start_reg > NUM_DEV_REGS) {
    return -1; 
  }

  // Change num_regs to an appropriate number
  if (num_regs > (NUM_DATA_REGS - start_reg))
    read_bytes = NUM_DATA_REGS - start_reg; 

  memcpy(kbuf, hw->data_regs + start_reg, read_bytes); 

  hw->status_regs[READ_COUNT_L_REG] += 1; 
  if (hw->status_regs[READ_COUNT_L_REG] == 0)
    hw->status_regs[READ_COUNT_H_REG] += 1; 

  return read_bytes;
}

static ssize_t vchar_driver_read(struct file *filp, char __user *user_buf, size_t len, loff_t *off)
{
  char * kernel_buf = NULL; 
  int num_bytes = 0; 

  printk("Handle read event start from %lld, %zu bytest\n", *off, len); 
  kernel_buf = kzalloc(len, GFP_KERNEL); 

  if (kernel_buf == NULL)
    return 0; 

  num_bytes = vchar_hw_read_data(cdrv.vchar_hw, *off, len, kernel_buf); 
  printk("read %d bytes from HW\n", num_bytes); 

  if (num_bytes < 0)
    return -EFAULT; 

  if (copy_to_user(user_buf, kernel_buf, num_bytes)) 
    return -EFAULT; 

  *off += num_bytes; 

  return num_bytes;
}


int vchar_hw_write_data(vchar_dev_t *hw, int start_reg, int num_regs, char* kbuf)
{
  int write_bytes = num_regs; 

  if ((hw->control_regs[CONTROL_ACCESS_REG] & CTRL_WRITE_DATA_BIT) == DISABLE)
    return -1; 

  if (kbuf == NULL)
    return -1; 

  if (start_reg > NUM_DATA_REGS)
    return -1; 

  if (num_regs > (NUM_DATA_REGS - start_reg)) {
    write_bytes = NUM_DATA_REGS - start_reg; 
    hw->status_regs[DEVICE_STATUS_REG] |= STS_DATAREGS_OVERFLOW_BIT; 
  }

  memcpy(hw->data_regs + start_reg, kbuf, write_bytes); 

  hw->status_regs[WRITE_COUNT_L_REG] += 1; 
  if (hw->status_regs[WRITE_COUNT_L_REG] == 0)
    hw->status_regs[WRITE_COUNT_H_REG] += 1; 

  return write_bytes; 
}

static ssize_t vchar_driver_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
  char * kernel_buf = NULL; 
  int num_bytes = 0; 
  printk("Handle write event start from %lld, %zu bytes\n", *off, len); 

  kernel_buf = kzalloc(len, GFP_KERNEL);

  if (copy_from_user(kernel_buf, user_buf, len))
    return -EFAULT; 

  num_bytes = vchar_hw_write_data(cdrv.vchar_hw, *off, len, kernel_buf); 
  printk("writes %d bytes to HW\n", num_bytes); 

  if (num_bytes < 0)
    return -EFAULT; 

  *off += num_bytes; 
  return num_bytes; 
}

static struct file_operations fops = 
{
	.owner   = THIS_MODULE,
	.open    = vchar_driver_open,
	.release = vchar_driver_release,
	.read    = vchar_driver_read, 
	.write   = vchar_driver_write,
}; 

int vchar_hw_init(vchar_dev_t * hw)
{
  char * buf; 
  buf = kzalloc(NUM_DEV_REGS * REG_SIZE, GFP_KERNEL); 

  if (!buf) {
    return -ENOMEM; 
  }

  hw->control_regs = buf; 
  hw->status_regs = hw->control_regs + NUM_CTRL_REGS; 
  hw->data_regs = hw->status_regs + NUM_STS_REGS; 

  // Initialize for registers
  hw->control_regs[CONTROL_ACCESS_REG] = 0x03; 
  hw->status_regs[DEVICE_STATUS_REG] = 0x03;

  return 0; 
}

void vchar_hw_exit(vchar_dev_t * hw)
{
  kfree(hw->control_regs); 
}

static int __init my_init(void)
{
  int ret = 0; 
   
  printk("Hello my first driver"); 

  // grant a major device number. This is a way to statically set device number (not reccomend to use)
  cdrv.dev_num = MKDEV(468, 0); 

  // Here we dynamically allocate device number. This way is more flexible so it can run on other linux computer without running into device number conflictions.
  ret = alloc_chrdev_region(&cdrv.dev_num, 0, 1, "vchar_device"); 

  if (ret < 0) {
    printk("failed to register device number dynamically\n"); 
    return ret; 
  } else {
    printk("Initialize vchar driver successfully, the first number is for device driver, the second one is the the device in case there are more than one devices handled by one driver\n"); 
    printk("Allocated device number (%d, %d)\n", MAJOR(cdrv.dev_num), MINOR(cdrv.dev_num)); 

    // create a device file
    printk("Creating device class!!!"); 
    cdrv.dev_class = class_create(THIS_MODULE, "class_vchar_dev"); 
    if (cdrv.dev_class == NULL) {
      printk("failed to create a device class\n"); 
      unregister_chrdev_region(cdrv.dev_num, 1); 
      return -1; 
    } else {
      printk("Creating device using device a device number and a device class!!!"); 
      cdrv.dev = device_create(cdrv.dev_class, NULL, cdrv.dev_num, NULL, "vchar_dev"); 

      if (IS_ERR(cdrv.dev)) {
        printk("failed to create a device\n"); 
	class_destroy(cdrv.dev_class); 
	unregister_chrdev_region(cdrv.dev_num, 1); 
      } else {
	printk("allocate memory for driver's data structure!!!"); 
        cdrv.vchar_hw = kzalloc(sizeof(vchar_dev_t), GFP_KERNEL); 
	if (!cdrv.vchar_hw) {
          printk("failed to allocate data structure of the driver\n"); 
	  ret = -ENOMEM; 

          device_destroy(cdrv.dev_class, cdrv.dev_num); 
          class_destroy(cdrv.dev_class); 
          unregister_chrdev_region(cdrv.dev_num, 1); 

	  return ret; 
	} else {
          // initialize physical device
	  ret = vchar_hw_init(cdrv.vchar_hw); 
	  if (ret < 0) {
            printk("failed to initialize a virtual character device\n");
	    kfree(cdrv.vchar_hw);    
            device_destroy(cdrv.dev_class, cdrv.dev_num); 
            class_destroy(cdrv.dev_class); 
            unregister_chrdev_region(cdrv.dev_num, 1);
	    return ret;  
	  } else {
            printk("Initialize vchar driver successfully\n");

	    // register entry points to kernel
	    cdrv.vcdev = cdev_alloc(); 
	    if (cdrv.vcdev == NULL) {
              printk("failed to allocate cdev structure\n");
              vchar_hw_exit(cdrv.vchar_hw); 
	      kfree(cdrv.vchar_hw);    
              device_destroy(cdrv.dev_class, cdrv.dev_num); 
              class_destroy(cdrv.dev_class); 
              unregister_chrdev_region(cdrv.dev_num, 1);
	      return -1; 
	    } else {
              printk("allocate cdev structure successfully\n");
	      printk("Registering entry point to linux kernel modules!!!");
	      cdev_init(cdrv.vcdev, &fops); 
              ret = cdev_add(cdrv.vcdev, cdrv.dev_num, 1); 
	      if (ret < 0) {
                printk("failed to add a char device to the system\n");
                vchar_hw_exit(cdrv.vchar_hw); 
	        kfree(cdrv.vchar_hw);    
                device_destroy(cdrv.dev_class, cdrv.dev_num); 
                class_destroy(cdrv.dev_class); 
                unregister_chrdev_region(cdrv.dev_num, 1);
	        return ret; 	
	      } else {
                printk("Register entry points to kernel successfully\n"); 

	      }
	    }


	  }
	}
      }

    }

     
  }

  return 0; 
}


static void __exit my_exit(void)
{
  // Remove entry points from kernel
  cdev_del(cdrv.vcdev); 
	  
  // Release physical device	
  vchar_hw_exit(cdrv.vchar_hw);
  
  // Release memory for physical device struct 
  kfree(cdrv.vchar_hw); 

  // Destroy device
  device_destroy(cdrv.dev_class, cdrv.dev_num); 

  // Destroy class
  class_destroy(cdrv.dev_class); 

  // 
  unregister_chrdev_region(cdrv.dev_num, 1); 
  printk("Goodbye my first driver"); 
}

module_init(my_init); 
module_exit(my_exit); 

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR(DRIVER_AUTHOR); 
MODULE_DESCRIPTION("just a toy to know"); 
MODULE_SUPPORTED_DEVICE("a test device"); 

