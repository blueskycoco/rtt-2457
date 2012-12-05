/*
 * File      : rtdef.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-21     prife        the first version
 */

#include <rtdevice.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sst39vfxx_mtd.h"
#include <s3c44b0.h>
#ifdef RT_USING_MTD_NOR
#define ROM_BASE 0x00000000
#define MC_ID_SST39VF016               (0xBF)
#define MC_ID_SST39VF032               (0x4A)
#define MC_ID_SST39VF064               (0x4B)

#define SST39_MTD(device) 		((struct sst39_mtd*)(device))
struct sst39_mtd
{
    struct rt_mtd_nor_device parent;
    FILE * file;
};
static struct sst39_mtd _sst39_mtd;

static struct rt_mutex flash_lock;
#define	CHECK_DELAY	150000

#define inportw(r) 		(*(volatile rt_uint16_t *)(r))
#define outportw(r, d) 	(*(volatile rt_uint16_t *)(d) = r)
#define toogle_addr(r)	((r))
static void SWPIDExit(void)
{
    outportw(0x00aa, ROM_BASE+0xaaaa);
    outportw(0x0055, ROM_BASE+0x5554);
    outportw(0x00f0, ROM_BASE+0xaaaa);
}
static void SWPIDEntry(void)
{
    outportw(0x00aa, ROM_BASE+0xaaaa);
    outportw(0x0055, ROM_BASE+0x5554);
    outportw(0x0090, ROM_BASE+0xaaaa);
}
/* RT-Thread MTD device interface */
static rt_uint8_t check_toggle_ready(rt_uint32_t dst)
{
	rt_int16_t PreData,CurrData;
	rt_uint32_t TimeOut=0;

	PreData = inportw(dst);
	PreData = PreData & 0x0040;
	while(TimeOut < CHECK_DELAY)
	{
		CurrData = inportw(dst);
		CurrData = CurrData & 0x0040;
		if(CurrData == PreData)
		{
			return RT_EOK;
		}
		else
		{
			PreData = CurrData;
			TimeOut++;
		}
	}
	return -1;
}
static rt_uint32_t sst39vfxx_read_id(struct rt_mtd_nor_device* device)
{
    rt_uint32_t i;

    SWPIDEntry();
    i  = inportw(ROM_BASE);
    i |= inportw(ROM_BASE+2)<<16;
    SWPIDExit();
    return i;	
}
static rt_uint8_t SectorErase(rt_uint32_t sector)
{
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x8080, ROM_BASE+0xaaaa);
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x3030, ROM_BASE+sector);	
	return check_toggle_ready(ROM_BASE+sector);
}
static rt_uint8_t BlockErase(rt_uint32_t block)
{
	outportw(0x00aa, ROM_BASE+0xaaaa);
	outportw(0x0055, ROM_BASE+0x5554);
	outportw(0x0080, ROM_BASE+0xaaaa);
	outportw(0x00aa, ROM_BASE+0xaaaa);
	outportw(0x0055, ROM_BASE+0x5554);
	outportw(0x0050, ROM_BASE+block);	
	return check_toggle_ready(ROM_BASE+block);
}

static int FlashProg(rt_uint32_t ProgStart, rt_uint16_t *DataPtr, rt_uint32_t WordCnt)
{	

	for( ; WordCnt; ProgStart+=2, DataPtr++, WordCnt--) {
		outportw(0xaaaa, ROM_BASE+0xaaaa);
		outportw(0x5555, ROM_BASE+0x5554);
		outportw(0xa0a0, ROM_BASE+0xaaaa);
		outportw(*DataPtr, ROM_BASE+ProgStart);

		if(check_toggle_ready(ROM_BASE+ProgStart)!=RT_EOK)
		{
			return -1;
		}

	}
	return 0;
}
static int sst39vfxx_read(struct rt_mtd_nor_device* device, rt_off_t position, rt_uint8_t *data, rt_size_t size)
{
	struct sst39_mtd *sst39;
	int result;

	sst39 = SST39_MTD(device);
	RT_ASSERT(sst39 != RT_NULL);
	rt_mutex_take(&flash_lock, RT_WAITING_FOREVER);
	rt_memcpy(data, (rt_uint8_t *)(toogle_addr(position)), size);
	rt_mutex_release(&flash_lock);
	return size;
}

static int sst39vfxx_write(struct rt_mtd_nor_device* device, rt_off_t position,
        const rt_uint8_t *data, rt_size_t size)
{
	struct sst39_mtd *sst39;
	int result;
	rt_uint8_t *buf=rt_malloc(0x1000);
	rt_uint32_t  tmp = 0x1000-(position&0xfff);
	rt_uint16_t err;
	rt_uint32_t bak_size=size;
	sst39 = SST39_MTD(device);
	RT_ASSERT(sst39 != RT_NULL);
	rt_mutex_take(&flash_lock, RT_WAITING_FOREVER);
	rt_hw_interrupt_mask(INT_GLOBAL);
	if(tmp>size)
		tmp = size;
	if(tmp&1)
		tmp++;
	for(; size;) 
	{
		if(tmp<0x1000)
		{
			rt_memcpy(buf, (rt_uint8_t *)(toogle_addr(position&~0xfff)), 0x1000);
			rt_memcpy(buf+(position&0xfff), (rt_uint8_t *)data, tmp);
		} 
		else 
		{
			rt_memcpy(buf, (char *)data, 0x1000);
		}
		err = SectorErase(toogle_addr(position&~0xfff));
		if(err) 
		{	//4K Bytes boudary

			rt_hw_interrupt_umask(INT_GLOBAL);
			rt_mutex_release(&flash_lock);
			rt_free(buf);
			rt_kprintf("write nor failed\n");
			return -1;
		}
		
		FlashProg(toogle_addr(position&~0xfff), (rt_uint16_t *)buf, 0x1000>>1);
		//rt_kprintf("Program 0x%x %s\n", position&~0xfff, FlashProg(toogle_addr(position&~0xfff), (rt_uint16_t *)buf, 0x1000>>1)?"\tFail!!! Error!!!":"Ok");

		size -= tmp;
		position  += tmp;
		data  += tmp;
		tmp   = (size>0x1000)?0x1000:size;
	}	
	rt_hw_interrupt_umask(INT_GLOBAL);
	rt_mutex_release(&flash_lock);
	rt_free(buf);
	return bak_size;
}

static rt_err_t sst39vfxx_erase_block(struct rt_mtd_nor_device* device, rt_uint32_t block)
{
	struct sst39_mtd *sst39;
	int result,i;

	sst39 = SST39_MTD(device);

	RT_ASSERT(sst39 != RT_NULL);
	rt_mutex_take(&flash_lock, RT_WAITING_FOREVER);
	rt_hw_interrupt_mask(INT_GLOBAL);
	result=BlockErase(toogle_addr(block));
	rt_hw_interrupt_umask(INT_GLOBAL);
	rt_mutex_release(&flash_lock);
	return result;
}

const static struct rt_mtd_nor_driver_ops sst39vfxx_mtd_ops =
{
    sst39vfxx_read_id,
    sst39vfxx_read,
    sst39vfxx_write,
    sst39vfxx_erase_block,
};
static rt_err_t sst39vfxx_hw_init(struct sst39_mtd *mtd)
{
    mtd = mtd;
    return RT_EOK;
}

/**
 * SST39vfxx API
 */
rt_err_t sst39vfxx_mtd_init(const char * nor_name,
        rt_uint32_t block_start,
        rt_uint32_t block_end)
{
    rt_uint32_t id, total_block;
    struct sst39_mtd * sst39;
    struct rt_mtd_nor_device *mtd;

    sst39 = &_sst39_mtd;
    mtd = &(sst39->parent);

    /* set page size and block size */
    mtd->block_size = 64 * 1024; /* 64kByte */
    mtd->ops = &sst39vfxx_mtd_ops;

    /* initialize mutex */
    if (rt_mutex_init(&flash_lock, nor_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
    }

    /* initialize flash */
    id = sst39vfxx_read_id(mtd);
    rt_kprintf("sst39vfxx_mtd_init id %2x\n",id);
    switch (id & 0xff)
    {
        case MC_ID_SST39VF016:
            total_block = (16 * 1024 * 1024 / 8) / mtd->block_size;
            break;
        case MC_ID_SST39VF032:
            total_block = (32 * 1024 * 1024 / 8) / mtd->block_size;
            break;
        case MC_ID_SST39VF064:
            total_block = (64 * 1024 * 1024 / 8) / mtd->block_size;
            break;
        default:
            rt_kprintf("SST39 detection error, id: %x\n", id);
            return -RT_ERROR;
    }

    if ((block_end == RT_UINT32_MAX) || (block_end == 0))
    {
        block_end = total_block;
    }
    else if (block_end > total_block)
    {
        rt_kprintf("SST39 total block: %d, out of block\n", total_block);
        return -RT_ERROR;
    }

    mtd->block_start = block_start;
    mtd->block_end   = block_end;



    /* initialize hardware */
    sst39vfxx_hw_init(&_sst39_mtd);

    /* register MTD device */
    rt_mtd_nor_register_device("nor", mtd);

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void nor_read(const rt_uint32_t index,const rt_uint32_t index_end)
{
	rt_uint8_t *buf,*buf1;
	rt_uint32_t i,j;
	struct rt_mtd_nor_device *mtd;
	buf=(rt_uint8_t *)rt_malloc(0x10000);
	buf1=(rt_uint8_t *)rt_malloc(0x10000);
	for(i=0;i<0x10000;i++)
		buf[i]=i;
	if(buf == RT_NULL)
		rt_kprintf("alloc buffer failed\n");
	mtd = SST39_MTD(&_sst39_mtd);
	for(i=index;i<=index_end;i++)
	{
		sst39vfxx_read(mtd, i * mtd->block_size,buf1,0x10000);
		for(j=0;j<0x10000;j++)
		{
			if(buf1[j]!=buf[j])
			rt_kprintf("%d,is different from write (R %x,W %x)\n",j,buf1[j],buf[j]);
		}
	}
	rt_free(buf);
	rt_free(buf1);
}
FINSH_FUNCTION_EXPORT(nor_read, write block in SST39VF1601 flash);
void nor_write(const rt_uint32_t index,const rt_uint32_t index_end)
{
	//rt_uint32_t index;
	rt_uint8_t *buf;
	rt_uint32_t i;
	struct rt_mtd_nor_device *mtd;
	buf=rt_malloc(0x10000);
	for(i=0;i<0x10000;i++)
		buf[i]=i;
	mtd = SST39_MTD(&_sst39_mtd);
	for (i=index; i <= index_end; i++)
	{
		rt_uint32_t len = sst39vfxx_write(mtd, i * mtd->block_size,buf,0x10000);
		if(len!=0x10000)
		rt_kprintf("nor_write test failed %x\n",len);
	}
	rt_free(buf);
}
FINSH_FUNCTION_EXPORT(nor_write, write block in SST39VF1601 flash);
void nor_erase()
{
    rt_uint32_t index;
    struct rt_mtd_nor_device *mtd;

    mtd = SST39_MTD(&_sst39_mtd);
    for (index = mtd->block_start; index < mtd->block_end; index ++)
    {
        sst39vfxx_erase_block(mtd, index * mtd->block_size);
    }
}
FINSH_FUNCTION_EXPORT(nor_erase, erase block in SST39VF1601 flash);
#endif

#endif
