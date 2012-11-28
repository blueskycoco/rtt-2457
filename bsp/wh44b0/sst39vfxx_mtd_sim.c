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

#ifdef RT_USING_MTD_NOR
#define ROM_BASE 0x00000000
#define	CMD_ADDR0	*((volatile rt_uint16_t *)(0x5555*2+ROM_BASE))
#define	CMD_ADDR1	*((volatile rt_uint16_t *)(0x2aaa*2+ROM_BASE))
#define NOR_SIM "nor.bin"
/* JEDEC Manufacturer¡¯s ID */
#define MF_ID           (0xBF)
/* JEDEC Device ID : Memory Type */
#define MT_ID           (0x25)
/* JEDEC Device ID: Memory Capacity */
#define MC_ID_SST39VF016               (0xBF)
#define MC_ID_SST39VF032               (0x4A)
#define MC_ID_SST39VF064               (0x4B)

#define BLOCK_SIZE   (64*1024)


#define SST39_MTD(device) 		((struct sst39_mtd*)(device))
struct sst39_mtd
{
	struct rt_mtd_nor_device parent;
	FILE * file;
};
static struct sst39_mtd _sst39_mtd;

static struct rt_mutex flash_lock;
#define	CHECK_DELAY	150000

static rt_uint16_t state;
static rt_uint16_t support;
static rt_uint32_t chip_id; 
#define inportw(r) 		(*(volatile rt_uint16_t *)r)
#define outportw(r, d) 	(*(volatile rt_uint16_t *)r = d)

static void CFIQueryExit(void)
{
	outportw(0xaaaa, ROM_BASE+0xaaaa);//CMD_ADDR0 = 0xaaaa;
	outportw(0x5555, ROM_BASE+0x5554);//CMD_ADDR1 = 0x5555;
	outportw(0xf0f0, ROM_BASE+0xaaaa);//CMD_ADDR0 = 0xf0f0;
	state &= 0xfc;	
}

static void SWPIDExit(void)
{
	outportw(0xf0f0, ROM_BASE+0xaaaa);//CMD_ADDR0 = 0xf0f0;
	state &= 0xfc;
}
static void SWPIDEntry(void)
{
	if(state&1)
	{
		if(state&2)
			return;
		else
			CFIQueryExit();
	}

	outportw(0xaaaa, ROM_BASE+0xaaaa);//CMD_ADDR0 = 0xaaaa;
	outportw(0x5555, ROM_BASE+0x5554);//CMD_ADDR1 = 0x5555;
	outportw(0x9090, ROM_BASE+0xaaaa);//CMD_ADDR0 = 0x9090;
	state |= 3;
}
/* RT-Thread MTD device interface */
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
	rt_uint32_t tm, d1 ,d2;
	
	if(state&1) {
		if(state&2)
			SWPIDExit();
		else
			CFIQueryExit();						
	}
	
	sector += ROM_BASE;
	
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x8080, ROM_BASE+0xaaaa);
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x3030, sector);	
	d2 = inportw(sector);
	
	tm = CHECK_DELAY;
	while(1) {
	
		tm--;
		if(!tm)
			return -1;
		
		d1 = d2;
		d2 = inportw(sector);		
		
		if((d1^d2)&(1<<6)) {	//D6 == D6
			continue;
		}
		
		if(inportw(sector)==0xffff) {
			rt_kprintf("tm=%d\n", tm);
			return RT_EOK;
		}
		
	}
}
static int FlashProg(rt_uint32_t ProgStart, rt_uint16_t *DataPtr, rt_uint32_t WordCnt)
{	
	ProgStart += ROM_BASE;
	
	for( ; WordCnt; ProgStart+=2, DataPtr++, WordCnt--) {
		rt_uint32_t tm;
	
		outportw(0xaaaa, ROM_BASE+0xaaaa);
		outportw(0x5555, ROM_BASE+0x5554);
		outportw(0xa0a0, ROM_BASE+0xaaaa);
		outportw(*DataPtr, ProgStart);
		
		tm = CHECK_DELAY;
		while(1) {			
			if((inportw(ProgStart)^inportw(ProgStart))&(0x40)) {	//D6 == D6
				tm--;
				if(!tm)
					return -1;
				continue;
			}
			
			if(inportw(ProgStart)==*DataPtr)
				break;					//D7 == D7
			tm--;
			if(!tm)
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
	rt_memcpy(data, (rt_uint8_t *)position, (rt_uint8_t)size);
	rt_mutex_release(&flash_lock);
	return size;
}

static int sst39vfxx_write(struct rt_mtd_nor_device* device, rt_off_t position,
		const rt_uint8_t *data, rt_size_t size)
{
	struct sst39_mtd *sst39;
	int result;
	rt_uint8_t buf[0x1000];
	rt_uint32_t  tmp = 0x1000-(position&0xfff);
	rt_uint16_t err;
	sst39 = SST39_MTD(device);
	RT_ASSERT(sst39 != RT_NULL);

	rt_mutex_take(&flash_lock, RT_WAITING_FOREVER);

	if(tmp>size)
		tmp = size;
	if(tmp&1)
		tmp++;

	for(; size;) {
		if(tmp<0x1000)
		{
			rt_memcpy(buf, (position&~0xfff), 0x1000);
			memcpy(buf+(position&0xfff), (char *)data, tmp);
		} 
		else 
		{
			memcpy(buf, (char *)data, 0x1000);
		}

		err = SectorErase(position&~0xfff);
		if(err) {	//4K Bytes boudary
			rt_kprintf("\t\tErase 0x%x Fail!!!\n", position&~0xfff);
			return;
		}

		rt_kprintf("Program 0x%x %s\n", position&~0xfff, FlashProg(position&~0xfff, (rt_uint16_t *)buf, 0x1000>>1)?"\tFail!!! Error!!!":"Ok");

		size -= tmp;
		position  += tmp;
		data  += tmp;
		tmp   = (size>0x1000)?0x1000:size;
	}	

	rt_mutex_release(&flash_lock);
	return size;
	}

static char block_buffer[BLOCK_SIZE];

static rt_err_t sst39vfxx_erase_block(struct rt_mtd_nor_device* device, rt_uint32_t block)
{
	struct sst39_mtd *sst39;
	int result,i;

	sst39 = SST39_MTD(device);

	RT_ASSERT(sst39 != RT_NULL);

	rt_mutex_take(&flash_lock, RT_WAITING_FOREVER);
	for(i=0;i<16;i++)
		result=SectorErase(i+block*16);
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
void nor_erase(void)
{
	rt_uint32_t index;
    struct rt_mtd_nor_device *mtd;

    mtd = SST39_MTD(&_sst39_mtd);
	for (index = mtd->block_start; index < mtd->block_end; index ++)
	{
		sst39vfxx_erase_block(mtd, index * mtd->block_size);
	}
}
FINSH_FUNCTION_EXPORT(nor_erase, erase all block in SPI flash);
#endif

#endif
