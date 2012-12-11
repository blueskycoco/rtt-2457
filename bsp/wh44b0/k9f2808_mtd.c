/*
 * File      : rtthread.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-10-13     prife        the first version 
 * 2012-03-11     prife        use mtd device interface
 * 2012-12-11	 bbstr	  add sst39vf10601 1.2Mbyte managed by uffs together
*/

#include <rtdevice.h>
#include <s3c44b0.h>
#include "board.h"

/* nand flash commands. This appears to be generic across all NAND flash chips */
#define CMD_READ			0x00	//  Read
#define CMD_READ1			0x01	//  Read1
#define CMD_READ2			0x50	//  Read2
#define CMD_READID			0x90	//  ReadID
#define CMD_WRITE1			0x80	//  Write phase 1
#define CMD_WRITE2			0x10	//  Write phase 1
#define CMD_ERASE1			0x60	//  Erase phase 1
#define CMD_ERASE2			0xd0	//  Erase phase 1
#define CMD_STATUS			0x70	//  Status read
#define CMD_RESET			0xff	//  Reset

#define NF_CMD(cmd)			{*(volatile rt_uint8_t *)0x02000002 = (cmd); }
#define NF_ADDR(addr)		{*(volatile rt_uint8_t *)0x02000004 = (addr); }	
#define NF_CE_L()			{PDATC = PDATC & ~(1<<9) ; }
#define NF_CE_H()			{PDATC = PDATC | (1<<9) ; }
#define NF_RDDATA()			(*(volatile rt_uint8_t *)0x02000000)
#define NF_RDDATA8()		(*(volatile rt_uint8_t *)0x02000000)
#define NF_WRDATA(data)		{*(volatile rt_uint8_t *)0x02000000 = (data); }
#define NF_WRDATA8(data)	{*(volatile rt_uint8_t *)0x02000000 = (data); } 
#define NF_WAITRB()			{while(!(PDATC&(1<<8)));} 
#define STATUS_READY        0x40    // ready
#define STATUS_ERROR        0x01    // error
#define	STATUS_ILLACC       0x08    // illegal access
#define NAND_END_BLOCK 1024

/* configurations */
#define PAGE_DATA_SIZE                  512
#define BLOCK_MARK_SPARE_OFFSET         4
static struct rt_mutex nand;
/*	add for nor flash 2012-12-11	*/
//#define RT_USING_NOR_AS_NAND
#ifdef RT_USING_NOR_AS_NAND
#define NOR_START_BLOCK 10
#define NOR_SPARE_BLOCK 29
#define	CHECK_DELAY	150000
#define ROM_BASE 0x00000000
#define toogle_addr(r)	((r))
rt_base_t	baset;

#define inportw(r) 		(*(volatile rt_uint16_t *)(r))
#define outportw(r, d) 	(*(volatile rt_uint16_t *)(d) = r)

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
			rt_hw_interrupt_enable(baset);
			return RT_EOK;
		}
		else
		{
			PreData = CurrData;
			TimeOut++;
		}
	}
	rt_hw_interrupt_enable(baset);
	return -1;
}

static rt_err_t sst39vf_mtd_check_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	//for nor flash ,there is no bad block
	return RT_EOK;
}

static rt_err_t sst39vf_mtd_mark_bad_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	//for nor flash ,there is no bad block
	return RT_EOK;
}
static rt_uint8_t SectorErase(rt_uint32_t sector)
{
	baset=rt_hw_interrupt_disable();
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x8080, ROM_BASE+0xaaaa);
	outportw(0xaaaa, ROM_BASE+0xaaaa);
	outportw(0x5555, ROM_BASE+0x5554);
	outportw(0x3030, ROM_BASE+sector);	
	return check_toggle_ready(ROM_BASE+sector);
}

static int FlashProg(rt_uint32_t ProgStart, rt_uint16_t *DataPtr, rt_uint32_t WordCnt)
{	

	for( ; WordCnt; ProgStart+=2, DataPtr++, WordCnt--) {
		baset=rt_hw_interrupt_disable();
		outportw(0xaaaa, ROM_BASE+0xaaaa);
		outportw(0x5555, ROM_BASE+0x5554);
		outportw(0xa0a0, ROM_BASE+0xaaaa);
		outportw(*DataPtr, ROM_BASE+ProgStart);

		if(check_toggle_ready(ROM_BASE+ProgStart)!=RT_EOK)
		{
			return RT_ERROR;
		}

	}
	return RT_EOK;
}

static rt_err_t sst39vf_mtd_erase_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	//step1 get offset of sst39vf's sector position
	rt_uint32_t block_offs=(block-NAND_END_BLOCK)*16*1024+NOR_START_BLOCK*64*1024;//offs of sst39vf1601, form block 10 ,then to sector address
	int i;
	rt_uint16_t rt_err;
	rt_uint8_t *spare_buf=(rt_uint8_t *)rt_malloc(4096);
	//step2 erase 4 sector(one sector is 4k byte, nand's 1 block = 32 page = 32*512 byte = 4 sector, so we need erase 4 sector at a time )
	for(i=0;i<4;i++)
	{
		rt_err=SectorErase(block_offs & ~0xfff);
		block_offs=block_offs+4096;
		if(rt_err!=RT_EOK) 
		{	//4K Bytes boudary
			rt_kprintf("erase nor block %d failed\n",block_offs/(32*512));
			return RT_ERROR;
		}
	}
	//step3 update spare area at the last 10 sectors 16 bytes to 0xff
	/*read back spare data, 1 sector is enough*/
	rt_uint32_t spare_offs = ((block - NAND_END_BLOCK)/8)*4*1024 + NOR_SPARE_BLOCK*64*1024;
	rt_memcpy(spare_buf,(rt_uint8_t *)spare_offs,4096);
	/*erase this sector*/
	if(SectorErase(spare_offs&~0xfff)==RT_EOK)
	{
		for(i=((block-NAND_END_BLOCK)%8)*512;i<((block-NAND_END_BLOCK)%8+1)*512;i++)//one sector=4096byte can store 8block's spare info
			spare_buf[i]=0xff;
	}
	/*wrtie new data ,0xff to this block's spare*/
	if(FlashProg(spare_offs,(rt_uint16_t *)spare_buf,2048)!=RT_EOK);
	{
		rt_kprintf("prog nor block spare %d failed\n",spare_offs/(32*512));
		rt_free(spare_buf);
		return RT_ERROR;
	}
	
	rt_free(spare_buf);
	return RT_EOK;
}

static rt_err_t sst39vf_mtd_read(
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		rt_uint8_t * data, rt_uint32_t data_len, 
		rt_uint8_t * spare, rt_uint32_t spare_len)
{
	// get offset of sst39vf's position
	if (data != RT_NULL && data_len != 0)
	{	
		rt_uint32_t page_offs = (page-NAND_END_BLOCK*32)*512 + NOR_START_BLOCK*64*1024;
		rt_memcpy(data,(rt_uint8_t *)page_offs,data_len);
	}

	
	if (spare != RT_NULL && spare_len != 0)
	{
		rt_uint32_t spare_offs = (page-NAND_END_BLOCK*32)*16 + NOR_SPARE_BLOCK*64*1024;
		rt_memcpy(spare,(rt_uint8_t *)spare_offs,spare_len);
	}
	return RT_EOK;
}

static rt_err_t sst39vf_mtd_write (
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		const rt_uint8_t * data, rt_uint32_t data_len,
		const rt_uint8_t * spare, rt_uint32_t spare_len)
{
	if (data != RT_NULL && data_len != 0)
	{	
		//step1 get offset of sst39vf's position		
		rt_uint8_t *buf=(rt_uint8_t *)rt_malloc(4096);
		rt_uint32_t page_offs = ((page-NAND_END_BLOCK*32)/8)*4*1024 + NOR_START_BLOCK*64*1024;
		int i;
		//step2 copy one sector back
		rt_memcpy(buf,(rt_uint8_t *)page_offs,4096);
		//step3 erase this sector
		if(SectorErase(page_offs&~0xfff)==RT_EOK)
		{
			//step4 modify data
			for(i=((page-NAND_END_BLOCK*32)%8)*512;i<((page-NAND_END_BLOCK*32)%8+1)*512;i++)
				buf[i]=data[i%512];
		}
		/*wrtie new data to this block's spare*/
		if(FlashProg(page_offs,(rt_uint16_t *)buf,2048)!=RT_EOK);
		{
			rt_kprintf("prog nor block %d ,page %x failed\n",page_offs/(32*512),page_offs/512);
			rt_free(buf);
			return RT_ERROR;
		}		
	}

	
	if (spare != RT_NULL && spare_len != 0)
	{
		//step1 get offset of sst39vf's position		
		rt_uint8_t *spare_buf=(rt_uint8_t *)rt_malloc(4096);
		rt_uint32_t spare_offs = ((page-NAND_END_BLOCK*32)/256)*4*1024 + NOR_START_BLOCK*64*1024;
		int i;
		//step2 copy one sector back
		rt_memcpy(spare_buf,(rt_uint8_t *)spare_offs,4096);
		//step3 erase this sector
		if(SectorErase(spare_offs&~0xfff)==RT_EOK)
		{
			//step4 modify data
			for(i=((page-NAND_END_BLOCK*32)%256)*16;i<((page-NAND_END_BLOCK*32)%256+1)*16;i++)
				spare_buf[i]=spare[i%16];
		}
		/*wrtie new data to this block's spare*/
		if(FlashProg(spare_offs,(rt_uint16_t *)spare_buf,2048)!=RT_EOK);
		{
			rt_kprintf("prog nor block spare %d ,page spare %x failed\n",spare_offs/(32*512),spare_offs/512);
			rt_free(spare_buf);
			return RT_ERROR;
		}

	}
}

static rt_err_t sst39vf_read_id(
		struct rt_mtd_nand_device * dev)
{
    rt_uint32_t i;

    SWPIDEntry();
    i  = inportw(ROM_BASE);
    i |= inportw(ROM_BASE+2)<<16;
    SWPIDExit();
	rt_kprintf("sst39vf id 0X%x",i);
    return RT_EOK;	
}
#endif
/*	add for nor flash 2012-12-11	*/
rt_uint32_t NF_DETECT_RB(void)
{
	rt_uint8_t stat;
	
	NF_CMD(CMD_STATUS);
	do {
		stat = NF_RDDATA();
		//printf("%x\n", stat);
	}while(!(stat&0x40));
	NF_CMD(CMD_READ);
	return stat&1;
}
/*
 * In a page, data's ecc code is stored in spare area, from BYTE 0 to BYTEE 3.
 * Block's status byte which indicate a block is bad or not is BYTE4.
 */
static void nand_hw_init(void)
{

	/* reset nand flash */
	NF_CE_L();
	NF_CMD(CMD_RESET);
	NF_DETECT_RB();
	NF_CE_H();
}

/*
 *check the first byte in spare of the block's first page
 *return
 * good block,  RT_EOK
 * bad  blcok, return -RT_ERROR
 */
static rt_err_t k9f2808_mtd_check_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	rt_uint8_t status;
	block =  block << 5;

	NF_CE_L();
	NF_CMD(CMD_READ2);
	NF_ADDR(BLOCK_MARK_SPARE_OFFSET);
	NF_ADDR(block & 0xff);
	NF_ADDR((block >> 8) & 0xff);
	NF_ADDR((block >> 16) & 0xff);

	NF_DETECT_RB();	 /* wait for ready bit */

	status = NF_RDDATA8();
	NF_CMD(CMD_READ);
	NF_CE_H();
	//rt_kprintf("k9f2808_mtd_check_block %d %x\n",block,status);
	/* TODO: more check about status */
	return status == 0xFF ? RT_EOK : -RT_ERROR;

}

static rt_err_t k9f2808_mtd_mark_bad_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	/* get address of the fisrt page in the block */
	rt_err_t result = RT_EOK;
	block =  block << 5;

	NF_CE_L();
	NF_CMD(CMD_READ2);
	NF_CMD(CMD_WRITE1);

	NF_ADDR(BLOCK_MARK_SPARE_OFFSET);
	NF_ADDR(block & 0xff);
	NF_ADDR((block >> 8) & 0xff);

	/* write bad block mark in spare*/
	NF_WRDATA8(0);

	NF_CMD(CMD_WRITE2);
 	NF_DETECT_RB();	     /* wait for ready bit */
	NF_CMD(CMD_STATUS);	/* get the status */

	if (NF_RDDATA() &  STATUS_ERROR)
		result = -RT_ERROR;
	NF_CMD(CMD_READ);
	NF_CE_H(); /* disable chip select */
	//rt_kprintf("k9f2808_mtd_mark_bad_block %d\n",block);
    return result;
}

static rt_err_t k9f2808_mtd_erase_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	/* 1 block = 64 page= 2^6*/
    	rt_err_t result = RT_EOK;
	block <<= 5; /* get the first page's address in this block*/

	NF_CE_L();  /* enable chip */
	
	NF_CMD(CMD_ERASE1);	/* erase one block 1st command */
	NF_ADDR(block & 0xff);
	NF_ADDR((block >> 8) & 0xff);
	NF_CMD(CMD_ERASE2);	

	NF_DETECT_RB(); /* wait for ready bit */

	NF_CMD(CMD_STATUS);	/* check status	*/

	if (NF_RDDATA() & STATUS_ERROR) {
		result = -RT_ERROR;
	}

	NF_CE_H();
	//rt_kprintf("k9f2808_mtd_erase_block %d\n",block);
	return result;

}

/* return 0, ecc ok, 1, can be fixed , -1 can not be fixed */
static rt_err_t k9f2808_mtd_read(
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		rt_uint8_t * data, rt_uint32_t data_len, //may not always be 2048
		rt_uint8_t * spare, rt_uint32_t spare_len)
{
	rt_uint32_t i;
 	rt_uint32_t mecc;
	rt_uint32_t status;
	rt_err_t result = RT_EOK;
	
	NF_CE_L();
	if (data != RT_NULL && data_len != 0)
	{
		/* read page data area */
		NF_CE_L();

		NF_CMD(CMD_READ);
		NF_ADDR(0);
		NF_ADDR((page) & 0xff);
		NF_ADDR((page >> 8) & 0xff);

		NF_DETECT_RB();/* wait for ready bit */

		/*TODO: use a more quick method */
		for (i = 0; i < data_len; i++)
			data[i] = NF_RDDATA8();

	}

	if (spare != RT_NULL && spare_len != 0)
	{
		/* read page spare area */

		NF_CMD(CMD_READ2);
		NF_ADDR(0);
		NF_ADDR((page) & 0xff);
		NF_ADDR((page >> 8) & 0xff);
		//NF_ADDR((page >> 16) & 0xff);

		NF_DETECT_RB();/* wait for ready bit */
		/*TODO: use a more quick method */
		for (i = 0; i < spare_len; i++)
			spare[i] = NF_RDDATA8();

		//NF_CMD(CMD_READ);
		result = RT_EOK;
	}
	NF_CE_H();
	//for(i=0;i<spare_len;i++)
	//rt_kprintf("k9f2808_mtd_read %d\n",spare[i]);
	/* TODO: more check about status */
	return result;
}

static rt_err_t k9f2808_mtd_write (
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		const rt_uint8_t * data, rt_uint32_t data_len,//will be 2048 always!
		const rt_uint8_t * spare, rt_uint32_t spare_len)
{
	rt_uint32_t i;
	rt_uint32_t mecc0;
	rt_err_t result = RT_EOK;

	NF_CE_L();       /* enable chip */
	if (data != RT_NULL && data_len != 0)
	{
		RT_ASSERT(data_len == PAGE_DATA_SIZE);
		NF_CMD(CMD_WRITE1);

		NF_ADDR(0);
		NF_ADDR( (page) & 0xff);
		NF_ADDR((page >> 8) & 0xff);

		for(i=0; i<PAGE_DATA_SIZE; i++)
			NF_WRDATA8(data[i]);


		NF_CMD(CMD_WRITE2);
		NF_DETECT_RB();	 	/* wait for ready bit */
	}

	if (spare != RT_NULL && spare_len != 0)
	{
		NF_CMD(CMD_READ2);
		NF_CMD(CMD_WRITE1);

		NF_ADDR(0);
		NF_ADDR( (page )& 0xff);
		NF_ADDR((page >> 8) & 0xff);

		for(i=0; i<spare_len; i++)
			NF_WRDATA8(spare[i]);

		NF_CMD(CMD_WRITE2);
		//NF_CMD(CMD_READ);
		NF_DETECT_RB();
	}

__ret:
	NF_CE_H(); /* disable chip */
	//for(i=0;i<spare_len;i++)
	//rt_kprintf("k9f2808_mtd_write %d\n",spare[i]);
	return result;
}

static rt_err_t k9f2808_read_id(
		struct rt_mtd_nand_device * dev)
{
	rt_uint32_t id;

	NF_CE_L();
	NF_CMD(CMD_READID);
	NF_ADDR(0);
	NF_WAITRB();
	id  = NF_RDDATA8()<<8;
	id |= NF_RDDATA8();
	NF_CE_H();
	rt_kprintf("K9F2808 ID %x\n",id);
	return RT_EOK;
}
static rt_err_t nand_mtd_check_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	rt_err_t result=RT_EOK;
	
	rt_mutex_take(&nand, RT_WAITING_FOREVER);
	#ifdef RT_USING_NOR_AS_NAND
	if(block>=NAND_END_BLOCK)
		result = sst39vf_mtd_check_block(device,block);
	else
	#endif
		result = k9f2808_mtd_check_block(device,block);
	rt_mutex_release(&nand);
	return result;
}

static rt_err_t nand_mtd_mark_bad_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	rt_err_t result=RT_EOK;	
	rt_mutex_take(&nand, RT_WAITING_FOREVER);
	#ifdef RT_USING_NOR_AS_NAND
	if(block>=NAND_END_BLOCK)
		result = sst39vf_mtd_mark_bad_block(device,block);
	else
	#endif
		result = k9f2808_mtd_mark_bad_block(device,block);
	rt_mutex_release(&nand);
	return result;
}

static rt_err_t nand_mtd_erase_block(
		struct rt_mtd_nand_device* device,
		rt_uint32_t block)
{
	rt_err_t result=RT_EOK;	
	rt_mutex_take(&nand, RT_WAITING_FOREVER);
	#ifdef RT_USING_NOR_AS_NAND
	if(block>=NAND_END_BLOCK)
		result = sst39vf_mtd_erase_block(device,block);
	else
	#endif
		result = k9f2808_mtd_erase_block(device,block);
	rt_mutex_release(&nand);
	return result;
}

static rt_err_t nand_mtd_read(
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		rt_uint8_t * data, rt_uint32_t data_len, //may not always be 2048
		rt_uint8_t * spare, rt_uint32_t spare_len)
{
	rt_err_t result=RT_EOK;
	rt_mutex_take(&nand, RT_WAITING_FOREVER);
	#ifdef RT_USING_NOR_AS_NAND
	if(page>=NAND_END_BLOCK*32)
		result = sst39vf_mtd_read(dev,page,data,data_len,spare,spare_len);
	else
	#endif
		result = k9f2808_mtd_read(dev,page,data,data_len,spare,spare_len);
	rt_mutex_release(&nand);

	return result;
}

static rt_err_t nand_mtd_write (
		struct rt_mtd_nand_device * dev,
		rt_off_t page,
		const rt_uint8_t * data, rt_uint32_t data_len,//will be 2048 always!
		const rt_uint8_t * spare, rt_uint32_t spare_len)
{	
	rt_err_t result=RT_EOK;
	rt_mutex_take(&nand, RT_WAITING_FOREVER);
	#ifdef RT_USING_NOR_AS_NAND
	if(page>=NAND_END_BLOCK*32)
		result = sst39vf_mtd_write(dev,page,data,data_len,spare,spare_len);
	else
	#endif
		result = k9f2808_mtd_write(dev,page,data,data_len,spare,spare_len);
	rt_mutex_release(&nand);

	return result;
}

static rt_err_t nand_read_id(
		struct rt_mtd_nand_device * dev)
{
	#ifdef RT_USING_NOR_AS_NAND
	sst39vf_read_id(dev);
	#endif
	k9f2808_read_id(dev);
    return RT_EOK;	
}

const static struct rt_mtd_nand_driver_ops nand_mtd_ops =
{
	nand_read_id,
	nand_mtd_read,
	nand_mtd_write,
	RT_NULL,
	nand_mtd_erase_block,
	nand_mtd_check_block,
	nand_mtd_mark_bad_block,
};

/* interface of nand and rt-thread device */
static struct rt_mtd_nand_device nand_part[5];

void k9f2808_mtd_init()
{
	/* the first partition of nand */
	nand_part[0].page_size = PAGE_DATA_SIZE;
	nand_part[0].pages_per_block = 32;//don't caculate oob size
	nand_part[0].block_start = 0;
	nand_part[0].block_end = 255;
	nand_part[0].oob_size = 16;
	nand_part[0].ops = &nand_mtd_ops;
	rt_mtd_nand_register_device("nand0", &nand_part[0]);

	/* the second partition of nand */
	nand_part[1].page_size = PAGE_DATA_SIZE;
	nand_part[1].pages_per_block = 32;//don't caculate oob size
	nand_part[1].block_start = 256;
	nand_part[1].block_end = 512-1;
	nand_part[1].oob_size = 16;
	nand_part[1].ops = &nand_mtd_ops;
	rt_mtd_nand_register_device("nand1", &nand_part[1]);

	/* the third partition of nand */
	nand_part[2].page_size = PAGE_DATA_SIZE;
	nand_part[2].pages_per_block = 32;//don't caculate oob size
	nand_part[2].block_start = 512;
	nand_part[2].block_end = 512+256-1;
	nand_part[2].oob_size = 16;
	nand_part[2].ops = &nand_mtd_ops;
	rt_mtd_nand_register_device("nand2", &nand_part[2]);

	/* the 4th partition of nand */
	nand_part[3].page_size = PAGE_DATA_SIZE;
	nand_part[3].pages_per_block = 32;//don't caculate oob size
	nand_part[3].block_start = 512+256;
	nand_part[3].block_end = NAND_END_BLOCK-1;
	nand_part[3].oob_size = 16;
	nand_part[3].ops = &nand_mtd_ops;
	rt_mtd_nand_register_device("nand3", &nand_part[3]);
}
void nand_mtd_init()
{
	/* initialize nand controller of S3C2440 */
	nand_hw_init();

    /* initialize mutex */
	if (rt_mutex_init(&nand, "nand", RT_IPC_FLAG_FIFO) != RT_EOK)
	{
		rt_kprintf("init nand lock mutex failed\n");
	}


	k9f2808_mtd_init();
	
#ifdef RT_USING_NOR_AS_NAND
	/* the 5th partition of nand */
	nand_part[4].page_size = PAGE_DATA_SIZE;
	nand_part[4].pages_per_block = 32;//don't caculate oob size
	nand_part[4].block_start = NAND_END_BLOCK;
	nand_part[4].block_end = 1104-1;
	nand_part[4].oob_size = 16;
	nand_part[4].ops = &nand_mtd_ops;
	rt_mtd_nand_register_device("nand4", &nand_part[4]);
#endif

	nand_read_id(RT_NULL);

}
#include "finsh.h"
static char buf[PAGE_DATA_SIZE+16];
static char buf1[PAGE_DATA_SIZE+16];
static char spare[16];
static char spare1[16];
static int flag;
void nand_erase(int start, int end)
{
	int page;
	flag=1;
	rt_memset(buf, 0, PAGE_DATA_SIZE);
	rt_memset(spare, 0, 16);
	for(; start <= end; start ++)
	{
		page = start * 32;
		nand_mtd_erase_block(RT_NULL, start);
		nand_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, spare, 16);
		if (spare[0] != 0xFF)
		{
			rt_kprintf("block %d is bad, mark it bad\n", start);
			if (spare[4] == 0xFF)
			{
				spare[4] = 0x00;
				nand_mtd_write(RT_NULL, page, RT_NULL, 0, spare, 16);
			}
		}
	}
}

int nand_read(int start,int end)
{
	int i,page;
	int res;
	rt_memset(buf, 0, sizeof(buf));
	for(; start <= end; start ++)
	{
		page = start * 32;
		res = nand_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE+16, RT_NULL, 0);
		for(i=0; i<PAGE_DATA_SIZE; i++)
		{
			if(flag==0)
			{
				if(buf[i]!=buf1[i])
				rt_kprintf("nand_read block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(buf[i]!=0xff)
				rt_kprintf("nand_read block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);			
			}
		}

		for(i=0; i<16; i++)
		{
			if(flag==0)
			{
				if(buf[512+i]!=spare1[i])
				rt_kprintf("nand_read block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(buf[512+i]!=0xff)
				rt_kprintf("nand_read block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);			
			}
		}
	}
	return res;
}
int nand_write(int start,int end)
{
	int i;
	rt_err_t result=RT_EOK;
	rt_memset(buf, 0, PAGE_DATA_SIZE);
	rt_memset(spare, 0, 16);
	for(i=0; i<PAGE_DATA_SIZE; i++)
	{
		buf[i] = (i % 2) + i / 2;
		buf1[i] = buf[i];
	}
	for(i=0;i<16;i++)
	{
		spare[i]=i;
		spare1[i]=i;
	}
	flag=0;
	for(;start<=end;start++)		
	  for(i=0;i<32;i++)
		{
			result = nand_mtd_write(RT_NULL, start*32+i, buf, PAGE_DATA_SIZE, spare, 16);
			if(result!=RT_EOK)
			{
				rt_kprintf("nand_mtd_write block %d,page %d filed\n",start,start*32+i);
			}
	  	}
	return	result;
}

int nand_read2(int start,int end)
{
	int i,page;
	int res;
	rt_memset(buf, 0, sizeof(buf));		
	rt_memset(spare, 0, 16);
	for(; start <= end; start ++)
	{
		page = start * 32;

		res = nand_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, RT_NULL, 0);
		for(i=0; i<PAGE_DATA_SIZE; i++)
		{
			if(flag==0)
			{
				if(buf[i]!=buf1[i])
				rt_kprintf("nand_read2 block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(buf[i]!=0xff)
				rt_kprintf("nand_read2 block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);			
			}
		}


		res = nand_mtd_read(RT_NULL, page, RT_NULL, 0, spare, 16);
		for(i=0; i<16; i++)
		{
			if(flag==0)
			{
				if(spare[i]!=spare1[i])
				rt_kprintf("nand_read2 block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(spare[i]!=0xff)
				rt_kprintf("nand_read2 block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);			
			}
		}
	}
	return res;
}
int nand_read3(int start, int end)
{
	int i,page;
	int res;
	rt_memset(buf, 0, sizeof(buf));
	rt_memset(spare, 0, 16);
	for(; start <= end; start ++)
	{
		page = start * 32;
		res = nand_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, spare, 16);
		for(i=0; i<PAGE_DATA_SIZE; i++)
		{
			if(flag==0)
			{
				if(buf[i]!=buf1[i])
				rt_kprintf("nand_read3 block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(buf[i]!=0xff)
				rt_kprintf("nand_read3 block %d ,page %d ,i %d is not correct\n",page/32,page%32,i);			
			}
		}

		for(i=0; i<16; i++)
		{
			if(flag==0)
			{
				if(spare[i]!=spare1[i])
				rt_kprintf("nand_read3 block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);
			}else
			{
				if(spare[i]!=0xff)
				rt_kprintf("nand_read3 block %d ,page %d ,spare %d is not correct\n",page/32,page%32,i);			
			}
		}
	
	}
	return res;
}
void nand_read_total(int start, int end)
{

	nand_read(start,end);
	nand_read2(start,end);
	nand_read3(start,end);
}
int nand_check(int start, int end)
{
	for(; start <= end; start ++)
	{
		if ( nand_mtd_check_block(RT_NULL, start) != RT_EOK)
			rt_kprintf("block %d is bad\n", start);
	}
}

int nand_mark(int start,int end)
{
	rt_err_t result;
	for(; start <= end; start ++)
	{
		result = nand_mtd_mark_bad_block(RT_NULL, start);
		if(result !=RT_EOK)
			rt_kprintf("nand_mark %d block failed",start);
	}
}
void nand_id(void)
{
	nand_read_id(NULL);
}
void nand_test_erase(void)
{
	nand_erase(0,1103);
	rt_kprintf("total block are erased\n");
	nand_read_total(0,1103);
	rt_kprintf("total block are readed\n");
}
void nand_test_write(void)
{
	nand_erase(0,1103);
	rt_kprintf("total block are erased\n");
	nand_write(0,1103);
	rt_kprintf("total block are writed\n");
	nand_read_total(0,1103);
	rt_kprintf("total block are readed\n");
}
FINSH_FUNCTION_EXPORT(nand_id, nand_read_id);
FINSH_FUNCTION_EXPORT(nand_read, nand_read(0,1103).);
FINSH_FUNCTION_EXPORT(nand_read2, nand_read2(0,1103).);
FINSH_FUNCTION_EXPORT(nand_read3, nand_read3(0,1103).);
FINSH_FUNCTION_EXPORT(nand_read_total, nand_read_total(0,1103).);
FINSH_FUNCTION_EXPORT(nand_write, nand_write(0,1103).);
FINSH_FUNCTION_EXPORT(nand_check, nand_check(0,1103).);
FINSH_FUNCTION_EXPORT(nand_mark, nand_mark(0,1103).);
FINSH_FUNCTION_EXPORT(nand_test_erase, nand_test_erase().);
FINSH_FUNCTION_EXPORT(nand_test_write, nand_test_write().);
FINSH_FUNCTION_EXPORT(nand_erase, nand_erase(0, 1103). erase block in nand);
