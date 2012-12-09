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
//#define NF_DETECT_RB()		{while(!(PDATC&(1<<8)));}

/* HCLK=100Mhz, TACLS + TWRPH0 + TWRPH1 >= 50ns */
#define TACLS				1       // 1-clock(0ns)
#define TWRPH0				4       // 3-clock(25ns)
#define TWRPH1				0       // 1-clock(10ns)

/* status bit pattern */
#define STATUS_READY        0x40    // ready
#define STATUS_ERROR        0x01    // error
#define	STATUS_ILLACC       0x08    // illegal access

/* configurations */
#define PAGE_DATA_SIZE                  512
#define BLOCK_MARK_SPARE_OFFSET         4
//#define CONFIG_USE_HW_ECC
static struct rt_mutex nand;
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

	rt_mutex_take(&nand, RT_WAITING_FOREVER);
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
	rt_mutex_release(&nand);
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
	
	rt_mutex_take(&nand, RT_WAITING_FOREVER);

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
	rt_mutex_release(&nand);
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

	rt_mutex_take(&nand, RT_WAITING_FOREVER);


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
	rt_mutex_release(&nand);
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

const static struct rt_mtd_nand_driver_ops k9f2808_mtd_ops =
{
	k9f2808_read_id,
	k9f2808_mtd_read,
	k9f2808_mtd_write,
	k9f2808_mtd_erase_block,
	k9f2808_mtd_check_block,
	k9f2808_mtd_mark_bad_block,
};

/* interface of nand and rt-thread device */
static struct rt_mtd_nand_device nand_part[4];

void k9f2808_mtd_init()
{
	/* initialize nand controller of S3C2440 */
	nand_hw_init();

    /* initialize mutex */
	if (rt_mutex_init(&nand, "nand", RT_IPC_FLAG_FIFO) != RT_EOK)
	{
		rt_kprintf("init nand lock mutex failed\n");
	}
	/* the first partition of nand */
	nand_part[0].page_size = PAGE_DATA_SIZE;
	nand_part[0].pages_per_block = 32;//don't caculate oob size
	nand_part[0].block_start = 0;
	nand_part[0].block_end = 255;
	nand_part[0].oob_size = 16;
	nand_part[0].ops = &k9f2808_mtd_ops;
	rt_mtd_nand_register_device("nand0", &nand_part[0]);

	/* the second partition of nand */
	nand_part[1].page_size = PAGE_DATA_SIZE;
	nand_part[0].pages_per_block = 32;//don't caculate oob size
	nand_part[1].block_start = 256;
	nand_part[1].block_end = 512-1;
	nand_part[1].oob_size = 16;
	nand_part[1].ops = &k9f2808_mtd_ops;
	rt_mtd_nand_register_device("nand1", &nand_part[1]);

	/* the third partition of nand */
	nand_part[2].page_size = PAGE_DATA_SIZE;
	nand_part[0].pages_per_block = 32;//don't caculate oob size
	nand_part[2].block_start = 512;
	nand_part[2].block_end = 512+256-1;
	nand_part[2].oob_size = 16;
	nand_part[2].ops = &k9f2808_mtd_ops;
	rt_mtd_nand_register_device("nand2", &nand_part[2]);

	/* the 4th partition of nand */
	nand_part[3].page_size = PAGE_DATA_SIZE;
	nand_part[0].pages_per_block = 32;//don't caculate oob size
	nand_part[3].block_start = 512+256;
	nand_part[3].block_end = 1024-1;
	nand_part[3].oob_size = 16;
	nand_part[3].ops = &k9f2808_mtd_ops;
	rt_mtd_nand_register_device("nand3", &nand_part[3]);
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
	for(; start <= end; start ++)
	{
		page = start * 32;
		rt_memset(buf, 0, PAGE_DATA_SIZE);
		rt_memset(spare, 0, 16);

		k9f2808_mtd_erase_block(RT_NULL, start);

		k9f2808_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, spare, 16);
		if (spare[0] != 0xFF)
		{
			rt_kprintf("block %d is bad, mark it bad\n", start);

			//rt_memset(spare, 0xFF, 64);
			if (spare[4] == 0xFF)
			{
				spare[4] = 0x00;
				k9f2808_mtd_write(RT_NULL, page, RT_NULL, 0, spare, 16);
			}
		}
	}
}

int nand_read(int page)
{
	int i;
	int res;
	rt_memset(buf, 0, sizeof(buf));
//	rt_memset(spare, 0, 64);

//	res = k9f2808_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, spare, 64);
	res = k9f2808_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE+16, RT_NULL, 0);
	//rt_kprintf("block=%d, page=%d\n", page/32, page%32);
	for(i=0; i<PAGE_DATA_SIZE; i++)
	{
		//rt_kprintf("%02x ", buf[i]);
		//if((i+1)%16 == 0)
		//	rt_kprintf("\n");
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

	//rt_kprintf("spare:\n");
	for(i=0; i<16; i++)
	{
//		rt_kprintf("%02x ", spare[i]);
		//rt_kprintf("%02x ", buf[512+i]);
		//if((i+1)%8 == 0)
			//rt_kprintf("\n");
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
	return res;
}
int nand_write(int start,int end)
{
	int i;
	rt_memset(buf, 0, PAGE_DATA_SIZE);
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
		for(;start<end;start++)		
		  for(i=0;i<32;i++)
			k9f2808_mtd_write(RT_NULL, start*32+i, buf, PAGE_DATA_SIZE, spare, 16);
}

int nand_read2(int page)
{
	int i;
	int res;
	rt_memset(buf, 0, sizeof(buf));

	res = k9f2808_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, RT_NULL, 0);
	//rt_kprintf("block=%d, page=%d\n", page/32, page%32);
	for(i=0; i<PAGE_DATA_SIZE; i++)
	{
		//rt_kprintf("%02x ", buf[i]);
		//if((i+1)%16 == 0)
			//rt_kprintf("\n");
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

	rt_memset(spare, 0, 16);
	res = k9f2808_mtd_read(RT_NULL, page, RT_NULL, 0, spare, 16);
	//rt_kprintf("spare:\n");
	for(i=0; i<16; i++)
	{
		//rt_kprintf("%02x ", spare[i]);
		//if((i+1)%8 == 0)
		//	rt_kprintf("\n");
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
	return res;
}
int nand_read3(int page)
{
	int i;
	int res;
	rt_memset(buf, 0, sizeof(buf));
	rt_memset(spare, 0, 16);

	res = k9f2808_mtd_read(RT_NULL, page, buf, PAGE_DATA_SIZE, spare, 16);
	//rt_kprintf("block=%d, page=%d\n", page/32, page%32);
	for(i=0; i<PAGE_DATA_SIZE; i++)
	{
		//rt_kprintf("%02x ", buf[i]);
		//if((i+1)%16 == 0)
		//	rt_kprintf("\n");
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

	//rt_kprintf("spare:\n");
	for(i=0; i<16; i++)
	{
		//rt_kprintf("%02x ", spare[i]);
		//if((i+1)%8 == 0)
		//	rt_kprintf("\n");
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
	return res;
}
void nand_readt(int start, int end)
{
	int page,i;
	for(; start <= end; start ++)
	{
		page = start * 32;
		rt_memset(buf, 0, PAGE_DATA_SIZE+16);
		rt_memset(spare, 0, 16);
		for(i=0;i<32;i++){
		nand_read(page+i);
		nand_read2(page+i);
		nand_read3(page+i);
	}
	}
}
int nand_check(int block)
{
	if ( k9f2808_mtd_check_block(RT_NULL, block) != RT_EOK)
		rt_kprintf("block %d is bad\n", block);
	//else
	//	rt_kprintf("block %d is good\n", block);
}

int nand_mark(int block)
{
	return k9f2808_mtd_mark_bad_block(RT_NULL, block);
}
void nand_id(void)
{
	k9f2808_read_id(NULL);
}
void nand_checkt(void)
{
	int i;
	for(i=0;i<1024;i++)
	nand_check(i);
}
FINSH_FUNCTION_EXPORT(nand_id, nand_read_id);
FINSH_FUNCTION_EXPORT(nand_read, nand_read(0).);
FINSH_FUNCTION_EXPORT(nand_read2, nand_read(1).);
FINSH_FUNCTION_EXPORT(nand_read3, nand_read(1).);
FINSH_FUNCTION_EXPORT(nand_write, nand_write(0).);
FINSH_FUNCTION_EXPORT(nand_check, nand_check(1).);
FINSH_FUNCTION_EXPORT(nand_checkt, nand_check(1).);
FINSH_FUNCTION_EXPORT(nand_mark, nand_mark(1).);
FINSH_FUNCTION_EXPORT(nand_erase, nand_erase(0, 1024). erase block in nand);
FINSH_FUNCTION_EXPORT(nand_readt, nand_erase(0, 1024). erase block in nand);