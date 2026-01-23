/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * 一个 pvPortMalloc() 和 vPortFree() 的示例实现，该实现会在释放内存时合并相邻的内存块，
 * 从而限制内存碎片化。
 *
 * 可以参考 heap_1.c、heap_2.c 和 heap_3.c 中的其他实现方式，并访问 
 * http://www.FreeRTOS.org 查看更多相关信息。
 */
#include <stdlib.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 可防止 task.h 重新定义所有 API 函数以使用 MPU 包装器。
   这应该只在从应用程序文件中包含 task.h 时进行。 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

//#include "FreeRTOS.h"		/* 屏蔽 */
//#include "task.h"			/* 屏蔽 */
#include <stdio.h>
#include <stdint.h>	/* 添加 */
#include "osConfig.h"/* 添加 */
#include "osHardWare.h"/* 添加 */

/* 添加中断保护或者任务切换保护,由于OSAL无任务转换,故只需要屏蔽中断就行 */
void vTaskSuspendAll( void )/* 禁止任务转换 */
{
	DISABLE_IRQ();
}

void xTaskResumeAll( void )/* 使能任务切换 */
{
	ENABLE_IRQ();
}



#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* 如果动态分配支持被关闭，则不能使用此文件 */
//#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
//	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
//#endif

/* 块大小不能太小 */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )		/* X2 */

/* 假设每个字节为8位 */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

/* 分配堆内存 */
#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* 应用程序已经定义了用于RTOS堆的数组——可能是为了将其放在特定段或地址中 */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ]; /* 默认静态分配堆空间 */
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* 定义空闲内存块链表结构 */
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/* 下一个空闲块 */
	size_t xBlockSize;						/* 当前块大小 */
} BlockLink_t;
/*-----------------------------------------------------------*/

/*
 * 将一个正在被释放的内存块插入到空闲块列表中的正确位置。
 * 如果内存块彼此相邻，将合并它们。
 */
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

/*
 * 第一次调用 pvPortMalloc() 时自动调用，用于初始化所需的堆结构。
 */
static void prvHeapInit( void );
/*-----------------------------------------------------------*/

/* 每个已分配内存块开头的结构体大小，必须正确对齐 */
static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* 创建两个链表节点标记链表的开始和结束 */
static BlockLink_t xStart, *pxEnd = NULL;

/* 跟踪剩余可用字节数，但不涉及碎片情况 */
static size_t xFreeBytesRemaining = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;

/* 标记当前块是否已被分配（最高位设置） */
static size_t xBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

/* 
* pvPortMalloc函数用于从动态内存堆中分配指定大小的内存块
* 参数xWantedSize指定所需内存块的大小
* 返回值：如果成功分配内存，则返回指向分配内存块的指针；如果无法分配指定大小的内存块，则返回NULL
*/
void *pvPortMalloc( size_t xWantedSize )
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    // 关闭中断，确保原子操作
    vTaskSuspendAll();
    {
        // 首次调用需初始化堆
        if( pxEnd == NULL )
        {
            prvHeapInit();
        }

        // 检查请求的内存块大小是否需要对齐
        if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
        {
            if( xWantedSize > 0 )
            {
                xWantedSize += xHeapStructSize;

                // 确保内存块大小按字节对齐
                if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
                {
                    xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
                    configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
                }
            }

            // 检查是否有足够的可用内存
            if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
            {
                pxPreviousBlock = &xStart;
                pxBlock = xStart.pxNextFreeBlock;

                // 遍历空闲内存块，寻找适合的内存块
                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }

                // 检查是否找到合适的内存块
                if( pxBlock != pxEnd )
                {
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    // 如果剩余的内存块大小大于最小块大小，则分割内存块
                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                    {
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                        pxBlock->xBlockSize = xWantedSize;

                        prvInsertBlockIntoFreeList( pxNewBlockLink );
                    }

                    // 更新剩余可用内存大小
                    xFreeBytesRemaining -= pxBlock->xBlockSize;

                    // 更新历史最小剩余内存大小
                    if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
                    {
                        xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                    }

                    // 标记内存块为已分配
                    pxBlock->xBlockSize |= xBlockAllocatedBit;
                    pxBlock->pxNextFreeBlock = NULL;
                }
            }
        }
        // 跟踪内存分配事件
        traceMALLOC( pvReturn, xWantedSize );
    }
    // 恢复中断状态
    ( void ) xTaskResumeAll();

    // 打印内存分配信息
    HEAP_PRINTF("\nMalloc->total:%d,size:%d\n",xFreeBytesRemaining,xWantedSize);

    // 如果配置了内存分配失败的钩子函数，并且内存分配失败，则调用钩子函数
    #if( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )
        {
            extern void vApplicationMallocFailedHook(uint32_t total, uint32_t size);
            vApplicationMallocFailedHook(xFreeBytesRemaining,xWantedSize);
        }
    }
    #endif

    // 断言返回的内存地址按字节对齐
    configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
    return pvReturn;
}
/*-----------------------------------------------------------*/

/**
 * 释放之前分配的内存块。
 * 
 * 该函数负责将之前通过pvPortMalloc分配的内存块标记为可用，并更新内存管理结构。
 * 它通过将内存块的起始地址向前移动到包含块管理信息的位置，来访问和修改内存块的管理数据。
 * 
 * @param pv 指向要释放的内存块的指针。如果为NULL，则函数直接返回，不执行任何操作。
 */
void vPortFree( void *pv )
{
    // 将传入的指针转换为uint8_t指针，以便进行字节级别的操作。
    uint8_t *puc = ( uint8_t * ) pv;
    // 定义一个BlockLink_t类型的指针，用于指向内存块的管理信息。
    BlockLink_t *pxLink;

    // 检查传入的指针是否为NULL，如果为NULL，则直接返回。
    if( pv != NULL )
    {
        // 将指针向前移动xHeapStructSize字节，以指向内存块的管理信息。
        puc -= xHeapStructSize;
        // 将调整后的指针转换为BlockLink_t类型。
        pxLink = ( void * ) puc;

        // 检查内存块是否被标记为已分配。
        if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
        {
            // 如果内存块是最后一个被分配的块，则执行释放操作。
            if( pxLink->pxNextFreeBlock == NULL )
            {
                // 清除内存块的已分配标记。
                pxLink->xBlockSize &= ~xBlockAllocatedBit;

                // 暂停任务调度，以防止在更新内存管理结构时发生中断。
                vTaskSuspendAll();
                {
                    // 更新可用内存量。
                    xFreeBytesRemaining += pxLink->xBlockSize;
                    // 调用跟踪函数，记录内存释放事件。
                    traceFREE( pv, pxLink->xBlockSize );
                    // 将释放的内存块插入到空闲列表中。
                    prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
                }
                // 恢复任务调度。
                ( void ) xTaskResumeAll();

                // 打印释放内存后的总可用内存和释放的内存块大小。
                HEAP_PRINTF("\nFree->total:%d,size:%d\n",xFreeBytesRemaining,pxLink->xBlockSize);    
            }
        }
    }
}
/*-----------------------------------------------------------*/
size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}

size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return xMinimumEverFreeBytesRemaining;
}

void vPortInitialiseBlocks( void )
{
	/* 此函数只是为了保持链接器安静 */
}
/*-----------------------------------------------------------*/

/**
 * @brief 初始化内存堆
 * 
 * 该函数对内存堆进行初始化，为动态内存分配做准备它负责设置内存堆的起始和结束位置，
 * 以及初始化内存堆中的第一个空闲块和相关变量
 */
static void prvHeapInit( void )
{
    // 指向第一个空闲内存块的指针
    BlockLink_t *pxFirstFreeBlock;
    // 对齐后的内存堆起始地址
    uint8_t *pucAlignedHeap;
    // 用于计算和存储地址的变量
    size_t uxAddress;
    // 定义内存堆的总大小
    size_t xTotalHeapSize = configTOTAL_HEAP_SIZE;

    // 获取内存堆的起始地址
    uxAddress = ( size_t ) ucHeap;

    // 检查内存堆起始地址是否按字节对齐
    if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        // 如果未对齐，则调整地址到最近的对齐边界
        uxAddress += ( portBYTE_ALIGNMENT - 1 );
        uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
        // 调整总内存堆大小以反映对齐造成的损失
        xTotalHeapSize -= uxAddress - ( size_t ) ucHeap;
    }

    // 将对齐后的地址赋值给内存堆起始指针
    pucAlignedHeap = ( uint8_t * ) uxAddress;

    // 初始化内存堆起始位置的下一个空闲块指针和块大小
    xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    xStart.xBlockSize = ( size_t ) 0;

    // 计算内存堆结束地址，并确保其对齐
    uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
    // 初始化内存堆结束位置的指针和块大小
    pxEnd = ( void * ) uxAddress;
    pxEnd->xBlockSize = 0;
    pxEnd->pxNextFreeBlock = NULL;

    // 初始化第一个空闲块的指针和大小
    pxFirstFreeBlock = ( void * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

    // 初始化记录最小空闲字节数和当前空闲字节数的变量
    xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

    // 初始化内存分配标志位
    xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/

/**
 * 将内存块插入到空闲列表中的适当位置
 * 此函数旨在维护空闲内存块列表的有序性，以便于内存的有效管理和分配
 * 
 * @param pxBlockToInsert 指向要插入空闲列表的内存块
 */
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
    // 初始化列表迭代器
    BlockLink_t *pxIterator;
    // 用于计算内存块地址的辅助变量
    uint8_t *puc;

    // 遍历空闲列表，寻找新内存块的插入位置
    for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
    {
        /* 遍历找到合适的位置 */
    }

    // 检查新内存块是否与前一个内存块相邻，如果相邻则合并
    puc = ( uint8_t * ) pxIterator;
    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }

    // 检查新内存块是否与后一个内存块相邻，如果相邻则合并
    puc = ( uint8_t * ) pxBlockToInsert;
    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
    {
        if( pxIterator->pxNextFreeBlock != pxEnd )
        {
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = pxEnd;
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    // 更新列表中前一个内存块的指针，确保列表的连续性
    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
}

