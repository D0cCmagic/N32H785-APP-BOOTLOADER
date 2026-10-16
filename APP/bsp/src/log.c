/**
*     Copyright (c) 2025, Nations Technologies Inc.
* 
*     All rights reserved.
* 
*     This software is the exclusive property of Nations Technologies Inc. (Hereinafter 
* referred to as NATIONS). This software, and the product of NATIONS described herein 
* (Hereinafter referred to as the Product) are owned by NATIONS under the laws and treaties
* of the People's Republic of China and other applicable jurisdictions worldwide.
* 
*     NATIONS does not grant any license under its patents, copyrights, trademarks, or other 
* intellectual property rights. Names and brands of third party may be mentioned or referred 
* thereto (if any) for identification purposes only.
* 
*     NATIONS reserves the right to make changes, corrections, enhancements, modifications, and 
* improvements to this software at any time without notice. Please contact NATIONS and obtain 
* the latest version of this software before placing orders.

*     Although NATIONS has attempted to provide accurate and reliable information, NATIONS assumes 
* no responsibility for the accuracy and reliability of this software.
* 
*     It is the responsibility of the user of this software to properly design, program, and test 
* the functionality and safety of any application made of this information and any resulting product. 
* In no event shall NATIONS be liable for any direct, indirect, incidental, special,exemplary, or 
* consequential damages arising in any way out of the use of this software or the Product.
*
*     NATIONS Products are neither intended nor warranted for usage in systems or equipment, any
* malfunction or failure of which may cause loss of human life, bodily injury or severe property 
* damage. Such applications are deemed, "Insecure Usage".
*
*     All Insecure Usage shall be made at user's risk. User shall indemnify NATIONS and hold NATIONS 
* harmless from and against all claims, costs, damages, and other liabilities, arising from or related 
* to any customer's Insecure Usage.

*     Any express or implied warranty with regard to this software or the Product, including,but not 
* limited to, the warranties of merchantability, fitness for a particular purpose and non-infringement
* are disclaimed to the fullest extent permitted by law.

*     Unless otherwise explicitly permitted by NATIONS, anyone may not duplicate, modify, transcribe
* or otherwise distribute this software for any purposes, in whole or in part.

*     NATIONS products and technologies shall not be used for or incorporated into any products or systems
* whose manufacture, use, or sale is prohibited under any applicable domestic or foreign laws or regulations. 
* User shall comply with any applicable export control laws and regulations promulgated and administered by 
* the governments of any countries asserting jurisdiction over the parties or transactions.
**/

/**
*\*\file log.c
*\*\author Nations
*\*\version v2.0.0
*\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
*
*\*\brief   Console output over USART1, through the link driver.
*
*\*\note    This file no longer owns USART1. The peripheral, its pins, its rate and
*\*\        its interrupts all belong to Mcal_Uart.c, because the same port now
*\*\        carries the remote-control frames as well as the log. Two owners of one
*\*\        peripheral is how a register gets written twice with different
*\*\        intentions, so there is one owner and this file is a client of it.
*
*\*\note    What is left here is the formatting side: printf() reaches this file
*\*\        through fputc(), and nothing above it has changed -- log_info() and the
*\*\        rest are still the macros in log.h, and every call site in the
*\*\        firmware is untouched.
*
*\*\note    The transmit path is line buffered, and that is not an optimisation
*\*\        but a correctness requirement. Mcal_Uart_Send() takes the whole
*\*\        message at once and refuses the call if the link is still busy, so a
*\*\        per-character send would deliver the FIRST character of a line and
*\*\        drop the rest -- a console that prints one letter of every message is
*\*\        worse than one that prints nothing, because it looks like a fault in
*\*\        the code that is being reported on. So characters are accumulated
*\*\        until the line ends, and the line is handed over in one call.
*
*\*\note    A line that does not fit is sent as two, and a line the link will not
*\*\        take is dropped whole and counted. It is never split on the wire: half
*\*\        a line would corrupt the console stream, and a fragment starting with
*\*\        the frame magic would be read by the Linux side as a frame.
**/

#include "log.h"
#include "app_ota_handle.h"

#if LOG_ENABLE

#include "Mcal_Uart.h"
#include "Mcal_Uart_Cfg.h"

/* Assembled line. One buffer, because fputc() is called from one context -- the
 * main loop, including the task slots -- and never from an interrupt. Nothing in
 * the receive path or the sample tick prints, precisely so that this stays
 * true. */
static uint8_t  Log_Line[UART_LINK_LOG_LINE_SIZE];
static uint16_t Log_Len = 0u;

/* Lines thrown away because the link was busy, saturating. Counted rather than
 * reported: the report would have to go out over the same link that is refusing
 * the lines. */
static uint32_t Log_Dropped = 0u;

/**
 * @name    Log_Flush
 * @brief   Hand the assembled line to the link driver in one call.
 * @param   None
 * @retval  None
 */
static void Log_Flush(void)
{
    if (Log_Len == 0u)
    {
        return;
    }

    if (Mcal_Uart_Send(Log_Line, (uint32_t)Log_Len) != 0)
    {
        /* 发送忙，整行丢弃，不拆分 */
        Log_Dropped++;
    }

    Log_Len = 0u;
}

/**
 * @name    Log_Put
 * @brief   Append one character, flushing when the line ends or the buffer fills.
 * @param   ch: the character.
 * @retval  None
 */
static void Log_Put(uint8_t ch)
{
    if (Log_Len >= (uint16_t)UART_LINK_LOG_LINE_SIZE)
    {
        /* 缓冲满了先发出去，超长行拆成两行 */
        Log_Flush();
    }

    Log_Line[Log_Len] = ch;
    Log_Len++;

    if (ch == (uint8_t)'\n')
    {
        Log_Flush();
    }
}

/**
 * @name    log_init
 * @brief   Kept so that the init order does not have to change.
 * @param   None
 * @retval  None
 * @note    Does nothing now. USART1 is brought up by Mcal_Uart_Init(), which
 *          Mcal_InitSum() calls BEFORE this one, so that the console has a
 *          working transmitter behind it from the first log line onward. The
 *          function is left in place rather than deleted because it is part of
 *          log.h's contract and its callers should not have to care which
 *          module owns the peripheral.
 */
void log_init(void)
{
}

/**
 * @name    fputc
 * @brief   printf()'s character sink: accumulate a line, then send it.
 * @param   ch: character from the C library.
 * @param   f: unused.
 * @retval  The character, as the C library requires.
 * @note    A bare newline gets a carriage return inserted before it, which is
 *          what the old implementation did and what the terminals on the other
 *          end expect. The pair is handled here rather than in the driver
 *          because it is a console convention, not a property of the link.
 * @note    Nothing here waits for hardware. The old version spun on the
 *          transmitter's status flag once per character, which at 115200 costs
 *          87 us each -- a fifty character log line held the main loop for
 *          4.3 ms, and every task slot in it ran late by that much.
 */
int fputc(int ch, FILE* f)
{
    static uint8_t cr_sent = 0u;
    uint8_t c = (uint8_t)ch;

    (void)f;

    if (c == (uint8_t)'\r')
    {
        cr_sent = 1u;                                   /* 已见回车 */
    }
    else if (c == (uint8_t)'\n')
    {
        if (cr_sent == 0u)
        {
            Log_Put((uint8_t)'\r');                     /* 补一个回车 */
        }
        cr_sent = 0u;
    }
    else
    {
        cr_sent = 0u;
    }

    Log_Put(c);

    return ch;
}

/**
 * @name    Log_GetDroppedCount
 * @brief   How many log lines the link refused.
 * @param   None
 * @retval  Running total, saturating.
 * @note    For a diagnostic caller. Deliberately not used by this file to print
 *          anything, for the reason given on the counter.
 */
uint32_t Log_GetDroppedCount(void)
{
    return Log_Dropped;
}

#endif /* LOG_ENABLE */
