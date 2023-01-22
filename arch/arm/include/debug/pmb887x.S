/* SPDX-License-Identifier: GPL-2.0-only */

.macro	addruart, rp, rv, tmp
	mov \rp, #0xf1000000 @ USART1_BASE
	mov \rv, #0xf1000000 @ USART1_BASE
.endm

.macro	waituarttxrdy,rd,rx
.endm

.macro	waituartcts,rd,rx
.endm

.macro	senduart,rd,rx
	str	\rd, [\rx, #0x20] @ USART_TXB
.endm

.macro	busyuart,rd,rx
1002:
	ldr	\rd, [\rx, #0x68] @ USART_RIS
	tst	\rd, #0x01 @ USART_RIS_TX
	beq	1002b
	
	mov \rd, #0x01 @ USART_ICR_TX
	str	\rd, [\rx, #0x70] @ USART_ICR
.endm
