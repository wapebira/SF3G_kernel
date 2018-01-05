/* ----------------------------------------------------------------------- *
 *
 *   Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd
 *   Revision 1.00  2015/06/26 ZYF
 * ----------------------------------------------------------------------- */

	.file	"sofia3grftl.S"
	.text
	.p2align 4,,15
	.type	nand_ioctl, @function
nand_ioctl:
	pushl	%ebp
	cmpl	$22017, %ecx
	movl	%esp, %ebp
	pushl	%ebx
	movl	92(%eax), %eax
	movl	524(%eax), %ebx
	movl	(%ebx), %eax
	je	.L5
	jbe	.L12
	cmpl	$22018, %ecx
	je	.L6
	cmpl	$22019, %ecx
	je	.L13
.L9:
	movl	$-25, %eax
.L2:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L12:
	cmpl	$4705, %ecx
	je	.L3
	cmpl	$22016, %ecx
	jne	.L9
	movl	40(%ebx), %eax
	movl	$1, %edx
	movl	$1, 28(%ebx)
	call	set_disk_ro
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L13:
	movl	$0, 36(%ebx)
	xorl	%eax, %eax
	movl	$0, 32(%ebx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L6:
	movl	$1, 32(%ebx)
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L3:
	movl	20(%eax), %edx
	xorl	%eax, %eax
	testl	%edx, %edx
	je	.L2
	movl	%ebx, %eax
	call	*%edx
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L5:
	movl	40(%ebx), %eax
	xorl	%edx, %edx
	movl	$0, 36(%ebx)
	movl	$0, 28(%ebx)
	call	set_disk_ro
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_ioctl, .-nand_ioctl
	.p2align 4,,15
	.type	nand_release, @function
nand_release:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	524(%eax), %eax
	movl	(%eax), %ebx
	movl	28(%ebx), %edx
	testl	%edx, %edx
	je	.L15
	call	*%edx
	testl	%eax, %eax
	je	.L15
	popl	%ebx
	popl	%ebp
	.p2align 4,,7
	ret
	.p2align 4,,7
	.p2align 3
.L15:
	movl	112(%ebx), %eax
	.p2align 4,,5
	call	module_put
	popl	%ebx
	popl	%ebp
	.p2align 4,,2
	ret
	.size	nand_release, .-nand_release
	.p2align 4,,15
	.type	nand_open, @function
nand_open:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$12, %esp
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	movl	92(%eax), %eax
	movl	524(%eax), %edi
	movl	(%edi), %ebx
	movl	112(%ebx), %eax
	call	try_module_get
	testb	%al, %al
	je	.L20
	movl	24(%ebx), %edx
	xorl	%esi, %esi
	testl	%edx, %edx
	je	.L19
	movl	%edi, %eax
	call	*%edx
	testl	%eax, %eax
	movl	%eax, %esi
	je	.L19
.L18:
	movl	112(%ebx), %eax
	call	module_put
.L19:
	movl	%esi, %eax
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L20:
	movl	$-19, %esi
	jmp	.L18
	.size	nand_open, .-nand_open
	.p2align 4,,15
	.type	rk_nandc_interrupt, @function
rk_nandc_interrupt:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	372(%edx), %eax
	testb	$1, %al
	je	.L23
	movl	368(%edx), %ecx
	movl	$1, nandc0_xfer_completed_flag
	orl	$1, %ecx
	movl	%ecx, 368(%edx)
	movl	364(%edx), %ecx
	andl	$-2, %ecx
	movl	%ecx, 364(%edx)
.L23:
	testb	$2, %al
	je	.L24
	movl	368(%edx), %ecx
	movl	$1, nandc0_ready_completed_flag
	orl	$2, %ecx
	movl	%ecx, 368(%edx)
	movl	364(%edx), %ecx
	andl	$-3, %ecx
	movl	%ecx, 364(%edx)
.L24:
	testb	$16, %al
	je	.L25
	movl	368(%edx), %ecx
	movl	$1, nandc0_llp_completed_flag
	orl	$16, %ecx
	movl	%ecx, 368(%edx)
	movl	364(%edx), %ecx
	andl	$-17, %ecx
	movl	%ecx, 364(%edx)
.L25:
	testb	$8, %al
	je	.L26
	movl	368(%edx), %eax
	movl	$1, gNandcEccError
	movl	$1, nandc0_llp_completed_flag
	movl	$1, nandc0_xfer_completed_flag
	orl	$8, %eax
	movl	%eax, 368(%edx)
	movl	364(%edx), %eax
	andl	$-9, %eax
	movl	%eax, 364(%edx)
.L26:
	movl	$rk29_nandc_wait, %eax
	movl	$1, %ecx
	movl	$3, %edx
	movl	$0, (%esp)
	call	__wake_up
	movl	$1, %eax
	leave
	ret
	.size	rk_nandc_interrupt, .-rk_nandc_interrupt
	.p2align 4,,15
	.type	nand_remove_dev, @function
nand_remove_dev:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	40(%eax), %ebx
	movl	4(%eax), %ecx
	movl	8(%eax), %edx
	movl	%edx, 4(%ecx)
	movl	%ecx, (%edx)
	movl	$1048832, 4(%eax)
	movl	$2097664, 8(%eax)
	movl	%ebx, %eax
	movl	$0, 520(%ebx)
	call	del_gendisk
	movl	%ebx, %eax
	call	put_disk
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_remove_dev, .-nand_remove_dev
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC0:
	.string	"%s%d"
	.text
	.p2align 4,,15
	.type	nand_add_dev, @function
nand_add_dev:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	$-1, %esi
	pushl	%ebx
	subl	$28, %esp
	movl	36(%edx), %ebx
	movl	%eax, -16(%ebp)
	movl	%edx, -20(%ebp)
	testl	%ebx, %ebx
	je	.L29
	movl	kmalloc_caches+24, %eax
	movl	$208, %edx
	call	kmem_cache_alloc
	testl	%eax, %eax
	movl	%eax, %ebx
	je	.L29
	testb	$1, %al
	movl	%eax, %edi
	movl	$44, %edx
	jne	.L51
.L30:
	testl	$2, %edi
	jne	.L52
.L31:
	movl	%edx, %ecx
	xorl	%eax, %eax
	shrl	$2, %ecx
	testb	$2, %dl
	rep stosl
	jne	.L53
	andl	$1, %edx
	jne	.L54
.L33:
	movl	-20(%ebp), %edx
	movl	-16(%ebp), %eax
	movl	%eax, (%ebx)
	movl	36(%edx), %eax
	movl	%eax, 20(%ebx)
	movl	32(%edx), %eax
	movl	$-1, 16(%ebx)
	movl	%eax, 24(%ebx)
	movl	-16(%ebp), %eax
	movl	104(%eax), %edx
	addl	$104, %eax
	cmpl	%eax, %edx
	je	.L34
	movl	12(%edx), %ecx
	xorl	%esi, %esi
	testl	%ecx, %ecx
	je	.L36
	jmp	.L35
	.p2align 4,,7
	.p2align 3
.L38:
	movl	12(%edx), %ecx
	addl	$1, %esi
	cmpl	%esi, %ecx
	jne	.L35
	movl	%ecx, %esi
.L36:
	movl	(%edx), %edx
	cmpl	%edx, %eax
	jne	.L38
	movl	-16(%ebp), %edx
	addl	$1, %esi
	movl	%esi, 16(%ebx)
	movl	8(%edx), %ecx
	sall	%cl, %esi
	movl	%esi, %ecx
	movl	$-16, %esi
	cmpl	$256, %ecx
	jg	.L29
.L45:
	movl	-16(%ebp), %esi
	leal	4(%ebx), %ecx
	movl	108(%esi), %edx
	movl	%ecx, 108(%esi)
	movl	%eax, 4(%ebx)
	movl	%edx, 8(%ebx)
	movl	%ecx, (%edx)
.L37:
	movl	-16(%ebp), %eax
	movl	8(%eax), %ecx
	movl	$1, %eax
	sall	%cl, %eax
	call	alloc_disk
	testl	%eax, %eax
	movl	%eax, %esi
	je	.L55
	movl	-16(%ebp), %edx
	movl	4(%edx), %eax
	movl	%eax, (%esi)
	movl	8(%edx), %ecx
	movl	16(%ebx), %eax
	movl	$255, 8(%esi)
	movl	$nand_blktrans_ops, 516(%esi)
	sall	%cl, %eax
	movl	%eax, 4(%esi)
	movl	16(%ebx), %eax
	movl	%eax, 16(%esp)
	movl	(%edx), %eax
	movl	$.LC0, 8(%esp)
	movl	$32, 4(%esp)
	movl	%eax, 12(%esp)
	leal	12(%esi), %eax
	movl	%eax, (%esp)
	call	snprintf
	movl	20(%ebx), %eax
	movl	-16(%ebp), %edx
	movl	$0, 72(%esi)
	movl	%ebx, 524(%esi)
	movl	%eax, 68(%esi)
	movl	%esi, 40(%ebx)
	movl	72(%edx), %eax
	movl	%eax, 520(%esi)
	movl	$1, 912(%eax)
	movl	$64, 528(%esi)
	movl	116(%edx), %eax
	movl	-20(%ebp), %edx
	movl	%eax, 532(%esi)
	movzbl	40(%edx), %eax
	cmpb	$8, %al
	je	.L56
.L40:
	cmpb	$2, %al
	je	.L41
	movl	28(%ebx), %edx
.L42:
	cmpb	$4, %al
	je	.L57
.L43:
	testl	%edx, %edx
	.p2align 4,,2
	jne	.L58
.L44:
	movl	%esi, %eax
	xorl	%esi, %esi
	call	add_disk
.L29:
	addl	$28, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L54:
	movb	$0, (%edi)
	jmp	.L33
	.p2align 4,,7
	.p2align 3
.L53:
	movw	$0, (%edi)
	addl	$2, %edi
	andl	$1, %edx
	je	.L33
	jmp	.L54
	.p2align 4,,7
	.p2align 3
.L58:
	movl	$1, %edx
	movl	%esi, %eax
	call	set_disk_ro
	jmp	.L44
	.p2align 4,,7
	.p2align 3
.L57:
	movl	$1, 32(%ebx)
	jmp	.L43
	.p2align 4,,7
	.p2align 3
.L41:
	movl	-20(%ebp), %edx
	movl	$1, 28(%ebx)
	movzbl	40(%edx), %eax
	movl	$1, %edx
	jmp	.L42
	.p2align 4,,7
	.p2align 3
.L56:
	movl	$1, 36(%ebx)
	movzbl	40(%edx), %eax
	jmp	.L40
	.p2align 4,,7
	.p2align 3
.L35:
	movl	%esi, 16(%ebx)
	movl	4(%edx), %eax
	leal	4(%ebx), %ecx
	movl	%ecx, 4(%edx)
	movl	%edx, 4(%ebx)
	movl	%eax, 8(%ebx)
	movl	%ecx, (%eax)
	jmp	.L37
.L52:
	movw	$0, (%edi)
	subl	$2, %edx
	addl	$2, %edi
	jmp	.L31
.L51:
	movb	$0, (%eax)
	leal	1(%eax), %edi
	movb	$43, %dl
	jmp	.L30
.L34:
	movl	$0, 16(%ebx)
	jmp	.L45
.L55:
	movl	4(%ebx), %edx
	movl	$-12, %esi
	movl	8(%ebx), %eax
	movl	%eax, 4(%edx)
	movl	%edx, (%eax)
	movl	$1048832, 4(%ebx)
	movl	$2097664, 8(%ebx)
	jmp	.L29
	.size	nand_add_dev, .-nand_add_dev
	.p2align 4,,15
	.type	nand_proc_open, @function
nand_proc_open:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%edx, %ebx
	call	PDE_DATA
	movl	$nand_proc_show, %edx
	movl	%eax, %ecx
	movl	%ebx, %eax
	call	single_open
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_proc_open, .-nand_proc_open
	.p2align 4,,15
	.type	nand_blk_request, @function
nand_blk_request:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	subl	$4, %esp
	movl	552(%eax), %eax
	movl	52(%eax), %edx
	testl	%edx, %edx
	jne	.L65
	jmp	.L68
	.p2align 4,,7
	.p2align 3
.L63:
	movl	$-19, %edx
	call	__blk_end_request_all
.L65:
	movl	%ebx, %eax
	call	blk_fetch_request
	testl	%eax, %eax
	jne	.L63
	addl	$4, %esp
	popl	%ebx
	popl	%ebp
	ret
.L68:
	addl	$60, %eax
	movl	$1, %ecx
	movl	$3, %edx
	movl	$0, (%esp)
	movl	$1, rk_ftl_gc_do
	call	__wake_up
	addl	$4, %esp
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_blk_request, .-nand_blk_request
	.p2align 4,,15
	.type	rk_ftl_gc_timeout_hack, @function
rk_ftl_gc_timeout_hack:
	pushl	%ebp
	movl	$rk_ftl_gc_timeout, %eax
	movl	%esp, %ebp
	call	del_timer
	movl	rk_ftl_gc_do, %eax
	movl	jiffies, %edx
	addl	$1, %eax
	movl	%eax, rk_ftl_gc_do
	imull	rk_ftl_gc_jiffies, %eax
	addl	%edx, %eax
	movl	%eax, rk_ftl_gc_timeout+8
	movl	$rk_ftl_gc_timeout, %eax
	call	add_timer
	popl	%ebp
	ret
	.size	rk_ftl_gc_timeout_hack, .-rk_ftl_gc_timeout_hack
	.section	.rodata.str1.4,"aMS",@progbits,1
	.align 4
.LC1:
	.string	"include/asm-generic/dma-mapping-common.h"
	.section	.text.unlikely,"ax",@progbits
	.type	dma_unmap_single_attrs.part.11, @function
dma_unmap_single_attrs.part.11:
	pushl	%ebp
	movl	%esp, %ebp
#APP
# 36 "include/asm-generic/dma-mapping-common.h" 1
	1:	ud2
.pushsection __bug_table,"a"
2:	.long 1b, .LC1
	.word 36, 0
	.org 2b+12
.popsection
# 0 "" 2
#NO_APP
	.size	dma_unmap_single_attrs.part.11, .-dma_unmap_single_attrs.part.11
	.text
	.p2align 4,,15
	.globl	FlashRsvdBlkChk
	.type	FlashRsvdBlkChk, @function
FlashRsvdBlkChk:
	movzbl	gNandIDBResBlkNum, %ecx
	imull	gBlockPageAlignSize, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	cmpl	%edx, %ecx
	seta	%dl
	testb	%al, %al
	sete	%al
	andl	%eax, %edx
	xorl	$1, %edx
	movzbl	%dl, %eax
	ret
	.size	FlashRsvdBlkChk, .-FlashRsvdBlkChk
	.p2align 4,,15
	.globl	FlashGetRandomizer
	.type	FlashGetRandomizer, @function
FlashGetRandomizer:
	movl	%edx, %ecx
	andl	$127, %ecx
	cmpb	$0, gNandRandomizer
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	movzwl	random_seed(%ecx,%ecx), %eax
	je	.L73
	movzbl	gNandIDBResBlkNum, %ecx
	imull	gBlockPageAlignSize, %ecx
	cmpl	%ecx, %edx
	jb	.L75
.L74:
	orl	$-1073741824, %eax
.L73:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L75:
	testb	%bl, %bl
	jne	.L74
	popl	%ebx
	popl	%ebp
	.p2align 4,,1
	ret
	.size	FlashGetRandomizer, .-FlashGetRandomizer
	.p2align 4,,15
	.globl	FlashSetRandomizer
	.type	FlashSetRandomizer, @function
FlashSetRandomizer:
	movl	gNandcVer, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	cmpl	$5, %ecx
	jbe	.L77
	movl	%edx, %ecx
	andl	$127, %ecx
	cmpb	$0, gNandRandomizer
	movzwl	random_seed(%ecx,%ecx), %ecx
	je	.L81
	movzbl	gNandIDBResBlkNum, %ebx
	imull	gBlockPageAlignSize, %ebx
	cmpl	%ebx, %edx
	jae	.L79
	testb	%al, %al
	je	.L81
.L79:
	orl	$-1073741824, %ecx
.L81:
	movl	gpNandc, %eax
	movl	%ecx, 336(%eax)
.L76:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L77:
	cmpl	$4, %ecx
	jne	.L76
	movl	%edx, %ecx
	movzbl	gNandIDBResBlkNum, %ebx
	andl	$127, %ecx
	movzwl	random_seed(%ecx,%ecx), %ecx
	imull	gBlockPageAlignSize, %ebx
	sall	$8, %ecx
	cmpl	%ebx, %edx
	jae	.L83
	testb	%al, %al
	je	.L81
.L83:
	cmpb	$0, gNandRandomizer
	je	.L81
	addl	%edx, %edx
	andl	$255, %edx
	orl	$1, %edx
	orl	%edx, %ecx
	jmp	.L81
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.p2align 4,,15
	.globl	FlashReadCmd
	.type	FlashReadCmd, @function
FlashReadCmd:
	movzbl	%al, %eax
	leal	8(%eax), %ecx
	sall	$8, %ecx
	addl	gpNandc, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movzbl	%dl, %ebx
	movl	$0, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$8, %ebx
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$16, %ebx
	movl	%ebx, 4(%ecx)
	movl	$48, 8(%ecx)
	call	FlashSetRandomizer
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashReadCmd, .-FlashReadCmd
	.p2align 4,,15
	.globl	FlashReadDpDataOutCmd
	.type	FlashReadDpDataOutCmd, @function
FlashReadDpDataOutCmd:
	movzbl	%al, %eax
	leal	8(%eax), %ecx
	sall	$8, %ecx
	addl	gpNandc, %ecx
	cmpb	$1, gNandOptPara+16
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	je	.L89
	movzbl	%dl, %ebx
	movl	$0, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$8, %ebx
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$16, %ebx
	movl	%ebx, 4(%ecx)
	movl	$5, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	$224, 8(%ecx)
	call	FlashSetRandomizer
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L89:
	movzbl	%dl, %ebx
	movl	$6, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$8, %ebx
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$16, %ebx
	movl	%ebx, 4(%ecx)
	movl	$224, 8(%ecx)
	call	FlashSetRandomizer
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.p2align 4,,15
	.globl	FlashProgFirstCmd
	.type	FlashProgFirstCmd, @function
FlashProgFirstCmd:
	movzbl	%al, %eax
	leal	8(%eax), %ecx
	sall	$8, %ecx
	addl	gpNandc, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movzbl	%dl, %ebx
	movl	$128, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$8, %ebx
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$16, %ebx
	movl	%ebx, 4(%ecx)
	call	FlashSetRandomizer
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashProgFirstCmd, .-FlashProgFirstCmd
	.p2align 4,,15
	.globl	FlashEraseCmd
	.type	FlashEraseCmd, @function
FlashEraseCmd:
	movzbl	%al, %eax
	addl	$8, %eax
	sall	$8, %eax
	pushl	%ebp
	addl	gpNandc, %eax
	testl	%ecx, %ecx
	movl	%esp, %ebp
	je	.L92
	movzbl	%dl, %ecx
	movl	$96, 8(%eax)
	movl	%ecx, 4(%eax)
	movl	%edx, %ecx
	shrl	$8, %ecx
	movl	%ecx, 4(%eax)
	movl	%edx, %ecx
	addl	gBlockPageAlignSize, %edx
	shrl	$16, %ecx
	movl	%ecx, 4(%eax)
.L92:
	movzbl	%dl, %ecx
	movl	$96, 8(%eax)
	movl	%ecx, 4(%eax)
	movl	%edx, %ecx
	shrl	$8, %ecx
	shrl	$16, %edx
	movl	%ecx, 4(%eax)
	movl	%edx, 4(%eax)
	movl	$208, 8(%eax)
	popl	%ebp
	ret
	.size	FlashEraseCmd, .-FlashEraseCmd
	.p2align 4,,15
	.globl	FlashProgDpSecondCmd
	.type	FlashProgDpSecondCmd, @function
FlashProgDpSecondCmd:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	leal	8(%eax), %ecx
	pushl	%ebx
	movzbl	gNandOptPara+11, %ebx
	sall	$8, %ecx
	addl	gpNandc, %ecx
	movl	%ebx, 8(%ecx)
	movzbl	%dl, %ebx
	movl	$0, 4(%ecx)
	movl	$0, 4(%ecx)
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$8, %ebx
	movl	%ebx, 4(%ecx)
	movl	%edx, %ebx
	shrl	$16, %ebx
	movl	%ebx, 4(%ecx)
	call	FlashSetRandomizer
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashProgDpSecondCmd, .-FlashProgDpSecondCmd
	.p2align 4,,15
	.globl	FlashProgSecondCmd
	.type	FlashProgSecondCmd, @function
FlashProgSecondCmd:
	movzbl	%al, %eax
	addl	$8, %eax
	sall	$8, %eax
	addl	gpNandc, %eax
	pushl	%ebp
	movl	%esp, %ebp
	movl	$16, 8(%eax)
	popl	%ebp
	ret
	.size	FlashProgSecondCmd, .-FlashProgSecondCmd
	.p2align 4,,15
	.globl	FlashProgDpFirstCmd
	.type	FlashProgDpFirstCmd, @function
FlashProgDpFirstCmd:
	movzbl	%al, %eax
	movzbl	gNandOptPara+10, %edx
	addl	$8, %eax
	sall	$8, %eax
	addl	gpNandc, %eax
	pushl	%ebp
	movl	%esp, %ebp
	movl	%edx, 8(%eax)
	popl	%ebp
	ret
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.p2align 4,,15
	.globl	FlashReadStatus
	.type	FlashReadStatus, @function
FlashReadStatus:
	movzbl	%al, %eax
	pushl	%ebp
	addl	$8, %eax
	movl	%esp, %ebp
	subl	$4, %esp
	sall	$8, %eax
	addl	gpNandc, %eax
	movl	$112, 8(%eax)
	movl	$800, -4(%ebp)
	.p2align 4,,7
	.p2align 3
.L97:
	movl	-4(%ebp), %edx
	leal	-1(%edx), %ecx
	testl	%edx, %edx
	movl	%ecx, -4(%ebp)
	jne	.L97
	movl	(%eax), %eax
	leave
	ret
	.size	FlashReadStatus, .-FlashReadStatus
	.p2align 4,,15
	.globl	JSHash
	.type	JSHash, @function
JSHash:
	pushl	%ebp
	testl	%edx, %edx
	movl	%esp, %ebp
	movl	$1204201446, %ecx
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	je	.L100
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L101:
	movl	%ecx, %edi
	movl	%ecx, %esi
	shrl	$2, %edi
	sall	$5, %esi
	addl	%edi, %esi
	movzbl	(%eax,%ebx), %edi
	addl	$1, %ebx
	addl	%edi, %esi
	xorl	%esi, %ecx
	cmpl	%edx, %ebx
	jne	.L101
.L100:
	popl	%ebx
	movl	%ecx, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	JSHash, .-JSHash
	.p2align 4,,15
	.globl	FlashLoadIdbInfo
	.type	FlashLoadIdbInfo, @function
FlashLoadIdbInfo:
	pushl	%ebp
	xorl	%eax, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FlashLoadIdbInfo, .-FlashLoadIdbInfo
	.p2align 4,,15
	.globl	BuildFlashLsbPageTable
	.type	BuildFlashLsbPageTable, @function
BuildFlashLsbPageTable:
	pushl	%ebp
	testl	%eax, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%edx, %ebx
	je	.L107
	cmpl	$1, %eax
	je	.L146
	cmpl	$2, %eax
	je	.L147
	cmpl	$3, %eax
	.p2align 4,,2
	je	.L148
	cmpl	$4, %eax
	.p2align 4,,2
	je	.L149
	cmpl	$6, %eax
	.p2align 4,,2
	je	.L150
.L108:
	movl	$mlcPageToSlcPageTbl, %edx
	movl	$256, %ecx
	movl	$-1, %eax
	movl	%edx, %edi
	testw	%bx, %bx
	rep stosl
	je	.L105
	leal	-1(%ebx), %edx
	movl	$slcPageToMlcPageTbl, %eax
	movzwl	%dx, %edx
	leal	slcPageToMlcPageTbl+2(%edx,%edx), %ebx
	.p2align 4,,7
	.p2align 3
.L132:
	movzwl	(%eax), %edx
	addl	$2, %eax
	cmpl	%ebx, %eax
	movzwl	%dx, %ecx
	movw	%dx, mlcPageToSlcPageTbl(%ecx,%ecx)
	jne	.L132
.L105:
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L107:
	movw	%ax, slcPageToMlcPageTbl(%eax,%eax)
	addl	$1, %eax
	cmpl	$256, %eax
	jne	.L107
	jmp	.L108
.L146:
	movl	$1, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	jmp	.L109
	.p2align 4,,7
	.p2align 3
.L113:
	cmpw	$3, %dx
	movl	%eax, %ecx
	jbe	.L111
	movl	%eax, %edi
	addl	%edx, %edx
	andl	$1, %edi
	cmpl	$1, %edi
	sbbl	%edi, %edi
	addl	$3, %edi
	subw	%di, %dx
.L111:
	addl	$1, %eax
.L109:
	cmpl	$256, %eax
	movw	%dx, slcPageToMlcPageTbl(%ecx,%ecx)
	movl	%eax, %edx
	jne	.L113
	jmp	.L108
.L148:
	movl	$1, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	jmp	.L118
	.p2align 4,,7
	.p2align 3
.L122:
	cmpw	$5, %dx
	movl	%eax, %ecx
	jbe	.L120
	movl	%eax, %edi
	addl	%edx, %edx
	andl	$1, %edi
	cmpl	$1, %edi
	sbbl	%edi, %edi
	addl	$5, %edi
	subw	%di, %dx
.L120:
	addl	$1, %eax
.L118:
	cmpl	$256, %eax
	movw	%dx, slcPageToMlcPageTbl(%ecx,%ecx)
	movl	%eax, %edx
	jne	.L122
	jmp	.L108
.L147:
	movl	$1, %ecx
	movl	$1, %edx
	xorl	%edi, %edi
	xorl	%eax, %eax
	jmp	.L114
	.p2align 4,,7
	.p2align 3
.L117:
	cmpw	$1, %ax
	movl	%edx, %edi
	cmovne	%ecx, %eax
	addl	$1, %edx
	addl	$2, %ecx
.L114:
	cmpl	$256, %edx
	movw	%ax, slcPageToMlcPageTbl(%edi,%edi)
	movl	%edx, %eax
	jne	.L117
	jmp	.L108
.L149:
	movw	$0, slcPageToMlcPageTbl
	movb	$9, %al
	movl	$8, %edi
	movw	$1, slcPageToMlcPageTbl+2
	movl	$16, %ecx
	movl	$6, %edx
	movw	$2, slcPageToMlcPageTbl+4
	movw	$3, slcPageToMlcPageTbl+6
	movw	$4, slcPageToMlcPageTbl+8
	movw	$5, slcPageToMlcPageTbl+10
	movw	$7, slcPageToMlcPageTbl+12
	movw	$8, slcPageToMlcPageTbl+14
	jmp	.L124
.L126:
	movl	%eax, %edx
	movl	%eax, %edi
	andl	$1, %edx
	cmpl	$1, %edx
	sbbl	%edx, %edx
	leal	(%eax,%eax), %ecx
	addl	$7, %edx
	addl	$1, %eax
.L124:
	subw	%dx, %cx
	cmpl	$256, %eax
	movw	%cx, slcPageToMlcPageTbl(%edi,%edi)
	jne	.L126
	jmp	.L108
.L150:
	movl	$3, %ecx
	movl	$1, %eax
	xorl	%edi, %edi
	xorl	%edx, %edx
	jmp	.L127
	.p2align 4,,7
	.p2align 3
.L130:
	cmpw	$5, %dx
	movl	%eax, %edi
	jbe	.L128
	movl	%eax, %edx
	movl	%ecx, %esi
	andl	$1, %edx
	cmpl	$1, %edx
	sbbl	%edx, %edx
	andl	$-2, %edx
	addl	$12, %edx
	subw	%dx, %si
	movl	%esi, %edx
.L128:
	addl	$1, %eax
	addl	$3, %ecx
.L127:
	cmpl	$256, %eax
	movw	%dx, slcPageToMlcPageTbl(%edi,%edi)
	movl	%eax, %edx
	jne	.L130
	jmp	.L108
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
	.p2align 4,,15
	.globl	FlashPrintInfo
	.type	FlashPrintInfo, @function
FlashPrintInfo:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FlashPrintInfo, .-FlashPrintInfo
	.p2align 4,,15
	.globl	ToshibaSetRRPara
	.type	ToshibaSetRRPara, @function
ToshibaSetRRPara:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	cmpb	$0, g_maxRegNum
	je	.L152
	movzbl	%dl, %esi
	xorl	%ebx, %ebx
	leal	(%esi,%esi,4), %edi
	.p2align 4,,7
	.p2align 3
.L158:
	movl	$85, 8(%eax)
	movsbl	ToshibaA19RefValue(%ebx), %edx
	movl	%edx, 4(%eax)
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L154:
	movl	-16(%ebp), %ecx
	leal	-1(%ecx), %edx
	testl	%ecx, %ecx
	movl	%edx, -16(%ebp)
	jne	.L154
	movzbl	g_retryMode, %edx
	cmpb	$34, %dl
	je	.L161
	cmpb	$35, %dl
	je	.L162
	movsbl	ToshibaRefValue(%esi), %edx
	movl	%edx, (%eax)
.L156:
	movzbl	g_maxRegNum, %edx
	addl	$1, %ebx
	cmpl	%ebx, %edx
	ja	.L158
.L152:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L161:
	movsbl	ToshibaA19RefValue+5(%edi,%ebx), %edx
	movl	%edx, (%eax)
	jmp	.L156
	.p2align 4,,7
	.p2align 3
.L162:
	movsbl	Toshiba15RefValue+5(%edi,%ebx), %edx
	movl	%edx, (%eax)
	jmp	.L156
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.p2align 4,,15
	.globl	SamsungSetRRPara
	.type	SamsungSetRRPara, @function
SamsungSetRRPara:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	cmpb	$0, g_maxRegNum
	je	.L163
	movzbl	%dl, %edx
	xorl	%ebx, %ebx
	leal	1(%edx), %esi
.L166:
	movl	$161, 8(%eax)
	movl	$0, (%eax)
	movsbl	SamsungRefValue(%ebx), %edx
	movl	%edx, (%eax)
	movsbl	SamsungRefValue(%ebx,%esi,4), %edx
	movl	%edx, (%eax)
	movl	$3000, -12(%ebp)
	.p2align 4,,7
	.p2align 3
.L165:
	movl	-12(%ebp), %edx
	leal	-1(%edx), %ecx
	testl	%edx, %edx
	movl	%ecx, -12(%ebp)
	jne	.L165
	movzbl	g_maxRegNum, %edx
	addl	$1, %ebx
	cmpl	%ebx, %edx
	ja	.L166
.L163:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.p2align 4,,15
	.globl	HynixSetRRPara
	.type	HynixSetRRPara, @function
HynixSetRRPara:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	$1, %esi
	pushl	%ebx
	movl	%ecx, %ebx
	subl	$12, %esp
	movl	%eax, %ecx
	movl	gpNandc, %edi
	movzbl	8(%ebp), %eax
	sall	%cl, %esi
	movb	%dl, -17(%ebp)
	movzbl	%cl, %edx
	movl	%esi, %ecx
	movl	%edx, -24(%ebp)
	addl	$8, %edx
	sall	$8, %edx
	movb	%al, -18(%ebp)
	movl	(%edi), %eax
	addl	%edi, %edx
	cmpb	$0, -17(%ebp)
	movb	%cl, %al
	movl	%eax, (%edi)
	movl	$54, 8(%edx)
	je	.L170
	movzbl	-18(%ebp), %eax
	xorl	%esi, %esi
	movl	-24(%ebp), %ecx
	leal	(%eax,%ecx,8), %edi
	movl	%ebx, %eax
	sall	$3, %edi
	movl	%esi, %ebx
	movl	%eax, %esi
	.p2align 4,,7
	.p2align 3
.L172:
	movzbl	(%esi,%ebx), %eax
	movl	%eax, 4(%edx)
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L171:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %ecx
	testl	%eax, %eax
	movl	%ecx, -16(%ebp)
	jne	.L171
	movsbl	gReadRetryInfo+20(%edi,%ebx), %eax
	addl	$1, %ebx
	cmpb	%bl, -17(%ebp)
	movl	%eax, (%edx)
	ja	.L172
.L170:
	movl	$22, 8(%edx)
	movl	gpNandc, %edx
	movl	(%edx), %eax
	andl	$-131328, %eax
	movl	%eax, (%edx)
	movzbl	-18(%ebp), %eax
	movl	-24(%ebp), %edx
	movb	%al, read_retry_cur_offset(%edx)
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	HynixSetRRPara, .-HynixSetRRPara
	.p2align 4,,15
	.globl	FlashSetReadRetryDefault
	.type	FlashSetReadRetryDefault, @function
FlashSetReadRetryDefault:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	gpNandParaInfo, %eax
	movzbl	19(%eax), %eax
	subl	$1, %eax
	cmpb	$4, %al
	ja	.L175
	cmpb	$-83, IDByte
	je	.L178
.L177:
	cmpb	$-83, IDByte+8
	je	.L179
.L175:
	leave
	ret
	.p2align 4,,7
	.p2align 3
.L179:
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	movl	$1, %eax
	movl	$0, (%esp)
	call	HynixSetRRPara
	leave
	ret
	.p2align 4,,7
	.p2align 3
.L178:
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	xorl	%eax, %eax
	movl	$0, (%esp)
	call	HynixSetRRPara
	jmp	.L177
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.p2align 4,,15
	.globl	FlashDieInfoInit
	.type	FlashDieInfoInit, @function
FlashDieInfoInit:
	pushl	%ebp
	movl	$gDieOp, %edx
	movl	%esp, %ebp
	movl	$16, %ecx
	pushl	%edi
	movl	%edx, %edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	movzwl	gNandParaInfo+10, %eax
	movl	gpNandParaInfo, %esi
	movb	$0, gNandMaxDie
	movb	$0, gNandMaxChip
	movl	%eax, gBlockPageAlignSize
	movl	%eax, -24(%ebp)
	xorl	%eax, %eax
	rep stosl
	leal	1(%esi), %edx
	xorl	%edi, %edi
	movl	$0, DieCsIndex
	movl	$0, DieAddrs
	movl	$0, DieAddrs+4
	movl	$0, DieAddrs+8
	movl	$0, DieAddrs+12
	movzbl	(%esi), %ebx
	movl	%edx, -16(%ebp)
	movl	-16(%ebp), %eax
	movl	$IDByte, %edx
	movl	%ebx, %ecx
	call	memcmp
	testl	%eax, %eax
	jne	.L181
	movb	$1, gNandMaxDie
	movzbl	(%esi), %ebx
	movl	$1, %edi
.L181:
	movl	-16(%ebp), %eax
	movl	%ebx, %ecx
	movl	$IDByte+8, %edx
	call	memcmp
	testl	%eax, %eax
	je	.L189
	cmpb	$2, 8(%esi)
	movl	%edi, %edx
	movb	%dl, gNandMaxChip
	je	.L190
.L183:
	movzbl	13(%esi), %eax
	andw	$255, %di
	imulw	14(%esi), %ax
	imull	%edi, %eax
	movw	%ax, gTotleBlock
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L189:
	movl	%edi, %edx
	addl	$1, %edi
	movzbl	%dl, %eax
	movl	%edi, %edx
	movb	$1, DieCsIndex(%eax)
	movl	%edi, %eax
	cmpb	$2, 8(%esi)
	movb	%al, gNandMaxDie
	movb	%dl, gNandMaxChip
	jne	.L183
.L190:
	xorl	%ebx, %ebx
	movl	%ebx, %eax
	movl	%edi, %ebx
	movl	$IDByte, -20(%ebp)
	movl	%eax, %edi
.L186:
	movzbl	(%esi), %ecx
	movl	-20(%ebp), %edx
	movl	-16(%ebp), %eax
	call	memcmp
	testl	%eax, %eax
	jne	.L184
	movzbl	13(%esi), %eax
	movzbl	%bl, %ecx
	movzwl	14(%esi), %edx
	imull	-24(%ebp), %eax
	andl	$65280, %edx
	imull	%edx, %eax
	movl	%eax, DieAddrs(,%ecx,4)
	cmpb	$0, 23(%esi)
	jne	.L191
.L185:
	movl	%edi, %eax
	addl	$1, %ebx
	movb	%al, DieCsIndex(%ecx)
	movb	%bl, gNandMaxDie
.L184:
	addl	$1, %edi
	addl	$8, -20(%ebp)
	cmpl	$2, %edi
	jne	.L186
	movl	%ebx, %edi
	jmp	.L183
	.p2align 4,,7
	.p2align 3
.L191:
	movzbl	gNandMaxDie, %eax
	sall	DieAddrs(,%eax,4)
	jmp	.L185
	.size	FlashDieInfoInit, .-FlashDieInfoInit
	.p2align 4,,15
	.globl	FlashSuspend
	.type	FlashSuspend, @function
FlashSuspend:
	movl	gpNandc, %eax
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	movl	(%eax), %edx
	movl	%edx, NANDC_FMCTL
	movl	4(%eax), %edx
	movl	%edx, NANDC_FMWAIT
	movl	8(%eax), %edx
	movl	%edx, NANDC_FLCTL
	movl	12(%eax), %edx
	movl	%edx, NANDC_BCHCTL
	movl	304(%eax), %edx
	movl	%edx, NANDC_DLL_CTL_REG0
	movl	308(%eax), %edx
	movl	%edx, NANDC_DLL_CTL_REG1
	movl	336(%eax), %edx
	movl	344(%eax), %eax
	movl	%edx, NANDC_RANDMZ_CFG
	movl	%eax, NANDC_FMWAIT_SYN
	xorl	%eax, %eax
	ret
	.size	FlashSuspend, .-FlashSuspend
	.p2align 4,,15
	.globl	LogAddr2PhyAddr
	.type	LogAddr2PhyAddr, @function
LogAddr2PhyAddr:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$20, %esp
	movl	%ebx, -12(%ebp)
	movl	%eax, %ebx
	movzbl	12(%ebp), %eax
	movl	%edi, -4(%ebp)
	movl	4(%ebx), %edi
	movl	%esi, -8(%ebp)
	movl	%ecx, %esi
	movl	%edx, -16(%ebp)
	xorl	%edx, %edx
	movb	%al, -17(%ebp)
	movl	gpNandParaInfo, %eax
	shrl	$10, %edi
	movzbl	13(%eax), %ecx
	imulw	14(%eax), %cx
	movl	%edi, %eax
	divw	%cx
	imull	%eax, %ecx
	movzwl	%ax, %edx
	movzwl	4(%ebx), %eax
	subw	%cx, %di
	andw	$1023, %ax
	cmpl	$1, -16(%ebp)
	je	.L197
.L194:
	movzwl	%di, %edi
	movzwl	%ax, %eax
	imull	gBlockPageAlignSize, %edi
	addl	DieAddrs(,%edx,4), %edi
	addl	%eax, %edi
	movl	8(%ebp), %eax
	movl	%edi, (%esi)
	movl	%edx, (%eax)
	xorl	%eax, %eax
	cmpb	$1, -17(%ebp)
	jbe	.L195
	movl	4(%ebx), %eax
	addl	$1024, %eax
	cmpl	40(%ebx), %eax
	sete	%al
	movzbl	%al, %eax
.L195:
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L197:
	movzwl	%ax, %eax
	movzwl	slcPageToMlcPageTbl(%eax,%eax), %eax
	jmp	.L194
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.p2align 4,,15
	.globl	FlashReadStatusEN
	.type	FlashReadStatusEN, @function
FlashReadStatusEN:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	addl	$8, %eax
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	movl	gpNandParaInfo, %ebx
	sall	$8, %eax
	addl	gpNandc, %eax
	cmpb	$2, 8(%ebx)
	je	.L207
	movl	$112, 8(%eax)
.L202:
	movl	$800, -12(%ebp)
	.p2align 4,,7
	.p2align 3
.L204:
	movl	-12(%ebp), %edx
	leal	-1(%edx), %ecx
	testl	%edx, %edx
	movl	%ecx, -12(%ebp)
	jne	.L204
	movl	(%eax), %eax
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L207:
	testb	%cl, %cl
	je	.L208
	movzbl	gNandOptPara+14, %ecx
	movl	%ecx, 8(%eax)
.L201:
	cmpb	$0, gNandOptPara+15
	je	.L202
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L203:
	leal	0(,%ebx,8), %ecx
	movl	%edx, %esi
	shrl	%cl, %esi
	addl	$1, %ebx
	movl	%esi, %ecx
	andl	$255, %ecx
	movl	%ecx, 4(%eax)
	movzbl	gNandOptPara+15, %ecx
	cmpl	%ebx, %ecx
	ja	.L203
	jmp	.L202
	.p2align 4,,7
	.p2align 3
.L208:
	movzbl	gNandOptPara+13, %ecx
	movl	%ecx, 8(%eax)
	jmp	.L201
	.size	FlashReadStatusEN, .-FlashReadStatusEN
	.p2align 4,,15
	.globl	FlashWaitReadyEN
	.type	FlashWaitReadyEN, @function
FlashWaitReadyEN:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movzbl	%al, %edi
	pushl	%esi
	movzbl	%cl, %esi
	pushl	%ebx
	movl	%edx, %ebx
	.p2align 4,,7
	.p2align 3
.L210:
	movl	%esi, %ecx
	movl	%ebx, %edx
	movl	%edi, %eax
	call	FlashReadStatusEN
	testb	$64, %al
	je	.L210
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashWaitReadyEN, .-FlashWaitReadyEN
	.p2align 4,,15
	.globl	FlashWaitCmdDone
	.type	FlashWaitCmdDone, @function
FlashWaitCmdDone:
	movzbl	%al, %eax
	pushl	%ebp
	movl	%eax, %edx
	movl	%esp, %ebp
	subl	$12, %esp
	sall	$4, %edx
	movl	%edi, -4(%ebp)
	movl	gDieOp+8(%edx), %edi
	movl	%ebx, -12(%ebp)
	movzbl	gDieOp(%edx), %ecx
	leal	gDieOp(%edx), %ebx
	movl	%esi, -8(%ebp)
	testl	%edi, %edi
	je	.L213
	movl	gpNandc, %edx
	movzbl	%cl, %edi
	movl	(%edx), %esi
	movl	$1, %edx
	sall	%cl, %edx
	movl	%esi, %ecx
	movb	%dl, %cl
	movl	gpNandc, %edx
	movl	%ecx, %esi
	xorl	%ecx, %ecx
	movl	%esi, (%edx)
	movl	DieAddrs(,%eax,4), %esi
	movl	%edi, %eax
	movl	4(%ebx), %edx
	testl	%esi, %esi
	setne	%cl
	call	FlashWaitReadyEN
	movl	gpNandc, %ecx
	movl	(%ecx), %edx
	andl	$-131328, %edx
	andl	$1, %eax
	movl	%edx, (%ecx)
	movl	$-1, %edx
	cmovne	%edx, %eax
	movl	8(%ebx), %edx
	movl	%eax, (%edx)
	movl	12(%ebx), %edx
	movl	$0, 8(%ebx)
	testl	%edx, %edx
	je	.L213
	movl	%eax, (%edx)
	movl	$0, 12(%ebx)
.L213:
	movl	-12(%ebp), %ebx
	xorl	%eax, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.p2align 4,,15
	.globl	ReadFlashInfo
	.type	ReadFlashInfo, @function
ReadFlashInfo:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	movl	$0, 4(%eax)
	movw	$0, 8(%eax)
	movl	$0, (%eax)
	movb	$0, 10(%eax)
	movl	gpNandParaInfo, %ecx
	movzbl	gNandFlashEccBits, %edx
	movzbl	9(%ecx), %ebx
	imulw	gBlockPageAlignSize, %bx
	movb	%dl, 7(%eax)
	movw	%bx, 4(%eax)
	movzbl	13(%ecx), %edx
	movzwl	%bx, %ebx
	imull	%ebx, %edx
	movzwl	14(%ecx), %ebx
	movzbl	8(%ecx), %ecx
	imull	%ebx, %edx
	imull	%ecx, %edx
	movl	%edx, (%eax)
	movl	gpNandParaInfo, %edx
	movzbl	gNandMaxDie, %esi
	movzbl	9(%edx), %ecx
	movb	$32, 8(%eax)
	movb	%cl, 6(%eax)
	movzbl	7(%edx), %edx
	movb	%dl, 9(%eax)
	movl	%esi, %edx
	testb	%dl, %dl
	je	.L215
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L217:
	movzbl	DieCsIndex(%edx), %ecx
	movl	$1, %edi
	addl	$1, %edx
	sall	%cl, %edi
	movl	%esi, %ecx
	movl	%edi, -16(%ebp)
	orb	-16(%ebp), %bl
	cmpb	%dl, %cl
	ja	.L217
	movb	%bl, 10(%eax)
.L215:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	ReadFlashInfo, .-ReadFlashInfo
	.p2align 4,,15
	.globl	FlashScheduleEnSet
	.type	FlashScheduleEnSet, @function
FlashScheduleEnSet:
	movl	FlashWaitBusyScheduleEn, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	%eax, FlashWaitBusyScheduleEn
	popl	%ebp
	movl	%edx, %eax
	ret
	.size	FlashScheduleEnSet, .-FlashScheduleEnSet
	.p2align 4,,15
	.globl	FlashGetPageSize
	.type	FlashGetPageSize, @function
FlashGetPageSize:
	movl	gpNandParaInfo, %eax
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	movzbl	9(%eax), %eax
	ret
	.size	FlashGetPageSize, .-FlashGetPageSize
	.p2align 4,,15
	.globl	NandcReadDontCaseBusyEn
	.type	NandcReadDontCaseBusyEn, @function
NandcReadDontCaseBusyEn:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	NandcReadDontCaseBusyEn, .-NandcReadDontCaseBusyEn
	.p2align 4,,15
	.globl	NandcGetChipIf
	.type	NandcGetChipIf, @function
NandcGetChipIf:
	movzbl	%al, %eax
	pushl	%ebp
	addl	$8, %eax
	movl	%esp, %ebp
	sall	$8, %eax
	popl	%ebp
	addl	gpNandc, %eax
	ret
	.size	NandcGetChipIf, .-NandcGetChipIf
	.p2align 4,,15
	.globl	NandcSetDdrPara
	.type	NandcSetDdrPara, @function
NandcSetDdrPara:
	movzbl	%al, %eax
	movl	%eax, %edx
	sall	$16, %edx
	sall	$8, %eax
	orl	%eax, %edx
	movl	gpNandc, %eax
	pushl	%ebp
	orl	$1, %edx
	movl	%esp, %ebp
	movl	%edx, 304(%eax)
	popl	%ebp
	ret
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.p2align 4,,15
	.globl	NandcSetDdrMode
	.type	NandcSetDdrMode, @function
NandcSetDdrMode:
	movl	gpNandc, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	(%ecx), %edx
	andb	$223, %dh
	movl	%edx, %ebx
	orl	$253952, %ebx
	testl	%eax, %eax
	cmovne	%ebx, %edx
	movl	%edx, (%ecx)
	popl	%ebx
	popl	%ebp
	ret
	.size	NandcSetDdrMode, .-NandcSetDdrMode
	.p2align 4,,15
	.globl	NandcSetMode
	.type	NandcSetMode, @function
NandcSetMode:
	movl	gpNandc, %edx
	movzbl	%al, %eax
	pushl	%ebp
	testb	$6, %al
	movl	%esp, %ebp
	pushl	%ebx
	movl	(%edx), %ecx
	je	.L227
	movl	%ecx, %ebx
	orb	$96, %bh
	andb	$127, %bh
	movl	%ebx, %ecx
	orl	$196608, %ecx
	testb	$4, %al
	je	.L228
	movl	%ebx, %ecx
	orl	$229376, %ecx
.L228:
	movl	gp_nandc_info, %eax
	cmpl	$200, 8(%eax)
	jg	.L232
	movl	$16641, 344(%edx)
.L230:
	movl	$1973763, 304(%edx)
	xorl	%eax, %eax
	movl	$39, 308(%edx)
	movl	%ecx, (%edx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L227:
	andb	$223, %ch
	xorl	%eax, %eax
	movl	%ecx, (%edx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L232:
	movl	$8322, 344(%edx)
	jmp	.L230
	.size	NandcSetMode, .-NandcSetMode
	.p2align 4,,15
	.globl	NandcFlashCsDebug
	.type	NandcFlashCsDebug, @function
NandcFlashCsDebug:
	movl	gpNandc, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	(%edx), %ecx
	movb	%al, %cl
	movl	%ecx, (%edx)
	popl	%ebp
	ret
	.size	NandcFlashCsDebug, .-NandcFlashCsDebug
	.p2align 4,,15
	.globl	NandcFlashCs
	.type	NandcFlashCs, @function
NandcFlashCs:
	pushl	%ebp
	movl	%eax, %ecx
	movl	%esp, %ebp
	movl	$1, %edx
	pushl	%ebx
	movl	gpNandc, %ebx
	sall	%cl, %edx
	movl	(%ebx), %eax
	movb	%dl, %al
	movl	%eax, (%ebx)
	popl	%ebx
	popl	%ebp
	ret
	.size	NandcFlashCs, .-NandcFlashCs
	.p2align 4,,15
	.globl	NandcFlashDeCs
	.type	NandcFlashDeCs, @function
NandcFlashDeCs:
	movl	gpNandc, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	(%edx), %eax
	andl	$-131328, %eax
	movl	%eax, (%edx)
	popl	%ebp
	ret
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.p2align 4,,15
	.globl	NandcDelayns
	.type	NandcDelayns, @function
NandcDelayns:
	pushl	%ebp
	leal	(%eax,%eax,4), %eax
	movl	%esp, %ebp
	addl	%eax, %eax
	subl	$4, %esp
	movl	%eax, -4(%ebp)
	.p2align 4,,7
	.p2align 3
.L237:
	movl	-4(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -4(%ebp)
	jne	.L237
	leave
	ret
	.size	NandcDelayns, .-NandcDelayns
	.p2align 4,,15
	.globl	NandcWaitFlashReady
	.type	NandcWaitFlashReady, @function
NandcWaitFlashReady:
	pushl	%ebp
	movl	$100000, %ecx
	movl	%esp, %ebp
	pushl	%ebx
	subl	$8, %esp
	movl	gpNandc, %ebx
	.p2align 4,,7
	.p2align 3
.L242:
	movl	$1000, -8(%ebp)
	.p2align 4,,7
	.p2align 3
.L240:
	movl	-8(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -8(%ebp)
	jne	.L240
	movl	(%ebx), %eax
	movl	%eax, -12(%ebp)
	movl	-12(%ebp), %eax
	testb	$2, %ah
	jne	.L243
	subl	$1, %ecx
	jne	.L242
	addl	$8, %esp
	movl	$-1, %eax
	popl	%ebx
	popl	%ebp
	ret
.L243:
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
	.p2align 4,,15
	.globl	SandiskSetRRPara
	.type	SandiskSetRRPara, @function
SandiskSetRRPara:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$4, %esp
	movl	$239, 8(%eax)
	movl	$17, 4(%eax)
	movl	$2000, -8(%ebp)
	.p2align 4,,7
	.p2align 3
.L247:
	movl	-8(%ebp), %ecx
	leal	-1(%ecx), %ebx
	testl	%ecx, %ecx
	movl	%ebx, -8(%ebp)
	jne	.L247
	cmpb	$0, g_maxRegNum
	je	.L248
	movzbl	%dl, %edx
	leal	(%edx,%edx,4), %ebx
	jmp	.L251
	.p2align 4,,7
	.p2align 3
.L249:
	movsbl	Toshiba15RefValue+5(%ebx,%ecx), %edx
	addl	$1, %ecx
	movl	%edx, (%eax)
	movzbl	g_maxRegNum, %edx
	cmpl	%ecx, %edx
	jbe	.L248
.L251:
	cmpb	$67, g_retryMode
	jne	.L249
	movsbl	ToshibaA19RefValue+5(%ebx,%ecx), %edx
	addl	$1, %ecx
	movl	%edx, (%eax)
	movzbl	g_maxRegNum, %edx
	cmpl	%ecx, %edx
	ja	.L251
.L248:
	xorl	%eax, %eax
	call	NandcWaitFlashReady
	addl	$4, %esp
	popl	%ebx
	popl	%ebp
	ret
	.size	SandiskSetRRPara, .-SandiskSetRRPara
	.p2align 4,,15
	.globl	SandiskProgTestBadBlock
	.type	SandiskProgTestBadBlock, @function
SandiskProgTestBadBlock:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	movzbl	%dl, %ecx
	pushl	%ebx
	leal	8(%eax), %ebx
	subl	$4, %esp
	sall	$8, %ebx
	addl	gpNandc, %ebx
	movl	$162, 8(%ebx)
	movl	$128, 8(%ebx)
	movl	$0, 4(%ebx)
	movl	$0, 4(%ebx)
	movl	%ecx, 4(%ebx)
	movl	%edx, %ecx
	shrl	$8, %ecx
	shrl	$16, %edx
	movl	%ecx, 4(%ebx)
	movl	%edx, 4(%ebx)
	movl	$16, 8(%ebx)
	call	NandcWaitFlashReady
	movl	$112, 8(%ebx)
	movl	$800, -8(%ebp)
	.p2align 4,,7
	.p2align 3
.L255:
	movl	-8(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -8(%ebp)
	jne	.L255
	movl	(%ebx), %eax
	addl	$4, %esp
	popl	%ebx
	popl	%ebp
	andl	$1, %eax
	ret
	.size	SandiskProgTestBadBlock, .-SandiskProgTestBadBlock
	.p2align 4,,15
	.globl	FlashReadSpare
	.type	FlashReadSpare, @function
FlashReadSpare:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%esi
	movl	%ecx, %esi
	movzbl	gNandParaInfo+9, %ecx
	pushl	%ebx
	leal	8(%eax), %ebx
	sall	$8, %ebx
	addl	gpNandc, %ebx
	sall	$9, %ecx
	movl	$0, 8(%ebx)
	movl	%ecx, 4(%ebx)
	shrl	$8, %ecx
	movl	%ecx, 4(%ebx)
	movzbl	%dl, %ecx
	movl	%ecx, 4(%ebx)
	movl	%edx, %ecx
	shrl	$8, %ecx
	shrl	$16, %edx
	movl	%ecx, 4(%ebx)
	movl	%edx, 4(%ebx)
	movl	$48, 8(%ebx)
	call	NandcWaitFlashReady
	movl	(%ebx), %eax
	popl	%ebx
	movb	%al, (%esi)
	popl	%esi
	popl	%ebp
	ret
	.size	FlashReadSpare, .-FlashReadSpare
	.p2align 4,,15
	.globl	FlashSetInterfaceMode
	.type	FlashSetInterfaceMode, @function
FlashSetInterfaceMode:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
.L262:
	movzbl	IDByte(,%edx,8), %ecx
	cmpb	$69, %cl
	je	.L263
	cmpb	$-104, %cl
	jne	.L259
.L263:
	leal	8(%edx), %ecx
	sall	$8, %ecx
	addl	gpNandc, %ecx
	cmpl	$1, %eax
	je	.L265
	testb	$4, gFlashInterfaceMode
	jne	.L266
.L259:
	addl	$1, %edx
	cmpl	$2, %edx
	jne	.L262
	xorl	%eax, %eax
	call	NandcWaitFlashReady
	xorl	%eax, %eax
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L266:
	movl	$239, 8(%ecx)
	movl	$128, 4(%ecx)
	movl	$0, (%ecx)
	movl	$0, (%ecx)
	movl	$0, (%ecx)
	movl	$0, (%ecx)
	jmp	.L259
	.p2align 4,,7
	.p2align 3
.L265:
	testb	$1, gFlashInterfaceMode
	je	.L259
	movl	$239, 8(%ecx)
	movl	$128, 4(%ecx)
	movl	$1, (%ecx)
	movl	$0, (%ecx)
	movl	$0, (%ecx)
	movl	$0, (%ecx)
	jmp	.L259
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.p2align 4,,15
	.globl	FlashEraseBlock
	.type	FlashEraseBlock, @function
FlashEraseBlock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	movzbl	%al, %ebx
	subl	$12, %esp
	movl	%ebx, %eax
	movl	%edx, -20(%ebp)
	movl	%ecx, -24(%ebp)
	call	NandcWaitFlashReady
	movl	gpNandc, %edi
	movl	%esi, %ecx
	movl	$1, %edx
	sall	%cl, %edx
	movl	-24(%ebp), %ecx
	movl	(%edi), %eax
	movb	%dl, %al
	movl	-20(%ebp), %edx
	movl	%eax, (%edi)
	movl	%ebx, %eax
	call	FlashEraseCmd
	movl	%ebx, %eax
	addl	$8, %ebx
	call	NandcWaitFlashReady
	movl	gpNandc, %ecx
	sall	$8, %ebx
	addl	%ecx, %ebx
	movl	$112, 8(%ebx)
	movl	$800, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L268:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L268
	movl	(%ebx), %eax
	movl	(%ecx), %edx
	andl	$1, %eax
	andl	$-131328, %edx
	movl	%edx, (%ecx)
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashEraseBlock, .-FlashEraseBlock
	.p2align 4,,15
	.globl	FlashReset
	.type	FlashReset, @function
FlashReset:
	pushl	%ebp
	movl	%eax, %ecx
	movl	%esp, %ebp
	subl	$12, %esp
	movl	gpNandc, %edx
	movl	%ebx, -12(%ebp)
	movzbl	%al, %eax
	movl	$1, %ebx
	movl	%edi, -4(%ebp)
	sall	%cl, %ebx
	movl	%esi, -8(%ebp)
	leal	8(%eax), %edi
	movl	(%edx), %esi
	sall	$8, %edi
	addl	%edx, %edi
	movl	%esi, %ecx
	movb	%bl, %cl
	movl	%ecx, %esi
	movl	%esi, (%edx)
	movl	$255, 8(%edi)
	call	NandcWaitFlashReady
	movl	gpNandc, %edx
	movl	(%edx), %eax
	andl	$-131328, %eax
	movl	%eax, (%edx)
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	FlashReset, .-FlashReset
	.section	.rodata.str1.4
	.align 4
.LC2:
	.string	"\0013No.%d FLASH ID:%x %x %x %x %x %x\n"
	.text
	.p2align 4,,15
	.globl	FlashReadID
	.type	FlashReadID, @function
FlashReadID:
	pushl	%ebp
	movl	%eax, %ecx
	movl	%esp, %ebp
	movzbl	%al, %eax
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%edx, %ebx
	subl	$44, %esp
	movl	%eax, -20(%ebp)
	movb	%cl, -24(%ebp)
	call	FlashReset
	movl	gpNandc, %edi
	movl	$1, %edx
	movzbl	-24(%ebp), %ecx
	movl	-20(%ebp), %eax
	movl	(%edi), %esi
	sall	%cl, %edx
	addl	$8, %eax
	sall	$8, %eax
	movl	%esi, %ecx
	addl	%edi, %eax
	movb	%dl, %cl
	movl	%ecx, %esi
	movl	%esi, (%edi)
	movl	$144, 8(%eax)
	movl	$0, 4(%eax)
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L272:
	movl	-16(%ebp), %edx
	leal	-1(%edx), %ecx
	testl	%edx, %edx
	movl	%ecx, -16(%ebp)
	jne	.L272
	movl	(%eax), %edx
	movb	%dl, (%ebx)
	movl	(%eax), %edx
	movb	%dl, 1(%ebx)
	movl	(%eax), %edx
	movb	%dl, 2(%ebx)
	movl	(%eax), %edx
	movb	%dl, 3(%ebx)
	movl	(%eax), %edx
	movb	%dl, 4(%ebx)
	movl	(%eax), %edx
	movb	%dl, 5(%ebx)
	movl	(%eax), %edx
	movb	%dl, 6(%ebx)
	movl	gpNandc, %edx
	movl	(%eax), %eax
	movb	%al, 7(%ebx)
	movl	(%edx), %eax
	andl	$-131328, %eax
	movl	%eax, (%edx)
	movzbl	(%ebx), %eax
	leal	-1(%eax), %edx
	cmpb	$-3, %dl
	jbe	.L275
.L271:
	addl	$44, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L275:
	movzbl	5(%ebx), %edx
	movzbl	%al, %eax
	movl	%eax, 8(%esp)
	movl	-20(%ebp), %eax
	movl	$.LC2, (%esp)
	movl	%edx, 28(%esp)
	movzbl	4(%ebx), %edx
	addl	$1, %eax
	movl	%eax, 4(%esp)
	movl	%edx, 24(%esp)
	movzbl	3(%ebx), %edx
	movl	%edx, 20(%esp)
	movzbl	2(%ebx), %edx
	movl	%edx, 16(%esp)
	movzbl	1(%ebx), %edx
	movl	%edx, 12(%esp)
	call	printk
	jmp	.L271
	.size	FlashReadID, .-FlashReadID
	.p2align 4,,15
	.globl	FlashReadDpCmd
	.type	FlashReadDpCmd, @function
FlashReadDpCmd:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movzbl	%al, %ebx
	subl	$8, %esp
	movzbl	gNandOptPara+8, %eax
	leal	8(%ebx), %esi
	sall	$8, %esi
	addl	gpNandc, %esi
	cmpb	$1, gNandOptPara+16
	movl	%eax, 8(%esi)
	je	.L279
	movzbl	%dl, %eax
	movl	%eax, 4(%esi)
	movl	%edx, %eax
	shrl	$8, %eax
	movl	%eax, 4(%esi)
	movl	%edx, %eax
	shrl	$16, %eax
	movl	%eax, 4(%esi)
	movzbl	gNandOptPara+9, %eax
	movl	%eax, 8(%esi)
	movzbl	%cl, %eax
	movl	%eax, 4(%esi)
	movl	%ecx, %eax
	shrl	$8, %eax
	shrl	$16, %ecx
	movl	%eax, 4(%esi)
	movl	%ecx, 4(%esi)
	movl	$48, 8(%esi)
.L278:
	movl	%ebx, %eax
	call	FlashSetRandomizer
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L279:
	movzbl	%dl, %eax
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	%eax, 4(%esi)
	movl	%edx, %eax
	shrl	$8, %eax
	movl	%eax, 4(%esi)
	movl	%edx, %eax
	shrl	$16, %eax
	movl	%eax, 4(%esi)
	movzbl	gNandOptPara+9, %eax
	movl	%eax, 8(%esi)
	movl	%ebx, %eax
	movl	%edx, -12(%ebp)
	movl	%ecx, -16(%ebp)
	call	NandcWaitFlashReady
	movl	$0, 8(%esi)
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	-16(%ebp), %ecx
	movzbl	%cl, %eax
	movl	%eax, 4(%esi)
	movl	%ecx, %eax
	shrl	$8, %eax
	shrl	$16, %ecx
	movl	%eax, 4(%esi)
	movl	%ecx, 4(%esi)
	movl	$48, 8(%esi)
	movl	-12(%ebp), %edx
	jmp	.L278
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.p2align 4,,15
	.globl	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, @function
HynixGetReadRetryDefault:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$44, %esp
	cmpl	$2, %eax
	movl	%eax, -48(%ebp)
	movb	%al, gReadRetryInfo
	movb	$-84, gReadRetryInfo+4
	movb	$-83, gReadRetryInfo+5
	movb	$-82, gReadRetryInfo+6
	movb	$-81, gReadRetryInfo+7
	je	.L326
	cmpl	$3, -48(%ebp)
	je	.L327
	cmpl	$4, -48(%ebp)
	je	.L328
	cmpl	$5, -48(%ebp)
	je	.L329
	movl	-48(%ebp), %eax
	subl	$1, %eax
	cmpl	$1, %eax
	jbe	.L282
	movb	$7, -34(%ebp)
	movl	$4, %ebx
.L311:
	movzbl	-34(%ebp), %ecx
	movb	$0, -35(%ebp)
	movl	%ecx, -28(%ebp)
	addl	$1, %ecx
	cmpb	$0, gNandMaxDie
	movl	%ecx, -32(%ebp)
	je	.L288
.L314:
	movzbl	-35(%ebp), %eax
	movl	%eax, -56(%ebp)
	movzbl	DieCsIndex(%eax), %eax
	movl	%eax, -52(%ebp)
	movl	-52(%ebp), %esi
	sall	$6, %eax
	addl	$gReadRetryInfo+20, %eax
	movl	%eax, -40(%ebp)
	movl	-52(%ebp), %eax
	addl	$8, %esi
	sall	$8, %esi
	addl	gpNandc, %esi
	movl	$255, 8(%esi)
	call	NandcWaitFlashReady
	cmpl	$4, -48(%ebp)
	movl	$54, 8(%esi)
	je	.L330
	cmpl	$5, -48(%ebp)
	je	.L331
	movl	$174, 4(%esi)
	movl	$0, (%esi)
	movl	$176, 4(%esi)
.L325:
	movl	$77, (%esi)
	movl	$22, 8(%esi)
	movl	$23, 8(%esi)
	movl	$4, 8(%esi)
	movl	$25, 8(%esi)
	movl	$0, 8(%esi)
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	$2, 4(%esi)
	movl	$0, 4(%esi)
	movl	$48, 8(%esi)
	movl	-52(%ebp), %eax
	call	NandcWaitFlashReady
	movl	$2, %ecx
.L298:
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L299:
	addl	$1, %eax
	movl	(%esi), %edx
	cmpb	%al, %cl
	ja	.L299
	movb	$8, -33(%ebp)
.L300:
	movl	-40(%ebp), %ecx
	movl	$1, %edi
	movl	%ecx, -44(%ebp)
	.p2align 4,,7
	.p2align 3
.L302:
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L301:
	movl	(%esi), %edx
	movb	%dl, (%ecx,%eax)
	addl	$1, %eax
	cmpb	%al, %bl
	ja	.L301
	addl	$1, %edi
	addl	$8, %ecx
	cmpl	-32(%ebp), %edi
	jne	.L302
	movl	-44(%ebp), %edi
	movl	$1, %edx
	movl	$0, -24(%ebp)
	movb	%bl, -20(%ebp)
	.p2align 4,,7
	.p2align 3
.L303:
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L305:
	movl	(%esi), %ecx
	movzbl	(%edi,%eax), %ebx
	movzbl	%cl, %ecx
	addl	%ebx, %ecx
	cmpl	$255, %ecx
	movl	$0, %ecx
	cmovne	%ecx, %edx
	addl	$1, %eax
	cmpb	%al, -20(%ebp)
	ja	.L305
	addl	$1, -24(%ebp)
	addl	$8, %edi
	movl	-28(%ebp), %ecx
	cmpl	%ecx, -24(%ebp)
	jl	.L303
	testl	%edx, %edx
	movzbl	-20(%ebp), %ebx
	jne	.L307
	subb	$1, -33(%ebp)
	jne	.L300
.L307:
	movl	-52(%ebp), %eax
	movl	$255, 8(%esi)
	call	NandcWaitFlashReady
	cmpl	$5, -48(%ebp)
	je	.L332
	movl	$56, 8(%esi)
.L309:
	movl	-52(%ebp), %eax
	call	NandcWaitFlashReady
	addb	$1, -35(%ebp)
	movzbl	-35(%ebp), %ecx
	cmpb	%cl, gNandMaxDie
	ja	.L314
.L288:
	movzbl	-34(%ebp), %ecx
	movb	%bl, gReadRetryInfo+1
	movb	%cl, gReadRetryInfo+2
	addl	$44, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L330:
	movl	$255, 4(%esi)
	movl	$64, (%esi)
	movl	$204, 4(%esi)
	jmp	.L325
.L331:
	movl	-52(%ebp), %eax
	movl	$56, 4(%esi)
	movl	$82, (%esi)
	movl	$22, 8(%esi)
	movl	$23, 8(%esi)
	movl	$4, 8(%esi)
	movl	$25, 8(%esi)
	movl	$0, 8(%esi)
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	$0, 4(%esi)
	movl	$2, 4(%esi)
	movl	$0, 4(%esi)
	movl	$48, 8(%esi)
	call	NandcWaitFlashReady
	movl	$16, %ecx
	jmp	.L298
.L332:
	movl	-56(%ebp), %eax
	movl	$-1, %edx
	movl	$54, 8(%esi)
	movl	$56, 4(%esi)
	movl	$0, (%esi)
	movl	$22, 8(%esi)
	call	FlashReadCmd
	jmp	.L309
.L326:
	movb	$-89, gReadRetryInfo+4
	movb	$-9, refValueDefault+17
.L282:
	cmpb	$0, gNandMaxDie
	movl	$4, %ebx
	movb	$7, -34(%ebp)
	je	.L288
	movl	gpNandc, %eax
	xorl	%ebx, %ebx
	movl	%eax, -20(%ebp)
.L294:
	movzbl	%bl, %eax
	xorl	%esi, %esi
	movzbl	DieCsIndex(%eax), %edi
	movl	%edi, %ecx
	addl	$8, %edi
	sall	$8, %edi
	addl	-20(%ebp), %edi
	sall	$6, %ecx
	addl	$gReadRetryInfo+20, %ecx
.L290:
	movl	$55, 8(%edi)
	movzbl	gReadRetryInfo+4(%esi), %eax
	movl	%eax, 4(%edi)
	movl	$800, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L289:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L289
	movl	(%edi), %eax
	movb	%al, (%ecx,%esi)
	addl	$1, %esi
	cmpl	$4, %esi
	jne	.L290
	movl	%ecx, %esi
	xorl	%edi, %edi
.L291:
	movl	$1, %eax
	.p2align 4,,7
	.p2align 3
.L292:
	movzbl	refValueDefault(%edi,%eax,4), %edx
	addb	(%esi), %dl
	movb	%dl, (%esi,%eax,8)
	addl	$1, %eax
	cmpl	$7, %eax
	jne	.L292
	addl	$1, %edi
	addl	$1, %esi
	cmpl	$4, %edi
	jne	.L291
	addl	$1, %ebx
	cmpb	%bl, gNandMaxDie
	movb	$0, 16(%ecx)
	movb	$0, 24(%ecx)
	movb	$0, 32(%ecx)
	movb	$0, 40(%ecx)
	movb	$0, 48(%ecx)
	movb	$0, 41(%ecx)
	movb	$0, 49(%ecx)
	ja	.L294
	movb	$7, -34(%ebp)
	movl	$4, %ebx
	jmp	.L288
.L329:
	movb	$56, gReadRetryInfo+4
	movl	$4, %ebx
	movb	$57, gReadRetryInfo+5
	movb	$58, gReadRetryInfo+6
	movb	$59, gReadRetryInfo+7
.L284:
	movb	$8, -34(%ebp)
	jmp	.L311
.L328:
	movb	$-52, gReadRetryInfo+4
	movl	$8, %ebx
	movb	$-65, gReadRetryInfo+5
	movb	$-86, gReadRetryInfo+6
	movb	$-85, gReadRetryInfo+7
	movb	$-51, gReadRetryInfo+8
	movb	$-83, gReadRetryInfo+9
	movb	$-82, gReadRetryInfo+10
	movb	$-81, gReadRetryInfo+11
	jmp	.L284
.L327:
	movb	$-80, gReadRetryInfo+4
	movl	$8, %ebx
	movb	$-79, gReadRetryInfo+5
	movb	$-78, gReadRetryInfo+6
	movb	$-77, gReadRetryInfo+7
	movb	$-76, gReadRetryInfo+8
	movb	$-75, gReadRetryInfo+9
	movb	$-74, gReadRetryInfo+10
	movb	$-73, gReadRetryInfo+11
	jmp	.L284
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
	.p2align 4,,15
	.globl	FlashGetReadRetryDefault
	.type	FlashGetReadRetryDefault, @function
FlashGetReadRetryDefault:
	pushl	%ebp
	testl	%eax, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	je	.L333
	leal	-1(%eax), %edx
	cmpl	$4, %edx
	jbe	.L345
	cmpl	$49, %eax
	je	.L346
	cmpl	$65, %eax
	je	.L337
	cmpl	$33, %eax
	.p2align 4,,2
	je	.L337
	cmpl	$66, %eax
	.p2align 4,,2
	je	.L337
	cmpl	$67, %eax
	.p2align 4,,2
	je	.L342
	cmpl	$34, %eax
	.p2align 4,,2
	je	.L342
	cmpl	$68, %eax
	.p2align 4,,2
	jne	.L347
.L343:
	movb	%al, gReadRetryInfo
	movl	$gReadRetryInfo+4, %eax
	movl	$Toshiba15RefValue, %esi
	movl	%eax, %edi
	movl	$23, %ecx
	movb	$5, gReadRetryInfo+1
	movb	$17, gReadRetryInfo+2
	rep movsl
	movzwl	(%esi), %eax
	movw	%ax, (%edi)
	movzbl	Toshiba15RefValue+94, %eax
	movb	%al, gReadRetryInfo+98
	.p2align 4,,7
	.p2align 3
.L333:
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L345:
	call	HynixGetReadRetryDefault
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L337:
	movb	%al, gReadRetryInfo
	movb	$4, gReadRetryInfo+1
.L344:
	movl	ToshibaA19RefValue, %eax
	popl	%esi
	movb	$7, gReadRetryInfo+2
	popl	%edi
	movl	%eax, gReadRetryInfo+4
	movl	ToshibaA19RefValue+4, %eax
	popl	%ebp
	movl	%eax, gReadRetryInfo+8
	movl	ToshibaA19RefValue+8, %eax
	movl	%eax, gReadRetryInfo+12
	movl	ToshibaA19RefValue+12, %eax
	movl	%eax, gReadRetryInfo+16
	movl	ToshibaA19RefValue+16, %eax
	movl	%eax, gReadRetryInfo+20
	movl	ToshibaA19RefValue+20, %eax
	movl	%eax, gReadRetryInfo+24
	movl	ToshibaA19RefValue+24, %eax
	movl	%eax, gReadRetryInfo+28
	movl	ToshibaA19RefValue+28, %eax
	movl	%eax, gReadRetryInfo+32
	movl	ToshibaA19RefValue+32, %eax
	movl	%eax, gReadRetryInfo+36
	movl	ToshibaA19RefValue+36, %eax
	movl	%eax, gReadRetryInfo+40
	movl	ToshibaA19RefValue+40, %eax
	movl	%eax, gReadRetryInfo+44
	movzbl	ToshibaA19RefValue+44, %eax
	movb	%al, gReadRetryInfo+48
	ret
	.p2align 4,,7
	.p2align 3
.L346:
	movl	SamsungRefValue, %eax
	movb	$49, gReadRetryInfo
	movb	$4, gReadRetryInfo+1
	movb	$15, gReadRetryInfo+2
	movl	%eax, gReadRetryInfo+4
	movl	SamsungRefValue+4, %eax
	movl	%eax, gReadRetryInfo+8
	movl	SamsungRefValue+8, %eax
	movl	%eax, gReadRetryInfo+12
	movl	SamsungRefValue+12, %eax
	movl	%eax, gReadRetryInfo+16
	movl	SamsungRefValue+16, %eax
	movl	%eax, gReadRetryInfo+20
	movl	SamsungRefValue+20, %eax
	movl	%eax, gReadRetryInfo+24
	movl	SamsungRefValue+24, %eax
	movl	%eax, gReadRetryInfo+28
	movl	SamsungRefValue+28, %eax
	movl	%eax, gReadRetryInfo+32
	movl	SamsungRefValue+32, %eax
	movl	%eax, gReadRetryInfo+36
	movl	SamsungRefValue+36, %eax
	movl	%eax, gReadRetryInfo+40
	movl	SamsungRefValue+40, %eax
	movl	%eax, gReadRetryInfo+44
	movl	SamsungRefValue+44, %eax
	movl	%eax, gReadRetryInfo+48
	movl	SamsungRefValue+48, %eax
	movl	%eax, gReadRetryInfo+52
	movl	SamsungRefValue+52, %eax
	movl	%eax, gReadRetryInfo+56
	movl	SamsungRefValue+56, %eax
	movl	%eax, gReadRetryInfo+60
	movl	SamsungRefValue+60, %eax
	movl	%eax, gReadRetryInfo+64
	jmp	.L333
	.p2align 4,,7
	.p2align 3
.L342:
	movb	%al, gReadRetryInfo
	movb	$5, gReadRetryInfo+1
	jmp	.L344
	.p2align 4,,7
	.p2align 3
.L347:
	cmpl	$35, %eax
	je	.L343
	jmp	.L333
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.p2align 4,,15
	.globl	FlashDeInit
	.type	FlashDeInit, @function
FlashDeInit:
	pushl	%ebp
	xorl	%eax, %eax
	movl	%esp, %ebp
	call	NandcWaitFlashReady
	call	FlashSetReadRetryDefault
	cmpb	$0, gFlashToggleModeEn
	je	.L349
	movl	$1, %eax
	call	FlashSetInterfaceMode
	movl	$1, %eax
	call	NandcSetMode
	movb	$0, gFlashToggleModeEn
.L349:
	movl	gpNandc, %eax
	movl	$0, 336(%eax)
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FlashDeInit, .-FlashDeInit
	.p2align 4,,15
	.globl	NandcRandmzSel
	.type	NandcRandmzSel, @function
NandcRandmzSel:
	movl	gpNandc, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	%eax, 336(%edx)
	popl	%ebp
	ret
	.size	NandcRandmzSel, .-NandcRandmzSel
	.p2align 4,,15
	.globl	Nandc_fmwait_cfg
	.type	Nandc_fmwait_cfg, @function
Nandc_fmwait_cfg:
	movl	gpNandc, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	%eax, 4(%edx)
	popl	%ebp
	ret
	.size	Nandc_fmwait_cfg, .-Nandc_fmwait_cfg
	.section	.rodata.str1.1
.LC3:
	.string	"\0013FMCTL 0x%x: %d mhz %x\n"
	.text
	.p2align 4,,15
	.globl	NandcTimeCfg
	.type	NandcTimeCfg, @function
NandcTimeCfg:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$16, %esp
	movl	gp_nandc_info, %edx
	movl	gpNandc, %ebx
	movl	8(%edx), %ecx
	movl	(%ebx), %edx
	andl	$-117440513, %edx
	orl	$33554432, %edx
	cmpl	$300, %ecx
	movl	%edx, (%ebx)
	jg	.L359
	cmpl	$185, %ecx
	jg	.L360
	cmpl	$160, %ecx
	jg	.L361
	cmpl	$35, %eax
	jbe	.L362
	cmpl	$99, %eax
	movl	gpNandc, %eax
	ja	.L363
	movl	$4225, 4(%eax)
	jmp	.L354
	.p2align 4,,7
	.p2align 3
.L360:
	movl	gpNandc, %eax
	movl	$4258, 4(%eax)
.L354:
	movl	4(%eax), %edx
	movl	(%eax), %eax
	movl	%ecx, 8(%esp)
	movl	$.LC3, (%esp)
	movl	%edx, 12(%esp)
	movl	%eax, 4(%esp)
	call	printk
	addl	$16, %esp
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L359:
	movl	gpNandc, %eax
	movl	$8386, 4(%eax)
	jmp	.L354
	.p2align 4,,7
	.p2align 3
.L362:
	movl	gpNandc, %eax
	movl	$4193, 4(%eax)
	jmp	.L354
	.p2align 4,,7
	.p2align 3
.L361:
	movl	gpNandc, %eax
	movl	$4225, 4(%eax)
	jmp	.L354
	.p2align 4,,7
	.p2align 3
.L363:
	movl	$8354, 4(%eax)
	jmp	.L354
	.size	NandcTimeCfg, .-NandcTimeCfg
	.p2align 4,,15
	.globl	NandcGetTimeCfg
	.type	NandcGetTimeCfg, @function
NandcGetTimeCfg:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	gpNandc, %ebx
	movl	4(%ebx), %ebx
	movl	%ebx, (%eax)
	movl	gpNandc, %eax
	movl	(%eax), %eax
	movl	%eax, (%edx)
	movl	gpNandc, %eax
	movl	304(%eax), %eax
	movl	%eax, (%ecx)
	movl	gpNandc, %eax
	movl	308(%eax), %edx
	movl	344(%eax), %eax
	andl	$255, %edx
	sall	$16, %eax
	orl	%edx, %eax
	movl	8(%ebp), %edx
	movl	%eax, (%edx)
	popl	%ebx
	popl	%ebp
	ret
	.size	NandcGetTimeCfg, .-NandcGetTimeCfg
	.p2align 4,,15
	.globl	NandcBchSel
	.type	NandcBchSel, @function
NandcBchSel:
	movl	gpNandc, %ecx
	movzbl	%al, %edx
	cmpb	$16, %al
	pushl	%ebp
	movl	%esp, %ebp
	movl	%edx, gNandcEccBits
	movl	$4096, %edx
	movl	$1, 8(%ecx)
	je	.L367
	cmpb	$24, %al
	movl	$4112, %edx
	je	.L367
	cmpb	$40, %al
	movl	$266256, %edx
	movl	$266240, %eax
	cmove	%eax, %edx
.L367:
	orl	$1, %edx
	movl	%edx, 12(%ecx)
	popl	%ebp
	ret
	.size	NandcBchSel, .-NandcBchSel
	.p2align 4,,15
	.globl	FlashBchSel
	.type	FlashBchSel, @function
FlashBchSel:
	pushl	%ebp
	movl	%esp, %ebp
	movb	%al, gNandFlashEccBits
	movzbl	%al, %eax
	call	NandcBchSel
	popl	%ebp
	ret
	.size	FlashBchSel, .-FlashBchSel
	.p2align 4,,15
	.globl	FlashResume
	.type	FlashResume, @function
FlashResume:
	movl	NANDC_FMCTL, %edx
	movl	gpNandc, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	NANDC_DLL_CTL_REG0, %ebx
	movl	%edx, (%eax)
	movl	NANDC_FMWAIT, %edx
	movl	gpNandc, %eax
	movl	%edx, 4(%eax)
	movl	NANDC_FLCTL, %edx
	movl	%edx, 8(%eax)
	movl	NANDC_BCHCTL, %edx
	movl	%edx, 12(%eax)
	movl	NANDC_DLL_CTL_REG1, %edx
	movl	%ebx, 304(%eax)
	movl	%edx, 308(%eax)
	movl	NANDC_RANDMZ_CFG, %edx
	movl	%edx, 336(%eax)
	movl	NANDC_FMWAIT_SYN, %edx
	movl	%edx, 344(%eax)
	xorl	%eax, %eax
	call	FlashReset
	movl	$1, %eax
	call	FlashReset
	cmpb	$0, gFlashToggleModeEn
	je	.L371
	movl	$1, %eax
	movzbl	%bh, %ebx
	call	NandcSetMode
	movzbl	gFlashInterfaceMode, %eax
	call	FlashSetInterfaceMode
	movzbl	gFlashInterfaceMode, %eax
	call	NandcSetMode
	movl	%ebx, %eax
	movl	gpNandc, %edx
	sall	$16, %eax
	sall	$8, %ebx
	orl	%ebx, %eax
	orl	$1, %eax
	movl	%eax, 304(%edx)
.L371:
	movl	gpNandParaInfo, %eax
	movzbl	20(%eax), %eax
	call	FlashBchSel
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashResume, .-FlashResume
	.p2align 4,,15
	.globl	NandCIrqEnable
	.type	NandCIrqEnable, @function
NandCIrqEnable:
	pushl	%ebp
	movl	%edx, %ecx
	movl	%esp, %ebp
	subl	$8, %esp
	movl	%esi, -4(%ebp)
	movl	368(%eax), %esi
	movl	%ebx, -8(%ebp)
	movl	$1, %ebx
	sall	%cl, %ebx
	orl	%ebx, %esi
	movl	%esi, 368(%eax)
	movl	364(%eax), %edx
	movl	-4(%ebp), %esi
	orl	%ebx, %edx
	movl	-8(%ebp), %ebx
	movl	%edx, 364(%eax)
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	NandCIrqEnable, .-NandCIrqEnable
	.p2align 4,,15
	.globl	NandCIrqDisable
	.type	NandCIrqDisable, @function
NandCIrqDisable:
	pushl	%ebp
	movl	%edx, %ecx
	movl	%esp, %ebp
	subl	$8, %esp
	movl	%esi, -4(%ebp)
	movl	368(%eax), %esi
	movl	%ebx, -8(%ebp)
	movl	$1, %ebx
	sall	%cl, %ebx
	orl	%ebx, %esi
	notl	%ebx
	movl	%esi, 368(%eax)
	movl	364(%eax), %edx
	movl	-4(%ebp), %esi
	andl	%edx, %ebx
	movl	%ebx, 364(%eax)
	movl	-8(%ebp), %ebx
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	NandCIrqDisable, .-NandCIrqDisable
	.p2align 4,,15
	.globl	NandcIrqBChFaiLEnable
	.type	NandcIrqBChFaiLEnable, @function
NandcIrqBChFaiLEnable:
	movl	gpNandc, %eax
	pushl	%ebp
	movl	%esp, %ebp
	movl	$0, gNandcEccError
	movl	368(%eax), %edx
	orl	$8, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	orl	$8, %edx
	movl	%edx, 364(%eax)
	popl	%ebp
	ret
	.size	NandcIrqBChFaiLEnable, .-NandcIrqBChFaiLEnable
	.p2align 4,,15
	.globl	NandcGetIrqSts
	.type	NandcGetIrqSts, @function
NandcGetIrqSts:
	pushl	%ebp
	movl	372(%eax), %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	NandcGetIrqSts, .-NandcGetIrqSts
	.p2align 4,,15
	.globl	NandcFlashReady
	.type	NandcFlashReady, @function
NandcFlashReady:
	movl	368(%eax), %edx
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	orl	$2, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	andl	$-3, %edx
	movl	%edx, 364(%eax)
	ret
	.size	NandcFlashReady, .-NandcFlashReady
	.p2align 4,,15
	.globl	NandcFlashLlpDone
	.type	NandcFlashLlpDone, @function
NandcFlashLlpDone:
	movl	368(%eax), %edx
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	orl	$16, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	andl	$-17, %edx
	movl	%edx, 364(%eax)
	ret
	.size	NandcFlashLlpDone, .-NandcFlashLlpDone
	.p2align 4,,15
	.globl	NandcFlashEccFailClean
	.type	NandcFlashEccFailClean, @function
NandcFlashEccFailClean:
	movl	368(%eax), %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	$1, gNandcEccError
	orl	$8, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	popl	%ebp
	andl	$-9, %edx
	movl	%edx, 364(%eax)
	ret
	.size	NandcFlashEccFailClean, .-NandcFlashEccFailClean
	.p2align 4,,15
	.globl	NandcFlashXferComp
	.type	NandcFlashXferComp, @function
NandcFlashXferComp:
	movl	368(%eax), %edx
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	orl	$1, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	andl	$-2, %edx
	movl	%edx, 364(%eax)
	ret
	.size	NandcFlashXferComp, .-NandcFlashXferComp
	.p2align 4,,15
	.globl	NandcXferStart
	.type	NandcXferStart, @function
NandcXferStart:
	pushl	%ebp
	andl	$7, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$52, %esp
	movb	%dl, -48(%ebp)
	movl	gpNandc, %edx
	sall	$5, %eax
	movb	%cl, -49(%ebp)
	movzbl	8(%ebp), %ecx
	movl	12(%edx), %ebx
	shrb	%cl
	andl	$1, %ecx
	sall	$4, %ecx
	movb	$16, %bh
	andb	$23, %bl
	orl	%eax, %ebx
	movzbl	-48(%ebp), %eax
	movl	%ebx, -28(%ebp)
	movl	12(%ebp), %ebx
	andl	$1, %eax
	addl	%eax, %eax
	orl	%ecx, %eax
	xorl	%ecx, %ecx
	orl	$536871976, %eax
	movl	%eax, -20(%ebp)
	movl	%eax, %esi
	movl	16(%edx), %eax
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	andl	$-5, %eax
	movl	%eax, -16(%ebp)
	movl	16(%ebp), %eax
	testl	%eax, %eax
	setne	%cl
	xorl	%eax, %eax
	testl	%ebx, %ebx
	setne	%al
	orl	%eax, %ecx
	je	.L381
	cmpb	$0, -48(%ebp)
	je	.L382
	cmpl	$25, gNandcEccBits
	sbbl	%eax, %eax
	andl	$-64, %eax
	subl	$-128, %eax
	movl	%eax, -24(%ebp)
	movzbl	-49(%ebp), %eax
	shrb	%al
	movzbl	%al, %eax
	testl	%eax, %eax
	movl	%eax, -32(%ebp)
	je	.L382
	movl	16(%ebp), %edx
	xorl	%ebx, %ebx
	xorl	%ecx, %ecx
	jmp	.L386
	.p2align 4,,7
	.p2align 3
.L401:
	movzwl	2(%edx), %eax
	movl	%ebx, %edi
	addl	$1, %ecx
	movzwl	(%edx), %esi
	shrl	$2, %edi
	addl	$4, %edx
	addl	-24(%ebp), %ebx
	sall	$16, %eax
	orl	%esi, %eax
	movl	gMasterInfo+24, %esi
	cmpl	-32(%ebp), %ecx
	movl	%eax, (%esi,%edi,4)
	jae	.L400
.L386:
	movl	16(%ebp), %eax
	testl	%eax, %eax
	jne	.L401
	movl	%ebx, %esi
	movl	gMasterInfo+24, %eax
	addl	$1, %ecx
	shrl	$2, %esi
	addl	-24(%ebp), %ebx
	cmpl	-32(%ebp), %ecx
	movl	$-1, (%eax,%esi,4)
	jb	.L386
	.p2align 4,,7
	.p2align 3
.L400:
	movl	gpNandc, %edx
.L382:
	movl	368(%edx), %eax
	movzbl	-49(%ebp), %esi
	movl	$0, gNandcEccError
	movl	$0, nandc0_xfer_completed_flag
	orl	$1, %eax
	movl	%eax, 368(%edx)
	movl	364(%edx), %eax
	addl	$1, %esi
	sall	$21, %esi
	andl	$264241152, %esi
	orl	-20(%ebp), %esi
	orl	$1, %eax
	movl	%eax, 364(%edx)
	movl	gMasterInfo+40, %eax
	movl	$0, gMasterInfo+32
	testl	%eax, %eax
	jne	.L402
	movl	gMasterInfo+28, %eax
	movl	%eax, 24(%edx)
	movl	12(%ebp), %eax
	testl	%eax, %eax
	je	.L403
	cmpb	$1, -48(%ebp)
	movl	12(%ebp), %edx
	movl	12(%ebp), %ecx
	sbbl	%eax, %eax
	movl	dma_ops, %ebx
	notl	%eax
	addl	$2, %eax
	addl	$-2147483648, %edx
	movl	%eax, 4(%esp)
	movl	%esi, %eax
	andl	$4095, %ecx
	shrl	$12, %eax
	andl	$31744, %eax
	shrl	$12, %edx
	movl	%eax, (%esp)
	sall	$5, %edx
	xorl	%eax, %eax
	addl	mem_map, %edx
	movl	$0, 8(%esp)
	call	*16(%ebx)
	movl	gpNandc, %edx
	movl	$1, gMasterInfo+32
	movl	%eax, gMasterInfo+8
	movl	%eax, 20(%edx)
.L392:
	movl	$0, -16(%ebp)
	movl	-16(%ebp), %eax
	andb	$193, %ah
	orb	$32, %ah
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	orl	$448, %eax
	testb	$3, 12(%ebp)
	movl	%eax, -16(%ebp)
	je	.L394
.L396:
	movl	-16(%ebp), %eax
	xorl	%ecx, %ecx
	orl	$4, %eax
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	cmpb	$0, -48(%ebp)
	sete	%cl
	andl	$-3, %eax
	addl	%ecx, %ecx
	orl	%ecx, %eax
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	orl	$1, %eax
	movl	%eax, -16(%ebp)
.L381:
	movl	-16(%ebp), %eax
	movl	-28(%ebp), %ebx
	movl	%eax, 16(%edx)
	movl	%ebx, 12(%edx)
	movl	%esi, 8(%edx)
	orl	$4, %esi
	movl	%esi, 8(%edx)
	addl	$52, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L402:
	movl	12(%ebp), %eax
	testl	%eax, %eax
	movl	gMasterInfo+20, %eax
	cmovne	gMasterInfo+8, %eax
	movl	%eax, 20(%edx)
	movl	16(%ebp), %eax
	testl	%eax, %eax
	movl	gMasterInfo+28, %eax
	cmovne	gMasterInfo+12, %eax
	movl	%eax, 24(%edx)
	jmp	.L392
	.p2align 4,,7
	.p2align 3
.L403:
	movl	gMasterInfo+20, %eax
	movl	%eax, 20(%edx)
	movl	$0, -16(%ebp)
	movl	-16(%ebp), %eax
	andb	$193, %ah
	orb	$32, %ah
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	orl	$448, %eax
	movl	%eax, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L394:
	movl	-16(%ebp), %eax
	andl	$-57, %eax
	orl	$16, %eax
	movl	%eax, -16(%ebp)
	jmp	.L396
	.size	NandcXferStart, .-NandcXferStart
	.p2align 4,,15
	.globl	NandcCopy1KB
	.type	NandcCopy1KB, @function
NandcCopy1KB:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$24, %esp
	movl	%edi, -4(%ebp)
	movl	%edx, %edi
	movl	%ecx, %edx
	movl	%esi, -8(%ebp)
	movzbl	%cl, %esi
	movl	%edi, %ecx
	sall	$9, %esi
	cmpb	$1, %cl
	movl	%ebx, -12(%ebp)
	movl	%eax, %ebx
	movl	12(%ebp), %eax
	leal	4096(%ebx,%esi), %esi
	je	.L417
	movl	8(%ebp), %edi
	testl	%edi, %edi
	je	.L412
	movl	8(%ebp), %edi
	movl	$1024, -16(%ebp)
	testl	$1, %edi
	jne	.L418
.L413:
	testl	$2, %edi
	jne	.L419
.L414:
	movl	-16(%ebp), %ecx
	shrl	$2, %ecx
	rep movsl
	movl	$0, -20(%ebp)
	testb	$2, -16(%ebp)
	jne	.L420
	testb	$1, -16(%ebp)
	jne	.L421
	.p2align 4,,7
	.p2align 3
.L412:
	testl	%eax, %eax
	je	.L404
	shrb	%dl
	movzbl	%dl, %edx
	leal	(%edx,%edx,2), %edx
	leal	128(,%edx,4), %edx
	movl	(%ebx,%edx,4), %edx
	movl	%edx, %ecx
	shrl	$8, %ecx
	movb	%cl, 1(%eax)
	movl	%edx, %ecx
	movb	%dl, (%eax)
	shrl	$16, %ecx
	shrl	$24, %edx
	movb	%cl, 2(%eax)
	movb	%dl, 3(%eax)
.L404:
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L421:
	movl	-20(%ebp), %ecx
	movzbl	(%esi,%ecx), %ecx
	movl	-20(%ebp), %esi
	movb	%cl, (%edi,%esi)
	jmp	.L412
	.p2align 4,,7
	.p2align 3
.L420:
	movzwl	(%esi), %ecx
	movw	%cx, (%edi)
	testb	$1, -16(%ebp)
	movl	$2, -20(%ebp)
	je	.L412
	jmp	.L421
	.p2align 4,,7
	.p2align 3
.L417:
	movl	8(%ebp), %ecx
	testl	%ecx, %ecx
	je	.L406
	movl	%esi, %edi
	movl	8(%ebp), %esi
	testl	$1, %edi
	movl	$1024, -16(%ebp)
	jne	.L422
.L407:
	testl	$2, %edi
	jne	.L423
.L408:
	movl	-16(%ebp), %ecx
	shrl	$2, %ecx
	rep movsl
	movl	$0, -20(%ebp)
	testb	$2, -16(%ebp)
	je	.L409
	movzwl	(%esi), %ecx
	movw	%cx, (%edi)
	movl	$2, -20(%ebp)
.L409:
	testb	$1, -16(%ebp)
	je	.L406
	movl	-20(%ebp), %ecx
	movzbl	(%esi,%ecx), %ecx
	movl	-20(%ebp), %esi
	movb	%cl, (%edi,%esi)
.L406:
	testl	%eax, %eax
	je	.L404
	movzbl	1(%eax), %ecx
	shrb	%dl
	movzbl	2(%eax), %esi
	movzbl	%dl, %edx
	movl	-4(%ebp), %edi
	sall	$8, %ecx
	sall	$16, %esi
	orl	%esi, %ecx
	movzbl	(%eax), %esi
	movzbl	3(%eax), %eax
	orl	%esi, %ecx
	movl	-8(%ebp), %esi
	sall	$24, %eax
	orl	%eax, %ecx
	leal	(%edx,%edx,2), %eax
	leal	128(,%eax,4), %eax
	movl	%ecx, (%ebx,%eax,4)
	movl	-12(%ebp), %ebx
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L418:
	movzbl	(%esi), %ecx
	addl	$1, %esi
	movb	%cl, (%edi)
	addl	$1, %edi
	movl	$1023, -16(%ebp)
	jmp	.L413
	.p2align 4,,7
	.p2align 3
.L419:
	movzwl	(%esi), %ecx
	addl	$2, %esi
	movw	%cx, (%edi)
	addl	$2, %edi
	subl	$2, -16(%ebp)
	jmp	.L414
	.p2align 4,,7
	.p2align 3
.L422:
	movzbl	(%esi), %ecx
	addl	$1, %esi
	movb	%cl, (%edi)
	addl	$1, %edi
	movl	$1023, -16(%ebp)
	jmp	.L407
	.p2align 4,,7
	.p2align 3
.L423:
	movzwl	(%esi), %ecx
	addl	$2, %esi
	movw	%cx, (%edi)
	addl	$2, %edi
	subl	$2, -16(%ebp)
	jmp	.L408
	.size	NandcCopy1KB, .-NandcCopy1KB
	.p2align 4,,15
	.globl	NandcGetBchErrorValue
	.type	NandcGetBchErrorValue, @function
NandcGetBchErrorValue:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	movl	%eax, -20(%ebp)
	xorl	%eax, %eax
	shrl	$2, -20(%ebp)
	movl	-20(%ebp), %esi
	testl	%esi, %esi
	je	.L425
	movl	gNandcEccBits, %ebx
	testl	%ebx, %ebx
	je	.L425
	movl	gpNandc, %edx
	movl	%edx, -24(%ebp)
	xorl	%edx, %edx
	jmp	.L429
	.p2align 4,,7
	.p2align 3
.L435:
	movl	-16(%ebp), %ecx
	andb	$128, %ch
	jne	.L433
	movl	-16(%ebp), %edi
	movl	-16(%ebp), %ebx
	movl	-16(%ebp), %esi
	movl	-16(%ebp), %ecx
	shrl	$3, %edi
	shrl	$22, %ebx
	andl	$31, %edi
	shrl	$16, %esi
	andl	$32, %ebx
	shrl	$24, %ecx
	andl	$31, %esi
	andl	$32, %ecx
	orl	%edi, %ebx
	orl	%esi, %ecx
	cmpl	%ecx, %ebx
	movl	-16(%ebp), %ebx
	jbe	.L427
	movl	-16(%ebp), %ecx
	shrl	$3, %ebx
	andl	$31, %ebx
	shrl	$22, %ecx
	andl	$32, %ecx
	orl	%ebx, %ecx
.L428:
	cmpl	%ecx, %eax
	cmovb	%ecx, %eax
	addl	$1, %edx
	cmpl	-20(%ebp), %edx
	je	.L425
.L429:
	movl	-24(%ebp), %ebx
	leal	8(%edx), %ecx
	movl	(%ebx,%ecx,4), %ecx
	movl	%ecx, -16(%ebp)
	movl	-16(%ebp), %ecx
	andl	$4, %ecx
	je	.L435
.L433:
	addl	$1, %edx
	movl	$-1, %eax
	cmpl	-20(%ebp), %edx
	jne	.L429
.L425:
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L427:
	movl	-16(%ebp), %ecx
	shrl	$16, %ebx
	andl	$31, %ebx
	shrl	$24, %ecx
	andl	$32, %ecx
	orl	%ebx, %ecx
	jmp	.L428
	.size	NandcGetBchErrorValue, .-NandcGetBchErrorValue
	.p2align 4,,15
	.globl	NandcLlpInit
	.type	NandcLlpInit, @function
NandcLlpInit:
	pushl	%ebp
	movl	$g_llp_tbl_cpu, %ecx
	movl	%esp, %ebp
	movl	$g_llp_tbl_dma, %edx
	pushl	%edi
	movl	$4096, %eax
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	call	nand_malloc_dma_buf
	movl	$g_llp_data_buffer_cpu, %ecx
	movl	$g_llp_data_buffer_dma, %edx
	movl	$16384, %eax
	call	nand_malloc_dma_buf
	movl	$65536, %eax
	movl	$g_llp_spare_buffer_cpu, %ecx
	movl	$g_llp_spare_buffer_dma, %edx
	call	nand_malloc_dma_buf
	movl	g_llp_tbl_dma, %esi
	movl	$-1, %eax
	testl	%esi, %esi
	je	.L437
	movl	g_llp_tbl_cpu, %ecx
	testl	%ecx, %ecx
	je	.L437
	movl	g_llp_data_buffer_dma, %ebx
	testl	%ebx, %ebx
	je	.L437
	movl	g_llp_spare_buffer_dma, %edx
	testl	%edx, %edx
	movl	%edx, -16(%ebp)
	je	.L437
	movl	$g_llp_page_buf+12, %eax
	movl	g_llp_spare_buffer_cpu, %edx
	subl	%eax, %ecx
	movl	g_llp_data_buffer_cpu, %edi
	subl	%eax, %esi
	movl	%ecx, -20(%ebp)
	movl	-16(%ebp), %ecx
	subl	%edx, %ecx
	movl	%ecx, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L438:
	leal	(%esi,%eax), %ecx
	movl	%ecx, 16(%eax)
	movl	-20(%ebp), %ecx
	movl	%edx, 28(%eax)
	movl	$0, (%eax)
	movl	$0, 4(%eax)
	addl	%eax, %ecx
	movl	%ecx, 12(%eax)
	movl	-16(%ebp), %ecx
	movl	$0, 8(%eax)
	movl	$0, -4(%eax)
	movl	%edi, 20(%eax)
	addl	%edx, %ecx
	addl	$2048, %edx
	movl	%ebx, 24(%eax)
	movl	%ecx, 32(%eax)
	addl	$64, %eax
	cmpl	$g_llp_page_buf+2060, %eax
	jne	.L438
	xorl	%eax, %eax
.L437:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	NandcLlpInit, .-NandcLlpInit
	.p2align 4,,15
	.globl	NandcInit
	.type	NandcInit, @function
NandcInit:
	pushl	%ebp
	movl	%esp, %ebp
	movl	%eax, gpNandc
	movl	(%eax), %edx
	movl	$6, gNandcVer
	movl	%edx, %ecx
	andl	$507904, %edx
	shrl	$13, %ecx
	orb	$1, %dh
	andl	$1, %ecx
	movl	%ecx, gBootDdrMode
	movl	%edx, (%eax)
	movl	gpNandc, %eax
	movl	$0, 336(%eax)
	movl	$40, %eax
	call	NandcTimeCfg
	movl	gpNandc, %eax
	movl	$gMasterInfo+16, %ecx
	movl	$gMasterInfo+20, %edx
	movl	$1973763, 304(%eax)
	movl	$16642, 344(%eax)
	movl	$16384, %eax
	call	nand_malloc_dma_buf
	movl	$gMasterInfo+24, %ecx
	movl	$gMasterInfo+28, %edx
	movl	$2048, %eax
	call	nand_malloc_dma_buf
	movl	$0, gMasterInfo
	movl	$0, gMasterInfo+4
	movl	$0, gMasterInfo+32
	movl	$0, gMasterInfo+40
	movl	$0, gNandcEccError
	call	NandcLlpInit
	popl	%ebp
	ret
	.size	NandcInit, .-NandcInit
	.p2align 4,,15
	.globl	NandcLlpReadPagesConfig
	.type	NandcLlpReadPagesConfig, @function
NandcLlpReadPagesConfig:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$52, %esp
	movl	%edx, -40(%ebp)
	movl	-40(%ebp), %edi
	movl	%ecx, -44(%ebp)
	movzbl	gNandParaInfo+9, %edx
	testl	%edi, %edi
	je	.L446
	movzbl	-40(%ebp), %ecx
	movzbl	%dl, %edx
	movl	%eax, %edi
	movl	$0, -28(%ebp)
	movl	%ecx, -48(%ebp)
	movl	%edx, %ecx
	shrl	%ecx
	sall	$22, %ecx
	orl	$1033, %ecx
	sall	$9, %edx
	movl	%ecx, -36(%ebp)
	movl	%edx, -52(%ebp)
	jmp	.L449
	.p2align 4,,7
	.p2align 3
.L451:
	movl	%eax, %ecx
	movl	dma_ops, %ebx
	leal	-2147483648(%eax), %edx
	movl	-52(%ebp), %eax
	andl	$4095, %ecx
	shrl	$12, %edx
	sall	$5, %edx
	movl	$0, 8(%esp)
	addl	mem_map, %edx
	movl	%eax, (%esp)
	xorl	%eax, %eax
	movl	$2, 4(%esp)
	call	*16(%ebx)
	movl	-24(%ebp), %edx
	movl	$1, 8(%edx)
	movl	%eax, 20(%edx)
	movl	-20(%ebp), %edx
	movl	%eax, 48(%esi)
.L448:
	movzbl	-29(%ebp), %ebx
	addl	$36, %edi
	movl	$8662, 44(%esi)
	movl	%ebx, %eax
	sall	$24, %eax
	movl	%eax, %ecx
	orl	$671481968, %ecx
	movl	%ecx, 4(%esi)
	movl	%eax, %ecx
	orl	$2013528128, %ecx
	movl	%ecx, 8(%esi)
	movl	%eax, %ecx
	orl	$671481856, %ecx
	movl	%ecx, 12(%esi)
	movl	%eax, %ecx
	orl	$671416320, %ecx
	movl	%ecx, 16(%esi)
	movl	%ecx, 20(%esi)
	movzbl	%dl, %ecx
	orl	$671416320, %ecx
	orl	%eax, %ecx
	movl	%ecx, 24(%esi)
	movzbl	%dh, %ecx
	orl	$671416320, %ecx
	orl	%eax, %ecx
	movl	%ecx, 28(%esi)
	movl	%edx, %ecx
	shrl	$16, %ecx
	andl	$255, %ecx
	orl	$671416320, %ecx
	orl	%eax, %ecx
	movl	%ecx, 32(%esi)
	movl	%eax, %ecx
	orl	$-2013003776, %eax
	orl	$671481904, %ecx
	movl	%ecx, 36(%esi)
	movl	-24(%ebp), %ecx
	movl	%eax, 40(%esi)
	movl	44(%ecx), %eax
	movl	%eax, 52(%esi)
	movl	%ebx, %eax
	call	FlashGetRandomizer
	movl	-40(%ebp), %edx
	addl	$1, -28(%ebp)
	movl	%eax, 56(%esi)
	movl	-36(%ebp), %eax
	movl	%eax, 60(%esi)
	movl	-28(%ebp), %eax
	sall	$6, %eax
	movl	g_llp_page_buf+28(%eax), %eax
	andl	$-64, %eax
	orl	$7, %eax
	cmpl	%edx, -28(%ebp)
	movl	%eax, (%esi)
	je	.L446
.L449:
	movl	-48(%ebp), %eax
	leal	-16(%ebp), %edx
	movl	%edx, (%esp)
	movl	-44(%ebp), %edx
	leal	-20(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	%edi, %eax
	call	LogAddr2PhyAddr
	movl	-16(%ebp), %eax
	movzbl	DieCsIndex(%eax), %eax
	movzbl	gNandChipMap+4(,%eax,8), %eax
	movb	%al, -29(%ebp)
	movl	-28(%ebp), %eax
	sall	$6, %eax
	leal	g_llp_page_buf(%eax), %ecx
	movl	g_llp_page_buf+24(%eax), %esi
	movl	%ecx, 20(%edi)
	movl	gpNandc, %edx
	movl	%ecx, -24(%ebp)
	movzbl	-29(%ebp), %ecx
	movl	$0, (%edi)
	movl	%edx, 24(%edi)
	movl	-20(%ebp), %edx
	movl	%ecx, 32(%edi)
	movl	%edx, 28(%edi)
	movl	%ecx, g_llp_page_buf(%eax)
	movl	-24(%ebp), %eax
	movl	-24(%ebp), %ecx
	movl	%edx, 4(%eax)
	movl	8(%edi), %eax
	movl	%eax, 12(%ecx)
	movl	12(%edi), %eax
	movl	$0, 20(%ecx)
	movl	%eax, 16(%ecx)
	movl	8(%edi), %eax
	testl	%eax, %eax
	jne	.L451
	movl	-24(%ebp), %ecx
	movl	36(%ecx), %eax
	movl	$0, 8(%ecx)
	movl	%eax, 48(%esi)
	jmp	.L448
	.p2align 4,,7
	.p2align 3
.L446:
	movl	-40(%ebp), %eax
	sall	$6, %eax
	movl	g_llp_page_buf-40(%eax), %eax
	movl	$0, (%eax)
	addl	$52, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	NandcLlpReadPagesConfig, .-NandcLlpReadPagesConfig
	.p2align 4,,15
	.globl	NandcStartLlp
	.type	NandcStartLlp, @function
NandcStartLlp:
	movl	gpNandc, %edx
	pushl	%ebp
	movl	%esp, %ebp
	movl	12(%edx), %ecx
	andl	$-65289, %ecx
	orb	$16, %ch
	movl	%ecx, 12(%edx)
	.p2align 4,,7
	.p2align 3
.L457:
	movl	360(%edx), %ecx
	andl	$1, %ecx
	je	.L457
	movl	368(%edx), %ecx
	movl	$0, nandc0_llp_completed_flag
	movl	$0, gNandcEccError
	orl	$16, %ecx
	movl	%ecx, 368(%edx)
	movl	364(%edx), %ecx
	orl	$16, %ecx
	movl	%ecx, 364(%edx)
	movl	368(%edx), %ecx
	orl	$8, %ecx
	movl	%ecx, 368(%edx)
	movl	364(%edx), %ecx
	orl	$8, %ecx
	movl	%ecx, 364(%edx)
	movl	28(%eax), %eax
	andl	$-64, %eax
	orl	$7, %eax
	movl	%eax, 356(%edx)
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	NandcStartLlp, .-NandcStartLlp
	.p2align 4,,15
	.globl	Ftl_log2
	.type	Ftl_log2, @function
Ftl_log2:
	pushl	%ebp
	xorl	%ecx, %ecx
	cmpl	$1, %eax
	movl	%esp, %ebp
	jbe	.L459
	movl	$1, %edx
	.p2align 4,,7
	.p2align 3
.L460:
	addl	%edx, %edx
	addl	$1, %ecx
	cmpl	%edx, %eax
	ja	.L460
.L459:
	movl	%ecx, %eax
	popl	%ebp
	ret
	.size	Ftl_log2, .-Ftl_log2
	.p2align 4,,15
	.globl	FtlPrintInfo
	.type	FtlPrintInfo, @function
FtlPrintInfo:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlPrintInfo, .-FtlPrintInfo
	.p2align 4,,15
	.globl	FtlSysBlkNumInit
	.type	FtlSysBlkNumInit, @function
FtlSysBlkNumInit:
	pushl	%ebp
	cmpw	$23, %ax
	movl	%esp, %ebp
	movzwl	%ax, %edx
	ja	.L465
	movl	$24, %edx
	movl	$24, %eax
.L465:
	movzwl	c_ftl_nand_planes_num, %ecx
	movl	%edx, c_ftl_nand_sys_blks_per_plane
	popl	%ebp
	imull	%edx, %ecx
	movzwl	c_ftl_nand_blk_pre_plane, %edx
	subw	%ax, %dx
	movl	c_ftl_nand_totle_phy_blks, %eax
	movl	%ecx, c_ftl_nand_max_sys_blks
	movw	%dx, c_ftl_nand_data_blks_per_plane
	subl	%ecx, %eax
	movl	%eax, c_ftl_nand_max_data_blks
	xorl	%eax, %eax
	ret
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.p2align 4,,15
	.globl	FtlConstantsInit
	.type	FtlConstantsInit, @function
FtlConstantsInit:
	pushl	%ebp
	movl	%eax, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$32, %esp
	movzwl	10(%edx), %ecx
	movl	%eax, -36(%ebp)
	movzwl	8(%eax), %eax
	movzwl	12(%edx), %esi
	movw	%cx, -42(%ebp)
	movw	%ax, c_ftl_nand_type
	movzwl	14(%edx), %eax
	movw	%cx, c_ftl_nand_die_num
	movw	%si, -32(%ebp)
	movw	%si, c_ftl_nand_planes_per_die
	movw	%ax, -20(%ebp)
	movw	%ax, c_ftl_nand_blk_pre_plane
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L468:
	movb	%al, p_plane_order_table(%eax)
	addl	$1, %eax
	cmpl	$32, %eax
	jne	.L468
	movl	-36(%ebp), %edx
	movzwl	20(%edx), %eax
	movzbl	15(%edx), %edx
	cmpw	%ax, %dx
	jbe	.L469
	movzwl	-32(%ebp), %ecx
	testl	%ecx, %ecx
	movl	%ecx, -16(%ebp)
	je	.L470
	movzwl	-42(%ebp), %esi
	movzbl	-32(%ebp), %eax
	movl	$0, -28(%ebp)
	movl	%esi, -40(%ebp)
	movl	-40(%ebp), %edi
	addl	$1, %esi
	addl	%eax, %eax
	movb	%al, -18(%ebp)
	movl	%esi, -24(%ebp)
	imull	%ecx, %edi
	.p2align 4,,7
	.p2align 3
.L471:
	movl	-40(%ebp), %eax
	testl	%eax, %eax
	je	.L473
	movl	-28(%ebp), %eax
	movl	$1, %ecx
	movzbl	-32(%ebp), %esi
	movzbl	-28(%ebp), %edx
	addl	$p_plane_order_table, %eax
	.p2align 4,,7
	.p2align 3
.L472:
	leal	(%edx,%esi), %ebx
	addl	$1, %ecx
	movb	%dl, (%eax)
	movb	%bl, (%eax,%edi)
	addb	-18(%ebp), %dl
	addl	-16(%ebp), %eax
	cmpl	-24(%ebp), %ecx
	jne	.L472
.L473:
	addl	$1, -28(%ebp)
	movl	-16(%ebp), %edx
	cmpl	%edx, -28(%ebp)
	jne	.L471
	movl	-36(%ebp), %ecx
	movzwl	20(%ecx), %eax
.L470:
	salw	-42(%ebp)
	shrw	-20(%ebp)
	movzwl	-42(%ebp), %esi
	movzwl	-20(%ebp), %edx
	movw	%si, c_ftl_nand_die_num
	movw	%dx, c_ftl_nand_blk_pre_plane
.L469:
	movzbl	-20(%ebp), %esi
	xorl	%ecx, %ecx
	movzwl	-20(%ebp), %ebx
	imulw	-32(%ebp), %bx
	movw	$5, c_mlc_erase_count_value
	movw	%si, -18(%ebp)
	movw	%si, c_ftl_nand_ext_blk_pre_plane
	movzwl	-42(%ebp), %esi
	imulw	-32(%ebp), %si
	movw	%bx, c_ftl_nand_blks_per_die
	movzwl	%bx, %ebx
	cmpl	$1, %ebx
	movw	%si, -16(%ebp)
	movw	%si, c_ftl_nand_planes_num
	jbe	.L474
	movl	$1, %edx
	.p2align 4,,7
	.p2align 3
.L475:
	addl	%edx, %edx
	addl	$1, %ecx
	cmpl	%edx, %ebx
	ja	.L475
.L474:
	movl	-36(%ebp), %edx
	movw	%cx, c_ftl_nand_blks_per_die_shift
	movw	%ax, c_ftl_nand_sec_pre_page
	movzwl	16(%edx), %edi
	movzwl	18(%edx), %esi
	movzwl	-16(%ebp), %edx
	movw	%di, c_ftl_nand_page_pre_blk
	movw	%si, -24(%ebp)
	imull	%edi, %edx
	movw	%si, c_ftl_nand_page_pre_slc_blk
	movw	%dx, c_ftl_nand_page_pre_super_blk
	movzwl	%ax, %edx
	cmpl	$1, %edx
	jbe	.L481
	xorl	%ebx, %ebx
	movl	$1, %ecx
	.p2align 4,,7
	.p2align 3
.L477:
	addl	%ecx, %ecx
	addl	$1, %ebx
	cmpl	%ecx, %edx
	ja	.L477
	movzwl	%bx, %ecx
	movl	$4352, %esi
	sarl	%cl, %esi
	addl	$9, %ecx
	addl	$2, %esi
	movw	%si, -28(%ebp)
	movl	%ecx, -32(%ebp)
.L476:
	movl	-36(%ebp), %ecx
	sall	$9, %eax
	movzwl	%di, %edi
	movw	%ax, c_ftl_nand_byte_pre_page
	shrw	$8, %ax
	movw	%bx, c_ftl_nand_sec_pre_page_shift
	movzwl	-16(%ebp), %ebx
	movw	%ax, c_ftl_nand_byte_pre_oob
	movzwl	26(%ecx), %eax
	movzwl	-20(%ebp), %ecx
	movw	%ax, c_ftl_nand_reserved_blks
	movl	%ebx, %eax
	imull	%ecx, %eax
	movl	%ecx, %esi
	movl	%eax, c_ftl_nand_totle_phy_blks
	movzwl	-18(%ebp), %eax
	subl	%eax, %esi
	movl	%esi, %eax
	movzwl	-24(%ebp), %esi
	imull	%ebx, %eax
	imull	%edx, %esi
	imull	%edx, %eax
	imull	%edi, %eax
	sarl	$11, %eax
	movl	%eax, DeviceCapacity
	movl	$34816, %eax
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%esi
	cmpw	$4, %ax
	movl	%eax, %edx
	movw	%ax, c_ftl_nand_max_vendor_blks
	ja	.L478
	movw	$4, c_ftl_nand_max_vendor_blks
	movl	$4, %edx
.L478:
	movl	%ecx, %eax
	movzbl	-32(%ebp), %ecx
	sall	$6, %eax
	movzwl	-28(%ebp), %esi
	sarl	%cl, %eax
	movzwl	%ax, %ecx
	movw	%ax, c_ftl_nand_map_blks_per_plane
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	-16(%ebp)
	movw	%si, c_ftl_nand_vendor_region_num
	imull	%ecx, %ebx
	cmpw	$1, -16(%ebp)
	movl	%ebx, c_ftl_nand_max_map_blks
	movzwl	%ax, %eax
	leal	8(%ecx,%eax), %eax
	movl	%eax, c_ftl_nand_sys_blks_per_plane
	jne	.L479
	addl	$4, %eax
	movl	%eax, c_ftl_nand_sys_blks_per_plane
.L479:
	movzwl	%ax, %eax
	call	FtlSysBlkNumInit
	movl	c_ftl_nand_sys_blks_per_plane, %eax
	movzwl	c_ftl_nand_page_pre_blk, %edx
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	$0, g_MaxLbaSector
	movl	%eax, c_ftl_nand_init_sys_blks_per_plane
	movl	c_ftl_nand_max_data_blks, %eax
	addl	$9, %ecx
	sall	$2, %eax
	imull	%edx, %eax
	shrl	%cl, %eax
	movzwl	c_ftl_nand_sec_pre_page, %ecx
	addl	$2, %eax
	movw	%ax, c_ftl_nand_map_region_num
	movl	$512, %eax
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%ecx
	movw	%ax, c_ftl_nand_l2pmap_ram_region_num
	addl	$32, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L481:
	movl	$9, -32(%ebp)
	xorl	%ebx, %ebx
	movw	$4354, -28(%ebp)
	jmp	.L476
	.size	FtlConstantsInit, .-FtlConstantsInit
	.p2align 4,,15
	.globl	IsBlkInVendorPart
	.type	IsBlkInVendorPart, @function
IsBlkInVendorPart:
	movl	%eax, %ecx
	xorl	%eax, %eax
	cmpw	$0, g_totle_vendor_block
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	je	.L488
	movzwl	c_ftl_nand_max_vendor_blks, %edx
	testw	%dx, %dx
	je	.L488
	movl	p_vendor_block_table, %ebx
	movb	$1, %al
	cmpw	(%ebx), %cx
	je	.L488
	leal	-1(%edx), %eax
	xorl	%edx, %edx
	movzwl	%ax, %eax
	addl	%eax, %eax
	jmp	.L490
	.p2align 4,,7
	.p2align 3
.L491:
	addl	$2, %edx
	cmpw	(%ebx,%edx), %cx
	je	.L495
.L490:
	cmpl	%eax, %edx
	jne	.L491
	xorl	%eax, %eax
.L488:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L495:
	movl	$1, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.p2align 4,,15
	.globl	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, @function
FtlCacheMetchLpa:
	pushl	%ebp
	movl	%eax, %ecx
	movl	%esp, %ebp
	xorl	%eax, %eax
	pushl	%edi
	movl	g_wr_page_num, %edi
	pushl	%esi
	pushl	%ebx
	testl	%edi, %edi
	je	.L497
	movl	req_wr_io, %eax
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L500:
	movl	16(%eax), %esi
	cmpl	%esi, %ecx
	ja	.L498
	cmpl	%esi, %edx
	jae	.L502
.L498:
	addl	$1, %ebx
	addl	$36, %eax
	cmpl	%edi, %ebx
	jne	.L500
	xorl	%eax, %eax
.L497:
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L502:
	popl	%ebx
	movl	$1, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlCacheMetchLpa, .-FtlCacheMetchLpa
	.p2align 4,,15
	.globl	FtlGetCap
	.type	FtlGetCap, @function
FtlGetCap:
	pushl	%ebp
	movl	g_MaxLbaSector, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlGetCap, .-FtlGetCap
	.p2align 4,,15
	.globl	FtlGetCapacity
	.type	FtlGetCapacity, @function
FtlGetCapacity:
	pushl	%ebp
	movl	g_MaxLbaSector, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlGetCapacity, .-FtlGetCapacity
	.p2align 4,,15
	.globl	FtlGetLpn
	.type	FtlGetLpn, @function
FtlGetLpn:
	pushl	%ebp
	movl	g_MaxLpn, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlGetLpn, .-FtlGetLpn
	.p2align 4,,15
	.globl	FtlGetCurEraseBlock
	.type	FtlGetCurEraseBlock, @function
FtlGetCurEraseBlock:
	pushl	%ebp
	movzwl	c_ftl_nand_planes_num, %eax
	movl	%esp, %ebp
	imull	g_cur_erase_blk, %eax
	popl	%ebp
	ret
	.size	FtlGetCurEraseBlock, .-FtlGetCurEraseBlock
	.p2align 4,,15
	.globl	FtlGetAllBlockNum
	.type	FtlGetAllBlockNum, @function
FtlGetAllBlockNum:
	movzwl	c_ftl_nand_blk_pre_plane, %eax
	movzwl	c_ftl_nand_planes_num, %edx
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	imull	%edx, %eax
	ret
	.size	FtlGetAllBlockNum, .-FtlGetAllBlockNum
	.section	.rodata.str1.4
	.align 4
.LC4:
	.string	"\0013phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\n"
	.text
	.p2align 4,,15
	.globl	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, @function
FtlBbmMapBadBlock:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$20, %esp
	movzwl	c_ftl_nand_blks_per_die, %ecx
	divw	%cx
	imull	%eax, %ecx
	movzwl	%ax, %edx
	movl	%ebx, %eax
	movzwl	%bx, %ebx
	subw	%cx, %ax
	movl	%eax, %ecx
	movl	%eax, %esi
	movl	gBbtInfo+28(,%edx,4), %eax
	shrw	$5, %si
	movzwl	%cx, %ecx
	movzwl	%si, %esi
	leal	(%eax,%esi,4), %esi
	movl	$1, %eax
	sall	%cl, %eax
	orl	(%esi), %eax
	movl	%eax, (%esi)
	movl	%ebx, 4(%esp)
	movl	%eax, 16(%esp)
	movl	%ecx, 12(%esp)
	movl	%edx, 8(%esp)
	movl	$.LC4, (%esp)
	call	printk
	xorl	%eax, %eax
	addw	$1, gBbtInfo+6
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.p2align 4,,15
	.globl	FtlBbmIsBadBlock
	.type	FtlBbmIsBadBlock, @function
FtlBbmIsBadBlock:
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %eax
	movl	%edx, %ecx
	shrl	%cl, %eax
	andl	$1, %eax
	ret
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.p2align 4,,15
	.globl	FtlBbtInfoPrint
	.type	FtlBbtInfoPrint, @function
FtlBbtInfoPrint:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.p2align 4,,15
	.globl	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, @function
FtlBbt2Bitmap:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	movl	%edx, %ebx
	subl	$8, %esp
	movzwl	c_ftl_nand_bbm_buf_size, %ecx
	xorl	%edx, %edx
	movl	%ebx, %eax
	sall	$2, %ecx
	call	memset
	xorl	%eax, %eax
	jmp	.L514
	.p2align 4,,7
	.p2align 3
.L516:
	movl	%ecx, %edx
	movl	$1, %edi
	shrw	$5, %dx
	addl	$1, %eax
	movzwl	%dx, %edx
	sall	%cl, %edi
	orl	%edi, (%ebx,%edx,4)
	cmpl	$512, %eax
	je	.L512
.L514:
	movzwl	(%esi,%eax,2), %ecx
	cmpw	$-1, %cx
	movw	%cx, -14(%ebp)
	jne	.L516
.L512:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.p2align 4,,15
	.globl	FtlBbtMemInit
	.type	FtlBbtMemInit, @function
FtlBbtMemInit:
	pushl	%ebp
	movl	%esp, %ebp
	movw	$-1, gBbtInfo
	movw	$0, gBbtInfo+6
	movl	$-1, gBbtInfo+12
	movl	$-1, gBbtInfo+16
	movl	$-1, gBbtInfo+20
	movl	$-1, gBbtInfo+24
	popl	%ebp
	ret
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.p2align 4,,15
	.globl	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, @function
FtlBbtCalcTotleCnt:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	xorl	%esi, %esi
	pushl	%ebx
	subl	$4, %esp
	movzwl	c_ftl_nand_blks_per_die, %eax
	movzwl	%ax, %edi
	movw	%ax, -14(%ebp)
	movzwl	c_ftl_nand_die_num, %eax
	imull	%eax, %edi
	testl	%edi, %edi
	je	.L519
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L521:
	movl	%ebx, %eax
	xorl	%edx, %edx
	divw	-14(%ebp)
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %eax
	movl	%edx, %ecx
	shrl	%cl, %eax
	andl	$1, %eax
	cmpl	$1, %eax
	sbbw	$-1, %si
	addl	$1, %ebx
	movzwl	%bx, %eax
	cmpl	%edi, %eax
	jl	.L521
.L519:
	addl	$4, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.p2align 4,,15
	.globl	V2P_block
	.type	V2P_block, @function
V2P_block:
	movzwl	c_ftl_nand_planes_per_die, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%edx, %ebx
	xorl	%edx, %edx
	divw	%cx
	imull	%ebx, %ecx
	popl	%ebx
	popl	%ebp
	imulw	c_ftl_nand_blks_per_die, %ax
	addl	%ecx, %eax
	addl	%edx, %eax
	ret
	.size	V2P_block, .-V2P_block
	.p2align 4,,15
	.globl	P2V_plane
	.type	P2V_plane, @function
P2V_plane:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	subl	$8, %esp
	movl	%eax, %ecx
	divw	c_ftl_nand_blks_per_die
	xorl	%edx, %edx
	movl	%esi, -4(%ebp)
	movzwl	c_ftl_nand_planes_per_die, %esi
	movl	%ebx, -8(%ebp)
	movl	%eax, %ebx
	movl	%ecx, %eax
	divw	%si
	imull	%esi, %ebx
	movl	-4(%ebp), %esi
	leal	(%ebx,%edx), %eax
	movl	-8(%ebp), %ebx
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	P2V_plane, .-P2V_plane
	.p2align 4,,15
	.globl	P2V_block_in_plane
	.type	P2V_block_in_plane, @function
P2V_block_in_plane:
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	ret
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.p2align 4,,15
	.globl	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, @function
ftl_cmp_data_ver:
	pushl	%ebp
	cmpl	%edx, %eax
	movl	%esp, %ebp
	jbe	.L528
	subl	%edx, %eax
	cmpl	$-2147483648, %eax
	setbe	%al
	movzbl	%al, %eax
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L528:
	subl	%eax, %edx
	xorl	%eax, %eax
	cmpl	$-2147483648, %edx
	seta	%al
	popl	%ebp
	ret
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.p2align 4,,15
	.globl	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, @function
FtlFreeSysBlkQueueInit:
	pushl	%ebp
	movl	$gSysFreeQueue+8, %edx
	movl	%esp, %ebp
	movl	$512, %ecx
	pushl	%edi
	movl	%edx, %edi
	movw	%ax, gSysFreeQueue
	xorl	%eax, %eax
	rep stosl
	movw	$0, gSysFreeQueue+2
	movw	$0, gSysFreeQueue+4
	popl	%edi
	movw	$0, gSysFreeQueue+6
	popl	%ebp
	ret
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.p2align 4,,15
	.globl	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, @function
FtlFreeSysBlkQueueEmpty:
	xorl	%eax, %eax
	cmpw	$0, gSysFreeQueue+6
	pushl	%ebp
	movl	%esp, %ebp
	sete	%al
	popl	%ebp
	ret
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.p2align 4,,15
	.globl	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, @function
FtlFreeSysBlkQueueFull:
	xorl	%eax, %eax
	cmpw	$1024, gSysFreeQueue+6
	pushl	%ebp
	movl	%esp, %ebp
	sete	%al
	popl	%ebp
	ret
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.p2align 4,,15
	.globl	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, @function
FtlFreeSysBLkSort:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	xorl	%edi, %edi
	pushl	%esi
	pushl	%ebx
	subl	$16, %esp
	cmpw	$0, gSysFreeQueue+6
	je	.L534
	xorl	%ecx, %ecx
	.p2align 4,,7
	.p2align 3
.L535:
	movzwl	gSysFreeQueue+2, %eax
	movzwl	%cx, %ebx
	xorl	%edx, %edx
	movl	p_erase_count_table, %esi
	addl	$1, %ecx
	addl	%ebx, %eax
	movzwl	gSysFreeQueue+8(%eax,%eax), %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	movzwl	(%esi,%eax,2), %edx
	movl	p_sys_data_buf, %eax
	movl	%edx, (%eax,%ebx,4)
	movzwl	gSysFreeQueue+6, %edi
	cmpw	%cx, %di
	ja	.L535
.L534:
	movzwl	%di, %eax
	movw	$0, -14(%ebp)
	movl	%eax, -24(%ebp)
	.p2align 4,,7
	.p2align 3
.L536:
	movl	-24(%ebp), %eax
	movzwl	-14(%ebp), %esi
	subl	$1, %eax
	cmpl	%eax, %esi
	movl	%esi, -20(%ebp)
	jge	.L545
	movzwl	-14(%ebp), %ecx
	addl	$1, %ecx
	cmpw	%di, %cx
	movw	%cx, -16(%ebp)
	jae	.L537
	movl	p_sys_data_buf, %ecx
	movzwl	-16(%ebp), %eax
	movzwl	-14(%ebp), %edx
	movw	%di, -26(%ebp)
	.p2align 4,,7
	.p2align 3
.L539:
	movzwl	%ax, %ebx
	movzwl	%dx, %esi
	movl	(%ecx,%ebx,4), %edi
	cmpl	%edi, (%ecx,%esi,4)
	cmova	%eax, %edx
	addl	$1, %eax
	cmpw	-26(%ebp), %ax
	jb	.L539
	cmpw	%dx, -14(%ebp)
	movzwl	-26(%ebp), %edi
	je	.L537
	movl	-20(%ebp), %esi
	movzwl	%dx, %edx
	leal	(%ecx,%edx,4), %eax
	movl	(%eax), %ebx
	movl	(%ecx,%esi,4), %ecx
	movl	%ecx, (%eax)
	movl	p_sys_data_buf, %eax
	movl	%ebx, (%eax,%esi,4)
	movzwl	gSysFreeQueue+2, %eax
	addl	%eax, %edx
	addl	%esi, %eax
	movzwl	gSysFreeQueue+8(%eax,%eax), %ebx
	movzwl	gSysFreeQueue+8(%edx,%edx), %ecx
	movw	%bx, gSysFreeQueue+8(%edx,%edx)
	movw	%cx, gSysFreeQueue+8(%eax,%eax)
	movzwl	gSysFreeQueue+6, %edi
	movzwl	%di, %eax
	movl	%eax, -24(%ebp)
.L537:
	movzwl	-16(%ebp), %ecx
	movw	%cx, -14(%ebp)
	jmp	.L536
.L545:
	addl	$16, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.p2align 4,,15
	.globl	IsInFreeQueue
	.type	IsInFreeQueue, @function
IsInFreeQueue:
	movzwl	gSysFreeQueue+6, %edx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	xorl	%eax, %eax
	cmpw	$1024, %dx
	je	.L547
	movzwl	%dx, %esi
	testl	%esi, %esi
	je	.L547
	movzwl	gSysFreeQueue+2, %ecx
	movb	$1, %al
	movl	%ecx, %edx
	andl	$1023, %edx
	cmpw	%bx, gSysFreeQueue+8(%edx,%edx)
	je	.L547
	leal	1(%ecx), %edx
	leal	(%esi,%ecx), %eax
	jmp	.L548
	.p2align 4,,7
	.p2align 3
.L549:
	movl	%edx, %ecx
	addl	$1, %edx
	andl	$1023, %ecx
	cmpw	%bx, gSysFreeQueue+8(%ecx,%ecx)
	je	.L553
.L548:
	cmpl	%eax, %edx
	jne	.L549
	xorl	%eax, %eax
.L547:
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L553:
	popl	%ebx
	movl	$1, %eax
	popl	%esi
	popl	%ebp
	ret
	.size	IsInFreeQueue, .-IsInFreeQueue
	.p2align 4,,15
	.globl	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, @function
FtlFreeSysBlkQueueOut:
	movzwl	gSysFreeQueue+6, %edx
	xorl	%eax, %eax
	pushl	%ebp
	movl	%esp, %ebp
	testw	%dx, %dx
	je	.L555
	movzwl	gSysFreeQueue+2, %ecx
	subl	$1, %edx
	movzwl	gSysFreeQueue+8(%ecx,%ecx), %eax
	movw	%dx, gSysFreeQueue+6
	leal	1(%ecx), %edx
	andl	$1023, %edx
	movw	%dx, gSysFreeQueue+2
.L555:
	popl	%ebp
	ret
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.p2align 4,,15
	.globl	insert_data_list
	.type	insert_data_list, @function
insert_data_list:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$24, %esp
	cmpw	$-1, %ax
	movw	%ax, -16(%ebp)
	je	.L558
	movl	p_data_block_list_table, %ebx
	movzwl	%ax, %ecx
	leal	(%ecx,%ecx), %eax
	leal	(%eax,%ecx), %edx
	addl	%edx, %edx
	addl	%ebx, %edx
	movl	%edx, -28(%ebp)
	movw	$-1, 2(%edx)
	movw	$-1, (%edx)
	movl	p_data_block_list_head, %edx
	testl	%edx, %edx
	je	.L571
	movl	p_valid_page_count_table, %ebx
	movl	$-1, %edi
	movl	-28(%ebp), %esi
	movl	%ebx, -20(%ebp)
	movzwl	(%ebx,%ecx,2), %ebx
	movzwl	4(%esi), %ecx
	testw	%cx, %cx
	jne	.L572
.L560:
	movl	p_data_block_list_table, %ecx
	movl	%edx, %esi
	movl	p_erase_count_table, %ebx
	subl	%ecx, %esi
	sarl	%esi
	addl	%ebx, %eax
	movl	%ecx, -24(%ebp)
	imulw	$-21845, %si, %si
	movl	%ebx, -32(%ebp)
	movl	%eax, -36(%ebp)
	jmp	.L566
	.p2align 4,,7
	.p2align 3
.L562:
	cmpl	%eax, %edi
	jb	.L563
.L564:
	movzwl	(%edx), %eax
	cmpw	$-1, %ax
	je	.L573
	movl	-24(%ebp), %esi
	movzwl	%ax, %edx
	leal	(%edx,%edx,2), %edx
	leal	(%esi,%edx,2), %edx
	movl	%eax, %esi
.L566:
	movl	-20(%ebp), %eax
	movzwl	%si, %ebx
	movzwl	4(%edx), %ecx
	movzwl	(%eax,%ebx,2), %eax
	testw	%cx, %cx
	movw	%ax, -14(%ebp)
	movl	$-1, %eax
	je	.L561
	movzwl	-14(%ebp), %eax
	movzwl	%cx, %ecx
	imull	%ecx, %eax
.L561:
	cmpl	%edi, %eax
	jne	.L562
	movl	-36(%ebp), %ecx
	movzwl	(%ecx), %eax
	movl	-32(%ebp), %ecx
	cmpw	%ax, (%ecx,%ebx,2)
	jb	.L564
.L563:
	movl	-28(%ebp), %eax
	movl	-28(%ebp), %ecx
	movw	%si, (%eax)
	movzwl	2(%edx), %eax
	movw	%ax, 2(%ecx)
	cmpl	%edx, p_data_block_list_head
	je	.L574
	movzwl	2(%edx), %eax
	movl	p_data_block_list_table, %ecx
	movzwl	-16(%ebp), %esi
	leal	(%eax,%eax,2), %eax
	leal	(%ecx,%eax,2), %eax
	movw	%si, (%eax)
	movw	%si, 2(%edx)
.L558:
	addl	$24, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L572:
	movzwl	%bx, %edi
	movzwl	%cx, %ecx
	imull	%ecx, %edi
	jmp	.L560
	.p2align 4,,7
	.p2align 3
.L573:
	movl	-28(%ebp), %ecx
	xorl	%eax, %eax
	movzwl	-16(%ebp), %ebx
	movw	%si, 2(%ecx)
	movw	%bx, (%edx)
	movl	%ecx, p_data_block_list_tail
	addl	$24, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L571:
	movl	-28(%ebp), %esi
	movl	%esi, p_data_block_list_head
	jmp	.L558
.L574:
	movzwl	-16(%ebp), %ebx
	movw	%bx, 2(%edx)
	movl	%ecx, p_data_block_list_head
	jmp	.L558
	.size	insert_data_list, .-insert_data_list
	.p2align 4,,15
	.globl	INSERT_DATA_LIST
	.type	INSERT_DATA_LIST, @function
INSERT_DATA_LIST:
	pushl	%ebp
	movzwl	%ax, %eax
	movl	%esp, %ebp
	call	insert_data_list
	addw	$1, g_num_data_superblocks
	popl	%ebp
	ret
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.p2align 4,,15
	.globl	insert_free_list
	.type	insert_free_list, @function
insert_free_list:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	cmpw	$-1, %ax
	movw	%ax, -18(%ebp)
	je	.L577
	movl	p_data_block_list_table, %ecx
	movzwl	%ax, %eax
	leal	(%eax,%eax,2), %edx
	addl	%edx, %edx
	addl	%ecx, %edx
	movl	%edx, -16(%ebp)
	movw	$-1, 2(%edx)
	movw	$-1, (%edx)
	movl	p_free_data_block_list_head, %edx
	testl	%edx, %edx
	je	.L590
	movl	p_erase_count_table, %ebx
	movl	p_data_block_list_table, %edi
	movzwl	(%ebx,%eax,2), %esi
	movl	%edx, %eax
	subl	%edi, %eax
	sarl	%eax
	imull	$-1431655765, %eax, %eax
	movl	%eax, -24(%ebp)
	movzwl	-24(%ebp), %eax
	cmpw	(%ebx,%eax,2), %si
	jbe	.L585
	movzwl	(%edx), %ecx
	cmpw	$-1, %cx
	jne	.L588
	jmp	.L593
	.p2align 4,,7
	.p2align 3
.L583:
	movzwl	(%edx), %eax
	cmpw	$-1, %ax
	je	.L580
	movl	%eax, %ecx
.L588:
	movzwl	%cx, %eax
	cmpw	(%ebx,%eax,2), %si
	leal	(%eax,%eax,2), %edx
	leal	(%edi,%edx,2), %edx
	ja	.L583
.L579:
	movzwl	2(%edx), %eax
	movl	-16(%ebp), %ebx
	movw	%ax, 2(%ebx)
	movw	%cx, (%ebx)
	cmpl	p_free_data_block_list_head, %edx
	je	.L592
	movzwl	2(%edx), %eax
	movl	p_data_block_list_table, %ebx
	movzwl	-18(%ebp), %ecx
	leal	(%eax,%eax,2), %eax
	leal	(%ebx,%eax,2), %eax
	movw	%cx, (%eax)
	movw	%cx, 2(%edx)
.L577:
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L593:
	movzwl	-24(%ebp), %ecx
	.p2align 4,,7
	.p2align 3
.L580:
	movl	-16(%ebp), %eax
	movw	%cx, 2(%eax)
	movzwl	-18(%ebp), %ecx
	xorl	%eax, %eax
	movw	%cx, (%edx)
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L590:
	movl	-16(%ebp), %ebx
	movl	%ebx, p_free_data_block_list_head
	jmp	.L577
.L592:
	movzwl	-18(%ebp), %eax
	movw	%ax, 2(%edx)
	movl	%ebx, p_free_data_block_list_head
	jmp	.L577
.L585:
	movzwl	-24(%ebp), %ecx
	jmp	.L579
	.size	insert_free_list, .-insert_free_list
	.p2align 4,,15
	.globl	INSERT_FREE_LIST
	.type	INSERT_FREE_LIST, @function
INSERT_FREE_LIST:
	pushl	%ebp
	movzwl	%ax, %eax
	movl	%esp, %ebp
	call	insert_free_list
	addw	$1, g_num_free_superblocks
	popl	%ebp
	ret
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.p2align 4,,15
	.globl	List_remove_node
	.type	List_remove_node, @function
List_remove_node:
	movl	p_data_block_list_table, %ecx
	movzwl	%dx, %edx
	pushl	%ebp
	leal	(%edx,%edx,2), %edx
	movl	%esp, %ebp
	pushl	%ebx
	leal	(%ecx,%edx,2), %edx
	cmpl	%edx, (%eax)
	je	.L600
	movzwl	(%edx), %eax
	cmpw	$-1, %ax
	je	.L601
	movzwl	2(%edx), %ebx
	movzwl	%ax, %eax
	leal	(%eax,%eax,2), %eax
	movw	%bx, 2(%ecx,%eax,2)
	movzwl	2(%edx), %eax
	movl	p_data_block_list_table, %ebx
	movzwl	(%edx), %ecx
	leal	(%eax,%eax,2), %eax
	leal	(%ebx,%eax,2), %eax
	movw	%cx, (%eax)
.L598:
	movw	$-1, (%edx)
	xorl	%eax, %eax
	movw	$-1, 2(%edx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L601:
	movzwl	2(%edx), %eax
	leal	(%eax,%eax,2), %eax
	movw	$-1, (%ecx,%eax,2)
	xorl	%eax, %eax
	movw	$-1, (%edx)
	movw	$-1, 2(%edx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L600:
	movzwl	(%edx), %ebx
	cmpw	$-1, %bx
	je	.L602
	movzwl	%bx, %ebx
	leal	(%ebx,%ebx,2), %ebx
	leal	(%ecx,%ebx,2), %ecx
	movl	%ecx, (%eax)
	xorl	%eax, %eax
	movw	$-1, 2(%ecx)
	movw	$-1, (%edx)
	movw	$-1, 2(%edx)
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L602:
	movl	$0, (%eax)
	jmp	.L598
	.size	List_remove_node, .-List_remove_node
	.p2align 4,,15
	.globl	List_pop_index_node
	.type	List_pop_index_node, @function
List_pop_index_node:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	(%eax), %esi
	movl	$65535, %ebx
	testl	%esi, %esi
	je	.L604
	testw	%dx, %dx
	movl	p_data_block_list_table, %ebx
	je	.L606
	movzwl	(%esi), %ecx
	movl	p_data_block_list_table, %ebx
	cmpw	$-1, %cx
	jne	.L610
	jmp	.L606
	.p2align 4,,7
	.p2align 3
.L612:
	movzwl	(%esi), %ecx
	cmpw	$-1, %cx
	je	.L606
.L610:
	movzwl	%cx, %ecx
	subw	$1, %dx
	leal	(%ecx,%ecx,2), %ecx
	leal	(%ebx,%ecx,2), %esi
	jne	.L612
.L606:
	subl	%ebx, %esi
	sarl	%esi
	imull	$-1431655765, %esi, %esi
	movzwl	%si, %ebx
	movl	%ebx, %edx
	call	List_remove_node
.L604:
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	List_pop_index_node, .-List_pop_index_node
	.p2align 4,,15
	.globl	List_pop_head_node
	.type	List_pop_head_node, @function
List_pop_head_node:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	call	List_pop_index_node
	popl	%ebp
	ret
	.size	List_pop_head_node, .-List_pop_head_node
	.p2align 4,,15
	.globl	List_get_gc_head_node
	.type	List_get_gc_head_node, @function
List_get_gc_head_node:
	movl	p_data_block_list_head, %ecx
	movl	%eax, %edx
	movl	$65535, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	testl	%ecx, %ecx
	je	.L615
	testw	%dx, %dx
	movl	p_data_block_list_table, %ebx
	je	.L617
	movzwl	(%ecx), %eax
	movl	p_data_block_list_table, %ebx
	cmpw	$-1, %ax
	jne	.L621
	jmp	.L617
	.p2align 4,,7
	.p2align 3
.L623:
	movzwl	(%ecx), %eax
	cmpw	$-1, %ax
	je	.L617
.L621:
	movzwl	%ax, %eax
	subw	$1, %dx
	leal	(%eax,%eax,2), %eax
	leal	(%ebx,%eax,2), %ecx
	jne	.L623
.L617:
	subl	%ebx, %ecx
	sarl	%ecx
	imull	$-1431655765, %ecx, %ecx
	movzwl	%cx, %eax
.L615:
	popl	%ebx
	popl	%ebp
	ret
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.p2align 4,,15
	.globl	List_update_data_list
	.type	List_update_data_list, @function
List_update_data_list:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$16, %esp
	cmpw	%ax, g_active_superblock
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	je	.L625
	cmpw	%ax, g_buffer_superblock
	je	.L625
	cmpw	%ax, g_gc_temp_superblock
	je	.L625
	movl	p_data_block_list_table, %edx
	movzwl	%ax, %ebx
	leal	(%ebx,%ebx,2), %eax
	leal	(%edx,%eax,2), %esi
	cmpl	p_data_block_list_head, %esi
	je	.L625
	movl	p_valid_page_count_table, %ecx
	movl	$-1, %edi
	movzwl	(%ecx,%ebx,2), %eax
	movzwl	4(%esi), %ecx
	movzwl	2(%esi), %esi
	imull	%ecx, %eax
	movl	p_valid_page_count_table, %ecx
	leal	(%esi,%esi,2), %esi
	movzwl	4(%edx,%esi,2), %edx
	testl	%eax, %eax
	cmove	%edi, %eax
	imull	$-1431655765, %esi, %edi
	movzwl	%di, %edi
	movzwl	(%ecx,%edi,2), %ecx
	movl	$-1, %edi
	imull	%ecx, %edx
	testl	%edx, %edx
	cmovne	%edx, %edi
	cmpl	%edi, %eax
	jb	.L630
.L625:
	movl	-12(%ebp), %ebx
	xorl	%eax, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L630:
	movl	%ebx, %edx
	movl	$p_data_block_list_head, %eax
	call	List_remove_node
	movl	%ebx, %eax
	subw	$1, g_num_data_superblocks
	call	INSERT_DATA_LIST
	movl	-12(%ebp), %ebx
	xorl	%eax, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	List_update_data_list, .-List_update_data_list
	.p2align 4,,15
	.globl	ftl_map_blk_alloc_new_blk
	.type	ftl_map_blk_alloc_new_blk, @function
ftl_map_blk_alloc_new_blk:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%eax, %edi
	pushl	%esi
	pushl	%ebx
	movzwl	10(%eax), %edx
	movl	12(%eax), %esi
	testw	%dx, %dx
	je	.L632
	xorl	%ebx, %ebx
	cmpw	$0, (%esi)
	leal	2(%esi), %eax
	jne	.L635
	jmp	.L641
	.p2align 4,,7
	.p2align 3
.L636:
	movl	%eax, %esi
	addl	$2, %eax
	cmpw	$0, -2(%eax)
	je	.L633
.L635:
	addl	$1, %ebx
	cmpw	%dx, %bx
	jne	.L636
.L632:
	popl	%ebx
	xorl	%eax, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L641:
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L633:
	call	FtlFreeSysBlkQueueOut
	testw	%ax, %ax
	movw	%ax, (%esi)
	je	.L632
	movw	%bx, (%edi)
	xorl	%eax, %eax
	popl	%ebx
	movw	$0, 2(%edi)
	popl	%esi
	addl	$1, 28(%edi)
	addw	$1, 8(%edi)
	popl	%edi
	popl	%ebp
	ret
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.p2align 4,,15
	.globl	select_l2p_ram_region
	.type	select_l2p_ram_region, @function
select_l2p_ram_region:
	pushl	%ebp
	xorl	%eax, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %edx
	testw	%dx, %dx
	je	.L643
	movl	p_l2p_ram_map, %esi
	cmpw	$-1, (%esi)
	movl	%esi, %edi
	movl	%esi, %ecx
	jne	.L644
	jmp	.L643
	.p2align 4,,7
	.p2align 3
.L645:
	addl	$12, %ecx
	cmpw	$-1, (%ecx)
	je	.L643
.L644:
	addl	$1, %eax
	cmpw	%dx, %ax
	jne	.L645
	movl	$-2147483648, %ebx
	movl	%edx, %eax
	movl	%edi, -16(%ebp)
	xorl	%ecx, %ecx
	movl	%ebx, %edi
	.p2align 4,,7
	.p2align 3
.L647:
	movl	4(%esi), %ebx
	testl	%ebx, %ebx
	js	.L646
	cmpl	%edi, %ebx
	jae	.L646
	movl	%ebx, %edi
	movl	%ecx, %eax
.L646:
	addl	$1, %ecx
	addl	$12, %esi
	cmpw	%dx, %cx
	jne	.L647
	cmpw	%ax, %dx
	movl	-16(%ebp), %edi
	ja	.L643
	movzwl	g_l2p_last_update_region_id, %eax
	movl	$-1, %esi
	xorl	%ecx, %ecx
	movw	%dx, -18(%ebp)
	movw	%ax, -16(%ebp)
	movl	%edx, %eax
	.p2align 4,,7
	.p2align 3
.L649:
	movl	4(%edi), %ebx
	cmpl	%ebx, %esi
	jbe	.L648
	movzwl	-16(%ebp), %edx
	cmpw	%dx, (%edi)
	cmovne	%ebx, %esi
	cmovne	%ecx, %eax
.L648:
	addl	$1, %ecx
	addl	$12, %edi
	cmpw	-18(%ebp), %cx
	jne	.L649
.L643:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.p2align 4,,15
	.globl	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, @function
FtlUpdateVaildLpn:
	movzwl	FtlUpdateVaildLpnCount, %edx
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	leal	1(%edx), %ecx
	cmpw	$4, %dx
	movw	%cx, FtlUpdateVaildLpnCount
	ja	.L657
	testl	%eax, %eax
	je	.L656
.L657:
	movzwl	c_ftl_nand_data_blks_per_plane, %ecx
	movw	$0, FtlUpdateVaildLpnCount
	movl	$0, g_validLpn
	testw	%cx, %cx
	je	.L656
	movl	p_valid_page_count_table, %ebx
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L660:
	movzwl	(%ebx,%eax,2), %edx
	cmpw	$-1, %dx
	je	.L659
	movzwl	%dx, %edx
	addl	%edx, g_validLpn
.L659:
	addl	$1, %eax
	cmpw	%ax, %cx
	ja	.L660
.L656:
	popl	%ebx
	popl	%ebp
	ret
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.p2align 4,,15
	.globl	FtlL2PDataInit
	.type	FtlL2PDataInit, @function
FtlL2PDataInit:
	movl	c_ftl_nand_max_map_blks, %ecx
	xorl	%edx, %edx
	movl	p_map_block_valid_page_count, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	addl	%ecx, %ecx
	pushl	%ebx
	call	memset
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %eax
	movl	$255, %edx
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	imull	%eax, %ecx
	movl	p_l2p_map_buf, %eax
	call	memset
	cmpw	$0, c_ftl_nand_l2pmap_ram_region_num
	je	.L663
	movl	p_l2p_ram_map, %ebx
	xorl	%edx, %edx
	.p2align 4,,7
	.p2align 3
.L664:
	movzwl	%dx, %ecx
	addl	$1, %edx
	leal	(%ecx,%ecx,2), %esi
	sall	$2, %esi
	addl	%esi, %ebx
	movw	$-1, (%ebx)
	movzwl	c_ftl_nand_byte_pre_page, %eax
	movl	$0, 4(%ebx)
	movl	p_l2p_ram_map, %ebx
	imull	%ecx, %eax
	andl	$-4, %eax
	addl	p_l2p_map_buf, %eax
	cmpw	%dx, c_ftl_nand_l2pmap_ram_region_num
	movl	%eax, 8(%ebx,%esi)
	ja	.L664
.L663:
	movl	c_ftl_nand_max_map_blks, %eax
	popl	%ebx
	movw	$-1, gL2pMapInfo+2
	popl	%esi
	movw	%ax, gL2pMapInfo+10
	movzwl	g_totle_map_block, %eax
	movw	$-1, gL2pMapInfo
	movw	$-3902, gL2pMapInfo+4
	popl	%ebp
	movw	%ax, gL2pMapInfo+8
	movzwl	c_ftl_nand_map_region_num, %eax
	movw	%ax, gL2pMapInfo+6
	movl	p_map_block_table, %eax
	movl	%eax, gL2pMapInfo+12
	movl	p_map_block_ver_table, %eax
	movl	%eax, gL2pMapInfo+16
	movl	p_map_block_valid_page_count, %eax
	movl	%eax, gL2pMapInfo+20
	movl	p_map_region_ppn_table, %eax
	movl	%eax, gL2pMapInfo+24
	ret
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.p2align 4,,15
	.globl	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, @function
ftl_sb_update_avl_pages:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movzwl	c_ftl_nand_planes_num, %esi
	pushl	%ebx
	movw	$0, 4(%eax)
	cmpw	%cx, %si
	jbe	.L667
	.p2align 4,,7
	.p2align 3
.L669:
	movzwl	%cx, %ebx
	cmpw	$-1, 16(%eax,%ebx,2)
	je	.L668
	addw	$1, 4(%eax)
.L668:
	addl	$1, %ecx
	cmpw	%si, %cx
	jb	.L669
.L667:
	testw	%si, %si
	je	.L666
	movzwl	c_ftl_nand_page_pre_blk, %edi
	subl	$1, %esi
	movl	%eax, %ecx
	movzwl	%si, %esi
	notl	%edx
	leal	2(%eax,%esi,2), %esi
	.p2align 4,,7
	.p2align 3
.L672:
	cmpw	$-1, 16(%ecx)
	je	.L671
	movl	%edi, %ebx
	addw	4(%eax), %bx
	addl	%edx, %ebx
	movw	%bx, 4(%eax)
.L671:
	addl	$2, %ecx
	cmpl	%esi, %ecx
	jne	.L672
.L666:
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.p2align 4,,15
	.globl	make_superblock
	.type	make_superblock, @function
make_superblock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	subl	$8, %esp
	movw	$0, 4(%eax)
	movb	$0, 7(%eax)
	movzwl	c_ftl_nand_planes_num, %eax
	movw	%ax, -18(%ebp)
	xorl	%eax, %eax
	cmpw	$0, -18(%ebp)
	je	.L676
	movzwl	c_ftl_nand_planes_per_die, %ecx
	xorl	%ebx, %ebx
	movzwl	c_ftl_nand_blks_per_die, %eax
	movw	%cx, -16(%ebp)
	imulw	(%esi), %cx
	movw	%ax, -14(%ebp)
	movw	%cx, -20(%ebp)
	.p2align 4,,7
	.p2align 3
.L678:
	movzbl	p_plane_order_table(%ebx), %eax
	xorl	%edx, %edx
	movzwl	-14(%ebp), %edi
	movw	$-1, 16(%esi,%ebx,2)
	divw	-16(%ebp)
	addw	-20(%ebp), %dx
	imull	%eax, %edi
	addl	%edx, %edi
	xorl	%edx, %edx
	movl	%edi, %eax
	divw	-14(%ebp)
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %eax
	movl	%edx, %ecx
	shrl	%cl, %eax
	testb	$1, %al
	jne	.L677
	movw	%di, 16(%esi,%ebx,2)
	addb	$1, 7(%esi)
.L677:
	addl	$1, %ebx
	cmpw	%bx, -18(%ebp)
	ja	.L678
	movzbl	7(%esi), %eax
	imulw	c_ftl_nand_page_pre_blk, %ax
.L676:
	movl	g_inkDie_check_enable, %edi
	movw	%ax, 4(%esi)
	movb	$0, 9(%esi)
	testl	%edi, %edi
	je	.L679
	movzwl	(%esi), %edx
	movl	p_erase_count_table, %eax
	cmpw	$59, (%eax,%edx,2)
	ja	.L679
	movb	$1, 9(%esi)
.L679:
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	make_superblock, .-make_superblock
	.p2align 4,,15
	.globl	SupperBlkListInit
	.type	SupperBlkListInit, @function
SupperBlkListInit:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movzwl	c_ftl_nand_blk_pre_plane, %eax
	leal	(%eax,%eax,2), %ecx
	movl	p_data_block_list_table, %eax
	addl	%ecx, %ecx
	call	memset
	cmpw	$0, c_ftl_nand_data_blks_per_plane
	movl	$0, p_free_data_block_list_head
	movl	$0, p_data_block_list_head
	movl	$0, p_data_block_list_tail
	movw	$0, g_num_data_superblocks
	movw	$0, g_num_free_superblocks
	movw	$0, -30(%ebp)
	movw	$0, -28(%ebp)
	je	.L684
	movw	$0, -26(%ebp)
	.p2align 4,,7
	.p2align 3
.L683:
	movzwl	c_ftl_nand_planes_num, %eax
	xorl	%esi, %esi
	testw	%ax, %ax
	je	.L687
	movzwl	c_ftl_nand_blks_per_die, %ecx
	subl	$1, %eax
	movl	$p_plane_order_table, %ebx
	movzwl	c_ftl_nand_planes_per_die, %edx
	movzwl	%ax, %eax
	addl	$p_plane_order_table+1, %eax
	movl	%eax, -24(%ebp)
	movw	%cx, -14(%ebp)
	movzwl	-26(%ebp), %ecx
	movw	%dx, -16(%ebp)
	imull	%edx, %ecx
	movzwl	c_ftl_nand_page_pre_blk, %edx
	movw	%cx, -20(%ebp)
	movw	%dx, -18(%ebp)
	.p2align 4,,7
	.p2align 3
.L686:
	movzbl	(%ebx), %eax
	xorl	%edx, %edx
	divw	-16(%ebp)
	imulw	-14(%ebp), %ax
	addw	-20(%ebp), %dx
	addl	%edx, %eax
	xorl	%edx, %edx
	divw	-14(%ebp)
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %edi
	movl	%edx, %ecx
	movzwl	-18(%ebp), %eax
	shrl	%cl, %edi
	addl	%esi, %eax
	andl	$1, %edi
	cmove	%eax, %esi
	addl	$1, %ebx
	cmpl	-24(%ebp), %ebx
	jne	.L686
	testw	%si, %si
	je	.L687
	movl	$32768, %eax
	movzwl	%si, %esi
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%esi
	movl	%eax, %esi
.L687:
	movzwl	-26(%ebp), %eax
	movl	p_data_block_list_table, %ecx
	leal	(%eax,%eax,2), %edx
	leal	(%ecx,%edx,2), %edx
	movw	%si, 4(%edx)
	movzwl	-26(%ebp), %edx
	cmpw	%dx, g_active_superblock
	je	.L688
	cmpw	%dx, g_buffer_superblock
	je	.L688
	cmpw	%dx, g_gc_temp_superblock
	je	.L688
	movl	p_valid_page_count_table, %edx
	cmpw	$0, (%edx,%eax,2)
	jne	.L689
	addw	$1, -30(%ebp)
	call	INSERT_FREE_LIST
	.p2align 4,,7
	.p2align 3
.L688:
	addw	$1, -26(%ebp)
	movzwl	-26(%ebp), %ecx
	cmpw	%cx, c_ftl_nand_data_blks_per_plane
	ja	.L683
.L684:
	movzwl	-30(%ebp), %eax
	movzwl	-28(%ebp), %ecx
	movw	%ax, g_num_free_superblocks
	xorl	%eax, %eax
	movw	%cx, g_num_data_superblocks
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L689:
	addw	$1, -28(%ebp)
	call	INSERT_DATA_LIST
	jmp	.L688
	.size	SupperBlkListInit, .-SupperBlkListInit
	.p2align 4,,15
	.globl	update_multiplier_value
	.type	update_multiplier_value, @function
update_multiplier_value:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	xorl	%esi, %esi
	pushl	%ebx
	subl	$16, %esp
	movw	%ax, -26(%ebp)
	movzwl	c_ftl_nand_planes_num, %eax
	testw	%ax, %ax
	je	.L694
	movzwl	c_ftl_nand_blks_per_die, %ecx
	subl	$1, %eax
	movl	$p_plane_order_table, %ebx
	movzwl	c_ftl_nand_planes_per_die, %edx
	movzwl	%ax, %eax
	addl	$p_plane_order_table+1, %eax
	movl	%eax, -24(%ebp)
	movw	%cx, -14(%ebp)
	movzwl	-26(%ebp), %ecx
	movw	%dx, -16(%ebp)
	imull	%edx, %ecx
	movzwl	c_ftl_nand_page_pre_blk, %edx
	movw	%cx, -20(%ebp)
	movw	%dx, -18(%ebp)
	.p2align 4,,7
	.p2align 3
.L696:
	movzbl	(%ebx), %eax
	xorl	%edx, %edx
	divw	-16(%ebp)
	imulw	-14(%ebp), %ax
	addw	-20(%ebp), %dx
	addl	%edx, %eax
	xorl	%edx, %edx
	divw	-14(%ebp)
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %edi
	movl	%edx, %ecx
	movzwl	-18(%ebp), %eax
	shrl	%cl, %edi
	addl	%esi, %eax
	andl	$1, %edi
	cmove	%eax, %esi
	addl	$1, %ebx
	cmpl	-24(%ebp), %ebx
	jne	.L696
	testw	%si, %si
	je	.L694
	movl	$32768, %eax
	movzwl	%si, %ecx
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%ecx
	movl	%eax, %esi
.L694:
	movzwl	-26(%ebp), %eax
	movl	p_data_block_list_table, %ebx
	leal	(%eax,%eax,2), %eax
	leal	(%ebx,%eax,2), %eax
	movw	%si, 4(%eax)
	addl	$16, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	update_multiplier_value, .-update_multiplier_value
	.p2align 4,,15
	.globl	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, @function
GetFreeBlockMinEraseCount:
	movl	p_free_data_block_list_head, %edx
	xorl	%eax, %eax
	pushl	%ebp
	movl	%esp, %ebp
	testl	%edx, %edx
	je	.L700
	movl	%edx, %eax
	movl	p_erase_count_table, %edx
	subl	p_data_block_list_table, %eax
	sarl	%eax
	imull	$-1431655765, %eax, %eax
	movzwl	%ax, %eax
	movzwl	(%edx,%eax,2), %eax
.L700:
	popl	%ebp
	ret
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.p2align 4,,15
	.globl	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, @function
GetFreeBlockMaxEraseCount:
	movl	p_free_data_block_list_head, %edx
	xorl	%eax, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	testl	%edx, %edx
	pushl	%ebx
	je	.L703
	movzwl	g_num_free_superblocks, %eax
	movl	p_data_block_list_table, %ebx
	leal	0(,%eax,8), %esi
	subl	%eax, %esi
	movl	%edx, %eax
	subl	%ebx, %eax
	sarl	%eax
	imulw	$-21845, %ax, %ax
	shrl	$3, %esi
	testw	%si, %si
	movzwl	%ax, %eax
	je	.L705
	leal	(%eax,%eax,2), %edx
	movzwl	(%ebx,%edx,2), %edx
	cmpw	$-1, %dx
	je	.L705
	subl	$1, %esi
	xorl	%ecx, %ecx
	movzwl	%si, %esi
	jmp	.L706
	.p2align 4,,7
	.p2align 3
.L707:
	leal	(%eax,%eax,2), %edx
	addl	$1, %ecx
	movzwl	(%ebx,%edx,2), %edx
	cmpw	$-1, %dx
	je	.L705
.L706:
	cmpl	%esi, %ecx
	movzwl	%dx, %eax
	jne	.L707
.L705:
	movl	p_erase_count_table, %edx
	movzwl	(%edx,%eax,2), %eax
.L703:
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.section	.rodata.str1.1
.LC5:
	.string	"FLASH INFO:\n"
.LC6:
	.string	"FLASH ID: %x\n"
.LC7:
	.string	"Device Capacity: %d MB\n"
.LC8:
	.string	"FMWAIT: %x %x %x %x\n"
.LC9:
	.string	"FTL INFO:\n"
.LC10:
	.string	"g_MaxLpn = 0x%x\n"
.LC11:
	.string	"g_validLpn = 0x%x\n"
.LC12:
	.string	"read_page_count = 0x%x\n"
.LC13:
	.string	"discard_page_count = 0x%x\n"
.LC14:
	.string	"write_page_count = 0x%x\n"
.LC15:
	.string	"cache_write_count = 0x%x\n"
.LC16:
	.string	"l2p_write_count = 0x%x\n"
.LC17:
	.string	"gc_page_count = 0x%x\n"
.LC18:
	.string	"totle_write = %d MB\n"
.LC19:
	.string	"totle_read = %d MB\n"
.LC20:
	.string	"GSV = 0x%x\n"
.LC21:
	.string	"GDV = 0x%x\n"
.LC22:
	.string	"bad blk num = %d %d\n"
.LC23:
	.string	"free_superblocks = 0x%x\n"
.LC24:
	.string	"mlc_EC = 0x%x\n"
.LC25:
	.string	"slc_EC = 0x%x\n"
.LC26:
	.string	"avg_EC = 0x%x\n"
.LC27:
	.string	"sys_EC = 0x%x\n"
.LC28:
	.string	"max_EC = 0x%x\n"
.LC29:
	.string	"min_EC = 0x%x\n"
.LC30:
	.string	"PLT = 0x%x\n"
.LC31:
	.string	"POT = 0x%x\n"
.LC32:
	.string	"MaxSector = 0x%x\n"
.LC33:
	.string	"init_sys_blks_pp = 0x%x\n"
.LC34:
	.string	"sys_blks_pp = 0x%x\n"
.LC35:
	.string	"free sysblock = 0x%x\n"
.LC36:
	.string	"data_blks_pp = 0x%x\n"
.LC37:
	.string	"data_op_blks_pp = 0x%x\n"
.LC38:
	.string	"max_data_blks = 0x%x\n"
.LC39:
	.string	"Sys.id = 0x%x\n"
.LC40:
	.string	"Bbt.id = 0x%x\n"
.LC41:
	.string	"ACT.page = 0x%x\n"
.LC42:
	.string	"ACT.plane = 0x%x\n"
.LC43:
	.string	"ACT.id = 0x%x\n"
.LC44:
	.string	"ACT.mode = 0x%x\n"
.LC45:
	.string	"ACT.a_pages = 0x%x\n"
.LC46:
	.string	"ACT VPC = 0x%x\n"
.LC47:
	.string	"BUF.page = 0x%x\n"
.LC48:
	.string	"BUF.plane = 0x%x\n"
.LC49:
	.string	"BUF.id = 0x%x\n"
.LC50:
	.string	"BUF.mode = 0x%x\n"
.LC51:
	.string	"BUF.a_pages = 0x%x\n"
.LC52:
	.string	"BUF VPC = 0x%x\n"
.LC53:
	.string	"TMP.page = 0x%x\n"
.LC54:
	.string	"TMP.plane = 0x%x\n"
.LC55:
	.string	"TMP.id = 0x%x\n"
.LC56:
	.string	"TMP.mode = 0x%x\n"
.LC57:
	.string	"TMP.a_pages = 0x%x\n"
.LC58:
	.string	"GC.page = 0x%x\n"
.LC59:
	.string	"GC.plane = 0x%x\n"
.LC60:
	.string	"GC.id = 0x%x\n"
.LC61:
	.string	"GC.mode = 0x%x\n"
.LC62:
	.string	"GC.a_pages = 0x%x\n"
.LC63:
	.string	"WR_CHK = 0x%x %x %x %x\n"
.LC64:
	.string	"Read Err Cnt = 0x%x\n"
.LC65:
	.string	"Prog Err Cnt = 0x%x\n"
.LC66:
	.string	"gc_free_blk_th= 0x%x\n"
.LC67:
	.string	"gc_merge_free_blk_th= 0x%x\n"
.LC68:
	.string	"gc_skip_write_count= 0x%x\n"
.LC69:
	.string	"gc_blk_index= 0x%x\n"
.LC70:
	.string	"free min EC= 0x%x\n"
.LC71:
	.string	"free max EC= 0x%x\n"
.LC72:
	.string	"GC__SB VPC = 0x%x\n"
.LC73:
	.string	"%d. [0x%x]=0x%x 0x%x  0x%x\n"
.LC74:
	.string	"free %d. [0x%x] 0x%x  0x%x\n"
	.text
	.p2align 4,,15
	.globl	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, @function
FtlPrintInfo2buf:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$32, %esp
	movl	$.LC5, 4(%esp)
	movl	%eax, -16(%ebp)
	movl	%eax, (%esp)
	call	sprintf
	movl	-16(%ebp), %ebx
	movl	$.LC6, 4(%esp)
	addl	%eax, %ebx
	movl	gNandPhyInfo, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC7, 4(%esp)
	addl	%eax, %ebx
	movl	DeviceCapacity, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	addl	%eax, %ebx
	cmpl	$1, gFtlInitStatus
	je	.L710
.L718:
	movl	%ebx, %eax
	subl	-16(%ebp), %eax
	addl	$32, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L710:
	movl	gpNandc, %eax
	movl	4(%eax), %edx
	movl	(%eax), %ecx
	movl	304(%eax), %esi
	movl	308(%eax), %edi
	movl	344(%eax), %eax
	movl	%ecx, 12(%esp)
	movl	%edx, 8(%esp)
	andl	$255, %edi
	sall	$16, %eax
	orl	%edi, %eax
	movl	%ebx, (%esp)
	movl	%eax, 20(%esp)
	movl	%esi, 16(%esp)
	movl	$.LC8, 4(%esp)
	call	sprintf
	movl	$.LC9, 4(%esp)
	addl	%eax, %ebx
	movl	%ebx, (%esp)
	call	sprintf
	movl	$.LC10, 4(%esp)
	addl	%eax, %ebx
	movl	g_MaxLpn, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC11, 4(%esp)
	addl	%eax, %ebx
	movl	g_validLpn, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC12, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_read_page_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC13, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_discard_page_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC14, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_write_page_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC15, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_cache_write_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC16, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_l2p_write_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC17, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_gc_page_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC18, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_write_sector, %eax
	movl	%ebx, (%esp)
	shrl	$11, %eax
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC19, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_read_sector, %eax
	movl	%ebx, (%esp)
	shrl	$11, %eax
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC20, 4(%esp)
	addl	%eax, %ebx
	movl	g_GlobalSysVersion, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC21, 4(%esp)
	addl	%eax, %ebx
	movl	g_GlobalDataVersion, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	addl	%eax, %ebx
	call	FtlBbtCalcTotleCnt
	movl	%ebx, (%esp)
	movl	$.LC22, 4(%esp)
	movzwl	%ax, %eax
	movl	%eax, 12(%esp)
	movzwl	gBbtInfo+6, %eax
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC23, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_num_free_superblocks, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC24, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_mlc_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC25, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_slc_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC26, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_avg_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC27, 4(%esp)
	addl	%eax, %ebx
	movl	g_totle_sys_slc_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC28, 4(%esp)
	addl	%eax, %ebx
	movl	g_max_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC29, 4(%esp)
	addl	%eax, %ebx
	movl	g_min_erase_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC30, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_sys_save_data+30, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC31, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_sys_save_data+28, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC32, 4(%esp)
	addl	%eax, %ebx
	movl	g_MaxLbaSector, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC33, 4(%esp)
	addl	%eax, %ebx
	movl	c_ftl_nand_init_sys_blks_per_plane, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC34, 4(%esp)
	addl	%eax, %ebx
	movl	c_ftl_nand_sys_blks_per_plane, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC35, 4(%esp)
	addl	%eax, %ebx
	movzwl	gSysFreeQueue+6, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC36, 4(%esp)
	addl	%eax, %ebx
	movzwl	c_ftl_nand_data_blks_per_plane, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC37, 4(%esp)
	addl	%eax, %ebx
	movzwl	c_ftl_nand_data_op_blks_per_plane, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC38, 4(%esp)
	addl	%eax, %ebx
	movl	c_ftl_nand_max_data_blks, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC39, 4(%esp)
	addl	%eax, %ebx
	movzwl	gSysInfo, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC40, 4(%esp)
	addl	%eax, %ebx
	movzwl	gBbtInfo, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC41, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_active_superblock+2, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC42, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_active_superblock+6, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC43, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_active_superblock, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC44, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_active_superblock+8, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC45, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_active_superblock+4, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movzwl	g_active_superblock, %edx
	addl	%eax, %ebx
	movl	p_valid_page_count_table, %eax
	movzwl	(%eax,%edx,2), %eax
	movl	%ebx, (%esp)
	movl	$.LC46, 4(%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC47, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_buffer_superblock+2, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC48, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_buffer_superblock+6, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC49, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_buffer_superblock, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC50, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_buffer_superblock+8, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC51, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_buffer_superblock+4, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movzwl	g_buffer_superblock, %edx
	addl	%eax, %ebx
	movl	p_valid_page_count_table, %eax
	movzwl	(%eax,%edx,2), %eax
	movl	%ebx, (%esp)
	movl	$.LC52, 4(%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC53, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_temp_superblock+2, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC54, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_gc_temp_superblock+6, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC55, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_temp_superblock, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC56, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_gc_temp_superblock+8, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC57, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_temp_superblock+4, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC58, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_superblock+2, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC59, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_gc_superblock+6, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC60, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_superblock, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC61, 4(%esp)
	addl	%eax, %ebx
	movzbl	g_gc_superblock+8, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC62, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_superblock+4, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC63, 4(%esp)
	addl	%eax, %ebx
	movl	g_sys_ext_data+76, %eax
	movl	%ebx, (%esp)
	movl	%eax, 20(%esp)
	movl	g_sys_ext_data+84, %eax
	movl	%eax, 16(%esp)
	movl	g_sys_ext_data+80, %eax
	movl	%eax, 12(%esp)
	movl	g_all_blk_used_slc_mode, %eax
	sall	$8, %eax
	orl	g_inkDie_check_enable, %eax
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC64, 4(%esp)
	addl	%eax, %ebx
	movl	g_sys_ext_data+72, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC65, 4(%esp)
	addl	%eax, %ebx
	movl	g_sys_ext_data+96, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC66, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_free_blk_threshold, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC67, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_merge_free_blk_threshold, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC68, 4(%esp)
	addl	%eax, %ebx
	movl	g_gc_skip_write_count, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	movl	$.LC69, 4(%esp)
	addl	%eax, %ebx
	movzwl	g_gc_blk_index, %eax
	movl	%ebx, (%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	addl	%eax, %ebx
	movl	p_free_data_block_list_head, %eax
	xorl	%edx, %edx
	testl	%eax, %eax
	je	.L712
	subl	p_data_block_list_table, %eax
	movl	p_erase_count_table, %edx
	sarl	%eax
	imull	$-1431655765, %eax, %eax
	movzwl	%ax, %eax
	movzwl	(%edx,%eax,2), %edx
.L712:
	movl	%edx, 8(%esp)
	movl	%ebx, (%esp)
	movl	$.LC70, 4(%esp)
	call	sprintf
	addl	%eax, %ebx
	call	GetFreeBlockMaxEraseCount
	movl	%ebx, (%esp)
	movl	$.LC71, 4(%esp)
	movzwl	%ax, %eax
	movl	%eax, 8(%esp)
	call	sprintf
	addl	%eax, %ebx
	movzwl	g_gc_superblock, %eax
	cmpw	$-1, %ax
	je	.L713
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	movzwl	(%edx,%eax,2), %eax
	movl	%ebx, (%esp)
	movl	$.LC72, 4(%esp)
	movl	%eax, 8(%esp)
	call	sprintf
	addl	%eax, %ebx
.L713:
	xorl	%eax, %eax
	xorl	%esi, %esi
	call	List_get_gc_head_node
	movl	p_data_block_list_table, %edx
	cmpw	$-1, %ax
	jne	.L715
	jmp	.L716
	.p2align 4,,7
	.p2align 3
.L717:
	cmpw	$-1, %ax
	je	.L716
.L715:
	movl	p_erase_count_table, %ecx
	movzwl	%ax, %eax
	leal	(%eax,%eax,2), %edi
	movzwl	(%ecx,%eax,2), %ecx
	movl	%ecx, 24(%esp)
	movzwl	4(%edx,%edi,2), %edx
	movl	%edx, 20(%esp)
	movl	p_valid_page_count_table, %edx
	movzwl	(%edx,%eax,2), %edx
	movl	%esi, 8(%esp)
	addl	$1, %esi
	movl	%ebx, (%esp)
	movl	%eax, 12(%esp)
	movl	%edx, 16(%esp)
	movl	$.LC73, 4(%esp)
	call	sprintf
	movl	p_data_block_list_table, %edx
	addl	%eax, %ebx
	cmpl	$16, %esi
	movzwl	(%edx,%edi,2), %eax
	jne	.L717
.L716:
	movl	p_free_data_block_list_head, %eax
	xorl	%esi, %esi
	subl	%edx, %eax
	sarl	%eax
	imulw	$-21845, %ax, %ax
	cmpw	$-1, %ax
	je	.L718
.L719:
	movl	p_erase_count_table, %ecx
	movzwl	%ax, %eax
	leal	(%eax,%eax,2), %edi
	movzwl	(%ecx,%eax,2), %ecx
	movl	%ecx, 20(%esp)
	movzwl	4(%edx,%edi,2), %edx
	movl	%esi, 8(%esp)
	addl	$1, %esi
	movl	%ebx, (%esp)
	movl	%eax, 12(%esp)
	movl	%edx, 16(%esp)
	movl	$.LC74, 4(%esp)
	call	sprintf
	movl	p_data_block_list_table, %edx
	addl	%eax, %ebx
	cmpl	$4, %esi
	movzwl	(%edx,%edi,2), %eax
	je	.L718
	cmpw	$-1, %ax
	jne	.L719
	jmp	.L718
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.section	.rodata.str1.1
.LC75:
	.string	"FTL version: 5.0.27 20150715"
.LC76:
	.string	"%s\n"
	.text
	.p2align 4,,15
	.globl	rkNand_proc_ftlread
	.type	rkNand_proc_ftlread, @function
rkNand_proc_ftlread:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$20, %esp
	movl	$.LC75, 8(%esp)
	movl	%ebx, -8(%ebp)
	movl	%eax, %ebx
	movl	%esi, -4(%ebp)
	movl	$.LC76, 4(%esp)
	movl	%eax, (%esp)
	call	sprintf
	leal	(%ebx,%eax), %esi
	movl	%esi, %eax
	call	FtlPrintInfo2buf
	addl	%esi, %eax
	movl	-4(%ebp), %esi
	subl	%ebx, %eax
	movl	-8(%ebp), %ebx
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	rkNand_proc_ftlread, .-rkNand_proc_ftlread
	.section	.rodata.str1.1
.LC77:
	.string	"Totle Read %ld KB\n"
.LC78:
	.string	"Totle Write %ld KB\n"
.LC79:
	.string	"TotleWriteCount %ld\n"
.LC80:
	.string	"TotleReadCount %ld\n"
	.text
	.p2align 4,,15
	.type	nand_proc_show, @function
nand_proc_show:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	subl	$12, %esp
	movl	(%eax), %eax
	call	rkNand_proc_ftlread
	movl	%ebx, (%esp)
	movl	$.LC77, 4(%esp)
	movl	%eax, 12(%ebx)
	movl	TotleReadData, %eax
	shrl	%eax
	movl	%eax, 8(%esp)
	call	seq_printf
	movl	TotleWriteData, %eax
	movl	%ebx, (%esp)
	movl	$.LC78, 4(%esp)
	shrl	%eax
	movl	%eax, 8(%esp)
	call	seq_printf
	movl	TotleWriteCount, %eax
	movl	%ebx, (%esp)
	movl	$.LC79, 4(%esp)
	movl	%eax, 8(%esp)
	call	seq_printf
	movl	TotleReadCount, %eax
	movl	%ebx, (%esp)
	movl	$.LC80, 4(%esp)
	movl	%eax, 8(%esp)
	call	seq_printf
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_proc_show, .-nand_proc_show
	.section	.rodata.str1.4
	.align 4
.LC81:
	.string	"\0013GetSwlReplaceBlock min_ec_id =%x %x\n"
	.align 4
.LC82:
	.string	"\0013GetSwlReplaceBlock %x ,avg = %x max= %x vpc= %x,ec=%x\n"
	.text
	.p2align 4,,15
	.globl	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, @function
GetSwlReplaceBlock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$40, %esp
	movl	g_min_erase_count, %ebx
	movl	g_totle_avg_erase_count, %edx
	leal	341(%ebx), %eax
	cmpl	%edx, %eax
	movl	%edx, -24(%ebp)
	jbe	.L727
	leal	1024(%ebx), %eax
	cmpl	g_max_erase_count, %eax
	jbe	.L727
	movl	g_inkDie_check_enable, %eax
	movw	$-1, -14(%ebp)
	testl	%eax, %eax
	jne	.L747
.L728:
	movzwl	-14(%ebp), %eax
	addl	$40, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L747:
	cmpl	$30, %ebx
	ja	.L728
	.p2align 4,,7
	.p2align 3
.L727:
	call	GetFreeBlockMaxEraseCount
	movzwl	%ax, %eax
	movl	%eax, -28(%ebp)
	leal	64(%ebx), %eax
	cmpl	%eax, -28(%ebp)
	jae	.L729
	cmpl	$30, %ebx
	movw	$-1, -14(%ebp)
	ja	.L728
.L729:
	movl	p_data_block_list_head, %esi
	movw	$-1, -14(%ebp)
	testl	%esi, %esi
	je	.L728
	movzwl	(%esi), %eax
	cmpw	$-1, %ax
	je	.L728
	movl	p_erase_count_table, %edi
	movl	p_data_block_list_table, %ecx
	movw	$-1, -16(%ebp)
	movl	%edi, -20(%ebp)
	.p2align 4,,7
	.p2align 3
.L732:
	cmpw	$0, 4(%esi)
	je	.L730
	subl	%ecx, %esi
	movl	-20(%ebp), %edi
	sarl	%esi
	imull	$-1431655765, %esi, %esi
	movzwl	%si, %edx
	movzwl	(%edi,%edx,2), %edx
	movzwl	%dx, %edi
	cmpl	%edi, %ebx
	jae	.L742
	cmpw	-16(%ebp), %dx
	jae	.L730
	movw	%dx, -16(%ebp)
	movw	%si, -14(%ebp)
.L730:
	movzwl	%ax, %eax
	leal	(%eax,%eax,2), %eax
	leal	(%ecx,%eax,2), %esi
	movzwl	(%esi), %eax
	cmpw	$-1, %ax
	jne	.L732
.L731:
	cmpw	$-1, -14(%ebp)
	je	.L728
	movzwl	-14(%ebp), %edi
	movl	-20(%ebp), %eax
	movzwl	(%eax,%edi,2), %esi
	movzwl	%si, %edx
	cmpl	%edx, %ebx
	jae	.L733
	movl	p_free_data_block_list_head, %eax
	testl	%eax, %eax
	je	.L733
	subl	%ecx, %eax
	movl	-20(%ebp), %ecx
	sarl	%eax
	imull	$-1431655765, %eax, %eax
	movzwl	%ax, %eax
	movzwl	(%ecx,%eax,2), %eax
	cmpl	%eax, %ebx
	jae	.L733
	movzwl	-16(%ebp), %eax
	movl	%eax, g_min_erase_count
.L733:
	cmpw	$29, %si
	ja	.L734
	movl	g_inkDie_check_enable, %eax
	testl	%eax, %eax
	je	.L734
	leal	10(%edx), %eax
	cmpl	%eax, -24(%ebp)
	jae	.L735
	cmpw	c_mlc_erase_count_value, %si
	jae	.L734
.L735:
	cmpw	$64, g_num_data_superblocks
	ja	.L748
.L734:
	cmpl	%edx, -24(%ebp)
	ja	.L749
.L745:
	movw	$-1, -14(%ebp)
	movzwl	-14(%ebp), %eax
	addl	$40, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L749:
	leal	64(%edx), %eax
	cmpl	%eax, -28(%ebp)
	jle	.L745
	leal	341(%edx), %eax
	cmpl	%eax, -24(%ebp)
	ja	.L736
	addl	$1024, %edx
	cmpl	g_max_erase_count, %edx
	jae	.L745
.L736:
	movl	-20(%ebp), %ecx
	movl	-24(%ebp), %edx
	movzwl	(%ecx,%edi,2), %eax
	movl	%eax, 20(%esp)
	movl	p_valid_page_count_table, %eax
	movzwl	(%eax,%edi,2), %eax
	movl	%edx, 8(%esp)
	movl	%edi, 4(%esp)
	movl	$.LC82, (%esp)
	movl	%eax, 16(%esp)
	movl	g_max_erase_count, %eax
	movl	%eax, 12(%esp)
	call	printk
	movl	$1, g_in_swl_replace
	jmp	.L728
	.p2align 4,,7
	.p2align 3
.L742:
	movw	%si, -14(%ebp)
	jmp	.L731
.L748:
	movl	-20(%ebp), %edx
	movzwl	(%edx,%edi,2), %eax
	movl	%edi, 4(%esp)
	movl	$.LC81, (%esp)
	movl	%eax, 8(%esp)
	call	printk
	movl	$1, g_in_swl_replace
	jmp	.L728
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.p2align 4,,15
	.globl	free_data_superblock
	.type	free_data_superblock, @function
free_data_superblock:
	pushl	%ebp
	cmpw	$-1, %ax
	movl	%esp, %ebp
	je	.L751
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	movw	$0, (%edx,%eax,2)
	call	INSERT_FREE_LIST
.L751:
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	free_data_superblock, .-free_data_superblock
	.p2align 4,,15
	.globl	VPN_Printf_N
	.type	VPN_Printf_N, @function
VPN_Printf_N:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	VPN_Printf_N, .-VPN_Printf_N
	.p2align 4,,15
	.globl	FtlGcBufInit
	.type	FtlGcBufInit, @function
FtlGcBufInit:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	movzwl	c_ftl_nand_planes_num, %ebx
	movl	$0, g_gc_num_req
	testw	%bx, %bx
	je	.L754
	movl	gp_gc_page_buf_info, %ecx
	xorl	%edx, %edx
	movzwl	c_ftl_nand_byte_pre_page, %ebx
	jmp	.L755
	.p2align 4,,7
	.p2align 3
.L760:
	movzwl	c_ftl_nand_byte_pre_page, %ebx
.L755:
	movzwl	%dx, %eax
	movzwl	%bx, %ebx
	imull	%eax, %ebx
	leal	(%eax,%eax,2), %edi
	leal	0(,%edi,4), %esi
	movl	$1, 8(%ecx,%esi)
	testl	%ebx, %ebx
	movl	%esi, -20(%ebp)
	leal	3(%ebx), %esi
	cmovs	%esi, %ebx
	movl	-20(%ebp), %esi
	andl	$-4, %ebx
	addl	p_gc_data_buf, %ebx
	movl	%ebx, (%ecx,%edi,4)
	movzwl	c_ftl_nand_byte_pre_oob, %ebx
	movl	gp_gc_page_buf_info, %ecx
	imull	%eax, %ebx
	addl	%ecx, %esi
	leal	(%eax,%eax,8), %eax
	testl	%ebx, %ebx
	leal	3(%ebx), %edi
	cmovs	%edi, %ebx
	addl	$1, %edx
	andl	$-4, %ebx
	addl	p_gc_spare_buf, %ebx
	movl	%ebx, 4(%esi)
	movl	req_gc, %ebx
	leal	(%ebx,%eax,4), %eax
	movl	(%esi), %ebx
	movl	%ebx, 8(%eax)
	movl	4(%esi), %ebx
	movl	%ebx, 12(%eax)
	movzwl	c_ftl_nand_planes_num, %ebx
	cmpw	%dx, %bx
	ja	.L760
.L754:
	movzwl	%bx, %eax
	cmpl	c_gc_page_buf_num, %eax
	jae	.L753
	movl	gp_gc_page_buf_info, %edx
	.p2align 4,,7
	.p2align 3
.L757:
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	leal	(%eax,%eax,2), %esi
	sall	$2, %esi
	addl	%esi, %edx
	movl	$0, 8(%edx)
	imull	%eax, %ecx
	testl	%ecx, %ecx
	leal	3(%ecx), %edi
	cmovs	%edi, %ecx
	andl	$-4, %ecx
	addl	p_gc_data_buf, %ecx
	movl	%ecx, (%edx)
	movzwl	c_ftl_nand_byte_pre_oob, %ecx
	movl	gp_gc_page_buf_info, %edx
	imull	%ecx, %eax
	testl	%eax, %eax
	leal	3(%eax), %ecx
	cmovs	%ecx, %eax
	addl	$1, %ebx
	andl	$-4, %eax
	addl	p_gc_spare_buf, %eax
	movl	%eax, 4(%edx,%esi)
	movzwl	%bx, %eax
	cmpl	c_gc_page_buf_num, %eax
	jb	.L757
.L753:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlGcBufInit, .-FtlGcBufInit
	.p2align 4,,15
	.globl	FtlVariablesInit
	.type	FtlVariablesInit, @function
FtlVariablesInit:
	movzwl	c_ftl_nand_max_vendor_blks, %ecx
	xorl	%edx, %edx
	movl	p_vendor_block_table, %eax
	pushl	%ebp
	movl	%esp, %ebp
	addl	%ecx, %ecx
	pushl	%edi
	movl	$0, g_req_cache
	movw	$-1, g_tmp_data_superblock_id
	movl	$0, g_totle_swl_count
	movl	$0, ftl_gc_temp_power_lost_recovery_flag
	movl	$-1, g_recovery_page_min_ver
	movl	$0, g_inkDie_check_enable
	movw	$0, g_totle_vendor_block
	call	memset
	movzwl	c_ftl_nand_blk_pre_plane, %ecx
	xorl	%edx, %edx
	movl	p_erase_count_table, %eax
	addl	%ecx, %ecx
	call	memset
	movzwl	c_ftl_nand_blk_pre_plane, %ecx
	xorl	%edx, %edx
	movl	p_swl_mul_table, %eax
	addl	%ecx, %ecx
	call	memset
	movl	$g_sys_save_data, %edx
	xorl	%eax, %eax
	movl	%edx, %edi
	movl	$12, %ecx
	rep stosl
	movl	$g_sys_ext_data, %edx
	movl	%edx, %edi
	movb	$-128, %cl
	rep stosl
	call	FtlGcBufInit
	call	FtlL2PDataInit
	xorl	%eax, %eax
	popl	%edi
	popl	%ebp
	ret
	.size	FtlVariablesInit, .-FtlVariablesInit
	.p2align 4,,15
	.globl	FtlGcBufFree
	.type	FtlGcBufFree, @function
FtlGcBufFree:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	testl	%edx, %edx
	movl	%eax, -20(%ebp)
	movl	%edx, -16(%ebp)
	je	.L762
	movl	c_gc_page_buf_num, %ebx
	xorl	%eax, %eax
	xorl	%edi, %edi
	movl	gp_gc_page_buf_info, %esi
	.p2align 4,,7
	.p2align 3
.L764:
	testl	%ebx, %ebx
	je	.L766
	movl	-20(%ebp), %edx
	leal	(%eax,%eax,8), %eax
	movl	8(%edx,%eax,4), %ecx
	xorl	%edx, %edx
	cmpl	%ecx, (%esi)
	jne	.L765
	jmp	.L772
	.p2align 4,,7
	.p2align 3
.L767:
	leal	(%eax,%eax,2), %eax
	leal	(%esi,%eax,4), %eax
	cmpl	%ecx, (%eax)
	je	.L768
.L765:
	addl	$1, %edx
	movzwl	%dx, %eax
	cmpl	%ebx, %eax
	jb	.L767
.L766:
	addl	$1, %edi
	movzwl	%di, %eax
	cmpl	-16(%ebp), %eax
	jb	.L764
.L762:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L772:
	movl	%esi, %eax
	.p2align 4,,7
	.p2align 3
.L768:
	addl	$1, %edi
	movl	$0, 8(%eax)
	movzwl	%di, %eax
	cmpl	-16(%ebp), %eax
	jb	.L764
	jmp	.L762
	.size	FtlGcBufFree, .-FtlGcBufFree
	.p2align 4,,15
	.globl	FtlGcBufAlloc
	.type	FtlGcBufAlloc, @function
FtlGcBufAlloc:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	testl	%edx, %edx
	movl	%eax, -20(%ebp)
	movl	%edx, -16(%ebp)
	je	.L773
	movl	c_gc_page_buf_num, %ecx
	xorl	%edi, %edi
	xorl	%esi, %esi
	movl	gp_gc_page_buf_info, %ebx
	.p2align 4,,7
	.p2align 3
.L775:
	testl	%ecx, %ecx
	je	.L777
	movl	8(%ebx), %eax
	xorl	%edx, %edx
	testl	%eax, %eax
	jne	.L776
	jmp	.L783
	.p2align 4,,7
	.p2align 3
.L778:
	leal	(%eax,%eax,2), %eax
	leal	(%ebx,%eax,4), %eax
	cmpl	$0, 8(%eax)
	je	.L779
.L776:
	addl	$1, %edx
	movzwl	%dx, %eax
	cmpl	%ecx, %eax
	jb	.L778
.L777:
	addl	$1, %esi
	movzwl	%si, %edi
	cmpl	-16(%ebp), %edi
	jb	.L775
.L773:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L783:
	movl	%ebx, %eax
	.p2align 4,,7
	.p2align 3
.L779:
	leal	(%edi,%edi,8), %edx
	movl	-20(%ebp), %edi
	addl	$1, %esi
	movl	$1, 8(%eax)
	leal	(%edi,%edx,4), %edx
	movl	(%eax), %edi
	movl	%edi, 8(%edx)
	movl	4(%eax), %eax
	movzwl	%si, %edi
	cmpl	-16(%ebp), %edi
	movl	%eax, 12(%edx)
	jb	.L775
	jmp	.L773
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.p2align 4,,15
	.globl	IsBlkInGcList
	.type	IsBlkInGcList, @function
IsBlkInGcList:
	movzwl	g_gc_blk_num, %edx
	movl	%eax, %ecx
	xorl	%eax, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	testw	%dx, %dx
	je	.L785
	movl	p_gc_blk_tbl, %ebx
	movb	$1, %al
	cmpw	%cx, (%ebx)
	je	.L785
	leal	-1(%edx), %eax
	xorl	%edx, %edx
	movzwl	%ax, %eax
	addl	%eax, %eax
	jmp	.L786
	.p2align 4,,7
	.p2align 3
.L787:
	addl	$2, %edx
	cmpw	%cx, (%ebx,%edx)
	je	.L790
.L786:
	cmpl	%eax, %edx
	jne	.L787
	xorl	%eax, %eax
.L785:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L790:
	movl	$1, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	IsBlkInGcList, .-IsBlkInGcList
	.p2align 4,,15
	.globl	FtlGcUpdatePage
	.type	FtlGcUpdatePage, @function
FtlGcUpdatePage:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$8, %esp
	movzwl	g_gc_blk_num, %esi
	shrl	$10, %eax
	movl	p_gc_blk_tbl, %edi
	movl	%edx, -16(%ebp)
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	%ecx, -20(%ebp)
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	testw	%si, %si
	je	.L793
	xorl	%edx, %edx
	movl	%edi, %ecx
	cmpw	%ax, (%edi)
	jne	.L795
	jmp	.L794
	.p2align 4,,7
	.p2align 3
.L796:
	addl	$2, %ecx
	cmpw	%ax, (%ecx)
	je	.L794
.L795:
	addl	$1, %edx
	cmpw	%si, %dx
	jne	.L796
.L793:
	movzwl	%si, %esi
	movw	%ax, (%edi,%esi,2)
	addw	$1, g_gc_blk_num
.L794:
	movzwl	g_gc_page_offset, %eax
	movl	p_gc_page_info, %edx
	leal	(%eax,%eax,2), %eax
	leal	(%edx,%eax,4), %eax
	movl	-16(%ebp), %edx
	movl	%ebx, (%eax)
	addw	$1, g_gc_page_offset
	movl	%edx, 4(%eax)
	movl	-20(%ebp), %edx
	movl	%edx, 8(%eax)
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.p2align 4,,15
	.globl	FtlGcPageVarInit
	.type	FtlGcPageVarInit, @function
FtlGcPageVarInit:
	movzwl	c_ftl_nand_page_pre_super_blk, %ecx
	movl	$255, %edx
	movl	p_gc_blk_tbl, %eax
	pushl	%ebp
	movl	%esp, %ebp
	addl	%ecx, %ecx
	movw	$0, g_gc_blk_num
	movw	$0, g_gc_page_offset
	call	memset
	movzwl	c_ftl_nand_page_pre_super_blk, %eax
	movl	$255, %edx
	leal	(%eax,%eax,2), %ecx
	movl	p_gc_page_info, %eax
	sall	$2, %ecx
	call	memset
	call	FtlGcBufInit
	popl	%ebp
	ret
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.section	.rodata.str1.1
.LC83:
	.string	"\0013FtlGcRefreshBlock 0x%x\n"
	.text
	.p2align 4,,15
	.globl	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, @function
FtlGcRefreshBlock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	subl	$8, %esp
	movzwl	%ax, %eax
	movl	%eax, 4(%esp)
	movl	$.LC83, (%esp)
	call	printk
	cmpw	$-1, g_gc_next_blk
	je	.L803
	cmpw	$-1, g_gc_next_blk_1
	je	.L804
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L804:
	movw	%bx, g_gc_next_blk_1
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L803:
	movw	%bx, g_gc_next_blk
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.section	.rodata.str1.1
.LC84:
	.string	"\0013FtlGcMarkBadPhyBlk %d 0x%x\n"
	.text
	.p2align 4,,15
	.globl	FtlGcMarkBadPhyBlk
	.type	FtlGcMarkBadPhyBlk, @function
FtlGcMarkBadPhyBlk:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$12, %esp
	divw	c_ftl_nand_blks_per_die
	movl	$.LC84, (%esp)
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movl	%eax, %esi
	movzwl	%bx, %eax
	movl	%eax, 8(%esp)
	movzwl	g_gc_bad_block_temp_num, %eax
	movzwl	%si, %esi
	movl	%eax, 4(%esp)
	call	printk
	movl	%esi, %eax
	call	FtlGcRefreshBlock
	movl	g_inkDie_check_enable, %eax
	testl	%eax, %eax
	je	.L806
	movl	p_erase_count_table, %ecx
	leal	(%ecx,%esi,2), %esi
	movzwl	(%esi), %eax
	cmpw	$29, %ax
	ja	.L811
.L806:
	movzwl	g_gc_bad_block_temp_num, %ecx
	testw	%cx, %cx
	je	.L807
	cmpw	%bx, g_gc_bad_block_temp_tbl
	je	.L808
	leal	-1(%ecx), %edx
	movl	$g_gc_bad_block_temp_tbl+2, %eax
	movzwl	%dx, %edx
	leal	g_gc_bad_block_temp_tbl+2(%edx,%edx), %edx
	jmp	.L809
	.p2align 4,,7
	.p2align 3
.L810:
	addl	$2, %eax
	cmpw	%bx, -2(%eax)
	je	.L808
.L809:
	cmpl	%edx, %eax
	jne	.L810
	cmpw	$15, %cx
	ja	.L808
.L807:
	movzwl	%cx, %eax
	addl	$1, %ecx
	movw	%bx, g_gc_bad_block_temp_tbl(%eax,%eax)
	movw	%cx, g_gc_bad_block_temp_num
.L808:
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L811:
	subl	$30, %eax
	movw	%ax, (%esi)
	jmp	.L806
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.p2align 4,,15
	.globl	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, @function
FtlGcReFreshBadBlk:
	movzwl	g_gc_bad_block_temp_num, %eax
	pushl	%ebp
	movl	%esp, %ebp
	testw	%ax, %ax
	je	.L813
	cmpw	$-1, g_gc_next_blk
	je	.L816
.L813:
	xorl	%eax, %eax
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L816:
	movzwl	g_gc_bad_block_gc_index, %edx
	cmpw	%dx, %ax
	jbe	.L814
	movzwl	%dx, %edx
.L815:
	movzwl	g_gc_bad_block_temp_tbl(%edx,%edx), %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	call	FtlGcRefreshBlock
	xorl	%eax, %eax
	addw	$1, g_gc_bad_block_gc_index
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L814:
	movw	$0, g_gc_bad_block_gc_index
	xorl	%edx, %edx
	jmp	.L815
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.p2align 4,,15
	.globl	ftl_malloc
	.type	ftl_malloc, @function
ftl_malloc:
	pushl	%ebp
	movl	$209, %edx
	movl	%esp, %ebp
	call	__kmalloc
	popl	%ebp
	ret
	.size	ftl_malloc, .-ftl_malloc
	.section	.rodata.str1.4
	.align 4
.LC85:
	.string	"\0013%s error allocating memory. return -1\n"
	.text
	.p2align 4,,15
	.globl	FtlMemInit
	.type	FtlMemInit, @function
FtlMemInit:
	pushl	%ebp
	movl	$512, %eax
	movl	%esp, %ebp
	movl	%eax, %edx
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$8, %esp
	movzwl	c_ftl_nand_sec_pre_page, %ecx
	sarl	$31, %edx
	movl	$0, g_SlcPartLbaEndSector
	movl	$0, g_all_blk_used_slc_mode
	idivl	%ecx
	movl	$0, g_GlobalSysVersion
	movl	$0, g_GlobalDataVersion
	movl	$0, g_totle_gc_page_count
	movl	$0, g_totle_write_page_count
	movl	$0, g_totle_discard_page_count
	movl	$0, g_totle_cache_write_count
	movl	$0, g_totle_l2p_write_count
	movl	$0, g_totle_read_page_count
	movl	$0, g_totle_mlc_erase_count
	movl	$0, g_totle_slc_erase_count
	movl	$0, g_totle_sys_slc_erase_count
	movl	$0, g_max_erase_count
	movl	$0, g_min_erase_count
	movl	$0, g_in_gc_progress
	movl	$0, g_in_swl_replace
	movl	$65535, g_gc_head_data_block
	movl	$0, g_gc_head_data_block_count
	movl	$0, g_gc_skip_write_count
	movl	$0, g_cur_erase_blk
	movw	$-1, g_gc_next_blk
	movw	$-1, g_gc_next_blk_1
	movw	$32, g_gc_free_blk_threshold
	movw	$128, g_gc_merge_free_blk_threshold
	movw	$0, g_gc_blk_index
	movw	$0, g_gc_bad_block_temp_num
	movw	$0, g_gc_bad_block_gc_index
	movl	%eax, c_wr_page_buf_num
	movzwl	c_ftl_nand_planes_num, %edx
	sall	$2, %edx
	cmpl	%edx, %eax
	jbe	.L819
	movl	%edx, c_wr_page_buf_num
.L819:
	movzwl	c_ftl_nand_page_pre_super_blk, %eax
	movl	$0, g_wr_page_num
	addl	%eax, %eax
	call	ftl_malloc
	movl	%eax, p_gc_blk_tbl
	movzwl	c_ftl_nand_page_pre_super_blk, %eax
	leal	(%eax,%eax,2), %eax
	sall	$2, %eax
	call	ftl_malloc
	movl	%eax, p_gc_page_info
	movzwl	c_ftl_nand_planes_num, %eax
	leal	(%eax,%eax,8), %esi
	leal	0(,%esi,4), %ebx
	sall	$4, %esi
	movl	%esi, %eax
	call	ftl_malloc
	movl	%eax, req_read
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, req_gc_dst
	movl	%esi, %eax
	call	ftl_malloc
	movl	%eax, req_prgm
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, req_erase
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, req_gc
	movl	c_wr_page_buf_num, %eax
	leal	(%eax,%eax,8), %eax
	sall	$2, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_byte_pre_page, %ebx
	movl	%eax, req_wr_io
	movzwl	c_ftl_nand_planes_num, %eax
	leal	1(%eax,%eax), %eax
	movl	%eax, c_gc_page_buf_num
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_sys_data_buf
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_sys_data_buf_1
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_vendor_data_buf
	movl	c_gc_page_buf_num, %eax
	imull	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_gc_data_buf
	movl	c_wr_page_buf_num, %eax
	imull	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_wr_io_data_buf
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_io_data_buf_0
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_io_data_buf_1
	movl	c_gc_page_buf_num, %eax
	leal	(%eax,%eax,2), %eax
	sall	$2, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_byte_pre_oob, %ebx
	movl	%eax, gp_gc_page_buf_info
	movzwl	c_ftl_nand_planes_num, %eax
	imull	%eax, %ebx
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_sys_spare_buf
	leal	0(,%ebx,4), %eax
	call	ftl_malloc
	movl	%eax, p_io_spare_buf
	movzwl	c_ftl_nand_byte_pre_oob, %eax
	imull	c_gc_page_buf_num, %eax
	call	ftl_malloc
	movl	%eax, p_gc_spare_buf
	movzwl	c_ftl_nand_byte_pre_oob, %eax
	imull	c_wr_page_buf_num, %eax
	call	ftl_malloc
	movl	%eax, p_wr_io_spare_buf
	movzwl	c_ftl_nand_blk_pre_plane, %eax
	addl	%eax, %eax
	movw	%ax, g_ect_tbl_info_size
	movzwl	%ax, %eax
	call	ftl_malloc
	movl	%eax, p_swl_mul_table
	movzwl	g_ect_tbl_info_size, %eax
	addl	$547, %eax
	shrl	$9, %eax
	movw	%ax, g_ect_tbl_info_size
	sall	$9, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_blk_pre_plane, %ebx
	addl	%ebx, %ebx
	movl	%eax, gp_ect_tbl_info
	addl	$32, %eax
	movl	%eax, p_erase_count_table
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_valid_page_count_check_table
	movl	%ebx, %eax
	call	ftl_malloc
	movl	c_ftl_nand_max_map_blks, %ebx
	addl	%ebx, %ebx
	movl	%eax, p_valid_page_count_table
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_map_block_table
	movl	%ebx, %eax
	call	ftl_malloc
	movl	%eax, p_map_block_valid_page_count
	movzwl	c_ftl_nand_max_vendor_blks, %eax
	addl	%eax, %eax
	call	ftl_malloc
	movl	%eax, p_vendor_block_table
	movzwl	c_ftl_nand_max_vendor_blks, %eax
	addl	%eax, %eax
	call	ftl_malloc
	movl	%eax, p_vendor_block_valid_page_count
	movzwl	c_ftl_nand_max_vendor_blks, %eax
	sall	$2, %eax
	call	ftl_malloc
	movl	%eax, p_vendor_block_ver_table
	movzwl	c_ftl_nand_vendor_region_num, %eax
	sall	$2, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_vendor_region_num, %ecx
	xorl	%edx, %edx
	sall	$2, %ecx
	movl	%eax, p_vendor_region_ppn_table
	call	memset
	movzwl	c_ftl_nand_map_region_num, %eax
	sall	$2, %eax
	call	ftl_malloc
	movl	%eax, p_map_region_ppn_table
	movl	c_ftl_nand_max_map_blks, %eax
	sall	$2, %eax
	call	ftl_malloc
	movl	%eax, p_map_block_ver_table
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %eax
	leal	(%eax,%eax,2), %eax
	sall	$2, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_byte_pre_page, %edx
	movl	%eax, p_l2p_ram_map
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %eax
	imull	%edx, %eax
	call	ftl_malloc
	movl	%eax, p_l2p_map_buf
	movzwl	c_ftl_nand_blk_pre_plane, %eax
	leal	(%eax,%eax,2), %eax
	addl	%eax, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_die_num, %edx
	movl	%eax, p_data_block_list_table
	movzwl	c_ftl_nand_blks_per_die, %eax
	addl	$31, %eax
	sarl	$5, %eax
	movw	%ax, c_ftl_nand_bbm_buf_size
	imull	%edx, %eax
	sall	$2, %eax
	call	ftl_malloc
	movzwl	c_ftl_nand_die_num, %edx
	cmpw	$1, %dx
	movl	%eax, gBbtInfo+28
	jbe	.L855
	movzwl	c_ftl_nand_bbm_buf_size, %esi
	movzwl	%dx, %edx
	movl	$gBbtInfo+32, %ecx
	leal	gBbtInfo+28(,%edx,4), %edi
	sall	$2, %esi
	movl	%esi, %ebx
	jmp	.L823
	.p2align 4,,7
	.p2align 3
.L856:
	movl	gBbtInfo+28, %eax
.L823:
	addl	%ebx, %eax
	addl	%esi, %ebx
	movl	%eax, (%ecx)
	addl	$4, %ecx
	cmpl	%edi, %ecx
	jne	.L856
	cmpl	$7, %edx
	ja	.L857
	.p2align 4,,7
	.p2align 3
.L850:
	movl	$0, gBbtInfo+28(,%edx,4)
	addl	$1, %edx
	cmpl	$7, %edx
	jbe	.L850
.L857:
	movl	p_map_block_table, %edi
	testl	%edi, %edi
	je	.L853
	movl	p_map_block_valid_page_count, %esi
	testl	%esi, %esi
	je	.L853
	movl	p_map_region_ppn_table, %ebx
	testl	%ebx, %ebx
	je	.L853
	movl	p_map_block_ver_table, %ecx
	testl	%ecx, %ecx
	je	.L853
	movl	p_l2p_ram_map, %edx
	testl	%edx, %edx
	je	.L853
	movl	p_l2p_map_buf, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_data_block_list_table, %eax
	testl	%eax, %eax
	je	.L853
	movl	gBbtInfo+28, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_valid_page_count_table, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_gc_blk_tbl, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_gc_page_info, %eax
	testl	%eax, %eax
	je	.L853
	movl	req_read, %edi
	testl	%edi, %edi
	je	.L853
	movl	req_prgm, %esi
	testl	%esi, %esi
	je	.L853
	movl	req_erase, %ebx
	testl	%ebx, %ebx
	je	.L853
	movl	req_gc, %ecx
	testl	%ecx, %ecx
	je	.L853
	movl	req_gc_dst, %edx
	testl	%edx, %edx
	je	.L853
	movl	p_sys_data_buf, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_sys_data_buf_1, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_vendor_data_buf, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_gc_data_buf, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_io_data_buf_0, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_io_data_buf_1, %eax
	testl	%eax, %eax
	je	.L853
	movl	gp_gc_page_buf_info, %edi
	testl	%edi, %edi
	je	.L853
	movl	p_sys_spare_buf, %esi
	testl	%esi, %esi
	je	.L853
	movl	p_io_spare_buf, %ebx
	testl	%ebx, %ebx
	je	.L853
	movl	p_gc_spare_buf, %ecx
	testl	%ecx, %ecx
	je	.L853
	movl	p_erase_count_table, %edx
	testl	%edx, %edx
	je	.L853
	movl	p_swl_mul_table, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_vendor_block_table, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_vendor_block_valid_page_count, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_vendor_block_ver_table, %eax
	testl	%eax, %eax
	je	.L853
	movl	p_vendor_region_ppn_table, %eax
	testl	%eax, %eax
	je	.L853
	xorl	%eax, %eax
.L827:
	addl	$8, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L855:
	movl	$1, %edx
	jmp	.L850
.L853:
	movl	$__func__.23916, 4(%esp)
	movl	$.LC85, (%esp)
	call	printk
	movl	$-1, %eax
	jmp	.L827
	.size	FtlMemInit, .-FtlMemInit
	.p2align 4,,15
	.globl	ftl_free
	.type	ftl_free, @function
ftl_free:
	pushl	%ebp
	movl	%esp, %ebp
	call	kfree
	popl	%ebp
	ret
	.size	ftl_free, .-ftl_free
	.p2align 4,,15
	.globl	ftl_memset
	.type	ftl_memset, @function
ftl_memset:
	pushl	%ebp
	movl	%esp, %ebp
	call	memset
	popl	%ebp
	ret
	.size	ftl_memset, .-ftl_memset
	.p2align 4,,15
	.globl	ftl_memcpy
	.type	ftl_memcpy, @function
ftl_memcpy:
	pushl	%ebp
	movl	%esp, %ebp
	call	memcpy
	popl	%ebp
	ret
	.size	ftl_memcpy, .-ftl_memcpy
	.p2align 4,,15
	.globl	ftl_memcmp
	.type	ftl_memcmp, @function
ftl_memcmp:
	pushl	%ebp
	movl	%esp, %ebp
	call	memcmp
	popl	%ebp
	ret
	.size	ftl_memcmp, .-ftl_memcmp
	.p2align 4,,15
	.globl	NandcXferIrqInit
	.type	NandcXferIrqInit, @function
NandcXferIrqInit:
	pushl	%ebp
	movl	%esp, %ebp
	movl	$0, nandc0_xfer_completed_flag
	popl	%ebp
	ret
	.size	NandcXferIrqInit, .-NandcXferIrqInit
	.p2align 4,,15
	.globl	NandcRBIrqInit
	.type	NandcRBIrqInit, @function
NandcRBIrqInit:
	pushl	%ebp
	movl	%esp, %ebp
	movl	$0, nandc0_ready_completed_flag
	popl	%ebp
	ret
	.size	NandcRBIrqInit, .-NandcRBIrqInit
	.p2align 4,,15
	.globl	NandcLLPIrqInit
	.type	NandcLLPIrqInit, @function
NandcLLPIrqInit:
	pushl	%ebp
	movl	%esp, %ebp
	movl	$0, nandc0_llp_completed_flag
	popl	%ebp
	ret
	.size	NandcLLPIrqInit, .-NandcLLPIrqInit
	.p2align 4,,15
	.globl	WaitNandcLlpComp
	.type	WaitNandcLlpComp, @function
WaitNandcLlpComp:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$20, %esp
	movl	FlashWaitBusyScheduleEn, %ebx
	testl	%ebx, %ebx
	je	.L866
	movl	nandc0_llp_completed_flag, %ecx
	testl	%ecx, %ecx
	jne	.L866
	leal	-12(%ebp), %eax
	movl	$1, %ebx
	movl	%eax, -12(%ebp)
	movl	%eax, -8(%ebp)
	movl	$0, -24(%ebp)
	.p2align 4,,7
	.p2align 3
.L870:
	leal	-24(%ebp), %edx
	movl	$1, %ecx
	movl	$rk29_nandc_wait, %eax
	call	prepare_to_wait_event
	movl	nandc0_llp_completed_flag, %edx
	testl	%edx, %edx
	je	.L875
.L868:
	leal	-24(%ebp), %edx
	movl	$rk29_nandc_wait, %eax
	call	finish_wait
.L866:
	movl	$0, nandc0_llp_completed_flag
	addl	$20, %esp
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L875:
	testl	%ebx, %ebx
	je	.L868
	testl	%eax, %eax
	jne	.L868
	movl	%ebx, %eax
	.p2align 4,,5
	call	schedule_timeout
	movl	%eax, %ebx
	.p2align 4,,2
	jmp	.L870
	.size	WaitNandcLlpComp, .-WaitNandcLlpComp
	.p2align 4,,15
	.globl	WaitNandcXferComp
	.type	WaitNandcXferComp, @function
WaitNandcXferComp:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$20, %esp
	movl	FlashWaitBusyScheduleEn, %ebx
	testl	%ebx, %ebx
	je	.L877
	movl	nandc0_xfer_completed_flag, %ecx
	testl	%ecx, %ecx
	jne	.L877
	leal	-12(%ebp), %eax
	movl	$1, %ebx
	movl	%eax, -12(%ebp)
	movl	%eax, -8(%ebp)
	movl	$0, -24(%ebp)
	.p2align 4,,7
	.p2align 3
.L881:
	leal	-24(%ebp), %edx
	movl	$1, %ecx
	movl	$rk29_nandc_wait, %eax
	call	prepare_to_wait_event
	movl	nandc0_xfer_completed_flag, %edx
	testl	%edx, %edx
	je	.L886
.L879:
	leal	-24(%ebp), %edx
	movl	$rk29_nandc_wait, %eax
	call	finish_wait
.L877:
	movl	$0, nandc0_xfer_completed_flag
	addl	$20, %esp
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L886:
	testl	%ebx, %ebx
	je	.L879
	testl	%eax, %eax
	jne	.L879
	movl	%ebx, %eax
	.p2align 4,,5
	call	schedule_timeout
	movl	%eax, %ebx
	.p2align 4,,2
	jmp	.L881
	.size	WaitNandcXferComp, .-WaitNandcXferComp
	.p2align 4,,15
	.globl	NandcXferComp
	.type	NandcXferComp, @function
NandcXferComp:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L888:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L888
	cmpl	$3, gNandcVer
	movl	gpNandc, %ebx
	jbe	.L902
	movl	16(%ebx), %eax
	testb	$4, %al
	jne	.L908
	.p2align 4,,7
	.p2align 3
.L902:
	movl	8(%ebx), %eax
	movl	%eax, -24(%ebp)
	movl	-24(%ebp), %eax
	testl	$1048576, %eax
	je	.L902
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L908:
	movl	%ebx, %eax
	call	WaitNandcXferComp
	movl	16(%ebx), %eax
	movl	gpNandc, %edx
	testb	$2, %al
	je	.L901
	movl	8(%edx), %eax
	movl	%edx, %esi
	movl	gNandcVer, %ecx
	movl	gNandcEccError, %edi
	movl	%eax, -24(%ebp)
	.p2align 4,,7
	.p2align 3
.L907:
	movl	28(%ebx), %edx
	movl	-24(%ebp), %eax
	shrl	$16, %edx
	shrl	$22, %eax
	andl	$31, %edx
	andl	$63, %eax
	cmpl	%eax, %edx
	jge	.L895
	cmpl	$5, %ecx
	jbe	.L907
	movl	(%esi), %eax
	movl	%eax, -20(%ebp)
	movl	-20(%ebp), %eax
	testb	$32, %ah
	je	.L893
	movl	-20(%ebp), %eax
	testl	$131072, %eax
	jne	.L894
.L893:
	testl	%edi, %edi
	je	.L907
.L894:
	movl	$0, gNandcEccError
.L895:
	movl	gMasterInfo+32, %edi
	testl	%edi, %edi
	jne	.L909
.L897:
	movl	$0, gMasterInfo+32
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L901:
	movl	8(%edx), %eax
	movl	%eax, -24(%ebp)
	movl	-24(%ebp), %eax
	testl	$1048576, %eax
	je	.L901
	movl	gMasterInfo+32, %esi
	testl	%esi, %esi
	je	.L897
	movl	dma_ops, %ebx
	movl	-24(%ebp), %eax
	movl	gMasterInfo+8, %edx
	movl	20(%ebx), %ebx
	shrl	$22, %eax
	andl	$31, %eax
	testl	%ebx, %ebx
	je	.L897
	movzbl	%al, %ecx
	xorl	%eax, %eax
	sall	$10, %ecx
	movl	$0, 4(%esp)
	movl	$1, (%esp)
	call	*%ebx
	jmp	.L897
.L909:
	movl	dma_ops, %ebx
	movl	-24(%ebp), %eax
	movl	gMasterInfo+8, %edx
	movl	20(%ebx), %ebx
	shrl	$22, %eax
	andl	$31, %eax
	testl	%ebx, %ebx
	je	.L897
	movzbl	%al, %ecx
	xorl	%eax, %eax
	sall	$10, %ecx
	movl	$0, 4(%esp)
	movl	$2, (%esp)
	call	*%ebx
	jmp	.L897
	.size	NandcXferComp, .-NandcXferComp
	.p2align 4,,15
	.globl	NandcXferData
	.type	NandcXferData, @function
NandcXferData:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$116, %esp
	movl	8(%ebp), %edi
	movb	%dl, -113(%ebp)
	movb	%cl, -96(%ebp)
	testl	$63, %edi
	movl	%edi, %esi
	jne	.L911
	movl	12(%ebp), %esi
	testl	%esi, %esi
	je	.L950
.L912:
	movzbl	%al, %ebx
	movl	12(%ebp), %eax
	movzbl	-96(%ebp), %ecx
	movzbl	-113(%ebp), %edx
	movl	%edi, 4(%esp)
	xorl	%edi, %edi
	movl	%eax, 8(%esp)
	movl	%ebx, %eax
	movl	$0, (%esp)
	call	NandcXferStart
	movl	%ebx, %eax
	call	NandcXferComp
	cmpb	$0, -113(%ebp)
	jne	.L913
	movl	gMasterInfo+40, %ebx
	testl	%ebx, %ebx
	je	.L951
.L914:
	movzbl	-96(%ebp), %edx
	shrb	$2, %dl
	movzbl	%dl, %edx
	testl	%edx, %edx
	je	.L952
	movl	gNandcEccBits, %ecx
	testl	%ecx, %ecx
	je	.L953
	movl	gNandcVer, %eax
	subl	$1, %edx
	xorl	%edi, %edi
	movl	gpNandc, %ecx
	movl	%eax, -100(%ebp)
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L925:
	leal	8(%eax), %ebx
	movl	(%ecx,%ebx,4), %ebx
	movl	%ebx, -16(%ebp)
	movl	-16(%ebp), %ebx
	andl	$4, %ebx
	jne	.L937
	movl	-16(%ebp), %ebx
	andb	$128, %bh
	jne	.L937
	cmpl	$5, -100(%ebp)
	jbe	.L921
	movl	-16(%ebp), %ebx
	movl	-16(%ebp), %esi
	shrl	$3, %ebx
	movl	%ebx, -96(%ebp)
	movl	-16(%ebp), %ebx
	shrl	$22, %esi
	andl	$31, -96(%ebp)
	andl	$32, %esi
	orl	-96(%ebp), %esi
	shrl	$16, %ebx
	movl	%ebx, -104(%ebp)
	movl	-16(%ebp), %ebx
	andl	$31, -104(%ebp)
	shrl	$24, %ebx
	andl	$32, %ebx
	orl	-104(%ebp), %ebx
	cmpl	%ebx, %esi
	movl	-16(%ebp), %esi
	jbe	.L922
	movl	-16(%ebp), %ebx
	shrl	$3, %esi
	andl	$31, %esi
	shrl	$22, %ebx
	andl	$32, %ebx
	orl	%esi, %ebx
.L923:
	cmpl	%ebx, %edi
	cmovb	%ebx, %edi
.L959:
	cmpl	%edx, %eax
	jne	.L954
	.p2align 4,,7
	.p2align 3
.L918:
	cmpl	$5, -100(%ebp)
	jbe	.L913
	cmpb	$0, -113(%ebp)
	jne	.L913
	movl	gpNandc, %eax
	movl	(%eax), %edx
	movl	%edx, %ecx
	andl	$139264, %ecx
	cmpl	$139264, %ecx
	jne	.L913
	orl	$131072, %edx
	movl	$-1, %edi
	movl	%edx, (%eax)
.L913:
	addl	$116, %esp
	movl	%edi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L911:
	cmpb	$1, -113(%ebp)
	je	.L955
	movzbl	%al, %eax
	xorl	%edx, %edx
	movl	$2, %ecx
	xorl	%edi, %edi
	movl	%eax, -112(%ebp)
	movl	$0, 8(%esp)
	movl	$0, 4(%esp)
	movl	$0, (%esp)
	call	NandcXferStart
	movzbl	-96(%ebp), %ebx
	movl	%ebx, -104(%ebp)
	movl	-104(%ebp), %eax
	xorl	%ebx, %ebx
	testl	%eax, %eax
	je	.L956
	movl	%edi, -108(%ebp)
	movl	%esi, %edi
	jmp	.L947
	.p2align 4,,7
	.p2align 3
.L958:
	movl	-16(%ebp), %ecx
	movl	-16(%ebp), %edx
	shrl	$3, %ecx
	shrl	$22, %edx
	andl	$31, %ecx
	andl	$32, %edx
	orl	%ecx, %edx
	cmpl	%edx, -108(%ebp)
	cmovae	-108(%ebp), %edx
	movl	%edx, -108(%ebp)
.L932:
	movl	12(%ebp), %ecx
	addl	%ebx, %ebx
	xorl	%edx, %edx
	movl	%edi, (%esp)
	testl	%ecx, %ecx
	movzbl	-96(%ebp), %ecx
	cmovne	%ebx, %edx
	addl	$1024, %edi
	addl	12(%ebp), %edx
	andl	$3, %ecx
	movl	%edx, 4(%esp)
	xorl	%edx, %edx
	call	NandcCopy1KB
	cmpl	%esi, -104(%ebp)
	jbe	.L957
	movl	%esi, %ebx
.L947:
	movl	-112(%ebp), %eax
	leal	2(%ebx), %esi
	call	NandcXferComp
	movl	gpNandc, %eax
	cmpl	%esi, -104(%ebp)
	movb	%bl, -96(%ebp)
	movl	32(%eax), %edx
	movl	%edx, -16(%ebp)
	ja	.L930
.L931:
	movl	-16(%ebp), %edx
	andl	$4, %edx
	je	.L958
	movl	$-1, -108(%ebp)
	jmp	.L932
	.p2align 4,,7
	.p2align 3
.L955:
	movzbl	-96(%ebp), %ebx
	xorl	%edi, %edi
	testl	%ebx, %ebx
	movl	%ebx, -104(%ebp)
	je	.L913
	movzbl	%al, %eax
	xorl	%ebx, %ebx
	movl	%eax, -96(%ebp)
	.p2align 4,,7
	.p2align 3
.L928:
	movl	12(%ebp), %edx
	leal	(%ebx,%ebx), %eax
	movl	%ebx, %edi
	movl	%esi, (%esp)
	testl	%edx, %edx
	movl	$0, %edx
	cmove	%edx, %eax
	andl	$3, %edi
	addl	12(%ebp), %eax
	movl	%edi, %ecx
	movb	$1, %dl
	addl	$2, %ebx
	addl	$1024, %esi
	movl	%eax, 4(%esp)
	movl	gpNandc, %eax
	call	NandcCopy1KB
	movl	-96(%ebp), %eax
	movl	$2, %ecx
	movl	$1, %edx
	movl	$0, 8(%esp)
	movl	$0, 4(%esp)
	movl	%edi, (%esp)
	call	NandcXferStart
	movl	-96(%ebp), %eax
	call	NandcXferComp
	cmpl	-104(%ebp), %ebx
	jb	.L928
	addl	$116, %esp
	xorl	%edi, %edi
	popl	%ebx
	movl	%edi, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L937:
	cmpl	%edx, %eax
	movl	$-1, %edi
	je	.L918
.L954:
	addl	$1, %eax
	jmp	.L925
	.p2align 4,,7
	.p2align 3
.L930:
	movl	%ebx, %eax
	movl	$2, %ecx
	addl	$2, %eax
	xorl	%edx, %edx
	andl	$3, %eax
	movl	%eax, (%esp)
	movl	-112(%ebp), %eax
	movl	$0, 8(%esp)
	movl	$0, 4(%esp)
	call	NandcXferStart
	movl	gpNandc, %eax
	jmp	.L931
	.p2align 4,,7
	.p2align 3
.L921:
	xorl	%ebx, %ebx
	cmpl	$3, -100(%ebp)
	jbe	.L923
	movl	-16(%ebp), %ebx
	movl	-16(%ebp), %esi
	shrl	$3, %ebx
	movl	%ebx, -96(%ebp)
	movl	-16(%ebp), %ebx
	shrl	$23, %esi
	andl	$31, -96(%ebp)
	andl	$32, %esi
	orl	-96(%ebp), %esi
	shrl	$16, %ebx
	movl	%ebx, -104(%ebp)
	movl	-16(%ebp), %ebx
	andl	$31, -104(%ebp)
	shrl	$25, %ebx
	andl	$32, %ebx
	orl	-104(%ebp), %ebx
	cmpl	%ebx, %esi
	movl	-16(%ebp), %esi
	jbe	.L924
	movl	-16(%ebp), %ebx
	shrl	$3, %esi
	andl	$31, %esi
	shrl	$23, %ebx
	andl	$32, %ebx
	orl	%esi, %ebx
	jmp	.L923
	.p2align 4,,7
	.p2align 3
.L957:
	movl	gNandcVer, %edx
	movl	-108(%ebp), %edi
	movl	%edx, -100(%ebp)
	jmp	.L918
	.p2align 4,,7
	.p2align 3
.L922:
	movl	-16(%ebp), %ebx
	shrl	$16, %esi
	andl	$31, %esi
	shrl	$24, %ebx
	andl	$32, %ebx
	orl	%esi, %ebx
	cmpl	%ebx, %edi
	cmovb	%ebx, %edi
	jmp	.L959
	.p2align 4,,7
	.p2align 3
.L951:
	movzbl	-96(%ebp), %eax
	cmpl	$25, gNandcEccBits
	sbbl	%esi, %esi
	shrb	%al
	andl	$-64, %esi
	movzbl	%al, %eax
	subl	$-128, %esi
	testl	%eax, %eax
	movl	%eax, -104(%ebp)
	je	.L914
	movl	12(%ebp), %eax
	xorl	%ecx, %ecx
	xorl	%ebx, %ebx
	movl	%ecx, -100(%ebp)
	.p2align 4,,7
	.p2align 3
.L916:
	movl	gMasterInfo+24, %edx
	movl	%ebx, %edi
	addl	%esi, %ebx
	shrl	$2, %edi
	addl	$1, -100(%ebp)
	movl	(%edx,%edi,4), %edx
	movl	%edx, %edi
	shrl	$8, %edi
	movl	%edi, %ecx
	movl	%edx, %edi
	movb	%dl, (%eax)
	shrl	$16, %edi
	shrl	$24, %edx
	movb	%cl, 1(%eax)
	movl	%edi, %ecx
	movb	%dl, 3(%eax)
	movl	-104(%ebp), %edx
	movb	%cl, 2(%eax)
	addl	$4, %eax
	cmpl	%edx, -100(%ebp)
	jb	.L916
	jmp	.L914
	.p2align 4,,7
	.p2align 3
.L924:
	movl	-16(%ebp), %ebx
	shrl	$16, %esi
	andl	$31, %esi
	shrl	$25, %ebx
	andl	$32, %ebx
	orl	%esi, %ebx
	jmp	.L923
	.p2align 4,,7
	.p2align 3
.L950:
	leal	-80(%ebp), %ebx
	movl	$-1, -80(%ebp)
	movl	$-1, -76(%ebp)
	movl	$-1, -72(%ebp)
	movl	$-1, -68(%ebp)
	movl	$-1, -64(%ebp)
	movl	$-1, -60(%ebp)
	movl	$-1, -56(%ebp)
	movl	$-1, -52(%ebp)
	movl	$-1, -48(%ebp)
	movl	$-1, -44(%ebp)
	movl	$-1, -40(%ebp)
	movl	$-1, -36(%ebp)
	movl	$-1, -32(%ebp)
	movl	$-1, -28(%ebp)
	movl	$-1, -24(%ebp)
	movl	$-1, -20(%ebp)
	movl	%ebx, 12(%ebp)
	jmp	.L912
.L956:
	movl	gNandcVer, %eax
	movl	%eax, -100(%ebp)
	jmp	.L918
.L952:
	movl	gNandcVer, %ecx
	xorl	%edi, %edi
	movl	%ecx, -100(%ebp)
	jmp	.L918
.L953:
	movl	gNandcVer, %ebx
	xorl	%edi, %edi
	movl	%ebx, -100(%ebp)
	jmp	.L918
	.size	NandcXferData, .-NandcXferData
	.p2align 4,,15
	.globl	FlashProgPageDp
	.type	FlashProgPageDp, @function
FlashProgPageDp:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	subl	$24, %esp
	movzbl	gNandParaInfo+9, %eax
	movl	%ecx, -28(%ebp)
	movl	%esi, %ecx
	movzbl	%cl, %ebx
	movl	%edx, -20(%ebp)
	movb	%al, -21(%ebp)
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	gpNandc, %edi
	movl	%esi, %ecx
	movl	$1, %edx
	sall	%cl, %edx
	leal	8(%ebx), %esi
	movl	(%edi), %eax
	sall	$8, %esi
	movb	%dl, %al
	movl	-20(%ebp), %edx
	movl	%eax, (%edi)
	movl	%ebx, %eax
	call	FlashProgFirstCmd
	movzbl	-21(%ebp), %edi
	movl	$1, %edx
	movl	8(%ebp), %eax
	movl	-28(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	%ebx, %eax
	movl	%ecx, (%esp)
	movl	%edi, %ecx
	call	NandcXferData
	movl	gpNandc, %eax
	movzbl	gNandOptPara+10, %edx
	addl	%esi, %eax
	movl	%edx, 8(%eax)
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	-20(%ebp), %edx
	movl	%ebx, %eax
	addl	gBlockPageAlignSize, %edx
	call	FlashProgDpSecondCmd
	movl	16(%ebp), %eax
	movl	%edi, %ecx
	movl	$1, %edx
	movl	%eax, 4(%esp)
	movl	12(%ebp), %eax
	movl	%eax, (%esp)
	movl	%ebx, %eax
	call	NandcXferData
	movl	gpNandc, %eax
	addl	%esi, %eax
	movl	$16, 8(%eax)
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	gpNandc, %ecx
	addl	%ecx, %esi
	movl	$112, 8(%esi)
	movl	$800, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L961:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L961
	movl	(%esi), %eax
	movl	(%ecx), %edx
	andl	$1, %eax
	andl	$-131328, %edx
	movl	%edx, (%ecx)
	addl	$24, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashProgPageDp, .-FlashProgPageDp
	.p2align 4,,15
	.globl	FlashProgPage
	.type	FlashProgPage, @function
FlashProgPage:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	subl	$20, %esp
	testb	%al, %al
	movzbl	gNandParaInfo+9, %edi
	movl	%edx, -20(%ebp)
	movl	%ecx, -24(%ebp)
	jne	.L964
	movzbl	gNandIDBResBlkNum, %eax
	leal	-2(%edi), %edx
	imull	gBlockPageAlignSize, %eax
	cmpl	-20(%ebp), %eax
	cmova	%edx, %edi
.L964:
	movl	%esi, %eax
	movzbl	%al, %ebx
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	gpNandc, %edx
	movl	%esi, %ecx
	leal	8(%ebx), %esi
	sall	$8, %esi
	movl	(%edx), %eax
	movl	$1, %edx
	sall	%cl, %edx
	movb	%dl, %al
	movl	gpNandc, %edx
	movl	%eax, (%edx)
	movl	-20(%ebp), %edx
	movl	%ebx, %eax
	call	FlashProgFirstCmd
	movl	%edi, %eax
	movl	-24(%ebp), %edx
	movzbl	%al, %ecx
	movl	8(%ebp), %eax
	movl	%edx, (%esp)
	movl	$1, %edx
	movl	%eax, 4(%esp)
	movl	%ebx, %eax
	call	NandcXferData
	movl	gpNandc, %eax
	addl	%esi, %eax
	movl	$16, 8(%eax)
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	gpNandc, %ecx
	addl	%ecx, %esi
	movl	$112, 8(%esi)
	movl	$800, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L965:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L965
	movl	(%esi), %eax
	movl	(%ecx), %edx
	andl	$1, %eax
	andl	$-131328, %edx
	movl	%edx, (%ecx)
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashProgPage, .-FlashProgPage
	.p2align 4,,15
	.globl	FlashPageProgMsbFFData
	.type	FlashPageProgMsbFFData, @function
FlashPageProgMsbFFData:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%edx, %esi
	pushl	%ebx
	movl	%ecx, %ebx
	subl	$4, %esp
	movl	gpNandParaInfo, %edx
	cmpb	$5, 19(%edx)
	je	.L971
.L967:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L971:
	movzbl	%cl, %ecx
	cmpw	10(%edx), %cx
	jae	.L967
	movzbl	%bl, %edx
	movzbl	%al, %edi
	cmpw	$-1, mlcPageToSlcPageTbl(%edx,%edx)
	je	.L969
	jmp	.L967
	.p2align 4,,7
	.p2align 3
.L970:
	movzbl	%bl, %eax
	cmpw	$-1, mlcPageToSlcPageTbl(%eax,%eax)
	jne	.L967
.L969:
	movl	gFlashPageBuffer1, %eax
	movl	$16384, %ecx
	movl	$255, %edx
	call	memset
	movl	gFlashPageBuffer1, %ecx
	movzbl	%bl, %edx
	addl	%esi, %edx
	movl	%edi, %eax
	movl	$0, (%esp)
	addl	$1, %ebx
	call	FlashProgPage
	movl	gpNandParaInfo, %eax
	movzbl	%bl, %edx
	cmpw	10(%eax), %dx
	jb	.L970
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.p2align 4,,15
	.globl	FlashReadRawPage
	.type	FlashReadRawPage, @function
FlashReadRawPage:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$28, %esp
	movl	%edx, -16(%ebp)
	movzbl	gNandParaInfo+9, %edx
	testb	%al, %al
	movl	%esi, -8(%ebp)
	movl	%eax, %esi
	movl	%edi, -4(%ebp)
	movl	%ebx, -12(%ebp)
	movl	%ecx, -20(%ebp)
	movzbl	%dl, %edi
	jne	.L974
	movzbl	gNandIDBResBlkNum, %eax
	movl	$4, %edi
	imull	gBlockPageAlignSize, %eax
	cmpl	-16(%ebp), %eax
	cmovbe	%edx, %edi
.L974:
	movl	%esi, %eax
	movzbl	%al, %ebx
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	gpNandc, %edx
	movl	%esi, %ecx
	movl	(%edx), %eax
	movl	$1, %edx
	sall	%cl, %edx
	movb	%dl, %al
	movl	gpNandc, %edx
	movl	%eax, (%edx)
	movl	-16(%ebp), %edx
	movl	%ebx, %eax
	call	FlashReadCmd
	movl	%ebx, %eax
	call	NandcWaitFlashReady
	movl	8(%ebp), %eax
	xorl	%edx, %edx
	movl	-20(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	%ebx, %eax
	movl	%ecx, (%esp)
	movl	%edi, %ecx
	call	NandcXferData
	movl	gpNandc, %ecx
	movl	(%ecx), %edx
	andl	$-131328, %edx
	movl	%edx, (%ecx)
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	FlashReadRawPage, .-FlashReadRawPage
	.section	.rodata.str1.4
	.align 4
.LC86:
	.string	"\0013ReadRetry pageadd=%x  ecc=%x err=%x\n"
	.text
	.p2align 4,,15
	.globl	FlashReadPages
	.type	FlashReadPages, @function
FlashReadPages:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%eax, %edi
	pushl	%esi
	pushl	%ebx
	subl	$44, %esp
	movzbl	gNandRandomizer, %eax
	testl	%edx, %edx
	movl	%edx, -28(%ebp)
	movzbl	gNandParaInfo+9, %ebx
	movb	%al, -29(%ebp)
	jne	.L1013
.L977:
	addl	$44, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1013:
	movl	%edi, %eax
	call	NandcLlpReadPagesConfig
	movl	gpNandParaInfo, %eax
	movl	20(%edi), %esi
	movzbl	19(%eax), %eax
	subl	$1, %eax
	cmpb	$4, %al
	ja	.L978
	movzbl	gReadRetryInfo+12, %eax
	cmpb	%al, read_retry_cur_offset
	je	.L978
	movzbl	gReadRetryInfo+1, %edx
	movzbl	%al, %eax
	movl	$gReadRetryInfo+4, %ecx
	movl	%eax, (%esp)
	xorl	%eax, %eax
	call	HynixSetRRPara
.L978:
	movl	%esi, %eax
	movl	$0, gNandcEccError
	call	NandcStartLlp
	movl	gpNandc, %eax
	call	WaitNandcLlpComp
	movl	gpNandc, %edx
	.p2align 4,,7
	.p2align 3
.L979:
	movl	360(%edx), %eax
	testb	$1, %al
	je	.L979
	movzbl	%bl, %ebx
	movl	%ebx, %eax
	movl	%ebx, -16(%ebp)
	call	NandcGetBchErrorValue
	cmpl	$-1, %eax
	movl	%eax, -24(%ebp)
	je	.L980
	movl	gNandcEccError, %eax
	testl	%eax, %eax
	jne	.L980
	movl	gpNandc, %edx
	movl	%edx, -20(%ebp)
.L981:
	movl	-20(%ebp), %edx
	movl	-16(%ebp), %esi
	movl	-24(%ebp), %ebx
	movl	%edi, -20(%ebp)
	movl	368(%edx), %eax
	sall	$9, %esi
	movl	%esi, -36(%ebp)
	movl	-20(%ebp), %esi
	movl	$0, -24(%ebp)
	orl	$8, %eax
	movl	%eax, 368(%edx)
	movl	364(%edx), %eax
	andl	$-9, %eax
	movl	%eax, 364(%edx)
	movl	-28(%ebp), %edx
	leal	-9(%edx,%edx,8), %eax
	movl	%ebx, (%edi,%eax,4)
	movl	-16(%ebp), %ebx
	movl	(%esi), %eax
	movl	20(%esi), %edi
	shrl	%ebx
	cmpl	$-1, %eax
	movl	%ebx, -16(%ebp)
	je	.L988
	.p2align 4,,7
	.p2align 3
.L1015:
	cmpl	$256, %eax
	setne	%al
.L989:
	testb	%al, %al
	je	.L995
	movl	-20(%ebp), %ebx
	movl	$0, (%ebx)
.L995:
	movl	8(%edi), %eax
	testl	%eax, %eax
	jne	.L1014
.L996:
	movl	-20(%ebp), %ebx
	movl	12(%ebx), %esi
	testl	%esi, %esi
	je	.L998
	cmpl	$25, gNandcEccBits
	movl	-16(%ebp), %eax
	movl	40(%edi), %edi
	sbbl	%ebx, %ebx
	andl	$-64, %ebx
	subl	$-128, %ebx
	testl	%eax, %eax
	je	.L998
	xorl	%edx, %edx
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L1000:
	movl	%edx, %ecx
	addl	%ebx, %edx
	shrl	$2, %ecx
	movl	(%edi,%ecx,4), %ecx
	movl	%ecx, (%esi,%eax,4)
	addl	$1, %eax
	cmpl	-16(%ebp), %eax
	jne	.L1000
.L998:
	addl	$1, -24(%ebp)
	movl	-28(%ebp), %edx
	addl	$36, -20(%ebp)
	cmpl	%edx, -24(%ebp)
	je	.L977
	movl	-20(%ebp), %esi
	movl	(%esi), %eax
	movl	20(%esi), %edi
	cmpl	$-1, %eax
	jne	.L1015
.L988:
	movl	$1, gMasterInfo+40
	movl	8(%edi), %eax
	testl	%eax, %eax
	je	.L990
	movl	20(%edi), %eax
	movl	%eax, gMasterInfo+8
.L991:
	movl	-20(%ebp), %ebx
	movl	44(%edi), %eax
	movb	$0, gNandRandomizer
	movl	%eax, gMasterInfo+12
	movl	8(%ebx), %ecx
	movl	28(%ebx), %edx
	movzbl	32(%ebx), %eax
	movl	12(%ebx), %ebx
	movl	%ebx, (%esp)
	call	FlashReadRawPage
	movl	%eax, %ebx
	movzbl	-29(%ebp), %eax
	cmpl	$-1, %ebx
	movb	%al, gNandRandomizer
	je	.L992
.L1011:
	cmpl	$256, %ebx
	setne	%al
.L993:
	movl	-20(%ebp), %esi
	movl	$0, gMasterInfo+40
	movl	%ebx, (%esi)
	jmp	.L989
.L980:
	movl	gpNandc, %ebx
	movl	20(%edi), %eax
	movl	360(%ebx), %esi
	movl	%ebx, -20(%ebp)
	andl	$-64, %esi
	cmpl	%esi, 28(%eax)
	je	.L1016
	leal	36(%edi), %edx
	xorl	%eax, %eax
	movl	%edi, -36(%ebp)
	movl	-28(%ebp), %edi
	jmp	.L984
	.p2align 4,,7
	.p2align 3
.L986:
	movl	%edx, %ecx
	addl	$36, %edx
	movl	20(%ecx), %ebx
	cmpl	%esi, 28(%ebx)
	je	.L1017
.L984:
	addl	$1, %eax
	cmpl	%edi, %eax
	jne	.L986
	movl	-36(%ebp), %edi
	jmp	.L981
	.p2align 4,,7
	.p2align 3
.L1014:
	movl	dma_ops, %eax
	movl	20(%edi), %edx
	movl	20(%eax), %ebx
	testl	%ebx, %ebx
	je	.L997
	movl	-36(%ebp), %ecx
	xorl	%eax, %eax
	movl	$0, 4(%esp)
	movl	$2, (%esp)
	call	*%ebx
.L997:
	movl	$0, 8(%edi)
	jmp	.L996
	.p2align 4,,7
	.p2align 3
.L990:
	movl	36(%edi), %eax
	movl	%eax, gMasterInfo+8
	jmp	.L991
.L992:
	movl	-20(%ebp), %ebx
	movl	8(%ebx), %ecx
	movl	28(%ebx), %edx
	movzbl	32(%ebx), %eax
	movl	12(%ebx), %ebx
	movl	%ebx, (%esp)
	call	FlashReadRawPage
	cmpl	$-1, %eax
	movl	%eax, %ebx
	jne	.L1011
	movl	gpReadRetrial, %esi
	testl	%esi, %esi
	jne	.L1018
	xorl	%eax, %eax
	jmp	.L993
.L1017:
	movl	-36(%ebp), %edi
.L982:
	cmpl	%eax, -28(%ebp)
	movl	$-1, (%ecx)
	movl	$0, gNandcEccError
	jbe	.L1019
	movl	-28(%ebp), %ecx
	leal	(%eax,%eax,8), %edx
	leal	(%edi,%edx,4), %edx
	.p2align 4,,7
	.p2align 3
.L987:
	addl	$1, %eax
	movl	$-1, (%edx)
	addl	$36, %edx
	cmpl	%eax, %ecx
	ja	.L987
	movl	gpNandc, %eax
	movl	%eax, -20(%ebp)
	jmp	.L981
.L1016:
	movl	%edi, %ecx
	xorl	%eax, %eax
	jmp	.L982
.L1019:
	movl	gpNandc, %esi
	movl	%esi, -20(%ebp)
	jmp	.L981
.L1018:
	movl	-20(%ebp), %ebx
	movl	28(%ebx), %edx
	movl	8(%ebx), %ecx
	movzbl	32(%ebx), %eax
	movl	12(%ebx), %ebx
	movl	%ebx, (%esp)
	call	*%esi
	movl	-20(%ebp), %esi
	movl	%eax, %ebx
	movl	%eax, 12(%esp)
	movzbl	gNandFlashEccBits, %eax
	movl	%eax, 8(%esp)
	movl	4(%esi), %eax
	movl	$.LC86, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	cmpl	$-1, %ebx
	setne	%al
	cmpl	$256, %ebx
	setne	%dl
	andl	%edx, %eax
	jmp	.L993
	.size	FlashReadPages, .-FlashReadPages
	.p2align 4,,15
	.globl	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, @function
FtlGcScanTempBlk:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$36, %esp
	movzwl	ftl_gc_temp_block_bops_scan_page_addr, %ebx
	movl	%eax, -28(%ebp)
	movl	%edx, -44(%ebp)
	cmpw	$-1, %bx
	movw	%bx, -34(%ebp)
	je	.L1042
	testw	%bx, %bx
	je	.L1022
.L1021:
	movzwl	c_ftl_nand_page_pre_blk, %eax
	cmpl	-44(%ebp), %eax
	je	.L1022
.L1023:
	movl	-28(%ebp), %eax
	movl	$-1, -48(%ebp)
	movl	$0, -40(%ebp)
	cmpw	$-1, (%eax)
	movb	$0, 8(%eax)
	je	.L1054
.L1051:
	movzwl	c_ftl_nand_planes_num, %esi
	testw	%si, %si
	je	.L1026
.L1056:
	movl	req_read, %edx
	movzwl	c_ftl_nand_byte_pre_oob, %ebx
	movl	p_gc_spare_buf, %ecx
	movzwl	-34(%ebp), %eax
	movl	%edx, -16(%ebp)
	xorl	%edx, %edx
	movl	%ebx, -24(%ebp)
	xorl	%ebx, %ebx
	movl	%ecx, -20(%ebp)
	movl	%eax, -32(%ebp)
	movw	%si, -36(%ebp)
	.p2align 4,,7
	.p2align 3
.L1028:
	movl	-28(%ebp), %ecx
	movzwl	16(%ecx,%edx,2), %eax
	cmpw	$-1, %ax
	je	.L1027
	movl	-16(%ebp), %esi
	movzwl	%ax, %eax
	movzwl	%bx, %edi
	sall	$10, %eax
	orl	-32(%ebp), %eax
	leal	(%edi,%edi,8), %ecx
	leal	(%esi,%ecx,4), %ecx
	movl	%eax, 4(%ecx)
	movl	-24(%ebp), %eax
	movl	$0, 8(%ecx)
	imull	%edi, %eax
	testl	%eax, %eax
	leal	3(%eax), %edi
	cmovs	%edi, %eax
	addl	$1, %ebx
	andl	$-4, %eax
	addl	-20(%ebp), %eax
	movl	%eax, 12(%ecx)
.L1027:
	addl	$1, %edx
	cmpw	%dx, -36(%ebp)
	ja	.L1028
	movl	req_read, %eax
	xorl	%ecx, %ecx
	movzwl	%bx, %edx
	call	FlashReadPages
	testw	%bx, %bx
	je	.L1030
	movl	req_read, %esi
	movl	4(%esi), %edx
	movl	%edx, %edi
	shrl	$10, %edi
	movl	%edx, -20(%ebp)
	movl	%edi, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	xorl	%edx, %edx
	movl	%eax, %ecx
	movl	%edi, %eax
	divw	c_ftl_nand_planes_per_die
	movl	(%esi), %eax
	imulw	c_ftl_nand_planes_per_die, %cx
	addl	%edx, %ecx
	testl	%eax, %eax
	jne	.L1031
	movl	12(%esi), %eax
	cmpw	$-1, (%eax)
	je	.L1038
	subl	$1, %ebx
	movl	$36, %esi
	movzwl	%bx, %ebx
	leal	9(%ebx,%ebx,8), %edx
	sall	$2, %edx
	movl	%edx, -24(%ebp)
	jmp	.L1032
	.p2align 4,,7
	.p2align 3
.L1033:
	movl	req_read, %ebx
	addl	%esi, %ebx
	movl	4(%ebx), %edx
	movl	%edx, %edi
	shrl	$10, %edi
	movl	%edx, -20(%ebp)
	movl	%edi, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	xorl	%edx, %edx
	movl	%eax, %ecx
	movl	%edi, %eax
	divw	c_ftl_nand_planes_per_die
	movl	(%ebx), %eax
	imulw	c_ftl_nand_planes_per_die, %cx
	addl	%edx, %ecx
	testl	%eax, %eax
	jne	.L1031
	movl	12(%ebx), %eax
	addl	$36, %esi
	cmpw	$-1, (%eax)
	je	.L1038
.L1032:
	movl	8(%eax), %ecx
	movl	-20(%ebp), %edx
	movl	12(%eax), %eax
	call	FtlGcUpdatePage
	cmpl	-24(%ebp), %esi
	jne	.L1033
.L1030:
	addl	$1, -40(%ebp)
	movl	-40(%ebp), %ebx
	addw	$1, -34(%ebp)
	cmpl	%ebx, -44(%ebp)
	ja	.L1039
	movzwl	ftl_gc_temp_block_bops_scan_page_addr, %eax
	cmpw	$-1, %ax
	je	.L1039
	movzwl	-34(%ebp), %esi
	addl	%ebx, %eax
	cmpw	%si, c_ftl_nand_page_pre_blk
	movw	%ax, ftl_gc_temp_block_bops_scan_page_addr
	ja	.L1040
	movzwl	%si, %edx
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	jmp	.L1025
	.p2align 4,,7
	.p2align 3
.L1039:
	movzwl	-34(%ebp), %ebx
	cmpw	c_ftl_nand_page_pre_blk, %bx
	jb	.L1051
	movzwl	%bx, %edx
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	jmp	.L1025
	.p2align 4,,7
	.p2align 3
.L1031:
	movl	g_inkDie_check_enable, %edx
	testl	%edx, %edx
	jne	.L1034
	movl	-28(%ebp), %ecx
	movzwl	(%ecx), %edx
	addl	%edx, %edx
.L1035:
	cmpl	$-1, %eax
	je	.L1036
.L1037:
	movl	p_valid_page_count_table, %eax
	movw	$0, (%eax,%edx)
	movl	-28(%ebp), %edx
	movzwl	(%edx), %eax
	call	INSERT_FREE_LIST
	movl	-28(%ebp), %eax
	movw	$-1, (%eax)
	call	FtlGcPageVarInit
	movl	-28(%ebp), %eax
	cmpw	$-1, (%eax)
	movb	$0, 8(%eax)
	je	.L1055
.L1024:
	movzwl	c_ftl_nand_planes_num, %esi
	movw	$0, -34(%ebp)
	testw	%si, %si
	jne	.L1056
.L1026:
	movl	req_read, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	jmp	.L1030
	.p2align 4,,7
	.p2align 3
.L1038:
	movzwl	-34(%ebp), %edx
	movl	%ecx, %eax
	movzwl	%cx, %ecx
	movl	$1, ftl_gc_temp_power_lost_recovery_flag
.L1025:
	movl	-28(%ebp), %esi
	movzwl	-34(%ebp), %ebx
	movw	$-1, ftl_gc_temp_block_bops_scan_page_addr
	movb	%al, 6(%esi)
	movl	%esi, %eax
	movw	%bx, 2(%esi)
	call	ftl_sb_update_avl_pages
.L1040:
	movl	-48(%ebp), %eax
	addl	$36, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1034:
	movl	-28(%ebp), %ebx
	movzwl	(%ebx), %ecx
	movl	p_erase_count_table, %ebx
	leal	(%ecx,%ecx), %edx
	cmpw	$119, (%ebx,%ecx,2)
	ja	.L1035
.L1036:
	movl	-20(%ebp), %esi
	movl	%esi, -48(%ebp)
	jmp	.L1037
	.p2align 4,,7
	.p2align 3
.L1022:
	call	FtlGcPageVarInit
	.p2align 4,,2
	jmp	.L1023
	.p2align 4,,7
	.p2align 3
.L1042:
	movw	$0, -34(%ebp)
	.p2align 4,,2
	jmp	.L1021
.L1054:
	movzwl	-34(%ebp), %edx
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	jmp	.L1025
.L1055:
	xorl	%edx, %edx
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	movw	$0, -34(%ebp)
	jmp	.L1025
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.p2align 4,,15
	.globl	FtlVendorPartRead
	.type	FtlVendorPartRead, @function
FtlVendorPartRead:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%edx, %esi
	pushl	%ebx
	subl	$56, %esp
	movl	%eax, -52(%ebp)
	addl	%edx, %eax
	cmpl	$4352, %eax
	movl	%ecx, -56(%ebp)
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	$-1, -68(%ebp)
	jbe	.L1068
.L1058:
	movl	-68(%ebp), %eax
	addl	$56, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1068:
	testl	%edx, %edx
	movl	$0, -68(%ebp)
	je	.L1058
	movl	-52(%ebp), %eax
	shrl	%cl, %eax
	sall	$2, %eax
	movl	%eax, -60(%ebp)
	jmp	.L1063
	.p2align 4,,7
	.p2align 3
.L1069:
	movl	p_vendor_data_buf, %eax
	movl	$1, %edx
	movl	%ebx, %edi
	movl	%ecx, -44(%ebp)
	movl	$1, %ecx
	movl	$0, -36(%ebp)
	movl	%eax, -40(%ebp)
	leal	-48(%ebp), %eax
	call	FlashReadPages
	movl	$-1, %eax
	movzwl	-62(%ebp), %edx
	cmpl	$-1, -48(%ebp)
	cmovne	-68(%ebp), %eax
	sall	$9, %edi
	sall	$9, %edx
	movl	%edi, %ecx
	movl	%eax, -68(%ebp)
	movl	-56(%ebp), %eax
	addl	p_vendor_data_buf, %edx
	call	memcpy
.L1062:
	addl	%ebx, -52(%ebp)
	addl	%edi, -56(%ebp)
	addl	$4, -60(%ebp)
	subl	%ebx, %esi
	je	.L1058
.L1063:
	movl	p_vendor_region_ppn_table, %eax
	movl	-60(%ebp), %edx
	movzwl	c_ftl_nand_sec_pre_page, %edi
	movl	(%eax,%edx), %ecx
	xorl	%edx, %edx
	movl	-52(%ebp), %eax
	movzwl	%di, %ebx
	divl	%ebx
	movzwl	%si, %eax
	subw	%dx, %di
	movzwl	%di, %ebx
	cmpl	%esi, %ebx
	cmova	%eax, %ebx
	testl	%ecx, %ecx
	movw	%dx, -62(%ebp)
	jne	.L1069
	movl	-56(%ebp), %eax
	movl	%ebx, %edi
	xorl	%edx, %edx
	sall	$9, %edi
	movl	%edi, %ecx
	call	memset
	jmp	.L1062
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.p2align 4,,15
	.globl	Ftl_load_ext_data
	.type	Ftl_load_ext_data, @function
Ftl_load_ext_data:
	pushl	%ebp
	movl	$1, %edx
	movl	%esp, %ebp
	xorl	%eax, %eax
	subl	$48, %esp
	movl	$g_sys_ext_data, %ecx
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	call	FtlVendorPartRead
	movl	$g_sys_ext_data, %edx
	cmpl	$1179929683, g_sys_ext_data
	je	.L1074
	movl	$128, %ecx
	xorl	%eax, %eax
	movl	%edx, %edi
	xorl	%ebx, %ebx
	rep stosl
	xorl	%esi, %esi
	xorl	%edi, %edi
	movl	$1179929683, g_sys_ext_data
	movl	$0, -44(%ebp)
	movl	$0, -40(%ebp)
	movl	$0, -36(%ebp)
	movl	$0, -32(%ebp)
	movl	$0, -28(%ebp)
	movl	$0, -24(%ebp)
	movl	$0, -20(%ebp)
	movl	$0, -16(%ebp)
	movl	$0, -48(%ebp)
.L1072:
	movl	-48(%ebp), %edx
	cmpl	$305432421, -44(%ebp)
	movl	%edi, g_totle_write_sector
	movl	%esi, g_totle_read_sector
	movl	%edx, g_totle_read_page_count
	movl	-16(%ebp), %edx
	movl	%ebx, g_totle_gc_page_count
	movl	%ecx, g_totle_write_page_count
	movl	%eax, g_totle_slc_erase_count
	movl	%edx, g_totle_l2p_write_count
	movl	-20(%ebp), %edx
	movl	$0, g_SlcPartLbaEndSector
	movl	%edx, g_totle_sys_slc_erase_count
	movl	-24(%ebp), %edx
	movl	%edx, g_totle_discard_page_count
	movl	-28(%ebp), %edx
	movl	%edx, g_totle_cache_write_count
	movl	-32(%ebp), %edx
	movl	%edx, g_max_erase_count
	movl	-36(%ebp), %edx
	movl	%edx, g_min_erase_count
	movl	-40(%ebp), %edx
	movl	%edx, g_all_blk_used_slc_mode
	je	.L1075
.L1073:
	movzwl	c_mlc_erase_count_value, %edx
	imull	g_totle_mlc_erase_count, %edx
	movzwl	c_ftl_nand_data_blks_per_plane, %ecx
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	addl	%edx, %eax
	xorl	%edx, %edx
	movl	-4(%ebp), %edi
	divl	%ecx
	movl	%eax, g_totle_avg_erase_count
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1075:
	movl	$1, g_inkDie_check_enable
	jmp	.L1073
	.p2align 4,,7
	.p2align 3
.L1074:
	movl	g_sys_ext_data+20, %edx
	movl	g_sys_ext_data+16, %eax
	movl	g_sys_ext_data+88, %edi
	movl	g_sys_ext_data+92, %esi
	movl	%edx, -16(%ebp)
	movl	g_sys_ext_data+32, %edx
	movl	%eax, -48(%ebp)
	movl	g_sys_ext_data+8, %ebx
	movl	g_sys_ext_data+12, %ecx
	movl	g_sys_ext_data+28, %eax
	movl	%edx, -20(%ebp)
	movl	g_sys_ext_data+36, %edx
	movl	%edx, -24(%ebp)
	movl	g_sys_ext_data+40, %edx
	movl	%edx, -28(%ebp)
	movl	g_sys_ext_data+44, %edx
	movl	%edx, -32(%ebp)
	movl	g_sys_ext_data+48, %edx
	movl	%edx, -36(%ebp)
	movl	g_sys_ext_data+60, %edx
	movl	%edx, -40(%ebp)
	movl	g_sys_ext_data+68, %edx
	movl	%edx, -44(%ebp)
	jmp	.L1072
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.section	.rodata.str1.1
.LC87:
	.string	"no ect"
.LC88:
	.string	"\0013%s\n"
	.text
	.p2align 4,,15
	.globl	FtlLoadEctTbl
	.type	FtlLoadEctTbl, @function
FtlLoadEctTbl:
	pushl	%ebp
	movl	$64, %eax
	movl	%esp, %ebp
	subl	$8, %esp
	movzwl	g_ect_tbl_info_size, %edx
	movl	gp_ect_tbl_info, %ecx
	call	FtlVendorPartRead
	movl	gp_ect_tbl_info, %eax
	cmpl	$1112818501, (%eax)
	jne	.L1078
.L1077:
	xorl	%eax, %eax
	leave
	ret
.L1078:
	movl	$.LC87, 4(%esp)
	movl	$.LC88, (%esp)
	call	printk
	movzwl	g_ect_tbl_info_size, %ecx
	xorl	%edx, %edx
	movl	gp_ect_tbl_info, %eax
	sall	$9, %ecx
	call	memset
	jmp	.L1077
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.p2align 4,,15
	.globl	load_l2p_region
	.type	load_l2p_region, @function
load_l2p_region:
	pushl	%ebp
	movzwl	%ax, %ecx
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	movl	p_map_region_ppn_table, %eax
	movl	(%eax,%ecx,4), %eax
	testl	%eax, %eax
	je	.L1082
	movl	%eax, req_sys+4
	movl	p_l2p_ram_map, %eax
	movzwl	%dx, %edx
	leal	(%edx,%edx,2), %esi
	movl	$1, %ecx
	sall	$2, %esi
	movl	$1, %edx
	movl	8(%eax,%esi), %eax
	movl	%eax, req_sys+8
	movl	p_sys_spare_buf, %eax
	movl	%eax, req_sys+12
	movl	$req_sys, %eax
	call	FlashReadPages
	xorl	%eax, %eax
	addl	p_l2p_ram_map, %esi
	movl	$0, 4(%esi)
	movw	%bx, (%esi)
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1082:
	movl	p_l2p_ram_map, %eax
	movzwl	%dx, %edx
	leal	(%edx,%edx,2), %esi
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movl	$255, %edx
	sall	$2, %esi
	movl	8(%eax,%esi), %eax
	call	memset
	movl	p_l2p_ram_map, %eax
	movw	%bx, (%eax,%esi)
	movl	p_l2p_ram_map, %eax
	movl	$0, 4(%eax,%esi)
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	load_l2p_region, .-load_l2p_region
	.p2align 4,,15
	.globl	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, @function
FtlGetLastWrittenPage:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$112, %esp
	cmpl	$1, %edx
	movl	%eax, -120(%ebp)
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	cmovne	c_ftl_nand_page_pre_blk, %ax
	sall	$10, -120(%ebp)
	subl	$1, %eax
	movw	%ax, -114(%ebp)
	movswl	%ax, %edi
	movl	-120(%ebp), %eax
	movl	%edx, -124(%ebp)
	movl	-124(%ebp), %ecx
	leal	-112(%ebp), %edx
	movl	%edx, -36(%ebp)
	movl	$1, %edx
	movl	$0, -40(%ebp)
	orl	%edi, %eax
	movl	%eax, -44(%ebp)
	leal	-48(%ebp), %eax
	call	FlashReadPages
	cmpl	$-1, -112(%ebp)
	je	.L1094
.L1087:
	addl	$112, %esp
	movl	%edi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1094:
	xorl	%esi, %esi
	cmpw	$0, -114(%ebp)
	jns	.L1092
	jmp	.L1087
	.p2align 4,,7
	.p2align 3
.L1089:
	leal	1(%ebx), %esi
.L1090:
	cmpw	%si, -114(%ebp)
	jl	.L1087
.L1092:
	movswl	%si, %ebx
	movl	-124(%ebp), %ecx
	movl	$1, %edx
	addl	%edi, %ebx
	movl	%ebx, %eax
	shrl	$31, %eax
	addl	%eax, %ebx
	shrl	%ebx
	movswl	%bx, %eax
	orl	-120(%ebp), %eax
	movl	%eax, -44(%ebp)
	leal	-48(%ebp), %eax
	call	FlashReadPages
	cmpl	$-1, -112(%ebp)
	jne	.L1089
	subl	$1, %ebx
	movw	%bx, -114(%ebp)
	movswl	%bx, %edi
	jmp	.L1090
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.p2align 4,,15
	.globl	FtlLoadSysInfo
	.type	FtlLoadSysInfo, @function
FtlLoadSysInfo:
	movl	p_sys_spare_buf, %eax
	xorl	%edx, %edx
	movzwl	c_ftl_nand_data_blks_per_plane, %ecx
	pushl	%ebp
	movl	%esp, %ebp
	movl	%eax, req_sys+12
	movl	p_valid_page_count_table, %eax
	pushl	%edi
	addl	%ecx, %ecx
	pushl	%esi
	pushl	%ebx
	movl	$-1, %ebx
	movl	$0, req_sys+8
	call	memset
	movzwl	gSysInfo, %eax
	cmpw	$-1, %ax
	je	.L1096
	movl	$1, %edx
	movzwl	%ax, %eax
	call	FtlGetLastWrittenPage
	movl	%eax, %edx
	testw	%ax, %ax
	leal	1(%edx), %ecx
	movw	%cx, gSysInfo+2
	js	.L1097
	movzwl	%ax, %edi
	movl	%eax, %esi
	addl	$1, %edi
	xorl	%ebx, %ebx
	jmp	.L1098
	.p2align 4,,7
	.p2align 3
.L1109:
	addl	$1, %ebx
	cmpl	%edi, %ebx
	je	.L1097
.L1098:
	movzwl	gSysInfo, %edx
	movl	%esi, %eax
	movl	$1, %ecx
	subw	%bx, %ax
	cwtl
	sall	$10, %edx
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	p_sys_data_buf, %eax
	movl	%eax, req_sys+8
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1109
.L1097:
	movl	req_sys+8, %edx
	movl	$-1, %ebx
	movzwl	c_ftl_nand_data_blks_per_plane, %ecx
	movl	(%edx), %eax
	addl	%ecx, %ecx
	movl	%eax, g_sys_save_data
	movl	4(%edx), %eax
	movl	%eax, g_sys_save_data+4
	movl	8(%edx), %eax
	movl	%eax, g_sys_save_data+8
	movl	12(%edx), %eax
	movl	%eax, g_sys_save_data+12
	movl	16(%edx), %eax
	movl	%eax, g_sys_save_data+16
	movl	20(%edx), %eax
	movl	%eax, g_sys_save_data+20
	movl	24(%edx), %eax
	movl	%eax, g_sys_save_data+24
	movl	28(%edx), %eax
	movl	%eax, g_sys_save_data+28
	movl	32(%edx), %eax
	movl	%eax, g_sys_save_data+32
	movl	36(%edx), %eax
	movl	%eax, g_sys_save_data+36
	movl	40(%edx), %eax
	movl	%eax, g_sys_save_data+40
	movl	44(%edx), %eax
	addl	$48, %edx
	movl	%eax, g_sys_save_data+44
	movl	p_valid_page_count_table, %eax
	call	memcpy
	cmpl	$1179929683, g_sys_save_data
	jne	.L1096
	movzwl	g_sys_save_data+8, %eax
	movzbl	g_sys_save_data+10, %edx
	cmpw	c_ftl_nand_die_num, %dx
	movw	%ax, gSysInfo+6
	jne	.L1096
	movzwl	%ax, %ebx
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movzwl	gBbtInfo+6, %edx
	movl	%ebx, g_MaxLbn
	imull	%ebx, %eax
	movw	$-1, g_gc_superblock
	movw	$0, g_gc_superblock+2
	movb	$0, g_gc_superblock+6
	movl	%eax, g_MaxLpn
	sall	%cl, %eax
	movzwl	c_ftl_nand_planes_num, %ecx
	movl	%eax, g_MaxLbaSector
	movl	c_ftl_nand_max_data_blks, %eax
	movb	$0, g_gc_superblock+8
	subl	%edx, %eax
	xorl	%edx, %edx
	subl	%ebx, %eax
	divl	%ecx
	movzwl	g_sys_save_data+14, %edx
	movw	%dx, g_active_superblock
	movw	%ax, c_ftl_nand_data_op_blks_per_plane
	movzwl	g_sys_save_data+16, %eax
	movl	%eax, %ecx
	andl	$63, %eax
	shrw	$6, %cx
	movw	%cx, g_active_superblock+2
	movzwl	g_sys_save_data+20, %ecx
	movb	%al, g_active_superblock+6
	movzbl	g_sys_save_data+11, %eax
	movl	%ecx, %ebx
	andl	$63, %ecx
	movb	%cl, g_buffer_superblock+6
	movzbl	g_sys_save_data+12, %ecx
	shrw	$6, %bx
	movw	%bx, g_buffer_superblock+2
	movb	%al, g_active_superblock+8
	movzwl	g_sys_save_data+18, %eax
	movb	%cl, g_buffer_superblock+8
	movzwl	g_sys_save_data+22, %ecx
	movw	%ax, g_buffer_superblock
	movw	%cx, g_gc_temp_superblock
	movzwl	g_sys_save_data+24, %ecx
	movl	%ecx, %ebx
	andl	$63, %ecx
	movb	%cl, g_gc_temp_superblock+6
	movzbl	g_sys_save_data+13, %ecx
	shrw	$6, %bx
	movw	%bx, g_gc_temp_superblock+2
	movb	%cl, g_gc_temp_superblock+8
	movl	g_sys_save_data+32, %ecx
	movl	$0, g_totle_gc_page_count
	movl	$0, g_totle_write_page_count
	movl	$0, g_totle_read_page_count
	movl	%ecx, g_totle_mlc_erase_count
	movl	g_sys_save_data+40, %ecx
	cmpl	g_GlobalSysVersion, %ecx
	movl	$0, g_totle_l2p_write_count
	movl	$0, g_totle_slc_erase_count
	movl	$0, g_max_erase_count
	movl	$0, g_totle_cache_write_count
	jbe	.L1099
	movl	%ecx, g_GlobalSysVersion
.L1099:
	movl	g_sys_save_data+36, %ecx
	cmpl	g_GlobalDataVersion, %ecx
	jbe	.L1100
	movl	%ecx, g_GlobalDataVersion
.L1100:
	cmpw	$-1, %dx
	je	.L1101
	movl	$g_active_superblock, %eax
	call	make_superblock
	movzwl	g_buffer_superblock, %eax
.L1101:
	cmpw	$-1, %ax
	je	.L1102
	movl	$g_buffer_superblock, %eax
	call	make_superblock
.L1102:
	cmpw	$-1, g_gc_temp_superblock
	je	.L1103
	movl	$g_gc_temp_superblock, %eax
	call	make_superblock
.L1103:
	xorl	%ebx, %ebx
	cmpw	$-1, g_gc_superblock
	je	.L1096
	movl	$g_gc_superblock, %eax
	call	make_superblock
.L1096:
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.p2align 4,,15
	.globl	FtlLoadBbt
	.type	FtlLoadBbt, @function
FtlLoadBbt:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	movl	p_sys_spare_buf, %esi
	movl	$0, req_sys+8
	movl	%esi, req_sys+12
	call	FtlBbtMemInit
	movzwl	c_ftl_nand_blks_per_die, %edx
	leal	-1(%edx), %ebx
	subl	$48, %edx
	movzwl	%bx, %eax
	cmpl	%edx, %eax
	jg	.L1129
	jmp	.L1134
	.p2align 4,,7
	.p2align 3
.L1113:
	cmpw	$-3887, (%esi)
	je	.L1137
.L1114:
	movzwl	c_ftl_nand_blks_per_die, %edx
	subl	$1, %ebx
	movzwl	%bx, %eax
	subl	$48, %edx
	cmpl	%edx, %eax
	jle	.L1134
.L1129:
	sall	$10, %eax
	movl	$1, %ecx
	movl	%eax, req_sys+4
	movl	$1, %edx
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	jne	.L1113
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	addl	$1, req_sys+4
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1114
	jmp	.L1113
	.p2align 4,,7
	.p2align 3
.L1134:
	movzwl	gBbtInfo, %ebx
.L1112:
	cmpw	$-1, %bx
	je	.L1125
	movzwl	gBbtInfo+4, %eax
	cmpw	$-1, %ax
	je	.L1117
	movzwl	%ax, %eax
	movl	$1, %ecx
	sall	$10, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1136
	cmpw	$-3887, (%esi)
	je	.L1119
.L1136:
	movzwl	gBbtInfo, %ebx
.L1117:
	movl	$1, %edx
	movzwl	%bx, %eax
	call	FtlGetLastWrittenPage
	movl	%eax, %edx
	testw	%ax, %ax
	leal	1(%edx), %ecx
	movw	%cx, gBbtInfo+2
	js	.L1121
	movl	%eax, %edi
	movzwl	%ax, %eax
	addl	$1, %eax
	xorl	%ebx, %ebx
	movl	%esi, -16(%ebp)
	movl	%eax, %esi
	jmp	.L1122
	.p2align 4,,7
	.p2align 3
.L1138:
	addl	$1, %ebx
	cmpl	%esi, %ebx
	je	.L1133
.L1122:
	movzwl	gBbtInfo, %edx
	movl	%edi, %eax
	movl	$1, %ecx
	subw	%bx, %ax
	cwtl
	sall	$10, %edx
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	p_sys_data_buf, %eax
	movl	%eax, req_sys+8
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1138
.L1133:
	movl	-16(%ebp), %esi
.L1121:
	movzwl	12(%esi), %edx
	movzwl	10(%esi), %eax
	cmpw	$-1, %dx
	movw	%ax, gBbtInfo+6
	je	.L1123
	movl	c_ftl_nand_sys_blks_per_plane, %ebx
	movzwl	%dx, %eax
	cmpl	%ebx, %eax
	je	.L1123
	movzwl	c_ftl_nand_blk_pre_plane, %ecx
	shrw	$2, %cx
	movzwl	%cx, %esi
	cmpl	%esi, %ebx
	jb	.L1139
.L1123:
	xorl	%eax, %eax
	cmpw	$0, c_ftl_nand_die_num
	je	.L1116
	movzwl	c_ftl_nand_bbm_buf_size, %esi
	xorl	%ebx, %ebx
	jmp	.L1124
	.p2align 4,,7
	.p2align 3
.L1140:
	movzwl	c_ftl_nand_bbm_buf_size, %esi
.L1124:
	movzwl	%si, %esi
	movl	gBbtInfo+28(,%ebx,4), %eax
	leal	0(,%ebx,4), %edx
	addl	$1, %ebx
	imull	%esi, %edx
	addl	req_sys+8, %edx
	leal	0(,%esi,4), %ecx
	call	memcpy
	movzwl	c_ftl_nand_die_num, %eax
	cmpl	%ebx, %eax
	ja	.L1140
	xorl	%eax, %eax
.L1116:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1139:
	cmpw	%cx, %dx
	jae	.L1123
	call	FtlSysBlkNumInit
	.p2align 4,,5
	jmp	.L1123
	.p2align 4,,7
	.p2align 3
.L1137:
	movl	4(%esi), %eax
	movw	%bx, gBbtInfo
	movl	%eax, gBbtInfo+8
	movzwl	8(%esi), %eax
	movw	%ax, gBbtInfo+4
	jmp	.L1112
.L1119:
	movl	4(%esi), %eax
	cmpl	gBbtInfo+8, %eax
	jbe	.L1136
	movzwl	gBbtInfo+4, %ebx
	movl	%eax, gBbtInfo+8
	movzwl	8(%esi), %eax
	movw	%bx, gBbtInfo
	movw	%ax, gBbtInfo+4
	jmp	.L1117
.L1125:
	movl	$-1, %eax
	jmp	.L1116
	.size	FtlLoadBbt, .-FtlLoadBbt
	.p2align 4,,15
	.globl	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, @function
FtlLoadFactoryBbt:
	pushl	%ebp
	movl	p_sys_data_buf, %eax
	movl	%esp, %ebp
	pushl	%edi
	movl	p_sys_spare_buf, %edi
	cmpw	$0, c_ftl_nand_die_num
	pushl	%esi
	pushl	%ebx
	movl	%eax, req_sys+8
	movl	%edi, req_sys+12
	je	.L1142
	xorl	%esi, %esi
	.p2align 4,,7
	.p2align 3
.L1146:
	movzwl	c_ftl_nand_blks_per_die, %eax
	movw	$-1, gBbtInfo+12(%esi,%esi)
	leal	-1(%eax), %ebx
	movzwl	%bx, %edx
	leal	-16(%eax), %ecx
	cmpl	%ecx, %edx
	jle	.L1143
	.p2align 4,,7
	.p2align 3
.L1147:
	imull	%esi, %eax
	movl	$1, %ecx
	addl	%edx, %eax
	movl	$1, %edx
	sall	$10, %eax
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1144
	cmpw	$-3872, (%edi)
	je	.L1150
.L1144:
	movzwl	c_ftl_nand_blks_per_die, %eax
	subl	$1, %ebx
	movzwl	%bx, %edx
	leal	-16(%eax), %ecx
	cmpl	%ecx, %edx
	jg	.L1147
.L1143:
	movzwl	c_ftl_nand_die_num, %eax
	addl	$1, %esi
	cmpl	%esi, %eax
	ja	.L1146
.L1142:
	popl	%ebx
	xorl	%eax, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1150:
	movzwl	c_ftl_nand_die_num, %eax
	movw	%bx, gBbtInfo+12(%esi,%esi)
	addl	$1, %esi
	cmpl	%esi, %eax
	ja	.L1146
	jmp	.L1142
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.p2align 4,,15
	.globl	HynixReadRetrial
	.type	HynixReadRetrial, @function
HynixReadRetrial:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$24, %esp
	movzbl	gReadRetryInfo+12(%eax), %ebx
	movl	%eax, -16(%ebp)
	movzbl	gReadRetryInfo+2, %eax
	movl	%edx, -28(%ebp)
	movl	%ecx, -20(%ebp)
	movb	%al, -21(%ebp)
	movl	-16(%ebp), %eax
	call	NandcWaitFlashReady
	movzbl	-21(%ebp), %eax
	testl	%eax, %eax
	je	.L1152
	addl	$1, %eax
	movl	$1, %edi
	movl	%eax, -32(%ebp)
	movl	$-1, %esi
	.p2align 4,,7
	.p2align 3
.L1157:
	addl	$1, %ebx
	cmpb	%bl, -21(%ebp)
	movzbl	%bl, %eax
	ja	.L1153
	xorl	%eax, %eax
	xorl	%ebx, %ebx
.L1153:
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	movl	%eax, (%esp)
	movl	-16(%ebp), %eax
	call	HynixSetRRPara
	movl	8(%ebp), %edx
	movl	-20(%ebp), %ecx
	movl	-16(%ebp), %eax
	movl	%edx, (%esp)
	movl	-28(%ebp), %edx
	call	FlashReadRawPage
	cmpl	$-1, %eax
	je	.L1154
	movzbl	gNandFlashEccBits, %edx
	cmpl	$-1, %esi
	cmove	%eax, %esi
	leal	(%edx,%edx,2), %edx
	sarl	$2, %edx
	cmpl	%edx, %eax
	jb	.L1156
	movl	$0, 8(%ebp)
	movl	$0, -20(%ebp)
.L1154:
	addl	$1, %edi
	cmpl	-32(%ebp), %edi
	jne	.L1157
	movzbl	gNandFlashEccBits, %eax
	cmpl	$-1, %esi
	movl	-16(%ebp), %edx
	leal	(%eax,%eax,2), %eax
	movb	%bl, gReadRetryInfo+12(%edx)
	je	.L1158
	sarl	$2, %eax
	cmpl	%eax, %esi
	movl	$256, %eax
	cmovae	%eax, %esi
.L1158:
	addl	$24, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1156:
	movl	-16(%ebp), %edx
	movl	%eax, %esi
	movl	%esi, %eax
	movb	%bl, gReadRetryInfo+12(%edx)
	addl	$24, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1152:
	movl	-16(%ebp), %eax
	movl	$-1, %esi
	movb	%bl, gReadRetryInfo+12(%eax)
	jmp	.L1158
	.size	HynixReadRetrial, .-HynixReadRetrial
	.p2align 4,,15
	.globl	MicronReadRetrial
	.type	MicronReadRetrial, @function
MicronReadRetrial:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%edi
	movl	$-1, %edi
	pushl	%esi
	xorl	%esi, %esi
	pushl	%ebx
	subl	$20, %esp
	movl	%edx, -28(%ebp)
	movl	%ecx, -20(%ebp)
	movl	%eax, -24(%ebp)
	call	NandcWaitFlashReady
	movl	-24(%ebp), %ebx
	addl	$8, %ebx
	sall	$8, %ebx
	addl	gpNandc, %ebx
	.p2align 4,,7
	.p2align 3
.L1167:
	movl	$239, 8(%ebx)
	movl	$137, 4(%ebx)
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L1163:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L1163
	movl	8(%ebp), %eax
	addl	$1, %esi
	movl	%esi, (%ebx)
	movl	-20(%ebp), %ecx
	movl	$0, (%ebx)
	movl	-28(%ebp), %edx
	movl	$0, (%ebx)
	movl	$0, (%ebx)
	movl	%eax, (%esp)
	movl	-24(%ebp), %eax
	call	FlashReadRawPage
	cmpl	$-1, %eax
	je	.L1164
	movzbl	gNandFlashEccBits, %edx
	cmpl	$-1, %edi
	cmove	%eax, %edi
	leal	(%edx,%edx,2), %edx
	sarl	$2, %edx
	cmpl	%edx, %eax
	jb	.L1171
	movl	$0, 8(%ebp)
	movl	$0, -20(%ebp)
.L1164:
	cmpl	$7, %esi
	jne	.L1167
.L1166:
	movl	$239, 8(%ebx)
	movl	$137, 4(%ebx)
	movl	$2000, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L1168:
	movl	-16(%ebp), %eax
	leal	-1(%eax), %edx
	testl	%eax, %eax
	movl	%edx, -16(%ebp)
	jne	.L1168
	cmpl	$-1, %edi
	movl	$0, (%ebx)
	movl	$0, (%ebx)
	movl	$0, (%ebx)
	movl	$0, (%ebx)
	je	.L1169
	movzbl	gNandFlashEccBits, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%edi, %eax
	movl	$256, %eax
	cmovbe	%eax, %edi
.L1169:
	addl	$20, %esp
	movl	%edi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1171:
	movl	%eax, %edi
	jmp	.L1166
	.size	MicronReadRetrial, .-MicronReadRetrial
	.p2align 4,,15
	.globl	SamsungReadRetrial
	.type	SamsungReadRetrial, @function
SamsungReadRetrial:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movl	8(%ebp), %edi
	movl	%edx, -28(%ebp)
	movl	%ecx, -16(%ebp)
	movl	%eax, -24(%ebp)
	call	NandcWaitFlashReady
	movl	-24(%ebp), %eax
	addl	$8, %eax
	sall	$8, %eax
	movl	%eax, -20(%ebp)
	movl	gpNandc, %eax
	addl	%eax, -20(%ebp)
	cmpb	$0, g_maxRetryCount
	je	.L1176
	movl	$1, %ebx
	movl	$-1, %esi
	.p2align 4,,7
	.p2align 3
.L1180:
	movl	-20(%ebp), %eax
	movzbl	%bl, %edx
	call	SamsungSetRRPara
	movl	-16(%ebp), %ecx
	movl	-28(%ebp), %edx
	movl	-24(%ebp), %eax
	movl	%edi, (%esp)
	call	FlashReadRawPage
	cmpl	$-1, %eax
	je	.L1177
	movzbl	gNandFlashEccBits, %edx
	cmpl	$-1, %esi
	cmove	%eax, %esi
	leal	(%edx,%edx,2), %edx
	sarl	$2, %edx
	cmpl	%edx, %eax
	jb	.L1183
	xorl	%edi, %edi
	movl	$0, -16(%ebp)
.L1177:
	movzbl	g_maxRetryCount, %eax
	addl	$1, %ebx
	addl	$1, %eax
	cmpl	%ebx, %eax
	ja	.L1180
.L1179:
	movl	-20(%ebp), %eax
	xorl	%edx, %edx
	call	SamsungSetRRPara
	cmpl	$-1, %esi
	je	.L1181
	movzbl	gNandFlashEccBits, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%esi, %eax
	movl	$256, %eax
	cmovbe	%eax, %esi
.L1181:
	addl	$20, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1183:
	movl	%eax, %esi
	jmp	.L1179
.L1176:
	movl	-20(%ebp), %eax
	xorl	%edx, %edx
	movl	$-1, %esi
	call	SamsungSetRRPara
	jmp	.L1181
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.p2align 4,,15
	.globl	ToshibaReadRetrial
	.type	ToshibaReadRetrial, @function
ToshibaReadRetrial:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$24, %esp
	movl	%ecx, -16(%ebp)
	movl	%edx, -28(%ebp)
	movl	%eax, -20(%ebp)
	call	NandcWaitFlashReady
	movl	-20(%ebp), %ebx
	movzbl	g_retryMode, %eax
	movl	$0, -24(%ebp)
	addl	$8, %ebx
	leal	-67(%eax), %ecx
	sall	$8, %ebx
	addl	gpNandc, %ebx
	cmpb	$1, %cl
	jbe	.L1186
	cmpb	$0, gFlashToggleModeEn
	jne	.L1207
.L1187:
	movl	$92, 8(%ebx)
	movl	$197, 8(%ebx)
.L1186:
	xorl	%esi, %esi
	movl	$-1, %edi
	cmpb	$0, g_maxRetryCount
	je	.L1188
	movl	$1, %esi
	movl	%esi, %edx
	movl	%ebx, %esi
	movl	%edx, %ebx
	jmp	.L1197
	.p2align 4,,7
	.p2align 3
.L1211:
	movl	$4, %eax
	call	NandcSetDdrMode
	movl	8(%ebp), %eax
	movl	-16(%ebp), %ecx
	movl	-28(%ebp), %edx
	movl	%eax, (%esp)
	movl	-20(%ebp), %eax
	call	FlashReadRawPage
	movl	%eax, %edx
	xorl	%eax, %eax
	movl	%edx, -32(%ebp)
	call	NandcSetDdrMode
	movl	-32(%ebp), %edx
.L1193:
	cmpl	$-1, %edx
	je	.L1194
	movzbl	gNandFlashEccBits, %eax
	cmpl	$-1, %edi
	cmove	%edx, %edi
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%eax, %edx
	jb	.L1208
	movl	$0, 8(%ebp)
	movl	$0, -16(%ebp)
.L1194:
	movzbl	g_maxRetryCount, %eax
	addl	$1, %ebx
	addl	$1, %eax
	cmpl	%ebx, %eax
	jbe	.L1196
	movzbl	g_retryMode, %eax
.L1197:
	subl	$67, %eax
	movzbl	%bl, %edx
	cmpb	$1, %al
	movl	%esi, %eax
	jbe	.L1209
	call	ToshibaSetRRPara
.L1190:
	cmpb	$34, g_retryMode
	je	.L1210
.L1191:
	movl	-24(%ebp), %eax
	movl	$38, 8(%esi)
	movl	$93, 8(%esi)
	testl	%eax, %eax
	jne	.L1211
	movl	8(%ebp), %edx
	movl	-16(%ebp), %ecx
	movl	-20(%ebp), %eax
	movl	%edx, (%esp)
	movl	-28(%ebp), %edx
	call	FlashReadRawPage
	movl	%eax, %edx
	jmp	.L1193
	.p2align 4,,7
	.p2align 3
.L1210:
	movzbl	g_maxRetryCount, %eax
	subl	$3, %eax
	cmpl	%ebx, %eax
	jne	.L1191
	movl	$179, 8(%esi)
	jmp	.L1191
	.p2align 4,,7
	.p2align 3
.L1209:
	call	SandiskSetRRPara
	jmp	.L1190
	.p2align 4,,7
	.p2align 3
.L1196:
	movzbl	g_retryMode, %ecx
	cmpl	$-1, %edi
	movl	%esi, %ebx
	setne	%al
	movl	%eax, %esi
	subl	$67, %ecx
.L1188:
	xorl	%edx, %edx
	cmpb	$1, %cl
	movl	%ebx, %eax
	jbe	.L1212
.L1198:
	call	ToshibaSetRRPara
.L1199:
	movzbl	gNandFlashEccBits, %eax
	movl	$255, 8(%ebx)
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%edi, %eax
	ja	.L1200
	movl	%esi, %edx
	movl	$256, %eax
	testb	%dl, %dl
	cmovne	%eax, %edi
.L1200:
	movl	-20(%ebp), %eax
	call	NandcWaitFlashReady
	movl	-24(%ebp), %ecx
	testl	%ecx, %ecx
	je	.L1201
	movl	$4, %eax
	call	NandcSetDdrMode
.L1201:
	addl	$24, %esp
	movl	%edi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1208:
	movzbl	g_retryMode, %ecx
	movl	%esi, %ebx
	movl	%edx, %edi
	xorl	%edx, %edx
	movl	$1, %esi
	movl	%ebx, %eax
	subl	$67, %ecx
	cmpb	$1, %cl
	ja	.L1198
.L1212:
	call	SandiskSetRRPara
	jmp	.L1199
	.p2align 4,,7
	.p2align 3
.L1207:
	xorl	%eax, %eax
	.p2align 4,,6
	call	NandcSetDdrMode
	movzbl	g_retryMode, %eax
	movl	$1, -24(%ebp)
	leal	-67(%eax), %ecx
	jmp	.L1187
	.size	ToshibaReadRetrial, .-ToshibaReadRetrial
	.p2align 4,,15
	.globl	FlashSavePhyInfo
	.type	FlashSavePhyInfo, @function
FlashSavePhyInfo:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	movl	gFlashPageBuffer0, %eax
	movl	%eax, gpFlashSaveInfo
	movl	$60, %eax
	call	FlashBchSel
	movl	gFlashPageBuffer0, %edi
	movl	$2048, %edx
	testl	$1, %edi
	jne	.L1236
	testl	$2, %edi
	jne	.L1237
.L1215:
	movl	%edx, %ecx
	xorl	%eax, %eax
	shrl	$2, %ecx
	testb	$2, %dl
	rep stosl
	je	.L1216
	movw	$0, (%edi)
	addl	$2, %edi
.L1216:
	andl	$1, %edx
	je	.L1217
	movb	$0, (%edi)
.L1217:
	movl	gpFlashSaveInfo, %eax
	movl	$gReadRetryInfo, %esi
	movl	$1312902724, (%eax)
	movl	gpFlashSaveInfo, %eax
	movzbl	gNandMaxDie, %edx
	leal	224(%eax), %edi
	movw	%dx, 12(%eax)
	movzbl	gNandIDBResBlkNum, %edx
	testl	$1, %edi
	movw	%dx, 14(%eax)
	movzbl	gFlashToggleModeEn, %edx
	movl	DieAddrs, %ecx
	movl	%edx, 1076(%eax)
	movl	IDByte, %edx
	movl	%ecx, 96(%eax)
	movl	%edx, 16(%eax)
	movl	IDByte+4, %edx
	movl	DieAddrs+4, %ecx
	movl	%edx, 20(%eax)
	movl	IDByte+8, %edx
	movl	%ecx, 100(%eax)
	movl	DieAddrs+8, %ecx
	movl	%edx, 24(%eax)
	movl	IDByte+12, %edx
	movl	%ecx, 104(%eax)
	movl	DieAddrs+12, %ecx
	movl	%edx, 28(%eax)
	movl	DieCsIndex, %edx
	movl	%ecx, 108(%eax)
	movl	gNandParaInfo, %ecx
	movl	%edx, 80(%eax)
	movl	%ecx, 160(%eax)
	movl	gNandParaInfo+4, %ecx
	movl	%ecx, 164(%eax)
	movl	gNandParaInfo+8, %ecx
	movl	%ecx, 168(%eax)
	movl	gNandParaInfo+12, %ecx
	movl	%ecx, 172(%eax)
	movl	gNandOptPara, %edx
	movl	gNandParaInfo+16, %ecx
	movl	%edx, 192(%eax)
	movl	gNandOptPara+4, %edx
	movl	%ecx, 176(%eax)
	movl	gNandParaInfo+20, %ecx
	movl	%edx, 196(%eax)
	movl	gNandOptPara+8, %edx
	movl	%ecx, 180(%eax)
	movl	gNandParaInfo+24, %ecx
	movl	%edx, 200(%eax)
	movl	gNandOptPara+12, %edx
	movl	%ecx, 184(%eax)
	movl	gNandParaInfo+28, %ecx
	movl	%edx, 204(%eax)
	movl	gNandOptPara+16, %edx
	movl	%ecx, 188(%eax)
	movl	%edx, 208(%eax)
	movl	gNandOptPara+20, %edx
	movl	%edx, 212(%eax)
	movl	gNandOptPara+24, %edx
	movl	%edx, 216(%eax)
	movl	gNandOptPara+28, %edx
	movl	%edx, 220(%eax)
	movl	$852, %edx
	jne	.L1238
	testl	$2, %edi
	jne	.L1239
.L1219:
	movl	%edx, %ecx
	shrl	$2, %ecx
	testb	$2, %dl
	rep movsl
	je	.L1220
	movzwl	(%esi), %ecx
	movw	%cx, (%edi)
	movl	$2, %ecx
.L1220:
	andl	$1, %edx
	je	.L1221
	movzbl	(%esi,%ecx), %edx
	movb	%dl, (%edi,%ecx)
.L1221:
	xorl	%ecx, %ecx
	movl	$1204201446, %edx
	.p2align 4,,7
	.p2align 3
.L1222:
	movl	%edx, %esi
	movl	%edx, %ebx
	shrl	$2, %esi
	sall	$5, %ebx
	addl	%esi, %ebx
	movzbl	12(%eax,%ecx), %esi
	addl	$1, %ecx
	addl	%esi, %ebx
	xorl	%ebx, %edx
	cmpl	$2036, %ecx
	jne	.L1222
	movl	%edx, 8(%eax)
	movl	gBlockPageAlignSize, %edx
	xorl	%ebx, %ebx
	movl	$1592, 4(%eax)
	movl	gFlashPageBuffer1, %eax
	movl	$0, -16(%ebp)
	movl	%eax, gpFlashSaveInfo
.L1227:
	imull	%ebx, %edx
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	call	FlashEraseBlock
	movl	gBlockPageAlignSize, %edx
	xorl	%eax, %eax
	movl	gFlashPageBuffer0, %ecx
	movl	$0, (%esp)
	imull	%ebx, %edx
	call	FlashProgPage
	movl	gBlockPageAlignSize, %edx
	xorl	%eax, %eax
	movl	gFlashPageBuffer0, %ecx
	movl	$0, (%esp)
	imull	%ebx, %edx
	addl	$1, %edx
	call	FlashProgPage
	movl	gBlockPageAlignSize, %edx
	xorl	%eax, %eax
	movl	gFlashPageBuffer1, %ecx
	movl	$0, (%esp)
	imull	%ebx, %edx
	call	FlashReadRawPage
	leal	1(%ebx), %edx
	cmpl	$-1, %eax
	je	.L1223
	movl	gpFlashSaveInfo, %edi
	leal	1(%ebx), %edx
	cmpl	$1312902724, (%edi)
	je	.L1240
.L1223:
	cmpl	$4, %edx
	movl	%edx, %ebx
	je	.L1226
	movl	gBlockPageAlignSize, %edx
	jmp	.L1227
.L1226:
	cmpl	$1, -16(%ebp)
	sbbl	%eax, %eax
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1240:
	movl	8(%edi), %eax
	xorl	%edx, %edx
	movl	%eax, -20(%ebp)
	movl	$1204201446, %eax
	.p2align 4,,7
	.p2align 3
.L1224:
	movl	%eax, %esi
	movl	%eax, %ecx
	sall	$5, %ecx
	shrl	$2, %esi
	addl	%ecx, %esi
	movzbl	12(%edi,%edx), %ecx
	addl	$1, %edx
	addl	%esi, %ecx
	xorl	%ecx, %eax
	cmpl	$2036, %edx
	jne	.L1224
	cmpl	%eax, -20(%ebp)
	leal	1(%ebx), %edx
	jne	.L1223
	imull	gBlockPageAlignSize, %ebx
	addl	$1, -16(%ebp)
	cmpl	$1, -16(%ebp)
	movl	%edx, gNandFlashIdbBlockAddr
	movl	%ebx, gNandFlashInfoBlockAddr
	jbe	.L1223
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1236:
	movb	$0, (%edi)
	addl	$1, %edi
	movw	$2047, %dx
	testl	$2, %edi
	je	.L1215
.L1237:
	movw	$0, (%edi)
	subl	$2, %edx
	addl	$2, %edi
	jmp	.L1215
.L1238:
	movzbl	gReadRetryInfo, %edx
	leal	225(%eax), %edi
	movl	$gReadRetryInfo+1, %esi
	testl	$2, %edi
	movb	%dl, 224(%eax)
	movl	$851, %edx
	je	.L1219
.L1239:
	movzwl	(%esi), %ecx
	subl	$2, %edx
	addl	$2, %esi
	movw	%cx, (%edi)
	addl	$2, %edi
	jmp	.L1219
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.p2align 4,,15
	.globl	FlashLoadPhyInfo
	.type	FlashLoadPhyInfo, @function
FlashLoadPhyInfo:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	$-1, %edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movzwl	gNandParaInfo+10, %eax
	movl	$0, gNandFlashInfoBlockAddr
	movl	%eax, -24(%ebp)
	movl	gFlashPageBuffer0, %eax
	movl	%eax, gpFlashSaveInfo
	movl	$60, %eax
	call	FlashBchSel
	movl	$0, -16(%ebp)
	movl	$4, -20(%ebp)
.L1248:
	movl	gFlashPageBuffer0, %ecx
	xorl	%eax, %eax
	movl	-16(%ebp), %edx
	movl	$0, (%esp)
	call	FlashReadRawPage
	cmpl	$-1, %eax
	je	.L1251
.L1242:
	movl	gpFlashSaveInfo, %ebx
	cmpl	$1312902724, (%ebx)
	je	.L1252
.L1243:
	movl	-24(%ebp), %eax
	addl	%eax, -16(%ebp)
	subl	$1, -20(%ebp)
	jne	.L1248
.L1245:
	addl	$20, %esp
	movl	%edi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1252:
	testl	%edi, %edi
	je	.L1253
	movl	8(%ebx), %eax
	xorl	%edx, %edx
	movl	%eax, -28(%ebp)
	movl	$1204201446, %eax
	.p2align 4,,7
	.p2align 3
.L1246:
	movl	%eax, %esi
	movl	%eax, %ecx
	sall	$5, %ecx
	shrl	$2, %esi
	addl	%ecx, %esi
	movzbl	12(%ebx,%edx), %ecx
	addl	$1, %edx
	addl	%esi, %ecx
	xorl	%ecx, %eax
	cmpl	$2036, %edx
	jne	.L1246
	cmpl	%eax, -28(%ebp)
	jne	.L1243
	movl	188(%ebx), %eax
	leal	224(%ebx), %esi
	movl	$gReadRetryInfo, %edi
	movl	160(%ebx), %edx
	movl	$213, %ecx
	movl	%eax, gNandParaInfo+28
	movl	192(%ebx), %eax
	movl	%edx, gNandParaInfo
	movl	164(%ebx), %edx
	movl	%eax, gNandOptPara
	movl	196(%ebx), %eax
	movl	%edx, gNandParaInfo+4
	movl	168(%ebx), %edx
	movl	%eax, gNandOptPara+4
	movl	200(%ebx), %eax
	movl	%edx, gNandParaInfo+8
	movl	172(%ebx), %edx
	movl	%eax, gNandOptPara+8
	movl	204(%ebx), %eax
	movl	%edx, gNandParaInfo+12
	movl	176(%ebx), %edx
	movl	%eax, gNandOptPara+12
	movl	208(%ebx), %eax
	movl	%edx, gNandParaInfo+16
	movl	180(%ebx), %edx
	movl	%eax, gNandOptPara+16
	movl	212(%ebx), %eax
	movl	%edx, gNandParaInfo+20
	movl	184(%ebx), %edx
	movl	%eax, gNandOptPara+20
	movl	216(%ebx), %eax
	movl	%edx, gNandParaInfo+24
	xorl	%edx, %edx
	movl	%eax, gNandOptPara+24
	movl	220(%ebx), %eax
	movl	%eax, gNandOptPara+28
	rep movsl
	movl	1076(%ebx), %eax
	movb	%al, gFlashToggleModeEn
	movl	-16(%ebp), %eax
	movzwl	gNandParaInfo+10, %ecx
	movl	%eax, gNandFlashInfoBlockAddr
	divl	%ecx
	movl	$2, %edx
	addl	$1, %eax
	cmpl	$1, %eax
	cmovne	%eax, %edx
	movzwl	14(%ebx), %eax
	xorl	%edi, %edi
	movl	%edx, gNandFlashIdbBlockAddr
	movb	%al, gNandIDBResBlkNumSaveInFlash
	jmp	.L1243
.L1251:
	movl	-16(%ebp), %edx
	xorl	%eax, %eax
	movl	gFlashPageBuffer0, %ecx
	movl	$0, (%esp)
	addl	$1, %edx
	call	FlashReadRawPage
	cmpl	$-1, %eax
	jne	.L1242
	jmp	.L1243
.L1253:
	movzwl	gNandParaInfo+10, %ecx
	xorl	%edx, %edx
	movl	-16(%ebp), %eax
	divl	%ecx
	addl	$1, %eax
	movl	%eax, gNandFlashIdbBlockAddr
	jmp	.L1245
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.section	.rodata.str1.1
.LC89:
	.string	"\0013sync para %d\n"
.LC90:
	.string	"\0013TOG mode Read error %x %x\n"
	.text
	.p2align 4,,15
	.globl	FlashDdrTunningRead
	.type	FlashDdrTunningRead, @function
FlashDdrTunningRead:
	pushl	%ebp
	movzbl	%al, %eax
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	$16, %esi
	pushl	%ebx
	subl	$40, %esp
	movl	8(%ebp), %edi
	movl	%edx, -28(%ebp)
	movl	%ecx, -20(%ebp)
	movl	%eax, -24(%ebp)
	movl	$0, -40(%ebp)
	movl	$0, -32(%ebp)
	movl	$1024, -16(%ebp)
	movl	$-1, -36(%ebp)
	jmp	.L1257
	.p2align 4,,7
	.p2align 3
.L1266:
	addl	$1, -32(%ebp)
	cmpl	$3, -32(%ebp)
	movl	%esi, -40(%ebp)
	movl	%eax, -16(%ebp)
	movl	$0, -36(%ebp)
	ja	.L1256
.L1264:
	xorl	%edi, %edi
	movl	$0, -20(%ebp)
.L1255:
	addl	$2, %esi
	cmpl	$60, %esi
	je	.L1265
.L1257:
	movl	%esi, %ebx
	movl	%esi, %eax
	movl	-20(%ebp), %ecx
	sall	$8, %eax
	movl	-28(%ebp), %edx
	sall	$16, %ebx
	orl	%eax, %ebx
	movl	gpNandc, %eax
	orl	$1, %ebx
	movl	%ebx, 304(%eax)
	movl	-24(%ebp), %eax
	movl	%edi, (%esp)
	call	FlashReadRawPage
	cmpl	-16(%ebp), %eax
	ja	.L1255
	cmpl	$2, %eax
	jbe	.L1266
	movl	%esi, -40(%ebp)
	movl	%eax, -16(%ebp)
	movl	$0, -36(%ebp)
	jmp	.L1264
	.p2align 4,,7
	.p2align 3
.L1265:
	movl	-40(%ebp), %eax
	testl	%eax, %eax
	jne	.L1267
.L1258:
	cmpl	$-1, -36(%ebp)
	je	.L1268
	movl	-16(%ebp), %eax
	movl	%eax, -36(%ebp)
.L1259:
	movl	-36(%ebp), %eax
	addl	$40, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1267:
	movzbl	-40(%ebp), %eax
	movl	%eax, %ebx
	sall	$16, %ebx
	sall	$8, %eax
	orl	%eax, %ebx
	orl	$1, %ebx
	.p2align 4,,7
	.p2align 3
.L1256:
	movl	-40(%ebp), %eax
	movl	$.LC89, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	movl	gpNandc, %eax
	movl	%ebx, 304(%eax)
	jmp	.L1258
.L1268:
	movl	-28(%ebp), %eax
	movl	$.LC90, (%esp)
	movl	%eax, 8(%esp)
	movl	-24(%ebp), %eax
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L1259
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.section	.rodata.str1.1
.LC91:
	.string	"\0013read retry status %x %x %x\n"
	.text
	.p2align 4,,15
	.globl	FlashReadPage
	.type	FlashReadPage, @function
FlashReadPage:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$36, %esp
	movl	%esi, -8(%ebp)
	movzbl	%al, %esi
	movl	8(%ebp), %eax
	movl	%ebx, -12(%ebp)
	movl	%edx, %ebx
	movl	%edi, -4(%ebp)
	movl	%ecx, %edi
	movl	%eax, (%esp)
	movl	%esi, %eax
	call	FlashReadRawPage
	cmpl	$-1, %eax
	movl	%eax, %edx
	je	.L1275
.L1270:
	movl	-12(%ebp), %ebx
	movl	%edx, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1275:
	movzbl	gNandRandomizer, %ecx
	movl	$1, %eax
	testb	%cl, %cl
	movb	%cl, -16(%ebp)
	jne	.L1276
.L1271:
	movl	gpReadRetrial, %ecx
	testb	%al, %al
	movl	%ecx, -16(%ebp)
	jne	.L1277
.L1272:
	cmpl	$-1, %edx
	jne	.L1270
	cmpb	$0, gFlashToggleModeEn
	je	.L1270
	movl	gpNandc, %eax
	movl	%edi, %ecx
	movl	8(%ebp), %edx
	movl	304(%eax), %eax
	movl	%edx, (%esp)
	movl	%ebx, %edx
	movl	%eax, -16(%ebp)
	movl	%esi, %eax
	call	FlashDdrTunningRead
	cmpl	$-1, %eax
	movl	%eax, %edx
	je	.L1273
	movzbl	gNandFlashEccBits, %eax
	shrl	%eax
	cmpl	%eax, %edx
	jbe	.L1270
.L1273:
	movl	-16(%ebp), %eax
	movzbl	%ah, %ecx
	movl	%ecx, %eax
	sall	$16, %eax
	sall	$8, %ecx
	orl	%ecx, %eax
	movl	gpNandc, %ecx
	orl	$1, %eax
	movl	%eax, 304(%ecx)
	movl	-12(%ebp), %ebx
	movl	%edx, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1276:
	movl	8(%ebp), %eax
	movl	%edi, %ecx
	movl	%ebx, %edx
	movb	$0, gNandRandomizer
	movl	%eax, (%esp)
	movl	%esi, %eax
	call	FlashReadRawPage
	movzbl	-16(%ebp), %ecx
	movb	%cl, gNandRandomizer
	cmpl	$-1, %eax
	movl	%eax, %edx
	sete	%al
	jmp	.L1271
.L1277:
	testl	%ecx, %ecx
	je	.L1272
	movl	8(%ebp), %eax
	movl	%ebx, %edx
	movl	%edi, %ecx
	movl	%eax, (%esp)
	movl	%esi, %eax
	call	*-16(%ebp)
	movl	%ebx, 12(%esp)
	movl	%esi, 8(%esp)
	movl	$.LC91, (%esp)
	movl	%eax, 4(%esp)
	movl	%eax, -20(%ebp)
	call	printk
	movl	-20(%ebp), %edx
	jmp	.L1272
	.size	FlashReadPage, .-FlashReadPage
	.p2align 4,,15
	.globl	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, @function
FlashLoadFactorBbt:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$28, %esp
	movl	gpNandParaInfo, %eax
	cmpb	$0, gNandMaxDie
	movl	$0, FbbtBlk
	movl	$0, FbbtBlk+4
	movzwl	14(%eax), %edx
	movl	$-1, -24(%ebp)
	movzbl	13(%eax), %eax
	je	.L1279
	movzbl	%al, %eax
	imull	%edx, %eax
	movw	$0, -20(%ebp)
	movb	$0, -17(%ebp)
	leal	-1(%eax), %edx
	movzwl	%ax, %eax
	movw	%dx, -30(%ebp)
	subl	$16, %eax
	movzwl	%dx, %edx
	movl	%edx, -28(%ebp)
	movl	%eax, -16(%ebp)
	movl	%edx, -36(%ebp)
	.p2align 4,,7
	.p2align 3
.L1284:
	movzbl	-17(%ebp), %esi
	movl	-16(%ebp), %edx
	cmpl	%edx, -28(%ebp)
	movzbl	DieCsIndex(%esi), %eax
	jle	.L1280
	movzwl	-30(%ebp), %ebx
	movzbl	%al, %edi
	movl	-36(%ebp), %edx
	.p2align 4,,7
	.p2align 3
.L1282:
	movl	gFlashSpareBuffer, %eax
	xorl	%ecx, %ecx
	imull	gBlockPageAlignSize, %edx
	addl	DieAddrs(,%esi,4), %edx
	movl	%eax, (%esp)
	movl	%edi, %eax
	call	FlashReadPage
	cmpl	$-1, %eax
	je	.L1281
	movl	gFlashSpareBuffer, %eax
	cmpw	$-3872, (%eax)
	je	.L1289
.L1281:
	subl	$1, %ebx
	movzwl	%bx, %edx
	cmpl	-16(%ebp), %edx
	jg	.L1282
.L1280:
	movzbl	gNandMaxDie, %eax
	movzbl	%al, %edx
	cmpw	-20(%ebp), %dx
	movl	$0, %edx
	cmovne	-24(%ebp), %edx
	addb	$1, -17(%ebp)
	cmpb	-17(%ebp), %al
	movl	%edx, -24(%ebp)
	ja	.L1284
.L1279:
	movl	-24(%ebp), %eax
	addl	$28, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1289:
	movw	%bx, FbbtBlk(%esi,%esi)
	addw	$1, -20(%ebp)
	jmp	.L1280
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.p2align 4,,15
	.globl	FlashDdrParaScan
	.type	FlashDdrParaScan, @function
FlashDdrParaScan:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$16, %esp
	movl	%ebx, -12(%ebp)
	movl	%eax, %ebx
	movzbl	gFlashInterfaceMode, %eax
	movl	%esi, -8(%ebp)
	movl	%edx, %esi
	movzbl	%bl, %ebx
	movl	%edi, -4(%ebp)
	call	FlashSetInterfaceMode
	movzbl	gFlashInterfaceMode, %eax
	call	NandcSetMode
	xorl	%ecx, %ecx
	movl	%esi, %edx
	movl	%ebx, %eax
	movl	$0, (%esp)
	call	FlashDdrTunningRead
	xorl	%ecx, %ecx
	movl	%esi, %edx
	movl	$0, (%esp)
	movl	%eax, %edi
	movl	%ebx, %eax
	call	FlashReadRawPage
	cmpl	$-1, %eax
	je	.L1291
	cmpl	$-1, %edi
	je	.L1291
.L1292:
	movb	$1, gFlashToggleModeEn
.L1293:
	movl	-12(%ebp), %ebx
	xorl	%eax, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1291:
	testb	$1, gFlashInterfaceMode
	je	.L1292
	movl	$1, %eax
	call	FlashSetInterfaceMode
	movl	$1, %eax
	call	NandcSetMode
	movb	$0, gFlashToggleModeEn
	jmp	.L1293
	.size	FlashDdrParaScan, .-FlashDdrParaScan
	.p2align 4,,15
	.globl	WaitFlashReady
	.type	WaitFlashReady, @function
WaitFlashReady:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$20, %esp
	movl	FlashWaitBusyScheduleEn, %ebx
	testl	%ebx, %ebx
	je	.L1295
	movl	nandc0_ready_completed_flag, %ecx
	testl	%ecx, %ecx
	jne	.L1295
	leal	-12(%ebp), %eax
	movl	$1, %ebx
	movl	%eax, -12(%ebp)
	movl	%eax, -8(%ebp)
	movl	$0, -24(%ebp)
	.p2align 4,,7
	.p2align 3
.L1299:
	leal	-24(%ebp), %edx
	movl	$1, %ecx
	movl	$rk29_nandc_wait, %eax
	call	prepare_to_wait_event
	movl	nandc0_ready_completed_flag, %edx
	testl	%edx, %edx
	je	.L1304
.L1297:
	leal	-24(%ebp), %edx
	movl	$rk29_nandc_wait, %eax
	call	finish_wait
.L1295:
	movl	$0, nandc0_ready_completed_flag
	addl	$20, %esp
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1304:
	testl	%ebx, %ebx
	je	.L1297
	testl	%eax, %eax
	jne	.L1297
	movl	%ebx, %eax
	.p2align 4,,5
	call	schedule_timeout
	movl	%eax, %ebx
	.p2align 4,,2
	jmp	.L1299
	.size	WaitFlashReady, .-WaitFlashReady
	.p2align 4,,15
	.globl	NandcIqrWaitFlashReady
	.type	NandcIqrWaitFlashReady, @function
NandcIqrWaitFlashReady:
	movl	368(%eax), %edx
	movl	$0, nandc0_ready_completed_flag
	pushl	%ebp
	movl	%esp, %ebp
	orl	$2, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	orl	$2, %edx
	movl	%edx, 364(%eax)
	movl	(%eax), %edx
	andb	$2, %dh
	je	.L1308
	movl	368(%eax), %edx
	popl	%ebp
	orl	$2, %edx
	movl	%edx, 368(%eax)
	movl	364(%eax), %edx
	andl	$-3, %edx
	movl	%edx, 364(%eax)
	ret
	.p2align 4,,7
	.p2align 3
.L1308:
	call	WaitFlashReady
	popl	%ebp
	ret
	.size	NandcIqrWaitFlashReady, .-NandcIqrWaitFlashReady
	.p2align 4,,15
	.globl	FlashEraseBlocks
	.type	FlashEraseBlocks, @function
FlashEraseBlocks:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$36, %esp
	testl	%edx, %edx
	movl	%eax, -28(%ebp)
	movl	%edx, -32(%ebp)
	je	.L1310
	movb	%dl, -33(%ebp)
	xorl	%ebx, %ebx
	jmp	.L1316
	.p2align 4,,7
	.p2align 3
.L1322:
	addl	$1, %ebx
	cmpl	%ebx, -32(%ebp)
	movl	$-1, (%edi)
	jbe	.L1310
.L1316:
	movl	-28(%ebp), %edx
	leal	(%ebx,%ebx,8), %eax
	leal	-16(%ebp), %ecx
	movl	%ecx, (%esp)
	leal	-20(%ebp), %ecx
	leal	(%edx,%eax,4), %edi
	movzbl	-33(%ebp), %eax
	xorl	%edx, %edx
	subb	%bl, %al
	movzbl	%al, %eax
	movl	%eax, 4(%esp)
	movl	%edi, %eax
	call	LogAddr2PhyAddr
	movzbl	gNandMaxDie, %edx
	movl	%eax, %esi
	movl	-16(%ebp), %eax
	cmpl	%eax, %edx
	jbe	.L1322
	cmpb	$0, gMultiPageProgEn
	movl	$0, %edx
	cmove	%edx, %esi
	movl	%eax, %edx
	sall	$4, %edx
	movl	gDieOp+8(%edx), %edx
	testl	%edx, %edx
	je	.L1314
	movzbl	%al, %eax
	call	FlashWaitCmdDone
	movl	-16(%ebp), %eax
.L1314:
	movl	-20(%ebp), %edx
	movl	%eax, %ecx
	sall	$4, %ecx
	testl	%esi, %esi
	movl	%edi, gDieOp+8(%ecx)
	movl	$0, gDieOp+12(%ecx)
	movl	%edx, -40(%ebp)
	movl	%edx, gDieOp+4(%ecx)
	je	.L1315
	movl	-28(%ebp), %edx
	leal	9(%ebx,%ebx,8), %edi
	leal	(%edx,%edi,4), %edx
	movl	%edx, gDieOp+12(%ecx)
.L1315:
	movzbl	DieCsIndex(%eax), %ecx
	movl	%eax, %edi
	movl	DieAddrs(,%eax,4), %eax
	sall	$4, %edi
	movl	-40(%ebp), %edx
	movb	%cl, gDieOp(%edi)
	movzbl	%cl, %edi
	movb	%cl, -34(%ebp)
	xorl	%ecx, %ecx
	testl	%eax, %eax
	setne	%cl
	movl	%edi, %eax
	movl	%edi, -24(%ebp)
	movl	$1, %edi
	addl	%esi, %ebx
	call	FlashWaitReadyEN
	movl	gpNandc, %edx
	addl	$1, %ebx
	movzbl	-34(%ebp), %ecx
	movl	(%edx), %eax
	sall	%cl, %edi
	movl	%esi, %ecx
	movl	%edi, -40(%ebp)
	movb	-40(%ebp), %al
	movl	%eax, (%edx)
	movl	-24(%ebp), %eax
	movl	-20(%ebp), %edx
	call	FlashEraseCmd
	movl	gpNandc, %edx
	movl	(%edx), %eax
	andl	$-131328, %eax
	cmpl	%ebx, -32(%ebp)
	movl	%eax, (%edx)
	ja	.L1316
.L1310:
	movl	gpNandc, %eax
	call	NandcIqrWaitFlashReady
	cmpb	$0, gNandMaxDie
	je	.L1317
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L1318:
	movzbl	%bl, %eax
	addl	$1, %ebx
	call	FlashWaitCmdDone
	movzbl	gNandMaxDie, %eax
	cmpl	%ebx, %eax
	ja	.L1318
.L1317:
	addl	$36, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.p2align 4,,15
	.globl	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, @function
FtlFreeSysBlkQueueIn:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$8, %esp
	movl	%ebx, -8(%ebp)
	movl	%eax, %ebx
	movzwl	gSysFreeQueue+6, %eax
	movl	%esi, -4(%ebp)
	cmpw	$1024, %ax
	je	.L1323
	testw	%dx, %dx
	jne	.L1326
.L1325:
	addl	$1, %eax
	movw	%ax, gSysFreeQueue+6
	movzwl	gSysFreeQueue+4, %eax
	movw	%bx, gSysFreeQueue+8(%eax,%eax)
	addl	$1, %eax
	andl	$1023, %eax
	movw	%ax, gSysFreeQueue+4
.L1323:
	movl	-8(%ebp), %ebx
	movl	-4(%ebp), %esi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1326:
	movl	%ebx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%bx, %edx
	sall	$10, %edx
	movl	%eax, %esi
	movl	req_erase, %eax
	movzwl	%si, %esi
	movl	%edx, 4(%eax)
	movl	$1, %edx
	call	FlashEraseBlocks
	movl	p_erase_count_table, %ecx
	leal	(%ecx,%esi,2), %esi
	addw	$1, (%esi)
	addl	$1, g_totle_sys_slc_erase_count
	movzwl	gSysFreeQueue+6, %eax
	jmp	.L1325
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.p2align 4,,15
	.globl	FtlScanSysBlk
	.type	FtlScanSysBlk, @function
FtlScanSysBlk:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$44, %esp
	movl	c_ftl_nand_max_map_blks, %ecx
	movl	p_map_block_ver_table, %eax
	movw	$0, g_totle_map_block
	movw	$0, g_totle_vendor_block
	sall	$2, %ecx
	call	memset
	movl	c_ftl_nand_max_map_blks, %ecx
	xorl	%edx, %edx
	movl	p_map_block_table, %eax
	addl	%ecx, %ecx
	call	memset
	movzwl	c_ftl_nand_max_vendor_blks, %ecx
	xorl	%edx, %edx
	movl	p_vendor_block_ver_table, %eax
	sall	$2, %ecx
	call	memset
	movzwl	c_ftl_nand_max_vendor_blks, %ecx
	xorl	%edx, %edx
	movl	p_vendor_block_table, %eax
	addl	%ecx, %ecx
	call	memset
	movzwl	c_ftl_nand_data_blks_per_plane, %edi
	cmpw	c_ftl_nand_blk_pre_plane, %di
	movl	$-1, gSysInfo
	movl	$-1, gSysInfo+4
	movl	$-1, gSysInfo+8
	movw	%di, -54(%ebp)
	jae	.L1329
	.p2align 4,,7
	.p2align 3
.L1388:
	movzwl	c_ftl_nand_planes_num, %eax
	testw	%ax, %ax
	je	.L1374
	movzwl	c_ftl_nand_planes_per_die, %edi
	subl	$1, %eax
	movl	$p_plane_order_table, %ebx
	movzwl	-54(%ebp), %edx
	movzwl	%ax, %eax
	movl	req_read, %ecx
	addl	$p_plane_order_table+1, %eax
	movl	%eax, -24(%ebp)
	movw	%di, -16(%ebp)
	movzwl	c_ftl_nand_blks_per_die, %edi
	imulw	-16(%ebp), %dx
	movl	%ecx, -44(%ebp)
	movzwl	c_ftl_nand_byte_pre_oob, %ecx
	movw	$0, -20(%ebp)
	movw	%dx, -40(%ebp)
	movl	p_gc_spare_buf, %edx
	movl	%ecx, -52(%ebp)
	movl	%edx, -48(%ebp)
	.p2align 4,,7
	.p2align 3
.L1331:
	movzbl	(%ebx), %eax
	xorl	%edx, %edx
	divw	-16(%ebp)
	addw	-40(%ebp), %dx
	movl	%eax, %esi
	imull	%edi, %esi
	addl	%edx, %esi
	xorl	%edx, %edx
	movl	%esi, %eax
	divw	%di
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %eax
	movl	%edx, %ecx
	shrl	%cl, %eax
	testb	$1, %al
	jne	.L1330
	movzwl	-20(%ebp), %edx
	movzwl	%si, %esi
	movl	-44(%ebp), %ecx
	sall	$10, %esi
	leal	(%edx,%edx,8), %eax
	imull	-52(%ebp), %edx
	leal	(%ecx,%eax,4), %eax
	movl	%esi, 4(%eax)
	movl	$0, 8(%eax)
	testl	%edx, %edx
	leal	3(%edx), %ecx
	cmovs	%ecx, %edx
	andl	$-4, %edx
	addw	$1, -20(%ebp)
	addl	-48(%ebp), %edx
	movl	%edx, 12(%eax)
.L1330:
	addl	$1, %ebx
	cmpl	-24(%ebp), %ebx
	jne	.L1331
	movzwl	-20(%ebp), %edx
	movl	$1, %ecx
	movl	req_read, %eax
	call	FlashReadPages
	cmpw	$0, -20(%ebp)
	je	.L1332
	movzwl	-20(%ebp), %eax
	movl	$0, -16(%ebp)
	subl	$1, %eax
	movzwl	%ax, %eax
	leal	9(%eax,%eax,8), %eax
	sall	$2, %eax
	movl	%eax, -44(%ebp)
	jmp	.L1372
	.p2align 4,,7
	.p2align 3
.L1404:
	cmpw	$-3962, %ax
	je	.L1402
.L1339:
	addl	$36, -16(%ebp)
	movl	-44(%ebp), %edi
	cmpl	%edi, -16(%ebp)
	je	.L1332
.L1372:
	movl	-16(%ebp), %eax
	addl	req_read, %eax
	movl	4(%eax), %edx
	movl	%edx, %edi
	shrl	$10, %edi
	movw	%di, -40(%ebp)
	movl	12(%eax), %edi
	cmpl	$-1, (%eax)
	movl	%edi, -20(%ebp)
	je	.L1403
.L1333:
	movl	g_GlobalSysVersion, %eax
	movl	-20(%ebp), %edi
	cmpl	$-1, %eax
	movl	4(%edi), %edx
	je	.L1337
	cmpl	%edx, %eax
	ja	.L1338
.L1337:
	cmpl	$-1, %edx
	je	.L1338
	leal	1(%edx), %eax
	movl	%eax, g_GlobalSysVersion
.L1338:
	movl	-20(%ebp), %edi
	movzwl	(%edi), %eax
	cmpw	$-3932, %ax
	je	.L1341
	jbe	.L1404
	cmpw	$-3902, %ax
	je	.L1342
	cmpw	$-1, %ax
	.p2align 4,,3
	jne	.L1339
	movzwl	-40(%ebp), %eax
	xorl	%edx, %edx
	call	FtlFreeSysBlkQueueIn
	movl	-44(%ebp), %edi
	addl	$36, -16(%ebp)
	cmpl	%edi, -16(%ebp)
	jne	.L1372
	.p2align 4,,7
	.p2align 3
.L1332:
	addw	$1, -54(%ebp)
	movzwl	-54(%ebp), %edi
	cmpw	%di, c_ftl_nand_blk_pre_plane
	ja	.L1388
.L1329:
	movl	p_map_block_table, %ecx
	cmpw	$0, (%ecx)
	jne	.L1375
	cmpw	$0, g_totle_map_block
	je	.L1375
	movl	c_ftl_nand_max_map_blks, %esi
	xorl	%edx, %edx
	testl	%esi, %esi
	jne	.L1396
	jmp	.L1375
	.p2align 4,,7
	.p2align 3
.L1379:
	movzwl	(%ecx,%eax,2), %ebx
	testw	%bx, %bx
	jne	.L1405
.L1396:
	addl	$1, %edx
	movswl	%dx, %eax
	cmpl	%eax, %esi
	ja	.L1379
.L1375:
	movl	p_vendor_block_table, %ebx
	cmpw	$0, (%ebx)
	jne	.L1380
	cmpw	$0, g_totle_vendor_block
	je	.L1380
	movzwl	c_ftl_nand_max_vendor_blks, %ecx
	xorl	%edx, %edx
	testl	%ecx, %ecx
	jne	.L1387
	jmp	.L1380
	.p2align 4,,7
	.p2align 3
.L1383:
	movzwl	(%ebx,%eax,2), %esi
	testw	%si, %si
	jne	.L1406
.L1387:
	addl	$1, %edx
	movswl	%dx, %eax
	cmpl	%eax, %ecx
	jg	.L1383
.L1380:
	addl	$44, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1342:
	movl	c_ftl_nand_max_map_blks, %eax
	movzwl	g_totle_map_block, %ecx
	movl	%eax, %edi
	leal	-1(%edi), %esi
	subw	%cx, %di
	subl	$1, %edi
	cmpw	%di, %si
	movl	%eax, -48(%ebp)
	movw	%cx, -52(%ebp)
	jle	.L1345
	movl	p_map_block_ver_table, %ecx
	movswl	%si, %ebx
	leal	0(,%ebx,4), %eax
	movl	%eax, -24(%ebp)
	cmpl	(%ecx,%ebx,4), %edx
	ja	.L1346
	movzwl	-48(%ebp), %eax
	movw	%di, -56(%ebp)
	subl	$2, %eax
	jmp	.L1348
	.p2align 4,,7
	.p2align 3
.L1355:
	movswl	%ax, %ebx
	subl	$1, %eax
	cmpl	%edx, (%ecx,%ebx,4)
	leal	0(,%ebx,4), %edi
	movl	%edi, -24(%ebp)
	jb	.L1346
.L1348:
	cmpw	-56(%ebp), %ax
	movl	%eax, %esi
	jg	.L1355
.L1345:
	testw	%si, %si
	js	.L1335
	movzwl	g_totle_map_block, %edx
	movl	%edx, %eax
	notl	%eax
	addw	c_ftl_nand_max_map_blks, %ax
	cmpw	%si, %ax
	jl	.L1339
	movl	-20(%ebp), %edi
	addl	$1, %edx
	movswl	%si, %esi
	movw	%dx, g_totle_map_block
	movl	p_map_block_ver_table, %eax
	movl	4(%edi), %edx
	movzwl	-40(%ebp), %edi
	movl	%edx, (%eax,%esi,4)
	movl	p_map_block_table, %eax
	movw	%di, (%eax,%esi,2)
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1403:
	addl	$1, %edx
	movl	$1, %ecx
	movl	%edx, 4(%eax)
	movl	$1, %edx
	call	FlashReadPages
	cmpw	$-1, (%edi)
	je	.L1407
.L1334:
	movl	req_read, %eax
	movl	-16(%ebp), %edi
	cmpl	$-1, (%eax,%edi)
	jne	.L1333
.L1335:
	movzwl	-40(%ebp), %eax
	movl	$1, %edx
	call	FtlFreeSysBlkQueueIn
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1341:
	cmpw	$-1, gSysInfo
	je	.L1408
	movzwl	gSysInfo+4, %eax
	cmpw	$-1, %ax
	je	.L1370
	movzwl	%ax, %eax
	movl	$1, %edx
	call	FtlFreeSysBlkQueueIn
.L1370:
	movl	-20(%ebp), %edi
	movl	4(%edi), %edi
	cmpl	%edi, gSysInfo+8
	jae	.L1371
	movzwl	-40(%ebp), %edi
	movzwl	gSysInfo, %eax
	movw	%di, gSysInfo
	movl	-20(%ebp), %edi
	movw	%ax, gSysInfo+4
	movl	4(%edi), %eax
	movl	%eax, gSysInfo+8
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1402:
	movzwl	c_ftl_nand_max_vendor_blks, %edi
	movzwl	g_totle_vendor_block, %eax
	movzwl	%di, %ecx
	subl	$1, %ecx
	leal	-1(%edi), %esi
	movw	%ax, -24(%ebp)
	movzwl	%ax, %eax
	movswl	%si, %ebx
	subl	%eax, %ecx
	cmpl	%ecx, %ebx
	movl	%ecx, -52(%ebp)
	jle	.L1357
	movl	p_vendor_block_ver_table, %eax
	leal	0(,%ebx,4), %ecx
	movl	%ecx, -48(%ebp)
	cmpl	%edx, (%eax,%ebx,4)
	jb	.L1358
	movl	-52(%ebp), %ecx
	movw	%di, -52(%ebp)
	jmp	.L1389
	.p2align 4,,7
	.p2align 3
.L1367:
	cmpl	(%eax,%ebx,4), %edx
	leal	0(,%ebx,4), %edi
	ja	.L1409
.L1389:
	subl	$1, %esi
	movswl	%si, %ebx
	cmpl	%ebx, %ecx
	jl	.L1367
.L1357:
	testw	%si, %si
	js	.L1335
	movzwl	g_totle_vendor_block, %edx
	movl	%edx, %eax
	notl	%eax
	addw	c_ftl_nand_max_vendor_blks, %ax
	cmpw	%si, %ax
	jl	.L1339
	movl	-20(%ebp), %edi
	addl	$1, %edx
	movw	%dx, g_totle_vendor_block
	movl	p_vendor_block_ver_table, %eax
	movl	4(%edi), %edx
	movzwl	-40(%ebp), %edi
	movl	%edx, (%eax,%ebx,4)
	movl	p_vendor_block_table, %eax
	movw	%di, (%eax,%ebx,2)
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1346:
	movl	(%ecx), %edi
	testl	%edi, %edi
	jne	.L1349
	movzwl	-52(%ebp), %eax
	cmpl	%eax, -48(%ebp)
	je	.L1349
	movzwl	-52(%ebp), %eax
	addl	$1, %eax
	movw	%ax, g_totle_map_block
.L1351:
	testw	%si, %si
	je	.L1352
	xorl	%eax, %eax
	movl	%ebx, -48(%ebp)
	jmp	.L1354
	.p2align 4,,7
	.p2align 3
.L1410:
	movl	p_map_block_ver_table, %ecx
.L1354:
	movswl	%ax, %edx
	addl	$1, %eax
	leal	1(%edx), %ebx
	cmpw	%si, %ax
	movl	(%ecx,%ebx,4), %edi
	movl	%edi, (%ecx,%edx,4)
	movl	p_map_block_table, %ecx
	movzwl	(%ecx,%ebx,2), %ebx
	movw	%bx, (%ecx,%edx,2)
	jne	.L1410
	movl	-48(%ebp), %ebx
	movl	p_map_block_ver_table, %ecx
.L1352:
	movl	-20(%ebp), %edi
	movl	4(%edi), %eax
	movl	-24(%ebp), %edi
	movl	%eax, (%ecx,%edi)
	movzwl	-40(%ebp), %edi
	movl	p_map_block_table, %eax
	movw	%di, (%eax,%ebx,2)
	jmp	.L1345
	.p2align 4,,7
	.p2align 3
.L1409:
	movl	%edi, -48(%ebp)
	movzwl	-52(%ebp), %edi
.L1358:
	movl	(%eax), %edx
	testl	%edx, %edx
	jne	.L1361
	cmpw	%di, -24(%ebp)
	je	.L1361
	movzwl	-24(%ebp), %edx
	addl	$1, %edx
	movw	%dx, g_totle_vendor_block
.L1363:
	testw	%si, %si
	je	.L1364
	xorl	%edx, %edx
	movl	%ebx, -24(%ebp)
	jmp	.L1366
	.p2align 4,,7
	.p2align 3
.L1411:
	movl	p_vendor_block_ver_table, %eax
.L1366:
	movswl	%dx, %ecx
	addl	$1, %edx
	leal	1(%ecx), %ebx
	cmpw	%si, %dx
	movl	(%eax,%ebx,4), %edi
	movl	%edi, (%eax,%ecx,4)
	movl	p_vendor_block_table, %eax
	movzwl	(%eax,%ebx,2), %ebx
	movw	%bx, (%eax,%ecx,2)
	jne	.L1411
	movl	-24(%ebp), %ebx
	movl	p_vendor_block_ver_table, %eax
.L1364:
	movl	-20(%ebp), %edi
	movl	-48(%ebp), %ecx
	movl	4(%edi), %edx
	movzwl	-40(%ebp), %edi
	movl	%edx, (%eax,%ecx)
	movl	p_vendor_block_table, %eax
	movw	%di, (%eax,%ebx,2)
	jmp	.L1357
	.p2align 4,,7
	.p2align 3
.L1371:
	movzwl	-40(%ebp), %edi
	movw	%di, gSysInfo+4
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1408:
	movzwl	-40(%ebp), %edi
	movl	%edx, gSysInfo+8
	movw	%di, gSysInfo
	jmp	.L1339
	.p2align 4,,7
	.p2align 3
.L1361:
	movl	p_vendor_block_table, %eax
	movl	$1, %edx
	movzwl	(%eax), %eax
	call	FtlFreeSysBlkQueueIn
	movl	p_vendor_block_ver_table, %eax
	jmp	.L1363
	.p2align 4,,7
	.p2align 3
.L1349:
	movl	p_map_block_table, %eax
	movl	$1, %edx
	movzwl	(%eax), %eax
	call	FtlFreeSysBlkQueueIn
	movl	p_map_block_ver_table, %ecx
	jmp	.L1351
	.p2align 4,,7
	.p2align 3
.L1407:
	movl	req_read, %eax
	movl	-16(%ebp), %edi
	movl	$-1, (%eax,%edi)
	jmp	.L1334
.L1374:
	movl	req_read, %eax
	movl	$1, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	jmp	.L1332
.L1406:
	movl	%ebx, -16(%ebp)
	movl	%eax, %edi
	movl	%eax, %ecx
	movl	%eax, %ebx
	movl	-16(%ebp), %eax
	jmp	.L1382
	.p2align 4,,7
	.p2align 3
.L1412:
	movl	p_vendor_block_table, %eax
	movzwl	(%eax,%ecx,2), %esi
.L1382:
	subl	%edi, %ebx
	addl	$1, %edx
	movw	%si, (%eax,%ebx,2)
	movl	p_vendor_block_ver_table, %eax
	movl	(%eax,%ecx,4), %esi
	movl	%esi, (%eax,%ebx,4)
	movl	p_vendor_block_table, %eax
	movw	$0, (%eax,%ecx,2)
	movzwl	c_ftl_nand_max_vendor_blks, %eax
	movswl	%dx, %ecx
	movl	%ecx, %ebx
	cmpl	%eax, %ecx
	jl	.L1412
	addl	$44, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1405:
	movl	%eax, %edi
	movl	%eax, %esi
	jmp	.L1384
.L1378:
	movl	p_map_block_table, %ecx
	movzwl	(%ecx,%esi,2), %ebx
.L1384:
	subl	%edi, %esi
	addl	$1, %edx
	movw	%bx, (%ecx,%esi,2)
	movl	p_map_block_ver_table, %ecx
	movl	(%ecx,%eax,4), %ebx
	movl	%ebx, (%ecx,%esi,4)
	movl	p_map_block_table, %ecx
	movswl	%dx, %esi
	movw	$0, (%ecx,%eax,2)
	movl	%esi, %eax
	cmpl	c_ftl_nand_max_map_blks, %esi
	jb	.L1378
	jmp	.L1375
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.p2align 4,,15
	.globl	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, @function
ftl_free_no_use_map_blk:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movl	24(%eax), %esi
	movzwl	10(%eax), %ecx
	movl	%eax, -24(%ebp)
	movl	12(%eax), %ebx
	movl	%esi, -28(%ebp)
	movl	20(%eax), %esi
	addl	%ecx, %ecx
	movl	%esi, %eax
	movl	%esi, -16(%ebp)
	call	memset
	movl	-24(%ebp), %eax
	cmpw	$0, 6(%eax)
	je	.L1431
	movl	-24(%ebp), %eax
	movw	$0, -18(%ebp)
	movzwl	10(%eax), %ecx
	movl	%ecx, %edx
	.p2align 4,,7
	.p2align 3
.L1419:
	movzwl	-18(%ebp), %eax
	movl	-28(%ebp), %esi
	movl	(%esi,%eax,4), %edi
	shrl	$10, %edi
	testw	%cx, %cx
	je	.L1416
	xorl	%eax, %eax
	jmp	.L1418
	.p2align 4,,7
	.p2align 3
.L1417:
	addl	$1, %eax
	movl	%ecx, %edx
	cmpw	%ax, %cx
	jbe	.L1416
.L1418:
	movzwl	%ax, %edx
	cmpw	%di, (%ebx,%edx,2)
	leal	(%edx,%edx), %esi
	jne	.L1417
	movl	-16(%ebp), %edx
	addl	$1, %eax
	addw	$1, (%edx,%esi)
	movl	-24(%ebp), %esi
	movzwl	10(%esi), %ecx
	cmpw	%ax, %cx
	movl	%ecx, %edx
	ja	.L1418
.L1416:
	movl	-24(%ebp), %eax
	addw	$1, -18(%ebp)
	movzwl	-18(%ebp), %esi
	cmpw	%si, 6(%eax)
	ja	.L1419
.L1415:
	movl	-16(%ebp), %esi
	xorl	%eax, %eax
	testw	%dx, %dx
	movzwl	(%esi), %ecx
	je	.L1420
	movl	%ebx, -28(%ebp)
	xorl	%esi, %esi
	movl	%ecx, %edi
	movw	$0, -18(%ebp)
	movl	-24(%ebp), %ebx
	jmp	.L1424
	.p2align 4,,7
	.p2align 3
.L1421:
	movl	-16(%ebp), %edx
	leal	(%eax,%eax), %ecx
	movzwl	(%edx,%eax,2), %eax
	cmpw	%di, %ax
	jae	.L1422
	movw	%si, -18(%ebp)
	movl	%eax, %edi
.L1422:
	testw	%ax, %ax
	jne	.L1423
	addl	-28(%ebp), %ecx
	movzwl	(%ecx), %eax
	testw	%ax, %ax
	jne	.L1432
.L1423:
	addl	$1, %esi
	cmpw	%si, 10(%ebx)
	jbe	.L1433
.L1424:
	cmpw	%si, (%ebx)
	movzwl	%si, %eax
	jne	.L1421
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	cmpw	%dx, 2(%ebx)
	jae	.L1421
	movl	-16(%ebp), %ecx
	movw	%dx, (%ecx,%eax,2)
	jmp	.L1421
	.p2align 4,,7
	.p2align 3
.L1432:
	movzwl	%ax, %eax
	movl	$1, %edx
	movl	%ecx, -32(%ebp)
	addl	$1, %esi
	call	FtlFreeSysBlkQueueIn
	movl	-32(%ebp), %ecx
	movw	$0, (%ecx)
	subw	$1, 8(%ebx)
	cmpw	%si, 10(%ebx)
	ja	.L1424
.L1433:
	movzwl	-18(%ebp), %eax
.L1420:
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1431:
	movzwl	10(%eax), %edx
	jmp	.L1415
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.section	.rodata.str1.1
.LC92:
	.string	"\0013prog error: = %x\n"
.LC93:
	.string	"\0013prog read error: = %x\n"
	.section	.rodata.str1.4
	.align 4
.LC94:
	.string	"\0013prog read s error: = %x %x %x\n"
	.align 4
.LC95:
	.string	"\0013prog read d error: = %x %x %x\n"
	.text
	.p2align 4,,15
	.globl	FlashProgPages
	.type	FlashProgPages, @function
FlashProgPages:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$60, %esp
	testl	%edx, %edx
	movl	%eax, -44(%ebp)
	movzbl	gNandParaInfo+9, %eax
	movl	%edx, -32(%ebp)
	movl	%ecx, -40(%ebp)
	je	.L1435
	movzbl	%al, %eax
	xorl	%ebx, %ebx
	movb	%dl, -34(%ebp)
	movl	%eax, -48(%ebp)
	jmp	.L1445
	.p2align 4,,7
	.p2align 3
.L1458:
	addl	$1, %ebx
	cmpl	%ebx, -32(%ebp)
	movl	$-1, (%esi)
	jbe	.L1435
.L1445:
	movl	-44(%ebp), %edx
	leal	(%ebx,%ebx,8), %eax
	leal	-16(%ebp), %ecx
	movl	%ecx, (%esp)
	leal	-20(%ebp), %ecx
	leal	(%edx,%eax,4), %esi
	movzbl	-34(%ebp), %eax
	movl	-40(%ebp), %edx
	subb	%bl, %al
	movzbl	%al, %eax
	movl	%eax, 4(%esp)
	movl	%esi, %eax
	call	LogAddr2PhyAddr
	movzbl	gNandMaxDie, %edx
	movzbl	%dl, %ecx
	movl	%eax, %edi
	movl	-16(%ebp), %eax
	cmpl	%eax, %ecx
	jbe	.L1458
	cmpb	$0, gMultiPageProgEn
	movl	$0, %ecx
	cmove	%ecx, %edi
	movl	%eax, %ecx
	sall	$4, %ecx
	movl	gDieOp+8(%ecx), %ecx
	testl	%ecx, %ecx
	je	.L1439
	cmpb	$1, %dl
	je	.L1459
.L1440:
	movzbl	%al, %eax
	call	FlashWaitCmdDone
	movl	-16(%ebp), %eax
.L1439:
	movl	%eax, %edx
	sall	$4, %edx
	testl	%edi, %edi
	movl	%edx, -52(%ebp)
	movl	-52(%ebp), %ecx
	movl	%esi, gDieOp+8(%edx)
	movl	$0, gDieOp+12(%edx)
	movl	-20(%ebp), %edx
	movl	%edx, -24(%ebp)
	movl	%edx, gDieOp+4(%ecx)
	je	.L1441
	movl	-44(%ebp), %ecx
	leal	9(%ebx,%ebx,8), %edx
	leal	(%ecx,%edx,4), %ecx
	movl	-52(%ebp), %edx
	movl	%ecx, gDieOp+12(%edx)
.L1441:
	movzbl	DieCsIndex(%eax), %ecx
	movl	%eax, %edx
	movl	DieAddrs(,%eax,4), %eax
	sall	$4, %edx
	movb	%cl, gDieOp(%edx)
	movzbl	%cl, %edx
	movl	%edx, -28(%ebp)
	movl	-24(%ebp), %edx
	movb	%cl, -33(%ebp)
	xorl	%ecx, %ecx
	testl	%eax, %eax
	movl	-28(%ebp), %eax
	setne	%cl
	call	FlashWaitReadyEN
	movl	gpNandParaInfo, %eax
	movzbl	19(%eax), %eax
	subl	$1, %eax
	cmpb	$4, %al
	ja	.L1442
	movl	-28(%ebp), %eax
	cmpb	$0, read_retry_cur_offset(%eax)
	jne	.L1460
.L1442:
	movl	gpNandc, %edx
	movzbl	-33(%ebp), %ecx
	movl	(%edx), %eax
	movl	$1, %edx
	sall	%cl, %edx
	movb	%dl, %al
	movl	gpNandc, %edx
	movl	%eax, (%edx)
	movl	-28(%ebp), %eax
	movl	-20(%ebp), %edx
	call	FlashProgFirstCmd
	movl	12(%esi), %eax
	movl	$1, %edx
	movl	-48(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	8(%esi), %eax
	movl	%eax, (%esp)
	movl	-28(%ebp), %eax
	call	NandcXferData
	testl	%edi, %edi
	movzbl	-33(%ebp), %esi
	jne	.L1443
	addl	$8, %esi
.L1444:
	movl	gpNandc, %eax
	sall	$8, %esi
	addl	%edi, %ebx
	addl	$1, %ebx
	addl	%eax, %esi
	movl	$16, 8(%esi)
	movl	(%eax), %edx
	andl	$-131328, %edx
	cmpl	%ebx, -32(%ebp)
	movl	%edx, (%eax)
	ja	.L1445
.L1435:
	movl	gpNandc, %eax
	call	NandcIqrWaitFlashReady
	cmpb	$0, gNandMaxDie
	je	.L1446
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L1447:
	movzbl	%bl, %eax
	addl	$1, %ebx
	call	FlashWaitCmdDone
	movzbl	gNandMaxDie, %eax
	cmpl	%ebx, %eax
	ja	.L1447
.L1446:
	movl	8(%ebp), %eax
	testl	%eax, %eax
	je	.L1448
	movl	-32(%ebp), %eax
	testl	%eax, %eax
	je	.L1448
	movl	-44(%ebp), %ebx
	xorl	%esi, %esi
	leal	-16(%ebp), %edi
	.p2align 4,,7
	.p2align 3
.L1453:
	cmpl	$-1, (%ebx)
	je	.L1461
	movzbl	-32(%ebp), %eax
	movl	%esi, %edx
	leal	-20(%ebp), %ecx
	movl	%edi, (%esp)
	subb	%dl, %al
	movl	-40(%ebp), %edx
	movzbl	%al, %eax
	movl	%eax, 4(%esp)
	movl	%ebx, %eax
	call	LogAddr2PhyAddr
	movl	gFlashProgCheckBuffer, %eax
	movl	$0, (%eax)
	movl	gFlashProgCheckSpareBuffer, %eax
	movl	$0, (%eax)
	movl	gFlashProgCheckSpareBuffer, %edx
	movl	-16(%ebp), %eax
	movl	gFlashProgCheckBuffer, %ecx
	movl	%edx, (%esp)
	movl	-20(%ebp), %edx
	movzbl	DieCsIndex(%eax), %eax
	call	FlashReadPage
	cmpl	$-1, %eax
	je	.L1462
.L1451:
	movl	12(%ebx), %eax
	testl	%eax, %eax
	je	.L1452
	movl	gFlashProgCheckSpareBuffer, %edx
	movl	(%eax), %eax
	movl	(%edx), %edx
	cmpl	%edx, %eax
	jne	.L1463
.L1452:
	movl	8(%ebx), %eax
	testl	%eax, %eax
	je	.L1450
	movl	gFlashProgCheckBuffer, %edx
	movl	(%eax), %eax
	movl	(%edx), %edx
	cmpl	%edx, %eax
	jne	.L1464
.L1450:
	addl	$1, %esi
	addl	$36, %ebx
	cmpl	-32(%ebp), %esi
	jne	.L1453
.L1448:
	addl	$60, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1443:
	addl	$8, %esi
	movzbl	gNandOptPara+10, %edx
	xorl	%ecx, %ecx
	movl	%esi, %eax
	sall	$8, %eax
	addl	gpNandc, %eax
	movl	%edx, 8(%eax)
	movl	-16(%ebp), %eax
	movl	-20(%ebp), %edx
	movl	DieAddrs(,%eax,4), %eax
	testl	%eax, %eax
	movl	-28(%ebp), %eax
	setne	%cl
	call	FlashWaitReadyEN
	movl	-28(%ebp), %eax
	movl	gBlockPageAlignSize, %edx
	addl	-20(%ebp), %edx
	call	FlashProgDpSecondCmd
	movl	-44(%ebp), %ecx
	leal	9(%ebx,%ebx,8), %eax
	leal	(%ecx,%eax,4), %eax
	movl	-48(%ebp), %ecx
	movl	12(%eax), %edx
	movl	%edx, 4(%esp)
	movl	8(%eax), %eax
	movl	$1, %edx
	movl	%eax, (%esp)
	movl	-28(%ebp), %eax
	call	NandcXferData
	jmp	.L1444
	.p2align 4,,7
	.p2align 3
.L1460:
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	movl	$0, (%esp)
	call	HynixSetRRPara
	jmp	.L1442
	.p2align 4,,7
	.p2align 3
.L1459:
	movl	gpNandc, %eax
	call	NandcIqrWaitFlashReady
	movl	-16(%ebp), %eax
	jmp	.L1440
.L1462:
	movl	4(%ebx), %eax
	movl	$.LC93, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	movl	$-1, (%ebx)
	jmp	.L1451
.L1464:
	movl	%edx, 12(%esp)
	movl	%eax, 8(%esp)
	movl	4(%ebx), %eax
	movl	$.LC95, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	movl	$-1, (%ebx)
	jmp	.L1450
.L1463:
	movl	%edx, 12(%esp)
	movl	%eax, 8(%esp)
	movl	4(%ebx), %eax
	movl	$.LC94, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	movl	$-1, (%ebx)
	jmp	.L1452
.L1461:
	movl	4(%ebx), %eax
	movl	$.LC92, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L1450
	.size	FlashProgPages, .-FlashProgPages
	.p2align 4,,15
	.globl	FtlVpcTblFlush
	.type	FtlVpcTblFlush, @function
FtlVpcTblFlush:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	movl	$-1, %esi
	pushl	%ebx
	subl	$4, %esp
	movl	p_sys_data_buf, %eax
	movl	p_sys_spare_buf, %ebx
	movl	%eax, req_sys+8
	movzwl	gSysInfo, %eax
	movl	%ebx, req_sys+12
	movw	$-3932, (%ebx)
	movl	$0, 12(%ebx)
	movw	%ax, 2(%ebx)
	movl	gSysInfo+8, %eax
	movl	$0, 8(%ebx)
	movzwl	g_active_superblock+2, %edx
	movl	$1179929683, g_sys_save_data
	movl	%eax, 4(%ebx)
	movzwl	gSysInfo+6, %eax
	sall	$6, %edx
	movl	$1342177319, g_sys_save_data+4
	movw	%ax, g_sys_save_data+8
	movzwl	c_ftl_nand_die_num, %eax
	movb	%al, g_sys_save_data+10
	movzwl	g_active_superblock, %eax
	movw	%ax, g_sys_save_data+14
	movzbl	g_active_superblock+6, %eax
	orl	%edx, %eax
	movzwl	g_buffer_superblock+2, %edx
	movw	%ax, g_sys_save_data+16
	movzbl	g_active_superblock+8, %eax
	sall	$6, %edx
	movb	%al, g_sys_save_data+11
	movzwl	g_buffer_superblock, %eax
	movw	%ax, g_sys_save_data+18
	movzbl	g_buffer_superblock+6, %eax
	orl	%edx, %eax
	movw	%ax, g_sys_save_data+20
	movzbl	g_buffer_superblock+8, %eax
	movb	%al, g_sys_save_data+12
	movzwl	g_gc_temp_superblock, %eax
	movzwl	g_gc_temp_superblock+2, %edx
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movw	%ax, g_sys_save_data+22
	movzbl	g_gc_temp_superblock+6, %eax
	sall	$6, %edx
	orl	%edx, %eax
	movl	$255, %edx
	movw	%ax, g_sys_save_data+24
	movzbl	g_gc_temp_superblock+8, %eax
	movb	%al, g_sys_save_data+13
	movl	g_totle_mlc_erase_count, %eax
	movl	%eax, g_sys_save_data+32
	movl	g_GlobalSysVersion, %eax
	movl	%eax, g_sys_save_data+40
	movl	g_GlobalDataVersion, %eax
	movl	%eax, g_sys_save_data+36
	movl	req_sys+8, %eax
	call	memset
	movl	req_sys+8, %eax
	movl	$1179929683, (%eax)
	movl	g_sys_save_data+4, %edx
	movl	%edx, 4(%eax)
	movl	g_sys_save_data+8, %edx
	movl	%edx, 8(%eax)
	movl	g_sys_save_data+12, %edx
	movl	%edx, 12(%eax)
	movl	g_sys_save_data+16, %edx
	movl	%edx, 16(%eax)
	movl	g_sys_save_data+20, %edx
	movl	%edx, 20(%eax)
	movl	g_sys_save_data+24, %edx
	movl	%edx, 24(%eax)
	movl	g_sys_save_data+28, %edx
	movl	%edx, 28(%eax)
	movl	g_sys_save_data+32, %edx
	movl	%edx, 32(%eax)
	movl	g_sys_save_data+36, %edx
	movl	%edx, 36(%eax)
	movl	g_sys_save_data+40, %edx
	movl	%edx, 40(%eax)
	movl	g_sys_save_data+44, %edx
	movl	%edx, 44(%eax)
	movl	req_sys+8, %eax
	movzwl	c_ftl_nand_data_blks_per_plane, %ecx
	movl	p_valid_page_count_table, %edx
	addl	$48, %eax
	addl	%ecx, %ecx
	call	memcpy
	xorl	%eax, %eax
	call	FtlUpdateVaildLpn
	movzwl	gSysInfo+2, %eax
	.p2align 4,,7
	.p2align 3
.L1473:
	movl	p_sys_data_buf, %edx
	movzwl	%ax, %eax
	movl	$1, %ecx
	movl	$1, (%esp)
	movl	%edx, req_sys+8
	movl	p_sys_spare_buf, %edx
	movl	%edx, req_sys+12
	movzwl	gSysInfo, %edx
	sall	$10, %edx
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashProgPages
	movzwl	gSysInfo+2, %eax
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	movzwl	%ax, %ecx
	subl	$1, %edx
	cmpl	%edx, %ecx
	jge	.L1474
.L1467:
	addl	$1, %eax
	cmpl	$-1, req_sys
	movw	%ax, gSysInfo+2
	je	.L1475
	cmpw	$1, %ax
	je	.L1473
	cmpw	$-1, %si
	je	.L1470
	movzwl	%si, %eax
	movl	$1, %edx
	call	FtlFreeSysBlkQueueIn
.L1470:
	addl	$4, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1474:
	movzwl	gSysInfo, %eax
	movzwl	gSysInfo+4, %esi
	movw	$0, gSysInfo+2
	movw	%ax, gSysInfo+4
	call	FtlFreeSysBlkQueueOut
	movl	g_GlobalSysVersion, %edx
	leal	1(%edx), %ecx
	movl	%edx, gSysInfo+8
	movw	%ax, gSysInfo
	movzwl	%ax, %eax
	sall	$10, %eax
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	movl	%edx, 4(%ebx)
	movl	$1, %edx
	movl	%ecx, g_GlobalSysVersion
	movl	$1, %ecx
	movl	$1, (%esp)
	call	FlashProgPages
	movzwl	gSysInfo+2, %eax
	jmp	.L1467
	.p2align 4,,7
	.p2align 3
.L1475:
	cmpw	$1, %ax
	jne	.L1473
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	subl	$1, %eax
	movw	%ax, gSysInfo+2
	jmp	.L1473
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.p2align 4,,15
	.globl	FtlMapWritePage
	.type	FtlMapWritePage, @function
FtlMapWritePage:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%ecx, %esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$8, %esp
	movl	%edx, -16(%ebp)
	movzwl	-16(%ebp), %edi
.L1482:
	addl	$1, g_totle_l2p_write_count
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	movzwl	2(%ebx), %ecx
	subl	$1, %eax
	cmpl	%eax, %ecx
	jge	.L1477
	movzwl	(%ebx), %edx
	cmpw	$-1, %dx
	je	.L1477
.L1478:
	movl	12(%ebx), %eax
	movzwl	%dx, %edx
	movzwl	(%eax,%edx,2), %edx
	movl	%esi, req_sys+8
	movzwl	%dx, %eax
	sall	$10, %eax
	orl	%ecx, %eax
	movl	%eax, req_sys+4
	movl	p_sys_spare_buf, %eax
	movl	%eax, req_sys+12
	movl	$0, (%eax)
	movl	$0, 4(%eax)
	movl	$0, 8(%eax)
	movl	$0, 12(%eax)
	movl	req_sys+12, %eax
	movl	28(%ebx), %ecx
	movw	%di, 8(%eax)
	movl	%ecx, 4(%eax)
	movzwl	4(%ebx), %ecx
	movw	%dx, 2(%eax)
	movl	$1, %edx
	movw	%cx, (%eax)
	movl	$1, %ecx
	movl	$req_sys, %eax
	movl	$1, (%esp)
	call	FlashProgPages
	movzwl	2(%ebx), %eax
	addl	$1, %eax
	cmpw	$1, %ax
	movw	%ax, 2(%ebx)
	je	.L1482
	cmpl	$-1, req_sys
	je	.L1482
	movl	24(%ebx), %eax
	movl	-16(%ebp), %ecx
	movl	req_sys+4, %edx
	movl	%edx, (%eax,%ecx,4)
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1477:
	movl	%ebx, %eax
	call	Ftl_write_map_blk_to_last_page
	movzwl	(%ebx), %edx
	movzwl	2(%ebx), %ecx
	jmp	.L1478
	.size	FtlMapWritePage, .-FtlMapWritePage
	.p2align 4,,15
	.globl	FtlMapBlkWriteDumpData
	.type	FtlMapBlkWriteDumpData, @function
FtlMapBlkWriteDumpData:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$16, %esp
	movl	%ebx, -12(%ebp)
	movl	%eax, %ebx
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	movzwl	6(%eax), %esi
	movl	24(%eax), %edi
	movl	36(%eax), %eax
	testl	%eax, %eax
	je	.L1483
	movl	p_sys_data_buf_1, %eax
	movl	$0, 36(%ebx)
	movl	%eax, req_sys+8
	movl	p_sys_spare_buf, %eax
	movl	%eax, req_sys+12
	movzwl	2(%ebx), %ecx
	movl	%eax, -16(%ebp)
	testw	%cx, %cx
	je	.L1485
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	movzwl	%cx, %ecx
	subl	$1, %eax
	cmpl	%eax, %ecx
	jge	.L1485
	movzwl	(%ebx), %edx
	cmpw	$-1, %dx
	je	.L1485
	movl	12(%ebx), %eax
	movzwl	%dx, %edx
	subl	$1, %ecx
	movzwl	(%eax,%edx,2), %eax
	movl	$1, %edx
	sall	$10, %eax
	orl	%ecx, %eax
	movl	$1, %ecx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1485
	movl	-16(%ebp), %ecx
	movl	24(%ebx), %eax
	movzwl	8(%ecx), %edx
	movl	req_sys+4, %ecx
	cmpl	%ecx, (%eax,%edx,4)
	je	.L1488
	.p2align 4,,7
	.p2align 3
.L1485:
	subl	$1, %esi
	movzwl	%si, %esi
	movl	(%edi,%esi,4), %eax
	testl	%eax, %eax
	movl	%eax, req_sys+4
	jne	.L1489
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movl	$255, %edx
	movl	req_sys+8, %eax
	call	memset
.L1487:
	movl	req_sys+8, %ecx
	movl	%esi, %edx
	movl	%ebx, %eax
	call	FtlMapWritePage
.L1483:
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1489:
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	call	FlashReadPages
	jmp	.L1487
.L1488:
	movl	req_sys+8, %ecx
	movl	%ebx, %eax
	call	FtlMapWritePage
	jmp	.L1483
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.p2align 4,,15
	.globl	FtlVendorPartWrite
	.type	FtlVendorPartWrite, @function
FtlVendorPartWrite:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$60, %esp
	movl	%eax, -52(%ebp)
	addl	%edx, %eax
	cmpl	$4352, %eax
	movl	%ecx, -60(%ebp)
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	$-1, -72(%ebp)
	jbe	.L1501
.L1491:
	movl	-72(%ebp), %eax
	addl	$60, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1501:
	testl	%edx, %edx
	movl	$0, -72(%ebp)
	je	.L1491
	movl	-52(%ebp), %eax
	movl	%edx, -56(%ebp)
	shrl	%cl, %eax
	movl	%eax, -64(%ebp)
	sall	$2, %eax
	movl	%eax, -68(%ebp)
	jmp	.L1496
	.p2align 4,,7
	.p2align 3
.L1502:
	movl	p_vendor_data_buf, %eax
	movl	$1, %ecx
	movl	$1, %edx
	movl	%esi, -44(%ebp)
	movl	$0, -36(%ebp)
	movl	%eax, -40(%ebp)
	leal	-48(%ebp), %eax
	call	FlashReadPages
.L1494:
	movl	-60(%ebp), %edx
	movzwl	%di, %eax
	movl	%ebx, %esi
	sall	$9, %eax
	addl	p_vendor_data_buf, %eax
	sall	$9, %esi
	movl	%esi, %ecx
	call	memcpy
	movl	p_vendor_data_buf, %ecx
	movl	$gVendorBlkInfo, %eax
	movl	-64(%ebp), %edx
	call	FtlMapWritePage
	cmpl	$-1, %eax
	movl	$-1, %eax
	cmovne	-72(%ebp), %eax
	addl	$1, -64(%ebp)
	addl	%ebx, -52(%ebp)
	addl	%esi, -60(%ebp)
	addl	$4, -68(%ebp)
	subl	%ebx, -56(%ebp)
	movl	%eax, -72(%ebp)
	je	.L1491
.L1496:
	movl	p_vendor_region_ppn_table, %eax
	movl	-68(%ebp), %edx
	movzwl	c_ftl_nand_sec_pre_page, %ecx
	movl	(%eax,%edx), %esi
	xorl	%edx, %edx
	movl	-52(%ebp), %eax
	movzwl	%cx, %ebx
	divl	%ebx
	movl	%ecx, %eax
	subw	%dx, %ax
	movl	%edx, %edi
	movzwl	%ax, %ebx
	cmpl	-56(%ebp), %ebx
	jbe	.L1492
	movzwl	-56(%ebp), %eax
	movzwl	%ax, %ebx
.L1492:
	testl	%esi, %esi
	je	.L1493
	cmpw	%cx, %ax
	jne	.L1502
.L1493:
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	xorl	%edx, %edx
	movl	p_vendor_data_buf, %eax
	call	memset
	jmp	.L1494
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.p2align 4,,15
	.globl	Ftl_save_ext_data
	.type	Ftl_save_ext_data, @function
Ftl_save_ext_data:
	cmpl	$1179929683, g_sys_ext_data
	pushl	%ebp
	movl	%esp, %ebp
	je	.L1505
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1505:
	movl	g_totle_write_sector, %eax
	movl	$g_sys_ext_data, %ecx
	movl	$1, %edx
	movl	$1342177319, g_sys_ext_data+4
	movl	%eax, g_sys_ext_data+88
	movl	g_totle_read_sector, %eax
	movl	%eax, g_sys_ext_data+92
	movl	g_totle_gc_page_count, %eax
	movl	%eax, g_sys_ext_data+8
	movl	g_totle_write_page_count, %eax
	movl	%eax, g_sys_ext_data+12
	movl	g_totle_read_page_count, %eax
	movl	%eax, g_sys_ext_data+16
	movl	g_totle_l2p_write_count, %eax
	movl	%eax, g_sys_ext_data+20
	movl	g_totle_slc_erase_count, %eax
	movl	%eax, g_sys_ext_data+28
	movl	g_totle_sys_slc_erase_count, %eax
	movl	%eax, g_sys_ext_data+32
	movl	g_totle_discard_page_count, %eax
	movl	%eax, g_sys_ext_data+36
	movl	g_totle_cache_write_count, %eax
	movl	%eax, g_sys_ext_data+40
	movl	g_max_erase_count, %eax
	movl	%eax, g_sys_ext_data+44
	movl	g_min_erase_count, %eax
	movl	%eax, g_sys_ext_data+48
	movl	g_all_blk_used_slc_mode, %eax
	movl	%eax, g_sys_ext_data+60
	movl	g_SlcPartLbaEndSector, %eax
	movl	%eax, g_sys_ext_data+64
	xorl	%eax, %eax
	call	FtlVendorPartWrite
	popl	%ebp
	ret
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.p2align 4,,15
	.globl	FtlEctTblFlush
	.type	FtlEctTblFlush, @function
FtlEctTblFlush:
	movl	g_inkDie_check_enable, %ecx
	movl	$32, %edx
	pushl	%ebp
	movl	%esp, %ebp
	testl	%ecx, %ecx
	je	.L1507
	cmpl	$30, g_min_erase_count
	sbbl	%edx, %edx
	andl	$-28, %edx
	addl	$32, %edx
.L1507:
	movzwl	g_ect_tbl_power_up_flush, %ecx
	cmpw	$31, %cx
	ja	.L1508
	addl	$1, %ecx
	movl	$1, %edx
	movw	%cx, g_ect_tbl_power_up_flush
.L1508:
	testl	%eax, %eax
	movl	gp_ect_tbl_info, %eax
	jne	.L1509
	addl	20(%eax), %edx
	cmpl	%edx, 16(%eax)
	jb	.L1510
.L1509:
	movl	16(%eax), %edx
	movl	$1112818501, (%eax)
	movl	gp_ect_tbl_info, %ecx
	movl	%edx, 20(%eax)
	movzwl	g_ect_tbl_info_size, %edx
	addl	$1, 8(%ecx)
	movl	$0, 4(%ecx)
	movl	%edx, %eax
	sall	$9, %eax
	movl	%eax, 12(%ecx)
	movl	$64, %eax
	call	FtlVendorPartWrite
	call	Ftl_save_ext_data
.L1510:
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.p2align 4,,15
	.globl	flush_l2p_region
	.type	flush_l2p_region, @function
flush_l2p_region:
	pushl	%ebp
	movzwl	%ax, %eax
	movl	%esp, %ebp
	pushl	%ebx
	leal	(%eax,%eax,2), %ebx
	movl	p_l2p_ram_map, %eax
	sall	$2, %ebx
	addl	%ebx, %eax
	movl	8(%eax), %ecx
	movzwl	(%eax), %edx
	movl	$gL2pMapInfo, %eax
	call	FtlMapWritePage
	xorl	%eax, %eax
	addl	p_l2p_ram_map, %ebx
	andl	$2147483647, 4(%ebx)
	popl	%ebx
	popl	%ebp
	ret
	.size	flush_l2p_region, .-flush_l2p_region
	.p2align 4,,15
	.globl	log2phys
	.type	log2phys, @function
log2phys:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$20, %esp
	movl	p_l2p_ram_map, %edi
	movl	%ecx, -24(%ebp)
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%edx, -20(%ebp)
	movl	$1, %edx
	addl	$7, %ecx
	sall	%cl, %edx
	shrl	%cl, %ebx
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %ecx
	subl	$1, %edx
	andl	%eax, %edx
	movw	%dx, -14(%ebp)
	testw	%cx, %cx
	je	.L1518
	xorl	%esi, %esi
	movl	%edi, %edx
	cmpw	%bx, (%edi)
	je	.L1519
	subl	$1, %ecx
	movzwl	%cx, %ecx
	leal	12(%edi), %eax
	leal	(%ecx,%ecx,2), %edx
	leal	(%eax,%edx,4), %ecx
	jmp	.L1521
	.p2align 4,,7
	.p2align 3
.L1525:
	movl	%eax, %esi
	movl	%eax, %edx
	addl	$12, %eax
	subl	%edi, %esi
	cmpw	%bx, -12(%eax)
	je	.L1519
.L1521:
	cmpl	%ecx, %eax
	jne	.L1525
.L1518:
	call	select_l2p_ram_region
	movzwl	%ax, %edx
	leal	(%edx,%edx,2), %ecx
	sall	$2, %ecx
	addl	%ecx, %edi
	movl	%ecx, %esi
	cmpw	$-1, (%edi)
	je	.L1527
	movl	4(%edi), %edi
	testl	%edi, %edi
	js	.L1532
.L1527:
	movzwl	%bx, %eax
	movl	%ecx, -28(%ebp)
	call	load_l2p_region
	movl	-28(%ebp), %ecx
	movl	p_l2p_ram_map, %edx
	addl	%ecx, %edx
.L1519:
	movl	-24(%ebp), %eax
	testl	%eax, %eax
	jne	.L1522
	movl	8(%edx), %eax
	movzwl	-14(%ebp), %ecx
	movl	-20(%ebp), %edx
	movl	(%eax,%ecx,4), %eax
	movl	%eax, (%edx)
	addl	p_l2p_ram_map, %esi
	movl	4(%esi), %eax
.L1523:
	cmpl	$-1, %eax
	je	.L1524
	addl	$1, %eax
	movl	%eax, 4(%esi)
.L1524:
	addl	$20, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1522:
	movl	-20(%ebp), %eax
	movzwl	-14(%ebp), %edi
	movl	(%eax), %ecx
	movl	8(%edx), %eax
	movl	%ecx, (%eax,%edi,4)
	addl	p_l2p_ram_map, %esi
	movw	%bx, g_l2p_last_update_region_id
	movl	4(%esi), %eax
	orl	$-2147483648, %eax
	movl	%eax, 4(%esi)
	jmp	.L1523
.L1532:
	movl	%edx, %eax
	movl	%edx, -32(%ebp)
	movl	%ecx, -28(%ebp)
	call	flush_l2p_region
	movl	-28(%ebp), %ecx
	movl	-32(%ebp), %edx
	jmp	.L1527
	.size	log2phys, .-log2phys
	.p2align 4,,15
	.globl	FtlReadRefresh
	.type	FtlReadRefresh, @function
FtlReadRefresh:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	subl	$40, %esp
	movl	g_sys_ext_data+80, %eax
	testl	%eax, %eax
	je	.L1534
	movl	g_sys_ext_data+84, %eax
	movl	$2048, %ebx
	cmpl	g_MaxLpn, %eax
	jae	.L1547
	.p2align 4,,7
	.p2align 3
.L1545:
	xorl	%ecx, %ecx
	leal	-12(%ebp), %edx
	call	log2phys
	movl	g_sys_ext_data+84, %eax
	movl	-12(%ebp), %edx
	addl	$1, %eax
	cmpl	$-1, %edx
	movl	%eax, g_sys_ext_data+84
	jne	.L1548
	subl	$1, %ebx
	je	.L1542
	cmpl	%eax, g_MaxLpn
	ja	.L1545
.L1542:
	addl	$40, %esp
	movl	$-1, %ebx
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1534:
	movl	g_totle_read_page_count, %eax
	movl	g_sys_ext_data+76, %edx
	movl	g_max_erase_count, %esi
	movl	g_inkDie_check_enable, %ecx
	leal	1048576(%eax), %ebx
	cmpl	%ebx, %edx
	jbe	.L1549
.L1540:
	movl	$1, g_sys_ext_data+80
	xorl	%ebx, %ebx
	movl	$0, g_sys_ext_data+84
	movl	%eax, g_sys_ext_data+76
.L1537:
	addl	$40, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1549:
	shrl	$10, %esi
	movl	$33554432, %ebx
	addl	%esi, %ecx
	sarl	%cl, %ebx
	addl	%ebx, %edx
	xorl	%ebx, %ebx
	cmpl	%edx, %eax
	ja	.L1540
	jmp	.L1537
	.p2align 4,,7
	.p2align 3
.L1547:
	movl	g_totle_read_page_count, %eax
	xorl	%ebx, %ebx
	movl	$0, g_sys_ext_data+80
	movl	$0, g_sys_ext_data+84
	movl	%eax, g_sys_ext_data+76
	addl	$40, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1548:
	movl	%edx, -44(%ebp)
	xorl	%ecx, %ecx
	movl	$1, %edx
	movl	%eax, -32(%ebp)
	leal	-48(%ebp), %eax
	movl	$-1, %ebx
	movl	$0, -40(%ebp)
	movl	$0, -36(%ebp)
	movl	$0, -48(%ebp)
	call	FlashReadPages
	cmpl	$256, -48(%ebp)
	jne	.L1537
	movl	-12(%ebp), %eax
	xorl	%edx, %edx
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	call	FtlGcRefreshBlock
	jmp	.L1537
	.size	FtlReadRefresh, .-FtlReadRefresh
	.p2align 4,,15
	.globl	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, @function
FtlReUsePrevPpa:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	movl	%edx, %eax
	subl	$12, %esp
	shrl	$10, %eax
	movl	%edx, -16(%ebp)
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %edx
	movl	%edx, -20(%ebp)
	addl	%edx, %edx
	movl	%edx, -24(%ebp)
	addl	p_valid_page_count_table, %edx
	movzwl	(%edx), %ecx
	testw	%cx, %cx
	jne	.L1551
	movl	p_free_data_block_list_head, %edx
	testl	%edx, %edx
	je	.L1552
	movzwl	g_num_free_superblocks, %edi
	movl	p_data_block_list_table, %esi
	testw	%di, %di
	je	.L1552
	subl	%esi, %edx
	sarl	%edx
	imulw	$-21845, %dx, %dx
	cmpw	%ax, %dx
	jne	.L1555
	jmp	.L1553
	.p2align 4,,7
	.p2align 3
.L1559:
	addl	$1, %ecx
	cmpw	%di, %cx
	je	.L1552
	cmpw	%ax, %dx
	.p2align 4,,2
	je	.L1553
.L1555:
	movzwl	%dx, %edx
	leal	(%edx,%edx,2), %edx
	movzwl	(%esi,%edx,2), %edx
	cmpw	$-1, %dx
	jne	.L1559
	.p2align 4,,7
	.p2align 3
.L1552:
	leal	-16(%ebp), %edx
	movl	%ebx, %eax
	movl	$1, %ecx
	call	log2phys
	addl	$12, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1551:
	addl	$1, %ecx
	movw	%cx, (%edx)
	jmp	.L1552
	.p2align 4,,7
	.p2align 3
.L1553:
	movl	-20(%ebp), %edx
	movl	$p_free_data_block_list_head, %eax
	call	List_remove_node
	movl	-20(%ebp), %eax
	subw	$1, g_num_free_superblocks
	call	INSERT_DATA_LIST
	movl	-24(%ebp), %eax
	addl	p_valid_page_count_table, %eax
	addw	$1, (%eax)
	jmp	.L1552
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.p2align 4,,15
	.globl	ftl_map_blk_gc
	.type	ftl_map_blk_gc, @function
ftl_map_blk_gc:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$24, %esp
	movl	%ebx, -12(%ebp)
	movl	%eax, %ebx
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	movl	12(%eax), %esi
	movl	24(%eax), %eax
	movl	%eax, -16(%ebp)
	movl	%ebx, %eax
	call	ftl_free_no_use_map_blk
	movzwl	10(%ebx), %edx
	movzwl	8(%ebx), %ecx
	subl	$1, %edx
	cmpl	%edx, %ecx
	jl	.L1561
	movzwl	%ax, %eax
	leal	(%esi,%eax,2), %eax
	movzwl	(%eax), %esi
	testw	%si, %si
	jne	.L1570
.L1561:
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	cmpw	%ax, 2(%ebx)
	jb	.L1567
	movl	%ebx, %eax
	call	ftl_map_blk_alloc_new_blk
.L1567:
	movl	-12(%ebp), %ebx
	xorl	%eax, %eax
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1570:
	movl	32(%ebx), %edx
	testl	%edx, %edx
	jne	.L1561
	movl	$1, 32(%ebx)
	movw	$0, (%eax)
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	subw	$1, 8(%ebx)
	cmpw	%ax, 2(%ebx)
	jae	.L1571
.L1562:
	cmpw	$0, 6(%ebx)
	movzwl	%si, %esi
	movl	%esi, -24(%ebp)
	je	.L1563
	xorl	%esi, %esi
	movl	%ebx, -20(%ebp)
	jmp	.L1566
	.p2align 4,,7
	.p2align 3
.L1564:
	movl	-20(%ebp), %eax
	addl	$1, %esi
	cmpw	%si, 6(%eax)
	jbe	.L1572
.L1566:
	movl	-16(%ebp), %eax
	movzwl	%si, %edi
	leal	(%eax,%edi,4), %ebx
	movl	(%ebx), %eax
	shrl	$10, %eax
	cmpl	%eax, -24(%ebp)
	jne	.L1564
	movl	p_sys_data_buf_1, %eax
	movl	$1, %ecx
	movl	$1, %edx
	movl	%eax, req_sys+8
	movl	p_sys_spare_buf, %eax
	movl	%eax, req_sys+12
	movl	(%ebx), %eax
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	jne	.L1565
	movl	$0, (%ebx)
	jmp	.L1564
	.p2align 4,,7
	.p2align 3
.L1572:
	movl	%eax, %ebx
.L1563:
	movl	-24(%ebp), %eax
	movl	$1, %edx
	call	FtlFreeSysBlkQueueIn
	movl	$0, 32(%ebx)
	jmp	.L1561
	.p2align 4,,7
	.p2align 3
.L1565:
	movl	req_sys+8, %ecx
	movl	%edi, %edx
	movl	-20(%ebp), %eax
	call	FtlMapWritePage
	jmp	.L1564
	.p2align 4,,7
	.p2align 3
.L1571:
	movl	%ebx, %eax
	call	ftl_map_blk_alloc_new_blk
	.p2align 4,,6
	jmp	.L1562
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.p2align 4,,15
	.globl	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, @function
Ftl_write_map_blk_to_last_page:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$16, %esp
	movl	12(%eax), %esi
	movl	24(%eax), %eax
	movl	%eax, -20(%ebp)
	movzwl	(%ebx), %eax
	cmpw	$-1, %ax
	je	.L1580
	movzwl	%ax, %eax
	movzwl	2(%ebx), %ecx
	movzwl	(%esi,%eax,2), %edx
	movzwl	%dx, %esi
	movl	%esi, %eax
	sall	$10, %eax
	orl	%ecx, %eax
	movl	28(%ebx), %ecx
	movl	%eax, req_sys+4
	movl	p_sys_data_buf, %eax
	movl	%esi, -16(%ebp)
	movl	%eax, req_sys+8
	movl	p_sys_spare_buf, %eax
	movl	%eax, req_sys+12
	movl	%ecx, 4(%eax)
	movw	$-1291, 8(%eax)
	movzwl	4(%ebx), %ecx
	movw	%dx, 2(%eax)
	movl	$255, %edx
	movw	%cx, (%eax)
	movzwl	c_ftl_nand_page_pre_slc_blk, %ecx
	movl	p_sys_data_buf, %eax
	sall	$3, %ecx
	call	memset
	cmpw	$0, 6(%ebx)
	je	.L1576
	movw	$0, -22(%ebp)
	xorl	%eax, %eax
	jmp	.L1578
	.p2align 4,,7
	.p2align 3
.L1577:
	addl	$1, %eax
	cmpw	%ax, 6(%ebx)
	jbe	.L1576
.L1578:
	movl	-20(%ebp), %esi
	movzwl	%ax, %ecx
	leal	(%esi,%ecx,4), %edx
	movl	(%edx), %esi
	shrl	$10, %esi
	cmpl	%esi, -16(%ebp)
	jne	.L1577
	addw	$1, -22(%ebp)
	addl	$1, %eax
	movzwl	-22(%ebp), %esi
	movl	p_sys_data_buf, %edi
	movl	%ecx, (%edi,%esi,8)
	movl	(%edx), %ecx
	movl	p_sys_data_buf, %edx
	movl	%ecx, 4(%edx,%esi,8)
	cmpw	%ax, 6(%ebx)
	ja	.L1578
.L1576:
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	movl	$0, (%esp)
	call	FlashProgPages
	movl	%ebx, %eax
	addw	$1, 2(%ebx)
	call	ftl_map_blk_gc
	addl	$16, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1580:
	addw	$1, 8(%ebx)
	call	FtlFreeSysBlkQueueOut
	movw	%ax, (%esi)
	xorl	%eax, %eax
	movw	$0, 2(%ebx)
	movw	$0, (%ebx)
	addl	$1, 28(%ebx)
	addl	$16, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.p2align 4,,15
	.globl	FtlMapTblRecovery
	.type	FtlMapTblRecovery, @function
FtlMapTblRecovery:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$40, %esp
	movl	24(%eax), %ebx
	movl	%eax, -20(%ebp)
	movl	%ebx, -28(%ebp)
	movl	12(%eax), %ebx
	movl	%ebx, -36(%ebp)
	movl	16(%eax), %ebx
	movl	%ebx, -44(%ebp)
	movzwl	6(%eax), %ebx
	movw	%bx, -14(%ebp)
	movzwl	-14(%ebp), %ecx
	movzwl	8(%eax), %ebx
	movl	-28(%ebp), %eax
	sall	$2, %ecx
	call	memset
	movl	p_sys_data_buf, %eax
	testl	%ebx, %ebx
	movl	p_sys_spare_buf, %esi
	movl	%ebx, -32(%ebp)
	movl	%eax, req_sys+8
	movl	-20(%ebp), %eax
	movl	%esi, req_sys+12
	movl	$0, 32(%eax)
	movw	$-1, (%eax)
	movw	$-1, 2(%eax)
	movl	$0, 28(%eax)
	movl	$1, 36(%eax)
	je	.L1582
	subl	$1, %ebx
	movl	%ebx, -40(%ebp)
	je	.L1596
	movw	$0, -24(%ebp)
	xorl	%eax, %eax
	movl	%esi, %edi
	.p2align 4,,7
	.p2align 3
.L1584:
	movl	-36(%ebp), %ebx
	movl	$1, %ecx
	movl	p_sys_data_buf, %edx
	leal	(%ebx,%eax,2), %esi
	movl	%edx, req_sys+8
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	movzwl	(%esi), %eax
	subl	$1, %edx
	sall	$10, %eax
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1587
	movl	-20(%ebp), %eax
	movzwl	4(%eax), %eax
	cmpw	%ax, (%edi)
	je	.L1601
.L1587:
	cmpw	$0, c_ftl_nand_page_pre_slc_blk
	movl	$0, req_sys+8
	je	.L1588
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	jmp	.L1592
	.p2align 4,,7
	.p2align 3
.L1591:
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	addl	$1, %ebx
	movswl	%bx, %edx
	cmpl	%eax, %edx
	jge	.L1588
.L1592:
	movzwl	(%esi), %eax
	movl	$1, %ecx
	sall	$10, %eax
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1591
	movzwl	8(%edi), %eax
	cmpw	%ax, -14(%ebp)
	jbe	.L1591
	movl	-20(%ebp), %edx
	movzwl	4(%edx), %edx
	cmpw	%dx, (%edi)
	jne	.L1591
	movl	req_sys+4, %edx
	movzwl	%ax, %eax
	movl	-28(%ebp), %ecx
	movl	%edx, (%ecx,%eax,4)
	jmp	.L1591
	.p2align 4,,7
	.p2align 3
.L1601:
	cmpw	$-1291, 8(%edi)
	jne	.L1587
	movzwl	c_ftl_nand_page_pre_slc_blk, %ebx
	leal	-1(%ebx), %eax
	testl	%eax, %eax
	jle	.L1588
	movl	%edi, -48(%ebp)
	movzwl	-14(%ebp), %edi
	xorl	%eax, %eax
	xorl	%edx, %edx
	.p2align 4,,7
	.p2align 3
.L1590:
	movl	p_sys_data_buf, %ecx
	leal	0(,%eax,8), %esi
	movzwl	(%ecx,%eax,8), %eax
	cmpw	%ax, %di
	jbe	.L1589
	movl	-28(%ebp), %ebx
	movzwl	%ax, %eax
	movl	4(%ecx,%esi), %ecx
	movl	%ecx, (%ebx,%eax,4)
	movzwl	c_ftl_nand_page_pre_slc_blk, %ebx
.L1589:
	addl	$1, %edx
	movswl	%dx, %eax
	leal	-1(%ebx), %ecx
	cmpl	%ecx, %eax
	jl	.L1590
	movl	-48(%ebp), %edi
	.p2align 4,,7
	.p2align 3
.L1588:
	addw	$1, -24(%ebp)
	movswl	-24(%ebp), %eax
	cmpl	-32(%ebp), %eax
	jge	.L1582
	cmpl	-40(%ebp), %eax
	jne	.L1584
	movl	%edi, %esi
	movl	-40(%ebp), %ebx
	movl	-40(%ebp), %edi
	sall	$2, %ebx
	addl	%edi, %edi
.L1583:
	addl	-36(%ebp), %edi
	movl	$1, %edx
	movzwl	(%edi), %eax
	call	FtlGetLastWrittenPage
	movl	-20(%ebp), %ecx
	leal	1(%eax), %edx
	cwtl
	movw	%dx, 2(%ecx)
	movzwl	-24(%ebp), %edx
	addl	$1, %eax
	testl	%eax, %eax
	movl	%eax, -24(%ebp)
	movw	%dx, (%ecx)
	movl	-44(%ebp), %ecx
	movl	(%ecx,%ebx), %edx
	movl	-20(%ebp), %ebx
	movl	$0, req_sys+8
	movl	%edx, 28(%ebx)
	jle	.L1582
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	jmp	.L1586
	.p2align 4,,7
	.p2align 3
.L1585:
	addl	$1, %ebx
	movswl	%bx, %edx
	cmpl	-24(%ebp), %edx
	jge	.L1582
.L1586:
	movzwl	(%edi), %eax
	movl	$1, %ecx
	sall	$10, %eax
	orl	%edx, %eax
	movl	$1, %edx
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	cmpl	$-1, req_sys
	je	.L1585
	movzwl	8(%esi), %eax
	cmpw	%ax, -14(%ebp)
	jbe	.L1585
	movl	-20(%ebp), %edx
	movzwl	4(%edx), %edx
	cmpw	%dx, (%esi)
	jne	.L1585
	movl	req_sys+4, %edx
	addl	$1, %ebx
	movzwl	%ax, %eax
	movl	-28(%ebp), %ecx
	movl	%edx, (%ecx,%eax,4)
	movswl	%bx, %edx
	cmpl	-24(%ebp), %edx
	jl	.L1586
	.p2align 4,,7
	.p2align 3
.L1582:
	movl	-20(%ebp), %eax
	call	ftl_free_no_use_map_blk
	movl	-20(%ebp), %eax
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	cmpw	%dx, 2(%eax)
	je	.L1602
.L1594:
	movl	-20(%ebp), %eax
	movzwl	10(%eax), %edx
	cmpw	%dx, 8(%eax)
	jb	.L1595
	call	ftl_map_blk_gc
.L1595:
	addl	$40, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1602:
	call	ftl_map_blk_alloc_new_blk
	jmp	.L1594
.L1596:
	xorl	%ebx, %ebx
	xorl	%edi, %edi
	movw	$0, -24(%ebp)
	jmp	.L1583
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.p2align 4,,15
	.globl	FtlLoadMapInfo
	.type	FtlLoadMapInfo, @function
FtlLoadMapInfo:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlL2PDataInit
	movl	$gL2pMapInfo, %eax
	call	FtlMapTblRecovery
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.p2align 4,,15
	.globl	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, @function
FtlLoadVonderInfo:
	movzwl	c_ftl_nand_max_vendor_blks, %eax
	pushl	%ebp
	movl	%esp, %ebp
	movw	$-3962, gVendorBlkInfo+4
	movw	%ax, gVendorBlkInfo+10
	movzwl	g_totle_vendor_block, %eax
	movw	%ax, gVendorBlkInfo+8
	movzwl	c_ftl_nand_vendor_region_num, %eax
	movw	%ax, gVendorBlkInfo+6
	movl	p_vendor_block_table, %eax
	movl	%eax, gVendorBlkInfo+12
	movl	p_vendor_block_ver_table, %eax
	movl	%eax, gVendorBlkInfo+16
	movl	p_vendor_block_valid_page_count, %eax
	movl	%eax, gVendorBlkInfo+20
	movl	p_vendor_region_ppn_table, %eax
	movl	%eax, gVendorBlkInfo+24
	movl	$gVendorBlkInfo, %eax
	call	FtlMapTblRecovery
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.section	.rodata.str1.4
	.align 4
.LC96:
	.string	"\0013FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\n"
	.section	.rodata.str1.1
.LC97:
	.string	"\0013FtlBbmTblFlush error:%x\n"
	.text
	.p2align 4,,15
	.globl	FtlBbmTblFlush
	.type	FtlBbmTblFlush, @function
FtlBbmTblFlush:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movl	p_sys_spare_buf, %edx
	movl	p_sys_data_buf, %eax
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movl	%edx, req_sys+12
	xorl	%edx, %edx
	movl	%eax, req_sys+8
	call	memset
	cmpw	$0, c_ftl_nand_die_num
	je	.L1606
	movzwl	c_ftl_nand_bbm_buf_size, %ecx
	xorl	%ebx, %ebx
	jmp	.L1607
	.p2align 4,,7
	.p2align 3
.L1613:
	movzwl	c_ftl_nand_bbm_buf_size, %ecx
.L1607:
	movzwl	%cx, %ecx
	movl	%ebx, %eax
	movl	req_sys+8, %esi
	imull	%ecx, %eax
	movl	gBbtInfo+28(,%ebx,4), %edx
	addl	$1, %ebx
	sall	$2, %ecx
	leal	(%esi,%eax,4), %eax
	call	memcpy
	movzwl	c_ftl_nand_die_num, %eax
	cmpl	%ebx, %eax
	jg	.L1613
.L1606:
	movl	req_sys+12, %ebx
	xorl	%esi, %esi
	movl	$-1, (%ebx)
	movl	$-1, 8(%ebx)
	movl	$-1, 4(%ebx)
	movl	$-1, 12(%ebx)
	movw	$-3887, (%ebx)
	movl	gBbtInfo+8, %eax
	movl	%eax, 4(%ebx)
	movzwl	gBbtInfo, %eax
	movw	%ax, 2(%ebx)
	movzwl	gBbtInfo+6, %ecx
	movzwl	gBbtInfo+4, %edx
	movw	%cx, 10(%ebx)
	movl	c_ftl_nand_sys_blks_per_plane, %ecx
	movw	%dx, 8(%ebx)
	movw	%cx, 12(%ebx)
.L1608:
	movl	p_sys_data_buf, %ecx
	movzwl	%dx, %edx
	movzwl	gBbtInfo+2, %edi
	movl	$0, req_sys
	movl	%ecx, req_sys+8
	movl	p_sys_spare_buf, %ecx
	movl	%ecx, req_sys+12
	movzwl	%ax, %ecx
	movl	%ecx, %eax
	sall	$10, %eax
	orl	%edi, %eax
	movl	%eax, req_sys+4
	movzwl	10(%ebx), %eax
	movl	%edx, 12(%esp)
	movl	%ecx, 4(%esp)
	movl	%edi, 8(%esp)
	movl	%eax, 16(%esp)
	movl	$.LC96, (%esp)
	call	printk
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	movl	$1, (%esp)
	call	FlashProgPages
	movzwl	gBbtInfo+2, %eax
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	movzwl	%ax, %ecx
	subl	$1, %edx
	cmpl	%edx, %ecx
	jge	.L1614
.L1609:
	addl	$1, %eax
	cmpl	$-1, req_sys
	movw	%ax, gBbtInfo+2
	je	.L1615
	testl	%esi, %esi
	je	.L1612
	addl	$20, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1614:
	movl	gBbtInfo+8, %eax
	movw	$0, gBbtInfo+2
	addl	$1, %eax
	movl	%eax, gBbtInfo+8
	movl	%eax, 4(%ebx)
	movzwl	gBbtInfo, %eax
	movzwl	gBbtInfo+4, %edx
	movw	%ax, 8(%ebx)
	movw	%ax, gBbtInfo+4
	movl	req_erase, %eax
	movw	%dx, gBbtInfo
	sall	$10, %edx
	movl	%edx, req_sys+4
	movl	%edx, 4(%eax)
	movl	$1, %edx
	call	FlashEraseBlocks
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	movl	$1, (%esp)
	call	FlashProgPages
	movzwl	gBbtInfo+2, %eax
	jmp	.L1609
.L1612:
	movl	$1, %esi
.L1611:
	movzwl	gBbtInfo, %eax
	movzwl	gBbtInfo+4, %edx
	jmp	.L1608
.L1615:
	movl	req_sys+4, %eax
	movl	$.LC97, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L1611
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.section	.rodata.str1.1
.LC98:
	.string	"\0013FtlGcFreeBadSuperBlk 0x%x\n"
	.text
	.p2align 4,,15
	.globl	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, @function
FtlGcFreeBadSuperBlk:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movzwl	g_gc_bad_block_temp_num, %ecx
	movw	%ax, -14(%ebp)
	testw	%cx, %cx
	je	.L1617
	cmpw	$0, c_ftl_nand_planes_num
	je	.L1618
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L1625:
	movzwl	%bx, %eax
	movzwl	c_ftl_nand_planes_per_die, %esi
	xorl	%edx, %edx
	movzbl	p_plane_order_table(%eax), %eax
	divw	%si
	imulw	-14(%ebp), %si
	imulw	c_ftl_nand_blks_per_die, %ax
	addl	%edx, %esi
	addl	%eax, %esi
	testw	%cx, %cx
	je	.L1619
	xorl	%edi, %edi
	movzwl	%si, %edx
	.p2align 4,,7
	.p2align 3
.L1624:
	movzwl	%di, %eax
	cmpw	%si, g_gc_bad_block_temp_tbl(%eax,%eax)
	je	.L1620
	movzwl	g_gc_bad_block_temp_num, %eax
.L1621:
	addl	$1, %edi
	cmpw	%ax, %di
	jb	.L1624
.L1619:
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_planes_num
	jbe	.L1618
	movzwl	g_gc_bad_block_temp_num, %ecx
	jmp	.L1625
.L1618:
	call	FtlGcReFreshBadBlk
.L1617:
	addl	$20, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1620:
	movl	%edx, 4(%esp)
	movl	%edx, -20(%ebp)
	movl	$.LC98, (%esp)
	call	printk
	movl	-20(%ebp), %edx
	movl	%edx, %eax
	call	FtlBbmMapBadBlock
	call	FtlBbmTblFlush
	movzwl	g_gc_bad_block_temp_num, %eax
	movl	-20(%ebp), %edx
	cmpw	%ax, %di
	movw	%ax, -16(%ebp)
	jae	.L1622
	movl	%edi, %eax
	movl	%edx, -24(%ebp)
.L1623:
	movzwl	%ax, %ecx
	addl	$1, %eax
	movzwl	g_gc_bad_block_temp_tbl+2(%ecx,%ecx), %edx
	cmpw	-16(%ebp), %ax
	movw	%dx, g_gc_bad_block_temp_tbl(%ecx,%ecx)
	jb	.L1623
	movl	-24(%ebp), %edx
.L1622:
	movzwl	-16(%ebp), %eax
	subl	$1, %eax
	movw	%ax, g_gc_bad_block_temp_num
	jmp	.L1621
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.p2align 4,,15
	.globl	update_vpc_list
	.type	update_vpc_list, @function
update_vpc_list:
	pushl	%ebp
	movl	%eax, %edx
	movl	%esp, %ebp
	pushl	%ebx
	movzwl	%ax, %ebx
	movl	p_valid_page_count_table, %eax
	cmpw	$0, (%eax,%ebx,2)
	jne	.L1629
	cmpw	%dx, g_gc_superblock
	je	.L1636
	xorl	%eax, %eax
	cmpw	%dx, g_active_superblock
	je	.L1632
	cmpw	%dx, g_buffer_superblock
	je	.L1632
	cmpw	%dx, g_gc_temp_superblock
	je	.L1632
.L1631:
	movl	%ebx, %edx
	movl	$p_data_block_list_head, %eax
	call	List_remove_node
	movl	%ebx, %eax
	subw	$1, g_num_data_superblocks
	call	free_data_superblock
	movl	%ebx, %eax
	call	FtlGcFreeBadSuperBlk
	movl	$1, %eax
.L1632:
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1629:
	movl	%ebx, %eax
	call	List_update_data_list
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1636:
	movw	$-1, g_gc_superblock
	jmp	.L1631
	.size	update_vpc_list, .-update_vpc_list
	.section	.rodata.str1.4
	.align 4
.LC99:
	.string	"\0013decrement_vpc_count %x = %d\n"
	.text
	.p2align 4,,15
	.globl	decrement_vpc_count
	.type	decrement_vpc_count, @function
decrement_vpc_count:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	subl	$12, %esp
	cmpw	$-1, %ax
	je	.L1638
	movzwl	%ax, %ecx
	leal	(%ecx,%ecx), %eax
	addl	p_valid_page_count_table, %eax
	movzwl	(%eax), %edx
	testw	%dx, %dx
	je	.L1643
	subl	$1, %edx
	movw	%dx, (%eax)
.L1638:
	movzwl	g_tmp_data_superblock_id, %edx
	cmpw	$-1, %dx
	je	.L1644
	xorl	%eax, %eax
	cmpw	%dx, %bx
	je	.L1640
	movzwl	%dx, %eax
	call	update_vpc_list
	movw	%bx, g_tmp_data_superblock_id
	testl	%eax, %eax
	setne	%al
	movzbl	%al, %eax
.L1640:
	addl	$12, %esp
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1644:
	movw	%bx, g_tmp_data_superblock_id
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
.L1643:
	movl	$0, 8(%esp)
	movl	%ecx, 4(%esp)
	movl	$.LC99, (%esp)
	call	printk
	xorl	%eax, %eax
	jmp	.L1640
	.size	decrement_vpc_count, .-decrement_vpc_count
	.p2align 4,,15
	.globl	get_new_active_ppa
	.type	get_new_active_ppa, @function
get_new_active_ppa:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$4, %esp
	movb	$0, 10(%eax)
	movzbl	6(%eax), %eax
	movzwl	16(%ebx,%eax,2), %eax
	jmp	.L1646
	.p2align 4,,7
	.p2align 3
.L1649:
	movzwl	16(%ebx,%eax,2), %eax
.L1646:
	cmpw	$-1, %ax
	je	.L1647
	movzbl	8(%ebx), %ecx
	cmpb	$1, %cl
	jne	.L1650
	movzwl	2(%ebx), %edx
	cmpw	$-1, mlcPageToSlcPageTbl(%edx,%edx)
	jne	.L1658
	subw	$1, 4(%ebx)
	movzwl	(%ebx), %eax
	call	decrement_vpc_count
.L1647:
	movzbl	6(%ebx), %eax
	addl	$1, %eax
	movzbl	%al, %edx
	cmpw	c_ftl_nand_planes_num, %dx
	movb	%al, 6(%ebx)
	movzbl	%al, %eax
	jne	.L1649
	addw	$1, 2(%ebx)
	xorl	%eax, %eax
	movb	$0, 6(%ebx)
	jmp	.L1649
.L1650:
	movzwl	2(%ebx), %edx
	movl	%ecx, %edi
.L1651:
	movzwl	%ax, %eax
	subw	$1, 4(%ebx)
	sall	$10, %eax
	orl	%edx, %eax
	movl	%eax, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L1656:
	movzbl	6(%ebx), %edx
	movzwl	c_ftl_nand_planes_num, %esi
	jmp	.L1654
	.p2align 4,,7
	.p2align 3
.L1660:
	movzbl	%al, %ecx
	movl	%eax, %edx
	cmpw	$-1, 16(%ebx,%ecx,2)
	jne	.L1659
.L1654:
	leal	1(%edx), %eax
	movzbl	%al, %edx
	cmpw	%si, %dx
	jne	.L1660
	addw	$1, 2(%ebx)
	xorl	%ecx, %ecx
	xorl	%eax, %eax
	xorl	%edx, %edx
	cmpw	$-1, 16(%ebx,%ecx,2)
	je	.L1654
.L1659:
	movb	%al, 6(%ebx)
	movl	%edi, %eax
	cmpb	$1, %al
	jne	.L1655
	movzwl	2(%ebx), %eax
	cmpw	$-1, mlcPageToSlcPageTbl(%eax,%eax)
	jne	.L1655
	movzwl	4(%ebx), %eax
	testw	%ax, %ax
	je	.L1655
	subl	$1, %eax
	movw	%ax, 4(%ebx)
	movzwl	(%ebx), %eax
	call	decrement_vpc_count
	movzbl	8(%ebx), %edi
	jmp	.L1656
.L1655:
	movl	-16(%ebp), %eax
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1658:
	movl	%ecx, %edi
	jmp	.L1651
	.size	get_new_active_ppa, .-get_new_active_ppa
	.p2align 4,,15
	.globl	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, @function
FtlSlcSuperblockCheck:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	cmpw	$0, 4(%eax)
	je	.L1661
	movzbl	6(%eax), %eax
	movzwl	16(%ebx,%eax,2), %eax
	jmp	.L1663
	.p2align 4,,7
	.p2align 3
.L1666:
	movzwl	16(%ebx,%eax,2), %eax
.L1663:
	cmpw	$-1, %ax
	je	.L1664
	cmpb	$1, 8(%ebx)
	jne	.L1661
	movzwl	2(%ebx), %eax
	cmpw	$-1, mlcPageToSlcPageTbl(%eax,%eax)
	jne	.L1661
	subw	$1, 4(%ebx)
	movzwl	(%ebx), %eax
	call	decrement_vpc_count
	cmpw	$0, 4(%ebx)
	je	.L1667
.L1664:
	movzbl	6(%ebx), %eax
	addl	$1, %eax
	movzbl	%al, %edx
	cmpw	c_ftl_nand_planes_num, %dx
	movb	%al, 6(%ebx)
	movzbl	%al, %eax
	jne	.L1666
	addw	$1, 2(%ebx)
	xorl	%eax, %eax
	movb	$0, 6(%ebx)
	jmp	.L1666
	.p2align 4,,7
	.p2align 3
.L1667:
	addw	$1, 2(%ebx)
	movb	$0, 6(%ebx)
	.p2align 4,,7
	.p2align 3
.L1661:
	popl	%ebx
	popl	%ebp
	ret
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.section	.rodata.str1.1
.LC100:
	.string	"\0013spuer block %x vpn is 0\n "
	.text
	.p2align 4,,15
	.globl	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, @function
FtlRecoverySuperblock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$76, %esp
	movzwl	2(%eax), %edi
	movl	%eax, -56(%ebp)
	movzbl	6(%eax), %eax
	cmpw	c_ftl_nand_page_pre_blk, %di
	movw	%di, -52(%ebp)
	movb	%al, -70(%ebp)
	je	.L1783
	movl	-56(%ebp), %edx
	movzwl	16(%edx), %eax
	cmpw	$-1, %ax
	jne	.L1671
	movl	-56(%ebp), %ecx
	xorl	%edx, %edx
	.p2align 4,,7
	.p2align 3
.L1672:
	addl	$1, %edx
	movzwl	%dx, %eax
	movzwl	16(%ecx,%eax,2), %eax
	cmpw	$-1, %ax
	je	.L1672
.L1671:
	movl	-56(%ebp), %ecx
	movzwl	%ax, %eax
	cmpb	$1, 8(%ecx)
	je	.L1784
	xorl	%edx, %edx
	call	FtlGetLastWrittenPage
	cmpl	$-1, %eax
	movl	%eax, -68(%ebp)
	je	.L1674
	movl	%eax, -28(%ebp)
.L1675:
	movzwl	c_ftl_nand_planes_num, %edi
	testw	%di, %di
	je	.L1785
	movl	req_read, %edx
	xorl	%ebx, %ebx
	movl	p_gc_spare_buf, %ecx
	movzwl	c_ftl_nand_byte_pre_oob, %eax
	movw	%di, -64(%ebp)
	movl	%edx, -32(%ebp)
	xorl	%edx, %edx
	movl	%ecx, -36(%ebp)
	movl	%eax, -60(%ebp)
	.p2align 4,,7
	.p2align 3
.L1679:
	movl	-56(%ebp), %ecx
	movzwl	16(%ecx,%edx,2), %eax
	cmpw	$-1, %ax
	je	.L1678
	movl	-32(%ebp), %edi
	movzwl	%ax, %eax
	movzwl	%bx, %esi
	sall	$10, %eax
	orl	-28(%ebp), %eax
	leal	(%esi,%esi,8), %ecx
	leal	(%edi,%ecx,4), %ecx
	movl	%eax, 4(%ecx)
	movl	-60(%ebp), %eax
	movl	$0, 8(%ecx)
	imull	%esi, %eax
	testl	%eax, %eax
	leal	3(%eax), %esi
	cmovs	%esi, %eax
	addl	$1, %ebx
	andl	$-4, %eax
	addl	-36(%ebp), %eax
	movl	%eax, 12(%ecx)
.L1678:
	addl	$1, %edx
	cmpw	%dx, -64(%ebp)
	ja	.L1679
	movl	req_read, %eax
	xorl	%ecx, %ecx
	movzwl	%bx, %edx
	call	FlashReadPages
	movl	g_GlobalDataVersion, %eax
	subl	$1, %eax
	testw	%bx, %bx
	movl	%eax, -64(%ebp)
	je	.L1680
	movl	req_read, %eax
	xorl	%edx, %edx
	movzwl	-28(%ebp), %ecx
	movw	$-1, -32(%ebp)
	movl	%eax, -60(%ebp)
	movw	%cx, -36(%ebp)
	jmp	.L1688
	.p2align 4,,7
	.p2align 3
.L1787:
	movl	12(%eax), %esi
	movl	4(%esi), %ecx
	cmpl	$-1, %ecx
	je	.L1682
	movl	g_GlobalDataVersion, %edi
	cmpl	%edi, %ecx
	movl	%edi, -76(%ebp)
	jbe	.L1683
	movl	%ecx, %edi
	subl	-76(%ebp), %edi
	cmpl	$-2147483648, %edi
	ja	.L1682
.L1684:
	addl	$1, %ecx
	movl	%ecx, g_GlobalDataVersion
.L1682:
	cmpl	$-1, (%esi)
	je	.L1786
.L1685:
	addl	$1, %edx
	addl	$36, %eax
	cmpw	%bx, %dx
	je	.L1686
.L1688:
	movl	(%eax), %ecx
	testl	%ecx, %ecx
	je	.L1787
	movzwl	-36(%ebp), %ecx
	addl	$1, %edx
	addl	$36, %eax
	cmpw	%bx, %dx
	movw	%cx, -32(%ebp)
	jne	.L1688
.L1686:
	movzwl	-68(%ebp), %edi
	movl	-60(%ebp), %eax
	addl	$1, %edi
	movw	%di, -68(%ebp)
	movl	4(%eax), %ebx
	jmp	.L1778
	.p2align 4,,7
	.p2align 3
.L1683:
	movl	-76(%ebp), %edi
	subl	%ecx, %edi
	cmpl	$-2147483648, %edi
	ja	.L1684
	cmpl	$-1, (%esi)
	jne	.L1685
.L1786:
	cmpw	%bx, %dx
	je	.L1686
	movl	-60(%ebp), %edi
	movzwl	%dx, %edx
	leal	(%edx,%edx,8), %eax
	movl	4(%edi,%eax,4), %ebx
.L1778:
	shrl	$10, %ebx
	xorl	%edx, %edx
	movzwl	c_ftl_nand_planes_per_die, %esi
	movl	%ebx, %eax
	divw	c_ftl_nand_blks_per_die
	xorl	%edx, %edx
	movl	%eax, %ecx
	movl	%ebx, %eax
	divw	%si
	imull	%esi, %ecx
	addl	%ecx, %edx
	movw	%dx, -72(%ebp)
	movl	-56(%ebp), %edx
	movzbl	8(%edx), %eax
	cmpb	$1, %al
	je	.L1788
.L1690:
	movzwl	-68(%ebp), %edi
	cmpw	c_ftl_nand_page_pre_blk, %di
	je	.L1789
.L1691:
	movzbl	-70(%ebp), %edi
	movw	%di, -70(%ebp)
	movzwl	-52(%ebp), %edi
	cmpw	%di, -68(%ebp)
	je	.L1790
.L1692:
	cmpw	$-1, -32(%ebp)
	je	.L1791
.L1693:
	movl	g_recovery_page_min_ver, %edi
	cmpl	$-1, %edi
	movl	%edi, -80(%ebp)
	je	.L1792
.L1695:
	movzwl	-52(%ebp), %edx
	movzwl	-28(%ebp), %ecx
	movzwl	-28(%ebp), %eax
	movzwl	-52(%ebp), %edi
	addl	$7, %edx
	movl	$-1, -36(%ebp)
	subl	$7, %eax
	cmpl	%edx, %ecx
	cmovg	%eax, %edi
	cmpw	%di, -28(%ebp)
	movw	%di, -60(%ebp)
	movl	$-1, -76(%ebp)
	jb	.L1698
.L1773:
	movzwl	c_ftl_nand_planes_num, %esi
	testw	%si, %si
	je	.L1710
	movzwl	-60(%ebp), %eax
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	movl	req_read, %edi
	movl	%eax, -32(%ebp)
	.p2align 4,,7
	.p2align 3
.L1700:
	movl	-56(%ebp), %ecx
	movzwl	16(%ecx,%edx,2), %eax
	cmpw	$-1, %ax
	je	.L1699
	movzwl	%ax, %eax
	movzwl	%bx, %ecx
	sall	$10, %eax
	addl	$1, %ebx
	orl	-32(%ebp), %eax
	leal	(%ecx,%ecx,8), %ecx
	movl	%eax, 4(%edi,%ecx,4)
.L1699:
	addl	$1, %edx
	cmpw	%dx, %si
	ja	.L1700
	movl	req_read, %eax
	xorl	%ecx, %ecx
	movzwl	%bx, %edx
	call	FlashReadPages
	testw	%bx, %bx
	je	.L1701
	movl	req_read, %ecx
	movl	(%ecx), %eax
	testl	%eax, %eax
	jne	.L1702
	leal	36(%ecx), %eax
	xorl	%edx, %edx
	movzwl	-60(%ebp), %esi
	movl	-76(%ebp), %edi
	jmp	.L1703
	.p2align 4,,7
	.p2align 3
.L1704:
	addl	$1, %edx
	cmpw	%bx, %dx
	je	.L1793
.L1709:
	movl	%eax, %ecx
	addl	$36, %eax
	cmpl	$0, -36(%eax)
	jne	.L1777
.L1703:
	movl	12(%ecx), %ecx
	cmpw	$-1, (%ecx)
	je	.L1704
	movl	4(%ecx), %ecx
	cmpl	$-1, %ecx
	je	.L1704
	cmpl	$-1, -36(%ebp)
	movl	g_recovery_page_min_ver, %edi
	movl	%ecx, g_recovery_page_min_ver
	jne	.L1704
	cmpw	$-1, mlcPageToSlcPageTbl(%esi,%esi)
	je	.L1704
	movl	-36(%ebp), %ecx
	cmpl	-64(%ebp), %edi
	cmovne	%edi, %ecx
	addl	$1, %edx
	cmpw	%bx, %dx
	movl	%ecx, -36(%ebp)
	jne	.L1709
	.p2align 4,,7
	.p2align 3
.L1793:
	movl	%edi, -76(%ebp)
.L1701:
	addw	$1, -60(%ebp)
	movzwl	-60(%ebp), %edi
	cmpw	%di, -28(%ebp)
	jae	.L1773
.L1698:
	movl	$-1, g_recovery_page_min_ver
	.p2align 4,,7
	.p2align 3
.L1694:
	movl	$gL2pMapInfo, %eax
	movw	$1, g_power_lost_recovery_flag
	call	FtlMapBlkWriteDumpData
	movzwl	-52(%ebp), %edi
	movw	%di, -36(%ebp)
.L1711:
	movzwl	c_ftl_nand_planes_num, %esi
	testw	%si, %si
	je	.L1712
	movzwl	-36(%ebp), %eax
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	movl	req_read, %edi
	movl	%eax, -28(%ebp)
	.p2align 4,,7
	.p2align 3
.L1714:
	movl	-56(%ebp), %ecx
	movzwl	16(%ecx,%edx,2), %eax
	cmpw	$-1, %ax
	je	.L1713
	movzwl	%ax, %eax
	movzwl	%bx, %ecx
	sall	$10, %eax
	addl	$1, %ebx
	orl	-28(%ebp), %eax
	leal	(%ecx,%ecx,8), %ecx
	movl	%eax, 4(%edi,%ecx,4)
.L1713:
	addl	$1, %edx
	cmpw	%dx, %si
	ja	.L1714
	movl	req_read, %eax
	xorl	%ecx, %ecx
	movzwl	%bx, %edx
	call	FlashReadPages
	testw	%bx, %bx
	je	.L1715
	subl	$1, %ebx
	xorl	%esi, %esi
	movzwl	%bx, %ebx
	leal	9(%ebx,%ebx,8), %eax
	sall	$2, %eax
	movl	%eax, -60(%ebp)
	jmp	.L1754
	.p2align 4,,7
	.p2align 3
.L1721:
	movl	4(%eax), %edi
	cmpl	$-1, %edi
	movl	%edi, -64(%ebp)
	je	.L1722
	movl	g_GlobalDataVersion, %edx
	cmpl	%edx, %edi
	jbe	.L1723
	subl	%edx, %edi
	cmpl	$-2147483648, %edi
	ja	.L1722
.L1724:
	movl	-64(%ebp), %edx
	addl	$1, %edx
	movl	%edx, g_GlobalDataVersion
.L1722:
	movl	8(%eax), %edi
	xorl	%ecx, %ecx
	movl	12(%eax), %eax
	leal	-20(%ebp), %edx
	movl	%eax, -24(%ebp)
	movl	%edi, %eax
	call	log2phys
	movl	g_recovery_page_min_ver, %eax
	cmpl	$-1, %eax
	je	.L1725
	cmpl	%eax, -64(%ebp)
	jbe	.L1726
	movl	-64(%ebp), %edx
	subl	%eax, %edx
	cmpl	$-2147483648, %edx
	ja	.L1725
.L1727:
	movl	-24(%ebp), %edx
	cmpl	$-1, %edx
	je	.L1728
	movl	req_read, %eax
	xorl	%ecx, %ecx
	addl	%esi, %eax
	movl	%edx, 4(%eax)
	movl	$1, %edx
	movl	12(%eax), %ebx
	call	FlashReadPages
	movl	req_read, %eax
	movl	4(%ebx), %ecx
	addl	%esi, %eax
	cmpl	$-1, (%eax)
	movl	%ecx, -32(%ebp)
	je	.L1730
	movl	8(%ebx), %ecx
	cmpl	%edi, %ecx
	movl	%ecx, -28(%ebp)
	je	.L1794
.L1730:
	movl	-56(%ebp), %edx
	movl	$-1, -24(%ebp)
	movzwl	(%edx), %eax
	call	decrement_vpc_count
	movl	-24(%ebp), %ecx
.L1743:
	cmpl	$-1, %ecx
	je	.L1716
.L1746:
	movl	%ecx, %eax
	xorl	%edx, %edx
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	cmpw	$0, (%edx,%eax,2)
	je	.L1751
	call	decrement_vpc_count
	.p2align 4,,7
	.p2align 3
.L1716:
	addl	$36, %esi
	cmpl	-60(%ebp), %esi
	je	.L1715
.L1754:
	movl	req_read, %ebx
	movzwl	c_ftl_nand_planes_per_die, %eax
	movzwl	-52(%ebp), %edx
	movzwl	c_ftl_nand_blks_per_die, %edi
	addl	%esi, %ebx
	movl	4(%ebx), %ecx
	cmpw	%dx, -36(%ebp)
	movw	%ax, -28(%ebp)
	movl	%ecx, -16(%ebp)
	jb	.L1716
	movl	%ecx, %eax
	xorl	%edx, %edx
	shrl	$10, %eax
	movw	%ax, -32(%ebp)
	divw	%di
	xorl	%edx, %edx
	movl	%eax, %edi
	movzwl	-32(%ebp), %eax
	imulw	-28(%ebp), %di
	divw	-28(%ebp)
	addl	%edx, %edi
	movzwl	-52(%ebp), %edx
	cmpw	%dx, -36(%ebp)
	jne	.L1763
	cmpw	%di, -70(%ebp)
	ja	.L1716
.L1763:
	movzwl	-68(%ebp), %eax
	cmpw	%ax, -36(%ebp)
	je	.L1795
.L1764:
	cmpl	$-1, (%ebx)
	je	.L1720
	movl	12(%ebx), %eax
	cmpw	$-3947, (%eax)
	je	.L1721
.L1731:
	movl	-56(%ebp), %edx
	addl	$36, %esi
	movzwl	(%edx), %eax
	call	decrement_vpc_count
	cmpl	-60(%ebp), %esi
	jne	.L1754
	.p2align 4,,7
	.p2align 3
.L1715:
	addw	$1, -36(%ebp)
	movzwl	-36(%ebp), %edi
	cmpw	c_ftl_nand_page_pre_blk, %di
	jne	.L1711
	movzwl	c_ftl_nand_planes_num, %ecx
	movl	-56(%ebp), %eax
	testw	%cx, %cx
	movw	%di, 2(%eax)
	movw	$0, 4(%eax)
	je	.L1670
	cmpw	$-1, 16(%eax)
	jne	.L1796
	movl	-56(%ebp), %edx
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L1757:
	addl	$1, %eax
	cmpw	%cx, %ax
	je	.L1670
	addl	$2, %edx
	cmpw	$-1, 16(%edx)
	je	.L1757
.L1755:
	movl	-56(%ebp), %edx
	movb	%al, 6(%edx)
	jmp	.L1670
.L1784:
	movl	$1, %edx
	call	FtlGetLastWrittenPage
	cmpl	$-1, %eax
	movl	%eax, -68(%ebp)
	je	.L1674
	movzwl	slcPageToMlcPageTbl(%eax,%eax), %edi
	movl	%edi, -28(%ebp)
	jmp	.L1675
	.p2align 4,,7
	.p2align 3
.L1777:
	movl	%edi, -76(%ebp)
.L1702:
	movl	-56(%ebp), %eax
	cmpb	$0, 8(%eax)
	jne	.L1694
	movzwl	-60(%ebp), %eax
	cmpw	$-1, mlcPageToSlcPageTbl(%eax,%eax)
	je	.L1797
	movl	-64(%ebp), %edi
	cmpl	%edi, -76(%ebp)
	je	.L1708
	cmpl	$-1, -76(%ebp)
	je	.L1694
	movl	-76(%ebp), %edi
.L1779:
	movl	%edi, g_recovery_page_min_ver
	jmp	.L1694
	.p2align 4,,7
	.p2align 3
.L1720:
	movl	g_recovery_page_num, %eax
	cmpl	$31, %eax
	ja	.L1752
	movl	%ecx, g_recovery_ppa_tbl(,%eax,4)
	addl	$1, %eax
	movl	%eax, g_recovery_page_num
.L1752:
	movl	-56(%ebp), %edi
	movzwl	(%edi), %eax
	call	decrement_vpc_count
	movl	g_recovery_page_min_ver, %eax
	cmpl	$-1, %eax
	je	.L1781
	cmpl	%eax, -64(%ebp)
	jae	.L1716
.L1781:
	movl	-64(%ebp), %edi
	movl	%edi, g_recovery_page_min_ver
	jmp	.L1716
	.p2align 4,,7
	.p2align 3
.L1726:
	subl	-64(%ebp), %eax
	cmpl	$-2147483648, %eax
	ja	.L1727
.L1725:
	movl	-16(%ebp), %ecx
	cmpl	%ecx, -20(%ebp)
	je	.L1780
	movl	$1, %ecx
	movl	%edi, %eax
	leal	-16(%ebp), %edx
	call	log2phys
	movl	-20(%ebp), %ecx
	cmpl	$-1, %ecx
	movl	%ecx, -28(%ebp)
	movl	-24(%ebp), %ecx
	je	.L1743
	cmpl	%ecx, -28(%ebp)
	je	.L1746
	movl	-28(%ebp), %eax
	xorl	%edx, %edx
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	cmpw	%ax, g_active_superblock
	je	.L1747
	cmpw	%ax, g_buffer_superblock
	je	.L1747
	cmpw	%ax, g_gc_temp_superblock
	jne	.L1743
.L1747:
	movl	req_read, %eax
	movl	$1, %edx
	movl	-28(%ebp), %ecx
	movl	12(%eax), %ebx
	movl	%ecx, 4(%eax)
	xorl	%ecx, %ecx
	call	FlashReadPages
	movl	req_read, %eax
	movl	4(%ebx), %edx
	cmpl	$-1, (%eax)
	je	.L1780
	cmpl	%edx, -64(%ebp)
	jbe	.L1749
	movl	-64(%ebp), %eax
	subl	%edx, %eax
	cmpl	$-2147483648, %eax
	jbe	.L1780
.L1750:
	movl	$1, %ecx
	movl	%edi, %eax
	leal	-20(%ebp), %edx
	call	log2phys
	movl	-24(%ebp), %ecx
	jmp	.L1743
.L1749:
	subl	-64(%ebp), %edx
	cmpl	$-2147483648, %edx
	jbe	.L1750
	.p2align 4,,7
	.p2align 3
.L1780:
	movl	-24(%ebp), %ecx
	jmp	.L1743
	.p2align 4,,7
	.p2align 3
.L1723:
	subl	-64(%ebp), %edx
	cmpl	$-2147483648, %edx
	jbe	.L1722
	jmp	.L1724
.L1728:
	movl	-16(%ebp), %eax
	cmpl	%eax, -20(%ebp)
	jne	.L1731
	movl	$1, %ecx
	movl	%edi, %eax
	leal	-24(%ebp), %edx
	call	log2phys
	jmp	.L1731
	.p2align 4,,7
	.p2align 3
.L1795:
	cmpw	%di, -72(%ebp)
	jne	.L1764
.L1718:
	movzbl	-72(%ebp), %ecx
	movl	-56(%ebp), %eax
	movzwl	-68(%ebp), %edi
	movb	%cl, 6(%eax)
	movzwl	-72(%ebp), %ecx
	movw	%di, 2(%eax)
	movzwl	%di, %edx
	call	ftl_sb_update_avl_pages
.L1670:
	addl	$76, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1783:
	movl	-56(%ebp), %eax
	movw	$0, 4(%eax)
	movb	$0, 6(%eax)
	addl	$76, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1674:
	movl	-56(%ebp), %eax
	movw	$0, 2(%eax)
	movb	$0, 6(%eax)
	addl	$76, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1797:
	cmpl	$-1, -36(%ebp)
	je	.L1706
	movl	-36(%ebp), %edi
	movl	%edi, g_recovery_page_min_ver
	jmp	.L1694
.L1789:
	movl	-56(%ebp), %edx
	movw	%di, 2(%edx)
	movb	$0, 6(%edx)
	movw	$0, 4(%edx)
	jmp	.L1691
.L1788:
	movzwl	-68(%ebp), %edx
	movzwl	slcPageToMlcPageTbl(%edx,%edx), %edx
	movw	%dx, -68(%ebp)
	jmp	.L1690
.L1712:
	movl	req_read, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	jmp	.L1715
.L1791:
	testb	%al, %al
	jne	.L1694
	.p2align 4,,9
	jmp	.L1693
.L1790:
	movzwl	-70(%ebp), %edi
	cmpw	%di, -72(%ebp)
	jne	.L1692
	movl	-56(%ebp), %eax
	movzwl	-72(%ebp), %ecx
	movzwl	-68(%ebp), %edx
	call	ftl_sb_update_avl_pages
	addl	$76, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1792:
	movl	-64(%ebp), %edi
	movl	%edi, g_recovery_page_min_ver
	movl	%edi, -80(%ebp)
	jmp	.L1695
.L1794:
	movl	g_recovery_page_min_ver, %edx
	cmpl	%edx, -32(%ebp)
	jb	.L1798
	movl	-32(%ebp), %edi
	subl	%edx, %edi
	cmpl	$-2147483648, %edi
	jbe	.L1730
.L1733:
	movl	-20(%ebp), %edx
	cmpl	-16(%ebp), %edx
	je	.L1737
	cmpl	-24(%ebp), %edx
	je	.L1730
	cmpl	$-1, %edx
	je	.L1735
	movl	%edx, 4(%eax)
	xorl	%ecx, %ecx
	movl	$1, %edx
	movl	12(%eax), %ebx
	call	FlashReadPages
.L1736:
	movl	req_read, %eax
	cmpl	$-1, (%eax,%esi)
	je	.L1737
	movl	4(%ebx), %eax
	movl	g_recovery_page_min_ver, %edx
	cmpl	%edx, %eax
	jae	.L1738
	subl	%eax, %edx
	cmpl	$-2147483648, %edx
	ja	.L1737
.L1740:
	cmpl	%eax, -32(%ebp)
	jbe	.L1741
	movl	-32(%ebp), %edi
	subl	%eax, %edi
	cmpl	$-2147483648, %edi
	ja	.L1730
.L1737:
	movl	-24(%ebp), %edx
	movl	-28(%ebp), %eax
	call	FtlReUsePrevPpa
	jmp	.L1730
.L1710:
	movl	req_read, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	jmp	.L1701
.L1785:
	movl	req_read, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	movl	g_GlobalDataVersion, %eax
	subl	$1, %eax
	movl	%eax, -64(%ebp)
.L1680:
	movl	req_read, %eax
	movw	$-1, -32(%ebp)
	movl	%eax, -60(%ebp)
	jmp	.L1686
.L1798:
	subl	-32(%ebp), %edx
	cmpl	$-2147483648, %edx
	ja	.L1730
	jmp	.L1733
.L1706:
	movl	-80(%ebp), %edi
	cmpl	%edi, -64(%ebp)
	jne	.L1779
	subl	$1, g_recovery_page_min_ver
	jmp	.L1694
	.p2align 4,,7
	.p2align 3
.L1708:
	movl	g_recovery_page_min_ver, %eax
	cmpl	-64(%ebp), %eax
	je	.L1694
	subl	$1, %eax
	movl	%eax, g_recovery_page_min_ver
	jmp	.L1694
.L1735:
	movl	$-1, (%eax)
	jmp	.L1736
.L1738:
	movl	%eax, %ecx
	subl	%edx, %ecx
	cmpl	$-2147483648, %ecx
	jbe	.L1737
	jmp	.L1740
.L1741:
	subl	-32(%ebp), %eax
	cmpl	$-2147483648, %eax
	jbe	.L1730
	.p2align 4,,3
	jmp	.L1737
.L1796:
	xorl	%eax, %eax
	jmp	.L1755
.L1751:
	movl	%eax, 4(%esp)
	movl	$.LC100, (%esp)
	call	printk
	jmp	.L1716
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.p2align 4,,15
	.globl	allocate_data_superblock
	.type	allocate_data_superblock, @function
allocate_data_superblock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$28, %esp
	.p2align 4,,7
	.p2align 3
.L1832:
	cmpl	$g_gc_temp_superblock, %ebx
	je	.L1833
.L1801:
	xorl	%edx, %edx
	cmpb	$1, 8(%ebx)
	je	.L1834
.L1803:
	movl	$p_free_data_block_list_head, %eax
	call	List_pop_index_node
	subw	$1, g_num_free_superblocks
	movw	%ax, -22(%ebp)
	movw	%ax, (%ebx)
	movl	%ebx, %eax
	call	make_superblock
	cmpb	$0, 7(%ebx)
	je	.L1804
	movzwl	c_ftl_nand_planes_num, %ecx
	xorl	%edi, %edi
	testw	%cx, %cx
	je	.L1806
	movl	req_erase, %esi
	subl	$1, %ecx
	movl	%ebx, %edx
	movzwl	%cx, %ecx
	leal	(%ecx,%ecx,8), %ecx
	movl	%ebx, -20(%ebp)
	leal	36(%esi,%ecx,4), %edi
	movl	%esi, %eax
	movl	%esi, -16(%ebp)
	xorl	%ecx, %ecx
	movl	%edi, -36(%ebp)
	.p2align 4,,7
	.p2align 3
.L1809:
	movl	$0, 8(%eax)
	movl	$0, 12(%eax)
	movzwl	16(%edx), %ebx
	cmpw	$-1, %bx
	je	.L1808
	movl	-16(%ebp), %edi
	movzwl	%cx, %esi
	movzwl	%bx, %ebx
	leal	(%esi,%esi,8), %esi
	addl	$1, %ecx
	sall	$10, %ebx
	movl	%ebx, 4(%edi,%esi,4)
.L1808:
	addl	$36, %eax
	addl	$2, %edx
	cmpl	-36(%ebp), %eax
	jne	.L1809
	movl	-20(%ebp), %ebx
	movzwl	%cx, %edi
.L1806:
	movl	g_inkDie_check_enable, %esi
	movzwl	-22(%ebp), %eax
	testl	%esi, %esi
	movl	%eax, -20(%ebp)
	jne	.L1810
	addl	%eax, %eax
	movl	%eax, -16(%ebp)
	addl	p_erase_count_table, %eax
	movzwl	(%eax), %edx
.L1811:
	cmpb	$0, 8(%ebx)
	jne	.L1813
	testw	%dx, %dx
	je	.L1814
	addw	c_mlc_erase_count_value, %dx
	movw	%dx, (%eax)
.L1815:
	movl	g_totle_mlc_erase_count, %edx
	movl	g_totle_slc_erase_count, %esi
	addl	$1, %edx
	movl	%edx, g_totle_mlc_erase_count
	movl	%esi, -40(%ebp)
.L1816:
	movl	p_erase_count_table, %eax
	movl	-16(%ebp), %esi
	movzwl	(%eax,%esi), %esi
	cmpl	g_max_erase_count, %esi
	movl	-40(%ebp), %eax
	movl	%esi, -36(%ebp)
	cmovbe	g_max_erase_count, %esi
	movl	%esi, g_max_erase_count
	movzwl	c_mlc_erase_count_value, %esi
	imull	%esi, %edx
	movzwl	c_ftl_nand_data_blks_per_plane, %esi
	addl	%edx, %eax
	xorl	%edx, %edx
	divl	%esi
	movl	%eax, g_totle_avg_erase_count
	movl	gp_ect_tbl_info, %eax
	addl	$1, 16(%eax)
	testw	%cx, %cx
	je	.L1818
	subl	$1, %ecx
	movl	req_erase, %eax
	xorl	%edx, %edx
	movzwl	%cx, %ecx
	leal	9(%ecx,%ecx,8), %esi
	sall	$2, %esi
	.p2align 4,,7
	.p2align 3
.L1819:
	andl	$-1024, 4(%eax,%edx)
	addl	$36, %edx
	cmpl	%esi, %edx
	jne	.L1819
	movl	%edi, %edx
	xorl	%edi, %edi
	call	FlashEraseBlocks
	leal	16(%ebx), %edx
	movl	%esi, %eax
	xorl	%ecx, %ecx
	movl	%edx, %esi
	movl	%ebx, %edx
	movl	%eax, %ebx
	jmp	.L1821
	.p2align 4,,7
	.p2align 3
.L1820:
	addl	$36, %edi
	addl	$2, %esi
	cmpl	%edi, %ebx
	je	.L1835
.L1821:
	movl	req_erase, %eax
	addl	%edi, %eax
	cmpl	$-1, (%eax)
	jne	.L1820
	movl	4(%eax), %eax
	addl	$1, %ecx
	addl	$36, %edi
	movl	%edx, -32(%ebp)
	movl	%ecx, -28(%ebp)
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlBbmMapBadBlock
	movw	$-1, (%esi)
	movl	-32(%ebp), %edx
	addl	$2, %esi
	subb	$1, 7(%edx)
	cmpl	%edi, %ebx
	movl	-28(%ebp), %ecx
	jne	.L1821
	.p2align 4,,7
	.p2align 3
.L1835:
	testl	%ecx, %ecx
	movl	%edx, %ebx
	je	.L1822
	movl	-20(%ebp), %eax
	call	update_multiplier_value
	call	FtlBbmTblFlush
.L1822:
	movzbl	7(%ebx), %eax
	testb	%al, %al
	jne	.L1823
.L1837:
	movl	p_valid_page_count_table, %eax
	movl	-16(%ebp), %edi
	movw	$-1, (%eax,%edi)
	movl	-20(%ebp), %eax
	call	INSERT_DATA_LIST
	cmpl	$g_gc_temp_superblock, %ebx
	jne	.L1801
.L1833:
	movzwl	g_num_free_superblocks, %esi
	movl	g_in_swl_replace, %ecx
	movl	g_inkDie_check_enable, %edx
	movl	%esi, %eax
	movl	%esi, %edi
	imull	%ecx, %eax
	shrw	%di
	shrl	$2, %eax
	addl	%edi, %eax
	testl	%edx, %edx
	je	.L1802
	movl	g_min_erase_count, %edx
	cmpl	$29, %edx
	movl	%edx, -16(%ebp)
	ja	.L1802
	xorl	%edx, %edx
	cmpl	$2, -16(%ebp)
	jbe	.L1803
	andl	$1, %esi
	movl	%edi, %eax
	je	.L1802
	testl	%ecx, %ecx
	je	.L1803
	.p2align 4,,4
	jmp	.L1802
	.p2align 4,,7
	.p2align 3
.L1813:
	addl	$1, %edx
	movw	%dx, (%eax)
	movl	g_totle_slc_erase_count, %eax
	movl	g_totle_mlc_erase_count, %edx
	addl	$1, %eax
	movl	%eax, -40(%ebp)
	movl	%eax, g_totle_slc_erase_count
	jmp	.L1816
	.p2align 4,,7
	.p2align 3
.L1810:
	addl	%eax, %eax
	movl	%eax, -16(%ebp)
	addl	p_erase_count_table, %eax
	cmpl	$g_active_superblock, %ebx
	movzwl	(%eax), %edx
	jne	.L1811
	cmpw	$30, %dx
	jbe	.L1811
	movb	$0, g_active_superblock+8
	movzwl	(%eax), %edx
	jmp	.L1811
	.p2align 4,,7
	.p2align 3
.L1804:
	movzwl	-22(%ebp), %eax
	movl	p_valid_page_count_table, %edx
	movw	$-1, (%edx,%eax,2)
	call	INSERT_DATA_LIST
	jmp	.L1832
	.p2align 4,,7
	.p2align 3
.L1834:
	movzwl	g_num_free_superblocks, %edx
	movl	g_inkDie_check_enable, %edi
	movl	%edx, %eax
	shrw	$3, %ax
	testl	%edi, %edi
	je	.L1802
	cmpl	$1, g_min_erase_count
	jbe	.L1836
.L1802:
	testw	%ax, %ax
	je	.L1828
	subl	$1, %eax
	movzwl	%ax, %edx
	jmp	.L1803
	.p2align 4,,7
	.p2align 3
.L1814:
	movw	$2, (%eax)
	jmp	.L1815
.L1818:
	movl	req_erase, %eax
	movl	%edi, %edx
	call	FlashEraseBlocks
	movzbl	7(%ebx), %eax
	testb	%al, %al
	je	.L1837
.L1823:
	movzwl	-22(%ebp), %esi
	movzbl	%al, %eax
	movw	$0, 2(%ebx)
	imulw	c_ftl_nand_page_pre_blk, %ax
	movb	$0, 6(%ebx)
	movw	%si, (%ebx)
	movl	g_GlobalSysVersion, %edx
	movw	%ax, 4(%ebx)
	movl	%edx, 12(%ebx)
	addl	$1, %edx
	movl	%edx, g_GlobalSysVersion
	movl	p_valid_page_count_table, %edx
	movzwl	(%ebx), %ecx
	movw	%ax, (%edx,%ecx,2)
	addl	$28, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1836:
	movzwl	%dx, %edx
	leal	0(,%edx,8), %eax
	subl	%edx, %eax
	shrl	$3, %eax
	jmp	.L1802
.L1828:
	xorl	%edx, %edx
	jmp	.L1803
	.size	allocate_data_superblock, .-allocate_data_superblock
	.p2align 4,,15
	.globl	FtlWriteDumpData
	.type	FtlWriteDumpData, @function
FtlWriteDumpData:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$60, %esp
	movzwl	g_active_superblock+4, %eax
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	testw	%ax, %ax
	je	.L1839
	cmpb	$0, g_active_superblock+8
	je	.L1847
.L1839:
	movb	$0, g_active_superblock+10
.L1838:
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1847:
	movzbl	g_active_superblock+7, %edx
	movzwl	%ax, %eax
	movzwl	c_ftl_nand_page_pre_blk, %ecx
	imull	%ecx, %edx
	cmpl	%edx, %eax
	je	.L1839
	cmpb	$0, g_active_superblock+10
	movl	g_MaxLpn, %edi
	movzwl	c_ftl_nand_planes_num, %esi
	jne	.L1838
	subl	$1, %edi
	xorl	%ecx, %ecx
	leal	-16(%ebp), %edx
	movl	%edi, %eax
	call	log2phys
	movl	-16(%ebp), %edx
	movl	p_sys_spare_buf, %ebx
	movl	p_sys_data_buf, %eax
	movl	%edi, -36(%ebp)
	cmpl	$-1, %edx
	movl	%edx, -48(%ebp)
	movl	%eax, -44(%ebp)
	movl	%ebx, -40(%ebp)
	movl	$0, 4(%ebx)
	je	.L1841
	leal	-52(%ebp), %eax
	xorl	%ecx, %ecx
	movl	$1, %edx
	movl	%eax, -56(%ebp)
	call	FlashReadPages
.L1842:
	movzwl	%si, %esi
	leal	(%esi,%esi,2), %esi
	addl	%esi, %esi
	movw	$-3947, (%ebx)
	je	.L1843
	cmpw	$0, g_active_superblock+4
	je	.L1843
	leal	-52(%ebp), %eax
	movl	%eax, -56(%ebp)
	jmp	.L1844
	.p2align 4,,7
	.p2align 3
.L1846:
	subl	$1, %esi
	cmpw	$0, g_active_superblock+4
	je	.L1843
.L1844:
	movl	-48(%ebp), %eax
	movl	%edi, 8(%ebx)
	movl	%eax, 12(%ebx)
	movzwl	g_active_superblock, %eax
	movw	%ax, 2(%ebx)
	movl	$g_active_superblock, %eax
	call	get_new_active_ppa
	movl	$0, %edx
	movl	%eax, -48(%ebp)
	movl	g_GlobalDataVersion, %eax
	movl	%eax, 4(%ebx)
	addl	$1, %eax
	cmpl	$-1, %eax
	cmove	%edx, %eax
	xorl	%ecx, %ecx
	movl	%eax, g_GlobalDataVersion
	movl	-56(%ebp), %eax
	movb	$1, %dl
	movl	$0, (%esp)
	call	FlashProgPages
	movzwl	g_active_superblock, %eax
	call	decrement_vpc_count
	cmpl	$1, %esi
	jne	.L1846
.L1843:
	movb	$1, g_active_superblock+10
	jmp	.L1838
	.p2align 4,,7
	.p2align 3
.L1841:
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movl	$255, %edx
	call	memset
	jmp	.L1842
	.size	FtlWriteDumpData, .-FtlWriteDumpData
	.p2align 4,,15
	.globl	l2p_flush
	.type	l2p_flush, @function
l2p_flush:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	call	FtlWriteDumpData
	cmpw	$0, c_ftl_nand_l2pmap_ram_region_num
	je	.L1849
	xorl	%ebx, %ebx
	jmp	.L1851
	.p2align 4,,7
	.p2align 3
.L1850:
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_l2pmap_ram_region_num
	jbe	.L1849
.L1851:
	movl	p_l2p_ram_map, %ecx
	movzwl	%bx, %eax
	leal	(%eax,%eax,2), %edx
	leal	(%ecx,%edx,4), %edx
	movl	4(%edx), %edx
	testl	%edx, %edx
	jns	.L1850
	call	flush_l2p_region
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_l2pmap_ram_region_num
	ja	.L1851
	.p2align 4,,7
	.p2align 3
.L1849:
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	l2p_flush, .-l2p_flush
	.section	.rodata.str1.1
.LC101:
	.string	"\0013...%s enter...\n"
.LC102:
	.string	"\0013FtlCheckVpc %x = %x  %x\n"
	.text
	.p2align 4,,15
	.globl	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, @function
FtlVpcCheckAndModify:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	subl	$20, %esp
	movl	$__func__.24754, 4(%esp)
	movl	$.LC101, (%esp)
	call	printk
	movzwl	c_ftl_nand_blk_pre_plane, %ecx
	xorl	%edx, %edx
	movl	p_valid_page_count_check_table, %eax
	addl	%ecx, %ecx
	call	memset
	movl	g_MaxLpn, %ebx
	testl	%ebx, %ebx
	je	.L1854
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L1856:
	xorl	%ecx, %ecx
	movl	%ebx, %eax
	leal	-12(%ebp), %edx
	call	log2phys
	movl	-12(%ebp), %eax
	cmpl	$-1, %eax
	je	.L1855
	shrl	$10, %eax
	xorl	%edx, %edx
	movl	p_valid_page_count_check_table, %ecx
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	leal	(%ecx,%eax,2), %eax
	addw	$1, (%eax)
.L1855:
	addl	$1, %ebx
	cmpl	%ebx, g_MaxLpn
	ja	.L1856
.L1854:
	cmpw	$0, c_ftl_nand_data_blks_per_plane
	je	.L1853
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L1859:
	movl	p_valid_page_count_table, %eax
	movzwl	%bx, %esi
	movl	p_valid_page_count_check_table, %edx
	movzwl	(%eax,%esi,2), %eax
	movzwl	(%edx,%esi,2), %edx
	cmpw	%dx, %ax
	je	.L1858
	cmpw	$-1, %ax
	jne	.L1862
.L1858:
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_data_blks_per_plane
	ja	.L1859
.L1853:
	addl	$20, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
.L1862:
	movzwl	%dx, %edx
	movzwl	%ax, %eax
	movl	%edx, 12(%esp)
	movl	%eax, 8(%esp)
	movl	%esi, 4(%esp)
	movl	$.LC102, (%esp)
	call	printk
	cmpw	%bx, g_active_superblock
	je	.L1858
	cmpw	%bx, g_gc_temp_superblock
	je	.L1858
	cmpw	%bx, g_buffer_superblock
	je	.L1858
	movl	p_valid_page_count_check_table, %eax
	movzwl	(%eax,%esi,2), %edx
	movl	p_valid_page_count_table, %eax
	movw	%dx, (%eax,%esi,2)
	movl	%esi, %eax
	call	update_vpc_list
	call	l2p_flush
	call	FtlVpcTblFlush
	.p2align 4,,5
	jmp	.L1858
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.p2align 4,,15
	.globl	allocate_new_data_superblock
	.type	allocate_new_data_superblock, @function
allocate_new_data_superblock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movzwl	(%eax), %esi
	movl	%eax, %ebx
	cmpw	$-1, %si
	je	.L1864
	movl	p_valid_page_count_table, %edx
	movzwl	%si, %eax
	cmpw	$0, (%edx,%eax,2)
	je	.L1865
	call	INSERT_DATA_LIST
.L1864:
	cmpl	$g_buffer_superblock, %ebx
	movb	$0, 8(%ebx)
	je	.L1872
.L1866:
	cmpl	$g_active_superblock, %ebx
	je	.L1873
.L1867:
	movzwl	g_tmp_data_superblock_id, %edx
	cmpw	$-1, %dx
	je	.L1868
	cmpw	%dx, %si
	movzwl	%dx, %eax
	je	.L1874
.L1869:
	call	update_vpc_list
.L1870:
	movw	$-1, g_tmp_data_superblock_id
.L1868:
	movl	%ebx, %eax
	call	allocate_data_superblock
	call	l2p_flush
	call	FtlVpcTblFlush
	xorl	%eax, %eax
	call	FtlEctTblFlush
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1874:
	movl	p_valid_page_count_table, %edx
	cmpw	$0, (%edx,%eax,2)
	je	.L1870
	jmp	.L1869
	.p2align 4,,7
	.p2align 3
.L1865:
	call	INSERT_FREE_LIST
	cmpl	$g_buffer_superblock, %ebx
	movb	$0, 8(%ebx)
	jne	.L1866
.L1872:
	movb	$1, g_buffer_superblock+8
	jmp	.L1867
	.p2align 4,,7
	.p2align 3
.L1873:
	movl	g_inkDie_check_enable, %eax
	testl	%eax, %eax
	je	.L1867
	cmpl	$29, g_min_erase_count
	ja	.L1867
	movb	$1, g_active_superblock+8
	jmp	.L1867
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.section	.rodata.str1.4
	.align 4
.LC103:
	.string	"\0013Ftlwrite decrement_vpc_count %x = %d\n"
	.text
	.p2align 4,,15
	.globl	FtlCacheWriteBack
	.type	FtlCacheWriteBack, @function
FtlCacheWriteBack:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$24, %esp
	movl	g_wr_page_num, %edx
	movl	gp_last_act_superblock, %ebx
	testl	%edx, %edx
	jne	.L1892
	addl	$24, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1892:
	movzbl	9(%ebx), %eax
	xorl	%ecx, %ecx
	movl	%eax, (%esp)
	movl	req_wr_io, %eax
	call	FlashProgPages
	movl	g_wr_page_num, %eax
	testl	%eax, %eax
	je	.L1878
	xorl	%edi, %edi
	xorl	%esi, %esi
	movl	$0, -20(%ebp)
	.p2align 4,,7
	.p2align 3
.L1877:
	movl	req_wr_io, %eax
	addl	%edi, %eax
	cmpl	$-1, (%eax)
	je	.L1890
	jmp	.L1880
	.p2align 4,,7
	.p2align 3
.L1881:
	cmpw	$0, 4(%ebx)
	je	.L1882
.L1883:
	movl	req_wr_io, %eax
	addl	$1, g_sys_ext_data+96
	movl	4(%eax,%edi), %eax
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlGcMarkBadPhyBlk
	movl	%ebx, %eax
	call	get_new_active_ppa
	movl	req_wr_io, %esi
	xorl	%ecx, %ecx
	movl	$1, %edx
	addl	%edi, %esi
	movl	%eax, 4(%esi)
	movl	%eax, -16(%ebp)
	movzbl	9(%ebx), %eax
	movl	%eax, (%esp)
	movl	%esi, %eax
	call	FlashProgPages
	movl	req_wr_io, %eax
	addl	%edi, %eax
	cmpl	$-1, (%eax)
	jne	.L1893
.L1890:
	movl	4(%eax), %eax
	xorl	%edx, %edx
	movzwl	(%ebx), %ecx
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	cmpw	%ax, %cx
	jne	.L1881
	movl	p_valid_page_count_table, %eax
	movzwl	%cx, %ecx
	leal	(%eax,%ecx,2), %ecx
	movzwl	4(%ebx), %eax
	subw	%ax, (%ecx)
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movb	$0, 6(%ebx)
	movw	$0, 4(%ebx)
	movw	%ax, 2(%ebx)
.L1882:
	movl	%ebx, %eax
	call	allocate_new_data_superblock
	jmp	.L1883
	.p2align 4,,7
	.p2align 3
.L1893:
	movl	$1, %esi
.L1880:
	movl	4(%eax), %edx
	movl	$1, %ecx
	movl	16(%eax), %eax
	movl	%edx, -16(%ebp)
	leal	-16(%ebp), %edx
	call	log2phys
	movl	req_wr_io, %eax
	movl	12(%eax,%edi), %eax
	movl	12(%eax), %eax
	cmpl	$-1, %eax
	je	.L1885
	shrl	$10, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %edx
	movl	p_valid_page_count_table, %eax
	cmpw	$0, (%eax,%edx,2)
	je	.L1894
.L1887:
	movl	%edx, %eax
	call	decrement_vpc_count
.L1885:
	addl	$1, -20(%ebp)
	movl	-20(%ebp), %eax
	cmpl	%eax, g_wr_page_num
	jbe	.L1878
	testl	%esi, %esi
	je	.L1879
	movl	req_wr_io, %eax
	movl	$-1, 36(%eax,%edi)
.L1879:
	addl	$36, %edi
	jmp	.L1877
.L1878:
	movl	$0, g_wr_page_num
	addl	$24, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L1894:
	movl	%edx, 4(%esp)
	movl	%edx, -24(%ebp)
	movl	$0, 8(%esp)
	movl	$.LC103, (%esp)
	call	printk
	movl	-24(%ebp), %edx
	jmp	.L1887
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.p2align 4,,15
	.globl	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, @function
FtlGcFreeTempBlock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	movzwl	g_gc_temp_superblock, %ebx
	movzwl	c_ftl_nand_page_pre_blk, %edx
	cmpw	$-1, %bx
	je	.L1896
	testl	%eax, %eax
	je	.L1897
	cmpw	$-1, ftl_gc_temp_block_bops_scan_page_addr
	je	.L1915
.L1898:
	cmpw	$18, g_num_free_superblocks
	movl	$2, %eax
	cmovae	%eax, %edx
.L1897:
	movl	$g_gc_temp_superblock, %eax
	call	FtlGcScanTempBlk
	cmpl	$-1, %eax
	movl	%eax, -16(%ebp)
	je	.L1899
	movl	p_erase_count_table, %edx
	movzwl	%bx, %ebx
	addl	$1, g_sys_ext_data+96
	leal	(%edx,%ebx,2), %ebx
	movzwl	(%ebx), %eax
	cmpw	$4, %ax
	ja	.L1916
	movl	ftl_gc_temp_power_lost_recovery_flag, %eax
	testl	%eax, %eax
	je	.L1917
.L1901:
	movl	$0, ftl_gc_temp_power_lost_recovery_flag
	movl	$1, %eax
.L1902:
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1917:
	movl	-16(%ebp), %eax
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlBbmMapBadBlock
	call	FtlBbmTblFlush
	jmp	.L1901
	.p2align 4,,7
	.p2align 3
.L1896:
	movl	$0, ftl_gc_temp_power_lost_recovery_flag
	addl	$4, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1899:
	cmpw	$-1, ftl_gc_temp_block_bops_scan_page_addr
	movl	$1, %eax
	jne	.L1902
	cmpw	$-1, g_gc_temp_superblock
	movl	$0, ftl_gc_temp_power_lost_recovery_flag
	je	.L1912
	call	FtlCacheWriteBack
	movzbl	g_gc_temp_superblock+7, %eax
	imulw	c_ftl_nand_page_pre_blk, %ax
	movl	p_valid_page_count_table, %edx
	movzwl	g_gc_temp_superblock, %ecx
	movw	%ax, (%edx,%ecx,2)
	movzwl	g_gc_page_offset, %eax
	movzwl	%ax, %edx
	addl	%edx, g_totle_gc_page_count
	testw	%ax, %ax
	je	.L1903
	xorl	%esi, %esi
	jmp	.L1906
	.p2align 4,,7
	.p2align 3
.L1904:
	cmpl	4(%ebx), %ecx
	je	.L1905
	movzwl	g_gc_temp_superblock, %eax
	call	decrement_vpc_count
.L1905:
	addl	$1, %esi
	cmpw	%si, g_gc_page_offset
	jbe	.L1903
.L1906:
	movzwl	%si, %eax
	xorl	%ecx, %ecx
	leal	(%eax,%eax,2), %ebx
	movl	p_gc_page_info, %eax
	leal	-16(%ebp), %edx
	leal	(%eax,%ebx,4), %ebx
	movl	8(%ebx), %eax
	call	log2phys
	movl	(%ebx), %eax
	movl	-16(%ebp), %ecx
	cmpl	%ecx, %eax
	jne	.L1904
	shrl	$10, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_blks_per_die
	movl	$1, %ecx
	addl	$1, %esi
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	leal	4(%ebx), %edx
	movl	%eax, %edi
	movl	8(%ebx), %eax
	call	log2phys
	movzwl	%di, %eax
	call	decrement_vpc_count
	cmpw	%si, g_gc_page_offset
	ja	.L1906
	.p2align 4,,7
	.p2align 3
.L1903:
	movl	$65535, %eax
	call	decrement_vpc_count
	movzwl	g_gc_temp_superblock, %eax
	movl	p_valid_page_count_table, %edx
	cmpw	$0, (%edx,%eax,2)
	je	.L1907
	call	INSERT_DATA_LIST
.L1908:
	movw	$-1, g_gc_temp_superblock
	movw	$0, g_gc_page_offset
	movw	$0, g_gc_blk_num
	call	l2p_flush
	call	FtlVpcTblFlush
	movl	g_inkDie_check_enable, %eax
	testl	%eax, %eax
	je	.L1909
	cmpl	$29, g_min_erase_count
	ja	.L1909
	movzwl	c_ftl_nand_data_op_blks_per_plane, %eax
	cmpw	%ax, g_num_free_superblocks
	jae	.L1910
	addl	%eax, %eax
	movw	%ax, g_gc_free_blk_threshold
.L1910:
	movw	$-1, g_gc_superblock
	xorl	%eax, %eax
	jmp	.L1902
	.p2align 4,,7
	.p2align 3
.L1916:
	subl	$5, %eax
	movw	%ax, (%ebx)
	movl	$1, %eax
	call	FtlEctTblFlush
	movl	ftl_gc_temp_power_lost_recovery_flag, %eax
	testl	%eax, %eax
	jne	.L1901
	jmp	.L1917
	.p2align 4,,7
	.p2align 3
.L1909:
	movzwl	c_ftl_nand_data_op_blks_per_plane, %eax
	movzwl	g_num_free_superblocks, %ecx
	leal	(%eax,%eax,2), %edx
	xorl	%eax, %eax
	sarl	$2, %edx
	cmpl	%edx, %ecx
	jle	.L1902
	movw	$-1, g_gc_superblock
	movw	$20, g_gc_free_blk_threshold
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1907:
	call	INSERT_FREE_LIST
	jmp	.L1908
	.p2align 4,,7
	.p2align 3
.L1915:
	movw	$0, ftl_gc_temp_block_bops_scan_page_addr
	jmp	.L1898
.L1912:
	xorl	%eax, %eax
	jmp	.L1902
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.p2align 4,,15
	.globl	FtlGcPageRecovery
	.type	FtlGcPageRecovery, @function
FtlGcPageRecovery:
	movzwl	c_ftl_nand_page_pre_blk, %edx
	movl	$g_gc_temp_superblock, %eax
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlGcScanTempBlk
	movzwl	c_ftl_nand_page_pre_blk, %eax
	cmpw	%ax, g_gc_temp_superblock+2
	jae	.L1920
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1920:
	movl	$gL2pMapInfo, %eax
	call	FtlMapBlkWriteDumpData
	xorl	%eax, %eax
	call	FtlGcFreeTempBlock
	movl	$0, ftl_gc_temp_power_lost_recovery_flag
	popl	%ebp
	ret
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.p2align 4,,15
	.globl	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, @function
FtlPowerLostRecovery:
	pushl	%ebp
	movl	$g_active_superblock, %eax
	movl	%esp, %ebp
	movw	$0, g_power_lost_recovery_flag
	movl	$0, g_recovery_page_num
	call	FtlRecoverySuperblock
	movl	$g_active_superblock, %eax
	call	FtlSlcSuperblockCheck
	movl	$g_buffer_superblock, %eax
	call	FtlRecoverySuperblock
	movl	$g_buffer_superblock, %eax
	call	FtlSlcSuperblockCheck
	call	FtlGcPageRecovery
	movl	$65535, %eax
	call	decrement_vpc_count
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.p2align 4,,15
	.globl	FtlSysBlkInit
	.type	FtlSysBlkInit, @function
FtlSysBlkInit:
	movzwl	c_ftl_nand_max_sys_blks, %eax
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	$-1, %ebx
	call	FtlFreeSysBlkQueueInit
	call	FtlScanSysBlk
	cmpw	$-1, gSysInfo
	je	.L1923
	call	FtlLoadSysInfo
	testl	%eax, %eax
	jne	.L1923
	.p2align 4,,6
	call	FtlLoadMapInfo
	.p2align 4,,6
	call	FtlLoadVonderInfo
	.p2align 4,,5
	call	Ftl_load_ext_data
	.p2align 4,,5
	call	FtlLoadEctTbl
	.p2align 4,,5
	call	FtlFreeSysBLkSort
	.p2align 4,,5
	call	SupperBlkListInit
	.p2align 4,,5
	call	FtlPowerLostRecovery
	movzwl	c_ftl_nand_l2pmap_ram_region_num, %ebx
	testl	%ebx, %ebx
	je	.L1924
	movl	p_l2p_ram_map, %edx
	movl	4(%edx), %eax
	testl	%eax, %eax
	js	.L1925
	xorl	%eax, %eax
	jmp	.L1926
	.p2align 4,,7
	.p2align 3
.L1927:
	movl	16(%edx), %ecx
	addl	$12, %edx
	testl	%ecx, %ecx
	js	.L1925
.L1926:
	addl	$1, %eax
	cmpl	%ebx, %eax
	jl	.L1927
.L1924:
	addw	$1, g_sys_save_data+28
	cmpw	$0, g_power_lost_recovery_flag
	je	.L1930
.L1929:
	movl	p_valid_page_count_table, %ecx
	movzwl	g_active_superblock, %eax
	movzwl	g_active_superblock+4, %edx
	leal	(%ecx,%eax,2), %eax
	subw	%dx, (%eax)
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movl	p_valid_page_count_table, %ebx
	movzwl	g_buffer_superblock+4, %edx
	movb	$0, g_active_superblock+6
	movw	%ax, g_active_superblock+2
	movzwl	g_buffer_superblock, %eax
	movw	$0, g_active_superblock+4
	leal	(%ebx,%eax,2), %eax
	subw	%dx, (%eax)
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movb	$0, g_buffer_superblock+6
	movw	$0, g_buffer_superblock+4
	movw	%ax, g_buffer_superblock+2
	movl	$gL2pMapInfo, %eax
	call	FtlMapBlkWriteDumpData
	movl	$gVendorBlkInfo, %eax
	call	FtlMapBlkWriteDumpData
	addw	$1, g_sys_save_data+30
	call	l2p_flush
	call	FtlVpcTblFlush
.L1930:
	movl	$1, %eax
	call	FtlUpdateVaildLpn
	movzwl	g_active_superblock, %eax
	cmpw	$-1, %ax
	je	.L1928
	cmpw	$0, g_active_superblock+4
	jne	.L1928
	cmpw	$0, g_buffer_superblock+4
	je	.L1934
.L1928:
	xorl	%ebx, %ebx
	testb	$31, g_sys_save_data+28
	jne	.L1923
	call	FtlVpcCheckAndModify
.L1923:
	movl	%ebx, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1925:
	addw	$1, g_sys_save_data+28
	jmp	.L1929
	.p2align 4,,7
	.p2align 3
.L1934:
	movzwl	%ax, %eax
	call	FtlGcRefreshBlock
	movzwl	g_buffer_superblock, %eax
	call	FtlGcRefreshBlock
	movl	$g_active_superblock, %eax
	call	allocate_new_data_superblock
	movl	$g_buffer_superblock, %eax
	call	allocate_new_data_superblock
	jmp	.L1928
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.p2align 4,,15
	.globl	FtlInit
	.type	FtlInit, @function
FtlInit:
.L1936:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	subl	$8, %esp
	movl	$.LC75, 4(%esp)
	movl	$-1, gFtlInitStatus
	movl	$0, g_LowFormat
	movl	$.LC88, (%esp)
	call	printk
	movl	%ebx, %eax
	call	FtlConstantsInit
	call	FtlMemInit
	call	FtlVariablesInit
	movzwl	c_ftl_nand_max_sys_blks, %eax
	call	FtlFreeSysBlkQueueInit
	call	FtlLoadBbt
	testl	%eax, %eax
	je	.L1938
.L1937:
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1938:
	call	FtlSysBlkInit
	testl	%eax, %eax
	jne	.L1937
	movl	$1, gFtlInitStatus
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	FtlInit, .-FtlInit
	.p2align 4,,15
	.globl	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, @function
Ftl_get_new_temp_ppa:
	cmpw	$-1, g_gc_temp_superblock
	pushl	%ebp
	movl	%esp, %ebp
	je	.L1940
	cmpw	$0, g_gc_temp_superblock+4
	jne	.L1941
.L1940:
	call	FtlCacheWriteBack
	xorl	%eax, %eax
	call	FtlGcFreeTempBlock
	movl	$g_gc_temp_superblock, %eax
	movb	$0, g_gc_temp_superblock+8
	call	allocate_data_superblock
	movw	$0, g_gc_blk_num
	movw	$0, g_gc_page_offset
	call	l2p_flush
	call	FtlVpcTblFlush
	xorl	%eax, %eax
	call	FtlEctTblFlush
.L1941:
	movl	$g_gc_temp_superblock, %eax
	call	get_new_active_ppa
	popl	%ebp
	ret
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.p2align 4,,15
	.globl	FtlDiscard
	.type	FtlDiscard, @function
FtlDiscard:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%eax, %edi
	pushl	%esi
	movl	$-1, %esi
	pushl	%ebx
	movl	%edx, %ebx
	subl	$8, %esp
	leal	(%edx,%eax), %eax
	cmpl	g_MaxLbaSector, %eax
	ja	.L1943
	xorl	%esi, %esi
	cmpl	$31, %edx
	ja	.L1953
.L1943:
	addl	$8, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1953:
	call	FtlCacheWriteBack
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%edi, %esi
	shrl	%cl, %esi
	movl	%esi, %eax
	sall	%cl, %eax
	subw	%ax, %di
	jne	.L1944
	movzwl	c_ftl_nand_sec_pre_page, %eax
.L1945:
	cmpl	%eax, %ebx
	movl	$-1, -16(%ebp)
	jb	.L1946
	.p2align 4,,7
	.p2align 3
.L1948:
	xorl	%ecx, %ecx
	movl	%esi, %eax
	leal	-20(%ebp), %edx
	call	log2phys
	cmpl	$-1, -20(%ebp)
	je	.L1947
	leal	-16(%ebp), %edx
	movl	$1, %ecx
	movl	%esi, %eax
	addl	$1, gc_discard_updated
	addl	$1, g_totle_discard_page_count
	call	log2phys
	movl	-20(%ebp), %eax
	xorl	%edx, %edx
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	call	decrement_vpc_count
.L1947:
	movzwl	c_ftl_nand_sec_pre_page, %eax
	addl	$1, %esi
	subl	%eax, %ebx
	cmpl	%ebx, %eax
	jbe	.L1948
.L1946:
	xorl	%esi, %esi
	cmpl	$32, gc_discard_updated
	jbe	.L1943
	movl	$0, gc_discard_updated
	call	l2p_flush
	call	FtlVpcTblFlush
	addl	$8, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1944:
	movzwl	c_ftl_nand_sec_pre_page, %eax
	movzwl	%di, %edi
	addl	$1, %esi
	movl	%eax, %edx
	subl	%edi, %edx
	cmpl	%ebx, %edx
	cmova	%ebx, %edx
	movzwl	%dx, %edx
	subl	%edx, %ebx
	jmp	.L1945
	.size	FtlDiscard, .-FtlDiscard
	.p2align 4,,15
	.globl	FtlSysFlush
	.type	FtlSysFlush, @function
FtlSysFlush:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlCacheWriteBack
	call	l2p_flush
	call	FtlVpcTblFlush
	movl	$1, %eax
	call	FtlEctTblFlush
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	FtlSysFlush, .-FtlSysFlush
	.p2align 4,,15
	.globl	FtlDeInit
	.type	FtlDeInit, @function
FtlDeInit:
	cmpl	$1, gFtlInitStatus
	pushl	%ebp
	movl	%esp, %ebp
	je	.L1957
	xorl	%eax, %eax
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1957:
	call	FtlSysFlush
	xorl	%eax, %eax
	popl	%ebp
	.p2align 4,,3
	ret
	.size	FtlDeInit, .-FtlDeInit
	.p2align 4,,15
	.globl	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, @function
Ftl_gc_temp_data_write_back:
	pushl	%ebp
	xorl	%ecx, %ecx
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	subl	$4, %esp
	movl	g_gc_num_req, %edx
	movl	req_gc_dst, %eax
	movl	$0, (%esp)
	call	FlashProgPages
	movl	g_gc_num_req, %edx
	testl	%edx, %edx
	je	.L1959
	movl	req_gc_dst, %eax
	xorl	%ebx, %ebx
	cmpl	$-1, (%eax)
	jne	.L1961
	jmp	.L1969
	.p2align 4,,7
	.p2align 3
.L1963:
	leal	(%eax,%eax,8), %edx
	movl	req_gc_dst, %eax
	sall	$2, %edx
	addl	%edx, %eax
	cmpl	$-1, (%eax)
	je	.L1960
.L1961:
	movl	12(%eax), %esi
	addl	$1, %ebx
	movl	4(%eax), %edx
	movl	8(%esi), %ecx
	movl	12(%esi), %eax
	call	FtlGcUpdatePage
	movl	g_gc_num_req, %edx
	movzwl	%bx, %eax
	cmpl	%edx, %eax
	jb	.L1963
.L1959:
	movl	req_gc_dst, %eax
	call	FtlGcBufFree
	xorl	%eax, %eax
	cmpw	$0, g_gc_temp_superblock+4
	movl	$0, g_gc_num_req
	je	.L1968
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
.L1969:
	xorl	%edx, %edx
	.p2align 4,,7
	.p2align 3
.L1960:
	movzwl	g_gc_temp_superblock, %ecx
	movl	p_valid_page_count_table, %eax
	movw	$0, (%eax,%ecx,2)
	movl	req_gc_dst, %eax
	addl	$1, g_sys_ext_data+96
	movw	$-1, g_gc_temp_superblock
	movl	4(%eax,%edx), %eax
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlBbmMapBadBlock
	call	FtlBbmTblFlush
	call	FtlGcPageVarInit
	addl	$4, %esp
	movl	$1, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L1968:
	movb	$1, %al
	call	FtlGcFreeTempBlock
	addl	$4, %esp
	movl	$1, %eax
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.section	.rodata.str1.4
	.align 4
.LC104:
	.string	"\0013%d GC datablk  = %x vpc %x %x\n"
	.text
	.p2align 4,,15
	.globl	FtlGarbageCollect
	.type	FtlGarbageCollect, @function
FtlGarbageCollect:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%edx, %esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$60, %esp
	movl	g_in_gc_progress, %edx
	xorl	%eax, %eax
	testl	%edx, %edx
	jne	.L1971
	cmpw	$-1, ftl_gc_temp_block_bops_scan_page_addr
	je	.L1972
	cmpw	$-1, g_gc_temp_superblock
	je	.L1972
	movb	$1, %al
	call	FtlGcFreeTempBlock
	movl	%eax, %edx
	movl	$1, %eax
	testl	%edx, %edx
	jne	.L1971
.L1972:
	movzwl	g_gc_next_blk_1, %eax
	cmpw	$-1, %ax
	je	.L1973
	cmpw	$-1, g_gc_next_blk
	je	.L2097
.L1973:
	movl	g_gc_skip_write_count, %edx
	movl	%ebx, %eax
	sall	$7, %eax
	cmpl	$1, %ebx
	movw	$-1, -42(%ebp)
	leal	1(%edx,%eax), %eax
	movl	%eax, g_gc_skip_write_count
	je	.L2098
.L1974:
	cmpw	$-1, g_gc_superblock
	je	.L1976
.L2092:
	cmpw	$-1, -42(%ebp)
	sete	%cl
.L1975:
	testb	%cl, %cl
	jne	.L2079
.L1996:
	cmpw	$-1, g_gc_temp_superblock
	movzwl	g_gc_superblock, %ebx
	je	.L2099
.L2013:
	cmpl	$1, g_inkDie_check_enable
	sbbl	%edx, %edx
	addl	$2, %edx
	cmpw	$-1, %bx
	movw	%dx, -40(%ebp)
	je	.L2100
.L2019:
	cmpw	g_active_superblock, %bx
	je	.L2026
	cmpw	g_buffer_superblock, %bx
	je	.L2026
.L2087:
	cmpw	$-1, %bx
	jne	.L2029
	movzwl	g_gc_blk_index, %ebx
	movl	$0, g_in_swl_replace
	.p2align 4,,7
	.p2align 3
.L2086:
	movzwl	%bx, %eax
	call	List_get_gc_head_node
	cmpw	$-1, %ax
	movl	%eax, %ecx
	movw	%ax, g_gc_superblock
	je	.L2031
	movzwl	g_gc_blk_num, %edx
	testw	%dx, %dx
	je	.L2032
	movl	p_gc_blk_tbl, %esi
	cmpw	(%esi), %ax
	je	.L2094
	leal	-1(%edx), %edi
	xorl	%eax, %eax
	movzwl	%di, %edi
	addl	%edi, %edi
	jmp	.L2034
	.p2align 4,,7
	.p2align 3
.L2035:
	addl	$2, %eax
	cmpw	(%esi,%eax), %cx
	je	.L2094
.L2034:
	cmpl	%edi, %eax
	jne	.L2035
.L2032:
	movzwl	g_gc_blk_index, %eax
	movzwl	%cx, %ecx
	movzwl	c_ftl_nand_planes_num, %esi
	addl	$1, %eax
	movw	%ax, g_gc_blk_index
	movw	%ax, -20(%ebp)
	movl	p_valid_page_count_table, %eax
	leal	(%eax,%ecx,2), %ecx
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movzwl	(%ecx), %ebx
	imull	%esi, %eax
	movzwl	%bx, %edi
	movl	%eax, %esi
	shrl	$31, %esi
	addl	%eax, %esi
	sarl	%esi
	cmpl	%esi, %edi
	jg	.L2056
	cmpw	$48, -20(%ebp)
	jbe	.L2037
	cmpw	$8, %bx
	jbe	.L2037
	cmpw	$35, %dx
	jbe	.L2056
.L2037:
	cmpl	%edi, %eax
	jle	.L2101
.L2038:
	testw	%bx, %bx
	.p2align 4,,2
	jne	.L2039
	movl	$65535, %eax
	call	decrement_vpc_count
	movzwl	g_gc_blk_index, %ebx
.L2094:
	addl	$1, %ebx
	movw	%bx, g_gc_blk_index
	jmp	.L2086
.L2099:
	testb	%cl, %cl
	je	.L2078
	cmpw	$-1, g_gc_next_blk
	jne	.L2078
	cmpw	$-1, %bx
	movw	$-1, -42(%ebp)
	jne	.L2013
	movzwl	g_num_free_superblocks, %edi
	cmpw	g_gc_free_blk_threshold, %di
	movl	$0, g_in_swl_replace
	jbe	.L2014
	cmpw	$0, gc_ink_free_return_value
	jne	.L2015
	movl	g_MaxLpn, %eax
	leal	(%eax,%eax,2), %eax
	shrl	$2, %eax
	cmpl	g_validLpn, %eax
	jbe	.L2016
.L2015:
	movzwl	c_ftl_nand_data_op_blks_per_plane, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	movw	%ax, g_gc_free_blk_threshold
.L2096:
	call	FtlReadRefresh
	movzwl	gc_ink_free_return_value, %eax
.L1971:
	addl	$60, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2079:
	testl	%ebx, %ebx
	jne	.L1996
	movzwl	g_num_free_superblocks, %eax
	movw	$1, -40(%ebp)
	cmpw	$24, %ax
	ja	.L1997
	cmpw	$16, %ax
	movzwl	c_ftl_nand_page_pre_blk, %ebx
	jbe	.L1998
	shrw	$5, %bx
	movw	%bx, -40(%ebp)
.L1997:
	cmpw	g_gc_free_blk_threshold, %ax
	jbe	.L2001
	cmpw	$-1, g_gc_temp_superblock
	je	.L2102
.L2002:
	movzwl	c_ftl_nand_data_op_blks_per_plane, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	movw	%ax, g_gc_free_blk_threshold
.L2001:
	cmpl	$2, %esi
	ja	.L2093
	movl	g_inkDie_check_enable, %ecx
	testl	%ecx, %ecx
	je	.L2093
	addw	$1, -40(%ebp)
.L2093:
	movzwl	g_gc_superblock, %ebx
	movw	$-1, -42(%ebp)
	cmpw	$-1, %bx
	jne	.L2019
.L2100:
	cmpw	$-1, -42(%ebp)
	je	.L2020
	movzwl	-42(%ebp), %eax
	movw	%ax, g_gc_superblock
	movl	%eax, %edx
.L2021:
	movzwl	g_gc_blk_num, %eax
	testw	%ax, %ax
	je	.L2022
	movl	p_gc_blk_tbl, %esi
	cmpw	%dx, (%esi)
	je	.L2023
	subl	$1, %eax
	movzwl	%ax, %ecx
	xorl	%eax, %eax
	addl	%ecx, %ecx
	jmp	.L2024
.L2025:
	addl	$2, %eax
	cmpw	%dx, (%esi,%eax)
	je	.L2023
.L2024:
	cmpl	%ecx, %eax
	jne	.L2025
.L2022:
	cmpw	$-1, %dx
	je	.L2019
	movl	$g_gc_superblock, %eax
	call	make_superblock
	movzwl	g_gc_superblock, %ebx
	movl	p_valid_page_count_table, %eax
	movw	$0, g_gc_superblock+2
	movb	$0, g_gc_superblock+6
	movzwl	%bx, %edx
	movw	$0, g_gc_cur_blk_valid_pages
	movzwl	(%eax,%edx,2), %eax
	movw	%ax, g_gc_cur_blk_max_valid_pages
	jmp	.L2019
	.p2align 4,,7
	.p2align 3
.L2056:
	movw	$0, g_gc_blk_index
	movzwl	(%ecx), %ebx
	movzwl	%bx, %edi
	cmpl	%edi, %eax
	jg	.L2038
.L2101:
	cmpw	$-1, -42(%ebp)
	jne	.L2038
	movzwl	gc_ink_free_return_value, %eax
	movw	$-1, g_gc_superblock
	movw	$0, g_gc_blk_index
	addl	$60, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2039:
	movl	$g_gc_superblock, %eax
	movb	$0, g_gc_superblock+8
	call	make_superblock
	movzwl	g_gc_superblock, %edx
	movl	p_valid_page_count_table, %eax
	movw	$0, g_gc_cur_blk_valid_pages
	movzwl	(%eax,%edx,2), %eax
	movw	$0, g_gc_superblock+2
	movb	$0, g_gc_superblock+6
	movw	%ax, g_gc_cur_blk_max_valid_pages
.L2029:
	call	FtlReadRefresh
	movzwl	g_gc_superblock+2, %edx
	movzwl	-40(%ebp), %ecx
	movzwl	c_ftl_nand_page_pre_blk, %eax
	movl	$1, g_in_gc_progress
	movzwl	%dx, %esi
	addl	%ecx, %esi
	movzwl	%ax, %ebx
	movl	%eax, %ecx
	subw	%dx, %cx
	cmpl	%ebx, %esi
	movzwl	-40(%ebp), %ebx
	movl	$0, -24(%ebp)
	cmovg	%ecx, %ebx
	testw	%bx, %bx
	movw	%bx, -40(%ebp)
	je	.L2042
	.p2align 4,,7
	.p2align 3
.L2080:
	movzwl	c_ftl_nand_planes_num, %esi
	testw	%si, %si
	je	.L2049
	movzwl	g_gc_superblock+2, %eax
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	addl	-24(%ebp), %eax
	movl	req_gc, %edi
	movl	%eax, -20(%ebp)
	.p2align 4,,7
	.p2align 3
.L2044:
	movzwl	g_gc_superblock+16(%edx,%edx), %eax
	cmpw	$-1, %ax
	je	.L2043
	movzwl	%ax, %eax
	movzwl	%bx, %ecx
	sall	$10, %eax
	addl	$1, %ebx
	orl	-20(%ebp), %eax
	leal	(%ecx,%ecx,8), %ecx
	movl	%eax, 4(%edi,%ecx,4)
.L2043:
	addl	$1, %edx
	cmpw	%dx, %si
	ja	.L2044
	movl	req_gc, %eax
	xorl	%ecx, %ecx
	movzwl	%bx, %edx
	call	FlashReadPages
	testw	%bx, %bx
	je	.L2045
	subl	$1, %ebx
	movzwl	%bx, %ebx
	leal	9(%ebx,%ebx,8), %edi
	xorl	%ebx, %ebx
	sall	$2, %edi
	jmp	.L2048
	.p2align 4,,7
	.p2align 3
.L2046:
	addl	$36, %ebx
	cmpl	%edi, %ebx
	je	.L2045
.L2048:
	movl	req_gc, %eax
	addl	%ebx, %eax
	cmpl	$-1, (%eax)
	movl	12(%eax), %esi
	je	.L2046
	cmpw	$-3947, (%esi)
	jne	.L2046
	movl	8(%esi), %eax
	xorl	%ecx, %ecx
	leal	-16(%ebp), %edx
	call	log2phys
	movl	req_gc, %eax
	movl	-16(%ebp), %edx
	addl	%ebx, %eax
	cmpl	%edx, 4(%eax)
	jne	.L2046
	movl	g_gc_num_req, %edx
	movl	req_gc_dst, %ecx
	movl	16(%eax), %eax
	addw	$1, g_gc_cur_blk_valid_pages
	leal	(%edx,%edx,8), %edx
	leal	(%ecx,%edx,4), %edx
	movl	%eax, 16(%edx)
	movl	%edx, -48(%ebp)
	call	Ftl_get_new_temp_ppa
	movl	-48(%ebp), %edx
	movl	%eax, 4(%edx)
	movl	g_gc_num_req, %eax
	movl	req_gc_dst, %edx
	leal	(%eax,%eax,8), %eax
	leal	(%edx,%eax,4), %eax
	movl	req_gc, %edx
	addl	%ebx, %edx
	movl	8(%edx), %ecx
	movl	%ecx, 8(%eax)
	movl	12(%edx), %edx
	movl	%edx, 12(%eax)
	movl	-16(%ebp), %eax
	movl	$1, %edx
	movl	%eax, 12(%esi)
	movzwl	g_gc_temp_superblock, %eax
	movw	%ax, 2(%esi)
	movl	g_GlobalDataVersion, %eax
	addl	$1, g_gc_num_req
	movl	%eax, 4(%esi)
	movl	req_gc, %eax
	addl	%ebx, %eax
	call	FtlGcBufAlloc
	movzbl	g_gc_temp_superblock+7, %eax
	cmpl	g_gc_num_req, %eax
	je	.L2047
	cmpw	$0, g_gc_temp_superblock+4
	jne	.L2046
.L2047:
	call	Ftl_gc_temp_data_write_back
	testl	%eax, %eax
	je	.L2046
.L2095:
	movl	$0, g_in_gc_progress
	movzwl	gc_ink_free_return_value, %eax
	jmp	.L1971
	.p2align 4,,7
	.p2align 3
.L2049:
	movl	req_gc, %eax
	xorl	%ecx, %ecx
	xorl	%edx, %edx
	call	FlashReadPages
	.p2align 4,,7
	.p2align 3
.L2045:
	addl	$1, -24(%ebp)
	movzwl	-24(%ebp), %ecx
	cmpw	%cx, -40(%ebp)
	ja	.L2080
	movzwl	g_gc_superblock+2, %edx
	movzwl	c_ftl_nand_page_pre_blk, %eax
.L2042:
	addw	-40(%ebp), %dx
	cmpw	%dx, %ax
	movw	%dx, g_gc_superblock+2
	jbe	.L2050
	movzwl	g_gc_cur_blk_max_valid_pages, %ebx
	cmpw	%bx, g_gc_cur_blk_valid_pages
	je	.L2050
.L2051:
	movzwl	g_num_free_superblocks, %eax
	cmpw	$2, %ax
	ja	.L2054
	movzwl	c_ftl_nand_page_pre_blk, %ebx
	movw	%bx, -40(%ebp)
	movzwl	g_gc_superblock, %ebx
	jmp	.L2087
.L2050:
	movl	g_gc_num_req, %ecx
	testl	%ecx, %ecx
	jne	.L2103
.L2052:
	cmpw	$0, g_gc_cur_blk_valid_pages
	jne	.L2053
	movzwl	g_gc_superblock, %eax
	movl	p_valid_page_count_table, %edx
	leal	(%edx,%eax,2), %eax
	cmpw	$0, (%eax)
	je	.L2053
	movw	$0, (%eax)
	movzwl	g_gc_superblock, %eax
	call	update_vpc_list
	call	FtlCacheWriteBack
	call	l2p_flush
	.p2align 4,,5
	call	FtlVpcTblFlush
.L2053:
	movw	$-1, g_gc_superblock
	jmp	.L2051
.L2098:
	movl	g_inkDie_check_enable, %edi
	testl	%edi, %edi
	je	.L1974
	cmpl	$29, g_min_erase_count
	ja	.L1974
	movzwl	gc_ink_free_return_value, %edx
	addl	%edx, %eax
	movl	%eax, g_gc_skip_write_count
	call	FtlGcReFreshBadBlk
	movl	$1, %ecx
	cmpw	$-1, g_gc_superblock
	jne	.L2079
	cmpw	$-1, g_gc_next_blk
	jne	.L1976
	cmpl	$1024, g_gc_skip_write_count
	jbe	.L1976
	movl	g_min_erase_count, %eax
	movl	$0, g_gc_skip_write_count
	movw	$0, gc_ink_free_return_value
	testl	%eax, %eax
	jne	.L1977
	movw	$6, gc_ink_free_return_value
.L1978:
	movl	$32, %eax
	call	List_get_gc_head_node
	cmpw	$-1, %ax
	movl	%eax, %edx
	je	.L1979
	movzwl	g_gc_blk_index, %eax
	testw	%ax, %ax
	je	.L1980
	movl	p_valid_page_count_table, %ecx
	movzwl	%dx, %edx
	movzwl	c_ftl_nand_planes_num, %edi
	leal	(%ecx,%edx,2), %edx
	movl	%ecx, -20(%ebp)
	movzwl	(%edx), %ecx
	movl	%ecx, -24(%ebp)
	movzwl	c_ftl_nand_page_pre_slc_blk, %ecx
	imull	%edi, %ecx
	addl	$1, %ecx
	cmpl	%ecx, -24(%ebp)
	jg	.L1979
	leal	1(%eax), %ecx
	movzwl	%ax, %eax
	movw	%cx, g_gc_blk_index
	movl	%edx, -48(%ebp)
	movl	%ecx, -52(%ebp)
	movl	$0, g_in_swl_replace
	call	List_get_gc_head_node
	movl	-48(%ebp), %edx
	movl	-52(%ebp), %ecx
	cmpw	$-1, %ax
	movw	%ax, -42(%ebp)
	jne	.L2104
.L1979:
	call	GetSwlReplaceBlock
	cmpw	$-1, %ax
	movw	%ax, -42(%ebp)
	jne	.L1974
	movw	$0, gc_ink_free_return_value
	jmp	.L1974
	.p2align 4,,7
	.p2align 3
.L1976:
	cmpw	$-1, g_gc_temp_superblock
	jne	.L2092
	cmpw	$-1, g_gc_next_blk
	jne	.L2092
	cmpw	$24, g_num_free_superblocks
	sbbl	%eax, %eax
	andl	$4096, %eax
	addl	$1024, %eax
	cmpl	%eax, g_gc_skip_write_count
	jbe	.L2092
	movl	$0, g_gc_skip_write_count
	movw	$0, gc_ink_free_return_value
	call	GetSwlReplaceBlock
	xorl	%ecx, %ecx
	cmpw	$-1, %ax
	movw	%ax, -42(%ebp)
	je	.L2105
.L1986:
	movb	%cl, -52(%ebp)
	call	FtlGcReFreshBadBlk
	movzbl	-52(%ebp), %ecx
	jmp	.L1975
	.p2align 4,,7
	.p2align 3
.L2097:
	movw	%ax, g_gc_next_blk
	movw	$-1, g_gc_next_blk_1
	jmp	.L1973
.L2103:
	call	Ftl_gc_temp_data_write_back
	testl	%eax, %eax
	je	.L2052
	.p2align 4,,6
	jmp	.L2095
	.p2align 4,,7
	.p2align 3
.L2031:
	movw	$0, g_gc_blk_index
	addl	$60, %esp
	movl	$8, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2027:
.L2026:
	movzwl	gc_ink_free_return_value, %eax
	movw	$-1, g_gc_superblock
	addl	$60, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2023:
	movw	$-1, g_gc_superblock
	jmp	.L2019
.L1998:
	cmpw	$12, %ax
	jbe	.L1999
	shrw	$4, %bx
	movw	%bx, -40(%ebp)
	jmp	.L1997
.L2078:
	movzwl	g_gc_superblock, %ebx
	jmp	.L2013
.L2102:
	cmpw	$-1, g_gc_next_blk
	jne	.L2002
	movzwl	gc_ink_free_return_value, %eax
	testw	%ax, %ax
	jne	.L2003
	movl	g_MaxLpn, %edx
	leal	(%edx,%edx,2), %edx
	shrl	$2, %edx
	cmpl	g_validLpn, %edx
	jbe	.L2004
.L2003:
	movzwl	c_ftl_nand_data_op_blks_per_plane, %edx
	leal	(%edx,%edx,2), %edx
	sarl	$2, %edx
	movw	%dx, g_gc_free_blk_threshold
.L2005:
	movl	$0, g_in_swl_replace
	movzwl	%ax, %eax
	jmp	.L1971
.L2020:
	movzwl	g_gc_next_blk, %edx
	cmpw	$-1, %dx
	je	.L2021
	movw	%dx, g_gc_superblock
	movw	$-1, g_gc_next_blk
	jmp	.L2021
.L1999:
	movl	%ebx, %edx
	shrw	$2, %dx
	cmpw	$9, %ax
	cmovae	%edx, %ebx
	movw	%bx, -40(%ebp)
	jmp	.L1997
.L1977:
	cmpl	$5, %eax
	ja	.L1978
	movw	$18, gc_ink_free_return_value
	jmp	.L1978
.L2004:
	movw	$18, g_gc_free_blk_threshold
	jmp	.L2005
.L2104:
	movzwl	%ax, %edi
	movzwl	(%edx), %eax
	movzwl	%cx, %ecx
	movl	-20(%ebp), %edx
	movl	%eax, 16(%esp)
	movzwl	(%edx,%edi,2), %eax
	movl	%edi, 8(%esp)
	movl	%ecx, 4(%esp)
	movl	$.LC104, (%esp)
	movl	%eax, 12(%esp)
	call	printk
	cmpw	$40, g_gc_blk_index
	jbe	.L1981
	movl	p_valid_page_count_table, %eax
	cmpw	$32, (%eax,%edi,2)
	jbe	.L1981
	movw	$0, g_gc_blk_index
.L1981:
	movw	$6, gc_ink_free_return_value
	jmp	.L1974
.L1980:
	movw	$1, g_gc_blk_index
	jmp	.L1979
.L2014:
	cmpw	$0, gc_ink_free_return_value
	jne	.L2013
	movzwl	c_ftl_nand_data_op_blks_per_plane, %esi
	leal	(%esi,%esi,2), %eax
	sarl	$2, %eax
	movw	%ax, g_gc_free_blk_threshold
	xorl	%eax, %eax
	call	List_get_gc_head_node
	movzwl	c_ftl_nand_planes_num, %ecx
	movzwl	%ax, %edx
	movl	p_valid_page_count_table, %eax
	movzwl	(%eax,%edx,2), %edx
	movzwl	%dx, %eax
	movl	%eax, -20(%ebp)
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	imull	%ecx, %eax
	movl	%eax, %ecx
	shrl	$31, %ecx
	addl	%ecx, %eax
	sarl	%eax
	cmpl	%eax, -20(%ebp)
	jle	.L2018
	movzwl	%di, %edi
	subl	$1, %esi
	cmpl	%esi, %edi
	jge	.L2096
.L2018:
	testw	%dx, %dx
	movw	$-1, -42(%ebp)
	jne	.L2013
	movl	$65535, %eax
	call	decrement_vpc_count
	movzwl	g_num_free_superblocks, %eax
	addl	$1, %eax
	jmp	.L1971
.L2016:
	movw	$18, g_gc_free_blk_threshold
	jmp	.L2096
.L2054:
	movzwl	gc_ink_free_return_value, %edx
	movzwl	%ax, %ecx
	addl	$1, %ecx
	movl	$0, g_in_gc_progress
	movzwl	%dx, %eax
	testw	%dx, %dx
	cmove	%ecx, %eax
	jmp	.L1971
.L2105:
	movzwl	g_gc_merge_free_blk_threshold, %ecx
	cmpw	%cx, g_num_free_superblocks
	jae	.L1987
	movl	$32, %eax
	call	List_get_gc_head_node
	movl	$1, %ecx
	cmpw	$-1, %ax
	je	.L1986
	movl	g_SlcPartLbaEndSector, %ecx
	testl	%ecx, %ecx
	jne	.L1988
	cmpw	$3, c_ftl_nand_type
	je	.L1989
	movl	g_all_blk_used_slc_mode, %edx
	testl	%edx, %edx
	jne	.L1990
	movl	g_inkDie_check_enable, %edi
	testl	%edi, %edi
	je	.L2106
.L1990:
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	xorl	%ecx, %ecx
	movzwl	(%edx,%eax,2), %edi
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	movzwl	c_ftl_nand_planes_num, %edx
	imull	%edx, %eax
.L1992:
	addl	%ecx, %eax
	cmpl	%edi, %eax
	jl	.L1993
	xorl	%eax, %eax
	call	List_get_gc_head_node
	movw	%ax, -42(%ebp)
	movl	g_MaxLpn, %eax
	leal	(%eax,%eax,2), %eax
	shrl	$2, %eax
	cmpl	g_validLpn, %eax
	jae	.L1994
	cmpw	$-1, -42(%ebp)
	movw	$128, g_gc_merge_free_blk_threshold
	sete	%cl
	jmp	.L1986
.L1989:
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	movzwl	c_ftl_nand_planes_num, %ecx
	movzwl	(%edx,%eax,2), %edi
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	movzwl	%dx, %eax
	imull	%ecx, %eax
.L2058:
	shrw	%dx
	movzwl	%dx, %ecx
	jmp	.L1992
.L1988:
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	movzwl	c_ftl_nand_planes_num, %ecx
	movzwl	(%edx,%eax,2), %edi
	movzwl	c_ftl_nand_page_pre_slc_blk, %edx
	movzwl	%dx, %eax
	imull	%ecx, %eax
	xorl	%ecx, %ecx
	cmpw	$3, c_ftl_nand_type
	jne	.L1992
	jmp	.L2058
.L1987:
	movw	$80, g_gc_merge_free_blk_threshold
	movl	$1, %ecx
	jmp	.L1986
.L2106:
	movl	p_valid_page_count_table, %edx
	movzwl	%ax, %eax
	cmpw	$7, (%edx,%eax,2)
	ja	.L1995
	xorl	%eax, %eax
	call	List_get_gc_head_node
	movw	$128, g_gc_merge_free_blk_threshold
	cmpw	$-1, %ax
	movw	%ax, -42(%ebp)
	sete	%cl
	jmp	.L1986
.L1994:
	cmpw	$-1, -42(%ebp)
	movw	$160, g_gc_merge_free_blk_threshold
	sete	%cl
	jmp	.L1986
.L1993:
	movw	$128, g_gc_merge_free_blk_threshold
	movl	$1, %ecx
	jmp	.L1986
.L1995:
	movw	$64, g_gc_merge_free_blk_threshold
	movl	$1, %ecx
	jmp	.L1986
	.size	FtlGarbageCollect, .-FtlGarbageCollect
	.p2align 4,,15
	.globl	FtlRead
	.type	FtlRead, @function
FtlRead:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$64, %esp
	cmpb	$16, %al
	movl	%edx, -24(%ebp)
	movl	%ecx, -52(%ebp)
	je	.L2141
	movl	-52(%ebp), %eax
	addl	-24(%ebp), %eax
	cmpl	g_MaxLbaSector, %eax
	movl	$-1, -48(%ebp)
	movl	%eax, -40(%ebp)
	ja	.L2109
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	subl	$1, %eax
	movl	-24(%ebp), %edx
	shrl	%cl, %eax
	shrl	%cl, %edx
	movl	-52(%ebp), %ecx
	movl	%eax, -36(%ebp)
	subl	%edx, %eax
	addl	$1, %eax
	movl	%edx, -32(%ebp)
	movl	-36(%ebp), %edx
	addl	%eax, g_totle_read_page_count
	addl	%ecx, g_totle_read_sector
	movl	%eax, -28(%ebp)
	movl	-32(%ebp), %eax
	call	FtlCacheMetchLpa
	testl	%eax, %eax
	jne	.L2142
.L2110:
	movl	-28(%ebp), %eax
	testl	%eax, %eax
	je	.L2143
	movl	-32(%ebp), %ebx
	movl	$0, -60(%ebp)
	movl	$0, -56(%ebp)
	movl	$0, -20(%ebp)
	movl	$0, -48(%ebp)
	.p2align 4,,7
	.p2align 3
.L2139:
	xorl	%ecx, %ecx
	movl	%ebx, %eax
	leal	-16(%ebp), %edx
	call	log2phys
	movl	-16(%ebp), %edx
	cmpl	$-1, %edx
	je	.L2144
	movl	-20(%ebp), %ecx
	movl	req_read, %edi
	cmpl	-32(%ebp), %ebx
	leal	(%ecx,%ecx,8), %eax
	leal	(%edi,%eax,4), %eax
	movl	%edx, 4(%eax)
	je	.L2145
	cmpl	-36(%ebp), %ebx
	je	.L2146
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%ebx, %edx
	sall	%cl, %edx
	movl	%edx, %ecx
.L2140:
	subl	-24(%ebp), %ecx
	sall	$9, %ecx
	addl	8(%ebp), %ecx
	movl	%ecx, 8(%eax)
.L2122:
	movzwl	c_ftl_nand_byte_pre_oob, %edx
	imull	-20(%ebp), %edx
	addl	$1, -20(%ebp)
	movl	%ebx, 16(%eax)
	andl	$-4, %edx
	addl	p_io_spare_buf, %edx
	movl	%edx, 12(%eax)
.L2114:
	addl	$1, %ebx
	subl	$1, -28(%ebp)
	je	.L2124
	movzwl	c_ftl_nand_planes_num, %eax
	sall	$2, %eax
	cmpl	-20(%ebp), %eax
	jne	.L2139
.L2124:
	movl	-20(%ebp), %esi
	testl	%esi, %esi
	jne	.L2147
	movl	-28(%ebp), %ecx
	testl	%ecx, %ecx
	je	.L2112
.L2134:
	movl	$0, -20(%ebp)
	jmp	.L2139
	.p2align 4,,7
	.p2align 3
.L2144:
	cmpw	$0, c_ftl_nand_sec_pre_page
	je	.L2114
	xorl	%edx, %edx
	jmp	.L2120
	.p2align 4,,7
	.p2align 3
.L2115:
	movzwl	c_ftl_nand_sec_pre_page, %eax
	addl	$1, %edx
	cmpl	%edx, %eax
	jbe	.L2114
.L2120:
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%ebx, %eax
	sall	%cl, %eax
	addl	%edx, %eax
	cmpl	%eax, -24(%ebp)
	ja	.L2115
	cmpl	%eax, -40(%ebp)
	jbe	.L2115
	subl	-24(%ebp), %eax
	movl	$512, %esi
	sall	$9, %eax
	addl	8(%ebp), %eax
	testb	$1, %al
	movl	%eax, %edi
	jne	.L2148
.L2116:
	testl	$2, %edi
	jne	.L2149
.L2117:
	movl	%esi, %ecx
	xorl	%eax, %eax
	shrl	$2, %ecx
	testl	$2, %esi
	rep stosl
	je	.L2118
	movw	$0, (%edi)
	addl	$2, %edi
.L2118:
	andl	$1, %esi
	je	.L2115
	movb	$0, (%edi)
	jmp	.L2115
	.p2align 4,,7
	.p2align 3
.L2145:
	movl	p_io_data_buf_0, %edx
	movl	%edx, 8(%eax)
	movzwl	c_ftl_nand_sec_pre_page, %edx
	leal	-1(%edx), %ecx
	movl	%ecx, -44(%ebp)
	movl	-24(%ebp), %ecx
	andl	%ecx, -44(%ebp)
	movl	%edx, %ecx
	subl	-44(%ebp), %ecx
	cmpl	-52(%ebp), %ecx
	cmova	-52(%ebp), %ecx
	cmpl	%ecx, %edx
	movl	%ecx, -56(%ebp)
	jne	.L2122
	movl	8(%ebp), %ecx
	movl	%ecx, 8(%eax)
	jmp	.L2122
	.p2align 4,,7
	.p2align 3
.L2147:
	movl	-20(%ebp), %edx
	xorl	%ecx, %ecx
	xorl	%esi, %esi
	movl	req_read, %eax
	xorl	%edi, %edi
	call	FlashReadPages
	movl	-56(%ebp), %ecx
	movl	-44(%ebp), %eax
	movl	-60(%ebp), %edx
	movl	%ebx, -64(%ebp)
	movl	-20(%ebp), %ebx
	sall	$9, %ecx
	sall	$9, %eax
	sall	$9, %edx
	movl	%ecx, -68(%ebp)
	movl	%eax, -72(%ebp)
	movl	%edx, -76(%ebp)
	jmp	.L2131
	.p2align 4,,7
	.p2align 3
.L2127:
	cmpl	%edx, -36(%ebp)
	je	.L2150
.L2128:
	movl	(%eax), %edx
	cmpl	$-1, %edx
	je	.L2151
.L2129:
	cmpl	$256, %edx
	je	.L2152
.L2130:
	addl	$1, %edi
	addl	$36, %esi
	cmpl	%ebx, %edi
	je	.L2153
.L2131:
	movl	req_read, %eax
	addl	%esi, %eax
	movl	16(%eax), %edx
	cmpl	-32(%ebp), %edx
	jne	.L2127
	movl	8(%eax), %edx
	cmpl	p_io_data_buf_0, %edx
	jne	.L2128
	movl	-68(%ebp), %ecx
	movl	8(%ebp), %eax
	addl	-72(%ebp), %edx
	call	memcpy
	movl	req_read, %eax
	addl	%esi, %eax
	movl	(%eax), %edx
	cmpl	$-1, %edx
	jne	.L2129
	.p2align 4,,7
	.p2align 3
.L2151:
	addl	$1, g_sys_ext_data+72
	movl	(%eax), %edx
	movl	$-1, -48(%ebp)
	cmpl	$256, %edx
	jne	.L2130
.L2152:
	movl	4(%eax), %eax
	xorl	%edx, %edx
	addl	$1, %edi
	addl	$36, %esi
	shrl	$10, %eax
	divw	c_ftl_nand_blks_per_die
	movl	%edx, %eax
	xorl	%edx, %edx
	divw	c_ftl_nand_planes_per_die
	movzwl	%ax, %eax
	call	FtlGcRefreshBlock
	cmpl	%ebx, %edi
	jne	.L2131
	.p2align 4,,7
	.p2align 3
.L2153:
	movl	-28(%ebp), %ecx
	movl	-64(%ebp), %ebx
	testl	%ecx, %ecx
	jne	.L2134
.L2112:
	cmpw	$0, g_gc_bad_block_temp_num
	jne	.L2154
.L2109:
	movl	-48(%ebp), %eax
	addl	$64, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2150:
	movl	8(%eax), %edx
	cmpl	p_io_data_buf_1, %edx
	jne	.L2128
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	-36(%ebp), %eax
	sall	%cl, %eax
	movl	%eax, %ecx
	movl	8(%ebp), %eax
	subl	-24(%ebp), %ecx
	sall	$9, %ecx
	addl	%ecx, %eax
	movl	-76(%ebp), %ecx
	call	memcpy
	movl	req_read, %eax
	addl	%esi, %eax
	jmp	.L2128
	.p2align 4,,7
	.p2align 3
.L2146:
	movl	p_io_data_buf_1, %edx
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%edx, 8(%eax)
	movl	%ebx, %edx
	sall	%cl, %edx
	movl	%edx, %ecx
	movl	-40(%ebp), %edx
	subl	%ecx, %edx
	movl	%edx, -60(%ebp)
	movzwl	c_ftl_nand_sec_pre_page, %edx
	cmpl	-60(%ebp), %edx
	jne	.L2122
	jmp	.L2140
	.p2align 4,,7
	.p2align 3
.L2154:
	movl	$1, %edx
	xorl	%eax, %eax
	call	FtlGarbageCollect
	movl	-48(%ebp), %eax
	addl	$64, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2149:
	movw	$0, (%edi)
	subl	$2, %esi
	addl	$2, %edi
	jmp	.L2117
	.p2align 4,,7
	.p2align 3
.L2148:
	movb	$0, (%eax)
	addl	$1, %edi
	movw	$511, %si
	jmp	.L2116
	.p2align 4,,7
	.p2align 3
.L2142:
	call	FtlCacheWriteBack
	jmp	.L2110
.L2141:
	movl	%edx, %eax
	movl	8(%ebp), %ecx
	movl	-52(%ebp), %edx
	addl	$256, %eax
	call	FtlVendorPartRead
	movl	%eax, -48(%ebp)
	jmp	.L2109
.L2143:
	movl	$0, -48(%ebp)
	jmp	.L2112
	.size	FtlRead, .-FtlRead
	.section	.rodata.str1.1
.LC105:
	.string	"\0013FtlWrite: lpa error:%x %x\n"
	.text
	.p2align 4,,15
	.globl	FtlWrite
	.type	FtlWrite, @function
FtlWrite:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%ecx, %ebx
	subl	$108, %esp
	cmpb	$16, %al
	movl	%edx, -80(%ebp)
	je	.L2211
	movl	-80(%ebp), %edx
	movl	$-1, %eax
	addl	%ecx, %edx
	cmpl	g_MaxLbaSector, %edx
	ja	.L2157
	movzwl	c_ftl_nand_sec_pre_page_shift, %esi
	subl	$1, %edx
	movl	-80(%ebp), %eax
	movl	$g_active_superblock, -60(%ebp)
	addl	%ebx, g_totle_write_sector
	movl	%esi, %ecx
	shrl	%cl, %eax
	shrl	%cl, %edx
	movl	%edx, -72(%ebp)
	subl	%eax, %edx
	movl	%eax, -76(%ebp)
	movl	$g_buffer_superblock, %eax
	movl	%edx, -108(%ebp)
	addl	$1, %edx
	addl	%edx, g_totle_write_page_count
	cmpl	$9, %ebx
	cmovae	-60(%ebp), %eax
	movl	$2048, g_ftl_nand_free_count
	movl	%edx, -92(%ebp)
	movl	%eax, -60(%ebp)
	movl	g_wr_page_num, %eax
	testl	%eax, %eax
	je	.L2197
	movl	req_wr_io, %edx
	leal	-9(%eax,%eax,8), %eax
	movl	%ebx, -104(%ebp)
	leal	(%edx,%eax,4), %eax
	movl	16(%eax), %edi
	cmpl	-76(%ebp), %edi
	je	.L2212
.L2160:
	movl	gp_last_act_superblock, %eax
	movl	-72(%ebp), %edx
	movl	$0, last_cache_match_count
	movl	%eax, -60(%ebp)
	movl	-76(%ebp), %eax
	call	FtlCacheMetchLpa
	testl	%eax, %eax
	jne	.L2213
.L2162:
	movl	-92(%ebp), %eax
	movl	-60(%ebp), %edx
	testl	%eax, %eax
	movl	%edx, gp_last_act_superblock
	je	.L2163
	movl	%esi, %ecx
	movl	$2, %eax
	movl	-104(%ebp), %edx
	sall	%cl, %eax
	movl	-76(%ebp), %esi
	cmpl	%eax, %ebx
	setae	%al
	addl	-80(%ebp), %edx
	movzbl	%al, %eax
	movl	%eax, -88(%ebp)
	movl	%edx, -100(%ebp)
	.p2align 4,,7
	.p2align 3
.L2209:
	movl	-60(%ebp), %ecx
	movzwl	4(%ecx), %edi
	testw	%di, %di
	jne	.L2165
	cmpl	$g_active_superblock, %ecx
	je	.L2214
	movzwl	g_active_superblock+4, %edi
	movl	$0, power_up_flag
	testw	%di, %di
	je	.L2215
	movl	$g_active_superblock, -60(%ebp)
.L2170:
	movl	-60(%ebp), %ecx
	movl	%ecx, gp_last_act_superblock
.L2165:
	movl	c_wr_page_buf_num, %eax
	movzwl	%di, %ecx
	movl	g_wr_page_num, %edx
	movl	%eax, -64(%ebp)
	movl	-64(%ebp), %ebx
	movl	%edx, %eax
	subl	%edx, %ebx
	cmpl	%ebx, -92(%ebp)
	cmovbe	-92(%ebp), %ebx
	cmpl	%ecx, %ebx
	cmovbe	%ebx, %ecx
	testl	%ecx, %ecx
	movl	%ecx, -68(%ebp)
	je	.L2216
	testw	%di, %di
	je	.L2173
	movzbl	-88(%ebp), %edx
	movl	$0, -56(%ebp)
	movb	%dl, -81(%ebp)
	jmp	.L2174
	.p2align 4,,7
	.p2align 3
.L2205:
	movzwl	c_ftl_nand_sec_pre_page, %edx
	leal	-1(%edx), %eax
	movl	%edx, %ecx
	andw	-80(%ebp), %ax
	movzwl	%ax, %eax
	movl	%eax, -96(%ebp)
	movl	%edx, %eax
	movl	-104(%ebp), %edx
	subl	-96(%ebp), %ecx
	cmpl	%ecx, -104(%ebp)
	cmovae	%ecx, %edx
	movl	%edx, -64(%ebp)
.L2179:
	cmpl	%eax, -64(%ebp)
	je	.L2217
	movl	-16(%ebp), %eax
	cmpl	$-1, %eax
	je	.L2185
	movl	%eax, -48(%ebp)
	movl	g_wr_page_num, %eax
	movl	req_wr_io, %ecx
	movl	%esi, -36(%ebp)
	leal	(%eax,%eax,8), %eax
	leal	(%ecx,%eax,4), %eax
	xorl	%ecx, %ecx
	movl	8(%eax), %edx
	movl	12(%eax), %eax
	movl	%edx, -44(%ebp)
	movl	$1, %edx
	movl	%eax, -40(%ebp)
	leal	-52(%ebp), %eax
	call	FlashReadPages
	cmpl	$-1, -52(%ebp)
	je	.L2218
	cmpl	%esi, 8(%ebx)
	jne	.L2219
.L2187:
	movl	%edi, %eax
	testb	%al, %al
	movl	g_wr_page_num, %eax
	leal	(%eax,%eax,8), %eax
	jne	.L2220
.L2188:
	movl	req_wr_io, %edi
	leal	(%edi,%eax,4), %eax
	movl	8(%eax), %edi
	movl	-64(%ebp), %eax
	sall	$9, %eax
.L2210:
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%esi, %edx
	sall	%cl, %edx
	movl	%eax, %ecx
	subl	-80(%ebp), %edx
	movl	%edi, %eax
	sall	$9, %edx
	addl	8(%ebp), %edx
	call	memcpy
.L2184:
	movw	$-3947, (%ebx)
	movl	g_GlobalDataVersion, %edx
	xorl	%eax, %eax
	movl	-60(%ebp), %ecx
	movl	%esi, 8(%ebx)
	movl	%edx, 4(%ebx)
	addl	$1, %edx
	cmpl	$-1, %edx
	cmovne	%edx, %eax
	addl	$1, %esi
	movl	%eax, g_GlobalDataVersion
	movl	-16(%ebp), %eax
	addl	$1, -56(%ebp)
	movl	%eax, 12(%ebx)
	movzwl	(%ecx), %eax
	movl	-68(%ebp), %ecx
	cmpl	%ecx, -56(%ebp)
	movw	%ax, 2(%ebx)
	movl	g_wr_page_num, %eax
	leal	1(%eax), %edx
	movl	%edx, %eax
	movl	%edx, g_wr_page_num
	je	.L2221
	movl	-60(%ebp), %ecx
	cmpw	$0, 4(%ecx)
	je	.L2222
.L2174:
	cmpl	-72(%ebp), %esi
	sete	-64(%ebp)
	jne	.L2175
	cmpb	$0, -81(%ebp)
	je	.L2175
	movl	-56(%ebp), %edi
	testl	%edi, %edi
	je	.L2175
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%esi, %edx
	sall	%cl, %edx
	movl	%edx, %ecx
	movl	-100(%ebp), %edx
	subl	%ecx, %edx
	movl	%edx, %ecx
	movzwl	c_ftl_nand_sec_pre_page, %edx
	cmpl	%edx, %ecx
	jne	.L2223
.L2175:
	xorl	%ecx, %ecx
	movl	%esi, %eax
	leal	-16(%ebp), %edx
	call	log2phys
	movl	-60(%ebp), %eax
	call	get_new_active_ppa
	movl	g_wr_page_num, %edi
	movl	req_wr_io, %ebx
	movzwl	c_ftl_nand_byte_pre_oob, %ecx
	leal	(%edi,%edi,8), %edx
	leal	(%ebx,%edx,4), %edx
	movl	%eax, 4(%edx)
	movzwl	c_ftl_nand_byte_pre_page, %eax
	movl	%ecx, %ebx
	imull	%edi, %ebx
	movl	%esi, 16(%edx)
	imull	%edi, %eax
	andl	$-4, %ebx
	addl	p_wr_io_spare_buf, %ebx
	andl	$-4, %eax
	addl	p_wr_io_data_buf, %eax
	movl	%ebx, 12(%edx)
	movl	%eax, 8(%edx)
	xorl	%edx, %edx
	movl	%ebx, %eax
	call	memset
	cmpl	-76(%ebp), %esi
	sete	%dl
	movl	%edx, %edi
	je	.L2205
	cmpb	$0, -64(%ebp)
	je	.L2176
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%esi, %edx
	movzwl	-104(%ebp), %eax
	addw	-80(%ebp), %ax
	movl	$0, -96(%ebp)
	sall	%cl, %edx
	subw	%dx, %ax
	movzwl	%ax, %eax
	movl	%eax, -64(%ebp)
	movzwl	c_ftl_nand_sec_pre_page, %eax
	jmp	.L2179
	.p2align 4,,7
	.p2align 3
.L2176:
	movl	g_wr_page_num, %eax
	movl	-88(%ebp), %ecx
	movl	req_wr_io, %edx
	leal	(%eax,%eax,8), %eax
	testl	%ecx, %ecx
	leal	(%edx,%eax,4), %eax
	je	.L2189
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%esi, %edx
	sall	%cl, %edx
	movl	%edx, %ecx
	subl	-80(%ebp), %ecx
	sall	$9, %ecx
	addl	8(%ebp), %ecx
	movl	%ecx, 8(%eax)
	jmp	.L2184
	.p2align 4,,7
	.p2align 3
.L2189:
	movl	8(%eax), %edi
	movzwl	c_ftl_nand_byte_pre_page, %eax
	jmp	.L2210
	.p2align 4,,7
	.p2align 3
.L2217:
	movl	%edi, %ecx
	movl	8(%ebp), %edx
	testb	%cl, %cl
	jne	.L2182
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	movl	%esi, %edx
	sall	%cl, %edx
	subl	-80(%ebp), %edx
	sall	$9, %edx
	addl	8(%ebp), %edx
.L2182:
	movl	g_wr_page_num, %eax
	movl	-88(%ebp), %ecx
	movl	req_wr_io, %edi
	leal	(%eax,%eax,8), %eax
	testl	%ecx, %ecx
	leal	(%edi,%eax,4), %eax
	je	.L2183
	movl	%edx, 8(%eax)
	jmp	.L2184
	.p2align 4,,7
	.p2align 3
.L2185:
	movl	g_wr_page_num, %eax
	movl	req_wr_io, %edx
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	leal	(%eax,%eax,8), %eax
	leal	(%edx,%eax,4), %eax
	xorl	%edx, %edx
	movl	8(%eax), %eax
	call	memset
	movl	%edi, %eax
	testb	%al, %al
	movl	g_wr_page_num, %eax
	leal	(%eax,%eax,8), %eax
	je	.L2188
.L2220:
	movl	req_wr_io, %edx
	movl	-96(%ebp), %edi
	movl	-64(%ebp), %ecx
	leal	(%edx,%eax,4), %eax
	movl	8(%ebp), %edx
	sall	$9, %edi
	addl	8(%eax), %edi
	sall	$9, %ecx
	movl	%edi, %eax
	call	memcpy
	jmp	.L2184
	.p2align 4,,7
	.p2align 3
.L2218:
	addl	$1, g_sys_ext_data+72
	jmp	.L2187
	.p2align 4,,7
	.p2align 3
.L2183:
	movzwl	c_ftl_nand_byte_pre_page, %ecx
	movl	8(%eax), %eax
	call	memcpy
	jmp	.L2184
	.p2align 4,,7
	.p2align 3
.L2221:
	movl	c_wr_page_buf_num, %eax
	movl	%eax, -64(%ebp)
.L2172:
	movl	-68(%ebp), %ecx
	subl	%ecx, -92(%ebp)
	cmpl	-64(%ebp), %edx
	jae	.L2192
	cmpb	$0, -81(%ebp)
	jne	.L2192
	movl	-60(%ebp), %eax
	cmpw	$0, 4(%eax)
	jne	.L2193
.L2192:
	call	FtlCacheWriteBack
	cmpl	$3, -92(%ebp)
	movl	$0, g_wr_page_num
	ja	.L2209
.L2193:
	movl	-92(%ebp), %eax
	testl	%eax, %eax
	je	.L2163
	movl	$0, -88(%ebp)
	jmp	.L2209
	.p2align 4,,7
	.p2align 3
.L2222:
	movl	c_wr_page_buf_num, %eax
	movl	-56(%ebp), %ecx
	movl	%eax, -64(%ebp)
	movl	%ecx, -68(%ebp)
	jmp	.L2172
	.p2align 4,,7
	.p2align 3
.L2223:
	movl	%eax, %edx
	movl	c_wr_page_buf_num, %ecx
	movl	-56(%ebp), %eax
	movb	$1, -81(%ebp)
	movl	%ecx, -64(%ebp)
	movl	%eax, -68(%ebp)
	jmp	.L2172
.L2215:
	movl	-60(%ebp), %eax
	call	allocate_new_data_superblock
	movl	-60(%ebp), %eax
	movzwl	4(%eax), %edi
.L2169:
	testw	%di, %di
	jne	.L2170
	movl	-60(%ebp), %eax
	call	allocate_new_data_superblock
	movl	-60(%ebp), %edx
	movzwl	4(%edx), %edi
	jmp	.L2170
.L2163:
	movl	-108(%ebp), %edx
	xorl	%eax, %eax
	call	FtlGarbageCollect
	xorl	%eax, %eax
.L2157:
	addl	$108, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2214:
	cmpw	$0, g_buffer_superblock+4
	je	.L2224
.L2167:
	movl	$g_active_superblock, %eax
	call	allocate_new_data_superblock
	movl	power_up_flag, %eax
	movzwl	g_active_superblock+4, %edi
	testl	%eax, %eax
	je	.L2169
	movzwl	g_buffer_superblock+4, %edi
	movl	$g_buffer_superblock, -60(%ebp)
	jmp	.L2169
.L2216:
	movzbl	-88(%ebp), %ecx
	movb	%cl, -81(%ebp)
	jmp	.L2172
.L2197:
	movl	-72(%ebp), %edx
	movl	-76(%ebp), %eax
	movl	%ebx, -104(%ebp)
	call	FtlCacheMetchLpa
	testl	%eax, %eax
	je	.L2162
.L2213:
	call	FtlCacheWriteBack
	.p2align 4,,6
	jmp	.L2162
.L2173:
	movzbl	-88(%ebp), %ecx
	movl	$0, -68(%ebp)
	movb	%cl, -81(%ebp)
	jmp	.L2172
.L2224:
	movl	$g_buffer_superblock, %eax
	call	allocate_new_data_superblock
	movl	$0, power_up_flag
	jmp	.L2167
.L2212:
	movzwl	c_ftl_nand_sec_pre_page, %ecx
	addl	$1, g_totle_cache_write_count
	addl	$1, last_cache_match_count
	leal	-1(%ecx), %edx
	andw	-80(%ebp), %dx
	movzwl	%dx, %edx
	movl	%edx, -60(%ebp)
	movl	%ecx, %edx
	movl	8(%eax), %ecx
	subl	-60(%ebp), %edx
	cmpl	%ebx, %edx
	cmova	%ebx, %edx
	movl	%edx, -56(%ebp)
	sall	$9, %edx
	movl	%edx, -64(%ebp)
	movl	-60(%ebp), %edx
	sall	$9, %edx
	addl	%ecx, %edx
	movl	-64(%ebp), %ecx
	movl	%edx, -60(%ebp)
	movl	8(%ebp), %edx
	movl	-60(%ebp), %eax
	call	memcpy
	movl	-108(%ebp), %eax
	testl	%eax, %eax
	jne	.L2161
	xorl	%eax, %eax
	cmpl	$2, last_cache_match_count
	jle	.L2157
.L2161:
	movl	-64(%ebp), %ecx
	movl	%ebx, %eax
	addl	$1, %edi
	movl	-56(%ebp), %edx
	addl	%ecx, 8(%ebp)
	movl	-72(%ebp), %ecx
	addl	%edx, -80(%ebp)
	movl	-108(%ebp), %edx
	subl	-56(%ebp), %eax
	subl	%edi, %ecx
	movl	%edi, -76(%ebp)
	movl	%eax, -104(%ebp)
	movl	%edx, -92(%ebp)
	movl	%ecx, -108(%ebp)
	jmp	.L2160
.L2211:
	movl	8(%ebp), %ecx
	movl	%edx, %eax
	movl	%ebx, %edx
	addl	$256, %eax
	call	FtlVendorPartWrite
	addl	$108, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2219:
	addl	$1, g_sys_ext_data+72
	movl	%esi, 8(%esp)
	movl	8(%ebx), %eax
	movl	$.LC105, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L2187
	.size	FtlWrite, .-FtlWrite
	.p2align 4,,15
	.type	nand_transfer, @function
nand_transfer:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	subl	$4, %esp
	movl	8(%ebp), %ebx
	testl	%ebx, %ebx
	jne	.L2234
	addl	%edx, TotleReadData
	addl	$1, TotleReadCount
	movl	%ecx, (%esp)
	movl	%edx, %ecx
	movl	%eax, %edx
	xorl	%eax, %eax
	call	FtlRead
	cmpl	$1, %eax
	sbbl	%eax, %eax
	addl	$4, %esp
	notl	%eax
	andl	$-5, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2234:
	cmpl	$1, %ebx
	je	.L2235
	addl	$4, %esp
	movl	$-5, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2235:
	addl	%edx, TotleWriteData
	addl	$1, TotleWriteCount
	movl	%ecx, (%esp)
	movl	%edx, %ecx
	movl	%eax, %edx
	xorl	%eax, %eax
	call	FtlWrite
	cmpl	$1, %eax
	sbbl	%eax, %eax
	addl	$4, %esp
	notl	%eax
	andl	$-5, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_transfer, .-nand_transfer
	.p2align 4,,15
	.globl	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, @function
FtlLowFormatEraseBlock:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$32, %esp
	movw	%ax, -18(%ebp)
	movzwl	%ax, %eax
	movl	%eax, g_cur_erase_blk
	xorl	%eax, %eax
	cmpw	$0, c_ftl_nand_planes_num
	movb	%dl, -15(%ebp)
	je	.L2244
	xorl	%ebx, %ebx
	movw	$0, -14(%ebp)
	movw	$0, -20(%ebp)
	jmp	.L2242
	.p2align 4,,7
	.p2align 3
.L2269:
	movzwl	-14(%ebp), %edx
	sall	$10, %edi
	movl	req_erase, %ecx
	leal	(%edx,%edx,8), %eax
	leal	(%ecx,%eax,4), %eax
	movzwl	c_ftl_nand_byte_pre_oob, %ecx
	movl	%edi, 4(%eax)
	movl	$0, 8(%eax)
	imull	%ecx, %edx
	testl	%edx, %edx
	leal	3(%edx), %ecx
	cmovs	%ecx, %edx
	andl	$-4, %edx
	addw	$1, -14(%ebp)
	addl	p_io_spare_buf, %edx
	movl	%edx, 12(%eax)
.L2240:
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_planes_num
	jbe	.L2268
.L2242:
	movl	req_erase, %esi
	movzwl	%bx, %eax
	leal	(%eax,%eax,8), %edx
	leal	(%esi,%edx,4), %edx
	movl	$0, (%edx)
	movzwl	c_ftl_nand_planes_per_die, %ecx
	xorl	%edx, %edx
	movzbl	p_plane_order_table(%eax), %eax
	movzwl	c_ftl_nand_blks_per_die, %esi
	divw	%cx
	imulw	-18(%ebp), %cx
	imull	%esi, %eax
	addl	%edx, %ecx
	addl	%eax, %ecx
	cmpb	$0, -15(%ebp)
	movzwl	%cx, %edi
	jne	.L2238
.L2239:
	movl	%ecx, %eax
	xorl	%edx, %edx
	divw	%si
	movzwl	%ax, %eax
	movl	%edx, %esi
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %si
	movl	%edx, %ecx
	movzwl	%si, %esi
	movl	(%eax,%esi,4), %eax
	shrl	%cl, %eax
	testb	$1, %al
	je	.L2269
	addw	$1, -20(%ebp)
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_planes_num
	ja	.L2242
.L2268:
	cmpw	$0, -14(%ebp)
	jne	.L2243
.L2258:
	movzwl	-20(%ebp), %eax
.L2244:
	addl	$32, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2238:
	movl	%edi, %eax
	movl	%ecx, -40(%ebp)
	call	IsBlkInVendorPart
	movl	-40(%ebp), %ecx
	testl	%eax, %eax
	jne	.L2240
	jmp	.L2239
	.p2align 4,,7
	.p2align 3
.L2243:
	movzwl	-14(%ebp), %edx
	xorl	%ebx, %ebx
	movl	req_erase, %eax
	call	FlashEraseBlocks
	movzwl	-14(%ebp), %eax
	movzwl	-20(%ebp), %edi
	subl	$1, %eax
	movzwl	%ax, %eax
	leal	9(%eax,%eax,8), %esi
	sall	$2, %esi
	jmp	.L2246
	.p2align 4,,7
	.p2align 3
.L2245:
	addl	$36, %ebx
	cmpl	%esi, %ebx
	je	.L2270
.L2246:
	movl	req_erase, %eax
	addl	%ebx, %eax
	cmpl	$-1, (%eax)
	jne	.L2245
	movl	4(%eax), %eax
	addl	$36, %ebx
	addl	$1, %edi
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlBbmMapBadBlock
	cmpl	%esi, %ebx
	jne	.L2246
	.p2align 4,,7
	.p2align 3
.L2270:
	movw	%di, -20(%ebp)
	movl	g_inkDie_check_enable, %edi
	testl	%edi, %edi
	jne	.L2271
.L2261:
	xorl	%eax, %eax
	movw	$6, -34(%ebp)
	movw	$1, -28(%ebp)
.L2247:
	movzwl	%ax, %eax
	movw	$0, -26(%ebp)
	movl	%eax, -32(%ebp)
	.p2align 4,,7
	.p2align 3
.L2257:
	cmpw	$0, c_ftl_nand_planes_num
	je	.L2258
	movzwl	-26(%ebp), %edx
	xorl	%ebx, %ebx
	movw	$0, -14(%ebp)
	movl	%edx, -24(%ebp)
	jmp	.L2252
	.p2align 4,,7
	.p2align 3
.L2250:
	movl	%ecx, %eax
	xorl	%edx, %edx
	divw	%si
	movzwl	%ax, %eax
	movl	%edx, %esi
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %si
	movl	%edx, %ecx
	movzwl	%si, %esi
	movl	(%eax,%esi,4), %eax
	shrl	%cl, %eax
	testb	$1, %al
	jne	.L2251
	movzwl	-14(%ebp), %edx
	sall	$10, %edi
	movl	req_erase, %ecx
	addl	-24(%ebp), %edi
	leal	(%edx,%edx,8), %eax
	leal	(%ecx,%eax,4), %eax
	movl	p_io_data_buf_0, %ecx
	movl	%edi, 4(%eax)
	movl	%ecx, 8(%eax)
	movzwl	c_ftl_nand_byte_pre_oob, %ecx
	imull	%ecx, %edx
	testl	%edx, %edx
	leal	3(%edx), %ecx
	cmovs	%ecx, %edx
	andl	$-4, %edx
	addw	$1, -14(%ebp)
	addl	p_io_data_buf_1, %edx
	movl	%edx, 12(%eax)
.L2251:
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_planes_num
	jbe	.L2272
.L2252:
	movl	req_erase, %esi
	movzwl	%bx, %eax
	leal	(%eax,%eax,8), %edx
	leal	(%esi,%edx,4), %edx
	movl	$0, (%edx)
	movzwl	c_ftl_nand_planes_per_die, %ecx
	xorl	%edx, %edx
	movzbl	p_plane_order_table(%eax), %eax
	movzwl	c_ftl_nand_blks_per_die, %esi
	divw	%cx
	imulw	-18(%ebp), %cx
	imull	%esi, %eax
	addl	%edx, %ecx
	addl	%eax, %ecx
	cmpb	$0, -15(%ebp)
	movzwl	%cx, %edi
	je	.L2250
	movl	%edi, %eax
	movl	%ecx, -40(%ebp)
	call	IsBlkInVendorPart
	movl	-40(%ebp), %ecx
	testl	%eax, %eax
	je	.L2250
	addl	$1, %ebx
	cmpw	%bx, c_ftl_nand_planes_num
	ja	.L2252
	.p2align 4,,7
	.p2align 3
.L2272:
	cmpw	$0, -14(%ebp)
	je	.L2258
	movzwl	-14(%ebp), %edi
	xorl	%ebx, %ebx
	movl	-32(%ebp), %ecx
	movl	req_erase, %eax
	movl	$1, (%esp)
	movl	%edi, %edx
	call	FlashProgPages
	movzwl	-14(%ebp), %eax
	subl	$1, %eax
	movzwl	%ax, %eax
	leal	9(%eax,%eax,8), %esi
	sall	$2, %esi
	jmp	.L2256
	.p2align 4,,7
	.p2align 3
.L2255:
	addl	$36, %ebx
	cmpl	%esi, %ebx
	je	.L2273
.L2256:
	movl	req_erase, %eax
	addl	%ebx, %eax
	cmpl	$-1, (%eax)
	je	.L2274
	cmpb	$0, -15(%ebp)
	je	.L2255
	movl	4(%eax), %eax
	movl	$1, %edx
	addl	$36, %ebx
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlFreeSysBlkQueueIn
	cmpl	%esi, %ebx
	jne	.L2256
.L2273:
	movzwl	-34(%ebp), %eax
	addw	%ax, -26(%ebp)
	movzwl	-28(%ebp), %edx
	cmpw	%dx, -26(%ebp)
	jb	.L2257
	cmpw	$63, -18(%ebp)
	ja	.L2258
	movl	req_erase, %eax
	movl	%edi, %edx
	call	FlashEraseBlocks
	jmp	.L2258
	.p2align 4,,7
	.p2align 3
.L2274:
	movl	4(%eax), %eax
	addw	$1, -20(%ebp)
	shrl	$10, %eax
	movzwl	%ax, %eax
	call	FtlBbmMapBadBlock
	jmp	.L2255
.L2271:
	cmpb	$0, -15(%ebp)
	je	.L2261
	movzwl	c_ftl_nand_page_pre_slc_blk, %eax
	movw	%ax, -28(%ebp)
	movzwl	%ax, %eax
	imull	$52429, %eax, %eax
	shrl	$18, %eax
	subl	$1, %eax
	movw	%ax, -34(%ebp)
	movl	$1, %eax
	jmp	.L2247
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.p2align 4,,15
	.globl	FlashTestBlk
	.type	FlashTestBlk, @function
FlashTestBlk:
	pushl	%ebp
	movzwl	%ax, %eax
	movl	%esp, %ebp
	pushl	%ebx
	xorl	%ebx, %ebx
	subl	$40, %esp
	cmpl	gNandFlashIdbBlockAddr, %eax
	jb	.L2276
.L2277:
	movl	gFlashPageBuffer1, %edx
	sall	$10, %eax
	xorl	%ebx, %ebx
	movl	$0, -28(%ebp)
	movl	%edx, -32(%ebp)
	movl	$1515870810, (%edx)
	movl	$1515870810, 4(%edx)
	movl	$1, %edx
	movl	%eax, -36(%ebp)
	leal	-40(%ebp), %eax
	call	FlashEraseBlocks
	movl	$1, %edx
	movl	$1, %ecx
	leal	-40(%ebp), %eax
	movl	$1, (%esp)
	call	FlashProgPages
	movl	$1, %edx
	cmpl	$-1, -40(%ebp)
	leal	-40(%ebp), %eax
	setne	%bl
	subl	$1, %ebx
	call	FlashEraseBlocks
.L2276:
	addl	$40, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	FlashTestBlk, .-FlashTestBlk
	.section	.rodata.str1.1
.LC106:
	.string	"\0013bad block:%d %d\n"
.LC107:
	.string	"\0013E:bad block:%d\n"
.LC108:
	.string	"\0013FMFB:Save %d %d\n"
	.text
	.p2align 4,,15
	.globl	FlashMakeFactorBbt
	.type	FlashMakeFactorBbt, @function
FlashMakeFactorBbt:
	pushl	%ebp
	xorl	%edx, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$92, %esp
	movl	gFlashSpareBuffer, %eax
	movzbl	gNandParaInfo+9, %ecx
	movl	%eax, -56(%ebp)
	movl	gpNandParaInfo, %eax
	addl	%ecx, %ecx
	movzbl	13(%eax), %edi
	imulw	14(%eax), %di
	movzbl	24(%eax), %esi
	movl	-56(%ebp), %eax
	call	memset
	cmpb	$0, gNandMaxDie
	je	.L2280
	movl	%esi, %edx
	movl	%edi, %eax
	movzbl	%dl, %esi
	xorl	%ebx, %ebx
	shrw	$4, %ax
	movl	%esi, %ecx
	movzwl	%ax, %eax
	andl	$1, %ecx
	movl	%eax, -68(%ebp)
	movl	%esi, -60(%ebp)
	movl	%ecx, -64(%ebp)
	.p2align 4,,7
	.p2align 3
.L2301:
	movzbl	%bl, %esi
	cmpw	$0, FbbtBlk(%esi,%esi)
	je	.L2314
.L2282:
	addl	$1, %ebx
	cmpb	%bl, gNandMaxDie
	ja	.L2301
.L2280:
	addl	$92, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2314:
	movzbl	gNandParaInfo+9, %ecx
	xorl	%edx, %edx
	movl	gFlashPageBuffer0, %eax
	sall	$9, %ecx
	call	memset
	testw	%di, %di
	movzbl	DieCsIndex(%esi), %eax
	je	.L2302
	movl	-60(%ebp), %edx
	movzbl	%al, %eax
	movl	-60(%ebp), %ecx
	movb	%bl, -89(%ebp)
	movw	$0, -72(%ebp)
	andl	$2, %edx
	movl	%edx, -80(%ebp)
	andl	$4, %ecx
	xorl	%edx, %edx
	movl	%ecx, -84(%ebp)
	movl	%edx, %ebx
	movl	%eax, -88(%ebp)
	jmp	.L2291
	.p2align 4,,7
	.p2align 3
.L2284:
	movl	-80(%ebp), %ecx
	testl	%ecx, %ecx
	jne	.L2315
.L2285:
	movl	gpNandParaInfo, %eax
	movzbl	7(%eax), %eax
	cmpb	$8, %al
	je	.L2306
	cmpb	$1, %al
	jne	.L2286
.L2306:
	cmpb	$0, -14(%ebp)
	movl	$1, %eax
	je	.L2288
	xorl	%eax, %eax
	cmpb	$0, -13(%ebp)
	sete	%al
.L2288:
	movl	-84(%ebp), %edx
	testl	%edx, %edx
	jne	.L2316
.L2289:
	testl	%eax, %eax
	jne	.L2317
.L2290:
	addl	$1, %ebx
	cmpw	%bx, %di
	jbe	.L2318
.L2291:
	movl	-64(%ebp), %eax
	movw	%bx, -76(%ebp)
	movb	$-1, -14(%ebp)
	movb	$-1, -13(%ebp)
	testl	%eax, %eax
	je	.L2284
	movl	gBlockPageAlignSize, %edx
	leal	-14(%ebp), %ecx
	movl	-88(%ebp), %eax
	imull	%ebx, %edx
	addl	DieAddrs(,%esi,4), %edx
	call	FlashReadSpare
	jmp	.L2284
.L2286:
	cmpb	$-1, -14(%ebp)
	movl	$1, %eax
	jne	.L2288
	xorl	%eax, %eax
	cmpb	$-1, -13(%ebp)
	setne	%al
	jmp	.L2288
.L2316:
	movl	gBlockPageAlignSize, %edx
	movl	-88(%ebp), %eax
	imull	%ebx, %edx
	addl	DieAddrs(,%esi,4), %edx
	call	SandiskProgTestBadBlock
	jmp	.L2289
.L2315:
	movl	gpNandParaInfo, %eax
	leal	-13(%ebp), %ecx
	movl	gBlockPageAlignSize, %edx
	movzwl	10(%eax), %eax
	imull	%ebx, %edx
	addl	DieAddrs(,%esi,4), %eax
	leal	-1(%eax,%edx), %edx
	movl	-88(%ebp), %eax
	call	FlashReadSpare
	jmp	.L2285
.L2318:
	movzbl	gNandMaxDie, %eax
	movzwl	-72(%ebp), %edx
	imull	-68(%ebp), %eax
	movzbl	-89(%ebp), %ebx
	cmpl	%eax, %edx
	jge	.L2302
.L2292:
	testb	%bl, %bl
	je	.L2293
	movzwl	-72(%ebp), %eax
	movl	gNandFlashIdbBlockAddr, %edx
	movl	%eax, -76(%ebp)
	movzbl	gNandIDBResBlkNum, %eax
	movl	%edx, -72(%ebp)
.L2294:
	movzbl	%al, %eax
	subl	-72(%ebp), %eax
	cmpl	-76(%ebp), %eax
	ja	.L2297
	movzbl	gNandParaInfo+9, %ecx
	xorl	%edx, %edx
	movl	gFlashPageBuffer0, %eax
	sall	$9, %ecx
	call	memset
.L2297:
	leal	-1(%edi), %eax
	movzbl	%bl, %edx
	movw	%ax, -72(%ebp)
	movzwl	%di, %eax
	imull	%edx, %eax
	movw	%di, -80(%ebp)
	movl	%eax, -76(%ebp)
.L2298:
	movzwl	-72(%ebp), %edi
	movl	%esi, 4(%esp)
	movl	$.LC108, (%esp)
	movl	%edi, 8(%esp)
	call	printk
	movzwl	-72(%ebp), %eax
	movl	%edi, %ecx
	movl	gFlashPageBuffer0, %edx
	shrw	$5, %ax
	movzwl	%ax, %eax
	movl	(%edx,%eax,4), %eax
	shrl	%cl, %eax
	testb	$1, %al
	je	.L2299
	movzwl	-72(%ebp), %edi
.L2307:
	subl	$1, %edi
	movl	%edi, %eax
	movzwl	%di, %ecx
	shrw	$5, %ax
	movzwl	%ax, %eax
	movl	(%edx,%eax,4), %eax
	shrl	%cl, %eax
	testb	$1, %al
	jne	.L2307
	movw	%di, -72(%ebp)
	movl	%ecx, %edi
.L2299:
	movzwl	-72(%ebp), %eax
	movl	-56(%ebp), %edx
	addl	-76(%ebp), %edi
	movw	%ax, FbbtBlk(%esi,%esi)
	movw	$-3872, (%edx)
	sall	$10, %edi
	movw	%ax, 2(%edx)
	movl	gFlashPageBuffer0, %eax
	movl	$1, %edx
	movl	%edi, -48(%ebp)
	movl	%eax, -44(%ebp)
	movl	gFlashSpareBuffer, %eax
	movl	%eax, -40(%ebp)
	leal	-52(%ebp), %eax
	call	FlashEraseBlocks
	movl	$1, %ecx
	movl	$1, %edx
	leal	-52(%ebp), %eax
	movl	$1, (%esp)
	call	FlashProgPages
	movl	-52(%ebp), %edi
	testl	%edi, %edi
	jne	.L2319
	movzwl	-80(%ebp), %edi
	jmp	.L2282
.L2293:
	movl	gNandFlashIdbBlockAddr, %ecx
	movzbl	gNandIDBResBlkNum, %eax
	movl	$0, -76(%ebp)
	movl	%ecx, %edx
	movl	%ecx, -72(%ebp)
	movzbl	%al, %ecx
	cmpw	%cx, %dx
	jae	.L2294
	movw	%di, -80(%ebp)
	movl	%esi, %edi
	movl	%edx, %esi
	movw	$0, -72(%ebp)
.L2296:
	movzwl	%si, %ecx
	movl	%ecx, %eax
	movl	%ecx, -76(%ebp)
	call	FlashTestBlk
	testl	%eax, %eax
	jne	.L2320
.L2295:
	movzbl	gNandIDBResBlkNum, %eax
	addl	$1, %esi
	movzbl	%al, %edx
	cmpw	%si, %dx
	ja	.L2296
	movzwl	-72(%ebp), %edx
	movl	%edi, %esi
	movl	gNandFlashIdbBlockAddr, %ecx
	movzwl	-80(%ebp), %edi
	movl	%edx, -76(%ebp)
	movl	%ecx, -72(%ebp)
	jmp	.L2294
.L2317:
	movl	%ebx, 8(%esp)
	movl	%esi, 4(%esp)
	movl	$.LC106, (%esp)
	call	printk
	movzwl	-76(%ebp), %eax
	movl	$1, %edx
	movl	gFlashPageBuffer0, %ecx
	addw	$1, -72(%ebp)
	shrw	$5, %ax
	movzwl	%ax, %eax
	leal	(%ecx,%eax,4), %eax
	movl	%ebx, %ecx
	sall	%cl, %edx
	orl	%edx, (%eax)
	movzbl	gNandMaxDie, %eax
	movzwl	-72(%ebp), %edx
	imull	-68(%ebp), %eax
	cmpl	%eax, %edx
	jle	.L2290
	movzbl	-89(%ebp), %ebx
.L2302:
	movzbl	gNandParaInfo+9, %ecx
	xorl	%edx, %edx
	movl	gFlashPageBuffer0, %eax
	sall	$9, %ecx
	call	memset
	movw	$0, -72(%ebp)
	jmp	.L2292
.L2320:
	movl	-76(%ebp), %ecx
	movl	$.LC107, (%esp)
	movl	%ecx, 4(%esp)
	call	printk
	movl	gFlashPageBuffer0, %edx
	movl	%esi, %eax
	movl	-76(%ebp), %ecx
	shrw	$5, %ax
	movzwl	%ax, %eax
	addw	$1, -72(%ebp)
	leal	(%edx,%eax,4), %eax
	movl	$1, %edx
	sall	%cl, %edx
	orl	%edx, (%eax)
	jmp	.L2295
.L2319:
	subw	$1, -72(%ebp)
	jmp	.L2298
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.p2align 4,,15
	.globl	FtlWriteCacheEn
	.type	FtlWriteCacheEn, @function
FtlWriteCacheEn:
	pushl	%ebp
	xorl	%eax, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	FtlWriteCacheEn, .-FtlWriteCacheEn
	.p2align 4,,15
	.globl	NandRead
	.type	NandRead, @function
NandRead:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	%ecx, (%esp)
	movl	%edx, %ecx
	movl	%eax, %edx
	xorl	%eax, %eax
	call	FtlRead
	leave
	ret
	.size	NandRead, .-NandRead
	.p2align 4,,15
	.globl	NandWrite
	.type	NandWrite, @function
NandWrite:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	%ecx, (%esp)
	movl	%edx, %ecx
	movl	%eax, %edx
	xorl	%eax, %eax
	call	FtlWrite
	leave
	ret
	.size	NandWrite, .-NandWrite
	.p2align 4,,15
	.globl	NandCacheWriteBack
	.type	NandCacheWriteBack, @function
NandCacheWriteBack:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlCacheWriteBack
	popl	%ebp
	ret
	.size	NandCacheWriteBack, .-NandCacheWriteBack
	.p2align 4,,15
	.globl	NandDiscard
	.type	NandDiscard, @function
NandDiscard:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlDiscard
	popl	%ebp
	ret
	.size	NandDiscard, .-NandDiscard
	.p2align 4,,15
	.globl	NandWriteImage
	.type	NandWriteImage, @function
NandWriteImage:
	pushl	%ebp
	movl	%esp, %ebp
	call	NandWrite
	popl	%ebp
	ret
	.size	NandWriteImage, .-NandWriteImage
	.p2align 4,,15
	.globl	Nand_resume
	.type	Nand_resume, @function
Nand_resume:
	pushl	%ebp
	movl	%esp, %ebp
	call	FlashResume
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	Nand_resume, .-Nand_resume
	.p2align 4,,15
	.globl	Nand_suspend
	.type	Nand_suspend, @function
Nand_suspend:
	pushl	%ebp
	movl	%esp, %ebp
	call	FlashSuspend
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	Nand_suspend, .-Nand_suspend
	.p2align 4,,15
	.globl	RKFTL_DeInit
	.type	RKFTL_DeInit, @function
RKFTL_DeInit:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlDeInit
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	RKFTL_DeInit, .-RKFTL_DeInit
	.p2align 4,,15
	.globl	NandReInit
	.type	NandReInit, @function
NandReInit:
	pushl	%ebp
	xorl	%eax, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	NandReInit, .-NandReInit
	.p2align 4,,15
	.globl	NandDeInit
	.type	NandDeInit, @function
NandDeInit:
	pushl	%ebp
	movl	%esp, %ebp
	call	FlashDeInit
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	NandDeInit, .-NandDeInit
	.p2align 4,,15
	.globl	NandGetCapacity
	.type	NandGetCapacity, @function
NandGetCapacity:
	pushl	%ebp
	movl	g_MaxLbaSector, %eax
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	NandGetCapacity, .-NandGetCapacity
	.section	.rodata.str1.1
.LC109:
	.string	"\0013%s 0x%x:"
.LC110:
	.string	"\0013%x "
.LC111:
	.string	""
	.text
	.p2align 4,,15
	.globl	rknand_print_hex
	.type	rknand_print_hex, @function
rknand_print_hex:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$24, %esp
	movl	%eax, -24(%ebp)
	movl	8(%ebp), %eax
	movl	%edx, -20(%ebp)
	movl	%ecx, -16(%ebp)
	testl	%eax, %eax
	je	.L2334
	movl	%edx, %edi
	xorl	%ebx, %ebx
	xorl	%esi, %esi
	jmp	.L2340
	.p2align 4,,7
	.p2align 3
.L2338:
	movsbl	(%edi), %eax
	movl	$.LC110, (%esp)
	movl	%eax, 4(%esp)
	call	printk
.L2337:
	addl	$1, %ebx
	cmpl	$15, %ebx
	ja	.L2342
.L2339:
	addl	$1, %esi
	addl	$1, %edi
	cmpl	8(%ebp), %esi
	je	.L2334
.L2340:
	testl	%ebx, %ebx
	je	.L2343
.L2335:
	cmpl	$4, -16(%ebp)
	je	.L2344
	cmpl	$2, -16(%ebp)
	jne	.L2338
	movl	-20(%ebp), %edx
	movswl	(%edx,%esi,2), %eax
	movl	$.LC110, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L2337
	.p2align 4,,7
	.p2align 3
.L2334:
	movl	$.LC111, 4(%esp)
	movl	$.LC88, (%esp)
	call	printk
	addl	$24, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2343:
	movl	-24(%ebp), %eax
	movl	%edi, 8(%esp)
	movl	$.LC109, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L2335
.L2342:
	movl	$.LC111, 4(%esp)
	xorl	%ebx, %ebx
	movl	$.LC88, (%esp)
	call	printk
	jmp	.L2339
.L2344:
	movl	-20(%ebp), %edx
	movl	(%edx,%esi,4), %eax
	movl	$.LC110, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L2337
	.size	rknand_print_hex, .-rknand_print_hex
	.section	.rodata.str1.1
.LC112:
	.string	"BBT:"
	.text
	.p2align 4,,15
	.globl	FlashReadFacBbtData
	.type	FlashReadFacBbtData, @function
FlashReadFacBbtData:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%edx, %esi
	pushl	%ebx
	subl	$16, %esp
	movl	%eax, -20(%ebp)
	movl	gpNandParaInfo, %eax
	movl	%ecx, -24(%ebp)
	movzbl	DieCsIndex(%edx), %ecx
	movzbl	13(%eax), %edi
	imulw	14(%eax), %di
	movl	$-1, %eax
	leal	-1(%edi), %ebx
	movzwl	%di, %edi
	subl	$16, %edi
	movzwl	%bx, %edx
	cmpl	%edi, %edx
	jle	.L2346
	movzbl	%cl, %ecx
	movl	%ecx, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L2350:
	movl	gFlashSpareBuffer, %eax
	imull	gBlockPageAlignSize, %edx
	movl	gFlashPageBuffer0, %ecx
	addl	DieAddrs(,%esi,4), %edx
	movl	%eax, (%esp)
	movl	-16(%ebp), %eax
	call	FlashReadPage
	cmpl	$-1, %eax
	je	.L2347
	movl	gFlashSpareBuffer, %eax
	cmpw	$-3872, (%eax)
	je	.L2355
.L2347:
	subl	$1, %ebx
	movzwl	%bx, %edx
	cmpl	%edi, %edx
	jg	.L2350
	movl	$-1, %eax
.L2346:
	addl	$16, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2355:
	movl	-20(%ebp), %edx
	xorl	%eax, %eax
	testl	%edx, %edx
	je	.L2346
	testl	%esi, %esi
	jne	.L2348
	movl	gNandFlashIdbBlockAddr, %edi
	testl	%edi, %edi
	je	.L2348
	xorl	%edx, %edx
	xorl	%ecx, %ecx
	movl	$1, %ebx
	.p2align 4,,7
	.p2align 3
.L2349:
	movl	gFlashPageBuffer0, %esi
	movl	%edx, %eax
	addl	$1, %edx
	shrw	$5, %ax
	movzwl	%ax, %eax
	leal	(%esi,%eax,4), %eax
	movl	%ebx, %esi
	sall	%cl, %esi
	movzwl	%dx, %ecx
	orl	%esi, (%eax)
	cmpl	gNandFlashIdbBlockAddr, %ecx
	jb	.L2349
.L2348:
	movl	-24(%ebp), %ecx
	movl	gFlashPageBuffer0, %edx
	movl	-20(%ebp), %eax
	call	memcpy
	movl	-20(%ebp), %edx
	movl	$.LC112, %eax
	movl	$4, %ecx
	movl	$4, (%esp)
	call	rknand_print_hex
	addl	$16, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.p2align 4,,15
	.globl	FlashGetBadBlockList
	.type	FlashGetBadBlockList, @function
FlashGetBadBlockList:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	movl	%eax, -24(%ebp)
	movl	gpNandParaInfo, %eax
	movzbl	13(%eax), %ebx
	imulw	14(%eax), %bx
	movl	gFlashPageBuffer1, %eax
	movzwl	%bx, %esi
	leal	7(%esi), %ecx
	sarl	$3, %ecx
	call	FlashReadFacBbtData
	xorl	%edx, %edx
	cmpl	$-1, %eax
	je	.L2357
	subl	$1, %esi
	testl	%esi, %esi
	movl	%esi, -16(%ebp)
	jle	.L2357
	shrw	$4, %bx
	xorl	%ecx, %ecx
	movw	%bx, -18(%ebp)
	xorl	%eax, %eax
	movl	$1, %esi
	jmp	.L2359
	.p2align 4,,7
	.p2align 3
.L2365:
	addl	$1, %eax
	movzwl	%ax, %ecx
	cmpl	-16(%ebp), %ecx
	jge	.L2364
.L2359:
	movl	%esi, %edi
	movl	%eax, %ebx
	sall	%cl, %edi
	movl	gFlashPageBuffer1, %ecx
	shrw	$5, %bx
	movzwl	%bx, %ebx
	testl	%edi, (%ecx,%ebx,4)
	je	.L2358
	movl	-24(%ebp), %ebx
	movzwl	%dx, %ecx
	addl	$1, %edx
	movw	%ax, (%ebx,%ecx,2)
.L2358:
	cmpw	-18(%ebp), %dx
	jb	.L2365
	xorl	%edx, %edx
.L2357:
	movl	-24(%ebp), %eax
	movw	$-1, (%eax,%edx)
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2364:
	movzwl	%dx, %edx
	addl	%edx, %edx
	jmp	.L2357
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.p2align 4,,15
	.globl	FtlMakeBbt
	.type	FtlMakeBbt, @function
FtlMakeBbt:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$12, %esp
	call	FtlBbtMemInit
	call	FtlLoadFactoryBbt
	cmpw	$0, c_ftl_nand_die_num
	je	.L2367
	xorl	%ebx, %ebx
	jmp	.L2373
	.p2align 4,,7
	.p2align 3
.L2387:
	movzwl	c_ftl_nand_blks_per_die, %eax
	movzwl	%di, %edi
	movl	$1, %ecx
	movl	$1, %edx
	imull	%ebx, %eax
	addl	%eax, %edi
	movl	%edi, %eax
	sall	$10, %eax
	movl	%eax, req_sys+4
	movl	$req_sys, %eax
	call	FlashReadPages
	movzwl	c_ftl_nand_blks_per_die, %ecx
	movl	gBbtInfo+28(,%ebx,4), %eax
	movl	req_sys+8, %edx
	addl	$7, %ecx
	sarl	$3, %ecx
	call	memcpy
.L2369:
	movzwl	%di, %eax
	addl	$1, %ebx
	call	FtlBbmMapBadBlock
	movzwl	c_ftl_nand_die_num, %eax
	cmpl	%ebx, %eax
	jbe	.L2367
.L2373:
	movzwl	gBbtInfo+12(%ebx,%ebx), %edi
	movl	p_sys_data_buf, %eax
	movl	p_sys_spare_buf, %esi
	cmpw	$-1, %di
	movl	%eax, req_sys+8
	movl	%esi, req_sys+12
	jne	.L2387
	movl	%ebx, %edx
	call	FlashGetBadBlockList
	movl	req_sys+8, %eax
	movl	gBbtInfo+28(,%ebx,4), %edx
	call	FtlBbt2Bitmap
	movzwl	c_ftl_nand_blks_per_die, %eax
	movw	%bx, -18(%ebp)
	movw	%ax, -14(%ebp)
	subl	$1, %eax
	movw	%ax, -16(%ebp)
	.p2align 4,,7
	.p2align 3
.L2385:
	movzwl	-14(%ebp), %eax
	xorl	%edx, %edx
	imulw	-18(%ebp), %ax
	addw	-16(%ebp), %ax
	divw	-14(%ebp)
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %edi
	movl	%edx, %ecx
	shrl	%cl, %edi
	andl	$1, %edi
	jne	.L2371
	movzwl	-16(%ebp), %eax
	movw	%ax, gBbtInfo+12(%ebx,%ebx)
	movw	$-3872, (%esi)
	movzwl	c_ftl_nand_blks_per_die, %edi
	movl	$0, 4(%esi)
	movzwl	gBbtInfo+12(%ebx,%ebx), %eax
	movzwl	c_ftl_nand_bbm_buf_size, %ecx
	imull	%ebx, %edi
	movw	%ax, 2(%esi)
	movzwl	gBbtInfo+12(%ebx,%ebx), %eax
	movl	gBbtInfo+28(,%ebx,4), %edx
	sall	$2, %ecx
	addl	%eax, %edi
	movl	%edi, %eax
	sall	$10, %eax
	movl	%eax, req_sys+4
	movl	req_sys+8, %eax
	call	memcpy
	movl	$1, %edx
	movl	$req_sys, %eax
	call	FlashEraseBlocks
	movl	$1, %ecx
	movl	$1, %edx
	movl	$req_sys, %eax
	movl	$1, (%esp)
	call	FlashProgPages
	cmpl	$-1, req_sys
	jne	.L2369
	movzwl	%di, %eax
	call	FtlBbmMapBadBlock
	movzwl	c_ftl_nand_blks_per_die, %ecx
	movw	%cx, -14(%ebp)
	jmp	.L2385
	.p2align 4,,7
	.p2align 3
.L2371:
	subw	$1, -16(%ebp)
	jmp	.L2385
	.p2align 4,,7
	.p2align 3
.L2367:
	cmpw	$0, c_ftl_nand_reserved_blks
	je	.L2374
	xorl	%ebx, %ebx
	.p2align 4,,7
	.p2align 3
.L2375:
	movzwl	%bx, %eax
	addl	$1, %ebx
	call	FtlBbmMapBadBlock
	cmpw	%bx, c_ftl_nand_reserved_blks
	ja	.L2375
.L2374:
	movzwl	gBbtInfo+12, %eax
	leal	-1(%eax), %ebx
	subl	$48, %eax
	movzwl	%bx, %esi
	cmpl	%eax, %esi
	jg	.L2381
	jmp	.L2376
	.p2align 4,,7
	.p2align 3
.L2378:
	cmpw	$-1, gBbtInfo
	jne	.L2379
	movw	%bx, gBbtInfo
.L2377:
	movzwl	gBbtInfo+12, %eax
	subl	$1, %ebx
	movzwl	%bx, %esi
	subl	$48, %eax
	cmpl	%eax, %esi
	jle	.L2376
.L2381:
	xorl	%edx, %edx
	movl	%ebx, %eax
	divw	c_ftl_nand_blks_per_die
	movzwl	%ax, %eax
	movl	%edx, %ecx
	movl	gBbtInfo+28(,%eax,4), %eax
	shrw	$5, %cx
	movzwl	%cx, %ecx
	movl	(%eax,%ecx,4), %eax
	movl	%edx, %ecx
	shrl	%cl, %eax
	testb	$1, %al
	jne	.L2377
	movl	%esi, %eax
	call	FlashTestBlk
	testl	%eax, %eax
	je	.L2378
	movl	%esi, %eax
	call	FtlBbmMapBadBlock
	.p2align 4,,4
	jmp	.L2377
.L2379:
	movw	%bx, gBbtInfo+4
.L2376:
	movzwl	gBbtInfo, %edx
	movl	req_erase, %eax
	movl	$0, gBbtInfo+8
	movw	$0, gBbtInfo+2
	sall	$10, %edx
	movl	%edx, 4(%eax)
	movzwl	gBbtInfo+4, %edx
	sall	$10, %edx
	movl	%edx, 40(%eax)
	movl	$2, %edx
	call	FlashEraseBlocks
	movzwl	gBbtInfo, %eax
	call	FtlBbmMapBadBlock
	movzwl	gBbtInfo+4, %eax
	call	FtlBbmMapBadBlock
	call	FtlBbmTblFlush
	movzwl	gBbtInfo, %eax
	movzwl	gBbtInfo+4, %edx
	addl	$1, gBbtInfo+8
	movw	$0, gBbtInfo+2
	movw	%dx, gBbtInfo
	movw	%ax, gBbtInfo+4
	call	FtlBbmTblFlush
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	FtlMakeBbt, .-FtlMakeBbt
	.p2align 4,,15
	.globl	FtlLowFormat
	.type	FtlLowFormat, @function
FtlLowFormat:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$28, %esp
	movzwl	c_ftl_nand_max_sys_blks, %eax
	movl	$0, g_GlobalSysVersion
	movl	$0, g_GlobalDataVersion
	call	FtlFreeSysBlkQueueInit
	call	FtlLoadBbt
	testl	%eax, %eax
	jne	.L2419
.L2389:
	cmpw	$0, c_ftl_nand_sec_pre_page
	je	.L2390
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	xorl	%eax, %eax
	.p2align 4,,7
	.p2align 3
.L2391:
	movl	p_io_data_buf_0, %ecx
	notl	%edx
	addl	$1, %ebx
	sall	$16, %edx
	orl	%eax, %edx
	movl	%edx, (%ecx,%eax,4)
	movl	p_io_data_buf_1, %edx
	movl	$168778952, (%edx,%eax,4)
	movzwl	c_ftl_nand_sec_pre_page, %ecx
	movzwl	%bx, %eax
	movl	%eax, %edx
	sall	$7, %ecx
	cmpl	%ecx, %eax
	jl	.L2391
.L2390:
	movzwl	c_ftl_nand_data_blks_per_plane, %ebx
	cmpw	c_ftl_nand_blk_pre_plane, %bx
	movl	%ebx, %edi
	jae	.L2397
	xorl	%esi, %esi
	.p2align 4,,7
	.p2align 3
.L2393:
	movzwl	%bx, %eax
	movl	$1, %edx
	call	FtlLowFormatEraseBlock
	addl	$1, %ebx
	addl	%eax, %esi
	cmpw	%bx, c_ftl_nand_blk_pre_plane
	ja	.L2393
	movzwl	c_ftl_nand_planes_num, %ecx
	movzwl	%si, %eax
	subl	$3, %eax
	movzwl	%cx, %edx
	addl	%edx, %edx
	cmpl	%eax, %edx
	jle	.L2394
.L2417:
	movzwl	c_ftl_nand_data_blks_per_plane, %edi
.L2397:
	xorl	%esi, %esi
	xorl	%ebx, %ebx
	testw	%di, %di
	je	.L2420
	.p2align 4,,7
	.p2align 3
.L2410:
	movzwl	%bx, %eax
	xorl	%edx, %edx
	call	FtlLowFormatEraseBlock
	addl	$1, %ebx
	addl	%eax, %esi
	cmpw	%bx, c_ftl_nand_data_blks_per_plane
	ja	.L2410
	movzwl	%si, %eax
	movl	%eax, -24(%ebp)
.L2396:
	movzwl	c_ftl_nand_blk_pre_plane, %eax
	movzwl	c_ftl_nand_planes_num, %edx
	movl	%eax, g_cur_erase_blk
	movl	c_ftl_nand_max_data_blks, %eax
	movzwl	%dx, %ebx
	movw	%dx, -18(%ebp)
	xorl	%edx, %edx
	movl	%eax, -16(%ebp)
	divl	%ebx
	movl	%eax, %edi
	movl	%eax, g_MaxLpn
	shrl	$5, %eax
	movl	%eax, %ecx
	movw	%ax, -40(%ebp)
	leal	(%ebx,%ebx,2), %eax
	addl	$36, %ecx
	sall	$3, %eax
	cmpl	-24(%ebp), %eax
	movw	%cx, c_ftl_nand_data_op_blks_per_plane
	jge	.L2399
	movl	-16(%ebp), %edx
	movzwl	%si, %eax
	subl	%eax, %edx
	movl	%edx, %eax
	xorl	%edx, %edx
	divl	%ebx
	movl	%eax, g_MaxLpn
	shrl	$5, %eax
	leal	24(%eax), %ecx
	movw	%cx, c_ftl_nand_data_op_blks_per_plane
.L2399:
	cmpl	$1, g_inkDie_check_enable
	je	.L2421
.L2400:
	movzwl	c_ftl_nand_ext_blk_pre_plane, %eax
	testw	%ax, %ax
	je	.L2418
	movl	%eax, %edx
	shrw	%dx
	addl	%edx, %ecx
	movzwl	%ax, %edx
	imull	%ebx, %edx
	cmpl	-24(%ebp), %edx
	movw	%cx, c_ftl_nand_data_op_blks_per_plane
	jg	.L2403
.L2418:
	movl	g_MaxLpn, %edi
.L2402:
	movzwl	%cx, %ecx
	movzwl	c_ftl_nand_page_pre_blk, %eax
	subl	%ecx, %edi
	movzwl	c_ftl_nand_sec_pre_page_shift, %ecx
	imull	%ebx, %edi
	movl	%edi, g_MaxLbn
	imull	%eax, %edi
	movl	%edi, g_MaxLpn
	sall	%cl, %edi
	movl	%edi, g_MaxLbaSector
	call	FtlBbmTblFlush
	movzwl	c_ftl_nand_blk_pre_plane, %ecx
	xorl	%edx, %edx
	movl	p_valid_page_count_table, %eax
	addl	%ecx, %ecx
	call	memset
	movl	$0, g_validLpn
	movw	$-1, g_gc_superblock
	movw	$0, g_gc_superblock+2
	movb	$0, g_gc_superblock+6
	movb	$0, g_gc_superblock+8
	movw	$0, g_active_superblock+2
	movb	$0, g_active_superblock+6
	movw	$0, g_active_superblock
	movb	$0, g_active_superblock+8
	jmp	.L2404
	.p2align 4,,7
	.p2align 3
.L2422:
	movzwl	g_active_superblock, %edx
	movl	p_valid_page_count_table, %eax
	movw	$-1, (%eax,%edx,2)
	addw	$1, g_active_superblock
.L2404:
	movl	$g_active_superblock, %eax
	call	make_superblock
	cmpb	$0, g_active_superblock+7
	je	.L2422
	movl	g_GlobalSysVersion, %eax
	movzwl	g_active_superblock, %edx
	movzwl	g_active_superblock+4, %ecx
	movl	%eax, g_active_superblock+12
	addl	$1, %eax
	movl	%eax, g_GlobalSysVersion
	movl	p_valid_page_count_table, %eax
	movw	%cx, (%eax,%edx,2)
	movzwl	g_active_superblock, %eax
	movw	$0, g_buffer_superblock+2
	movb	$0, g_buffer_superblock+6
	movb	$1, g_buffer_superblock+8
	addl	$1, %eax
	movw	%ax, g_buffer_superblock
	jmp	.L2406
	.p2align 4,,7
	.p2align 3
.L2423:
	movzwl	g_buffer_superblock, %edx
	movl	p_valid_page_count_table, %eax
	movw	$-1, (%eax,%edx,2)
	addw	$1, g_buffer_superblock
.L2406:
	movl	$g_buffer_superblock, %eax
	call	make_superblock
	cmpb	$0, g_buffer_superblock+7
	je	.L2423
	movl	g_GlobalSysVersion, %eax
	movzwl	g_buffer_superblock, %edx
	movzwl	g_buffer_superblock+4, %ecx
	movl	%eax, g_buffer_superblock+12
	addl	$1, %eax
	movl	%eax, g_GlobalSysVersion
	movl	p_valid_page_count_table, %eax
	movw	%cx, (%eax,%edx,2)
	movw	$-1, g_gc_temp_superblock
	call	FtlFreeSysBlkQueueOut
	movw	$0, gSysInfo+2
	movw	$-1, gSysInfo+4
	movw	%ax, gSysInfo
	movl	g_MaxLbn, %eax
	movw	%ax, gSysInfo+6
	movl	g_GlobalSysVersion, %eax
	movl	%eax, gSysInfo+8
	addl	$1, %eax
	movl	%eax, g_GlobalSysVersion
	call	FtlVpcTblFlush
	call	FtlSysBlkInit
	testl	%eax, %eax
	jne	.L2408
	movl	$1, gFtlInitStatus
.L2408:
	addl	$28, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2403:
	movzwl	-40(%ebp), %edx
	leal	32(%edx,%eax), %ecx
	movw	%cx, c_ftl_nand_data_op_blks_per_plane
	jmp	.L2402
	.p2align 4,,7
	.p2align 3
.L2394:
	xorl	%edx, %edx
	movl	%esi, %eax
	divw	%cx
	addw	c_ftl_nand_init_sys_blks_per_plane, %ax
	movzwl	%ax, %eax
	call	FtlSysBlkNumInit
	movzwl	c_ftl_nand_max_sys_blks, %eax
	call	FtlFreeSysBlkQueueInit
	movzwl	c_ftl_nand_data_blks_per_plane, %edi
	cmpw	c_ftl_nand_blk_pre_plane, %di
	jae	.L2397
	.p2align 4,,7
	.p2align 3
.L2411:
	movzwl	%di, %eax
	movl	$1, %edx
	call	FtlLowFormatEraseBlock
	addl	$1, %edi
	cmpw	%di, c_ftl_nand_blk_pre_plane
	ja	.L2411
	jmp	.L2417
	.p2align 4,,7
	.p2align 3
.L2419:
	call	FtlMakeBbt
	.p2align 4,,8
	jmp	.L2389
	.p2align 4,,7
	.p2align 3
.L2421:
	movzwl	%cx, %eax
	xorl	%edx, %edx
	movl	%eax, -16(%ebp)
	movl	%esi, %eax
	divw	-18(%ebp)
	movzwl	%ax, %eax
	addl	-16(%ebp), %eax
	shrl	$2, %eax
	addl	%eax, %ecx
	movw	%cx, c_ftl_nand_data_op_blks_per_plane
	jmp	.L2400
.L2420:
	movl	$0, -24(%ebp)
	jmp	.L2396
	.size	FtlLowFormat, .-FtlLowFormat
	.section	.rodata.str1.4
	.align 4
.LC113:
	.string	"\0013Read pageadd=%x  ecc=%x err=%x\n"
	.section	.rodata.str1.1
.LC114:
	.string	"spare:"
	.text
	.p2align 4,,15
	.globl	FlashReadPages1
	.type	FlashReadPages1, @function
FlashReadPages1:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	subl	$60, %esp
	movl	%edx, -44(%ebp)
	movzbl	gNandRandomizer, %edx
	movl	%eax, -40(%ebp)
	movzbl	gNandParaInfo+9, %eax
	movl	%ecx, -52(%ebp)
	movb	%dl, -54(%ebp)
	movl	-44(%ebp), %edx
	testl	%edx, %edx
	je	.L2425
	movzbl	-44(%ebp), %ecx
	movzbl	%al, %eax
	movl	$0, -48(%ebp)
	movl	$0, -28(%ebp)
	movl	%eax, -32(%ebp)
	movb	%cl, -53(%ebp)
	jmp	.L2448
	.p2align 4,,7
	.p2align 3
.L2459:
	addl	$1, -28(%ebp)
	movl	-28(%ebp), %ecx
	cmpl	%ecx, -44(%ebp)
	movl	$-1, (%ebx)
	jbe	.L2425
.L2448:
	movl	-28(%ebp), %edx
	movl	-40(%ebp), %ecx
	leal	(%edx,%edx,8), %eax
	leal	(%ecx,%eax,4), %ebx
	movzbl	-53(%ebp), %eax
	leal	-16(%ebp), %ecx
	subb	%dl, %al
	movl	-52(%ebp), %edx
	movzbl	%al, %eax
	movl	%eax, 4(%esp)
	leal	-20(%ebp), %eax
	movl	%eax, (%esp)
	movl	%ebx, %eax
	call	LogAddr2PhyAddr
	movzbl	gNandMaxDie, %edx
	movl	%eax, %edi
	movl	-20(%ebp), %eax
	cmpl	%eax, %edx
	jbe	.L2459
	movzbl	DieCsIndex(%eax), %eax
	movl	$0, %edx
	cmpb	$0, gMultiPageReadEn
	movb	%al, -24(%ebp)
	movzbl	%al, %eax
	cmove	%edx, %edi
	movl	%eax, -36(%ebp)
	call	NandcWaitFlashReady
	movl	gpNandParaInfo, %eax
	movzbl	19(%eax), %eax
	subl	$1, %eax
	cmpb	$4, %al
	ja	.L2429
	movl	-36(%ebp), %edx
	movzbl	gReadRetryInfo+12(%edx), %eax
	cmpb	%al, read_retry_cur_offset(%edx)
	je	.L2429
	movzbl	%al, %eax
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	movl	%eax, (%esp)
	movl	-36(%ebp), %eax
	call	HynixSetRRPara
.L2429:
	movl	gpNandc, %esi
	movl	$1, %edx
	movzbl	-24(%ebp), %ecx
	movl	(%esi), %eax
	sall	%cl, %edx
	cmpb	$-1, %cl
	setne	-24(%ebp)
	movb	%dl, %al
	movl	%eax, (%esi)
	movl	-36(%ebp), %esi
	jmp	.L2430
	.p2align 4,,7
	.p2align 3
.L2457:
	testl	%edi, %edi
	je	.L2433
	movl	gBlockPageAlignSize, %ecx
	movl	%esi, %eax
	addl	%edx, %ecx
	call	FlashReadDpCmd
	movl	%esi, %eax
	call	NandcWaitFlashReady
	movl	-16(%ebp), %edx
	movl	%esi, %eax
	call	FlashReadDpDataOutCmd
.L2431:
	movl	12(%ebx), %eax
	xorl	%edx, %edx
	movl	-32(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	8(%ebx), %eax
	movl	%eax, (%esp)
	movl	%esi, %eax
	call	NandcXferData
	cmpl	$-1, %eax
	jne	.L2434
	cmpb	$0, gNandRandomizer
	je	.L2434
	movb	$0, gNandRandomizer
	xorl	%edi, %edi
.L2430:
	cmpb	$0, -24(%ebp)
	movl	-16(%ebp), %edx
	jne	.L2457
	cmpl	$-1, %edx
	jne	.L2457
	xorl	%edi, %edi
	jmp	.L2431
	.p2align 4,,7
	.p2align 3
.L2433:
	movl	%esi, %eax
	call	FlashReadCmd
	movl	%esi, %eax
	call	NandcWaitFlashReady
	jmp	.L2431
	.p2align 4,,7
	.p2align 3
.L2434:
	testl	%edi, %edi
	movl	%eax, %esi
	jne	.L2460
.L2435:
	movl	gpNandc, %edx
	movl	(%edx), %eax
	andl	$-131328, %eax
	cmpl	$-1, %esi
	movl	%eax, (%edx)
	movzbl	-54(%ebp), %edx
	movb	%dl, gNandRandomizer
	je	.L2461
	movzbl	gNandFlashEccBits, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%esi, %eax
	jae	.L2438
	movl	gpReadRetrial, %eax
	testl	%eax, %eax
	je	.L2462
.L2438:
	cmpl	$256, %esi
	movl	%esi, (%ebx)
	je	.L2443
	cmpl	$-1, %esi
	je	.L2443
	movl	$0, (%ebx)
.L2444:
	testl	%edi, %edi
	je	.L2445
	movzbl	gNandFlashEccBits, %eax
	leal	(%eax,%eax,2), %eax
	sarl	$2, %eax
	cmpl	%eax, -48(%ebp)
	jbe	.L2446
	movl	gpReadRetrial, %ecx
	testl	%ecx, %ecx
	je	.L2447
.L2446:
	movl	-28(%ebp), %edx
	movl	-40(%ebp), %ecx
	leal	9(%edx,%edx,8), %eax
	movl	-48(%ebp), %edx
	leal	(%ecx,%eax,4), %eax
	cmpl	$256, %edx
	movl	%edx, (%eax)
	je	.L2445
	cmpl	$-1, %edx
	je	.L2445
	movl	$0, (%eax)
.L2445:
	addl	%edi, -28(%ebp)
	addl	$1, -28(%ebp)
	movl	-28(%ebp), %ecx
	cmpl	%ecx, -44(%ebp)
	ja	.L2448
.L2425:
	addl	$60, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2462:
	movl	$256, %esi
.L2441:
	movl	%esi, (%ebx)
	.p2align 4,,7
	.p2align 3
.L2443:
	cmpl	$-1, %esi
	jne	.L2444
	movzbl	gNandFlashEccBits, %eax
	movl	$-1, 12(%esp)
	movl	%eax, 8(%esp)
	movl	4(%ebx), %eax
	movl	$.LC113, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	movl	12(%ebx), %edx
	testl	%edx, %edx
	je	.L2444
	movl	$4, %ecx
	movl	$.LC114, %eax
	movl	$4, (%esp)
	call	rknand_print_hex
	jmp	.L2444
	.p2align 4,,7
	.p2align 3
.L2460:
	movl	gBlockPageAlignSize, %edx
	movl	-36(%ebp), %eax
	addl	-16(%ebp), %edx
	call	FlashReadDpDataOutCmd
	movl	-28(%ebp), %edx
	movl	-40(%ebp), %ecx
	leal	9(%edx,%edx,8), %eax
	leal	(%ecx,%eax,4), %eax
	movl	-32(%ebp), %ecx
	movl	12(%eax), %edx
	movl	%edx, 4(%esp)
	movl	8(%eax), %eax
	xorl	%edx, %edx
	movl	%eax, (%esp)
	movl	-36(%ebp), %eax
	call	NandcXferData
	cmpl	$-1, %eax
	movl	%eax, -48(%ebp)
	movl	$0, %eax
	cmove	%eax, %edi
	jmp	.L2435
	.p2align 4,,7
	.p2align 3
.L2461:
	movl	gpReadRetrial, %esi
	movl	12(%ebx), %eax
	movl	8(%ebx), %ecx
	movl	-16(%ebp), %edx
	testl	%esi, %esi
	movl	%eax, (%esp)
	movl	-36(%ebp), %eax
	je	.L2437
	call	*%esi
	cmpl	$-1, %eax
	movl	%eax, %esi
	je	.L2463
	xorl	%edi, %edi
	.p2align 4,,4
	jmp	.L2438
.L2437:
	call	FlashReadRawPage
	movl	%eax, %esi
.L2440:
	xorl	%edi, %edi
	cmpl	$-1, %esi
	jne	.L2438
	cmpb	$0, gFlashToggleModeEn
	je	.L2441
	movl	gpNandc, %eax
	movl	8(%ebx), %ecx
	movl	-16(%ebp), %edx
	movl	304(%eax), %eax
	movl	%eax, -24(%ebp)
	movl	12(%ebx), %eax
	movl	%eax, (%esp)
	movl	-36(%ebp), %eax
	call	FlashDdrTunningRead
	cmpl	$-1, %eax
	movl	%eax, %esi
	je	.L2442
	movzbl	gNandFlashEccBits, %eax
	shrb	%al
	movzbl	%al, %eax
	cmpl	%eax, %esi
	jbe	.L2438
.L2442:
	movl	-24(%ebp), %ecx
	xorl	%edi, %edi
	movzbl	%ch, %eax
	movl	%eax, %edx
	sall	$16, %edx
	sall	$8, %eax
	orl	%eax, %edx
	movl	gpNandc, %eax
	orl	$1, %edx
	movl	%edx, 304(%eax)
	jmp	.L2438
.L2447:
	movl	-28(%ebp), %edx
	movl	-40(%ebp), %ecx
	movl	$256, -48(%ebp)
	leal	9(%edx,%edx,8), %eax
	movl	$256, (%ecx,%eax,4)
	jmp	.L2445
.L2463:
	movl	gpNandParaInfo, %eax
	movzbl	19(%eax), %eax
	subl	$1, %eax
	cmpb	$4, %al
	ja	.L2439
	movzbl	gReadRetryInfo+1, %edx
	movl	$gReadRetryInfo+4, %ecx
	movl	-36(%ebp), %eax
	movl	$0, (%esp)
	call	HynixSetRRPara
.L2439:
	movl	12(%ebx), %eax
	movl	8(%ebx), %ecx
	movl	-16(%ebp), %edx
	movl	%eax, (%esp)
	movl	-36(%ebp), %eax
	call	FlashReadRawPage
	movl	%eax, 12(%esp)
	movl	%eax, %esi
	movzbl	gNandFlashEccBits, %eax
	movl	%eax, 8(%esp)
	movl	4(%ebx), %eax
	movl	$.LC86, (%esp)
	movl	%eax, 4(%esp)
	call	printk
	jmp	.L2440
	.size	FlashReadPages1, .-FlashReadPages1
	.p2align 4,,15
	.globl	nand_spin_lock_init
	.type	nand_spin_lock_init, @function
nand_spin_lock_init:
	pushl	%ebp
	movl	%esp, %ebp
	movw	$0, (%eax)
	popl	%ebp
	ret
	.size	nand_spin_lock_init, .-nand_spin_lock_init
	.p2align 4,,15
	.globl	nand_spin_lock
	.type	nand_spin_lock, @function
nand_spin_lock:
	pushl	%ebp
	movl	%esp, %ebp
	call	_raw_spin_lock_irq
	popl	%ebp
	ret
	.size	nand_spin_lock, .-nand_spin_lock
	.p2align 4,,15
	.globl	nand_spin_unlock
	.type	nand_spin_unlock, @function
nand_spin_unlock:
	pushl	%ebp
	movl	%esp, %ebp
	call	_raw_spin_unlock_irq
	popl	%ebp
	ret
	.size	nand_spin_unlock, .-nand_spin_unlock
	.p2align 4,,15
	.globl	nand_device_lock_init
	.type	nand_device_lock_init, @function
nand_device_lock_init:
	pushl	%ebp
	movl	%esp, %ebp
	movw	$0, g_rk_nand_ops_mutex
	movl	$1, g_rk_nand_ops_mutex+4
	movl	$g_rk_nand_ops_mutex+8, g_rk_nand_ops_mutex+8
	movl	$g_rk_nand_ops_mutex+8, g_rk_nand_ops_mutex+12
	popl	%ebp
	ret
	.size	nand_device_lock_init, .-nand_device_lock_init
	.p2align 4,,15
	.globl	nand_device_lock
	.type	nand_device_lock, @function
nand_device_lock:
	pushl	%ebp
	movl	$g_rk_nand_ops_mutex, %eax
	movl	%esp, %ebp
	call	down
	popl	%ebp
	ret
	.size	nand_device_lock, .-nand_device_lock
	.p2align 4,,15
	.globl	nand_device_trylock
	.type	nand_device_trylock, @function
nand_device_trylock:
	pushl	%ebp
	movl	$g_rk_nand_ops_mutex, %eax
	movl	%esp, %ebp
	call	down_trylock
	popl	%ebp
	ret
	.size	nand_device_trylock, .-nand_device_trylock
	.p2align 4,,15
	.globl	nand_device_unlock
	.type	nand_device_unlock, @function
nand_device_unlock:
	pushl	%ebp
	movl	$g_rk_nand_ops_mutex, %eax
	movl	%esp, %ebp
	call	up
	popl	%ebp
	ret
	.size	nand_device_unlock, .-nand_device_unlock
	.p2align 4,,15
	.type	nand_blktrans_bg_thread, @function
nand_blktrans_bg_thread:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	%eax, %esi
	pushl	%ebx
	movl	$g_nand_bg_info, %eax
	subl	$28, %esp
	xorl	%ebx, %ebx
	call	_raw_spin_lock_irq
	.p2align 4,,7
	.p2align 3
.L2511:
	movl	g_nand_bg_info+16, %eax
	testl	%eax, %eax
	je	.L2473
	cmpl	$1, 12(%eax)
	je	.L2518
.L2473:
	movl	g_nand_bg_info+20, %eax
	testl	%eax, %eax
	je	.L2476
	cmpl	$1, 12(%eax)
	je	.L2519
.L2476:
	movl	g_nand_bg_info+24, %eax
	testl	%eax, %eax
	je	.L2483
	movl	$2, 12(%eax)
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	call	nand_device_lock
	movl	g_nand_bg_info+24, %eax
	movl	16(%eax), %ecx
	movl	4(%eax), %edx
	movl	$1, (%esp)
	movl	(%eax), %eax
	call	nand_transfer
	call	nand_device_unlock
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	movl	g_nand_bg_info+24, %eax
	movl	$3, 12(%eax)
	movl	g_nand_bg_info+24, %ecx
	movl	20(%ecx), %eax
	movl	%eax, g_nand_bg_info+24
	movl	$0, 20(%ecx)
	movl	g_nand_bg_info+28, %edx
	testl	%edx, %edx
	jne	.L2509
	jmp	.L2529
	.p2align 4,,7
	.p2align 3
.L2505:
	movl	%eax, %edx
.L2509:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2505
	movl	%ecx, 20(%edx)
.L2485:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	movl	$1, %ecx
	movl	$3, %edx
	movl	$rk29_buffer_wait, %eax
	movl	$1, rk29_buffer_write_done
	movl	$0, (%esp)
	call	__wake_up
	cmpl	$1, rk29_buffer_wait_read
	je	.L2521
.L2487:
	movl	$g_nand_bg_info, %eax
	xorl	%ebx, %ebx
	call	_raw_spin_lock_irq
.L2492:
	movl	g_nand_bg_info+24, %edi
	testl	%edi, %edi
	jne	.L2511
	jmp	.L2493
	.p2align 4,,7
	.p2align 3
.L2483:
	movl	g_nand_bg_info+20, %eax
	testl	%eax, %eax
	je	.L2522
.L2493:
	movl	56(%esi), %ecx
	testl	%ecx, %ecx
	je	.L2511
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	xorl	%edx, %edx
	movl	$g_nand_bg_info+40, %eax
	call	complete_and_exit
	.p2align 4,,7
	.p2align 3
.L2521:
	movl	nand_bg_do, %ebx
	testl	%ebx, %ebx
	jne	.L2488
	movl	48(%esi), %ecx
	testl	%ecx, %ecx
	jne	.L2488
	leal	-20(%ebp), %eax
	movl	$2, %ebx
	movl	%eax, -20(%ebp)
	movl	%eax, -16(%ebp)
	movl	$0, -32(%ebp)
.L2501:
	leal	-32(%ebp), %edx
	movl	$2, %ecx
	movl	$nand_bg_thread_wait, %eax
	call	prepare_to_wait_event
	movl	nand_bg_do, %edx
	testl	%edx, %edx
	je	.L2523
.L2490:
	leal	-32(%ebp), %edx
	movl	$nand_bg_thread_wait, %eax
	call	finish_wait
.L2488:
	movl	$2, rk29_buffer_wait_read
	jmp	.L2487
	.p2align 4,,7
	.p2align 3
.L2523:
	movl	48(%esi), %eax
	testl	%eax, %eax
	jne	.L2490
	testl	%ebx, %ebx
	je	.L2490
	movl	%ebx, %eax
	call	schedule_timeout
	movl	%eax, %ebx
	.p2align 4,,2
	jmp	.L2501
	.p2align 4,,7
	.p2align 3
.L2518:
	movl	$2, 12(%eax)
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	call	nand_device_lock
	movl	g_nand_bg_info+16, %eax
	movl	16(%eax), %ecx
	movl	4(%eax), %edx
	movl	$0, (%esp)
	movl	(%eax), %eax
	call	nand_transfer
	movl	%eax, %edi
	call	nand_device_unlock
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	testl	%edi, %edi
	movl	g_nand_bg_info+16, %eax
	je	.L2474
	movl	$4, 12(%eax)
.L2475:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	movl	g_nand_bg_info+16, %eax
	addl	$24, %eax
	call	complete
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	jmp	.L2473
	.p2align 4,,7
	.p2align 3
.L2519:
	movl	$2, 12(%eax)
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	call	nand_device_lock
	movl	g_nand_bg_info+20, %eax
	movl	4(%eax), %edx
	movl	16(%eax), %ecx
	movl	$0, (%esp)
	movl	(%eax), %eax
	call	nand_transfer
	movl	%eax, -36(%ebp)
	call	nand_device_unlock
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	movl	g_nand_bg_info+20, %edi
	movl	-36(%ebp), %edx
	movl	20(%edi), %eax
	testl	%edx, %edx
	movl	%eax, g_nand_bg_info+20
	je	.L2477
	movl	$0, 12(%edi)
	movl	$0, 20(%edi)
	movl	g_nand_bg_info+32, %edx
	testl	%edx, %edx
	jne	.L2507
	jmp	.L2530
	.p2align 4,,7
	.p2align 3
.L2503:
	movl	%eax, %edx
.L2507:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2503
.L2517:
	movl	%edi, 20(%edx)
.L2479:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	leal	24(%edi), %eax
	call	complete
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	jmp	.L2476
.L2529:
	movl	%ecx, g_nand_bg_info+28
	jmp	.L2485
.L2477:
	movl	$3, 12(%edi)
	movl	$0, 20(%edi)
	movl	g_nand_bg_info+28, %edx
	testl	%edx, %edx
	jne	.L2508
	jmp	.L2531
	.p2align 4,,7
	.p2align 3
.L2504:
	movl	%eax, %edx
.L2508:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2504
	.p2align 4,,4
	jmp	.L2517
.L2474:
	movl	$3, 12(%eax)
	jmp	.L2475
.L2522:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	movl	nand_bg_do, %eax
	testl	%eax, %eax
	jne	.L2494
	movl	48(%esi), %eax
	testl	%eax, %eax
	je	.L2526
.L2494:
	cmpl	$2, %ebx
	movl	$0, nand_bg_do
	ja	.L2498
.L2528:
	addl	$1, %ebx
.L2499:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	jmp	.L2492
.L2526:
	leal	-20(%ebp), %eax
	movl	$5, %edi
	movl	%eax, -20(%ebp)
	movl	%eax, -16(%ebp)
	movl	$0, -32(%ebp)
.L2502:
	movl	$2, %ecx
	movl	$nand_bg_thread_wait, %eax
	leal	-32(%ebp), %edx
	call	prepare_to_wait_event
	movl	nand_bg_do, %eax
	testl	%eax, %eax
	je	.L2527
.L2496:
	leal	-32(%ebp), %edx
	movl	$nand_bg_thread_wait, %eax
	call	finish_wait
	cmpl	$2, %ebx
	movl	$0, nand_bg_do
	jbe	.L2528
.L2498:
	call	nand_device_lock
	xorl	%ebx, %ebx
	call	FtlCacheWriteBack
	call	nand_device_unlock
	.p2align 4,,3
	jmp	.L2499
.L2527:
	movl	48(%esi), %eax
	testl	%eax, %eax
	.p2align 4,,3
	jne	.L2496
	testl	%edi, %edi
	.p2align 4,,4
	je	.L2496
	movl	%edi, %eax
	call	schedule_timeout
	movl	%eax, %edi
	.p2align 4,,2
	jmp	.L2502
.L2531:
	movl	%edi, g_nand_bg_info+28
	jmp	.L2479
.L2530:
	movl	%edi, g_nand_bg_info+32
	jmp	.L2479
	.size	nand_blktrans_bg_thread, .-nand_blktrans_bg_thread
	.p2align 4,,15
	.globl	nand_invalid_data_in_buf
	.type	nand_invalid_data_in_buf, @function
nand_invalid_data_in_buf:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%edx, %edi
	pushl	%esi
	movl	g_nand_bg_info+20, %esi
	pushl	%ebx
	movl	%eax, %ebx
	testl	%esi, %esi
	je	.L2533
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	leal	24(%esi), %eax
	call	wait_for_completion
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
.L2533:
	movl	g_nand_bg_info+28, %ecx
	testl	%ecx, %ecx
	je	.L2532
	movl	%ecx, %esi
	addl	%ebx, %edi
	.p2align 4,,7
	.p2align 3
.L2546:
	movl	(%ecx), %eax
	cmpl	%ebx, %eax
	jae	.L2552
	addl	4(%ecx), %eax
	cmpl	%eax, %ebx
	jae	.L2537
.L2536:
	movl	g_nand_bg_info+28, %edx
	movl	20(%ecx), %eax
	cmpl	%ecx, %edx
	je	.L2553
	movl	%eax, 20(%esi)
	movl	$0, 20(%ecx)
	movl	g_nand_bg_info+32, %edx
	testl	%edx, %edx
	jne	.L2551
	jmp	.L2556
	.p2align 4,,7
	.p2align 3
.L2548:
	movl	%eax, %edx
.L2551:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2548
	movl	%ecx, 20(%edx)
.L2544:
	movl	20(%esi), %ecx
.L2542:
	testl	%ecx, %ecx
	jne	.L2546
.L2532:
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2552:
	cmpl	%edi, %eax
	jb	.L2536
.L2537:
	movl	%ecx, %esi
	movl	20(%ecx), %ecx
	jmp	.L2542
	.p2align 4,,7
	.p2align 3
.L2553:
	movl	%eax, g_nand_bg_info+28
	movl	$0, 20(%ecx)
	movl	g_nand_bg_info+32, %esi
	testl	%esi, %esi
	jne	.L2550
	jmp	.L2557
	.p2align 4,,7
	.p2align 3
.L2547:
	movl	%eax, %esi
.L2550:
	movl	20(%esi), %eax
	testl	%eax, %eax
	jne	.L2547
	movl	%edx, 20(%esi)
.L2540:
	movl	g_nand_bg_info+28, %ecx
	movl	%ecx, %esi
	jmp	.L2542
.L2556:
	movl	%ecx, g_nand_bg_info+32
	jmp	.L2544
.L2557:
	movl	%ecx, g_nand_bg_info+32
	jmp	.L2540
	.size	nand_invalid_data_in_buf, .-nand_invalid_data_in_buf
	.section	.rodata.str1.1
.LC115:
	.string	"&x->wait"
	.text
	.p2align 4,,15
	.type	nand_bg_add_last_write_req, @function
nand_bg_add_last_write_req:
	pushl	%ebp
	movl	$g_nand_bg_info, %eax
	movl	%esp, %ebp
	pushl	%ebx
	call	_raw_spin_lock_irq
	movl	g_nand_bg_info+36, %ebx
	movl	$1, nand_bg_do
	testl	%ebx, %ebx
	je	.L2559
	movl	$0, g_nand_bg_info+36
	leal	28(%ebx), %eax
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movl	$0, 24(%ebx)
	call	__init_waitqueue_head
	movl	4(%ebx), %edx
	movl	(%ebx), %eax
	call	nand_invalid_data_in_buf
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+24, %edx
	testl	%edx, %edx
	jne	.L2563
	jmp	.L2565
	.p2align 4,,7
	.p2align 3
.L2562:
	movl	%eax, %edx
.L2563:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2562
	movl	%ebx, 20(%edx)
.L2559:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
.L2565:
	movl	%ebx, g_nand_bg_info+24
	jmp	.L2559
	.size	nand_bg_add_last_write_req, .-nand_bg_add_last_write_req
	.p2align 4,,15
	.type	nand_flush, @function
nand_flush:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	$100, %ebx
	call	nand_bg_add_last_write_req
	movl	$g_nand_bg_info, %eax
	movl	$1, nand_bg_do
	call	_raw_spin_lock_irq
	jmp	.L2569
	.p2align 4,,7
	.p2align 3
.L2567:
	movl	$g_nand_bg_info, %eax
	movl	$1, nand_bg_do
	call	_raw_spin_unlock_irq
	movl	$10, %eax
	call	msleep
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	subl	$1, %ebx
	je	.L2568
.L2569:
	movl	g_nand_bg_info+16, %eax
	testl	%eax, %eax
	jne	.L2567
	movl	g_nand_bg_info+20, %eax
	testl	%eax, %eax
	jne	.L2567
	movl	g_nand_bg_info+24, %eax
	testl	%eax, %eax
	jne	.L2567
	.p2align 4,,7
	.p2align 3
.L2568:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	call	nand_device_trylock
	testl	%eax, %eax
	jne	.L2570
	.p2align 4,,6
	call	FtlCacheWriteBack
	.p2align 4,,6
	call	nand_device_unlock
.L2570:
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	.p2align 4,,1
	ret
	.size	nand_flush, .-nand_flush
	.p2align 4,,15
	.globl	nand_copy_req_data_to_buf
	.type	nand_copy_req_data_to_buf, @function
nand_copy_req_data_to_buf:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$12, %esp
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	movl	%edi, -4(%ebp)
	movl	(%eax), %edi
	cmpl	%edx, %edi
	jb	.L2573
	addl	%edx, %ecx
	cmpl	%ecx, %edi
	jae	.L2572
	movl	%edi, %ebx
	subl	%edi, %ecx
	movl	16(%eax), %esi
	subl	%edx, %ebx
	movl	4(%eax), %edx
	movl	%ecx, %edi
	sall	$9, %ebx
	sall	$9, %edi
	addl	8(%ebp), %ebx
	movl	%edx, %eax
	sall	$9, %eax
	cmpl	%edx, %ecx
	cmovle	%edi, %eax
	testl	%eax, %eax
	jne	.L2578
	.p2align 4,,7
	.p2align 3
.L2572:
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2573:
	movl	4(%eax), %ebx
	addl	%edi, %ebx
	cmpl	%ebx, %edx
	jae	.L2572
	movl	%edx, %esi
	subl	%edx, %ebx
	subl	%edi, %esi
	movl	%ebx, %edx
	sall	$9, %esi
	movl	%ecx, %edi
	addl	16(%eax), %esi
	movl	%ebx, %eax
	sall	$9, %eax
	movl	8(%ebp), %ebx
	sall	$9, %edi
	cmpl	%edx, %ecx
	cmovb	%edi, %eax
	testl	%eax, %eax
	je	.L2572
.L2578:
	movl	%eax, %ecx
	movl	%esi, %edx
	movl	%ebx, %eax
	call	memcpy
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	nand_copy_req_data_to_buf, .-nand_copy_req_data_to_buf
	.p2align 4,,15
	.globl	nand_read_data_in_buf
	.type	nand_read_data_in_buf, @function
nand_read_data_in_buf:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%edx, %edi
	pushl	%esi
	movl	%ecx, %esi
	pushl	%ebx
	subl	$8, %esp
	movl	g_nand_bg_info+28, %ebx
	movl	%eax, -16(%ebp)
	testl	%ebx, %ebx
	je	.L2580
	.p2align 4,,7
	.p2align 3
.L2586:
	movl	-16(%ebp), %edx
	movl	%ebx, %eax
	movl	%edi, %ecx
	movl	%esi, (%esp)
	call	nand_copy_req_data_to_buf
	movl	20(%ebx), %ebx
	testl	%ebx, %ebx
	jne	.L2586
.L2580:
	movl	g_nand_bg_info+24, %ebx
	testl	%ebx, %ebx
	je	.L2582
	.p2align 4,,7
	.p2align 3
.L2585:
	movl	-16(%ebp), %edx
	movl	%ebx, %eax
	movl	%edi, %ecx
	movl	%esi, (%esp)
	call	nand_copy_req_data_to_buf
	movl	20(%ebx), %ebx
	testl	%ebx, %ebx
	jne	.L2585
.L2582:
	movl	g_nand_bg_info+36, %eax
	testl	%eax, %eax
	je	.L2584
	movl	-16(%ebp), %edx
	movl	%edi, %ecx
	movl	%esi, (%esp)
	call	nand_copy_req_data_to_buf
.L2584:
	addl	$8, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	nand_read_data_in_buf, .-nand_read_data_in_buf
	.p2align 4,,15
	.globl	nand_pre_read_data_in_buf
	.type	nand_pre_read_data_in_buf, @function
nand_pre_read_data_in_buf:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	g_nand_bg_info+28, %ebx
	testl	%ebx, %ebx
	je	.L2590
	leal	(%edx,%eax), %edi
	.p2align 4,,7
	.p2align 3
.L2600:
	movl	(%ebx), %esi
	cmpl	%eax, %esi
	ja	.L2591
	addl	4(%ebx), %esi
	cmpl	%edi, %esi
	jae	.L2596
.L2591:
	movl	20(%ebx), %ebx
	testl	%ebx, %ebx
	jne	.L2600
.L2590:
	movl	g_nand_bg_info+24, %ebx
	leal	(%edx,%eax), %edi
	testl	%ebx, %ebx
	je	.L2603
	.p2align 4,,7
	.p2align 3
.L2599:
	movl	(%ebx), %esi
	cmpl	%esi, %eax
	jb	.L2595
	addl	4(%ebx), %esi
	cmpl	%edi, %esi
	jae	.L2596
.L2595:
	movl	20(%ebx), %ebx
	testl	%ebx, %ebx
	jne	.L2599
.L2603:
	popl	%ebx
	xorl	%eax, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2596:
	call	nand_read_data_in_buf
	movl	$1, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.size	nand_pre_read_data_in_buf, .-nand_pre_read_data_in_buf
	.p2align 4,,15
	.globl	nand_set_next_pre_read
	.type	nand_set_next_pre_read, @function
nand_set_next_pre_read:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	movl	g_nand_bg_info+32, %ebx
	testl	%ebx, %ebx
	je	.L2615
.L2605:
	movl	20(%ebx), %ecx
	movl	%ecx, g_nand_bg_info+32
	movl	$__key.6352, %ecx
	movl	%eax, (%ebx)
	leal	28(%ebx), %eax
	movl	%edx, 4(%ebx)
	movl	$.LC115, %edx
	movl	$0, 8(%ebx)
	movl	$1, 12(%ebx)
	movl	$0, 24(%ebx)
	call	__init_waitqueue_head
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+20, %edx
	testl	%edx, %edx
	jne	.L2614
	jmp	.L2618
	.p2align 4,,7
	.p2align 3
.L2612:
	movl	%eax, %edx
.L2614:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2612
	movl	%ebx, 20(%edx)
	addl	$1, TotleCachePreReadCount
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2615:
	movl	g_nand_bg_info+28, %ebx
	testl	%ebx, %ebx
	je	.L2605
	movl	20(%ebx), %ecx
	movl	%ecx, g_nand_bg_info+28
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+32, %esi
	testl	%esi, %esi
	jne	.L2613
	jmp	.L2619
	.p2align 4,,7
	.p2align 3
.L2611:
	movl	%ecx, %esi
.L2613:
	movl	20(%esi), %ecx
	testl	%ecx, %ecx
	jne	.L2611
	movl	%ebx, 20(%esi)
	movl	g_nand_bg_info+32, %ebx
	jmp	.L2605
.L2618:
	movl	%ebx, g_nand_bg_info+20
	addl	$1, TotleCachePreReadCount
	popl	%ebx
	popl	%esi
	popl	%ebp
	ret
.L2619:
	movl	%ebx, g_nand_bg_info+32
	jmp	.L2605
	.size	nand_set_next_pre_read, .-nand_set_next_pre_read
	.p2align 4,,15
	.type	nand_dev_transfer.isra.30, @function
nand_dev_transfer.isra.30:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	movl	%ecx, %edi
	pushl	%esi
	pushl	%ebx
	movl	$-5, %ebx
	subl	$32, %esp
	movl	36(%eax), %esi
	movl	12(%ebp), %ecx
	testl	%esi, %esi
	jne	.L2621
	cmpl	$1, %ecx
	je	.L2683
	testl	%ecx, %ecx
	jne	.L2624
	movl	32(%eax), %ecx
	testl	%ecx, %ecx
	jne	.L2621
	movl	24(%eax), %esi
	movl	$g_nand_bg_info, %eax
	movl	$1, nand_bg_do
	addl	%edx, %esi
	call	_raw_spin_lock_irq
	movl	g_nand_bg_info+36, %ebx
	testl	%ebx, %ebx
	je	.L2648
	movl	$0, g_nand_bg_info+36
	leal	28(%ebx), %eax
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movl	$0, 24(%ebx)
	movl	$1, nand_bg_do
	call	__init_waitqueue_head
	movl	4(%ebx), %edx
	movl	(%ebx), %eax
	call	nand_invalid_data_in_buf
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+24, %edx
	testl	%edx, %edx
	jne	.L2673
	movl	%ebx, g_nand_bg_info+24
	.p2align 4,,7
	.p2align 3
.L2648:
	cmpl	%esi, g_next_read_Index
	je	.L2650
	movl	$0, g_next_read_match_sector
	movl	$0, g_next_set_read_Index
.L2651:
	movl	g_nand_bg_info+20, %ebx
	leal	(%edi,%esi), %eax
	movl	%eax, g_next_read_Index
	testl	%ebx, %ebx
	je	.L2652
	movl	(%ebx), %edx
	cmpl	%esi, %edx
	ja	.L2652
	addl	4(%ebx), %edx
	cmpl	%edx, %eax
	jbe	.L2684
.L2652:
	cmpl	$1023, g_next_read_match_sector
	ja	.L2685
	movl	$0, g_next_set_read_Index
.L2655:
	movl	8(%ebp), %ecx
	movl	%edi, %edx
	movl	%esi, %eax
	call	nand_pre_read_data_in_buf
	cmpl	$1, %eax
	je	.L2686
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	movl	$1, rk29_buffer_wait_read
	call	nand_device_lock
	movl	8(%ebp), %ecx
	movl	%edi, %edx
	movl	%esi, %eax
	movl	$0, (%esp)
	call	nand_transfer
	movl	%eax, %ebx
	call	nand_device_unlock
	cmpl	$2, rk29_buffer_wait_read
	je	.L2687
.L2659:
	movl	$g_nand_bg_info, %eax
	movl	$0, rk29_buffer_wait_read
	call	_raw_spin_lock_irq
	movl	8(%ebp), %ecx
	movl	%edi, %edx
	movl	%esi, %eax
	call	nand_read_data_in_buf
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
.L2621:
	addl	$32, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2668:
	movl	%eax, %edx
.L2673:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2668
	movl	%ebx, 20(%edx)
	jmp	.L2648
	.p2align 4,,7
	.p2align 3
.L2624:
	xorl	%ebx, %ebx
	movl	$1, nand_bg_do
	addl	$32, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2683:
	movl	28(%eax), %esi
	testl	%esi, %esi
	jne	.L2621
	movl	24(%eax), %esi
	movl	$g_nand_bg_info, %eax
	movl	$1, nand_bg_do
	addl	%edx, %esi
	call	_raw_spin_lock_irq
	movl	g_nand_bg_info+36, %ebx
	testl	%ebx, %ebx
	je	.L2681
	movl	$3, %edx
	movl	$nand_bg_thread_wait, %eax
	movl	$1, %ecx
	movl	$0, (%esp)
	call	__wake_up
	movl	4(%ebx), %eax
	movl	(%ebx), %edx
	addl	%eax, %edx
	cmpl	%edx, %esi
	je	.L2688
	movl	$0, g_nand_bg_info+36
	leal	28(%ebx), %eax
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movl	$0, 24(%ebx)
	call	__init_waitqueue_head
	movl	4(%ebx), %edx
	movl	(%ebx), %eax
	call	nand_invalid_data_in_buf
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+24, %edx
	testl	%edx, %edx
	jne	.L2670
	jmp	.L2698
	.p2align 4,,7
	.p2align 3
.L2665:
	movl	%eax, %edx
.L2670:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2665
	movl	%ebx, 20(%edx)
	.p2align 4,,7
	.p2align 3
.L2681:
	movl	g_nand_bg_info+32, %ebx
.L2676:
	testl	%ebx, %ebx
	jne	.L2690
.L2642:
	movl	g_nand_bg_info+28, %ebx
	testl	%ebx, %ebx
	je	.L2635
	movl	20(%ebx), %eax
	movl	%eax, g_nand_bg_info+28
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+32, %edx
	testl	%edx, %edx
	jne	.L2671
	jmp	.L2699
	.p2align 4,,7
	.p2align 3
.L2666:
	movl	%eax, %edx
.L2671:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2666
	movl	%ebx, 20(%edx)
	movl	g_nand_bg_info+32, %ebx
	testl	%ebx, %ebx
	je	.L2642
.L2690:
	movl	20(%ebx), %eax
	movl	%edi, %ecx
	movl	8(%ebp), %edx
	sall	$9, %ecx
	movl	%eax, g_nand_bg_info+32
	movl	16(%ebx), %eax
	movl	$1, 8(%ebx)
	movl	%esi, (%ebx)
	movl	%edi, 4(%ebx)
	call	memcpy
	cmpl	$255, 4(%ebx)
	jg	.L2692
	movl	%ebx, g_nand_bg_info+36
.L2682:
	movl	$g_nand_bg_info, %eax
	xorl	%ebx, %ebx
	call	_raw_spin_unlock_irq
	addl	$32, %esp
	movl	%ebx, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2685:
	movl	g_next_set_read_Index, %eax
	movl	g_next_read_Index, %edx
	cmpl	%edx, %eax
	jbe	.L2654
	leal	256(%edx), %ecx
	subl	%edi, %ecx
	cmpl	%ecx, %eax
	jne	.L2655
.L2654:
	movl	g_nand_bg_info+32, %ebx
	testl	%ebx, %ebx
	je	.L2693
.L2656:
	cmpl	%edx, %eax
	cmovb	%edx, %eax
	movl	$256, %edx
	addl	$256, %eax
	movl	%eax, g_next_set_read_Index
	call	nand_set_next_pre_read
	movl	$1, %ecx
	movl	$3, %edx
	movl	$nand_bg_thread_wait, %eax
	movl	$0, (%esp)
	movl	$1, nand_bg_do
	call	__wake_up
	jmp	.L2655
.L2699:
	movl	%ebx, g_nand_bg_info+32
	jmp	.L2676
.L2650:
	addl	%edi, g_next_read_match_sector
	jmp	.L2651
.L2635:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	movl	rk29_buffer_write_done, %eax
	testl	%eax, %eax
	jne	.L2639
	leal	-20(%ebp), %edx
	movl	%edx, -20(%ebp)
	movl	%edx, -16(%ebp)
	movl	$0, -32(%ebp)
	jmp	.L2641
	.p2align 4,,7
	.p2align 3
.L2694:
	call	schedule
.L2641:
	movl	$2, %ecx
	movl	$rk29_buffer_wait, %eax
	leal	-32(%ebp), %edx
	call	prepare_to_wait_event
	movl	rk29_buffer_write_done, %eax
	testl	%eax, %eax
	je	.L2694
	leal	-32(%ebp), %edx
	movl	$rk29_buffer_wait, %eax
	call	finish_wait
.L2639:
	movl	$g_nand_bg_info, %eax
	movl	$0, rk29_buffer_write_done
	call	_raw_spin_lock_irq
	jmp	.L2681
.L2687:
	movl	$1, nand_bg_do
	jmp	.L2659
.L2686:
	addl	$1, TotleCacheMatchCount
	jmp	.L2682
.L2688:
	movl	$256, %edx
	subl	%eax, %edx
	cmpl	%edx, %edi
	cmovle	%edi, %edx
	movl	%edx, -36(%ebp)
	sall	$9, %edx
	movl	%edx, -40(%ebp)
	movl	%edx, %ecx
	movl	8(%ebp), %edx
	sall	$9, %eax
	addl	16(%ebx), %eax
	call	memcpy
	movl	-36(%ebp), %eax
	addl	4(%ebx), %eax
	cmpl	$255, %eax
	movl	%eax, 4(%ebx)
	jg	.L2695
.L2628:
	movl	-40(%ebp), %eax
	addl	%eax, 8(%ebp)
	addl	-36(%ebp), %esi
	subl	-36(%ebp), %edi
	jne	.L2681
	jmp	.L2682
.L2684:
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_unlock_irq
	leal	24(%ebx), %eax
	call	wait_for_completion
	movl	$g_nand_bg_info, %eax
	call	_raw_spin_lock_irq
	jmp	.L2652
.L2692:
	movl	$.LC115, %edx
	movl	$__key.6352, %ecx
	movl	$0, 24(%ebx)
	leal	28(%ebx), %eax
	call	__init_waitqueue_head
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+24, %edx
	testl	%edx, %edx
	jne	.L2672
	jmp	.L2700
	.p2align 4,,7
	.p2align 3
.L2667:
	movl	%eax, %edx
.L2672:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2667
	movl	%ebx, 20(%edx)
	jmp	.L2682
.L2693:
	movl	g_nand_bg_info+28, %ecx
	testl	%ecx, %ecx
	jne	.L2656
	jmp	.L2655
.L2695:
	movl	$0, g_nand_bg_info+36
	leal	28(%ebx), %eax
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movl	$0, 24(%ebx)
	call	__init_waitqueue_head
	movl	4(%ebx), %edx
	movl	(%ebx), %eax
	call	nand_invalid_data_in_buf
	movl	$0, 20(%ebx)
	movl	g_nand_bg_info+24, %edx
	testl	%edx, %edx
	jne	.L2669
	jmp	.L2701
	.p2align 4,,7
	.p2align 3
.L2664:
	movl	%eax, %edx
.L2669:
	movl	20(%edx), %eax
	testl	%eax, %eax
	jne	.L2664
	movl	%ebx, 20(%edx)
	jmp	.L2628
.L2698:
	movl	%ebx, g_nand_bg_info+24
	jmp	.L2681
.L2700:
	movl	%ebx, g_nand_bg_info+24
	jmp	.L2682
.L2701:
	movl	%ebx, g_nand_bg_info+24
	jmp	.L2628
	.size	nand_dev_transfer.isra.30, .-nand_dev_transfer.isra.30
	.section	.rodata.str1.4
	.align 4
.LC116:
	.string	"Attempted to advance past end of bvec iter\n"
	.section	.rodata.str1.1
.LC117:
	.string	"include/linux/bio.h"
.LC118:
	.string	"\0013nand th quited\n"
.LC119:
	.string	"\0013wait for rk nand bg quit\n"
	.text
	.p2align 4,,15
	.type	nand_blktrans_thread, @function
nand_blktrans_thread:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	xorl	%ebx, %ebx
	subl	$120, %esp
	movl	%eax, -112(%ebp)
	movl	72(%eax), %eax
	movl	%eax, -96(%ebp)
	movl	572(%eax), %eax
	call	_raw_spin_lock_irq
	movl	jiffies, %eax
	movl	$500, rk_ftl_gc_jiffies
	movl	$0, rk_ftl_gc_do
	addl	$500, %eax
	movl	%eax, rk_ftl_gc_timeout+8
	movl	$rk_ftl_gc_timeout, %eax
	call	add_timer
	movl	-112(%ebp), %edx
#APP
# 14 "/home/zyf/sofia5.1/kernel/arch/x86/include/asm/current.h" 1
	movl %fs:current_task,%eax
# 0 "" 2
#NO_APP
	movl	$0, -108(%ebp)
	movl	%eax, -120(%ebp)
	addl	$60, %edx
	movl	%edx, -116(%ebp)
.L2785:
	movl	-112(%ebp), %eax
	movl	48(%eax), %eax
	testl	%eax, %eax
	jne	.L2792
.L2755:
	movl	-108(%ebp), %esi
	movl	-120(%ebp), %eax
	movl	$0, -56(%ebp)
	movl	$0, -44(%ebp)
	testl	%esi, %esi
	movl	$0, -40(%ebp)
	movl	%eax, -52(%ebp)
	movl	$default_wake_function, -48(%ebp)
	je	.L2793
.L2704:
	movl	-108(%ebp), %edx
	movl	$300, rk_ftl_gc_jiffies
	movl	112(%edx), %eax
	movl	56(%edx), %edx
	movl	524(%eax), %eax
	shrl	$9, %edx
	movl	%edx, -80(%ebp)
	movl	%eax, -100(%ebp)
	movl	-108(%ebp), %eax
	movl	60(%eax), %esi
	movl	64(%eax), %edi
	movl	36(%eax), %eax
	testb	$-128, %al
	jne	.L2794
	andl	$1, %eax
	movl	%eax, -104(%ebp)
	jne	.L2721
	movl	mtd_read_temp_buffer, %edx
	testl	%edx, %edx
	movl	%edx, -84(%ebp)
	je	.L2721
	movl	-108(%ebp), %eax
	movl	68(%eax), %eax
	testl	%eax, %eax
	movl	%eax, -72(%ebp)
	je	.L2763
	movl	$0, -76(%ebp)
	movl	$0, -68(%ebp)
	movl	%esi, -92(%ebp)
	movl	%edi, -88(%ebp)
.L2733:
	movl	-72(%ebp), %edx
	movl	24(%edx), %ebx
	movl	28(%edx), %edx
	movl	%edx, -60(%ebp)
	movl	-72(%ebp), %edx
	movl	32(%edx), %esi
	.p2align 4,,7
	.p2align 3
.L2782:
	testl	%ebx, %ebx
	je	.L2731
.L2797:
	movl	-60(%ebp), %edx
	leal	(%edx,%edx,2), %eax
	movl	-72(%ebp), %edx
	sall	$2, %eax
	addl	68(%edx), %eax
	movl	4(%eax), %edx
	movl	8(%eax), %edi
	movl	(%eax), %eax
	movl	%edx, -64(%ebp)
	addl	%esi, %edi
	call	nand_page_address
	movl	-68(%ebp), %edx
	movl	-76(%ebp), %ecx
	addl	%edi, %eax
	testl	%edx, %edx
	movl	-68(%ebp), %edx
	cmove	%eax, %edx
	testl	%ecx, %ecx
	movl	%edx, -68(%ebp)
	je	.L2724
	cmpl	%eax, -76(%ebp)
	jne	.L2790
.L2724:
	movl	-64(%ebp), %edi
	movl	-72(%ebp), %edx
	subl	%esi, %edi
	cmpl	%ebx, %edi
	cmova	%ebx, %edi
	addl	%edi, %eax
	testl	$640, 12(%edx)
	movl	%eax, -76(%ebp)
	jne	.L2795
	movl	-72(%ebp), %edx
	cmpl	%ebx, %edi
	movl	68(%edx), %edx
	movl	%edx, -64(%ebp)
	ja	.L2796
	.p2align 4,,7
	.p2align 3
.L2728:
	testl	%edi, %edi
	je	.L2782
	movl	-60(%ebp), %edx
	cmpl	%edi, %ebx
	leal	(%edx,%edx,2), %eax
	movl	-64(%ebp), %edx
	movl	4(%edx,%eax,4), %ecx
	movl	%edi, %eax
	cmovbe	%ebx, %eax
	movl	%ecx, %edx
	subl	%esi, %edx
	cmpl	%edx, %eax
	cmova	%edx, %eax
	addl	%eax, %esi
	subl	%eax, %edi
	subl	%eax, %ebx
	cmpl	%esi, %ecx
	jne	.L2728
	addl	$1, -60(%ebp)
	xorl	%esi, %esi
	jmp	.L2728
.L2795:
	subl	%edi, %ebx
	testl	%ebx, %ebx
	jne	.L2797
.L2731:
	movl	-72(%ebp), %edx
	movl	(%edx), %edx
	testl	%edx, %edx
	movl	%edx, -72(%ebp)
	jne	.L2733
	movl	-92(%ebp), %esi
	jmp	.L2722
	.p2align 4,,7
	.p2align 3
.L2721:
	movl	-108(%ebp), %eax
	movl	68(%eax), %eax
	testl	%eax, %eax
	movl	%eax, -72(%ebp)
	je	.L2734
	movl	$0, -84(%ebp)
	movl	$0, -68(%ebp)
	movl	%esi, -92(%ebp)
	movl	%edi, -88(%ebp)
	.p2align 4,,7
	.p2align 3
.L2754:
	movl	-72(%ebp), %edx
	movl	28(%edx), %eax
	movl	24(%edx), %esi
	movl	32(%edx), %edi
	movl	%eax, -60(%ebp)
	.p2align 4,,7
	.p2align 3
.L2784:
	testl	%esi, %esi
	je	.L2751
.L2802:
	movl	-60(%ebp), %edx
	leal	(%edx,%edx,2), %eax
	movl	-72(%ebp), %edx
	sall	$2, %eax
	addl	68(%edx), %eax
	movl	4(%eax), %ebx
	movl	(%eax), %edx
	movl	8(%eax), %eax
	subl	%edi, %ebx
	cmpl	%esi, %ebx
	cmova	%esi, %ebx
	addl	%edi, %eax
	movl	%eax, -76(%ebp)
	movl	%edx, %eax
	movl	%edx, -64(%ebp)
	call	nand_page_address
	movl	-76(%ebp), %edx
	addl	%eax, %edx
	movl	-84(%ebp), %eax
	addl	-68(%ebp), %eax
	cmpl	%eax, %edx
	je	.L2798
	movl	-68(%ebp), %eax
	movl	$0, -80(%ebp)
	testl	%eax, %eax
	jne	.L2799
.L2745:
	movl	-80(%ebp), %eax
	xorl	%edx, %edx
	addl	%eax, -92(%ebp)
	movl	-64(%ebp), %eax
	adcl	%edx, -88(%ebp)
	call	nand_page_address
	movl	%ebx, -68(%ebp)
	addl	-76(%ebp), %eax
	movl	%eax, -84(%ebp)
.L2744:
	movl	-72(%ebp), %eax
	testl	$640, 12(%eax)
	jne	.L2800
	movl	-72(%ebp), %edx
	cmpl	%esi, %ebx
	movl	68(%edx), %edx
	movl	%edx, -64(%ebp)
	ja	.L2801
	.p2align 4,,7
	.p2align 3
.L2748:
	testl	%ebx, %ebx
	je	.L2784
	movl	-60(%ebp), %edx
	cmpl	%ebx, %esi
	leal	(%edx,%edx,2), %eax
	movl	-64(%ebp), %edx
	movl	4(%edx,%eax,4), %ecx
	movl	%ebx, %eax
	cmovbe	%esi, %eax
	movl	%ecx, %edx
	subl	%edi, %edx
	cmpl	%edx, %eax
	cmova	%edx, %eax
	addl	%eax, %edi
	subl	%eax, %ebx
	subl	%eax, %esi
	cmpl	%edi, %ecx
	jne	.L2748
	addl	$1, -60(%ebp)
	xorl	%edi, %edi
	jmp	.L2748
	.p2align 4,,7
	.p2align 3
.L2800:
	subl	%ebx, %esi
	testl	%esi, %esi
	jne	.L2802
.L2751:
	movl	-72(%ebp), %eax
	movl	(%eax), %eax
	testl	%eax, %eax
	movl	%eax, -72(%ebp)
	jne	.L2754
	movl	-68(%ebp), %eax
	movl	-92(%ebp), %esi
	testl	%eax, %eax
	je	.L2734
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	movl	-104(%ebp), %eax
	movl	-84(%ebp), %edx
	movl	-68(%ebp), %ecx
	movl	%eax, 4(%esp)
	movl	-100(%ebp), %eax
	movl	%edx, (%esp)
	movl	%esi, %edx
	shrl	$9, %ecx
	call	nand_dev_transfer.isra.30
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_lock_irq
.L2734:
	movl	-108(%ebp), %eax
	xorl	%edx, %edx
	xorl	%ebx, %ebx
	call	__blk_end_request_all
	movl	-112(%ebp), %eax
	movl	$0, -108(%ebp)
	movl	48(%eax), %eax
	testl	%eax, %eax
	je	.L2755
.L2792:
	movl	$.LC118, (%esp)
	call	printk
	movl	-108(%ebp), %edi
	movl	-112(%ebp), %eax
	testl	%edi, %edi
	movl	$1, 52(%eax)
	je	.L2756
	movl	-108(%ebp), %eax
	movl	$-5, %edx
	call	__blk_end_request_all
.L2756:
	movl	$0, FlashWaitBusyScheduleEn
	movl	-96(%ebp), %ebx
	jmp	.L2757
.L2758:
	movl	$-19, %edx
	call	__blk_end_request_all
.L2757:
	movl	%ebx, %eax
	call	blk_fetch_request
	testl	%eax, %eax
	jne	.L2758
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	movl	$1, nand_bg_do
	call	nand_bg_add_last_write_req
	movl	-112(%ebp), %eax
	movl	$1, nand_bg_do
	movl	$1, 56(%eax)
	movl	$.LC119, (%esp)
	call	printk
	movl	$g_nand_bg_info+40, %eax
	call	wait_for_completion
	movl	-112(%ebp), %eax
	xorl	%edx, %edx
	addl	$32, %eax
	call	complete_and_exit
	.p2align 4,,7
	.p2align 3
.L2799:
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	movl	-104(%ebp), %edx
	movl	-68(%ebp), %eax
	movl	%edx, 4(%esp)
	movl	-84(%ebp), %edx
	shrl	$9, %eax
	movl	%eax, -80(%ebp)
	movl	%eax, %ecx
	movl	-100(%ebp), %eax
	movl	%edx, (%esp)
	movl	-92(%ebp), %edx
	call	nand_dev_transfer.isra.30
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_lock_irq
	jmp	.L2745
	.p2align 4,,7
	.p2align 3
.L2798:
	addl	%ebx, -68(%ebp)
	jmp	.L2744
.L2794:
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	call	nand_device_lock
	movl	-108(%ebp), %edx
	movl	60(%edx), %eax
	movl	-100(%ebp), %edx
	addl	24(%edx), %eax
	movl	-80(%ebp), %edx
	call	FtlDiscard
	cmpl	$1, %eax
	sbbl	%ebx, %ebx
	call	nand_device_unlock
	movl	-96(%ebp), %edx
	notl	%ebx
	andl	$-5, %ebx
	movl	572(%edx), %eax
	call	_raw_spin_lock_irq
	movl	-108(%ebp), %eax
	movl	%ebx, %edx
	xorl	%ebx, %ebx
	call	__blk_end_request_cur
	testb	%al, %al
	movl	-108(%ebp), %eax
	cmove	%ebx, %eax
	movl	%eax, -108(%ebp)
	jmp	.L2785
.L2790:
	movl	-92(%ebp), %esi
.L2725:
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	movl	-84(%ebp), %eax
	movl	%esi, %edx
	movl	-80(%ebp), %ecx
	movl	$0, 4(%esp)
	movl	%eax, (%esp)
	movl	-100(%ebp), %eax
	call	nand_dev_transfer.isra.30
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_lock_irq
	movl	mtd_read_temp_buffer, %eax
	movl	-84(%ebp), %edx
	movl	%eax, -72(%ebp)
	cmpl	%edx, %eax
	jne	.L2734
	movl	-108(%ebp), %eax
	movl	68(%eax), %eax
	testl	%eax, %eax
	movl	%eax, -68(%ebp)
	je	.L2734
.L2769:
	movl	-68(%ebp), %edx
	movl	28(%edx), %eax
	movl	24(%edx), %esi
	movl	32(%edx), %edi
	movl	%eax, -60(%ebp)
.L2783:
	testl	%esi, %esi
	je	.L2736
.L2808:
	movl	-60(%ebp), %edx
	leal	(%edx,%edx,2), %eax
	movl	-68(%ebp), %edx
	sall	$2, %eax
	addl	68(%edx), %eax
	movl	4(%eax), %ebx
	movl	8(%eax), %edx
	movl	(%eax), %eax
	subl	%edi, %ebx
	cmpl	%esi, %ebx
	cmova	%esi, %ebx
	addl	%edi, %edx
	movl	%edx, -64(%ebp)
	call	nand_page_address
	movl	-72(%ebp), %edx
	movl	%ebx, %ecx
	addl	-64(%ebp), %eax
	call	memcpy
	movl	-68(%ebp), %eax
	addl	%ebx, -72(%ebp)
	testl	$640, 12(%eax)
	jne	.L2803
	movl	-68(%ebp), %edx
	cmpl	%esi, %ebx
	movl	68(%edx), %edx
	movl	%edx, -64(%ebp)
	ja	.L2804
	.p2align 4,,7
	.p2align 3
.L2739:
	testl	%ebx, %ebx
	je	.L2783
	movl	-60(%ebp), %edx
	cmpl	%ebx, %esi
	movl	%ebx, %ecx
	cmovbe	%esi, %ecx
	leal	(%edx,%edx,2), %eax
	movl	-64(%ebp), %edx
	movl	4(%edx,%eax,4), %eax
	movl	%eax, %edx
	subl	%edi, %edx
	cmpl	%edx, %ecx
	cmovbe	%ecx, %edx
	addl	%edx, %edi
	subl	%edx, %ebx
	subl	%edx, %esi
	cmpl	%edi, %eax
	jne	.L2739
	addl	$1, -60(%ebp)
	xorl	%edi, %edi
	jmp	.L2739
.L2793:
	movl	-96(%ebp), %eax
	call	blk_fetch_request
	testl	%eax, %eax
	movl	%eax, -108(%ebp)
	jne	.L2704
	movl	-116(%ebp), %eax
	leal	-56(%ebp), %edx
	call	add_wait_queue
	movl	-120(%ebp), %edx
	movl	$1, -16(%ebp)
	movl	-16(%ebp), %eax
#APP
# 886 "drivers/sofia_nand/nand_blk_s.c" 1
	xchgl %eax, (%edx)

# 0 "" 2
#NO_APP
	movl	-96(%ebp), %edx
	movl	%eax, -16(%ebp)
	movl	-16(%ebp), %eax
	movl	572(%edx), %eax
	call	_raw_spin_unlock_irq
	call	nand_device_trylock
	testl	%eax, %eax
	je	.L2805
.L2705:
	movl	$2, rk_ftl_gc_jiffies
.L2707:
	addl	$1, %ebx
	leal	-5(%ebx), %eax
	cmpl	$1, %eax
	jbe	.L2806
	cmpl	$0, rk_ftl_gc_do
	movl	rk_ftl_gc_jiffies, %edi
	jne	.L2718
	movl	-112(%ebp), %eax
	cmpl	$0, 48(%eax)
	jne	.L2718
	testl	%edi, %edi
	jne	.L2807
.L2718:
	movl	$0, rk_ftl_gc_do
.L2791:
	movl	-96(%ebp), %edx
	movl	572(%edx), %eax
	call	_raw_spin_lock_irq
	movl	-116(%ebp), %eax
	leal	-56(%ebp), %edx
	call	remove_wait_queue
	jmp	.L2785
	.p2align 4,,7
	.p2align 3
.L2803:
	subl	%ebx, %esi
	testl	%esi, %esi
	jne	.L2808
.L2736:
	movl	-68(%ebp), %eax
	movl	(%eax), %eax
	testl	%eax, %eax
	movl	%eax, -68(%ebp)
	jne	.L2769
	jmp	.L2734
.L2763:
	movl	$0, -68(%ebp)
.L2722:
	movl	-68(%ebp), %edx
	movl	%edx, -84(%ebp)
	jmp	.L2725
.L2806:
	call	nand_bg_add_last_write_req
	cmpl	$0, rk_ftl_gc_do
	jne	.L2718
	movl	-112(%ebp), %eax
	cmpl	$0, 48(%eax)
	jne	.L2718
	leal	-24(%ebp), %eax
	movl	$2, %edi
	movl	%eax, -24(%ebp)
	leal	-36(%ebp), %esi
	movl	%eax, -20(%ebp)
	movl	$0, -36(%ebp)
.L2759:
	movl	-116(%ebp), %eax
	movl	$2, %ecx
	movl	%esi, %edx
	call	prepare_to_wait_event
	cmpl	$0, rk_ftl_gc_do
	jne	.L2716
	movl	-112(%ebp), %eax
	cmpl	$0, 48(%eax)
	jne	.L2716
	testl	%edi, %edi
	jne	.L2714
.L2716:
.L2711:
	movl	-116(%ebp), %eax
	movl	%esi, %edx
	call	finish_wait
	jmp	.L2718
.L2714:
	movl	%edi, %eax
	call	schedule_timeout
	movl	%eax, %edi
	.p2align 4,,4
	jmp	.L2759
.L2805:
	xorl	%edx, %edx
	movb	$1, %al
	call	FtlGarbageCollect
	movl	%eax, %esi
	call	nand_device_unlock
	testl	%esi, %esi
	jne	.L2706
	movl	$200, rk_ftl_gc_jiffies
	jmp	.L2707
.L2807:
	leal	-24(%ebp), %eax
	movl	%eax, -24(%ebp)
	leal	-36(%ebp), %esi
	movl	%eax, -20(%ebp)
	movl	$0, -36(%ebp)
.L2760:
	movl	-116(%ebp), %eax
	movl	$2, %ecx
	movl	%esi, %edx
	call	prepare_to_wait_event
	cmpl	$0, rk_ftl_gc_do
	jne	.L2716
	movl	-112(%ebp), %eax
	cmpl	$0, 48(%eax)
	jne	.L2716
	testl	%edi, %edi
	je	.L2716
	movl	%edi, %eax
	call	schedule_timeout
	movl	%eax, %edi
	.p2align 4,,2
	jmp	.L2760
.L2706:
	cmpl	$7, %esi
	jg	.L2705
	.p2align 4,,6
	jmp	.L2791
.L2804:
	cmpb	$1, __warned.31817
	je	.L2739
	movl	$.LC116, 8(%esp)
	movl	$202, 4(%esp)
	movl	$.LC117, (%esp)
	call	warn_slowpath_fmt
	movb	$1, __warned.31817
	jmp	.L2739
.L2801:
	cmpb	$1, __warned.31817
	je	.L2748
	movl	$.LC116, 8(%esp)
	movl	$202, 4(%esp)
	movl	$.LC117, (%esp)
	call	warn_slowpath_fmt
	movb	$1, __warned.31817
	jmp	.L2748
.L2796:
	cmpb	$1, __warned.31817
	je	.L2728
	movl	$.LC116, 8(%esp)
	movl	$202, 4(%esp)
	movl	$.LC117, (%esp)
	call	warn_slowpath_fmt
	movb	$1, __warned.31817
	jmp	.L2728
	.size	nand_blktrans_thread, .-nand_blktrans_thread
	.p2align 4,,15
	.globl	rknand_queue_cond_resched
	.type	rknand_queue_cond_resched, @function
rknand_queue_cond_resched:
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	ret
	.size	rknand_queue_cond_resched, .-rknand_queue_cond_resched
	.p2align 4,,15
	.globl	set_part_mod
	.type	set_part_mod, @function
set_part_mod:
	pushl	%ebp
	xorl	%ecx, %ecx
	movl	%esp, %ebp
	subl	$16, %esp
	movl	%edi, -4(%ebp)
	movl	%edx, %edi
	movl	$2, %edx
	movl	%ebx, -12(%ebp)
	movl	%esi, -8(%ebp)
	call	filp_open
	movl	%edi, %ecx
	movl	%eax, %esi
	movl	12(%eax), %eax
	movl	32(%eax), %eax
	movl	308(%eax), %eax
	movl	92(%eax), %edx
	movl	516(%edx), %ebx
	xorl	%edx, %edx
	movl	$0, (%esp)
	call	*8(%ebx)
#APP
# 14 "/home/zyf/sofia5.1/kernel/arch/x86/include/asm/current.h" 1
	movl %fs:current_task,%eax
# 0 "" 2
#NO_APP
	movl	908(%eax), %edx
	movl	%esi, %eax
	call	filp_close
	movl	-12(%ebp), %ebx
	movl	-8(%ebp), %esi
	movl	-4(%ebp), %edi
	movl	%ebp, %esp
	popl	%ebp
	ret
	.size	set_part_mod, .-set_part_mod
	.section	.rodata.str1.1
.LC120:
	.string	"&nandr->thread_wq"
.LC121:
	.string	"nand"
.LC122:
	.string	"nand_bg"
	.section	.rodata.str1.4
	.align 4
.LC123:
	.string	"nand part    \t:   offset        end  flag\n"
	.section	.rodata.str1.1
.LC124:
	.string	"%s_%s \t: %8lx - %8lx  (%02x)\n"
	.text
	.p2align 4,,15
	.globl	nand_blk_register
	.type	nand_blk_register, @function
nand_blk_register:
	pushl	%ebp
	movl	$5, %edx
	movl	%esp, %ebp
	pushl	%edi
	pushl	%esi
	movl	$-1, %esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$32, %esp
	movl	$0, 48(%eax)
	movl	$0, 52(%eax)
	movl	$0, 56(%eax)
	movl	$1065169, %eax
	movl	$1, FlashWaitBusyScheduleEn
	call	__get_free_pages
	movl	(%ebx), %edx
	movl	%eax, mtd_read_temp_buffer
	movl	4(%ebx), %eax
	call	register_blkdev
	testl	%eax, %eax
	jne	.L2812
	leal	36(%ebx), %eax
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movw	$0, 76(%ebx)
	movl	$0, 32(%ebx)
	call	__init_waitqueue_head
	leal	60(%ebx), %eax
	movl	$__key.34617, %ecx
	movl	$.LC120, %edx
	call	__init_waitqueue_head
	leal	76(%ebx), %edx
	movl	$nand_blk_request, %eax
	movw	$0, g_rk_nand_ops_mutex
	movl	$1, g_rk_nand_ops_mutex+4
	movl	$g_rk_nand_ops_mutex+8, g_rk_nand_ops_mutex+8
	movl	$g_rk_nand_ops_mutex+8, g_rk_nand_ops_mutex+12
	call	blk_init_queue
	testl	%eax, %eax
	movl	%eax, 72(%ebx)
	jne	.L2813
	movl	4(%ebx), %eax
	movl	(%ebx), %edx
	call	unregister_blkdev
.L2812:
	addl	$32, %esp
	movl	%esi, %eax
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2813:
	movl	$256, %edx
	call	blk_queue_max_hw_sectors
	movl	72(%ebx), %eax
	movl	$256, %edx
	call	blk_queue_max_segments
	movl	72(%ebx), %eax
#APP
# 95 "/home/zyf/sofia5.1/kernel/arch/x86/include/asm/bitops.h" 1
	bts $14,556(%eax)
# 0 "" 2
#NO_APP
	movl	72(%ebx), %eax
	movl	$8388607, %edx
	call	blk_queue_max_discard_sectors
	movl	72(%ebx), %eax
	movl	g_MaxLbaSector, %ecx
	imull	$44, g_max_part_num, %edx
	movl	$16, 660(%eax)
	movl	72(%ebx), %eax
	subl	$12288, %ecx
	movl	%ebx, 552(%eax)
	leal	104(%ebx), %eax
	movl	%eax, 104(%ebx)
	movl	%eax, 108(%ebx)
	movl	$.LC121, %eax
	movl	%ecx, disk_array+36(%edx)
	movl	%eax, %esi
	movl	$11, %ecx
	movl	$12288, disk_array+32(%edx)
	movb	$0, disk_array+40(%edx)
	addl	$disk_array, %edx
	movl	%edx, %edi
	movl	$7, %edx
	rep movsb
	movl	g_max_part_num, %eax
	addl	$1, %eax
	movl	%eax, g_max_part_num
	movl	$1065169, %eax
	call	__get_free_pages
	testl	%eax, %eax
	movl	%eax, mtd_rw_buffer
	je	.L2814
	movl	$g_nand_bg_req+8, %esi
	xorl	%eax, %eax
.L2815:
	movl	%eax, %edx
	movl	$__key.6352, %ecx
	sall	$17, %edx
	addl	mtd_rw_buffer, %edx
	leal	1(%eax), %edi
	leal	(%eax,%eax,4), %eax
	movl	%edx, 8(%esi)
	leal	(%edi,%edi,4), %edx
	leal	g_nand_bg_req(,%edx,8), %edx
	leal	g_nand_bg_req+16(,%eax,8), %eax
	movl	%edx, 12(%esi)
	movl	$.LC115, %edx
	movl	$0, (%esi)
	movl	$0, 4(%esi)
	movl	$0, -4(%esi)
	movl	$0, -8(%esi)
	addl	$40, %esi
	movl	$0, 8(%eax)
	addl	$12, %eax
	call	__init_waitqueue_head
	cmpl	$4, %edi
	movl	%edi, %eax
	jne	.L2815
	movl	$0, g_nand_bg_req+140
	movl	$0, g_nand_bg_info+16
	movl	$0, g_nand_bg_info+20
	movl	$0, g_nand_bg_info+28
	movl	$0, g_nand_bg_info+24
	movl	$g_nand_bg_req, g_nand_bg_info+32
.L2814:
	movl	$__key.6352, %ecx
	movl	$.LC115, %edx
	movl	$g_nand_bg_info+44, %eax
	movw	$0, g_nand_bg_info
	movl	$0, g_nand_bg_info+40
	call	__init_waitqueue_head
	movl	$.LC122, 12(%esp)
	movl	$-1, 8(%esp)
	movl	%ebx, 4(%esp)
	movl	$nand_blktrans_bg_thread, (%esp)
	call	kthread_create_on_node
	cmpl	$-4096, %eax
	ja	.L2816
	call	wake_up_process
.L2816:
	movl	$.LC121, 12(%esp)
	movl	$-1, 8(%esp)
	movl	%ebx, 4(%esp)
	movl	$nand_blktrans_thread, (%esp)
	call	kthread_create_on_node
	cmpl	$-4096, %eax
	ja	.L2817
	call	wake_up_process
.L2817:
	movl	$.LC123, (%esp)
	call	printk
	cmpl	$0, g_max_part_num
	jle	.L2818
	movl	$disk_array+40, %esi
	xorl	%edi, %edi
.L2819:
	movzbl	(%esi), %edx
	movl	-8(%esi), %eax
	movl	%edx, 20(%esp)
	movl	-4(%esi), %edx
	addl	$44, %esi
	movl	%eax, 12(%esp)
	movl	$.LC124, (%esp)
	addl	%eax, %edx
	movl	(%ebx), %eax
	movl	%edx, 16(%esp)
	imull	$44, %edi, %edx
	addl	$1, %edi
	movl	%eax, 4(%esp)
	addl	$disk_array, %edx
	movl	%edx, 8(%esp)
	movl	%edx, -20(%ebp)
	call	printk
	movl	-20(%ebp), %edx
	movl	%ebx, %eax
	call	*12(%ebx)
	cmpl	%edi, g_max_part_num
	jg	.L2819
.L2818:
	xorl	%ecx, %ecx
	movl	$1638, %edx
	movl	$.LC121, %eax
	xorl	%esi, %esi
	movl	$0, 4(%esp)
	movl	$nand_proc_fops, (%esp)
	call	proc_create_data
	jmp	.L2812
	.size	nand_blk_register, .-nand_blk_register
	.section	.rodata.str1.4
	.align 4
.LC125:
	.string	"drivers/sofia_nand/nand_blk_s.c"
	.text
	.p2align 4,,15
	.globl	nand_blk_unregister
	.type	nand_blk_unregister, @function
nand_blk_unregister:
	pushl	%ebp
	movl	$1, %ecx
	movl	%esp, %ebp
	movl	$3, %edx
	pushl	%edi
	pushl	%esi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$4, %esp
	movl	$1, 48(%eax)
	leal	60(%eax), %eax
	movl	$0, (%esp)
	leal	104(%ebx), %esi
	call	__wake_up
	leal	32(%ebx), %eax
	call	wait_for_completion
	movl	104(%ebx), %eax
	cmpl	%esi, %eax
	movl	(%eax), %edi
	jne	.L2828
	jmp	.L2824
	.p2align 4,,7
	.p2align 3
.L2827:
	movl	%edi, %eax
	movl	%edx, %edi
.L2828:
	subl	$4, %eax
	call	*16(%ebx)
	cmpl	%edi, %esi
	movl	(%edi), %edx
	jne	.L2827
.L2824:
	movl	72(%ebx), %eax
	call	blk_cleanup_queue
	movl	4(%ebx), %eax
	movl	(%ebx), %edx
	call	unregister_blkdev
	cmpl	%esi, 104(%ebx)
	jne	.L2829
	addl	$4, %esp
	popl	%ebx
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.L2829:
#APP
# 1366 "drivers/sofia_nand/nand_blk_s.c" 1
	1:	ud2
.pushsection __bug_table,"a"
2:	.long 1b, .LC125
	.word 1366, 0
	.org 2b+12
.popsection
# 0 "" 2
#NO_APP
	.size	nand_blk_unregister, .-nand_blk_unregister
	.p2align 4,,15
	.globl	nand_buffer_lock
	.type	nand_buffer_lock, @function
nand_buffer_lock:
	pushl	%ebp
	movl	%esp, %ebp
	call	nand_device_lock
	popl	%ebp
	ret
	.size	nand_buffer_lock, .-nand_buffer_lock
	.p2align 4,,15
	.globl	nand_buffer_unlock
	.type	nand_buffer_unlock, @function
nand_buffer_unlock:
	pushl	%ebp
	movl	%esp, %ebp
	call	nand_device_unlock
	popl	%ebp
	ret
	.size	nand_buffer_unlock, .-nand_buffer_unlock
	.p2align 4,,15
	.globl	nand_cache_flush
	.type	nand_cache_flush, @function
nand_cache_flush:
	pushl	%ebp
	movl	%esp, %ebp
	call	FtlCacheWriteBack
	popl	%ebp
	ret
	.size	nand_cache_flush, .-nand_cache_flush
	.p2align 4,,15
	.globl	nand_dev_suspend
	.type	nand_dev_suspend, @function
nand_dev_suspend:
	pushl	%ebp
	movl	%esp, %ebp
	call	nand_device_lock
	call	FlashSuspend
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	nand_dev_suspend, .-nand_dev_suspend
	.p2align 4,,15
	.globl	nand_dev_resume
	.type	nand_dev_resume, @function
nand_dev_resume:
	pushl	%ebp
	movl	%esp, %ebp
	call	FlashResume
	call	nand_device_unlock
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	nand_dev_resume, .-nand_dev_resume
	.p2align 4,,15
	.globl	nand_dev_shutdown
	.type	nand_dev_shutdown, @function
nand_dev_shutdown:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	mytr+48, %eax
	testl	%eax, %eax
	je	.L2837
	leave
	ret
	.p2align 4,,7
	.p2align 3
.L2837:
	movl	$1, %ecx
	movl	$3, %edx
	movl	$mytr+60, %eax
	movl	$0, (%esp)
	movl	$1, mytr+48
	call	__wake_up
	movl	$mytr+32, %eax
	call	wait_for_completion
	call	FtlDeInit
	call	FlashDeInit
	leave
	.p2align 4,,4
	ret
	.size	nand_dev_shutdown, .-nand_dev_shutdown
	.p2align 4,,15
	.globl	nand_get_clk_rate
	.type	nand_get_clk_rate, @function
nand_get_clk_rate:
	movl	gp_nandc_info, %eax
	pushl	%ebp
	movl	%esp, %ebp
	popl	%ebp
	movl	8(%eax), %eax
	ret
	.size	nand_get_clk_rate, .-nand_get_clk_rate
	.section	.rodata.str1.1
.LC126:
	.string	"nandc"
	.text
	.p2align 4,,15
	.globl	nand_nandc_irq_init
	.type	nand_nandc_irq_init, @function
nand_nandc_irq_init:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%ecx, %ebx
	subl	$12, %esp
	movl	gp_nandc_info, %ecx
	testl	%edx, %edx
	movl	4(%ecx), %eax
	jne	.L2842
	xorl	%edx, %edx
	call	free_irq
	addl	$12, %esp
	xorl	%eax, %eax
	popl	%ebx
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2842:
	movl	(%ecx), %edx
	xorl	%ecx, %ecx
	movl	$.LC126, 4(%esp)
	movl	$0, (%esp)
	movl	%edx, 8(%esp)
	movl	%ebx, %edx
	call	request_threaded_irq
	addl	$12, %esp
	popl	%ebx
	popl	%ebp
	ret
	.size	nand_nandc_irq_init, .-nand_nandc_irq_init
	.p2align 4,,15
	.globl	rk_nandc_int_deinit
	.type	rk_nandc_int_deinit, @function
rk_nandc_int_deinit:
	pushl	%ebp
	movl	$rk_nandc_interrupt, %ecx
	movl	%esp, %ebp
	xorl	%edx, %edx
	xorl	%eax, %eax
	call	nand_nandc_irq_init
	xorl	%eax, %eax
	popl	%ebp
	ret
	.size	rk_nandc_int_deinit, .-rk_nandc_int_deinit
	.p2align 4,,15
	.globl	rk_nandc_int_init
	.type	rk_nandc_int_init, @function
rk_nandc_int_init:
	pushl	%ebp
	movl	$rk_nandc_interrupt, %ecx
	movl	%esp, %ebp
	movl	$1, %edx
	xorl	%eax, %eax
	call	nand_nandc_irq_init
	xorl	%eax, %eax
	movl	$0, nandc0_ready_completed_flag
	movl	$0, nandc0_xfer_completed_flag
	movl	$0, nandc0_llp_completed_flag
	popl	%ebp
	ret
	.size	rk_nandc_int_init, .-rk_nandc_int_init
	.p2align 4,,15
	.globl	FlashLoadPhyInfoInRam
	.type	FlashLoadPhyInfoInRam, @function
FlashLoadPhyInfoInRam:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%esi
	pushl	%ebx
	xorl	%ebx, %ebx
	jmp	.L2849
	.p2align 4,,7
	.p2align 3
.L2846:
	addl	$1, %ebx
	cmpl	$60, %ebx
	je	.L2852
.L2849:
	movl	%ebx, %esi
	movl	$IDByte, %edx
	sall	$5, %esi
	movzbl	NandFlashParaTbl(%esi), %ecx
	addl	$NandFlashParaTbl, %esi
	leal	1(%esi), %eax
	call	memcmp
	testl	%eax, %eax
	jne	.L2846
	testl	%esi, %esi
	je	.L2852
	movzbl	22(%esi), %ecx
	xorl	%eax, %eax
.L2851:
	movl	%eax, %edx
	sall	$5, %edx
	cmpb	NandOptPara(%edx), %cl
	je	.L2850
	addl	$1, %eax
	cmpl	$4, %eax
	jne	.L2851
.L2850:
	sall	$5, %eax
	leal	NandOptPara(%eax), %edx
	movl	NandOptPara(%eax), %eax
	popl	%ebx
	movl	%eax, gNandOptPara
	movl	4(%edx), %eax
	movl	%eax, gNandOptPara+4
	movl	8(%edx), %eax
	movl	%eax, gNandOptPara+8
	movl	12(%edx), %eax
	movl	%eax, gNandOptPara+12
	movl	16(%edx), %eax
	movl	%eax, gNandOptPara+16
	movl	20(%edx), %eax
	movl	%eax, gNandOptPara+20
	movl	24(%edx), %eax
	movl	%eax, gNandOptPara+24
	movl	28(%edx), %eax
	movl	%eax, gNandOptPara+28
	movl	(%esi), %eax
	movl	%eax, gNandParaInfo
	movl	4(%esi), %eax
	movl	%eax, gNandParaInfo+4
	movl	8(%esi), %eax
	movl	%eax, gNandParaInfo+8
	movl	12(%esi), %eax
	movl	%eax, gNandParaInfo+12
	movl	16(%esi), %eax
	movl	%eax, gNandParaInfo+16
	movl	20(%esi), %eax
	movl	%eax, gNandParaInfo+20
	movl	24(%esi), %eax
	movl	%eax, gNandParaInfo+24
	movl	28(%esi), %eax
	popl	%esi
	popl	%ebp
	movl	%eax, gNandParaInfo+28
	xorl	%eax, %eax
	ret
	.p2align 4,,7
	.p2align 3
.L2852:
	popl	%ebx
	movl	$-1, %eax
	popl	%esi
	popl	%ebp
	ret
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.p2align 4,,15
	.globl	FlashTimingCfg
	.type	FlashTimingCfg, @function
FlashTimingCfg:
	leal	-4193(%eax), %edx
	pushl	%ebp
	cmpl	$1, %edx
	movl	%esp, %ebp
	jbe	.L2857
	cmpl	$4225, %eax
	je	.L2857
	cmpl	$8322, %eax
	jne	.L2861
.L2857:
	movl	gpNandc, %edx
	movl	%eax, 4(%edx)
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2861:
	cmpl	$4226, %eax
	je	.L2857
	movzbl	gNandParaInfo+21, %eax
	call	NandcTimeCfg
	popl	%ebp
	ret
	.size	FlashTimingCfg, .-FlashTimingCfg
	.section	.rodata.str1.1
.LC127:
	.string	"\0013FlashLoadPhyInfo error %x\n"
	.text
	.p2align 4,,15
	.globl	FlashInit
	.type	FlashInit, @function
FlashInit:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%edi
	pushl	%ebx
	movl	%eax, %ebx
	subl	$8, %esp
	movl	$16384, %eax
	call	ftl_malloc
	movl	%eax, gFlashPageBuffer0
	movl	$16384, %eax
	call	ftl_malloc
	movl	%eax, gFlashPageBuffer1
	movl	$2048, %eax
	call	ftl_malloc
	movl	%eax, gFlashSpareBuffer
	movl	$16384, %eax
	call	ftl_malloc
	movl	%eax, gFlashProgCheckBuffer
	movl	$2048, %eax
	call	ftl_malloc
	movb	$24, gNandIDBResBlkNum
	movb	$24, gNandIDBResBlkNumSaveInFlash
	movl	$0, gNandFlashIdbBlockAddr
	movl	$128, gBlockPageAlignSize
	movb	$0, gFlashToggleModeEn
	movl	%eax, gFlashProgCheckSpareBuffer
	movl	%ebx, %eax
	call	NandcInit
	xorl	%eax, %eax
	movl	$IDByte, %edx
	call	FlashReadID
	movzbl	IDByte, %edx
	movl	$-2, %eax
	subl	$1, %edx
	cmpb	$-3, %dl
	ja	.L2863
	movl	$IDByte+8, %edx
	movl	$1, %eax
	call	FlashReadID
	movl	gBootDdrMode, %eax
	call	NandcSetDdrMode
	movl	$gReadRetryInfo, %edx
	xorl	%eax, %eax
	movl	%edx, %edi
	movl	$213, %ecx
	rep stosl
	movl	$gNandParaInfo, gpNandParaInfo
	movb	$0, gNandRandomizer
	call	FlashLoadPhyInfoInRam
	testl	%eax, %eax
	jne	.L2864
	movl	gpNandParaInfo, %eax
	movzbl	17(%eax), %eax
	andl	$7, %eax
	movb	%al, gFlashInterfaceMode
	movzbl	%al, %eax
	testb	$1, %al
	je	.L2891
.L2864:
	call	FlashLoadPhyInfo
	testl	%eax, %eax
	je	.L2867
.L2896:
	movl	gBootDdrMode, %eax
	testl	%eax, %eax
	je	.L2892
.L2866:
	movl	$1, 4(%esp)
	movl	$.LC127, (%esp)
	call	printk
	call	FlashLoadPhyInfoInRam
	movl	%eax, %edx
	movl	$-1, %eax
	cmpl	$-1, %edx
	je	.L2863
	call	FlashDieInfoInit
	movl	gpNandParaInfo, %eax
	movzbl	19(%eax), %eax
	call	FlashGetReadRetryDefault
	movl	gpNandParaInfo, %eax
	movzwl	gTotleBlock, %edx
	movzbl	9(%eax), %ecx
	addl	$4095, %edx
	sarl	$12, %edx
	cmpl	%ecx, %edx
	jg	.L2868
	movzwl	14(%eax), %edx
	addl	$255, %edx
	sarl	$8, %edx
	cmpl	%edx, %ecx
	jge	.L2869
.L2868:
	andw	$-256, 14(%eax)
.L2869:
	testb	$4, gFlashInterfaceMode
	je	.L2870
	call	FlashSavePhyInfo
	movl	gNandFlashInfoBlockAddr, %edx
	xorl	%eax, %eax
	call	FlashDdrParaScan
.L2870:
	call	FlashSavePhyInfo
	.p2align 4,,7
	.p2align 3
.L2867:
	movl	gpNandParaInfo, %ecx
	movl	$0, gpReadRetrial
	movzwl	16(%ecx), %edx
	movzwl	%dx, %eax
	testb	$-128, %al
	setne	gNandRandomizer
	testb	$8, %al
	setne	gMultiPageReadEn
	testb	$16, %al
	setne	gMultiPageProgEn
	shrw	$8, %dx
	andl	$7, %edx
	testb	$64, %al
	movb	%dl, gFlashInterfaceMode
	je	.L2871
	movzbl	gReadRetryInfo+1, %edx
	movzbl	19(%ecx), %eax
	movb	%dl, g_maxRegNum
	movzbl	gReadRetryInfo+2, %edx
	movb	%al, g_retryMode
	movb	%dl, g_maxRetryCount
	leal	-1(%eax), %edx
	cmpb	$5, %dl
	ja	.L2872
	movl	$HynixReadRetrial, gpReadRetrial
.L2871:
	movzwl	10(%ecx), %eax
	movzbl	12(%ecx), %ebx
	movzbl	18(%ecx), %ecx
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%ebx
	movl	%eax, %edx
	movl	%ecx, %eax
	call	BuildFlashLsbPageTable
	call	FlashDieInfoInit
	movzbl	gFlashInterfaceMode, %eax
	testb	$4, %al
	jne	.L2893
.L2882:
	movl	gpNandParaInfo, %eax
	movzbl	20(%eax), %eax
	call	FlashBchSel
	movb	$24, gNandIDBResBlkNum
	call	FlashLoadFactorBbt
	testl	%eax, %eax
	jne	.L2894
.L2884:
	movl	gpNandParaInfo, %ecx
	movzbl	12(%ecx), %eax
	movw	%ax, gNandPhyInfo+8
	movzbl	7(%ecx), %eax
	movl	%eax, gNandPhyInfo+4
	movl	IDByte, %eax
	movl	%eax, gNandPhyInfo
	movzbl	gNandMaxDie, %eax
	movw	%ax, gNandPhyInfo+10
	movzbl	13(%ecx), %eax
	movw	%ax, gNandPhyInfo+12
	movzwl	14(%ecx), %eax
	movw	%ax, gNandPhyInfo+14
	movzwl	10(%ecx), %eax
	movw	%ax, gNandPhyInfo+16
	movzwl	10(%ecx), %eax
	movzbl	12(%ecx), %ebx
	movl	%eax, %edx
	sarl	$31, %edx
	idivl	%ebx
	movw	%ax, gNandPhyInfo+18
	movzbl	9(%ecx), %eax
	movw	%ax, gNandPhyInfo+20
	movzbl	9(%ecx), %eax
	imulw	10(%ecx), %ax
	movw	$512, gNandPhyInfo+24
	movw	%ax, gNandPhyInfo+22
	movzbl	gNandIDBResBlkNum, %eax
	movw	%ax, gNandPhyInfo+26
	movl	$40, %eax
	call	FlashTimingCfg
	call	FlashSuspend
	xorl	%eax, %eax
.L2863:
	addl	$8, %esp
	popl	%ebx
	popl	%edi
	popl	%ebp
	ret
	.p2align 4,,7
	.p2align 3
.L2893:
	cmpb	$0, gFlashToggleModeEn
	jne	.L2883
	testb	$1, %al
	jne	.L2882
.L2883:
	movl	gNandFlashInfoBlockAddr, %edx
	xorl	%eax, %eax
	call	FlashDdrParaScan
	jmp	.L2882
	.p2align 4,,7
	.p2align 3
.L2894:
	call	FlashMakeFactorBbt
	.p2align 4,,8
	jmp	.L2884
	.p2align 4,,7
	.p2align 3
.L2872:
	cmpb	$17, %al
	.p2align 4,,6
	je	.L2895
	cmpb	$65, %al
	.p2align 4,,7
	je	.L2874
	cmpb	$33, %al
	.p2align 4,,5
	je	.L2874
	cmpb	$66, %al
	.p2align 4,,5
	je	.L2874
	cmpb	$34, %al
	.p2align 4,,5
	je	.L2876
	cmpb	$67, %al
	.p2align 4,,5
	je	.L2876
	cmpb	$35, %al
	.p2align 4,,5
	je	.L2877
	cmpb	$68, %al
	.p2align 4,,5
	je	.L2877
	cmpb	$49, %al
	.p2align 4,,5
	jne	.L2871
	movl	$SamsungReadRetrial, gpReadRetrial
	jmp	.L2871
	.p2align 4,,7
	.p2align 3
.L2891:
	movb	$1, gFlashToggleModeEn
	call	FlashSetInterfaceMode
	movzbl	gFlashInterfaceMode, %eax
	call	NandcSetMode
	call	FlashLoadPhyInfo
	testl	%eax, %eax
	je	.L2867
	.p2align 4,,6
	jmp	.L2896
	.p2align 4,,7
	.p2align 3
.L2895:
	movl	$MicronReadRetrial, gpReadRetrial
	jmp	.L2871
	.p2align 4,,7
	.p2align 3
.L2874:
	movl	$ToshibaReadRetrial, gpReadRetrial
	movb	$4, g_maxRegNum
	movb	$7, g_maxRetryCount
	jmp	.L2871
	.p2align 4,,7
	.p2align 3
.L2892:
	movl	$4, %eax
	movb	$1, gFlashToggleModeEn
	call	NandcSetMode
	movb	$1, gFlashToggleModeEn
	call	FlashLoadPhyInfo
	testl	%eax, %eax
	je	.L2867
	movl	$1, %eax
	call	NandcSetMode
	movb	$0, gFlashToggleModeEn
	jmp	.L2866
	.p2align 4,,7
	.p2align 3
.L2876:
	cmpb	$68, %al
	movl	$ToshibaReadRetrial, gpReadRetrial
	movb	$7, g_maxRetryCount
	je	.L2886
	cmpb	$35, %al
	je	.L2886
.L2879:
	subl	$67, %eax
	cmpb	$2, %al
	sbbl	%eax, %eax
	addl	$5, %eax
	movb	%al, g_maxRegNum
	jmp	.L2871
	.p2align 4,,7
	.p2align 3
.L2877:
	movl	$ToshibaReadRetrial, gpReadRetrial
.L2886:
	movb	$17, g_maxRetryCount
	jmp	.L2879
	.size	FlashInit, .-FlashInit
	.p2align 4,,15
	.globl	FTLInit
	.type	FTLInit, @function
FTLInit:
	pushl	%ebp
	movl	%esp, %ebp
	pushl	%ebx
	movl	%eax, %ebx
	call	rk_nandc_int_init
	movl	%ebx, %eax
	call	FlashInit
	testl	%eax, %eax
	movl	%eax, %ebx
	jne	.L2898
	movl	$gNandPhyInfo, %eax
	call	FtlInit
.L2898:
	movl	%ebx, %eax
	popl	%ebx
	popl	%ebp
	ret
	.size	FTLInit, .-FTLInit
	.section	.rodata.str1.1
.LC128:
	.string	"\0013NandInit fail\n"
	.text
	.p2align 4,,15
	.globl	init_nand_blk_dev
	.type	init_nand_blk_dev, @function
init_nand_blk_dev:
	pushl	%ebp
	movl	%esp, %ebp
	subl	$4, %esp
	movl	%eax, mytr+116
	movl	%edx, %eax
	call	FTLInit
	testl	%eax, %eax
	jne	.L2902
	movl	$mytr, %eax
	movl	$0, TotleReadData
	movl	$0, TotleWriteData
	movl	$0, TotleWriteCount
	movl	$0, TotleReadCount
	movl	$0, TotleCachePreReadCount
	movl	$0, TotleCacheMatchCount
	call	nand_blk_register
	leave
	ret
.L2902:
	movl	$.LC128, (%esp)
	call	printk
	orl	$-1, %eax
	leave
	ret
	.size	init_nand_blk_dev, .-init_nand_blk_dev
	.globl	NandFlashParaTbl
	.data
	.align 32
	.type	NandFlashParaTbl, @object
	.size	NandFlashParaTbl, 1920
NandFlashParaTbl:
	.byte	6
	.byte	44
	.byte	100
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	68
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	1064
	.value	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	104
	.byte	4
	.byte	74
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	5
	.byte	44
	.byte	-120
	.byte	4
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-88
	.byte	5
	.byte	-53
	.byte	-87
	.byte	0
	.byte	4
	.byte	2
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	104
	.byte	4
	.byte	70
	.byte	-119
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	72
	.byte	4
	.byte	74
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	1024
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-124
	.byte	100
	.byte	60
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.value	512
	.byte	2
	.byte	2
	.value	1024
	.value	991
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-124
	.byte	100
	.byte	84
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.value	512
	.byte	2
	.byte	2
	.value	1024
	.value	3039
	.byte	4
	.byte	17
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-41
	.byte	-108
	.byte	62
	.byte	-124
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.value	128
	.byte	2
	.byte	2
	.value	4096
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	72
	.byte	4
	.byte	70
	.byte	-123
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	1024
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-120
	.byte	5
	.byte	-58
	.byte	-119
	.byte	0
	.byte	4
	.byte	2
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	5
	.byte	44
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	104
	.byte	0
	.byte	39
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.value	128
	.byte	1
	.byte	2
	.value	2048
	.value	287
	.byte	0
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-124
	.byte	-59
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	2
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	44
	.byte	-43
	.byte	-47
	.byte	-90
	.byte	104
	.byte	0
	.byte	4
	.byte	2
	.byte	8
	.value	64
	.byte	1
	.byte	2
	.value	2048
	.value	279
	.byte	0
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-46
	.byte	4
	.byte	67
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	473
	.byte	1
	.byte	1
	.byte	24
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-61
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	1024
	.value	473
	.byte	1
	.byte	2
	.byte	40
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-111
	.byte	96
	.byte	68
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	1046
	.value	473
	.byte	1
	.byte	3
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-60
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2090
	.value	473
	.byte	1
	.byte	4
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-21
	.byte	116
	.byte	68
	.byte	2
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1066
	.value	473
	.byte	1
	.byte	3
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-43
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-60
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	530
	.value	473
	.byte	1
	.byte	3
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-102
	.byte	116
	.byte	66
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	1024
	.value	281
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-34
	.byte	20
	.byte	-89
	.byte	66
	.byte	74
	.byte	2
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1060
	.value	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-41
	.byte	20
	.byte	-98
	.byte	52
	.byte	74
	.byte	2
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	1056
	.value	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-89
	.byte	66
	.byte	72
	.byte	2
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1060
	.value	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	100
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	-124
	.byte	7
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	104
	.byte	36
	.byte	74
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	104
	.byte	4
	.byte	74
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	-41
	.byte	-108
	.byte	62
	.byte	-124
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-119
	.byte	104
	.byte	4
	.byte	70
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.value	256
	.byte	2
	.byte	2
	.value	2048
	.value	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	1060
	.value	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	2092
	.value	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	58
	.byte	-123
	.byte	-109
	.byte	118
	.byte	87
	.byte	1
	.byte	2
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	2092
	.value	1505
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-43
	.byte	-124
	.byte	50
	.byte	114
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	1
	.value	2056
	.value	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-108
	.byte	50
	.byte	118
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2058
	.value	1489
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-126
	.byte	118
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2062
	.value	1489
	.byte	1
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	80
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1066
	.value	1497
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	58
	.byte	-107
	.byte	-109
	.byte	122
	.byte	80
	.byte	1
	.byte	2
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1066
	.value	1497
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-108
	.byte	50
	.byte	118
	.byte	85
	.byte	1
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2050
	.value	401
	.byte	2
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1058
	.value	1497
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	80
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	1060
	.value	1473
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1074
	.value	1497
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-104
	.byte	58
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	2106
	.value	1497
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	87
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1058
	.value	1497
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	1060
	.value	1473
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-34
	.byte	-92
	.byte	-126
	.byte	118
	.byte	86
	.byte	8
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2082
	.value	473
	.byte	1
	.byte	65
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	80
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1066
	.value	1497
	.byte	2
	.byte	67
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	80
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	1
	.value	1060
	.value	1473
	.byte	2
	.byte	67
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-34
	.byte	-92
	.byte	-126
	.byte	118
	.byte	-41
	.byte	8
	.byte	1
	.byte	16
	.value	256
	.byte	2
	.byte	2
	.value	2090
	.value	1241
	.byte	1
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	58
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	2106
	.value	1497
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	8
	.byte	1
	.byte	32
	.value	256
	.byte	2
	.byte	2
	.value	1074
	.value	1497
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-41
	.byte	-108
	.byte	126
	.byte	100
	.byte	68
	.byte	0
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2048
	.value	473
	.byte	2
	.byte	49
	.byte	60
	.byte	36
	.byte	3
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-34
	.byte	-43
	.byte	126
	.byte	104
	.byte	68
	.byte	0
	.byte	2
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2048
	.value	505
	.byte	2
	.byte	49
	.byte	60
	.byte	36
	.byte	3
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-41
	.byte	-108
	.byte	122
	.byte	84
	.byte	67
	.byte	0
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2076
	.value	409
	.byte	2
	.byte	0
	.byte	40
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-34
	.byte	-43
	.byte	122
	.byte	88
	.byte	67
	.byte	0
	.byte	2
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2076
	.value	441
	.byte	2
	.byte	0
	.byte	40
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-43
	.byte	-108
	.byte	118
	.byte	84
	.byte	67
	.byte	0
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	1038
	.value	281
	.byte	2
	.byte	0
	.byte	24
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.byte	6
	.byte	-20
	.byte	-41
	.byte	20
	.byte	118
	.byte	84
	.byte	-62
	.byte	0
	.byte	1
	.byte	16
	.value	128
	.byte	2
	.byte	2
	.value	2076
	.value	1169
	.byte	2
	.byte	0
	.byte	24
	.byte	40
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.zero	5
	.globl	NandOptPara
	.align 32
	.type	NandOptPara, @object
	.size	NandOptPara, 128
NandOptPara:
	.byte	1
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	0
	.byte	50
	.byte	17
	.byte	-128
	.byte	112
	.byte	120
	.byte	120
	.byte	3
	.byte	1
	.byte	0
	.zero	14
	.byte	2
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	0
	.byte	0
	.byte	17
	.byte	-127
	.byte	112
	.byte	-15
	.byte	-14
	.byte	0
	.byte	0
	.byte	0
	.zero	14
	.byte	3
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	96
	.byte	96
	.byte	17
	.byte	-127
	.byte	112
	.byte	-15
	.byte	-14
	.byte	0
	.byte	0
	.byte	0
	.zero	14
	.byte	4
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	96
	.byte	96
	.byte	17
	.byte	-127
	.byte	112
	.byte	112
	.byte	112
	.byte	0
	.byte	0
	.byte	0
	.zero	14
	.globl	gp_nandc_info
	.bss
	.align 4
	.type	gp_nandc_info, @object
	.size	gp_nandc_info, 4
gp_nandc_info:
	.zero	4
	.globl	nand_blktrans_ops
	.data
	.align 32
	.type	nand_blktrans_ops, @object
	.size	nand_blktrans_ops, 48
nand_blktrans_ops:
	.long	nand_open
	.long	nand_release
	.long	nand_ioctl
	.zero	32
	.long	0
	.globl	rk_ftl_gc_do
	.bss
	.align 4
	.type	rk_ftl_gc_do, @object
	.size	rk_ftl_gc_do, 4
rk_ftl_gc_do:
	.zero	4
	.globl	rk_ftl_gc_jiffies
	.align 4
	.type	rk_ftl_gc_jiffies, @object
	.size	rk_ftl_gc_jiffies, 4
rk_ftl_gc_jiffies:
	.zero	4
	.globl	g_nand_bg_info
	.align 32
	.type	g_nand_bg_info, @object
	.size	g_nand_bg_info, 56
g_nand_bg_info:
	.zero	56
	.globl	g_nand_bg_req
	.align 32
	.type	g_nand_bg_req, @object
	.size	g_nand_bg_req, 200
g_nand_bg_req:
	.zero	200
	.globl	g_rk_nand_ops_mutex
	.align 4
	.type	g_rk_nand_ops_mutex, @object
	.size	g_rk_nand_ops_mutex, 16
g_rk_nand_ops_mutex:
	.zero	16
	.globl	nand_handle
	.align 4
	.type	nand_handle, @object
	.size	nand_handle, 4
nand_handle:
	.zero	4
	.globl	TotleCacheMatchCount
	.align 4
	.type	TotleCacheMatchCount, @object
	.size	TotleCacheMatchCount, 4
TotleCacheMatchCount:
	.zero	4
	.globl	TotleCachePreReadCount
	.align 4
	.type	TotleCachePreReadCount, @object
	.size	TotleCachePreReadCount, 4
TotleCachePreReadCount:
	.zero	4
	.globl	TotleWriteCount
	.align 4
	.type	TotleWriteCount, @object
	.size	TotleWriteCount, 4
TotleWriteCount:
	.zero	4
	.globl	TotleReadCount
	.align 4
	.type	TotleReadCount, @object
	.size	TotleReadCount, 4
TotleReadCount:
	.zero	4
	.globl	TotleWriteData
	.align 4
	.type	TotleWriteData, @object
	.size	TotleWriteData, 4
TotleWriteData:
	.zero	4
	.globl	TotleReadData
	.align 4
	.type	TotleReadData, @object
	.size	TotleReadData, 4
TotleReadData:
	.zero	4
	.globl	g_max_part_num
	.align 4
	.type	g_max_part_num, @object
	.size	g_max_part_num, 4
g_max_part_num:
	.zero	4
	.globl	rk_timer_add
	.align 4
	.type	rk_timer_add, @object
	.size	rk_timer_add, 4
rk_timer_add:
	.zero	4
	.globl	nandc0_llp_completed_flag
	.data
	.align 4
	.type	nandc0_llp_completed_flag, @object
	.size	nandc0_llp_completed_flag, 4
nandc0_llp_completed_flag:
	.long	1
	.globl	nandc0_ready_completed_flag
	.align 4
	.type	nandc0_ready_completed_flag, @object
	.size	nandc0_ready_completed_flag, 4
nandc0_ready_completed_flag:
	.long	1
	.globl	nandc0_xfer_completed_flag
	.align 4
	.type	nandc0_xfer_completed_flag, @object
	.size	nandc0_xfer_completed_flag, 4
nandc0_xfer_completed_flag:
	.long	1
	.globl	gc_ink_free_return_value
	.bss
	.align 2
	.type	gc_ink_free_return_value, @object
	.size	gc_ink_free_return_value, 2
gc_ink_free_return_value:
	.zero	2
	.globl	FtlUpdateVaildLpnCount
	.align 2
	.type	FtlUpdateVaildLpnCount, @object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.zero	2
	.globl	g_ect_tbl_power_up_flush
	.align 2
	.type	g_ect_tbl_power_up_flush, @object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.zero	2
	.globl	last_cache_match_count
	.align 4
	.type	last_cache_match_count, @object
	.size	last_cache_match_count, 4
last_cache_match_count:
	.zero	4
	.globl	power_up_flag
	.data
	.align 4
	.type	power_up_flag, @object
	.size	power_up_flag, 4
power_up_flag:
	.long	1
	.globl	g_LowFormat
	.bss
	.align 4
	.type	g_LowFormat, @object
	.size	g_LowFormat, 4
g_LowFormat:
	.zero	4
	.globl	gFtlInitStatus
	.data
	.align 4
	.type	gFtlInitStatus, @object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.long	-1
	.globl	DeviceCapacity
	.bss
	.align 4
	.type	DeviceCapacity, @object
	.size	DeviceCapacity, 4
DeviceCapacity:
	.zero	4
	.globl	g_llp_page_buf
	.align 32
	.type	g_llp_page_buf, @object
	.size	g_llp_page_buf, 2048
g_llp_page_buf:
	.zero	2048
	.globl	g_llp_tbl_cpu
	.align 4
	.type	g_llp_tbl_cpu, @object
	.size	g_llp_tbl_cpu, 4
g_llp_tbl_cpu:
	.zero	4
	.globl	g_llp_tbl_dma
	.align 4
	.type	g_llp_tbl_dma, @object
	.size	g_llp_tbl_dma, 4
g_llp_tbl_dma:
	.zero	4
	.globl	g_llp_spare_buffer_dma
	.align 4
	.type	g_llp_spare_buffer_dma, @object
	.size	g_llp_spare_buffer_dma, 4
g_llp_spare_buffer_dma:
	.zero	4
	.globl	g_llp_spare_buffer_cpu
	.align 4
	.type	g_llp_spare_buffer_cpu, @object
	.size	g_llp_spare_buffer_cpu, 4
g_llp_spare_buffer_cpu:
	.zero	4
	.globl	g_llp_data_buffer_dma
	.align 4
	.type	g_llp_data_buffer_dma, @object
	.size	g_llp_data_buffer_dma, 4
g_llp_data_buffer_dma:
	.zero	4
	.globl	g_llp_data_buffer_cpu
	.align 4
	.type	g_llp_data_buffer_cpu, @object
	.size	g_llp_data_buffer_cpu, 4
g_llp_data_buffer_cpu:
	.zero	4
	.globl	ToshibaRefValue
	.data
	.type	ToshibaRefValue, @object
	.size	ToshibaRefValue, 8
ToshibaRefValue:
	.byte	0
	.byte	4
	.byte	124
	.byte	120
	.byte	116
	.byte	8
	.byte	12
	.byte	112
	.globl	Toshiba15RefValue
	.align 32
	.type	Toshiba15RefValue, @object
	.size	Toshiba15RefValue, 95
Toshiba15RefValue:
	.byte	4
	.byte	5
	.byte	6
	.byte	7
	.byte	13
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	2
	.byte	4
	.byte	2
	.byte	0
	.byte	0
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	0
	.byte	124
	.byte	0
	.byte	124
	.byte	124
	.byte	0
	.byte	122
	.byte	0
	.byte	122
	.byte	122
	.byte	0
	.byte	11
	.byte	126
	.byte	118
	.byte	116
	.byte	0
	.byte	120
	.byte	2
	.byte	120
	.byte	122
	.byte	0
	.byte	126
	.byte	4
	.byte	126
	.byte	122
	.byte	0
	.byte	16
	.byte	118
	.byte	114
	.byte	112
	.byte	0
	.byte	118
	.byte	4
	.byte	118
	.byte	120
	.byte	0
	.byte	4
	.byte	4
	.byte	4
	.byte	118
	.byte	0
	.byte	2
	.byte	0
	.byte	126
	.byte	124
	.byte	0
	.byte	6
	.byte	10
	.byte	6
	.byte	2
	.byte	0
	.byte	116
	.byte	124
	.byte	116
	.byte	118
	.byte	0
	.byte	4
	.byte	4
	.byte	124
	.byte	126
	.byte	0
	.byte	0
	.byte	124
	.byte	120
	.byte	120
	.byte	0
	.byte	124
	.byte	118
	.byte	116
	.byte	114
	.byte	0
	.globl	ToshibaA19RefValue
	.align 32
	.type	ToshibaA19RefValue, @object
	.size	ToshibaA19RefValue, 45
ToshibaA19RefValue:
	.byte	4
	.byte	5
	.byte	6
	.byte	7
	.byte	13
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	4
	.byte	4
	.byte	124
	.byte	126
	.byte	0
	.byte	0
	.byte	124
	.byte	120
	.byte	120
	.byte	0
	.byte	124
	.byte	118
	.byte	116
	.byte	114
	.byte	0
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	0
	.byte	11
	.byte	126
	.byte	118
	.byte	116
	.byte	0
	.byte	16
	.byte	118
	.byte	114
	.byte	112
	.byte	0
	.byte	2
	.byte	0
	.byte	126
	.byte	124
	.byte	0
	.globl	SamsungRefValue
	.align 32
	.type	SamsungRefValue, @object
	.size	SamsungRefValue, 64
SamsungRefValue:
	.byte	-89
	.byte	-92
	.byte	-91
	.byte	-90
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	5
	.byte	10
	.byte	0
	.byte	0
	.byte	40
	.byte	0
	.byte	-20
	.byte	-40
	.byte	-19
	.byte	-11
	.byte	-19
	.byte	-26
	.byte	10
	.byte	15
	.byte	5
	.byte	0
	.byte	15
	.byte	10
	.byte	-5
	.byte	-20
	.byte	-24
	.byte	-17
	.byte	-24
	.byte	-36
	.byte	-15
	.byte	-5
	.byte	-2
	.byte	-16
	.byte	10
	.byte	0
	.byte	-5
	.byte	-20
	.byte	-48
	.byte	-30
	.byte	-48
	.byte	-62
	.byte	20
	.byte	15
	.byte	-5
	.byte	-20
	.byte	-24
	.byte	-5
	.byte	-24
	.byte	-36
	.byte	30
	.byte	20
	.byte	-5
	.byte	-20
	.byte	-5
	.byte	-1
	.byte	-5
	.byte	-8
	.byte	7
	.byte	12
	.byte	2
	.byte	0
	.globl	refValueDefault
	.type	refValueDefault, @object
	.size	refValueDefault, 28
refValueDefault:
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	6
	.byte	10
	.byte	6
	.byte	0
	.byte	-3
	.byte	-7
	.byte	-8
	.byte	0
	.byte	-6
	.byte	-13
	.byte	-15
	.byte	0
	.byte	-11
	.byte	-20
	.byte	-23
	.byte	0
	.byte	0
	.byte	-26
	.byte	-30
	.byte	0
	.byte	0
	.byte	-32
	.byte	-37
	.globl	FbbtBlk
	.bss
	.align 2
	.type	FbbtBlk, @object
	.size	FbbtBlk, 8
FbbtBlk:
	.zero	8
	.globl	random_seed
	.data
	.align 32
	.type	random_seed, @object
	.size	random_seed, 256
random_seed:
	.value	22378
	.value	1512
	.value	25245
	.value	17827
	.value	25756
	.value	19440
	.value	9026
	.value	10030
	.value	29528
	.value	20467
	.value	29676
	.value	24432
	.value	31328
	.value	6872
	.value	13426
	.value	13842
	.value	8783
	.value	1108
	.value	782
	.value	28837
	.value	30729
	.value	9505
	.value	18676
	.value	23085
	.value	18730
	.value	1085
	.value	32609
	.value	14697
	.value	20858
	.value	15170
	.value	30365
	.value	1607
	.value	32298
	.value	4995
	.value	18905
	.value	1976
	.value	9592
	.value	20204
	.value	17443
	.value	13615
	.value	23330
	.value	29369
	.value	13947
	.value	9398
	.value	32398
	.value	8984
	.value	27600
	.value	21785
	.value	6019
	.value	6311
	.value	31598
	.value	30210
	.value	19327
	.value	13896
	.value	11347
	.value	27545
	.value	3107
	.value	26575
	.value	32270
	.value	19852
	.value	20601
	.value	8349
	.value	9290
	.value	29819
	.value	13579
	.value	3661
	.value	28676
	.value	27331
	.value	32574
	.value	8693
	.value	31253
	.value	9081
	.value	5399
	.value	6842
	.value	20087
	.value	5537
	.value	1274
	.value	11617
	.value	9530
	.value	4866
	.value	8035
	.value	23219
	.value	1178
	.value	23272
	.value	7383
	.value	18944
	.value	12488
	.value	12871
	.value	29340
	.value	20532
	.value	11022
	.value	22514
	.value	228
	.value	22363
	.value	24978
	.value	14584
	.value	12138
	.value	3092
	.value	17916
	.value	16863
	.value	14554
	.value	31457
	.value	29474
	.value	25311
	.value	24121
	.value	3684
	.value	28037
	.value	22865
	.value	22839
	.value	25217
	.value	13217
	.value	27186
	.value	14938
	.value	11180
	.value	29754
	.value	24180
	.value	15150
	.value	32455
	.value	20434
	.value	23848
	.value	29983
	.value	16120
	.value	14769
	.value	20041
	.value	29803
	.value	28406
	.value	17598
	.value	28087
	.globl	gNandParaInfo
	.align 32
	.type	gNandParaInfo, @object
	.size	gNandParaInfo, 32
gNandParaInfo:
	.byte	0
	.byte	0
	.zero	5
	.byte	0
	.byte	1
	.byte	8
	.value	128
	.byte	2
	.byte	1
	.value	2048
	.value	0
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.zero	5
	.globl	g_power_lost_recovery_flag
	.bss
	.align 2
	.type	g_power_lost_recovery_flag, @object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.zero	2
	.globl	c_mlc_erase_count_value
	.align 2
	.type	c_mlc_erase_count_value, @object
	.size	c_mlc_erase_count_value, 2
c_mlc_erase_count_value:
	.zero	2
	.globl	g_recovery_ppa_tbl
	.align 32
	.type	g_recovery_ppa_tbl, @object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.zero	128
	.globl	g_recovery_page_min_ver
	.align 4
	.type	g_recovery_page_min_ver, @object
	.size	g_recovery_page_min_ver, 4
g_recovery_page_min_ver:
	.zero	4
	.globl	g_recovery_page_num
	.align 4
	.type	g_recovery_page_num, @object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.zero	4
	.globl	g_cur_erase_blk
	.align 4
	.type	g_cur_erase_blk, @object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.zero	4
	.globl	g_gc_skip_write_count
	.align 4
	.type	g_gc_skip_write_count, @object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.zero	4
	.globl	g_gc_head_data_block_count
	.align 4
	.type	g_gc_head_data_block_count, @object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.zero	4
	.globl	g_gc_head_data_block
	.align 4
	.type	g_gc_head_data_block, @object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.zero	4
	.globl	g_ftl_nand_free_count
	.align 4
	.type	g_ftl_nand_free_count, @object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.zero	4
	.globl	g_in_swl_replace
	.align 4
	.type	g_in_swl_replace, @object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.zero	4
	.globl	g_in_gc_progress
	.align 4
	.type	g_in_gc_progress, @object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.zero	4
	.globl	g_all_blk_used_slc_mode
	.align 4
	.type	g_all_blk_used_slc_mode, @object
	.size	g_all_blk_used_slc_mode, 4
g_all_blk_used_slc_mode:
	.zero	4
	.globl	g_max_erase_count
	.align 4
	.type	g_max_erase_count, @object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.zero	4
	.globl	g_totle_sys_slc_erase_count
	.align 4
	.type	g_totle_sys_slc_erase_count, @object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.zero	4
	.globl	g_totle_slc_erase_count
	.align 4
	.type	g_totle_slc_erase_count, @object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
	.zero	4
	.globl	g_min_erase_count
	.align 4
	.type	g_min_erase_count, @object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.zero	4
	.globl	g_totle_avg_erase_count
	.align 4
	.type	g_totle_avg_erase_count, @object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.zero	4
	.globl	g_totle_mlc_erase_count
	.align 4
	.type	g_totle_mlc_erase_count, @object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.zero	4
	.globl	g_totle_l2p_write_count
	.align 4
	.type	g_totle_l2p_write_count, @object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.zero	4
	.globl	g_totle_cache_write_count
	.align 4
	.type	g_totle_cache_write_count, @object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.zero	4
	.globl	g_tmp_data_superblock_id
	.align 2
	.type	g_tmp_data_superblock_id, @object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.zero	2
	.globl	g_totle_read_page_count
	.align 4
	.type	g_totle_read_page_count, @object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.zero	4
	.globl	g_totle_discard_page_count
	.align 4
	.type	g_totle_discard_page_count, @object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.zero	4
	.globl	g_totle_read_sector
	.align 4
	.type	g_totle_read_sector, @object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.zero	4
	.globl	g_totle_write_sector
	.align 4
	.type	g_totle_write_sector, @object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.zero	4
	.globl	g_totle_write_page_count
	.align 4
	.type	g_totle_write_page_count, @object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.zero	4
	.globl	g_totle_gc_page_count
	.align 4
	.type	g_totle_gc_page_count, @object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.zero	4
	.globl	g_gc_blk_index
	.align 2
	.type	g_gc_blk_index, @object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.zero	2
	.globl	g_gc_merge_free_blk_threshold
	.align 2
	.type	g_gc_merge_free_blk_threshold, @object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.zero	2
	.globl	g_gc_free_blk_threshold
	.align 2
	.type	g_gc_free_blk_threshold, @object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.zero	2
	.globl	g_gc_bad_block_temp_tbl
	.align 32
	.type	g_gc_bad_block_temp_tbl, @object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.zero	34
	.globl	g_gc_bad_block_gc_index
	.align 2
	.type	g_gc_bad_block_gc_index, @object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.zero	2
	.globl	g_gc_bad_block_temp_num
	.align 2
	.type	g_gc_bad_block_temp_num, @object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.zero	2
	.globl	g_gc_next_blk_1
	.align 2
	.type	g_gc_next_blk_1, @object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.zero	2
	.globl	g_gc_next_blk
	.align 2
	.type	g_gc_next_blk, @object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.zero	2
	.globl	g_gc_cur_blk_max_valid_pages
	.align 2
	.type	g_gc_cur_blk_max_valid_pages, @object
	.size	g_gc_cur_blk_max_valid_pages, 2
g_gc_cur_blk_max_valid_pages:
	.zero	2
	.globl	g_gc_cur_blk_valid_pages
	.align 2
	.type	g_gc_cur_blk_valid_pages, @object
	.size	g_gc_cur_blk_valid_pages, 2
g_gc_cur_blk_valid_pages:
	.zero	2
	.globl	g_gc_page_offset
	.align 2
	.type	g_gc_page_offset, @object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.zero	2
	.globl	g_gc_blk_num
	.align 2
	.type	g_gc_blk_num, @object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.zero	2
	.globl	p_gc_blk_tbl
	.align 4
	.type	p_gc_blk_tbl, @object
	.size	p_gc_blk_tbl, 4
p_gc_blk_tbl:
	.zero	4
	.globl	p_gc_page_info
	.align 4
	.type	p_gc_page_info, @object
	.size	p_gc_page_info, 4
p_gc_page_info:
	.zero	4
	.globl	g_sys_ext_data
	.align 32
	.type	g_sys_ext_data, @object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.zero	512
	.globl	g_sys_save_data
	.align 32
	.type	g_sys_save_data, @object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.zero	48
	.globl	gp_last_act_superblock
	.align 4
	.type	gp_last_act_superblock, @object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
	.zero	4
	.globl	g_gc_superblock
	.align 32
	.type	g_gc_superblock, @object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.zero	48
	.globl	g_gc_temp_superblock
	.align 32
	.type	g_gc_temp_superblock, @object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.zero	48
	.globl	g_buffer_superblock
	.align 32
	.type	g_buffer_superblock, @object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.zero	48
	.globl	g_active_superblock
	.align 32
	.type	g_active_superblock, @object
	.size	g_active_superblock, 48
g_active_superblock:
	.zero	48
	.globl	g_num_data_superblocks
	.align 2
	.type	g_num_data_superblocks, @object
	.size	g_num_data_superblocks, 2
g_num_data_superblocks:
	.zero	2
	.globl	g_num_free_superblocks
	.align 2
	.type	g_num_free_superblocks, @object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.zero	2
	.globl	p_data_block_list_tail
	.align 4
	.type	p_data_block_list_tail, @object
	.size	p_data_block_list_tail, 4
p_data_block_list_tail:
	.zero	4
	.globl	p_data_block_list_head
	.align 4
	.type	p_data_block_list_head, @object
	.size	p_data_block_list_head, 4
p_data_block_list_head:
	.zero	4
	.globl	p_free_data_block_list_head
	.align 4
	.type	p_free_data_block_list_head, @object
	.size	p_free_data_block_list_head, 4
p_free_data_block_list_head:
	.zero	4
	.globl	p_data_block_list_table
	.align 4
	.type	p_data_block_list_table, @object
	.size	p_data_block_list_table, 4
p_data_block_list_table:
	.zero	4
	.globl	g_l2p_last_update_region_id
	.align 2
	.type	g_l2p_last_update_region_id, @object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.zero	2
	.globl	p_l2p_map_buf
	.align 4
	.type	p_l2p_map_buf, @object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.zero	4
	.globl	p_l2p_ram_map
	.align 4
	.type	p_l2p_ram_map, @object
	.size	p_l2p_ram_map, 4
p_l2p_ram_map:
	.zero	4
	.globl	g_totle_vendor_block
	.align 2
	.type	g_totle_vendor_block, @object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.zero	2
	.globl	p_vendor_region_ppn_table
	.align 4
	.type	p_vendor_region_ppn_table, @object
	.size	p_vendor_region_ppn_table, 4
p_vendor_region_ppn_table:
	.zero	4
	.globl	p_vendor_block_ver_table
	.align 4
	.type	p_vendor_block_ver_table, @object
	.size	p_vendor_block_ver_table, 4
p_vendor_block_ver_table:
	.zero	4
	.globl	p_vendor_block_valid_page_count
	.align 4
	.type	p_vendor_block_valid_page_count, @object
	.size	p_vendor_block_valid_page_count, 4
p_vendor_block_valid_page_count:
	.zero	4
	.globl	p_vendor_block_table
	.align 4
	.type	p_vendor_block_table, @object
	.size	p_vendor_block_table, 4
p_vendor_block_table:
	.zero	4
	.globl	g_totle_map_block
	.align 2
	.type	g_totle_map_block, @object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.zero	2
	.globl	p_map_region_ppn_table
	.align 4
	.type	p_map_region_ppn_table, @object
	.size	p_map_region_ppn_table, 4
p_map_region_ppn_table:
	.zero	4
	.globl	p_map_block_ver_table
	.align 4
	.type	p_map_block_ver_table, @object
	.size	p_map_block_ver_table, 4
p_map_block_ver_table:
	.zero	4
	.globl	p_map_block_valid_page_count
	.align 4
	.type	p_map_block_valid_page_count, @object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.zero	4
	.globl	p_map_block_table
	.align 4
	.type	p_map_block_table, @object
	.size	p_map_block_table, 4
p_map_block_table:
	.zero	4
	.globl	p_valid_page_count_check_table
	.align 4
	.type	p_valid_page_count_check_table, @object
	.size	p_valid_page_count_check_table, 4
p_valid_page_count_check_table:
	.zero	4
	.globl	p_valid_page_count_table
	.align 4
	.type	p_valid_page_count_table, @object
	.size	p_valid_page_count_table, 4
p_valid_page_count_table:
	.zero	4
	.globl	g_totle_swl_count
	.align 4
	.type	g_totle_swl_count, @object
	.size	g_totle_swl_count, 4
g_totle_swl_count:
	.zero	4
	.globl	p_swl_mul_table
	.align 4
	.type	p_swl_mul_table, @object
	.size	p_swl_mul_table, 4
p_swl_mul_table:
	.zero	4
	.globl	p_erase_count_table
	.align 4
	.type	p_erase_count_table, @object
	.size	p_erase_count_table, 4
p_erase_count_table:
	.zero	4
	.globl	g_ect_tbl_info_size
	.align 2
	.type	g_ect_tbl_info_size, @object
	.size	g_ect_tbl_info_size, 2
g_ect_tbl_info_size:
	.zero	2
	.globl	gp_ect_tbl_info
	.align 4
	.type	gp_ect_tbl_info, @object
	.size	gp_ect_tbl_info, 4
gp_ect_tbl_info:
	.zero	4
	.globl	g_gc_num_req
	.align 4
	.type	g_gc_num_req, @object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.zero	4
	.globl	c_gc_page_buf_num
	.align 4
	.type	c_gc_page_buf_num, @object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
	.zero	4
	.globl	gp_gc_page_buf_info
	.align 4
	.type	gp_gc_page_buf_info, @object
	.size	gp_gc_page_buf_info, 4
gp_gc_page_buf_info:
	.zero	4
	.globl	p_gc_data_buf
	.align 4
	.type	p_gc_data_buf, @object
	.size	p_gc_data_buf, 4
p_gc_data_buf:
	.zero	4
	.globl	p_gc_spare_buf
	.align 4
	.type	p_gc_spare_buf, @object
	.size	p_gc_spare_buf, 4
p_gc_spare_buf:
	.zero	4
	.globl	p_io_spare_buf
	.align 4
	.type	p_io_spare_buf, @object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
	.zero	4
	.globl	p_io_data_buf_1
	.align 4
	.type	p_io_data_buf_1, @object
	.size	p_io_data_buf_1, 4
p_io_data_buf_1:
	.zero	4
	.globl	p_io_data_buf_0
	.align 4
	.type	p_io_data_buf_0, @object
	.size	p_io_data_buf_0, 4
p_io_data_buf_0:
	.zero	4
	.globl	p_sys_spare_buf
	.align 4
	.type	p_sys_spare_buf, @object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.zero	4
	.globl	p_vendor_data_buf
	.align 4
	.type	p_vendor_data_buf, @object
	.size	p_vendor_data_buf, 4
p_vendor_data_buf:
	.zero	4
	.globl	p_sys_data_buf_1
	.align 4
	.type	p_sys_data_buf_1, @object
	.size	p_sys_data_buf_1, 4
p_sys_data_buf_1:
	.zero	4
	.globl	p_sys_data_buf
	.align 4
	.type	p_sys_data_buf, @object
	.size	p_sys_data_buf, 4
p_sys_data_buf:
	.zero	4
	.globl	g_wr_page_num
	.align 4
	.type	g_wr_page_num, @object
	.size	g_wr_page_num, 4
g_wr_page_num:
	.zero	4
	.globl	req_wr_io
	.align 4
	.type	req_wr_io, @object
	.size	req_wr_io, 4
req_wr_io:
	.zero	4
	.globl	c_wr_page_buf_num
	.align 4
	.type	c_wr_page_buf_num, @object
	.size	c_wr_page_buf_num, 4
c_wr_page_buf_num:
	.zero	4
	.globl	p_wr_io_data_buf
	.align 4
	.type	p_wr_io_data_buf, @object
	.size	p_wr_io_data_buf, 4
p_wr_io_data_buf:
	.zero	4
	.globl	p_wr_io_spare_buf
	.align 4
	.type	p_wr_io_spare_buf, @object
	.size	p_wr_io_spare_buf, 4
p_wr_io_spare_buf:
	.zero	4
	.globl	p_plane_order_table
	.align 32
	.type	p_plane_order_table, @object
	.size	p_plane_order_table, 32
p_plane_order_table:
	.zero	32
	.globl	g_req_cache
	.align 4
	.type	g_req_cache, @object
	.size	g_req_cache, 4
g_req_cache:
	.zero	4
	.globl	req_gc_dst
	.align 4
	.type	req_gc_dst, @object
	.size	req_gc_dst, 4
req_gc_dst:
	.zero	4
	.globl	req_gc
	.align 4
	.type	req_gc, @object
	.size	req_gc, 4
req_gc:
	.zero	4
	.globl	req_erase
	.align 4
	.type	req_erase, @object
	.size	req_erase, 4
req_erase:
	.zero	4
	.globl	req_prgm
	.align 4
	.type	req_prgm, @object
	.size	req_prgm, 4
req_prgm:
	.zero	4
	.globl	req_read
	.align 4
	.type	req_read, @object
	.size	req_read, 4
req_read:
	.zero	4
	.globl	req_sys
	.align 32
	.type	req_sys, @object
	.size	req_sys, 36
req_sys:
	.zero	36
	.globl	gVendorBlkInfo
	.align 32
	.type	gVendorBlkInfo, @object
	.size	gVendorBlkInfo, 40
gVendorBlkInfo:
	.zero	40
	.globl	gL2pMapInfo
	.align 32
	.type	gL2pMapInfo, @object
	.size	gL2pMapInfo, 40
gL2pMapInfo:
	.zero	40
	.globl	gSysFreeQueue
	.align 32
	.type	gSysFreeQueue, @object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.zero	2056
	.globl	gSysInfo
	.align 4
	.type	gSysInfo, @object
	.size	gSysInfo, 12
gSysInfo:
	.zero	12
	.globl	gBbtInfo
	.align 32
	.type	gBbtInfo, @object
	.size	gBbtInfo, 60
gBbtInfo:
	.zero	60
	.globl	g_inkDie_check_enable
	.align 4
	.type	g_inkDie_check_enable, @object
	.size	g_inkDie_check_enable, 4
g_inkDie_check_enable:
	.zero	4
	.globl	g_SlcPartLbaEndSector
	.align 4
	.type	g_SlcPartLbaEndSector, @object
	.size	g_SlcPartLbaEndSector, 4
g_SlcPartLbaEndSector:
	.zero	4
	.globl	g_MaxLbn
	.align 4
	.type	g_MaxLbn, @object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.zero	4
	.globl	g_validLpn
	.align 4
	.type	g_validLpn, @object
	.size	g_validLpn, 4
g_validLpn:
	.zero	4
	.globl	g_MaxLpn
	.align 4
	.type	g_MaxLpn, @object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.zero	4
	.globl	g_MaxLbaSector
	.align 4
	.type	g_MaxLbaSector, @object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.zero	4
	.globl	g_GlobalDataVersion
	.align 4
	.type	g_GlobalDataVersion, @object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.zero	4
	.globl	g_GlobalSysVersion
	.align 4
	.type	g_GlobalSysVersion, @object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.zero	4
	.globl	ftl_gc_temp_power_lost_recovery_flag
	.align 4
	.type	ftl_gc_temp_power_lost_recovery_flag, @object
	.size	ftl_gc_temp_power_lost_recovery_flag, 4
ftl_gc_temp_power_lost_recovery_flag:
	.zero	4
	.globl	c_ftl_nand_max_data_blks
	.align 4
	.type	c_ftl_nand_max_data_blks, @object
	.size	c_ftl_nand_max_data_blks, 4
c_ftl_nand_max_data_blks:
	.zero	4
	.globl	c_ftl_nand_data_op_blks_per_plane
	.align 2
	.type	c_ftl_nand_data_op_blks_per_plane, @object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.zero	2
	.globl	c_ftl_nand_data_blks_per_plane
	.align 2
	.type	c_ftl_nand_data_blks_per_plane, @object
	.size	c_ftl_nand_data_blks_per_plane, 2
c_ftl_nand_data_blks_per_plane:
	.zero	2
	.globl	c_ftl_nand_max_sys_blks
	.align 4
	.type	c_ftl_nand_max_sys_blks, @object
	.size	c_ftl_nand_max_sys_blks, 4
c_ftl_nand_max_sys_blks:
	.zero	4
	.globl	c_ftl_nand_init_sys_blks_per_plane
	.align 4
	.type	c_ftl_nand_init_sys_blks_per_plane, @object
	.size	c_ftl_nand_init_sys_blks_per_plane, 4
c_ftl_nand_init_sys_blks_per_plane:
	.zero	4
	.globl	c_ftl_nand_sys_blks_per_plane
	.align 4
	.type	c_ftl_nand_sys_blks_per_plane, @object
	.size	c_ftl_nand_sys_blks_per_plane, 4
c_ftl_nand_sys_blks_per_plane:
	.zero	4
	.globl	c_ftl_nand_max_vendor_blks
	.align 2
	.type	c_ftl_nand_max_vendor_blks, @object
	.size	c_ftl_nand_max_vendor_blks, 2
c_ftl_nand_max_vendor_blks:
	.zero	2
	.globl	c_ftl_nand_max_map_blks
	.align 4
	.type	c_ftl_nand_max_map_blks, @object
	.size	c_ftl_nand_max_map_blks, 4
c_ftl_nand_max_map_blks:
	.zero	4
	.globl	c_ftl_nand_map_blks_per_plane
	.align 2
	.type	c_ftl_nand_map_blks_per_plane, @object
	.size	c_ftl_nand_map_blks_per_plane, 2
c_ftl_nand_map_blks_per_plane:
	.zero	2
	.globl	c_ftl_nand_vendor_region_num
	.align 2
	.type	c_ftl_nand_vendor_region_num, @object
	.size	c_ftl_nand_vendor_region_num, 2
c_ftl_nand_vendor_region_num:
	.zero	2
	.globl	c_ftl_nand_l2pmap_ram_region_num
	.align 2
	.type	c_ftl_nand_l2pmap_ram_region_num, @object
	.size	c_ftl_nand_l2pmap_ram_region_num, 2
c_ftl_nand_l2pmap_ram_region_num:
	.zero	2
	.globl	c_ftl_nand_map_region_num
	.align 2
	.type	c_ftl_nand_map_region_num, @object
	.size	c_ftl_nand_map_region_num, 2
c_ftl_nand_map_region_num:
	.zero	2
	.globl	c_ftl_nand_totle_phy_blks
	.align 4
	.type	c_ftl_nand_totle_phy_blks, @object
	.size	c_ftl_nand_totle_phy_blks, 4
c_ftl_nand_totle_phy_blks:
	.zero	4
	.globl	c_ftl_nand_reserved_blks
	.align 2
	.type	c_ftl_nand_reserved_blks, @object
	.size	c_ftl_nand_reserved_blks, 2
c_ftl_nand_reserved_blks:
	.zero	2
	.globl	c_ftl_nand_byte_pre_oob
	.align 2
	.type	c_ftl_nand_byte_pre_oob, @object
	.size	c_ftl_nand_byte_pre_oob, 2
c_ftl_nand_byte_pre_oob:
	.zero	2
	.globl	c_ftl_nand_byte_pre_page
	.align 2
	.type	c_ftl_nand_byte_pre_page, @object
	.size	c_ftl_nand_byte_pre_page, 2
c_ftl_nand_byte_pre_page:
	.zero	2
	.globl	c_ftl_nand_sec_pre_page_shift
	.align 2
	.type	c_ftl_nand_sec_pre_page_shift, @object
	.size	c_ftl_nand_sec_pre_page_shift, 2
c_ftl_nand_sec_pre_page_shift:
	.zero	2
	.globl	c_ftl_nand_sec_pre_page
	.align 2
	.type	c_ftl_nand_sec_pre_page, @object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
	.zero	2
	.globl	c_ftl_nand_page_pre_super_blk
	.align 2
	.type	c_ftl_nand_page_pre_super_blk, @object
	.size	c_ftl_nand_page_pre_super_blk, 2
c_ftl_nand_page_pre_super_blk:
	.zero	2
	.globl	c_ftl_nand_page_pre_slc_blk
	.align 2
	.type	c_ftl_nand_page_pre_slc_blk, @object
	.size	c_ftl_nand_page_pre_slc_blk, 2
c_ftl_nand_page_pre_slc_blk:
	.zero	2
	.globl	c_ftl_nand_page_pre_blk
	.align 2
	.type	c_ftl_nand_page_pre_blk, @object
	.size	c_ftl_nand_page_pre_blk, 2
c_ftl_nand_page_pre_blk:
	.zero	2
	.globl	c_ftl_nand_bbm_buf_size
	.align 2
	.type	c_ftl_nand_bbm_buf_size, @object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.zero	2
	.globl	c_ftl_nand_ext_blk_pre_plane
	.align 2
	.type	c_ftl_nand_ext_blk_pre_plane, @object
	.size	c_ftl_nand_ext_blk_pre_plane, 2
c_ftl_nand_ext_blk_pre_plane:
	.zero	2
	.globl	c_ftl_nand_blk_pre_plane
	.align 2
	.type	c_ftl_nand_blk_pre_plane, @object
	.size	c_ftl_nand_blk_pre_plane, 2
c_ftl_nand_blk_pre_plane:
	.zero	2
	.globl	c_ftl_nand_planes_num
	.align 2
	.type	c_ftl_nand_planes_num, @object
	.size	c_ftl_nand_planes_num, 2
c_ftl_nand_planes_num:
	.zero	2
	.globl	c_ftl_nand_blks_per_die_shift
	.align 2
	.type	c_ftl_nand_blks_per_die_shift, @object
	.size	c_ftl_nand_blks_per_die_shift, 2
c_ftl_nand_blks_per_die_shift:
	.zero	2
	.globl	c_ftl_nand_blks_per_die
	.align 2
	.type	c_ftl_nand_blks_per_die, @object
	.size	c_ftl_nand_blks_per_die, 2
c_ftl_nand_blks_per_die:
	.zero	2
	.globl	c_ftl_nand_planes_per_die
	.align 2
	.type	c_ftl_nand_planes_per_die, @object
	.size	c_ftl_nand_planes_per_die, 2
c_ftl_nand_planes_per_die:
	.zero	2
	.globl	c_ftl_nand_die_num
	.align 2
	.type	c_ftl_nand_die_num, @object
	.size	c_ftl_nand_die_num, 2
c_ftl_nand_die_num:
	.zero	2
	.globl	c_ftl_nand_type
	.align 2
	.type	c_ftl_nand_type, @object
	.size	c_ftl_nand_type, 2
c_ftl_nand_type:
	.zero	2
	.globl	gMasterTempBuf
	.align 4
	.type	gMasterTempBuf, @object
	.size	gMasterTempBuf, 4
gMasterTempBuf:
	.zero	4
	.globl	gMasterInfo
	.align 32
	.type	gMasterInfo, @object
	.size	gMasterInfo, 44
gMasterInfo:
	.zero	44
	.globl	gNandcEccError
	.align 4
	.type	gNandcEccError, @object
	.size	gNandcEccError, 4
gNandcEccError:
	.zero	4
	.globl	gToggleModeClkDiv
	.align 4
	.type	gToggleModeClkDiv, @object
	.size	gToggleModeClkDiv, 4
gToggleModeClkDiv:
	.zero	4
	.globl	gBootDdrMode
	.align 4
	.type	gBootDdrMode, @object
	.size	gBootDdrMode, 4
gBootDdrMode:
	.zero	4
	.globl	gNandcEccBits
	.align 4
	.type	gNandcEccBits, @object
	.size	gNandcEccBits, 4
gNandcEccBits:
	.zero	4
	.globl	gpNandc1
	.align 4
	.type	gpNandc1, @object
	.size	gpNandc1, 4
gpNandc1:
	.zero	4
	.globl	gpNandc
	.align 4
	.type	gpNandc, @object
	.size	gpNandc, 4
gpNandc:
	.zero	4
	.globl	gNandcVer
	.align 4
	.type	gNandcVer, @object
	.size	gNandcVer, 4
gNandcVer:
	.zero	4
	.globl	gNandChipMap
	.align 4
	.type	gNandChipMap, @object
	.size	gNandChipMap, 16
gNandChipMap:
	.zero	16
	.globl	FlashWaitBusyScheduleEn
	.align 4
	.type	FlashWaitBusyScheduleEn, @object
	.size	FlashWaitBusyScheduleEn, 4
FlashWaitBusyScheduleEn:
	.zero	4
	.globl	gNandPhyInfo
	.align 4
	.type	gNandPhyInfo, @object
	.size	gNandPhyInfo, 28
gNandPhyInfo:
	.zero	28
	.globl	gFlashProgCheckSpareBuffer
	.align 4
	.type	gFlashProgCheckSpareBuffer, @object
	.size	gFlashProgCheckSpareBuffer, 4
gFlashProgCheckSpareBuffer:
	.zero	4
	.globl	gFlashProgCheckBuffer
	.align 4
	.type	gFlashProgCheckBuffer, @object
	.size	gFlashProgCheckBuffer, 4
gFlashProgCheckBuffer:
	.zero	4
	.globl	gFlashSpareBuffer
	.align 4
	.type	gFlashSpareBuffer, @object
	.size	gFlashSpareBuffer, 4
gFlashSpareBuffer:
	.zero	4
	.globl	gFlashPageBuffer1
	.align 4
	.type	gFlashPageBuffer1, @object
	.size	gFlashPageBuffer1, 4
gFlashPageBuffer1:
	.zero	4
	.globl	gFlashPageBuffer0
	.align 4
	.type	gFlashPageBuffer0, @object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
	.zero	4
	.globl	gpFlashSaveInfo
	.align 4
	.type	gpFlashSaveInfo, @object
	.size	gpFlashSaveInfo, 4
gpFlashSaveInfo:
	.zero	4
	.globl	gReadRetryInfo
	.align 32
	.type	gReadRetryInfo, @object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.zero	852
	.globl	gpNandParaInfo
	.align 4
	.type	gpNandParaInfo, @object
	.size	gpNandParaInfo, 4
gpNandParaInfo:
	.zero	4
	.globl	gNandOptPara
	.align 32
	.type	gNandOptPara, @object
	.size	gNandOptPara, 32
gNandOptPara:
	.zero	32
	.globl	g_maxRetryCount
	.type	g_maxRetryCount, @object
	.size	g_maxRetryCount, 1
g_maxRetryCount:
	.zero	1
	.globl	g_maxRegNum
	.type	g_maxRegNum, @object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.zero	1
	.globl	g_retryMode
	.type	g_retryMode, @object
	.size	g_retryMode, 1
g_retryMode:
	.zero	1
	.globl	gNandIDBResBlkNumSaveInFlash
	.type	gNandIDBResBlkNumSaveInFlash, @object
	.size	gNandIDBResBlkNumSaveInFlash, 1
gNandIDBResBlkNumSaveInFlash:
	.zero	1
	.globl	gNandIDBResBlkNum
	.type	gNandIDBResBlkNum, @object
	.size	gNandIDBResBlkNum, 1
gNandIDBResBlkNum:
	.zero	1
	.globl	gNandFlashResEndPageAddr
	.align 4
	.type	gNandFlashResEndPageAddr, @object
	.size	gNandFlashResEndPageAddr, 4
gNandFlashResEndPageAddr:
	.zero	4
	.globl	gNandFlashInfoBlockAddr
	.align 4
	.type	gNandFlashInfoBlockAddr, @object
	.size	gNandFlashInfoBlockAddr, 4
gNandFlashInfoBlockAddr:
	.zero	4
	.globl	gNandFlashIdbBlockAddr
	.align 4
	.type	gNandFlashIdbBlockAddr, @object
	.size	gNandFlashIdbBlockAddr, 4
gNandFlashIdbBlockAddr:
	.zero	4
	.globl	gNandFlashInfoBlockEcc
	.type	gNandFlashInfoBlockEcc, @object
	.size	gNandFlashInfoBlockEcc, 1
gNandFlashInfoBlockEcc:
	.zero	1
	.globl	gNandFlashIDBEccBits
	.type	gNandFlashIDBEccBits, @object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.zero	1
	.globl	gNandFlashEccBits
	.type	gNandFlashEccBits, @object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.zero	1
	.globl	gNandRandomizer
	.type	gNandRandomizer, @object
	.size	gNandRandomizer, 1
gNandRandomizer:
	.zero	1
	.globl	gBlockPageAlignSize
	.align 4
	.type	gBlockPageAlignSize, @object
	.size	gBlockPageAlignSize, 4
gBlockPageAlignSize:
	.zero	4
	.globl	gTotleBlock
	.align 2
	.type	gTotleBlock, @object
	.size	gTotleBlock, 2
gTotleBlock:
	.zero	2
	.globl	gNandMaxChip
	.type	gNandMaxChip, @object
	.size	gNandMaxChip, 1
gNandMaxChip:
	.zero	1
	.globl	gNandMaxDie
	.type	gNandMaxDie, @object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.zero	1
	.globl	gFlashInterfaceMode
	.type	gFlashInterfaceMode, @object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.zero	1
	.globl	gNandcLoaderDdrMode
	.align 4
	.type	gNandcLoaderDdrMode, @object
	.size	gNandcLoaderDdrMode, 4
gNandcLoaderDdrMode:
	.zero	4
	.globl	gFlashOnfiModeEn
	.type	gFlashOnfiModeEn, @object
	.size	gFlashOnfiModeEn, 1
gFlashOnfiModeEn:
	.zero	1
	.globl	gFlashToggleModeEn
	.type	gFlashToggleModeEn, @object
	.size	gFlashToggleModeEn, 1
gFlashToggleModeEn:
	.zero	1
	.globl	gFlashSdrModeEn
	.type	gFlashSdrModeEn, @object
	.size	gFlashSdrModeEn, 1
gFlashSdrModeEn:
	.zero	1
	.globl	gMultiPageProgEn
	.type	gMultiPageProgEn, @object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.zero	1
	.globl	gMultiPageReadEn
	.type	gMultiPageReadEn, @object
	.size	gMultiPageReadEn, 1
gMultiPageReadEn:
	.zero	1
	.globl	gpReadRetrial
	.align 4
	.type	gpReadRetrial, @object
	.size	gpReadRetrial, 4
gpReadRetrial:
	.zero	4
	.globl	mlcPageToSlcPageTbl
	.align 4
	.type	mlcPageToSlcPageTbl, @object
	.size	mlcPageToSlcPageTbl, 1024
mlcPageToSlcPageTbl:
	.zero	1024
	.globl	slcPageToMlcPageTbl
	.align 4
	.type	slcPageToMlcPageTbl, @object
	.size	slcPageToMlcPageTbl, 512
slcPageToMlcPageTbl:
	.zero	512
	.globl	DieAddrs
	.align 4
	.type	DieAddrs, @object
	.size	DieAddrs, 16
DieAddrs:
	.zero	16
	.globl	gDieOp
	.align 4
	.type	gDieOp, @object
	.size	gDieOp, 64
gDieOp:
	.zero	64
	.globl	DieCsIndex
	.align 4
	.type	DieCsIndex, @object
	.size	DieCsIndex, 4
DieCsIndex:
	.zero	4
	.globl	IDByte
	.align 4
	.type	IDByte, @object
	.size	IDByte, 16
IDByte:
	.zero	16
	.globl	read_retry_cur_offset
	.type	read_retry_cur_offset, @object
	.size	read_retry_cur_offset, 2
read_retry_cur_offset:
	.zero	2
	.local	NANDC_FMCTL
	.comm	NANDC_FMCTL,4,4
	.local	NANDC_FMWAIT
	.comm	NANDC_FMWAIT,4,4
	.local	NANDC_FLCTL
	.comm	NANDC_FLCTL,4,4
	.local	NANDC_BCHCTL
	.comm	NANDC_BCHCTL,4,4
	.local	NANDC_DLL_CTL_REG0
	.comm	NANDC_DLL_CTL_REG0,4,4
	.local	NANDC_DLL_CTL_REG1
	.comm	NANDC_DLL_CTL_REG1,4,4
	.local	NANDC_RANDMZ_CFG
	.comm	NANDC_RANDMZ_CFG,4,4
	.local	NANDC_FMWAIT_SYN
	.comm	NANDC_FMWAIT_SYN,4,4
	.section	.rodata
	.type	__func__.23916, @object
	.size	__func__.23916, 11
__func__.23916:
	.string	"FtlMemInit"
	.local	gc_discard_updated
	.comm	gc_discard_updated,4,4
	.type	__func__.24754, @object
	.size	__func__.24754, 21
__func__.24754:
	.string	"FtlVpcCheckAndModify"
	.data
	.align 2
	.type	ftl_gc_temp_block_bops_scan_page_addr, @object
	.size	ftl_gc_temp_block_bops_scan_page_addr, 2
ftl_gc_temp_block_bops_scan_page_addr:
	.value	-1
	.align 4
	.type	rk29_nandc_wait, @object
	.size	rk29_nandc_wait, 12
rk29_nandc_wait:
	.value	0
	.zero	2
	.long	rk29_nandc_wait+4
	.long	rk29_nandc_wait+4
	.local	__key.6352
	.comm	__key.6352,0,1
	.local	mtd_read_temp_buffer
	.comm	mtd_read_temp_buffer,4,4
	.local	__key.34617
	.comm	__key.34617,0,1
	.local	disk_array
	.comm	disk_array,880,32
	.local	mtd_rw_buffer
	.comm	mtd_rw_buffer,4,4
	.section	.rodata
	.align 32
	.type	nand_proc_fops, @object
	.size	nand_proc_fops, 112
nand_proc_fops:
	.long	0
	.long	seq_lseek
	.long	seq_read
	.zero	32
	.long	nand_proc_open
	.zero	4
	.long	single_release
	.zero	56
	.data
	.align 4
	.type	rk_ftl_gc_timeout, @object
	.size	rk_ftl_gc_timeout, 28
rk_ftl_gc_timeout:
	.zero	4
	.long	1953723489
	.long	0
	.long	boot_tvec_bases
	.long	rk_ftl_gc_timeout_hack
	.long	0
	.long	-1
	.local	nand_bg_do
	.comm	nand_bg_do,4,4
	.section	.data.unlikely,"aw",@progbits
	.type	__warned.31817, @object
	.size	__warned.31817, 1
__warned.31817:
	.zero	1
	.data
	.align 4
	.type	nand_bg_thread_wait, @object
	.size	nand_bg_thread_wait, 12
nand_bg_thread_wait:
	.value	0
	.zero	2
	.long	nand_bg_thread_wait+4
	.long	nand_bg_thread_wait+4
	.align 4
	.type	rk29_buffer_write_done, @object
	.size	rk29_buffer_write_done, 4
rk29_buffer_write_done:
	.long	1
	.align 4
	.type	rk29_buffer_wait, @object
	.size	rk29_buffer_wait, 12
rk29_buffer_wait:
	.value	0
	.zero	2
	.long	rk29_buffer_wait+4
	.long	rk29_buffer_wait+4
	.local	g_next_read_Index
	.comm	g_next_read_Index,4,4
	.local	g_next_read_match_sector
	.comm	g_next_read_match_sector,4,4
	.local	g_next_set_read_Index
	.comm	g_next_set_read_Index,4,4
	.align 4
	.type	rk29_buffer_wait_read, @object
	.size	rk29_buffer_wait_read, 4
rk29_buffer_wait_read:
	.long	2
	.align 32
	.type	mytr, @object
	.size	mytr, 120
mytr:
	.long	.LC121
	.long	31
	.long	0
	.long	nand_add_dev
	.long	nand_remove_dev
	.long	nand_flush
	.zero	88
	.long	0
	.zero	4
