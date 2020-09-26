	.cpu cortex-m4
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 6
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"nrf_strerror.c"
	.text
.Ltext0:
	.section	.rodata.m_unknown_str,"a"
	.align	2
	.type	m_unknown_str, %object
	.size	m_unknown_str, 19
m_unknown_str:
	.ascii	"Unknown error code\000"
	.section .rodata
	.align	2
.LC0:
	.ascii	"NRF_SUCCESS\000"
	.align	2
.LC1:
	.ascii	"NRF_ERROR_SVC_HANDLER_MISSING\000"
	.align	2
.LC2:
	.ascii	"NRF_ERROR_SOFTDEVICE_NOT_ENABLED\000"
	.align	2
.LC3:
	.ascii	"NRF_ERROR_INTERNAL\000"
	.align	2
.LC4:
	.ascii	"NRF_ERROR_NO_MEM\000"
	.align	2
.LC5:
	.ascii	"NRF_ERROR_NOT_FOUND\000"
	.align	2
.LC6:
	.ascii	"NRF_ERROR_NOT_SUPPORTED\000"
	.align	2
.LC7:
	.ascii	"NRF_ERROR_INVALID_PARAM\000"
	.align	2
.LC8:
	.ascii	"NRF_ERROR_INVALID_STATE\000"
	.align	2
.LC9:
	.ascii	"NRF_ERROR_INVALID_LENGTH\000"
	.align	2
.LC10:
	.ascii	"NRF_ERROR_INVALID_FLAGS\000"
	.align	2
.LC11:
	.ascii	"NRF_ERROR_INVALID_DATA\000"
	.align	2
.LC12:
	.ascii	"NRF_ERROR_DATA_SIZE\000"
	.align	2
.LC13:
	.ascii	"NRF_ERROR_TIMEOUT\000"
	.align	2
.LC14:
	.ascii	"NRF_ERROR_NULL\000"
	.align	2
.LC15:
	.ascii	"NRF_ERROR_FORBIDDEN\000"
	.align	2
.LC16:
	.ascii	"NRF_ERROR_INVALID_ADDR\000"
	.align	2
.LC17:
	.ascii	"NRF_ERROR_BUSY\000"
	.align	2
.LC18:
	.ascii	"NRF_ERROR_MODULE_NOT_INITIALZED\000"
	.align	2
.LC19:
	.ascii	"NRF_ERROR_MUTEX_INIT_FAILED\000"
	.align	2
.LC20:
	.ascii	"NRF_ERROR_MUTEX_LOCK_FAILED\000"
	.align	2
.LC21:
	.ascii	"NRF_ERROR_MUTEX_UNLOCK_FAILED\000"
	.align	2
.LC22:
	.ascii	"NRF_ERROR_MUTEX_COND_INIT_FAILED\000"
	.align	2
.LC23:
	.ascii	"NRF_ERROR_MODULE_ALREADY_INITIALIZED\000"
	.align	2
.LC24:
	.ascii	"NRF_ERROR_STORAGE_FULL\000"
	.align	2
.LC25:
	.ascii	"NRF_ERROR_API_NOT_IMPLEMENTED\000"
	.align	2
.LC26:
	.ascii	"NRF_ERROR_FEATURE_NOT_ENABLED\000"
	.align	2
.LC27:
	.ascii	"NRF_ERROR_DRV_TWI_ERR_OVERRUN\000"
	.align	2
.LC28:
	.ascii	"NRF_ERROR_DRV_TWI_ERR_ANACK\000"
	.align	2
.LC29:
	.ascii	"NRF_ERROR_DRV_TWI_ERR_DNACK\000"
	.section	.rodata.nrf_strerror_array,"a"
	.align	2
	.type	nrf_strerror_array, %object
	.size	nrf_strerror_array, 240
nrf_strerror_array:
	.word	0
	.word	.LC0
	.word	1
	.word	.LC1
	.word	2
	.word	.LC2
	.word	3
	.word	.LC3
	.word	4
	.word	.LC4
	.word	5
	.word	.LC5
	.word	6
	.word	.LC6
	.word	7
	.word	.LC7
	.word	8
	.word	.LC8
	.word	9
	.word	.LC9
	.word	10
	.word	.LC10
	.word	11
	.word	.LC11
	.word	12
	.word	.LC12
	.word	13
	.word	.LC13
	.word	14
	.word	.LC14
	.word	15
	.word	.LC15
	.word	16
	.word	.LC16
	.word	17
	.word	.LC17
	.word	128
	.word	.LC18
	.word	129
	.word	.LC19
	.word	130
	.word	.LC20
	.word	131
	.word	.LC21
	.word	132
	.word	.LC22
	.word	133
	.word	.LC23
	.word	134
	.word	.LC24
	.word	144
	.word	.LC25
	.word	145
	.word	.LC26
	.word	33280
	.word	.LC27
	.word	33281
	.word	.LC28
	.word	33282
	.word	.LC29
	.section	.text.nrf_strerror_get,"ax",%progbits
	.align	1
	.global	nrf_strerror_get
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	nrf_strerror_get, %function
nrf_strerror_get:
.LFB147:
	.file 1 "D:\\ikyhdst\\Git\\dwm1001-examples\\nRF5_SDK_14.2.0\\components\\libraries\\strerror\\nrf_strerror.c"
	.loc 1 116 1
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{lr}
.LCFI0:
	sub	sp, sp, #20
.LCFI1:
	str	r0, [sp, #4]
	.loc 1 117 26
	ldr	r0, [sp, #4]
	bl	nrf_strerror_find
	str	r0, [sp, #12]
	.loc 1 118 44
	ldr	r3, [sp, #12]
	cmp	r3, #0
	beq	.L2
	.loc 1 118 44 is_stmt 0 discriminator 1
	ldr	r3, [sp, #12]
	b	.L4
.L2:
	.loc 1 118 44 discriminator 2
	ldr	r3, .L5
.L4:
	.loc 1 119 1 is_stmt 1 discriminator 5
	mov	r0, r3
	add	sp, sp, #20
.LCFI2:
	@ sp needed
	ldr	pc, [sp], #4
.L6:
	.align	2
.L5:
	.word	m_unknown_str
.LFE147:
	.size	nrf_strerror_get, .-nrf_strerror_get
	.section	.text.nrf_strerror_find,"ax",%progbits
	.align	1
	.global	nrf_strerror_find
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	nrf_strerror_find, %function
nrf_strerror_find:
.LFB148:
	.loc 1 122 1
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	sp, sp, #24
.LCFI3:
	str	r0, [sp, #4]
	.loc 1 125 13
	ldr	r3, .L13
	str	r3, [sp, #20]
	.loc 1 126 13
	ldr	r3, .L13+4
	str	r3, [sp, #16]
	.loc 1 128 11
	b	.L8
.L12:
.LBB2:
	.loc 1 130 63
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	subs	r3, r2, r3
	asrs	r3, r3, #3
	.loc 1 130 74
	lsrs	r2, r3, #31
	add	r3, r3, r2
	asrs	r3, r3, #1
	.loc 1 130 53
	lsls	r3, r3, #3
	.loc 1 130 37
	ldr	r2, [sp, #20]
	add	r3, r3, r2
	str	r3, [sp, #12]
	.loc 1 131 20
	ldr	r3, [sp, #12]
	ldr	r3, [r3]
	str	r3, [sp, #8]
	.loc 1 132 12
	ldr	r2, [sp, #8]
	ldr	r3, [sp, #4]
	cmp	r2, r3
	bls	.L9
	.loc 1 134 19
	ldr	r3, [sp, #12]
	str	r3, [sp, #16]
	b	.L8
.L9:
	.loc 1 136 17
	ldr	r2, [sp, #8]
	ldr	r3, [sp, #4]
	cmp	r2, r3
	bcs	.L10
	.loc 1 138 21
	ldr	r3, [sp, #12]
	adds	r3, r3, #8
	str	r3, [sp, #20]
	b	.L8
.L10:
	.loc 1 142 25
	ldr	r3, [sp, #12]
	ldr	r3, [r3, #4]
	b	.L11
.L8:
.LBE2:
	.loc 1 128 11
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #16]
	cmp	r2, r3
	bcc	.L12
	.loc 1 145 12
	movs	r3, #0
.L11:
	.loc 1 146 1
	mov	r0, r3
	add	sp, sp, #24
.LCFI4:
	@ sp needed
	bx	lr
.L14:
	.align	2
.L13:
	.word	nrf_strerror_array
	.word	nrf_strerror_array+240
.LFE148:
	.size	nrf_strerror_find, .-nrf_strerror_find
	.section	.debug_frame,"",%progbits
.Lframe0:
	.4byte	.LECIE0-.LSCIE0
.LSCIE0:
	.4byte	0xffffffff
	.byte	0x3
	.ascii	"\000"
	.uleb128 0x1
	.sleb128 -4
	.uleb128 0xe
	.byte	0xc
	.uleb128 0xd
	.uleb128 0
	.align	2
.LECIE0:
.LSFDE0:
	.4byte	.LEFDE0-.LASFDE0
.LASFDE0:
	.4byte	.Lframe0
	.4byte	.LFB147
	.4byte	.LFE147-.LFB147
	.byte	0x4
	.4byte	.LCFI0-.LFB147
	.byte	0xe
	.uleb128 0x4
	.byte	0x8e
	.uleb128 0x1
	.byte	0x4
	.4byte	.LCFI1-.LCFI0
	.byte	0xe
	.uleb128 0x18
	.byte	0x4
	.4byte	.LCFI2-.LCFI1
	.byte	0xe
	.uleb128 0x4
	.align	2
.LEFDE0:
.LSFDE2:
	.4byte	.LEFDE2-.LASFDE2
.LASFDE2:
	.4byte	.Lframe0
	.4byte	.LFB148
	.4byte	.LFE148-.LFB148
	.byte	0x4
	.4byte	.LCFI3-.LFB148
	.byte	0xe
	.uleb128 0x18
	.byte	0x4
	.4byte	.LCFI4-.LCFI3
	.byte	0xe
	.uleb128 0
	.align	2
.LEFDE2:
	.text
.Letext0:
	.file 2 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 5.10/include/stdint.h"
	.file 3 "C:/Program Files/SEGGER/SEGGER Embedded Studio for ARM 5.10/include/__crossworks.h"
	.file 4 "../../../nRF5_SDK_14.2.0/components/libraries/util/sdk_errors.h"
	.file 5 "C:/Users/iky-h/AppData/Local/SEGGER/SEGGER Embedded Studio/v3/packages/CMSIS_5/CMSIS/Core/Include/core_cm4.h"
	.file 6 "../../../nRF5_SDK_14.2.0/components/toolchain/system_nrf52.h"
	.file 7 "../../../nRF5_SDK_14.2.0/components/libraries/util/app_util.h"
	.section	.debug_info,"",%progbits
.Ldebug_info0:
	.4byte	0x74d
	.2byte	0x4
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.4byte	.LASF99
	.byte	0xc
	.4byte	.LASF100
	.4byte	.LASF101
	.4byte	.Ldebug_ranges0+0
	.4byte	0
	.4byte	.Ldebug_line0
	.uleb128 0x2
	.byte	0x1
	.byte	0x6
	.4byte	.LASF0
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF1
	.uleb128 0x3
	.4byte	0x2c
	.uleb128 0x2
	.byte	0x2
	.byte	0x5
	.4byte	.LASF2
	.uleb128 0x2
	.byte	0x2
	.byte	0x7
	.4byte	.LASF3
	.uleb128 0x4
	.4byte	.LASF4
	.byte	0x2
	.byte	0x36
	.byte	0x1c
	.4byte	0x57
	.uleb128 0x5
	.4byte	0x46
	.uleb128 0x6
	.byte	0x4
	.byte	0x5
	.ascii	"int\000"
	.uleb128 0x4
	.4byte	.LASF5
	.byte	0x2
	.byte	0x37
	.byte	0x1c
	.4byte	0x6a
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF6
	.uleb128 0x2
	.byte	0x8
	.byte	0x5
	.4byte	.LASF7
	.uleb128 0x2
	.byte	0x8
	.byte	0x7
	.4byte	.LASF8
	.uleb128 0x7
	.4byte	.LASF59
	.byte	0x8
	.byte	0x3
	.byte	0x7c
	.byte	0x8
	.4byte	0xa7
	.uleb128 0x8
	.4byte	.LASF9
	.byte	0x3
	.byte	0x7d
	.byte	0x7
	.4byte	0x57
	.byte	0
	.uleb128 0x8
	.4byte	.LASF10
	.byte	0x3
	.byte	0x7e
	.byte	0x8
	.4byte	0xa7
	.byte	0x4
	.byte	0
	.uleb128 0x2
	.byte	0x4
	.byte	0x5
	.4byte	.LASF11
	.uleb128 0x9
	.4byte	0x57
	.4byte	0xc7
	.uleb128 0xa
	.4byte	0xc7
	.uleb128 0xa
	.4byte	0x6a
	.uleb128 0xa
	.4byte	0xd9
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0xcd
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF12
	.uleb128 0x3
	.4byte	0xcd
	.uleb128 0xb
	.byte	0x4
	.4byte	0x7f
	.uleb128 0x9
	.4byte	0x57
	.4byte	0xfd
	.uleb128 0xa
	.4byte	0xfd
	.uleb128 0xa
	.4byte	0x103
	.uleb128 0xa
	.4byte	0x6a
	.uleb128 0xa
	.4byte	0xd9
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x6a
	.uleb128 0xb
	.byte	0x4
	.4byte	0xd4
	.uleb128 0xc
	.byte	0x58
	.byte	0x3
	.byte	0x84
	.byte	0x9
	.4byte	0x2b3
	.uleb128 0x8
	.4byte	.LASF13
	.byte	0x3
	.byte	0x86
	.byte	0xf
	.4byte	0x103
	.byte	0
	.uleb128 0x8
	.4byte	.LASF14
	.byte	0x3
	.byte	0x87
	.byte	0xf
	.4byte	0x103
	.byte	0x4
	.uleb128 0x8
	.4byte	.LASF15
	.byte	0x3
	.byte	0x88
	.byte	0xf
	.4byte	0x103
	.byte	0x8
	.uleb128 0x8
	.4byte	.LASF16
	.byte	0x3
	.byte	0x8a
	.byte	0xf
	.4byte	0x103
	.byte	0xc
	.uleb128 0x8
	.4byte	.LASF17
	.byte	0x3
	.byte	0x8b
	.byte	0xf
	.4byte	0x103
	.byte	0x10
	.uleb128 0x8
	.4byte	.LASF18
	.byte	0x3
	.byte	0x8c
	.byte	0xf
	.4byte	0x103
	.byte	0x14
	.uleb128 0x8
	.4byte	.LASF19
	.byte	0x3
	.byte	0x8d
	.byte	0xf
	.4byte	0x103
	.byte	0x18
	.uleb128 0x8
	.4byte	.LASF20
	.byte	0x3
	.byte	0x8e
	.byte	0xf
	.4byte	0x103
	.byte	0x1c
	.uleb128 0x8
	.4byte	.LASF21
	.byte	0x3
	.byte	0x8f
	.byte	0xf
	.4byte	0x103
	.byte	0x20
	.uleb128 0x8
	.4byte	.LASF22
	.byte	0x3
	.byte	0x90
	.byte	0xf
	.4byte	0x103
	.byte	0x24
	.uleb128 0x8
	.4byte	.LASF23
	.byte	0x3
	.byte	0x92
	.byte	0x8
	.4byte	0xcd
	.byte	0x28
	.uleb128 0x8
	.4byte	.LASF24
	.byte	0x3
	.byte	0x93
	.byte	0x8
	.4byte	0xcd
	.byte	0x29
	.uleb128 0x8
	.4byte	.LASF25
	.byte	0x3
	.byte	0x94
	.byte	0x8
	.4byte	0xcd
	.byte	0x2a
	.uleb128 0x8
	.4byte	.LASF26
	.byte	0x3
	.byte	0x95
	.byte	0x8
	.4byte	0xcd
	.byte	0x2b
	.uleb128 0x8
	.4byte	.LASF27
	.byte	0x3
	.byte	0x96
	.byte	0x8
	.4byte	0xcd
	.byte	0x2c
	.uleb128 0x8
	.4byte	.LASF28
	.byte	0x3
	.byte	0x97
	.byte	0x8
	.4byte	0xcd
	.byte	0x2d
	.uleb128 0x8
	.4byte	.LASF29
	.byte	0x3
	.byte	0x98
	.byte	0x8
	.4byte	0xcd
	.byte	0x2e
	.uleb128 0x8
	.4byte	.LASF30
	.byte	0x3
	.byte	0x99
	.byte	0x8
	.4byte	0xcd
	.byte	0x2f
	.uleb128 0x8
	.4byte	.LASF31
	.byte	0x3
	.byte	0x9a
	.byte	0x8
	.4byte	0xcd
	.byte	0x30
	.uleb128 0x8
	.4byte	.LASF32
	.byte	0x3
	.byte	0x9b
	.byte	0x8
	.4byte	0xcd
	.byte	0x31
	.uleb128 0x8
	.4byte	.LASF33
	.byte	0x3
	.byte	0x9c
	.byte	0x8
	.4byte	0xcd
	.byte	0x32
	.uleb128 0x8
	.4byte	.LASF34
	.byte	0x3
	.byte	0x9d
	.byte	0x8
	.4byte	0xcd
	.byte	0x33
	.uleb128 0x8
	.4byte	.LASF35
	.byte	0x3
	.byte	0x9e
	.byte	0x8
	.4byte	0xcd
	.byte	0x34
	.uleb128 0x8
	.4byte	.LASF36
	.byte	0x3
	.byte	0x9f
	.byte	0x8
	.4byte	0xcd
	.byte	0x35
	.uleb128 0x8
	.4byte	.LASF37
	.byte	0x3
	.byte	0xa4
	.byte	0xf
	.4byte	0x103
	.byte	0x38
	.uleb128 0x8
	.4byte	.LASF38
	.byte	0x3
	.byte	0xa5
	.byte	0xf
	.4byte	0x103
	.byte	0x3c
	.uleb128 0x8
	.4byte	.LASF39
	.byte	0x3
	.byte	0xa6
	.byte	0xf
	.4byte	0x103
	.byte	0x40
	.uleb128 0x8
	.4byte	.LASF40
	.byte	0x3
	.byte	0xa7
	.byte	0xf
	.4byte	0x103
	.byte	0x44
	.uleb128 0x8
	.4byte	.LASF41
	.byte	0x3
	.byte	0xa8
	.byte	0xf
	.4byte	0x103
	.byte	0x48
	.uleb128 0x8
	.4byte	.LASF42
	.byte	0x3
	.byte	0xa9
	.byte	0xf
	.4byte	0x103
	.byte	0x4c
	.uleb128 0x8
	.4byte	.LASF43
	.byte	0x3
	.byte	0xaa
	.byte	0xf
	.4byte	0x103
	.byte	0x50
	.uleb128 0x8
	.4byte	.LASF44
	.byte	0x3
	.byte	0xab
	.byte	0xf
	.4byte	0x103
	.byte	0x54
	.byte	0
	.uleb128 0x4
	.4byte	.LASF45
	.byte	0x3
	.byte	0xac
	.byte	0x3
	.4byte	0x109
	.uleb128 0x3
	.4byte	0x2b3
	.uleb128 0xc
	.byte	0x20
	.byte	0x3
	.byte	0xc2
	.byte	0x9
	.4byte	0x336
	.uleb128 0x8
	.4byte	.LASF46
	.byte	0x3
	.byte	0xc4
	.byte	0x9
	.4byte	0x34a
	.byte	0
	.uleb128 0x8
	.4byte	.LASF47
	.byte	0x3
	.byte	0xc5
	.byte	0x9
	.4byte	0x35f
	.byte	0x4
	.uleb128 0x8
	.4byte	.LASF48
	.byte	0x3
	.byte	0xc6
	.byte	0x9
	.4byte	0x35f
	.byte	0x8
	.uleb128 0x8
	.4byte	.LASF49
	.byte	0x3
	.byte	0xc9
	.byte	0x9
	.4byte	0x379
	.byte	0xc
	.uleb128 0x8
	.4byte	.LASF50
	.byte	0x3
	.byte	0xca
	.byte	0xa
	.4byte	0x38e
	.byte	0x10
	.uleb128 0x8
	.4byte	.LASF51
	.byte	0x3
	.byte	0xcb
	.byte	0xa
	.4byte	0x38e
	.byte	0x14
	.uleb128 0x8
	.4byte	.LASF52
	.byte	0x3
	.byte	0xce
	.byte	0x9
	.4byte	0x394
	.byte	0x18
	.uleb128 0x8
	.4byte	.LASF53
	.byte	0x3
	.byte	0xcf
	.byte	0x9
	.4byte	0x39a
	.byte	0x1c
	.byte	0
	.uleb128 0x9
	.4byte	0x57
	.4byte	0x34a
	.uleb128 0xa
	.4byte	0x57
	.uleb128 0xa
	.4byte	0x57
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x336
	.uleb128 0x9
	.4byte	0x57
	.4byte	0x35f
	.uleb128 0xa
	.4byte	0x57
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x350
	.uleb128 0x9
	.4byte	0x57
	.4byte	0x379
	.uleb128 0xa
	.4byte	0xa7
	.uleb128 0xa
	.4byte	0x57
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x365
	.uleb128 0x9
	.4byte	0xa7
	.4byte	0x38e
	.uleb128 0xa
	.4byte	0xa7
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x37f
	.uleb128 0xb
	.byte	0x4
	.4byte	0xae
	.uleb128 0xb
	.byte	0x4
	.4byte	0xdf
	.uleb128 0x4
	.4byte	.LASF54
	.byte	0x3
	.byte	0xd0
	.byte	0x3
	.4byte	0x2c4
	.uleb128 0x3
	.4byte	0x3a0
	.uleb128 0xc
	.byte	0xc
	.byte	0x3
	.byte	0xd2
	.byte	0x9
	.4byte	0x3e2
	.uleb128 0x8
	.4byte	.LASF55
	.byte	0x3
	.byte	0xd3
	.byte	0xf
	.4byte	0x103
	.byte	0
	.uleb128 0x8
	.4byte	.LASF56
	.byte	0x3
	.byte	0xd4
	.byte	0x25
	.4byte	0x3e2
	.byte	0x4
	.uleb128 0x8
	.4byte	.LASF57
	.byte	0x3
	.byte	0xd5
	.byte	0x28
	.4byte	0x3e8
	.byte	0x8
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x2bf
	.uleb128 0xb
	.byte	0x4
	.4byte	0x3ac
	.uleb128 0x4
	.4byte	.LASF58
	.byte	0x3
	.byte	0xd6
	.byte	0x3
	.4byte	0x3b1
	.uleb128 0x3
	.4byte	0x3ee
	.uleb128 0x7
	.4byte	.LASF60
	.byte	0x14
	.byte	0x3
	.byte	0xda
	.byte	0x10
	.4byte	0x41a
	.uleb128 0x8
	.4byte	.LASF61
	.byte	0x3
	.byte	0xdb
	.byte	0x20
	.4byte	0x41a
	.byte	0
	.byte	0
	.uleb128 0xd
	.4byte	0x42a
	.4byte	0x42a
	.uleb128 0xe
	.4byte	0x6a
	.byte	0x4
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x3fa
	.uleb128 0xf
	.4byte	.LASF62
	.byte	0x3
	.2byte	0x104
	.byte	0x1a
	.4byte	0x3ff
	.uleb128 0xf
	.4byte	.LASF63
	.byte	0x3
	.2byte	0x10b
	.byte	0x24
	.4byte	0x3fa
	.uleb128 0xf
	.4byte	.LASF64
	.byte	0x3
	.2byte	0x10e
	.byte	0x2c
	.4byte	0x3ac
	.uleb128 0xf
	.4byte	.LASF65
	.byte	0x3
	.2byte	0x10f
	.byte	0x2c
	.4byte	0x3ac
	.uleb128 0xd
	.4byte	0x33
	.4byte	0x474
	.uleb128 0xe
	.4byte	0x6a
	.byte	0x7f
	.byte	0
	.uleb128 0x3
	.4byte	0x464
	.uleb128 0xf
	.4byte	.LASF66
	.byte	0x3
	.2byte	0x111
	.byte	0x23
	.4byte	0x474
	.uleb128 0xd
	.4byte	0xd4
	.4byte	0x491
	.uleb128 0x10
	.byte	0
	.uleb128 0x3
	.4byte	0x486
	.uleb128 0xf
	.4byte	.LASF67
	.byte	0x3
	.2byte	0x113
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF68
	.byte	0x3
	.2byte	0x114
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF69
	.byte	0x3
	.2byte	0x115
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF70
	.byte	0x3
	.2byte	0x116
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF71
	.byte	0x3
	.2byte	0x118
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF72
	.byte	0x3
	.2byte	0x119
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF73
	.byte	0x3
	.2byte	0x11a
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF74
	.byte	0x3
	.2byte	0x11b
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF75
	.byte	0x3
	.2byte	0x11c
	.byte	0x13
	.4byte	0x491
	.uleb128 0xf
	.4byte	.LASF76
	.byte	0x3
	.2byte	0x11d
	.byte	0x13
	.4byte	0x491
	.uleb128 0x9
	.4byte	0x57
	.4byte	0x527
	.uleb128 0xa
	.4byte	0x527
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x532
	.uleb128 0x11
	.4byte	.LASF102
	.uleb128 0x3
	.4byte	0x52d
	.uleb128 0xf
	.4byte	.LASF77
	.byte	0x3
	.2byte	0x133
	.byte	0xe
	.4byte	0x544
	.uleb128 0xb
	.byte	0x4
	.4byte	0x518
	.uleb128 0x9
	.4byte	0x57
	.4byte	0x559
	.uleb128 0xa
	.4byte	0x559
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x52d
	.uleb128 0xf
	.4byte	.LASF78
	.byte	0x3
	.2byte	0x134
	.byte	0xe
	.4byte	0x56c
	.uleb128 0xb
	.byte	0x4
	.4byte	0x54a
	.uleb128 0x12
	.4byte	.LASF79
	.byte	0x3
	.2byte	0x14b
	.byte	0x18
	.4byte	0x57f
	.uleb128 0xb
	.byte	0x4
	.4byte	0x585
	.uleb128 0x9
	.4byte	0x103
	.4byte	0x594
	.uleb128 0xa
	.4byte	0x57
	.byte	0
	.uleb128 0x13
	.4byte	.LASF80
	.byte	0x8
	.byte	0x3
	.2byte	0x14d
	.byte	0x10
	.4byte	0x5bf
	.uleb128 0x14
	.4byte	.LASF81
	.byte	0x3
	.2byte	0x14f
	.byte	0x1c
	.4byte	0x572
	.byte	0
	.uleb128 0x14
	.4byte	.LASF82
	.byte	0x3
	.2byte	0x150
	.byte	0x21
	.4byte	0x5bf
	.byte	0x4
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x594
	.uleb128 0x12
	.4byte	.LASF83
	.byte	0x3
	.2byte	0x151
	.byte	0x3
	.4byte	0x594
	.uleb128 0xf
	.4byte	.LASF84
	.byte	0x3
	.2byte	0x155
	.byte	0x1f
	.4byte	0x5df
	.uleb128 0xb
	.byte	0x4
	.4byte	0x5c5
	.uleb128 0x4
	.4byte	.LASF85
	.byte	0x4
	.byte	0x9d
	.byte	0x12
	.4byte	0x5e
	.uleb128 0xf
	.4byte	.LASF86
	.byte	0x5
	.2byte	0x804
	.byte	0x19
	.4byte	0x52
	.uleb128 0x15
	.4byte	.LASF87
	.byte	0x6
	.byte	0x21
	.byte	0x11
	.4byte	0x5e
	.uleb128 0x15
	.4byte	.LASF88
	.byte	0x7
	.byte	0x4b
	.byte	0x11
	.4byte	0x5e
	.uleb128 0x15
	.4byte	.LASF89
	.byte	0x7
	.byte	0x4c
	.byte	0x11
	.4byte	0x5e
	.uleb128 0xc
	.byte	0x8
	.byte	0x1
	.byte	0x37
	.byte	0x9
	.4byte	0x646
	.uleb128 0x8
	.4byte	.LASF90
	.byte	0x1
	.byte	0x39
	.byte	0x12
	.4byte	0x5e5
	.byte	0
	.uleb128 0x8
	.4byte	.LASF55
	.byte	0x1
	.byte	0x3a
	.byte	0x12
	.4byte	0x103
	.byte	0x4
	.byte	0
	.uleb128 0x4
	.4byte	.LASF91
	.byte	0x1
	.byte	0x3b
	.byte	0x2
	.4byte	0x622
	.uleb128 0x3
	.4byte	0x646
	.uleb128 0xd
	.4byte	0xd4
	.4byte	0x667
	.uleb128 0xe
	.4byte	0x6a
	.byte	0x12
	.byte	0
	.uleb128 0x3
	.4byte	0x657
	.uleb128 0x16
	.4byte	.LASF92
	.byte	0x1
	.byte	0x42
	.byte	0x13
	.4byte	0x667
	.uleb128 0x5
	.byte	0x3
	.4byte	m_unknown_str
	.uleb128 0xd
	.4byte	0x652
	.4byte	0x68e
	.uleb128 0xe
	.4byte	0x6a
	.byte	0x1d
	.byte	0
	.uleb128 0x3
	.4byte	0x67e
	.uleb128 0x16
	.4byte	.LASF93
	.byte	0x1
	.byte	0x4c
	.byte	0x22
	.4byte	0x68e
	.uleb128 0x5
	.byte	0x3
	.4byte	nrf_strerror_array
	.uleb128 0x17
	.4byte	.LASF103
	.byte	0x1
	.byte	0x79
	.byte	0xe
	.4byte	0x103
	.4byte	.LFB148
	.4byte	.LFE148-.LFB148
	.uleb128 0x1
	.byte	0x9c
	.4byte	0x715
	.uleb128 0x18
	.4byte	.LASF90
	.byte	0x1
	.byte	0x79
	.byte	0x2b
	.4byte	0x5e5
	.uleb128 0x2
	.byte	0x91
	.sleb128 -20
	.uleb128 0x16
	.4byte	.LASF94
	.byte	0x1
	.byte	0x7b
	.byte	0x21
	.4byte	0x715
	.uleb128 0x2
	.byte	0x91
	.sleb128 -4
	.uleb128 0x16
	.4byte	.LASF95
	.byte	0x1
	.byte	0x7c
	.byte	0x21
	.4byte	0x715
	.uleb128 0x2
	.byte	0x91
	.sleb128 -8
	.uleb128 0x19
	.4byte	.LBB2
	.4byte	.LBE2-.LBB2
	.uleb128 0x16
	.4byte	.LASF96
	.byte	0x1
	.byte	0x82
	.byte	0x25
	.4byte	0x715
	.uleb128 0x2
	.byte	0x91
	.sleb128 -12
	.uleb128 0x16
	.4byte	.LASF97
	.byte	0x1
	.byte	0x83
	.byte	0x14
	.4byte	0x5e5
	.uleb128 0x2
	.byte	0x91
	.sleb128 -16
	.byte	0
	.byte	0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x652
	.uleb128 0x1a
	.4byte	.LASF104
	.byte	0x1
	.byte	0x73
	.byte	0xe
	.4byte	0x103
	.4byte	.LFB147
	.4byte	.LFE147-.LFB147
	.uleb128 0x1
	.byte	0x9c
	.uleb128 0x18
	.4byte	.LASF90
	.byte	0x1
	.byte	0x73
	.byte	0x2a
	.4byte	0x5e5
	.uleb128 0x2
	.byte	0x91
	.sleb128 -20
	.uleb128 0x16
	.4byte	.LASF98
	.byte	0x1
	.byte	0x75
	.byte	0x12
	.4byte	0x103
	.uleb128 0x2
	.byte	0x91
	.sleb128 -12
	.byte	0
	.byte	0
	.section	.debug_abbrev,"",%progbits
.Ldebug_abbrev0:
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x25
	.uleb128 0xe
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1b
	.uleb128 0xe
	.uleb128 0x2134
	.uleb128 0x19
	.uleb128 0x55
	.uleb128 0x17
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x10
	.uleb128 0x17
	.byte	0
	.byte	0
	.uleb128 0x2
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.byte	0
	.byte	0
	.uleb128 0x3
	.uleb128 0x26
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x4
	.uleb128 0x16
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x5
	.uleb128 0x35
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x6
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.byte	0
	.byte	0
	.uleb128 0x7
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x8
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x9
	.uleb128 0x15
	.byte	0x1
	.uleb128 0x27
	.uleb128 0x19
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xa
	.uleb128 0x5
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xb
	.uleb128 0xf
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xc
	.uleb128 0x13
	.byte	0x1
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xd
	.uleb128 0x1
	.byte	0x1
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xe
	.uleb128 0x21
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2f
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0xf
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.byte	0
	.byte	0
	.uleb128 0x10
	.uleb128 0x21
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x11
	.uleb128 0x13
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3c
	.uleb128 0x19
	.byte	0
	.byte	0
	.uleb128 0x12
	.uleb128 0x16
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x13
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x14
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x15
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.byte	0
	.byte	0
	.uleb128 0x16
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x18
	.byte	0
	.byte	0
	.uleb128 0x17
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0x19
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x6
	.uleb128 0x40
	.uleb128 0x18
	.uleb128 0x2117
	.uleb128 0x19
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x18
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x18
	.byte	0
	.byte	0
	.uleb128 0x19
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x1a
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x39
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0x19
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x6
	.uleb128 0x40
	.uleb128 0x18
	.uleb128 0x2116
	.uleb128 0x19
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_pubnames,"",%progbits
	.4byte	0x62
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x751
	.4byte	0x66c
	.ascii	"m_unknown_str\000"
	.4byte	0x693
	.ascii	"nrf_strerror_array\000"
	.4byte	0x6a5
	.ascii	"nrf_strerror_find\000"
	.4byte	0x71b
	.ascii	"nrf_strerror_get\000"
	.4byte	0
	.section	.debug_pubtypes,"",%progbits
	.4byte	0x1a7
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x751
	.4byte	0x25
	.ascii	"signed char\000"
	.4byte	0x2c
	.ascii	"unsigned char\000"
	.4byte	0x38
	.ascii	"short int\000"
	.4byte	0x3f
	.ascii	"short unsigned int\000"
	.4byte	0x57
	.ascii	"int\000"
	.4byte	0x46
	.ascii	"int32_t\000"
	.4byte	0x6a
	.ascii	"unsigned int\000"
	.4byte	0x5e
	.ascii	"uint32_t\000"
	.4byte	0x71
	.ascii	"long long int\000"
	.4byte	0x78
	.ascii	"long long unsigned int\000"
	.4byte	0xa7
	.ascii	"long int\000"
	.4byte	0x7f
	.ascii	"__mbstate_s\000"
	.4byte	0xcd
	.ascii	"char\000"
	.4byte	0x2b3
	.ascii	"__RAL_locale_data_t\000"
	.4byte	0x3a0
	.ascii	"__RAL_locale_codeset_t\000"
	.4byte	0x3ee
	.ascii	"__RAL_locale_t\000"
	.4byte	0x3ff
	.ascii	"__locale_s\000"
	.4byte	0x572
	.ascii	"__RAL_error_decoder_fn_t\000"
	.4byte	0x594
	.ascii	"__RAL_error_decoder_s\000"
	.4byte	0x5c5
	.ascii	"__RAL_error_decoder_t\000"
	.4byte	0x5e5
	.ascii	"ret_code_t\000"
	.4byte	0x646
	.ascii	"nrf_strerror_desc_t\000"
	.4byte	0
	.section	.debug_aranges,"",%progbits
	.4byte	0x24
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.LFB147
	.4byte	.LFE147-.LFB147
	.4byte	.LFB148
	.4byte	.LFE148-.LFB148
	.4byte	0
	.4byte	0
	.section	.debug_ranges,"",%progbits
.Ldebug_ranges0:
	.4byte	.LFB147
	.4byte	.LFE147
	.4byte	.LFB148
	.4byte	.LFE148
	.4byte	0
	.4byte	0
	.section	.debug_line,"",%progbits
.Ldebug_line0:
	.section	.debug_str,"MS",%progbits,1
.LASF95:
	.ascii	"p_end\000"
.LASF26:
	.ascii	"p_sep_by_space\000"
.LASF59:
	.ascii	"__mbstate_s\000"
.LASF64:
	.ascii	"__RAL_codeset_ascii\000"
.LASF17:
	.ascii	"currency_symbol\000"
.LASF44:
	.ascii	"date_time_format\000"
.LASF21:
	.ascii	"positive_sign\000"
.LASF34:
	.ascii	"int_n_sep_by_space\000"
.LASF100:
	.ascii	"D:\\ikyhdst\\Git\\dwm1001-examples\\nRF5_SDK_14.2.0"
	.ascii	"\\components\\libraries\\strerror\\nrf_strerror.c\000"
.LASF2:
	.ascii	"short int\000"
.LASF60:
	.ascii	"__locale_s\000"
.LASF10:
	.ascii	"__wchar\000"
.LASF28:
	.ascii	"n_sep_by_space\000"
.LASF66:
	.ascii	"__RAL_ascii_ctype_map\000"
.LASF48:
	.ascii	"__tolower\000"
.LASF30:
	.ascii	"n_sign_posn\000"
.LASF7:
	.ascii	"long long int\000"
.LASF45:
	.ascii	"__RAL_locale_data_t\000"
.LASF98:
	.ascii	"p_ret\000"
.LASF65:
	.ascii	"__RAL_codeset_utf8\000"
.LASF97:
	.ascii	"mid_c\000"
.LASF69:
	.ascii	"__RAL_c_locale_month_names\000"
.LASF25:
	.ascii	"p_cs_precedes\000"
.LASF96:
	.ascii	"p_mid\000"
.LASF90:
	.ascii	"code\000"
.LASF33:
	.ascii	"int_p_sep_by_space\000"
.LASF80:
	.ascii	"__RAL_error_decoder_s\000"
.LASF83:
	.ascii	"__RAL_error_decoder_t\000"
.LASF43:
	.ascii	"time_format\000"
.LASF13:
	.ascii	"decimal_point\000"
.LASF39:
	.ascii	"month_names\000"
.LASF57:
	.ascii	"codeset\000"
.LASF40:
	.ascii	"abbrev_month_names\000"
.LASF61:
	.ascii	"__category\000"
.LASF15:
	.ascii	"grouping\000"
.LASF51:
	.ascii	"__towlower\000"
.LASF11:
	.ascii	"long int\000"
.LASF76:
	.ascii	"__RAL_data_empty_string\000"
.LASF68:
	.ascii	"__RAL_c_locale_abbrev_day_names\000"
.LASF88:
	.ascii	"__StackTop\000"
.LASF63:
	.ascii	"__RAL_c_locale\000"
.LASF22:
	.ascii	"negative_sign\000"
.LASF81:
	.ascii	"decode\000"
.LASF55:
	.ascii	"name\000"
.LASF29:
	.ascii	"p_sign_posn\000"
.LASF79:
	.ascii	"__RAL_error_decoder_fn_t\000"
.LASF77:
	.ascii	"__user_set_time_of_day\000"
.LASF1:
	.ascii	"unsigned char\000"
.LASF84:
	.ascii	"__RAL_error_decoder_head\000"
.LASF73:
	.ascii	"__RAL_data_utf8_space\000"
.LASF93:
	.ascii	"nrf_strerror_array\000"
.LASF0:
	.ascii	"signed char\000"
.LASF8:
	.ascii	"long long unsigned int\000"
.LASF5:
	.ascii	"uint32_t\000"
.LASF20:
	.ascii	"mon_grouping\000"
.LASF6:
	.ascii	"unsigned int\000"
.LASF37:
	.ascii	"day_names\000"
.LASF41:
	.ascii	"am_pm_indicator\000"
.LASF104:
	.ascii	"nrf_strerror_get\000"
.LASF50:
	.ascii	"__towupper\000"
.LASF101:
	.ascii	"D:\\ikyhdst\\Git\\dwm1001-examples\\examples\\ds_tw"
	.ascii	"r_init\\SES\000"
.LASF16:
	.ascii	"int_curr_symbol\000"
.LASF74:
	.ascii	"__RAL_data_utf8_plus\000"
.LASF23:
	.ascii	"int_frac_digits\000"
.LASF18:
	.ascii	"mon_decimal_point\000"
.LASF14:
	.ascii	"thousands_sep\000"
.LASF38:
	.ascii	"abbrev_day_names\000"
.LASF82:
	.ascii	"next\000"
.LASF12:
	.ascii	"char\000"
.LASF78:
	.ascii	"__user_get_time_of_day\000"
.LASF4:
	.ascii	"int32_t\000"
.LASF27:
	.ascii	"n_cs_precedes\000"
.LASF99:
	.ascii	"GNU C99 9.3.1 20200408 (release) -fmessage-length=0"
	.ascii	" -mcpu=cortex-m4 -mlittle-endian -mfloat-abi=hard -"
	.ascii	"mfpu=fpv4-sp-d16 -mthumb -mtp=soft -munaligned-acce"
	.ascii	"ss -std=gnu99 -g2 -gpubnames -fomit-frame-pointer -"
	.ascii	"fno-dwarf2-cfi-asm -fno-builtin -ffunction-sections"
	.ascii	" -fdata-sections -fshort-enums -fno-common\000"
.LASF103:
	.ascii	"nrf_strerror_find\000"
.LASF87:
	.ascii	"SystemCoreClock\000"
.LASF94:
	.ascii	"p_start\000"
.LASF56:
	.ascii	"data\000"
.LASF92:
	.ascii	"m_unknown_str\000"
.LASF3:
	.ascii	"short unsigned int\000"
.LASF24:
	.ascii	"frac_digits\000"
.LASF70:
	.ascii	"__RAL_c_locale_abbrev_month_names\000"
.LASF62:
	.ascii	"__RAL_global_locale\000"
.LASF35:
	.ascii	"int_p_sign_posn\000"
.LASF102:
	.ascii	"timeval\000"
.LASF67:
	.ascii	"__RAL_c_locale_day_names\000"
.LASF54:
	.ascii	"__RAL_locale_codeset_t\000"
.LASF71:
	.ascii	"__RAL_data_utf8_period\000"
.LASF53:
	.ascii	"__mbtowc\000"
.LASF36:
	.ascii	"int_n_sign_posn\000"
.LASF58:
	.ascii	"__RAL_locale_t\000"
.LASF42:
	.ascii	"date_format\000"
.LASF47:
	.ascii	"__toupper\000"
.LASF72:
	.ascii	"__RAL_data_utf8_comma\000"
.LASF19:
	.ascii	"mon_thousands_sep\000"
.LASF89:
	.ascii	"__StackLimit\000"
.LASF32:
	.ascii	"int_n_cs_precedes\000"
.LASF86:
	.ascii	"ITM_RxBuffer\000"
.LASF46:
	.ascii	"__isctype\000"
.LASF9:
	.ascii	"__state\000"
.LASF75:
	.ascii	"__RAL_data_utf8_minus\000"
.LASF31:
	.ascii	"int_p_cs_precedes\000"
.LASF49:
	.ascii	"__iswctype\000"
.LASF52:
	.ascii	"__wctomb\000"
.LASF91:
	.ascii	"nrf_strerror_desc_t\000"
.LASF85:
	.ascii	"ret_code_t\000"
	.ident	"GCC: (GNU) 9.3.1 20200408 (release)"
