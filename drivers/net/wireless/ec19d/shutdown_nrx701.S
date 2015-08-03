/*
   arm-none-eabi-gcc -c shutdown_nrx701.S
   arm-none-eabi-ld -Ttext=0 -o shutdown_nrx701.elf shutdown_nrx701.o
   arm-none-eabi-objcopy -O binary shutdown_nrx701.elf shutdown_nrx701.bin
 */
#define DCU_BASE                    0x70030
#define DCU_CTRL                    0x00 /* byte */
#define DCU_MEASURE_VREF            0x08 /* half */
#define DCU_LFC_COUNT               0x0C /* word */
#define DCU_VREF_BIAS_VSEL          0x18 /* byte */
#define DCU_DCDC_IBIAS              0x1A /* byte */

        .text
        .arm
        .global _start
_reset_vector:
        /* Nop at reset vector */
        nop
_start:
        ldr     r0, =DCU_BASE /* DCU register */

        /* dcu_reg.vref_bias_vsel = 0x46 */
        ldr     r1, =0x46
        strb    r1, [r0, #DCU_VREF_BIAS_VSEL]

        /* dcu_reg.dcu_measure_vref = 0x05 */
        ldr     r1, =0x05
        strh    r1, [r0, #DCU_MEASURE_VREF]

        /* dcu_reg.dcdc_ibias = 0x1f */
        ldr     r1, =0x1f
        strb    r1, [r0, #DCU_DCDC_IBIAS]

        /* dcu_reg.lfc_count = 0x00FFFFFF */
        ldr     r1, =0x00FFFFFF
        str     r1, [r0, #DCU_LFC_COUNT]

        /* dcu_reg.ctrl = 0x02 */
        ldr     r1, =0x02
        strb    r1, [r0, #DCU_CTRL]

2:      b       2b

        .end
