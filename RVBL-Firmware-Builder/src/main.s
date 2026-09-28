# ---------------------------------------------------------------- peripherals
.equ GPIO_BASE,       0xF0000000
.equ GPIO_DATAOUT,    0x0
.equ GPIO_DATAIN,     0x4
.equ GPIO_DATADIR,    0x8

.equ UART_BASE,       0xF1000000
.equ UART_TXDATA,     0x0
.equ UART_RXDATA,     0x4
.equ UART_CONTROL,    0x8
.equ UART_TRANSMIT,   0x1          # CONTROL bit 0 : start transmission
.equ UART_RXDONE,     0x2          # CONTROL bit 1 : byte received
.equ UART_TXDONE,     0x4          # CONTROL bit 2 : transmitter ready

# ------------------------------------------------------------------- Xicrc
.equ F3_CRC_BYTE,     0x0
.equ CRC_INIT,        0xFFFF

# --------------------------------------------------------------- application
.equ CMD_MEASURE,     0x01
.equ CMD_SHUTDOWN,    0x02
.equ ST_OK,           0x00
.equ ST_CRC_ERR,      0x01
.equ ST_CMD_ERR,      0x02
.equ NUM_SAMPLES,     4            # power of two: average = sum >> 2
.equ POWER_LIMIT_UW,  60000000     # 60 W rated input limit
.equ OUT_DONE,        0x10         # P4
.equ OUT_ERROR,       0x20         # P5
.equ OUT_RELAY,       0x40         # P6

# Register map
#   s0  GPIO base          s1  UART base
#   s2  running CRC        s3  power sum (high)   s4  power sum (low)
#   s5  command            s6  status             s7  average power
#   s8  sample counter     s9  voltage sample     s11 GPIO output value
#   ra  link for word-level routines, t5 link for byte-level routines

.section .text
.globl _start

# ================================================================ init
_start:
    li   s0, GPIO_BASE
    li   s1, UART_BASE

    lw   t0, GPIO_DATADIR(s0)      # P7-P4 outputs, P3-P0 inputs
    ori  t0, t0, 0xF0
    sw   t0, GPIO_DATADIR(s0)
    sw   zero, GPIO_DATAOUT(s0)    # all outputs low

# ================================================================ main loop
main_loop:
wait_trigger:                      # 1. wait for P0 = 1
    lw   t0, GPIO_DATAIN(s0)
    andi t0, t0, 0x1
    beqz t0, wait_trigger

    sw   zero, GPIO_DATAOUT(s0)    # clear previous result
    li   s2, CRC_INIT              # reset CRC
    li   s3, 0                     # reset 64-bit power sum
    li   s4, 0

    jal  ra, rx_byte_crc           # 2. command byte
    mv   s5, a0

    li   s8, NUM_SAMPLES           # 3. four (V, I) pairs
sample_loop:
    jal  ra, rx_word_crc           # voltage (mV)
    mv   s9, a0
    jal  ra, rx_word_crc           # current (mA) -> a0

    mul   t2, s9, a0               # P = V x I (uW), low word   [Zmmul]
    mulhu t3, s9, a0               #                 high word  [Zmmul]
    add  s4, s4, t2                # sum_lo += P_lo
    sltu t4, s4, t2                # carry out of low word
    add  s3, s3, t3                # sum_hi += P_hi
    add  s3, s3, t4                #         + carry

    addi s8, s8, -1
    bnez s8, sample_loop

    jal  ra, rx_half               # 4. received CRC-16 (not fed into the CRC)
    bne  a0, s2, crc_error

    li   t0, CMD_MEASURE           # 5. validate command
    beq  s5, t0, cmd_ok
    li   t0, CMD_SHUTDOWN
    beq  s5, t0, cmd_ok
    j    cmd_error

cmd_ok:                            # 6. average = sum >> 2 (64-bit shift)
    srli s7, s4, 2
    slli t0, s3, 30
    or   s7, s7, t0
    srli s3, s3, 2

    li   s11, OUT_DONE             # 7. protection decision
    bnez s3, trip_relay            # average >= 2^32 uW
    li   t0, POWER_LIMIT_UW
    bltu t0, s7, trip_relay        # average > limit
    li   t0, CMD_SHUTDOWN
    beq  s5, t0, trip_relay        # hub requested shutdown
    j    status_ok
trip_relay:
    ori  s11, s11, OUT_RELAY
status_ok:
    li   s6, ST_OK
    j    report

crc_error:
    li   s6, ST_CRC_ERR
    j    report_error
cmd_error:
    li   s6, ST_CMD_ERR
report_error:
    li   s7, 0                     # no power reported, relay untouched
    li   s11, OUT_DONE | OUT_ERROR

report:                            # 8. UART result, then GPIO
    mv   a0, s6
    jal  t5, tx_byte               # status
    mv   a0, s7
    jal  ra, tx_word               # average power
    sw   s11, GPIO_DATAOUT(s0)

wait_release:                      # 9. re-arm only after P0 returns to 0
    lw   t0, GPIO_DATAIN(s0)
    andi t0, t0, 0x1
    bnez t0, wait_release
    j    main_loop

# ================================================================ UART RX
# rx_byte: wait for RXDONE, acknowledge, return byte in a0.  Link: t5
rx_byte:
    lw   t0, UART_CONTROL(s1)
    andi t0, t0, UART_RXDONE
    beqz t0, rx_byte
    sw   zero, UART_CONTROL(s1)
    lw   a0, UART_RXDATA(s1)
    andi a0, a0, 0xFF
    jr   t5

# rx_byte_crc: receive one byte and feed it into the CRC.  Link: ra
rx_byte_crc:
    jal  t5, rx_byte
    .insn r 0x33, F3_CRC_BYTE, 0x40, s2, a0, s2  # crc s2 <- CRC(s2, a0[7:0])  [Xicrc]
    ret

# rx_word_crc: receive 4 bytes LSB first, feed each into the CRC,
#              return the 32-bit value in a0.  Link: ra
rx_word_crc:
    li   a1, 0                     # assembled word
    li   t1, 0                     # bit position
rxwc_loop:
    jal  t5, rx_byte
    .insn r 0x33, F3_CRC_BYTE, 0x40, s2, a0, s2  # crc s2 <- CRC(s2, a0[7:0])  [Xicrc]
    sll  t2, a0, t1
    or   a1, a1, t2
    addi t1, t1, 8
    li   t3, 32
    bne  t1, t3, rxwc_loop
    mv   a0, a1
    ret

# rx_half: receive 2 bytes LSB first without CRC update.  Link: ra
rx_half:
    li   a1, 0
    li   t1, 0
rxh_loop:
    jal  t5, rx_byte
    sll  t2, a0, t1
    or   a1, a1, t2
    addi t1, t1, 8
    li   t3, 16
    bne  t1, t3, rxh_loop
    mv   a0, a1
    ret

# ================================================================ UART TX
# tx_byte: wait for TXDONE, send byte in a0.  Link: t5
tx_byte:
    lw   t0, UART_CONTROL(s1)
    andi t0, t0, UART_TXDONE
    beqz t0, tx_byte
    sw   a0, UART_TXDATA(s1)
    lw   t0, UART_CONTROL(s1)
    ori  t0, t0, UART_TRANSMIT
    sw   t0, UART_CONTROL(s1)
    jr   t5

# tx_word: send a0 as 4 bytes, LSB first.  Link: ra
tx_word:
    mv   a1, a0
    li   t1, 4
txw_loop:
    andi a0, a1, 0xFF
    jal  t5, tx_byte
    srli a1, a1, 8
    addi t1, t1, -1
    bnez t1, txw_loop
    ret
