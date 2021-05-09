#define REG_SIZE 1 // size of one register is 1 byte (8 bits)
#define NUM_CTRL_REGS 1 // The number of control register
#define NUM_STS_REGS 5 // The number of status register
#define NUM_DATA_REGS 256 // The number of data register
#define NUM_DEV_REGS (NUM_CTRL_REGS + NUM_STS_REGS + NUM_DATA_REGS)  // The total number of registers

#define READ_COUNT_H_REG 0
#define READ_COUNT_L_REG 1

#define WRITE_COUNT_H_REG 2
#define WRITE_COUNT_L_REG 3

#define DEVICE_STATUS_REG 4 // device status

#define STS_READ_ACCESS_BIT (1 << 0)
#define STS_WRITE_ACCESS_BIT (1 << 1)
#define STS_DATAREGS_OVERFLOW_BIT (1 << 2)

#define READY 1
#define NOT_READY 0
#define OVERFLOW 1
#define NOT_OVERFLOW 0

#define CONTROL_ACCESS_REG 0

#define CTRL_READ_DATA_BIT (1 << 0)
#define CTRL_WRITE_DATA_BIT (1 << 1)

#define ENABLE 1
#define DISABLE 0


