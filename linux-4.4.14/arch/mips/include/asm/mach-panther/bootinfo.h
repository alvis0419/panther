#ifndef __PANTHER_BOOT_INFO_H__

#define BOOT_FROM_UART   0
#define BOOT_FROM_NOR    1
#define BOOT_FROM_NAND   2
#define BOOT_FROM_NAND_WITH_OTP 3
#define BOOT_FROM_SD     4

/* used for initializa linux cmdline*/
#define CMDLINE_PARAMS_ADDR 0x90000000
#define CMD_LINE_STRUCT_SIZE 76     // bytes
#define BAD_BLOCK_STR_SIZE 0x100
#define MAX_BAD_BLOCK_NUM 88
struct cmdline_param
{
  unsigned int tclk;
  unsigned int rclk;
  unsigned int img_size;
  char list_bad_block_str[BAD_BLOCK_STR_SIZE];    // with format 0,1,2,...,n-1,n => support worst case of n=88              
};

#define NAND_MAX_OOBSIZE        640
#define NAND_MAX_PAGESIZE       8192
                              
#endif // __PANTHER_BOOT_INFO_H__