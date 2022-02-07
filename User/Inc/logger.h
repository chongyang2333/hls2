// 参考头文件模板
// 修改成对应的名称
#ifndef __LOGGER_H
#define __LOGGER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "UserDataTypes.h"

#define LOGGER_BUFFER_MAX_SIZE     (128u)       //每组可操作缓冲区的大小，至少有两组
#define LOGGER_BUFFER_GROUP_SUM     (3)         //可操作缓冲区的组数

#define FORMAT_FLAG_LEFT_JUSTIFY   (1u << 0)
#define FORMAT_FLAG_PAD_ZERO       (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN     (1u << 2)
#define FORMAT_FLAG_ALTERNATE      (1u << 3)

typedef struct
{
    char*   pBuffer;
    UINT16  BufferSize;
    UINT16  Cnt;
    UINT16  ReturnValue;
}PrintBuffer_t;

typedef struct
{
    UINT8 txBusy;
    void (*send)(UINT8 *, UINT16);         //发送函数
}LogInfertace_t;

typedef struct
{
    PrintBuffer_t tBuffer[LOGGER_BUFFER_GROUP_SUM];
    LogInfertace_t tInterface;

    UINT8   cSendIndex;             //发送缓冲区
    UINT8   cActiveIndex;           //正在进行数据操作的缓冲区
    UINT8   cIndexSum;
    UINT8   cFull;                  //1-满，0-未满
}Logger_t;


// 日志输出函数
PUBLIC void Log(char * tag, char * content, ...);
void Logbuf(char * tag, char *buf, int size);

// 初始化
PUBLIC void init_log(LogInfertace_t *interface);
PUBLIC void log_tx_callback(void);
void log_flush(void);

#ifdef __cplusplus
}
#endif
#endif
