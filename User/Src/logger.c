#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "logger.h"
#include "HardApi.h"
#include "gd32f4xx.h"

PRIVATE char * gs_sIndicate = ": ";
PRIVATE char * gs_sNextLine = "\r\n";

PRIVATE Logger_t gs_tLogger;

PUBLIC void init_log(LogInfertace_t *interface)
{
    memset(&gs_tLogger, 0, sizeof(Logger_t));

    gs_tLogger.cIndexSum = sizeof(gs_tLogger.tBuffer)/sizeof(gs_tLogger.tBuffer[0]);

    for(UINT8 i = 0; i < gs_tLogger.cIndexSum; i++)
    {
        gs_tLogger.tBuffer[i].pBuffer = malloc(LOGGER_BUFFER_MAX_SIZE);
        gs_tLogger.tBuffer[i].BufferSize = LOGGER_BUFFER_MAX_SIZE;
        memset(gs_tLogger.tBuffer[i].pBuffer, 0, LOGGER_BUFFER_MAX_SIZE);
    }
    gs_tLogger.tInterface.send = interface->send;
}

/******************************************************************************************
 * @brief 	打印信息发送完毕后的回调函数
 * @retval	无
 ******************************************************************************************/
PUBLIC void log_tx_callback(void)
{
    PrintBuffer_t *temp;

    temp = &gs_tLogger.tBuffer[gs_tLogger.cSendIndex];
    temp->Cnt = 0;                  //清除当前缓冲区内容
    temp->ReturnValue = 0;

    gs_tLogger.cSendIndex++;
    gs_tLogger.cSendIndex = (gs_tLogger.cSendIndex >= gs_tLogger.cIndexSum) ? 0 : gs_tLogger.cSendIndex;
    gs_tLogger.tInterface.txBusy = 0;
}

void log_flush(void)
{
	static UINT32 lPreTick = 0;
	PrintBuffer_t *temp;

	if(gs_tLogger.tInterface.txBusy == 1)
	{
		if((ReadTimeStampTimer() - lPreTick) > 10)
		{
			temp = &gs_tLogger.tBuffer[gs_tLogger.cSendIndex];
			temp->Cnt = 0;
			temp->ReturnValue = 0;
			gs_tLogger.tInterface.txBusy = 0;
		}
	}
	else
	{
		if((gs_tLogger.cActiveIndex != gs_tLogger.cSendIndex) || (gs_tLogger.cFull != 0))     //缓冲区未满
		{
			gs_tLogger.cFull = 0;
			temp = &gs_tLogger.tBuffer[gs_tLogger.cSendIndex];
			if(temp->Cnt)       //如果下一个缓冲区内容不为空，那么再次启动发送
			{
				gs_tLogger.tInterface.send(temp->pBuffer, temp->Cnt);
				gs_tLogger.tInterface.txBusy = 1;
				lPreTick = ReadTimeStampTimer();
			}
		}
	}
}

void log_tx_start(void)
{
    PrintBuffer_t *temp;

    if(gs_tLogger.tInterface.txBusy == 0)
    {
        temp = &gs_tLogger.tBuffer[gs_tLogger.cSendIndex];
        if(temp->Cnt)
        {
            gs_tLogger.tInterface.send(temp->pBuffer, temp->Cnt);
            gs_tLogger.tInterface.txBusy = 1;
        }
    }
}

PRIVATE void storeChar(PrintBuffer_t ** p, char c)
{
    INT16 Cnt;

    Cnt = (*p)->Cnt;
    if ((Cnt + 1u) <= (*p)->BufferSize)
    {
        *((*p)->pBuffer + Cnt) = c;
        (*p)->Cnt = Cnt + 1u;
        (*p)->ReturnValue++;
    }

    if ((*p)->Cnt == (*p)->BufferSize)
    {
        gs_tLogger.cActiveIndex++;
        gs_tLogger.cActiveIndex = (gs_tLogger.cActiveIndex >= gs_tLogger.cIndexSum) ? 0 : gs_tLogger.cActiveIndex;

        *p = &gs_tLogger.tBuffer[gs_tLogger.cActiveIndex];
        if((*p)->Cnt > 0)
        {
            (*p)->ReturnValue = -1;				//缓冲区满
        }
    }
}

PRIVATE void PrintUnsigned(PrintBuffer_t * ppBufferDesc, unsigned v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  static const char _aV2C[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  unsigned Div;
  unsigned Digit;
  unsigned Number;
  unsigned Width;
  char c;

  Number = v;
  Digit = 1u;
  Width = 1u;

  while (Number >= Base) {
    Number = (Number / Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }

  if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) {
    if (FieldWidth != 0u) {
      if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (NumDigits == 0u)) {
        c = '0';
      } else {
        c = ' ';
      }
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        storeChar(&ppBufferDesc, c);
        if (ppBufferDesc->ReturnValue < 0) {
          break;
        }
      }
    }
  }
  if (ppBufferDesc->ReturnValue >= 0) {

    while (1) {
      if (NumDigits > 1u) {
        NumDigits--;
      } else {
        Div = v / Digit;
        if (Div < Base) {
          break;
        }
      }
      Digit *= Base;
    }

    do {
      Div = v / Digit;
      v -= Div * Digit;
      storeChar(&ppBufferDesc, _aV2C[Div]);
      if (ppBufferDesc->ReturnValue < 0) {
        break;
      }
      Digit /= Base;
    } while (Digit);

    if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == FORMAT_FLAG_LEFT_JUSTIFY) {
      if (FieldWidth != 0u) {
        while ((FieldWidth != 0u) && (Width < FieldWidth)) {
          FieldWidth--;
          storeChar(&ppBufferDesc, ' ');
          if (ppBufferDesc->ReturnValue < 0) {
            break;
          }
        }
      }
    }
  }
}

PRIVATE void PrintInt(PrintBuffer_t * ppBufferDesc, int v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  unsigned Width;
  int Number;

  Number = (v < 0) ? -v : v;

  Width = 1u;
  while (Number >= (int)Base) {
    Number = (Number / (int)Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }
  if ((FieldWidth > 0u) && ((v < 0) || ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN))) {
    FieldWidth--;
  }

  if ((((FormatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) || (NumDigits != 0u)) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u)) {
    if (FieldWidth != 0u) {
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        storeChar(&ppBufferDesc, ' ');
        if (ppBufferDesc->ReturnValue < 0) {
          break;
        }
      }
    }
  }

  if (ppBufferDesc->ReturnValue >= 0) {
    if (v < 0) {
      v = -v;
      storeChar(&ppBufferDesc, '-');
    } else if ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN) {
      storeChar(&ppBufferDesc, '+');
    } else {

    }
    if (ppBufferDesc->ReturnValue >= 0) {
      if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) && (NumDigits == 0u)) {
        if (FieldWidth != 0u) {
          while ((FieldWidth != 0u) && (Width < FieldWidth)) {
            FieldWidth--;
            storeChar(&ppBufferDesc, '0');
            if (ppBufferDesc->ReturnValue < 0) {
              break;
            }
          }
        }
      }
      if (ppBufferDesc->ReturnValue >= 0) {
        PrintUnsigned(ppBufferDesc, (unsigned)v, Base, NumDigits, FieldWidth, FormatFlags);
      }
    }
  }
}

PUBLIC void Log(char * tag, char * sFormat, ...)
{
    char c;
    PrintBuffer_t *pBufferDesc;
    int v;
    unsigned NumDigits;
    unsigned FormatFlags;
    unsigned FieldWidth;
    char *pIndicate = gs_sIndicate;
    char *pNextLine = gs_sNextLine;

    pBufferDesc = &gs_tLogger.tBuffer[gs_tLogger.cActiveIndex];

    if(gs_tLogger.tInterface.send == NULL)
        return;                          //如果没有实现发送接口，直接退出
    if(gs_tLogger.cFull)                  //缓冲区已满
        return;

    va_list ParamList;
    va_start(ParamList, sFormat);

    pBufferDesc->ReturnValue = 0;

    // PrintUnsigned(pBufferDesc, ReadTimeStampTimer(), 10u, 0, 0, 0);
    // storeChar(&pBufferDesc, ' ');

    do{
        c = *tag;
        if(c == 0u)
            break;
        tag++;
        storeChar(&pBufferDesc, c);
    }while (pBufferDesc->ReturnValue >= 0);

    do{
        c = *pIndicate;
        if(c == 0u)
            break;
        pIndicate++;
        storeChar(&pBufferDesc, c);
    }while (pBufferDesc->ReturnValue >= 0);

    do {
        c = *sFormat;
        sFormat++;
        if (c == 0u) {
            break;
        }
        if (c == '%')
        {
            //
            // Filter out flags
            //
            FormatFlags = 0u;
            v = 1;
            do {
            c = *sFormat;
            switch (c) {
                case '-': FormatFlags |= FORMAT_FLAG_LEFT_JUSTIFY; sFormat++; break;
                case '0': FormatFlags |= FORMAT_FLAG_PAD_ZERO;     sFormat++; break;
                case '+': FormatFlags |= FORMAT_FLAG_PRINT_SIGN;   sFormat++; break;
                case '#': FormatFlags |= FORMAT_FLAG_ALTERNATE;    sFormat++; break;
                default:  v = 0; break;
                }
            } while (v);
            //
            // filter out field with
            //
            FieldWidth = 0u;
            do {
            c = *sFormat;
            if ((c < '0') || (c > '9')) {
                break;
            }
            sFormat++;
            FieldWidth = (FieldWidth * 10u) + ((unsigned)c - '0');
            } while (1);

            //
            // Filter out precision (number of digits to display)
            //
            NumDigits = 0u;
            c = *sFormat;
            if (c == '.') {
            sFormat++;
            do {
                c = *sFormat;
                if ((c < '0') || (c > '9')) {
                break;
                }
                sFormat++;
                NumDigits = NumDigits * 10u + ((unsigned)c - '0');
            } while (1);
            }
            //
            // Filter out length modifier
            //
            c = *sFormat;
            do {
            if ((c == 'l') || (c == 'h')) {
                sFormat++;
                c = *sFormat;
            } else {
                break;
            }
            } while (1);
            //
            // Handle specifiers
            //
            switch (c) {
            case 'c': {
            char c0;
            v = va_arg(ParamList, int);
            c0 = (char)v;
            storeChar(&pBufferDesc, c0);
            break;
            }
            case 'd':
            v = va_arg(ParamList, int);
            PrintInt(pBufferDesc, v, 10u, NumDigits, FieldWidth, FormatFlags);
            break;
            case 'u':
            v = va_arg(ParamList, int);
            PrintUnsigned(pBufferDesc, (unsigned)v, 10u, NumDigits, FieldWidth, FormatFlags);
            break;
            case 'x':
            case 'X':
            v = va_arg(ParamList, int);
            PrintUnsigned(pBufferDesc, (unsigned)v, 16u, NumDigits, FieldWidth, FormatFlags);
            break;
            case 's':
            {
                const char * s = va_arg(ParamList, const char *);
                do {
                c = *s;
                s++;
                if (c == '\0') {
                    break;
                }
                storeChar(&pBufferDesc, c);
                } while (pBufferDesc->ReturnValue >= 0);
            }
            break;
            case 'p':
            v = va_arg(ParamList, int);
            PrintUnsigned(pBufferDesc, (unsigned)v, 16u, 8u, 8u, 0u);
            break;
            case '%':
            storeChar(&pBufferDesc, '%');
            break;
            default:
            break;
            }
            sFormat++;
        } else {
            storeChar(&pBufferDesc, c);
        }
    } while (pBufferDesc->ReturnValue >= 0);

    do{
        c = *pNextLine;
        if(c == 0u)
            break;
        pNextLine++;
        storeChar(&pBufferDesc, c);
    }while (pBufferDesc->ReturnValue >= 0);

    va_end(ParamList);

    if(pBufferDesc->Cnt != 0)
    {
		gs_tLogger.cActiveIndex++;
		gs_tLogger.cActiveIndex = (gs_tLogger.cActiveIndex >= gs_tLogger.cIndexSum) ? 0 : gs_tLogger.cActiveIndex;
		if(gs_tLogger.cActiveIndex == gs_tLogger.cSendIndex)
		{
			gs_tLogger.cFull = 1;
		}
    }
}

void Logbuf(char * tag, char *buf, int size)
{
	char c;
	unsigned char high, low;
	char *pIndicate = gs_sIndicate;
	char *pNextLine = gs_sNextLine;
	PrintBuffer_t *pBufferDesc;

	if(size <= 0)
		return;

	pBufferDesc = &gs_tLogger.tBuffer[gs_tLogger.cActiveIndex];

	if(gs_tLogger.tInterface.send == NULL)
		return;                          //如果没有实现发送接口，直接退出
	if(gs_tLogger.cFull)                  //缓冲区已满
		return;

	pBufferDesc->ReturnValue = 0;

	// PrintUnsigned(pBufferDesc, ReadTimeStampTimer(), 10u, 0, 0, 0);
	// storeChar(&pBufferDesc, ' ');

	do{
		c = *tag;
		if(c == 0u)
			break;
		tag++;
		storeChar(&pBufferDesc, c);
	}while (pBufferDesc->ReturnValue >= 0);

	do{
		c = *pIndicate;
		if(c == 0u)
			break;
		pIndicate++;
		storeChar(&pBufferDesc, c);
	}while (pBufferDesc->ReturnValue >= 0);

	for(unsigned int i=0; i < size; i++)
	{
		high = (buf[i] >> 4)&0xf;
		low  = buf[i] & 0xf;

		if(high > 9)
		{
			storeChar(&pBufferDesc, (high - 10) + 'A');
		}
		else
		{
			storeChar(&pBufferDesc, high + '0');
		}

		if(low > 9)
		{
			storeChar(&pBufferDesc, (low - 10) + 'A');
		}
		else
		{
			storeChar(&pBufferDesc, low + '0');
		}

		storeChar(&pBufferDesc, ' ');
	}

	do{
		c = *pNextLine;
		if(c == 0u)
			break;
		pNextLine++;
		storeChar(&pBufferDesc, c);
	}while (pBufferDesc->ReturnValue >= 0);

	if(pBufferDesc->Cnt != 0)
	{
		gs_tLogger.cActiveIndex++;
		gs_tLogger.cActiveIndex = (gs_tLogger.cActiveIndex >= gs_tLogger.cIndexSum) ? 0 : gs_tLogger.cActiveIndex;
		if(gs_tLogger.cActiveIndex == gs_tLogger.cSendIndex)
		{
			gs_tLogger.cFull = 1;
		}
	}
}
