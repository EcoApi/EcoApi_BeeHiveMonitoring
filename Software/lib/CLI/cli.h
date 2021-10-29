/************************************************************************* 
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * 
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 * For more information, please refer to <http://unlicense.org> 
 **************************************************************************/


#ifndef CLI_H_INCLUDED
#define CLI_H_INCLUDED

/* define maximum size of the input buffer */
#define C_MAX_INPUT_BUFFER_SIZE         100

/* define the maximum number of arguments a command can take */
#define C_MAX_ARGUMENTS                 5

/* define special characters */
#define CLI_ASCII_NUL     0
#define CLI_ASCII_SOH     1
#define CLI_ASCII_STX     2
#define CLI_ASCII_ETX     3
#define CLI_ASCII_EOT     4
#define CLI_ASCII_ENQ     5
#define CLI_ASCII_ACK     6
#define CLI_ASCII_BEL     7
#define CLI_ASCII_BS      8
#define CLI_ASCII_TAB     9
#define CLI_ASCII_LF      10
#define CLI_ASCII_VT      11
#define CLI_ASCII_FF      12
#define CLI_ASCII_CR      13
#define CLI_ASCII_SO      14
#define CLI_ASCII_SI      15
#define CLI_ASCII_DLE     16
#define CLI_ASCII_DC1     17
#define CLI_ASCII_DC2     18
#define CLI_ASCII_DC3     19
#define CLI_ASCII_DC4     20
#define CLI_ASCII_NAK     21
#define CLI_ASCII_SYN     22
#define CLI_ASCII_ETB     23
#define CLI_ASCII_CAN     24
#define CLI_ASCII_EM      25
#define CLI_ASCII_SUB     26
#define CLI_ASCII_ESC     27
#define CLI_ASCII_FS      28
#define CLI_ASCII_GS      29
#define CLI_ASCII_RS      30
#define CLI_ASCII_US      31
#define CLI_ASCII_SPACE   32
#define CLI_ASCII_DEL     127

/* return values */
#define CLI_RET_CHAR_ADDED          1
#define CLI_RET_CHAR_REMOVED        2
#define CLI_RET_BUFFER_FULL         3
#define CLI_RET_BUFFER_EMPTY        4
#define CLI_RET_COMMAND_EXECUTED    5
#define CLI_RET_COMMAND_INVALID     6
#define CLI_RET_COMMAND_EMPTY       7

/* command type */
typedef struct cmd
{
  char CommandName[20];
  void (*Handler)();
  //const char *CommandHelp;
}cmd_t;

/* the user needs to declare this table */
extern const cmd_t cli_CommandTable[];

/* function prototypes */
void cli_Inititalize();
int cli_ProcessChar(char c);
int cli_ExecuteCommand(char *buffer);

#endif // CLI_H_INCLUDED
