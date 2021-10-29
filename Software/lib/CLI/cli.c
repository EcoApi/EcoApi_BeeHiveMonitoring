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


#include "cli.h"

/* declare input buffer */
static char input_buffer[C_MAX_INPUT_BUFFER_SIZE];
static int input_buffer_count;

/* function prototypes */
static int cli_SplitIntoWords(char *buffer, char *Words[], int Max);
static cmd_t *cli_LookupCommand(char* command);
static int cli_IsPrintable(char c);
static void cli_ConvertToLowerCase(char *buffer);
static int cli_StrCmp(const char *s1, const char *s2);


/*************************************************
 * clear the input buffer
 *************************************************/
void cli_Inititalize()
{
  int i;

  input_buffer_count = 0;

  for(i = 0; i < C_MAX_INPUT_BUFFER_SIZE; i++)
  {
    input_buffer[i] = CLI_ASCII_NUL;
  }
}


/*************************************************
 * add char to the input buffer
 *************************************************/
int cli_ProcessChar(char c)
{
  int ret;

  if((c != CLI_ASCII_LF) && (c != CLI_ASCII_CR))
  {
    /* the character is not a line feed or cariage return */
    if(1 == cli_IsPrintable(c))
    {
      /* character is printable */

      if(input_buffer_count < (C_MAX_INPUT_BUFFER_SIZE-1))
      {
        /* ensure the input buffer is not full */
        input_buffer[input_buffer_count] = c;
        input_buffer_count++;
        return CLI_RET_CHAR_ADDED;
      }
      else
        return CLI_RET_BUFFER_FULL;

    }
    else
    {

      /* character is not printable */
      if((c == CLI_ASCII_BS) || (c == CLI_ASCII_DEL))
      {
        /* character is a backspace or delete */
        /* remove the last character from the input buffer */

        if(input_buffer_count > 0)
        {
          /* make sure there are characters in the buffer */
          input_buffer_count--;
          input_buffer[input_buffer_count] = CLI_ASCII_NUL;
          return CLI_RET_CHAR_REMOVED;
        }
        else
          return CLI_RET_BUFFER_EMPTY;
      }

    }

  }
  else
  {
    /* the character is a line feed or cariage return */
    /* process the line buffer */

    /* execute the command */
    ret = cli_ExecuteCommand(input_buffer);

    /* clear the input buffer */
    cli_Inititalize();

    return ret;
  }

  return 0;
}


/*************************************************
 * execute the command
 *************************************************/
int cli_ExecuteCommand(char *buffer)
{
  int nargs; /* number of words */
  char* args[C_MAX_ARGUMENTS];

  cmd_t *cp;

  /* split string into words */
  nargs = cli_SplitIntoWords(buffer, args, C_MAX_ARGUMENTS);

  if(nargs == 0)
    return CLI_RET_COMMAND_EMPTY;

  /* does the command exist in the table */
  if((cp = cli_LookupCommand(args[0])) == 0 || cp->Handler == 0)
  {
    return CLI_RET_COMMAND_INVALID;
  }

  /* execute the handler */
  (*cp->Handler)(args, nargs);

  return CLI_RET_COMMAND_EXECUTED;
}


/*************************************************
 * split the input buffer at the spaces
 *************************************************/
static int cli_SplitIntoWords(char *buffer, char *Words[], int Max)
{
  int i;
  int word_count;

  /* clear the Words[] array */
  for(i = 0; i < Max; i++)
  {
    Words[i] = 0;
  }

  /* iterate through the line buffer and split at spaces */
  word_count = 0;
  while(*buffer != CLI_ASCII_NUL)
  {
    while((*buffer == CLI_ASCII_SPACE))
    {
      buffer++;
    }

    if(*buffer == CLI_ASCII_NUL)
    {
      break;
    }

    if(word_count++ < Max)
    {
      Words[word_count-1] = buffer;
    }

    while((*buffer != CLI_ASCII_SPACE) && (*buffer != CLI_ASCII_NUL))
    {
      buffer++;
    }

    if(*buffer != CLI_ASCII_NUL)
    {
      *buffer++ = CLI_ASCII_NUL;
    }
  }
  return word_count;
}


/*************************************************
 * lookup the command in the table
 *************************************************/
static cmd_t *cli_LookupCommand(char* command)
{
  cmd_t *cp;

  for(cp = (cmd_t*)cli_CommandTable;(cp->CommandName[0] != CLI_ASCII_NUL); cp++)
  {
    if(cli_StrCmp((const char*)command, (const char*)(cp->CommandName)) == 0)
      return cp;
  }
  return 0;
}


/*************************************************
 * check if the character is a printable character
 *************************************************/
static int cli_IsPrintable(char c)
{
  if((c >= 32) && (c <= 126))
    return 1;
  else
    return 0;
}


/*************************************************
 * convert string to lower case
 *************************************************/
static void cli_ConvertToLowerCase(char *buffer)
{
  char *p;

  for(p = buffer; *p; p++)
  {
    if(((*p) >= 65) && ((*p) <= 90))
    {
      *p = (*p) + 32;
    }
  }
}


/*************************************************
 * compare two strings
 * this was stolen from:
 * http://www.opensource.apple.com/source/Libc/Libc-262/ppc/gen/strcmp.c
 *
 * The `strcmp' function compares the string pointed to by `s1' to the
 * string pointed to by `s2'.
 * The `strcmp' function returns an integer greater than, equal to, or less
 * than zero, according as the string pointed to by `s1' is greater than,
 * equal to, or less than the string pointed to by `s2'.
 *************************************************/
static int cli_StrCmp(const char *s1, const char *s2)
{
  for ( ; *s1 == *s2; s1++, s2++)
	  if (*s1 == '\0')
	    return 0;
  return ((*(unsigned char *)s1 < *(unsigned char *)s2) ? -1 : +1);

}

