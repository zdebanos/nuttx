/****************************************************************************
 * libs/libc/misc/lib_slcddecode.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/streams.h>
#include <nuttx/ascii.h>
#include <nuttx/lcd/slcd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indices, counts, helper macros *******************************************/

#define NDX_ESC        0
#define NDX_BRACKET    1
#define NDX_CODE3      2
#define NDX_COUNTH     2
#define NDX_COUNTL     3
#define NDX_CODE5      4

#define NCH_ESC        1
#define NCH_BRACKET    2
#define NCH_CODE3      3
#define NCH_COUNTH     3
#define NCH_COUNTL     4
#define NCH_CODE5      5

#define IS_HEX(a)      ((((a) >= '0') && ((a) <= '9')) || \
                        (((a) >= 'a') && ((a) <= 'f')))
#define CODE_MIN       ('A' + FIRST_SLCDCODE)
#define CODE_MAX       ('A' + LAST_SLCDCODE)
#define IS_CODE(a)     (((a) >= CODE_MIN) && ((a) <= CODE_MAX))
#define CODE_RETURN(a) (enum slcdcode_e)((a) - 'A')

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_nibble
 *
 * Description:
 *   Convert a ASCII hexadecimal character (using only lower case alphabetics
 *   into a binary nibble
 *
 * Input Parameters:
 *   ascii - The nibble characgter.
 *
 * Returned Value:
 *   The binary value of the nibble.
 *
 ****************************************************************************/

static uint8_t slcd_nibble(uint8_t ascii)
{
  if (ascii >= '0' && ascii <= '9')
    {
      return ascii - '0';
    }
  else
    {
      return ascii - 'a' + 10;
    }
}

/****************************************************************************
 * Name: slcd_reget
 *
 * Description:
 *   We have unused characters from the last, unsuccessful decode attempt.
 *   Return one of these instead of the new character from the stream.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location character to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     slcd_keycode_e
 *
 * Returned Value:
 *   Always SLCDRET_CHAR
 *
 ****************************************************************************/

static enum slcdret_e slcd_reget(FAR struct slcdstate_s *state,
                                 FAR uint8_t *pch, FAR uint8_t *parg)
{
  /* Return the next character */

  *pch = state->buf[state->ndx];
  *parg = 0;

  /* Bump up the indices and return false (meaning a normal character) */

  state->ndx++;
  state->nch--;
  return SLCDRET_CHAR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_decode
 *
 * Description:
 *   Get one byte of data or special command from the application provided
 *   input buffer.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that slcd_decode is called.
 *   pch - The location to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     slcdcode_e, depending on the return value from slcd_decode()
 *   parg - The location to save the count argument that accompanies some
 *     special actions
 *
 * Returned Value:
 *
 *   false:  Normal character
 *   true:   Special SLCD action code with possible argument
 *
 ****************************************************************************/

enum slcdret_e slcd_decode(FAR struct lib_instream_s *stream,
                           FAR struct slcdstate_s *state, FAR uint8_t *pch,
                           FAR uint8_t *parg)
{
  enum slcdcode_e code;
  uint8_t count;
  int ch;

  DEBUGASSERT(stream && state && pch && parg);

  /* Are their ungotten characters from the last, failed parse? */

  if (state->nch > 0)
    {
      /* Yes, return the next ungotten character */

      return slcd_reget(state, pch, parg);
    }

  state->ndx = 0;

  /* No, ungotten characters.  Get the next character from the buffer. */

  ch = lib_stream_getc(stream);
  if (lib_stream_eof(ch))
    {
      /* End of file/stream (or perhaps an I/O error) */

      return SLCDRET_EOF;
    }

  /* Save the character (whatever it is) in case we fail parsing later */

  state->buf[NDX_ESC] = (uint8_t)ch;
  state->nch = NCH_ESC;

  /* Check for the beginning of an escape sequence */

  if (ch != ASCII_ESC)
    {
      /* Not the beginning of an escape sequence.  Return the character. */

      return slcd_reget(state, pch, parg);
    }

  /* Get the next character from the buffer */

  ch = lib_stream_getc(stream);
  if (lib_stream_eof(ch))
    {
      /* End of file/stream.  Return the escape character now.  We will
       * return the EOF indication next time.
       */

      return slcd_reget(state, pch, parg);
    }

  /* Save the character (whatever it is) in case we fail parsing later */

  state->buf[NDX_BRACKET] = ch;
  state->nch = NCH_BRACKET;

  /* Check for ESC-[ */

  if (ch != '[')
    {
      /* Not the beginning of an escape sequence.  Return the ESC now,
       * return the following characters later.
       */

      lcderr("ERROR: Parsing failed: ESC followed by %02x\n", ch);
      return slcd_reget(state, pch, parg);
    }

  /* Get the next character from the buffer */

  ch = lib_stream_getc(stream);
  if (lib_stream_eof(ch))
    {
      /* End of file/stream.  Return the ESC now; return the following
       * characters later.
       */

      return slcd_reget(state, pch, parg);
    }

  /* If the next character is a hexadecimal value (with lower case
   * alphabetic characters), then we are parsing a 5-byte sequence.
   */

  if (!IS_HEX(ch))
    {
      /* Decode the value following the bracket */

      code  = CODE_RETURN(ch);
      count = 0;

      /* Verify the special CLCD action code */

      if (code < (int)FIRST_SLCDCODE || code > (int)LAST_SLCDCODE)
        {
          lcderr("ERROR: Parsing failed: ESC-L followed by %02x\n", ch);

          /* Not a special command code.. put the character in the reget
           * buffer.
           */

          state->buf[NDX_CODE3] = (uint8_t)ch;
          state->nch = NCH_CODE3;

          /* Return the ESC now and the next two characters later. */

          return slcd_reget(state, pch, parg);
        }
    }
  else
    {
      /* Save the first character of the two byte hexadecimal number */

      state->buf[NDX_COUNTH] = (uint8_t)ch;
      state->nch = NCH_COUNTH;

      /* Get the next character from the buffer */

      ch = lib_stream_getc(stream);
      if (lib_stream_eof(ch))
        {
          /* End of file/stream.  Return the ESC now; return the following
           * characters later.
           */

          return slcd_reget(state, pch, parg);
        }

      /* We expect the next character to be the second byte of hexadecimal
       * count value.
       */

      if (!IS_HEX(ch))
        {
          /* Not a 5-byte escape sequence.  Return the ESC now; return the
           * following characters later.
           */

          lcderr("ERROR: Parsing failed: ESC-L-%c followed by %02x\n",
                 state->buf[NDX_COUNTH], ch);

          return slcd_reget(state, pch, parg);
        }

      /* Save the second character of the two byte hexadecimal number */

      state->buf[NDX_COUNTL] = (uint8_t)ch;
      state->nch = NCH_COUNTL;

      /* Get the next character from the buffer */

      ch = lib_stream_getc(stream);
      if (lib_stream_eof(ch))
        {
          /* End of file/stream.  Return the ESC now; return the following
           * characters later.
           */

          return slcd_reget(state, pch, parg);
        }

      /* Put the character in the reget buffer because there is on more way
       * that we can fail.
       */

      state->buf[NDX_CODE5] = (uint8_t)ch;
      state->nch = NCH_CODE5;

      /* Get the code and the count values. All count values must be greater
       * than 0 or something is wrong.
       */

      code  = CODE_RETURN(ch);
      count = slcd_nibble(state->buf[NDX_COUNTH]) << 4 |
              slcd_nibble(state->buf[NDX_COUNTL]);

      /* Verify the special CLCD action code */

      if (code < (int)FIRST_SLCDCODE || code > (int)LAST_SLCDCODE)
        {
          /* Not a special command code. Return the ESC now and the rest
           * of the characters later.
           */

          lcderr("ERROR: Parsing failed: ESC-L-%c-%c followed by %02x\n",
                 state->buf[NDX_COUNTH], state->buf[NDX_COUNTL], ch);

          return slcd_reget(state, pch, parg);
        }
    }

  /* We have successfully parsed the entire escape sequence.  Return the
   * CLCD value in pch, return the count in parg, and an indication that this
   * is a special action.
   */

  *pch = code;
  *parg = count;
  state->nch = 0;
  return SLCDRET_SPEC;
}
